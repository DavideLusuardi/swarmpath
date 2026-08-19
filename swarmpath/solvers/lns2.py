"""MAPF-LNS2 (Li, Chen, Harabor, Stuckey and Koenig, AAAI 2022): repair a plan with collisions.

Instead of searching for a collision-free plan, start from a plan that may have collisions and
repair it with large neighborhood search (LNS):

1. plan every agent with soft prioritized planning: fewest collisions with the agents before it;
2. repeat until no collisions are left: choose a *neighborhood* of a few agents, delete their
   paths and replan them with soft PP around everyone else; keep the new paths unless they
   collide more.

The objective is the number of colliding pairs of agents: the edges of the *collision graph*.
Three heuristics choose the neighborhood, and adaptive LNS learns which one works best.
"""

import time
from random import Random

from swarmpath.conflicts import find_conflicts
from swarmpath.heuristics import bfs_distances
from swarmpath.instance import Instance
from swarmpath.solution import Path, Result, position
from swarmpath.solvers.pp import plan_in_order
from swarmpath.spacetime import ConflictTable

GAMMA = 0.01  # reaction factor of adaptive LNS: how fast the heuristic weights follow rewards


def collision_graph(paths: list[Path]) -> dict[int, set[int]]:
    """Agent -> the agents it collides with."""
    graph = {a: set() for a in range(len(paths))}
    for c in find_conflicts(paths):
        graph[c.a1].add(c.a2)
        graph[c.a2].add(c.a1)
    return graph


def colliding_pairs(graph: dict[int, set[int]]) -> int:
    return sum(len(others) for others in graph.values()) // 2


def fill(chosen: list[int], candidates: list[int], size: int):
    """Add candidates to ``chosen``, in order and without repeats, until it has ``size`` agents."""
    for agent in candidates:
        if len(chosen) >= size:
            return
        if agent not in chosen:
            chosen.append(agent)


# ---------------------------------------------------------------------------------------------
# Neighborhoods: each returns at most ``size`` distinct agents, at least one of them colliding
# ---------------------------------------------------------------------------------------------


def collision_neighborhood(instance: Instance, paths: list[Path], graph: dict, size: int, rng: Random) -> list[int]:
    """Agents that collide with each other: a connected part of the collision graph.

    Replanning the agents of a whole component together lets them sort out their collisions,
    which replanning them one at a time against fixed paths may not.
    """
    first = rng.choice([a for a in graph if graph[a]])
    component = {first}
    todo = [first]
    while todo:
        for other in graph[todo.pop()]:
            if other not in component:
                component.add(other)
                todo.append(other)

    if len(component) > size:
        # too big: a random walk on the component from ``first`` until ``size`` agents are seen
        chosen = [first]
        here = first
        while len(chosen) < size:
            here = rng.choice(sorted(graph[here]))
            fill(chosen, [here], size)
        return chosen

    # small enough: take all of it, then add agents that are in the way of its members
    chosen = sorted(component)
    for _ in range(10 * size):  # each try may find nobody, so the number of tries is bounded
        if len(chosen) >= size:
            break
        met = agent_in_the_way(instance, paths, rng.choice(chosen), chosen, rng)
        if met is not None:
            chosen.append(met)
    return chosen


def agent_in_the_way(instance: Instance, paths: list[Path], agent: int, chosen: list[int], rng: Random) -> int | None:
    """An agent met by a random walk that starts from a random point of ``agent``'s path.

    The walk only takes steps after which the agent could still arrive no later than it does
    now, so whoever it meets is in the way of another path as good as the current one. Meeting
    means being at the same cell at the same time, or swapping cells with the walk.
    """
    goal = instance.goals[agent]
    h = bfs_distances(instance.grid, goal)
    cost = len(paths[agent]) - 1
    t = rng.randrange(cost + 1)
    cell = paths[agent][t]
    while t < cost:
        steps = [n for n in instance.grid.neighbors(cell) + [cell] if t + 1 + h[n] <= cost]
        prev, cell = cell, rng.choice(steps)  # never empty: the agent's own next step qualifies
        t += 1
        for other in range(len(paths)):
            if other in chosen:
                continue
            here, before = position(paths[other], t), position(paths[other], t - 1)
            if here == cell or (here == prev and before == cell):
                return other
    return None


def failure_neighborhood(instance: Instance, paths: list[Path], graph: dict, size: int, rng: Random) -> list[int]:
    """A colliding agent and the agents that block it, or that it blocks, at a start or goal.

    Prioritized planning mostly fails because of starts and goals: an agent still waiting at its
    start, or already parked at its goal, sits in the way of the others. Take a colliding agent
    ``a``, then the agents whose paths visit ``a``'s start, then those whose goal is on ``a``'s
    path, then random agents.
    """
    a = rng.choice([b for b in graph if graph[b]])
    others = [b for b in range(len(paths)) if b != a]
    visit_start = [b for b in others if instance.starts[a] in paths[b]]
    goal_on_path = [b for b in others if instance.goals[b] in paths[a]]
    rng.shuffle(visit_start)
    rng.shuffle(goal_on_path)
    rng.shuffle(others)
    chosen = [a]
    fill(chosen, visit_start + goal_on_path + others, size)
    return chosen


def random_neighborhood(instance: Instance, paths: list[Path], graph: dict, size: int, rng: Random) -> list[int]:
    """Random colliding agents, then random other agents if there are not enough of them."""
    colliding = [a for a in graph if graph[a]]
    others = [a for a in graph if not graph[a]]
    rng.shuffle(colliding)
    rng.shuffle(others)
    chosen = []
    fill(chosen, colliding + others, size)
    return chosen


NEIGHBORHOODS = {"collision": collision_neighborhood, "failure": failure_neighborhood, "random": random_neighborhood}


# ---------------------------------------------------------------------------------------------
# The LNS loop
# ---------------------------------------------------------------------------------------------


def solve(
    instance: Instance,
    time_limit: float = 60.0,
    neighborhood: int = 8,
    seed: int = 0,
    adaptive: bool = True,
    collision: bool = True,
    failure: bool = True,
    random: bool = True,
    **params,
) -> Result:
    """``neighborhood`` agents are replanned per iteration. ``adaptive=False`` picks the
    heuristic uniformly at random; ``collision``, ``failure`` and ``random`` switch them on/off.
    """
    started = time.perf_counter()
    rng = Random(int(seed))
    size = int(neighborhood)
    heuristics = [name for name, on in [("collision", collision), ("failure", failure), ("random", random)] if on]
    weights = {name: 1.0 for name in heuristics}
    used = {name: 0 for name in heuristics}
    stats = {"iterations": 0, "accepted": 0}

    paths = [None] * instance.num_agents
    if not plan_in_order(instance, list(range(instance.num_agents)), paths, ConflictTable([]), soft=True):
        return Result("lns2", None, False, time.perf_counter() - started, {"status": "unreachable goal"})
    graph = collision_graph(paths)
    pairs = colliding_pairs(graph)
    stats["initial_pairs"] = pairs

    while pairs > 0 and time.perf_counter() - started <= time_limit:
        stats["iterations"] += 1
        # Adaptive LNS: choose a heuristic with probability proportional to its weight. After a
        # long run without improvements every weight has decayed to 0, and then all are equal.
        if adaptive and sum(weights.values()) > 0:
            name = rng.choices(heuristics, weights=[weights[n] for n in heuristics])[0]
        else:
            name = rng.choice(heuristics)
        used[name] += 1
        agents = NEIGHBORHOODS[name](instance, paths, graph, size, rng)

        old = {a: paths[a] for a in agents}
        table = ConflictTable([paths[b] for b in range(instance.num_agents) if b not in old])
        order = list(agents)
        rng.shuffle(order)
        plan_in_order(instance, order, paths, table, soft=True)  # soft PP always finds a path
        new_graph = collision_graph(paths)
        new_pairs = colliding_pairs(new_graph)

        # The reward of a heuristic is how many colliding pairs it removed.
        weights[name] = GAMMA * max(0, pairs - new_pairs) + (1 - GAMMA) * weights[name]
        if new_pairs <= pairs:
            graph, pairs = new_graph, new_pairs
            stats["accepted"] += 1
        else:
            for a, path in old.items():
                paths[a] = path

    stats.update(
        pairs=pairs,
        conflicts=len(find_conflicts(paths)),
        used=used,
        weights={name: round(weight, 3) for name, weight in weights.items()},
        status="solved" if pairs == 0 else "timeout",
    )
    return Result("lns2", paths, pairs == 0, time.perf_counter() - started, stats)
