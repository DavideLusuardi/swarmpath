"""EECBS: Explicit Estimation Conflict-Based Search (Li, Ruml and Koenig, AAAI 2021).

A bounded-suboptimal MAPF solver: the SOC it returns is at most ``w`` times the optimal one.
Like CBS it searches on two levels:

- the **high level** searches a constraint tree (CT): each node holds one path per agent and
  the constraints they were planned with. Expanding a node picks one conflict and makes two
  children, each forbidding the conflict to one of the two agents, who is then replanned;
- the **low level** plans one agent at a time with focal search, which may return a path up to
  ``w`` times longer than the shortest one if it collides less with the other agents.

``w = 1`` gives an optimal solver (CBS with conflict prioritization and bypass).
"""

import heapq
import itertools
import time
from dataclasses import dataclass, field

from swarmpath.conflicts import Conflict, find_conflicts
from swarmpath.grid import Grid
from swarmpath.heuristics import bfs_distances
from swarmpath.instance import Instance
from swarmpath.solution import Path, Result, soc
from swarmpath.spacetime import ConflictTable, Constraints

# ---------------------------------------------------------------------------------------------
# Low level: focal search in space-time
# ---------------------------------------------------------------------------------------------


@dataclass
class LowNode:
    cell: tuple[int, int]
    t: int
    f: int  # t + h(cell)
    conflicts: int  # collisions with the other agents along the path to this node
    parent: "LowNode | None"
    closed: bool = False  # expanded, or replaced by a copy with fewer conflicts


def focal_search(
    grid: Grid,
    start: tuple[int, int],
    goal: tuple[int, int],
    h: dict[tuple[int, int], int],
    constraints: Constraints,
    table: ConflictTable,
    w: float,
) -> tuple[Path | None, int]:
    """A path that respects ``constraints`` and is at most ``w`` times the shortest one, with
    few collisions with ``table``; returns ``(path, lb)``, or ``(None, 0)`` if there is none.

    OPEN holds every generated node not yet expanded, ordered by f. FOCAL is the part of OPEN
    with f <= w * f_min, ordered by number of collisions; nodes are always expanded from FOCAL.
    ``lb`` is f_min when the goal is expanded: no path can be shorter than it, and the path
    returned is at most ``w * lb`` long.
    """
    if start not in h:
        return None, 0
    # Stop at the goal only after its last constraint (as in spacetime.space_time_astar), and
    # give up at a horizon: after the last constraint, a shortest path needs at most one step
    # per free cell.
    last_goal_constraint = max((t for cell, t in constraints.vertex if cell == goal), default=-1)
    max_time = constraints.last_time() + len(grid.free_cells())

    tie = itertools.count()  # heap tie-breaker, so nodes themselves are never compared
    root = LowNode(start, 0, h[start], 0, None)
    open_heap = [(root.f, next(tie), root)]
    focal_heap = [(root.conflicts, root.f, next(tie), root)]
    waiting = []  # generated nodes with f above the focal bound, ordered by f
    best = {(start, 0): root}  # (cell, t) -> the copy of that state with fewest collisions

    while True:
        while open_heap and open_heap[0][2].closed:
            heapq.heappop(open_heap)
        if not open_heap:
            return None, 0
        f_min = open_heap[0][0]
        # f_min never decreases, so the focal bound only grows: let in the nodes now under it
        while waiting and waiting[0][0] <= w * f_min:
            _, _, node = heapq.heappop(waiting)
            heapq.heappush(focal_heap, (node.conflicts, node.f, next(tie), node))
        while focal_heap[0][3].closed:
            heapq.heappop(focal_heap)  # FOCAL is never empty: it contains OPEN's top
        _, _, _, node = heapq.heappop(focal_heap)
        node.closed = True

        if node.cell == goal and node.t > last_goal_constraint:
            path = []
            while node is not None:
                path.append(node.cell)
                node = node.parent
            return path[::-1], f_min
        if node.t == max_time:
            continue
        t = node.t + 1
        for nxt in grid.neighbors(node.cell) + [node.cell]:  # four moves and a wait
            if (nxt, t) in constraints.vertex or (node.cell, nxt, t) in constraints.edge:
                continue
            conflicts = node.conflicts + table.count(node.cell, nxt, t)
            # Every copy of the state (nxt, t) has the same f = t + h, so the only way one can
            # be better than another is with fewer collisions.
            old = best.get((nxt, t))
            if old is not None:
                if old.conflicts <= conflicts:
                    continue
                old.closed = True  # if it was already expanded, the state is re-opened
            child = LowNode(nxt, t, t + h[nxt], conflicts, node)
            best[(nxt, t)] = child
            heapq.heappush(open_heap, (child.f, next(tie), child))
            if child.f <= w * f_min:
                heapq.heappush(focal_heap, (child.conflicts, child.f, next(tie), child))
            else:
                heapq.heappush(waiting, (child.f, next(tie), child))


# ---------------------------------------------------------------------------------------------
# Conflict prioritization with multi-valued decision diagrams (MDDs)
# ---------------------------------------------------------------------------------------------


def build_mdd(
    grid: Grid,
    start: tuple[int, int],
    goal: tuple[int, int],
    h: dict[tuple[int, int], int],
    constraints: Constraints,
    cost: int,
) -> list[set[tuple[int, int]]]:
    """``levels[t]``: the cells where the agent can be at time t on a path of exactly ``cost``
    steps that respects ``constraints``. Meaningful when ``cost`` is the optimal cost, so that
    every such path is a shortest one.
    """
    # forward: cells reachable at time t from which the goal can still be reached in time
    levels = [{start}]
    for t in range(1, cost + 1):
        level = set()
        for cell in levels[-1]:
            for nxt in grid.neighbors(cell) + [cell]:
                if t + h[nxt] <= cost and (nxt, t) not in constraints.vertex and (cell, nxt, t) not in constraints.edge:
                    level.add(nxt)
        levels.append(level)
    # backward: keep only the cells that lead to the goal at time ``cost``
    levels[cost] = levels[cost] & {goal}
    for t in range(cost - 1, -1, -1):
        levels[t] = {
            cell
            for cell in levels[t]
            if any(nxt in levels[t + 1] and (cell, nxt, t + 1) not in constraints.edge for nxt in grid.neighbors(cell) + [cell])
        }
    return levels


def shortest_path_mdd(node: "CTNode", agent: int, instance: Instance) -> list[set[tuple[int, int]]] | None:
    """The MDD of the agent's path, or None if the path is not provably a shortest one.

    With ``w > 1`` the focal search may return a longer path; only when its cost equals the
    agent's lower bound do we know that it is shortest, and the MDD says something.
    """
    cost = len(node.paths[agent]) - 1
    if cost != node.lbs[agent]:
        return None
    goal = instance.goals[agent]
    h = bfs_distances(instance.grid, goal)
    return build_mdd(instance.grid, instance.starts[agent], goal, h, node.constraints[agent], cost)


def is_cardinal_for(conflict: Conflict, agent: int, levels: list[set[tuple[int, int]]] | None) -> bool:
    """Would forbidding the conflict to ``agent`` make its path longer?

    Yes when all its shortest paths go through the conflict: the MDD has a single cell there.
    """
    if levels is None:
        return False
    if conflict.t >= len(levels):  # the agent is parked on its goal: it has to leave and come back
        return True
    if conflict.kind == "vertex":
        return levels[conflict.t] == {conflict.u}
    u, v = (conflict.u, conflict.v) if agent == conflict.a1 else (conflict.v, conflict.u)
    return levels[conflict.t - 1] == {u} and levels[conflict.t] == {v}


def classify(conflict: Conflict, mdds: dict) -> str:
    """cardinal: both children cost more; semi-cardinal: one does; non-cardinal: neither.

    ``mdds`` maps each agent to ``shortest_path_mdd``.
    """
    cardinal = is_cardinal_for(conflict, conflict.a1, mdds[conflict.a1]) + is_cardinal_for(
        conflict, conflict.a2, mdds[conflict.a2]
    )
    return ["non-cardinal", "semi-cardinal", "cardinal"][cardinal]


# ---------------------------------------------------------------------------------------------
# High level: Explicit Estimation Search on the constraint tree
# ---------------------------------------------------------------------------------------------


@dataclass
class CTNode:
    constraints: list[Constraints]  # one set per agent
    paths: list[Path]
    lbs: list[int]  # per agent, a lower bound on its path cost under its constraints
    conflicts: list[Conflict] = field(init=False)
    cost: int = field(init=False)  # sum of costs of the paths
    lb: int = field(init=False)  # lower bound on the cost of any solution below this node
    d: int = field(init=False)  # distance-to-go: number of conflicts
    f_hat: float = 0.0  # cost + inadmissible estimate of the cost still to pay

    def __post_init__(self):
        self.update()

    def update(self):
        """Recompute what depends on the paths (after creation and after a bypass)."""
        self.conflicts = find_conflicts(self.paths)
        self.cost = soc(self.paths)
        self.lb = sum(self.lbs)
        self.d = len(self.conflicts)


class Estimator:
    """Online-learned inadmissible estimate of the cost still to pay below a CT node.

    One-step error (Thayer, Dionne and Ruml 2011): when a node N is expanded, compare it with
    its best child c. Resolving a conflict should remove one conflict and add no cost; the
    errors are what it really did:
        cost error      cost(c) - cost(N)
        distance error  d(c) + 1 - d(N)
    With their averages, N is about d / (1 - distance error) expansions away from a solution,
    each one adding the cost error.
    """

    def __init__(self):
        self.cost_errors = 0.0
        self.distance_errors = 0.0
        self.samples = 0

    def learn(self, parent: CTNode, best_child: CTNode):
        self.cost_errors += best_child.cost - parent.cost
        self.distance_errors += best_child.d + 1 - parent.d
        self.samples += 1

    def f_hat(self, node: CTNode) -> float:
        if self.samples == 0:
            return node.cost
        cost_error = max(0.0, self.cost_errors / self.samples)
        # If conflicts do not go down on average, the estimate would be infinite: cap it.
        distance_error = min(self.distance_errors / self.samples, 0.99)
        return node.cost + cost_error * node.d / (1 - distance_error)


def select_node(frontier: list[CTNode], w: float) -> CTNode:
    """EES node selection over three orderings of the same frontier.

    CLEANUP by lb (gives the bound), OPEN by f_hat (the best guess at the optimal solution),
    FOCAL = nodes of OPEN with f_hat <= w * best f_hat, by d (the node closest to a solution).
    Prefer the node closest to a solution, then the best guess, as long as their cost is within
    w times the lower bound; otherwise raise the lower bound.
    """
    best_lb = min(frontier, key=lambda n: (n.lb, n.d))
    best_f_hat = min(frontier, key=lambda n: (n.f_hat, n.d))
    focal = [n for n in frontier if n.f_hat <= w * best_f_hat.f_hat]
    best_d = min(focal, key=lambda n: (n.d, n.f_hat))
    if best_d.cost <= w * best_lb.lb:
        return best_d
    if best_f_hat.cost <= w * best_lb.lb:
        return best_f_hat
    return best_lb


def choose_conflict(node: CTNode, instance: Instance, prioritize: bool) -> Conflict:
    """With ``prioritize``: cardinal before semi-cardinal before non-cardinal, then earliest."""
    if not prioritize:
        return node.conflicts[0]  # the earliest
    agents = {a for c in node.conflicts for a in (c.a1, c.a2)}
    mdds = {a: shortest_path_mdd(node, a, instance) for a in agents}  # one per agent, not per conflict
    rank = {"cardinal": 0, "semi-cardinal": 1, "non-cardinal": 2}
    return min(node.conflicts, key=lambda c: (rank[classify(c, mdds)], c.t))


def split(conflict: Conflict) -> list[tuple[int, str, tuple]]:
    """The two ways out of a conflict: (agent, "vertex" or "edge", constraint)."""
    if conflict.kind == "vertex":
        return [
            (conflict.a1, "vertex", (conflict.u, conflict.t)),
            (conflict.a2, "vertex", (conflict.u, conflict.t)),
        ]
    return [
        (conflict.a1, "edge", (conflict.u, conflict.v, conflict.t)),
        (conflict.a2, "edge", (conflict.v, conflict.u, conflict.t)),
    ]


def make_child(node: CTNode, agent: int, kind: str, constraint: tuple, instance: Instance, w: float) -> CTNode | None:
    """Copy of ``node`` with one more constraint for ``agent``, who alone is replanned."""
    old = node.constraints[agent]
    new = Constraints(set(old.vertex), set(old.edge))
    (new.vertex if kind == "vertex" else new.edge).add(constraint)
    goal = instance.goals[agent]
    others = node.paths[:agent] + node.paths[agent + 1 :]
    path, lb = focal_search(
        instance.grid, instance.starts[agent], goal, bfs_distances(instance.grid, goal), new, ConflictTable(others), w
    )
    if path is None:
        return None
    constraints, paths, lbs = list(node.constraints), list(node.paths), list(node.lbs)
    constraints[agent], paths[agent] = new, path
    # More constraints can only make the shortest path longer, so the parent's bound still holds.
    lbs[agent] = max(lb, node.lbs[agent])
    return CTNode(constraints, paths, lbs)


def solve(
    instance: Instance, time_limit: float = 60.0, w: float = 1.0, prioritize: bool = True, bypass: bool = True, **params
) -> Result:
    started = time.perf_counter()
    stats = {"w": w, "expanded": 0, "generated": 1, "bypasses": 0}

    def stop(status: str) -> Result:
        return Result("eecbs", None, False, time.perf_counter() - started, {**stats, "status": status})

    # Root: every agent plans alone, avoiding the paths already planned where it can.
    paths, lbs = [], []
    for start, goal in zip(instance.starts, instance.goals):
        path, lb = focal_search(
            instance.grid, start, goal, bfs_distances(instance.grid, goal), Constraints(), ConflictTable(paths), w
        )
        if path is None:
            return stop("unreachable goal")
        paths.append(path)
        lbs.append(lb)
    root = CTNode([Constraints() for _ in paths], paths, lbs)
    estimator = Estimator()
    root.f_hat = estimator.f_hat(root)
    frontier = [root]

    while frontier:
        if time.perf_counter() - started > time_limit:
            return stop("timeout")
        node = select_node(frontier, w)
        if not node.conflicts:
            # Selected nodes cost at most w times the lowest lb on the frontier, which is at
            # most the optimal cost. For best_lb this holds because every node costs at most w
            # times its own lb: each path is within w of its lb, and bypass checks the same.
            lb = min(n.lb for n in frontier)
            stats.update(lb=lb, conflicts=0, status="solved")
            return Result("eecbs", node.paths, True, time.perf_counter() - started, stats)
        frontier.remove(node)
        stats["expanded"] += 1

        conflict = choose_conflict(node, instance, prioritize)
        children = []
        bypassed = False
        for agent, kind, constraint in split(conflict):
            child = make_child(node, agent, kind, constraint, instance, w)
            if child is None:
                continue
            stats["generated"] += 1
            # Bypass: a child as cheap as allowed but with fewer conflicts is a better version
            # of this node; take its path (not its constraint) and do not split this node now.
            if bypass and child.cost <= w * node.lb and child.d < node.d:
                node.paths[agent] = child.paths[agent]
                node.update()
                node.f_hat = estimator.f_hat(node)
                stats["bypasses"] += 1
                bypassed = True
                break
            children.append(child)
        if bypassed:
            frontier.append(node)
            continue
        if children:
            estimator.learn(node, min(children, key=lambda c: (c.cost, c.d)))
        for child in children:
            child.f_hat = estimator.f_hat(child)
            frontier.append(child)
    return stop("no solution")
