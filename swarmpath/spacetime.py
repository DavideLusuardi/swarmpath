"""Single-agent A* in space-time: a state is (cell, t), and waiting is an action."""

import heapq
import itertools
from dataclasses import dataclass, field

from swarmpath.grid import Grid
from swarmpath.solution import Path


@dataclass
class Constraints:
    """What one agent is not allowed to do.

    vertex: (cell, t)     -> the agent must not be at ``cell`` at time t
    edge:   (u, v, t)     -> the agent must not move from u to v arriving at time t
    """

    vertex: set = field(default_factory=set)
    edge: set = field(default_factory=set)

    def last_time(self) -> int:
        times = [t for _, t in self.vertex] + [t for _, _, t in self.edge]
        return max(times, default=0)


class ConflictTable:
    """Where the other agents are, so a search can count (or forbid) the collisions of a move.

    This is the "conflict avoidance table" of Standley (2010), or the reservation table of
    prioritized planning. The agent keeps occupying its goal after arriving, so the cells of the
    other agents' goals are "parked" from then on.
    """

    def __init__(self, paths: list[Path]):
        self.at = {}  # (cell, t) -> number of agents there at time t, before their arrival
        self.moves = {}  # (u, v, t) -> number of agents moving u -> v and arriving at time t
        self.parked = {}  # cell -> arrival times of the agents whose goal it is
        for path in paths:
            self.add(path)

    def add(self, path: Path):
        for t in range(len(path) - 1):
            self.at[(path[t], t)] = self.at.get((path[t], t), 0) + 1
        for t in range(1, len(path)):
            move = (path[t - 1], path[t], t)
            self.moves[move] = self.moves.get(move, 0) + 1
        self.parked.setdefault(path[-1], []).append(len(path) - 1)

    def count(self, u: tuple[int, int], v: tuple[int, int], t: int) -> int:
        """Collisions of a move u -> v arriving at time t (u == v for a wait).

        Collisions with agents that drive over our goal after we stop there are not counted
        (see ``later_visits``).
        """
        n = self.at.get((v, t), 0)  # someone else at v
        n += sum(1 for arrival in self.parked.get(v, []) if t >= arrival)  # parked at v
        if u != v:
            n += self.moves.get((v, u, t), 0)  # someone coming the other way: a swap
        return n

    def last_time(self) -> int:
        """When the last agent arrives at its goal: after that, nothing moves any more."""
        return max((t for arrivals in self.parked.values() for t in arrivals), default=0)

    def later_visits(self, cell: tuple[int, int], t: int) -> int:
        """Collisions of an agent that stops on ``cell`` at time t and stays there forever."""
        return sum(self.at.get((cell, s), 0) for s in range(t + 1, self.last_time() + 1))


def space_time_astar(
    grid: Grid,
    start: tuple[int, int],
    goal: tuple[int, int],
    h: dict[tuple[int, int], int],
    constraints: Constraints | None = None,
    table: ConflictTable | None = None,
    soft: bool = False,
) -> Path | None:
    """A path from ``start`` to ``goal`` that respects ``constraints``, or None.

    ``h`` is the BFS distance table of ``goal`` (``heuristics.bfs_distances``). ``table`` holds
    the paths of the other agents:

    - hard mode: the path must not collide with any of them; it is a shortest such path;
    - soft mode (``soft=True``): collisions are allowed, and the path has the fewest collisions
      and, among those, is shortest. The cost is the pair (collisions, length), compared first
      by collisions.
    """
    if constraints is None:
        constraints = Constraints()
    if table is None:
        table = ConflictTable([])
    if start not in h:
        return None
    # The agent stays at its goal forever, so it may only stop there after the last constraint
    # on the goal cell; before that it has to wait elsewhere or pass through.
    last_goal_constraint = max((t for cell, t in constraints.vertex if cell == goal), default=-1)
    # From time T on, no constraint applies and every other agent is parked on its goal for
    # good: being at a cell at time T or at any later time is the same situation, so the state
    # (cell, t) for t >= T is stored as (cell, T) and the search space is finite. The soft mode
    # needs this: it tries every collision-free state before accepting a single collision, and
    # with a plain time horizon that would be every cell at every time up to the horizon.
    T = max(constraints.last_time(), table.last_time()) + 1

    tie = itertools.count()  # heap tie-breaker, so states themselves are never compared
    # (collisions, f = t + h, tie, cell, t, parent state, done)
    open_list = [(0, h[start], next(tie), start, 0, None, False)]
    # Copies of a state can differ in collisions, so a state is closed (and its parent fixed)
    # when it leaves the open list: the first copy out is the one with the fewest collisions.
    came_from = {}
    while open_list:
        collisions, _, _, cell, t, parent, done = heapq.heappop(open_list)
        state = (cell, min(t, T))
        if done:
            return reconstruct(came_from, state)
        if state in came_from:
            continue
        came_from[state] = parent
        if cell == goal and t > last_goal_constraint:
            # Stopping here also collides with everyone who passes over the goal later on, so
            # stopping is a separate "done" entry with those collisions added (as in SIPPS).
            later = table.later_visits(goal, t)
            if soft or later == 0:
                heapq.heappush(open_list, (collisions + later, t, next(tie), cell, t, None, True))
        for nxt in grid.neighbors(cell) + [cell]:  # four moves and a wait
            if (nxt, t + 1) in constraints.vertex or (cell, nxt, t + 1) in constraints.edge:
                continue
            hits = table.count(cell, nxt, t + 1)
            if hits and not soft:
                continue
            if (nxt, min(t + 1, T)) in came_from:
                continue
            heapq.heappush(open_list, (collisions + hits, t + 1 + h[nxt], next(tie), nxt, t + 1, state, False))
    return None


def reconstruct(came_from: dict, state) -> Path:
    path = []
    while state is not None:
        path.append(state[0])
        state = came_from[state]
    return path[::-1]
