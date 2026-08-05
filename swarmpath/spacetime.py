"""Single-agent A* in space-time: a state is (cell, t), and waiting is an action."""

import heapq
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


def space_time_astar(
    grid: Grid,
    start: tuple[int, int],
    goal: tuple[int, int],
    h: dict[tuple[int, int], int],
    constraints: Constraints | None = None,
) -> Path | None:
    """Shortest path from ``start`` to ``goal`` that respects ``constraints``, or None.

    ``h`` is the BFS distance table of ``goal`` (``heuristics.bfs_distances``).
    """
    if constraints is None:
        constraints = Constraints()
    if start not in h:
        return None
    # The agent stays at its goal forever, so it may only stop there after the last
    # constraint on the goal cell; before that it has to wait elsewhere or pass through.
    last_goal_constraint = max((t for cell, t in constraints.vertex if cell == goal), default=-1)
    # After the last constraint nothing changes any more, and the agent needs at most one step
    # per free cell to reach its goal: a path longer than this would only go in circles.
    max_time = constraints.last_time() + len(grid.free_cells())

    open_list = [(h[start], 0, start)]  # (f = g + h, g = t, cell)
    # Every way of reaching (cell, t) costs the same g = t, so the first parent found for a
    # state is as good as any other and each state enters the open list only once.
    came_from = {(start, 0): None}
    while open_list:
        _, t, cell = heapq.heappop(open_list)
        if cell == goal and t > last_goal_constraint:
            return reconstruct(came_from, (cell, t))
        if t == max_time:
            continue
        for nxt in grid.neighbors(cell) + [cell]:  # four moves and a wait
            state = (nxt, t + 1)
            if state in came_from:
                continue
            if state in constraints.vertex or (cell, nxt, t + 1) in constraints.edge:
                continue
            came_from[state] = (cell, t)
            heapq.heappush(open_list, (t + 1 + h[nxt], t + 1, nxt))
    return None


def reconstruct(came_from: dict, state) -> Path:
    path = []
    while state is not None:
        path.append(state[0])
        state = came_from[state]
    return path[::-1]
