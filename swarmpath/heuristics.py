"""Exact single-agent distances, by breadth-first search backwards from the goal."""

from collections import deque

from swarmpath.grid import Grid


def bfs_distances(grid: Grid, goal: tuple[int, int]) -> dict[tuple[int, int], int]:
    """Distance from every cell that can reach ``goal``; unreachable cells are missing.

    Moves are reversible, so searching outward from the goal gives the distance *to* the goal.
    """
    dist = {goal: 0}
    queue = deque([goal])
    while queue:
        cell = queue.popleft()
        for n in grid.neighbors(cell):
            if n not in dist:
                dist[n] = dist[cell] + 1
                queue.append(n)
    return dist
