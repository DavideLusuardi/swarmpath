"""Baseline: every agent plans alone, ignoring the others.

Its SOC is a lower bound on the optimal SOC; its collisions are what MAPF has to remove.
The time limit is ignored: one A* search per agent is fast.
"""

import time

from swarmpath.conflicts import find_conflicts
from swarmpath.heuristics import bfs_distances
from swarmpath.instance import Instance
from swarmpath.solution import Result
from swarmpath.spacetime import space_time_astar


def solve(instance: Instance, time_limit: float | None = None, **params) -> Result:
    started = time.perf_counter()
    paths = []
    for start, goal in zip(instance.starts, instance.goals):
        h = bfs_distances(instance.grid, goal)
        path = space_time_astar(instance.grid, start, goal, h)
        if path is None:
            return Result("independent", None, False, time.perf_counter() - started, {"status": "unreachable goal"})
        paths.append(path)
    conflicts = len(find_conflicts(paths))
    return Result("independent", paths, conflicts == 0, time.perf_counter() - started, {"conflicts": conflicts})
