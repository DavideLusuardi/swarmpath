"""Prioritized planning: plan the agents one at a time, each around the ones already planned.

- **hard** mode: every path avoids the paths planned before it (a reservation table). Fast, but
  incomplete: an agent may find no path at all, e.g. when an earlier agent parks on its route;
- **soft** mode: every path has as few collisions as possible with the paths planned before
  it. It never fails, but the result may still contain collisions. This is how MAPF-LNS2 builds
  and repairs its solutions.

Agents are planned in the order of the scenario file.
"""

import time

from swarmpath.conflicts import find_conflicts
from swarmpath.heuristics import bfs_distances
from swarmpath.instance import Instance
from swarmpath.solution import Path, Result
from swarmpath.spacetime import ConflictTable, space_time_astar


def plan_in_order(instance: Instance, agents: list[int], paths: list[Path | None], table: ConflictTable, soft: bool) -> bool:
    """Plan ``agents`` in this order around the paths in ``table``, adding each new path to it.

    The paths are written into ``paths[agent]``. Returns False as soon as an agent has no path.
    """
    for agent in agents:
        goal = instance.goals[agent]
        path = space_time_astar(
            instance.grid, instance.starts[agent], goal, bfs_distances(instance.grid, goal), table=table, soft=soft
        )
        if path is None:
            return False
        paths[agent] = path
        table.add(path)
    return True


def solve(instance: Instance, time_limit: float = 60.0, soft: bool = False, **params) -> Result:
    started = time.perf_counter()
    name = "pp-soft" if soft else "pp"
    paths = [None] * instance.num_agents
    table = ConflictTable([])
    for agent in range(instance.num_agents):
        if time.perf_counter() - started > time_limit:
            return Result(name, None, False, time.perf_counter() - started, {"status": "timeout"})
        if not plan_in_order(instance, [agent], paths, table, soft):
            return Result(name, None, False, time.perf_counter() - started, {"status": f"no path for agent {agent}"})
    conflicts = len(find_conflicts(paths))
    status = "solved" if conflicts == 0 else "collisions left"
    return Result(name, paths, conflicts == 0, time.perf_counter() - started, {"conflicts": conflicts, "status": status})
