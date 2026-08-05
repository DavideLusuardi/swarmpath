"""MAPF instances: a grid plus one start and one goal per agent."""

import random
from dataclasses import dataclass
from pathlib import Path

from swarmpath.grid import Grid
from swarmpath.heuristics import bfs_distances


@dataclass
class Instance:
    grid: Grid
    starts: list[tuple[int, int]]
    goals: list[tuple[int, int]]
    name: str = ""

    @property
    def num_agents(self) -> int:
        return len(self.starts)


def load_scen(path: str | Path, grid: Grid, k: int) -> Instance:
    """The first ``k`` agents of a MovingAI .scen file.

    After a "version" line, each row is tab-separated:
    bucket, map, map width, map height, start x, start y, goal x, goal y, optimal length.
    """
    path = Path(path)
    rows = [line.split("\t") for line in path.read_text().splitlines()[1:]]
    if k > len(rows):
        raise ValueError(f"{path} has only {len(rows)} agents, asked for {k}")
    starts, goals = [], []
    for row in rows[:k]:
        width, height, sx, sy, gx, gy = [int(v) for v in row[2:8]]
        if (width, height) != (grid.width, grid.height):
            raise ValueError(f"{path} is for a {width}x{height} map, not {grid.width}x{grid.height}")
        starts.append((sx, sy))
        goals.append((gx, gy))
    return Instance(grid, starts, goals, name=path.stem)


def random_instance(grid: Grid, k: int, seed: int = 0) -> Instance:
    """Distinct random starts and distinct random goals, all in the largest connected area."""
    rng = random.Random(seed)
    area: list[tuple[int, int]] = []
    seen: set[tuple[int, int]] = set()
    for cell in grid.free_cells():
        if cell in seen:
            continue
        reachable = list(bfs_distances(grid, cell))  # the connected area containing ``cell``
        seen.update(reachable)
        if len(reachable) > len(area):
            area = reachable
    if k > len(area):
        raise ValueError(f"cannot place {k} agents in {len(area)} connected cells")
    return Instance(grid, rng.sample(area, k), rng.sample(area, k), name=f"{grid.name}-random-{k}-seed{seed}")
