"""Paths, solver results, cost functions and an independent solution validator.

A path is a list of cells: ``path[t]`` is where the agent is at time t. It ends when the agent
reaches its goal for the last time, and the agent stays there afterwards.
"""

from dataclasses import dataclass, field

from swarmpath.instance import Instance

Path = list[tuple[int, int]]


def position(path: Path, t: int) -> tuple[int, int]:
    """Where the agent is at time t (at its goal once the path is over)."""
    return path[min(t, len(path) - 1)]


def soc(paths: list[Path]) -> int:
    """Sum of costs: the arrival times of all agents added together."""
    return sum(len(p) - 1 for p in paths)


def makespan(paths: list[Path]) -> int:
    """Arrival time of the last agent."""
    return max(len(p) - 1 for p in paths)


@dataclass
class Result:
    algo: str
    paths: list[Path] | None  # None when the solver found no paths at all
    solved: bool  # True only for a collision-free solution
    runtime: float = 0.0
    stats: dict = field(default_factory=dict)


def validate(instance: Instance, paths: list[Path]) -> list[str]:
    """Every rule the solution breaks; an empty list means it is a valid MAPF solution.

    Written separately from ``conflicts.py`` on purpose, so a bug there cannot hide here.
    """
    grid = instance.grid
    if len(paths) != instance.num_agents:
        return [f"expected {instance.num_agents} paths, got {len(paths)}"]
    errors = []
    for a, path in enumerate(paths):
        if path[0] != instance.starts[a]:
            errors.append(f"agent {a}: starts at {path[0]}, not {instance.starts[a]}")
        if path[-1] != instance.goals[a]:
            errors.append(f"agent {a}: ends at {path[-1]}, not {instance.goals[a]}")
        for t in range(len(path)):
            if not grid.is_free(path[t]):
                errors.append(f"agent {a}: on blocked cell {path[t]} at t={t}")
            elif t > 0 and path[t] != path[t - 1] and path[t] not in grid.neighbors(path[t - 1]):
                errors.append(f"agent {a}: jumps {path[t - 1]}->{path[t]} at t={t}")
    if errors:
        return errors

    for t in range(max(len(p) for p in paths)):
        for a in range(len(paths)):
            for b in range(a + 1, len(paths)):
                here_a, here_b = position(paths[a], t), position(paths[b], t)
                if here_a == here_b:
                    errors.append(f"agents {a},{b}: both at {here_a} at t={t}")
                elif t > 0 and here_a == position(paths[b], t - 1) and here_b == position(paths[a], t - 1):
                    errors.append(f"agents {a},{b}: swap {here_b}<->{here_a} at t={t}")
    return errors
