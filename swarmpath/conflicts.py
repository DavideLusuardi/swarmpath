"""Conflicts between pairs of agents (an agent keeps occupying its goal after arriving)."""

from dataclasses import dataclass

from swarmpath.solution import Path, position


@dataclass
class Conflict:
    """vertex: a1 and a2 are both at cell u at time t.
    edge:   a1 moves u->v while a2 moves v->u, both arriving at time t.
    """

    kind: str  # "vertex" or "edge"
    a1: int
    a2: int
    t: int
    u: tuple[int, int]
    v: tuple[int, int] | None = None


def find_conflicts(paths: list[Path]) -> list[Conflict]:
    """All conflicts, ordered by time."""
    conflicts = []
    for t in range(max(len(p) for p in paths)):
        for a1 in range(len(paths)):
            for a2 in range(a1 + 1, len(paths)):
                here1, here2 = position(paths[a1], t), position(paths[a2], t)
                if here1 == here2:
                    conflicts.append(Conflict("vertex", a1, a2, t, here1))
                elif t > 0:
                    before1, before2 = position(paths[a1], t - 1), position(paths[a2], t - 1)
                    if here1 == before2 and here2 == before1:
                        conflicts.append(Conflict("edge", a1, a2, t, before1, here1))
    return conflicts
