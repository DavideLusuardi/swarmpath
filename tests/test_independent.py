from swarmpath.grid import Grid
from swarmpath.instance import Instance
from swarmpath.solution import makespan, soc, validate
from swarmpath.solvers import SOLVERS, independent


def test_corridor_swap_collides(tiny):
    instance = tiny("corridor-swap")
    result = independent.solve(instance)
    assert not result.solved
    assert soc(result.paths) == 12 and makespan(result.paths) == 6
    assert result.stats["conflicts"] > 0
    assert all("both at" in e or "swap" in e for e in validate(instance, result.paths))


def test_parked_agent_collides(tiny):
    result = independent.solve(tiny("dead-end"))
    assert not result.solved and result.stats["conflicts"] == 1


def test_disjoint_agents_are_solved():
    instance = Instance(Grid(["....", "...."]), [(0, 0), (0, 1)], [(3, 0), (3, 1)])
    result = independent.solve(instance)
    assert result.solved and soc(result.paths) == 6
    assert validate(instance, result.paths) == []


def test_unreachable_goal():
    result = independent.solve(Instance(Grid([".@."]), [(0, 0)], [(2, 0)]))
    assert result.paths is None and not result.solved


def test_registry():
    assert SOLVERS["independent"] is independent.solve
