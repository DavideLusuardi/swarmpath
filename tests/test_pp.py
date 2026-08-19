from swarmpath.conflicts import find_conflicts
from swarmpath.grid import Grid
from swarmpath.instance import Instance, random_instance
from swarmpath.solution import validate
from swarmpath.solvers import SOLVERS, pp

EMPTY_8_8 = Grid(["........"] * 8)


def test_hard_pp_fails_in_the_corridor(tiny):
    # agent 0 drives straight to the other end and parks there, on agent 1's start
    result = pp.solve(tiny("corridor-swap"))
    assert result.paths is None and result.stats["status"] == "no path for agent 1"


def test_soft_pp_returns_paths_with_collisions(tiny):
    instance = tiny("corridor-swap")
    result = pp.solve(instance, soft=True)
    assert not result.solved and result.stats["conflicts"] == len(find_conflicts(result.paths)) > 0
    assert [p[0] for p in result.paths] == instance.starts and [p[-1] for p in result.paths] == instance.goals


def test_hard_pp_solutions_are_valid():
    solved = 0
    for seed in range(10):
        instance = random_instance(EMPTY_8_8, 15, seed)
        result = pp.solve(instance)
        if result.paths is not None:
            assert result.solved and validate(instance, result.paths) == []
            solved += 1
    assert solved >= 5


def test_unreachable_goal():
    result = pp.solve(Instance(Grid([".@."]), [(0, 0)], [(2, 0)]), soft=True)
    assert result.paths is None and result.stats["status"] == "no path for agent 0"


def test_registry():
    assert SOLVERS["pp"] is pp.solve
