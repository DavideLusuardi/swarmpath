import pytest

from swarmpath.conflicts import find_conflicts
from swarmpath.grid import Grid
from swarmpath.instance import Instance, random_instance
from swarmpath.solution import makespan, soc, validate
from swarmpath.solvers import independent

GRID = Grid(["....", "..@.", "...."])


def test_soc_and_makespan():
    paths = [[(0, 0), (1, 0), (2, 0)], [(3, 2), (3, 1)]]
    assert soc(paths) == 3
    assert makespan(paths) == 2


def test_valid_solution():
    instance = Instance(GRID, [(0, 0), (0, 2)], [(2, 0), (2, 2)])
    assert validate(instance, [[(0, 0), (1, 0), (2, 0)], [(0, 2), (1, 2), (2, 2)]]) == []


@pytest.mark.parametrize(
    "starts, goals, paths, message",
    [
        ([(0, 0)], [(2, 0)], [], "expected 1 paths"),
        ([(0, 0)], [(2, 0)], [[(1, 0), (2, 0)]], "starts at"),
        ([(0, 0)], [(2, 0)], [[(0, 0), (1, 0)]], "ends at"),
        ([(0, 0)], [(2, 0)], [[(0, 0), (2, 0)]], "jumps"),
        ([(2, 0)], [(2, 2)], [[(2, 0), (2, 1), (2, 2)]], "blocked cell"),
        ([(0, 0), (2, 0)], [(2, 0), (0, 0)], [[(0, 0), (1, 0), (2, 0)], [(2, 0), (1, 0), (0, 0)]], "both at"),
        ([(0, 0), (1, 0)], [(1, 0), (0, 0)], [[(0, 0), (1, 0)], [(1, 0), (0, 0)]], "swap"),
        # agent 0 parks on (2, 0) at t=1, agent 1 drives through it at t=2
        ([(1, 0), (0, 0)], [(2, 0), (3, 0)], [[(1, 0), (2, 0)], [(0, 0), (1, 0), (2, 0), (3, 0)]], "both at"),
    ],
)
def test_validate_catches(starts, goals, paths, message):
    errors = validate(Instance(GRID, starts, goals), paths)
    assert any(message in e for e in errors), errors


def test_validator_agrees_with_conflict_detection():
    for seed in range(20):
        instance = random_instance(GRID, 6, seed)
        paths = independent.solve(instance).paths
        assert bool(validate(instance, paths)) == bool(find_conflicts(paths))
