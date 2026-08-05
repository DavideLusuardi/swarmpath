import pytest

from conftest import MAPS
from swarmpath.grid import Grid, load_map
from swarmpath.heuristics import bfs_distances
from swarmpath.instance import load_scen, random_instance


def test_load_scen(tiny):
    instance = tiny("corridor-swap")
    assert instance.num_agents == 2
    assert instance.starts == [(0, 1), (6, 1)]
    assert instance.goals == [(6, 1), (0, 1)]


def test_load_scen_first_k(tiny):
    assert tiny("bottleneck", 3).num_agents == 3


def test_load_scen_too_many_agents(tiny):
    with pytest.raises(ValueError):
        tiny("corridor-swap", 5)


def test_load_scen_checks_map_size():
    with pytest.raises(ValueError):
        load_scen(MAPS / "corridor-swap.scen", load_map(MAPS / "bottleneck.map"), 2)


def test_random_instance_is_reproducible_and_solvable():
    grid = Grid(["..@.....", "..@.....", "..@....."])
    instance = random_instance(grid, 6, seed=3)
    assert instance == random_instance(grid, 6, seed=3)
    assert len(set(instance.starts)) == 6 and len(set(instance.goals)) == 6
    for start, goal in zip(instance.starts, instance.goals):
        assert start in bfs_distances(grid, goal)
        assert start[0] > 2  # all in the larger area right of the wall
