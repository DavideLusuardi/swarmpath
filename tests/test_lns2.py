from random import Random

import pytest

from swarmpath.grid import Grid
from swarmpath.instance import Instance, random_instance
from swarmpath.solution import validate
from swarmpath.solvers import SOLVERS, independent, lns2

EMPTY_8_8 = Grid(["........"] * 8)


@pytest.mark.parametrize("name", ["dead-end", "bottleneck"])
def test_solves_tiny_maps(tiny, name):
    instance = tiny(name)
    result = lns2.solve(instance, 30)
    assert result.solved and validate(instance, result.paths) == []


def test_cannot_solve_the_corridor_swap(tiny):
    # Prioritized planning ignores the agents planned after it: whoever goes first drives
    # straight through, and the other cannot reach the side pocket in time. The solution needs
    # the first agent to wait for the second, so LNS2 (like PP) is incomplete.
    result = lns2.solve(tiny("corridor-swap"), 0.5)
    assert not result.solved and result.stats["pairs"] == 1 and result.stats["iterations"] > 0


@pytest.mark.parametrize("seed", range(3))
def test_solves_crowded_random_instance(seed):
    instance = random_instance(EMPTY_8_8, 24, seed)
    result = lns2.solve(instance, 30, seed=seed)
    assert result.solved and validate(instance, result.paths) == []


@pytest.mark.parametrize("heuristic", ["collision", "failure", "random"])
def test_each_heuristic_alone_solves(heuristic):
    instance = random_instance(EMPTY_8_8, 20, 1)
    switches = {name: name == heuristic for name in lns2.NEIGHBORHOODS}
    result = lns2.solve(instance, 30, **switches)
    assert result.solved and validate(instance, result.paths) == []
    assert set(result.stats["used"]) == {heuristic}


@pytest.mark.parametrize("name", ["collision", "failure", "random"])
def test_neighborhoods(name):
    instance = random_instance(EMPTY_8_8, 30, 2)
    paths = independent.solve(instance).paths
    graph = lns2.collision_graph(paths)
    rng = Random(0)
    for size in [2, 5, 8]:
        for _ in range(20):
            agents = lns2.NEIGHBORHOODS[name](instance, paths, graph, size, rng)
            assert len(agents) == len(set(agents)) <= size
            assert any(graph[a] for a in agents)


def test_collision_neighborhood_takes_a_whole_small_component():
    paths = [[(0, 0), (1, 0)], [(1, 0), (0, 0)], [(3, 0), (3, 1)], [(5, 5)]]  # agents 0, 1 swap
    instance = Instance(Grid(["......"] * 6), [p[0] for p in paths], [p[-1] for p in paths])
    graph = lns2.collision_graph(paths)
    assert lns2.colliding_pairs(graph) == 1
    agents = lns2.collision_neighborhood(instance, paths, graph, 8, Random(0))
    assert {0, 1} <= set(agents)


def test_agent_in_the_way_finds_a_swap():
    # Agent 1 swaps with agent 0 between t=0 and t=1 but never stands where the walk is. In a
    # row, the walk can only follow agent 0's path; it meets agent 1 when it starts at t=0.
    paths = [[(0, 0), (1, 0), (2, 0)], [(1, 0), (0, 0)]]
    instance = Instance(Grid(["....."]), [p[0] for p in paths], [p[-1] for p in paths])
    met = {lns2.agent_in_the_way(instance, paths, 0, [0], Random(seed)) for seed in range(20)}
    assert met == {1, None}


def test_same_seed_same_result():
    instance = random_instance(EMPTY_8_8, 24, 0)
    first, second = lns2.solve(instance, 30, seed=5), lns2.solve(instance, 30, seed=5)
    assert first.paths == second.paths and first.stats["iterations"] == second.stats["iterations"]


def test_timeout_keeps_the_initial_paths(tiny):
    result = lns2.solve(tiny("corridor-swap"), time_limit=0.0)
    assert result.paths is not None and not result.solved and result.stats["status"] == "timeout"
    assert result.stats["pairs"] == result.stats["initial_pairs"] > 0


def test_unreachable_goal():
    result = lns2.solve(Instance(Grid([".@."]), [(0, 0)], [(2, 0)]))
    assert result.paths is None and result.stats["status"] == "unreachable goal"


def test_registry():
    assert SOLVERS["lns2"] is lns2.solve
