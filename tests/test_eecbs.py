import pytest

from swarmpath.conflicts import find_conflicts
from swarmpath.grid import Grid
from swarmpath.heuristics import bfs_distances
from swarmpath.instance import Instance, random_instance
from swarmpath.solution import soc, validate
from swarmpath.solvers import SOLVERS, eecbs
from swarmpath.spacetime import Constraints

# optimal SOC of the tiny maps, checked with a brute-force search over joint states
OPTIMAL = {"corridor-swap": 15, "dead-end": 7, "bottleneck": 34}
ROW = Grid(["....."])  # cells (0, 0) ... (4, 0)


def focal(grid, start, goal, w, other_paths=(), constraints=None):
    h = bfs_distances(grid, goal)
    return eecbs.focal_search(grid, start, goal, h, constraints or Constraints(), eecbs.ConflictTable(list(other_paths)), w)


@pytest.mark.parametrize("name", OPTIMAL)
@pytest.mark.parametrize("prioritize", [True, False])
@pytest.mark.parametrize("bypass", [True, False])
def test_w1_is_optimal(tiny, name, prioritize, bypass):
    instance = tiny(name)
    result = eecbs.solve(instance, 30, w=1.0, prioritize=prioritize, bypass=bypass)
    assert result.solved and validate(instance, result.paths) == []
    assert soc(result.paths) == OPTIMAL[name] == result.stats["lb"]


@pytest.mark.parametrize("name", OPTIMAL)
@pytest.mark.parametrize("w", [1.2, 1.5, 2.0])
def test_bounded_suboptimal(tiny, name, w):
    instance = tiny(name)
    result = eecbs.solve(instance, 30, w=w)
    assert result.solved and validate(instance, result.paths) == []
    assert result.stats["lb"] <= OPTIMAL[name] <= soc(result.paths) <= w * OPTIMAL[name]


def test_random_instance_within_bound():
    instance = random_instance(Grid(["........"] * 8), 10, seed=3)
    optimal = soc(eecbs.solve(instance, 30, w=1.0).paths)
    for w in [1.1, 1.5]:
        result = eecbs.solve(instance, 30, w=w)
        assert validate(instance, result.paths) == []
        assert optimal <= soc(result.paths) <= w * optimal


def test_focal_search_w1_is_a_shortest_path():
    path, lb = focal(ROW, (0, 0), (4, 0), 1.0)
    assert path == [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)] and lb == 4


def test_focal_search_trades_length_for_fewer_conflicts():
    grid = Grid(["...", "..."])
    blocker = [(1, 0), (1, 0), (1, 0), (1, 1)]  # sits in the middle of the top row until t=2
    shortest, lb = focal(grid, (0, 0), (2, 0), 1.0, [blocker])
    assert len(shortest) - 1 == lb == 2  # straight through the blocker
    detour, lb = focal(grid, (0, 0), (2, 0), 2.0, [blocker])
    assert lb == 2 and len(detour) - 1 <= 2 * lb
    assert find_conflicts([detour, blocker]) == []


def test_focal_search_respects_constraints():
    path, lb = focal(ROW, (0, 0), (4, 0), 1.5, constraints=Constraints(vertex={((2, 0), 2)}))
    assert path[2] != (2, 0) and len(path) - 1 <= 1.5 * lb


def test_conflict_table_counts_parked_agents_and_swaps():
    table = eecbs.ConflictTable([[(1, 0), (2, 0)]])  # arrives at (2, 0) at t=1 and stays
    assert table.count((1, 0), (2, 0), 5) == 1  # driving onto a parked agent
    assert table.count((2, 0), (1, 0), 1) == 1  # swapping with it
    assert table.count((0, 0), (1, 0), 1) == 0  # following it


def test_mdd_widths():
    corridor = eecbs.build_mdd(ROW, (0, 0), (4, 0), bfs_distances(ROW, (4, 0)), Constraints(), 4)
    assert all(len(level) == 1 for level in corridor)
    two_rows = Grid(["...", "..."])
    levels = eecbs.build_mdd(two_rows, (0, 0), (2, 1), bfs_distances(two_rows, (2, 1)), Constraints(), 3)
    assert [len(level) for level in levels] == [1, 2, 2, 1]


def test_corridor_swap_conflict_is_cardinal(tiny):
    instance = tiny("corridor-swap")
    root_paths = [[(x, 1) for x in range(7)], [(x, 1) for x in range(6, -1, -1)]]
    node = eecbs.CTNode([Constraints(), Constraints()], root_paths, [6, 6])
    mdds = {a: eecbs.shortest_path_mdd(node, a, instance) for a in (0, 1)}
    assert all(eecbs.classify(c, mdds) == "cardinal" for c in node.conflicts)


def test_unreachable_goal():
    result = eecbs.solve(Instance(Grid([".@."]), [(0, 0)], [(2, 0)]))
    assert result.paths is None and result.stats["status"] == "unreachable goal"


def test_timeout(tiny):
    result = eecbs.solve(tiny("bottleneck"), time_limit=0.0)
    assert not result.solved and result.stats["status"] == "timeout"


def test_registry():
    assert SOLVERS["eecbs"] is eecbs.solve
