from swarmpath.conflicts import find_conflicts
from swarmpath.grid import Grid
from swarmpath.heuristics import bfs_distances
from swarmpath.spacetime import ConflictTable, Constraints, space_time_astar

ROW = Grid(["....."])  # cells (0, 0) ... (4, 0)
TWO_ROWS = Grid(["...", "..."])


def plan(grid, start, goal, constraints=None, others=(), soft=False):
    return space_time_astar(grid, start, goal, bfs_distances(grid, goal), constraints, ConflictTable(list(others)), soft)


def test_unconstrained_path_is_a_shortest_path():
    grid = Grid([".....", ".@@@.", ".....", "@@@@.", "....."])
    path = plan(grid, (0, 0), (0, 4))
    assert path[0] == (0, 0) and path[-1] == (0, 4)
    assert len(path) - 1 == bfs_distances(grid, (0, 4))[(0, 0)] == 12
    for here, there in zip(path, path[1:]):
        assert there in grid.neighbors(here)


def test_start_equals_goal():
    assert plan(ROW, (1, 0), (1, 0)) == [(1, 0)]


def test_vertex_constraint_forces_a_wait():
    path = plan(ROW, (0, 0), (4, 0), Constraints(vertex={((2, 0), 2)}))
    assert len(path) - 1 == 5
    assert path[2] != (2, 0)


def test_edge_constraint_forbids_one_move():
    path = plan(ROW, (0, 0), (4, 0), Constraints(edge={((1, 0), (2, 0), 2)}))
    assert len(path) - 1 == 5
    assert (path[1], path[2]) != ((1, 0), (2, 0))


def test_waiting_beats_a_longer_detour():
    grid = Grid(["...", "..."])
    path = plan(grid, (0, 0), (2, 0), Constraints(vertex={((1, 0), 1)}))
    assert path == [(0, 0), (0, 0), (1, 0), (2, 0)]  # the detour through the bottom row is 4


def test_later_constraint_on_goal_means_agent_cannot_stop_early():
    path = plan(ROW, (0, 0), (3, 0), Constraints(vertex={((3, 0), 10)}))
    assert path[-1] == (3, 0)
    assert len(path) - 1 == 11  # arrives right after the constraint
    assert path[10] != (3, 0)


def test_unreachable_goal_returns_none():
    assert plan(Grid(["..@.."]), (0, 0), (4, 0)) is None


def test_impossible_constraints_return_none():
    grid = Grid([".."])
    blocked_goal = {((1, 0), t) for t in range(1, 30)}
    must_leave_start = {((0, 0), 5)}  # but the only neighbour is forbidden
    assert plan(grid, (0, 0), (1, 0), Constraints(vertex=blocked_goal | must_leave_start)) is None


def test_table_last_time_and_later_visits():
    table = ConflictTable([[(0, 0), (1, 0), (2, 0)]])
    assert table.last_time() == 2
    assert table.later_visits((1, 0), 0) == 1  # the agent passes (1, 0) at t=1
    assert table.later_visits((1, 0), 1) == 0
    table.add([(4, 0), (4, 0), (4, 0), (3, 0), (1, 1)])
    assert table.last_time() == 4 and table.count((3, 0), (3, 0), 3) == 1


def test_soft_mode_without_others_is_a_shortest_path():
    assert plan(ROW, (0, 0), (4, 0), soft=True) == [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]


def test_both_modes_wait_for_another_agent():
    other = [(1, 1), (1, 0), (1, 1)]  # crosses the top row at t=1, then parks at (1, 1)
    for soft in [False, True]:
        path = plan(TWO_ROWS, (0, 0), (2, 0), others=[other], soft=soft)
        assert len(path) - 1 == 3 and find_conflicts([path, other]) == []


def test_parked_agent_on_the_only_route():
    other = [(2, 0)]  # parked in the middle of the row from t=0
    assert plan(ROW, (0, 0), (4, 0), others=[other]) is None
    path = plan(ROW, (0, 0), (4, 0), others=[other], soft=True)
    assert path == [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
    assert len(find_conflicts([path, other])) == 1


def test_do_not_park_where_someone_passes_later():
    other = [(2, 0), (2, 0), (2, 0), (1, 0), (1, 1), (0, 1)]  # over (1, 0) at t=3
    for soft in [False, True]:
        path = plan(TWO_ROWS, (0, 0), (1, 0), others=[other], soft=soft)
        assert path[-1] == (1, 0) and len(path) - 1 == 4
        assert find_conflicts([path, other]) == []


def test_soft_mode_counts_collisions_after_parking():
    other = [(2, 0), (1, 0), (0, 0)]  # drives through the only goal cell at t=1, parks at (0, 0)
    path = plan(Grid(["..."]), (1, 0), (1, 0), others=[other], soft=True)
    assert path[-1] == (1, 0) and len(find_conflicts([path, other])) == 1
