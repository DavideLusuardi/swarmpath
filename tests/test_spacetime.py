from swarmpath.grid import Grid
from swarmpath.heuristics import bfs_distances
from swarmpath.spacetime import Constraints, space_time_astar

ROW = Grid(["....."])  # cells (0, 0) ... (4, 0)


def plan(grid, start, goal, constraints=None):
    return space_time_astar(grid, start, goal, bfs_distances(grid, goal), constraints)


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
