from swarmpath.grid import Grid
from swarmpath.heuristics import bfs_distances


def test_bfs_distances_go_around_obstacles():
    grid = Grid(["...", "@@.", "..."])
    dist = bfs_distances(grid, (0, 2))
    assert dist[(0, 2)] == 0
    assert dist[(2, 2)] == 2
    assert dist[(0, 0)] == 6
    assert (0, 1) not in dist  # obstacle


def test_unreachable_cells_are_missing():
    grid = Grid(["..@.."])
    assert bfs_distances(grid, (0, 0)) == {(0, 0): 0, (1, 0): 1}
