import pytest

from conftest import MAPS
from swarmpath.grid import Grid, load_map


def test_only_dot_g_s_are_free():
    grid = Grid([".GS@TOW"])
    assert [grid.is_free((x, 0)) for x in range(7)] == [True, True, True, False, False, False, False]


def test_outside_the_map_is_not_free():
    grid = Grid(["..", ".."])
    assert not grid.is_free((-1, 0)) and not grid.is_free((2, 0)) and not grid.is_free((0, 2))


def test_neighbors_are_4_connected_and_skip_obstacles():
    grid = Grid(["...", ".@.", "..."])
    assert sorted(grid.neighbors((0, 0))) == [(0, 1), (1, 0)]
    assert sorted(grid.neighbors((1, 0))) == [(0, 0), (2, 0)]


def test_load_map():
    grid = load_map(MAPS / "corridor-swap.map")
    assert (grid.width, grid.height, grid.name) == (7, 3, "corridor-swap")
    assert len(grid.free_cells()) == 8
    assert grid.is_free((3, 0)) and not grid.is_free((2, 0))


def test_load_map_rejects_truncated_file(tmp_path):
    bad = tmp_path / "bad.map"
    bad.write_text("type octile\nheight 3\nwidth 2\nmap\n..\n..\n")
    with pytest.raises(ValueError):
        load_map(bad)
