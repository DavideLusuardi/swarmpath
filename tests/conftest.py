from pathlib import Path

import matplotlib
import pytest

from swarmpath.grid import load_map
from swarmpath.instance import load_scen

matplotlib.use("Agg")  # no windows during tests

MAPS = Path(__file__).parent / "maps"


@pytest.fixture
def tiny():
    """Load a tiny instance from tests/maps by name, e.g. ``tiny("corridor-swap")``."""

    def load(name, k=None):
        scen = MAPS / f"{name}.scen"
        if k is None:
            k = len(scen.read_text().splitlines()) - 1
        return load_scen(scen, load_map(MAPS / f"{name}.map"), k)

    return load
