"""4-connected grid maps in the MovingAI format.

A cell is an ``(x, y)`` tuple; ``y = 0`` is the first row of the map file.
"""

from pathlib import Path

FREE_CHARS = ".GS"  # every other character (@, T, O, W, ...) is an obstacle


class Grid:
    def __init__(self, rows: list[str], name: str = ""):
        self.rows = rows
        self.width = len(rows[0])
        self.height = len(rows)
        self.name = name

    def is_free(self, cell: tuple[int, int]) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height and self.rows[y][x] in FREE_CHARS

    def neighbors(self, cell: tuple[int, int]) -> list[tuple[int, int]]:
        """Free cells one step up, right, down or left."""
        x, y = cell
        candidates = [(x, y - 1), (x + 1, y), (x, y + 1), (x - 1, y)]
        return [c for c in candidates if self.is_free(c)]

    def free_cells(self) -> list[tuple[int, int]]:
        return [(x, y) for y in range(self.height) for x in range(self.width) if self.is_free((x, y))]


def load_map(path: str | Path) -> Grid:
    """Read a MovingAI .map file: a header (type, height, width), a "map" line, then the rows."""
    path = Path(path)
    lines = path.read_text().splitlines()
    start = lines.index("map") + 1
    height = int(lines[1].split()[1])
    width = int(lines[2].split()[1])
    rows = lines[start : start + height]
    if len(rows) != height or any(len(row) != width for row in rows):
        raise ValueError(f"{path}: expected {height} rows of width {width}")
    return Grid(rows, name=path.stem)
