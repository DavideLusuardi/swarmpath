"""Matplotlib animation of a MAPF solution, one frame per timestep; collisions are marked in red.

Each agent is a numbered circle, its goal a square with the same number and colour, and the
line in its colour is the part of its path already travelled, from its start to where it is.
A vertex collision is a cross on its cell, a swap a bar on the edge between the two cells; a
collision stays marked from the timestep it happens until the end.
"""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Rectangle

from swarmpath.conflicts import Conflict
from swarmpath.instance import Instance
from swarmpath.solution import Path, position


def animate(instance: Instance, paths: list[Path], conflicts: list[Conflict], show=True, save=None, title=""):
    """Show the animation in a window and/or save it as a GIF (``save="out.gif"``)."""
    grid = instance.grid
    horizon = max(len(p) for p in paths)

    fig, ax = plt.subplots(figsize=(8, 8 * grid.height / grid.width + 0.5))
    obstacles = [[0 if grid.is_free((x, y)) else 1 for x in range(grid.width)] for y in range(grid.height)]
    ax.imshow(obstacles, cmap="Greys", vmin=0, vmax=1.5)  # cell (x, y) is drawn centered on (x, y)
    ax.set_xticks([])
    ax.set_yticks([])
    cell_points = 8 * 72 / grid.width  # size of one cell in points, to scale the labels
    font = min(9, 0.45 * cell_points)

    # tab20 colours without its two reds, which are kept for collisions
    colors = [plt.get_cmap("tab20")(i) for i in range(20) if i not in (6, 7)]
    lines, circles, labels = [], [], []
    for a, path in enumerate(paths):
        color = colors[a % len(colors)]
        gx, gy = instance.goals[a]
        ax.add_patch(Rectangle((gx - 0.4, gy - 0.4), 0.8, 0.8, facecolor=color, edgecolor=color, alpha=0.4, lw=2))
        ax.text(gx, gy, str(a), ha="center", va="center", fontsize=font, color="dimgray")
        lines.append(ax.plot([], [], color=color, lw=2, alpha=0.8)[0])
        circles.append(ax.add_patch(Circle(path[0], 0.4, facecolor=color, edgecolor="black", zorder=3)))
        labels.append(ax.text(*path[0], str(a), ha="center", va="center", fontsize=font, zorder=5))
    # Collision marks, under the numbers so the colliding agents can still be read. A swap gets a
    # bar on the edge the two agents cross: vertical for a left-right swap, horizontal otherwise.
    style = dict(color="red", markersize=0.6 * cell_points, mew=max(1.5, cell_points / 16), zorder=4, clip_on=False)
    vertex_marks = ax.plot([], [], "x", linestyle="", label="vertex collision", **style)[0]
    across_marks = ax.plot([], [], "|", linestyle="", label="swap collision", **style)[0]
    along_marks = ax.plot([], [], "_", linestyle="", **style)[0]
    ax.legend(
        handles=[vertex_marks, across_marks],
        loc="upper center",
        bbox_to_anchor=(0.5, 0),
        ncol=2,
        frameon=False,
        markerscale=12 / style["markersize"],
    )

    def draw(t: int):
        for a, path in enumerate(paths):
            travelled = path[: t + 1]  # the whole path once the agent has arrived
            lines[a].set_data([x for x, _ in travelled], [y for _, y in travelled])
            circles[a].center = position(path, t)
            labels[a].set_position(position(path, t))
        happened = [c for c in conflicts if c.t <= t]  # a collision stays marked once it happens
        vertex = [c.u for c in happened if c.kind == "vertex"]
        swaps = [c for c in happened if c.kind == "edge"]
        across = [((c.u[0] + c.v[0]) / 2, c.u[1]) for c in swaps if c.u[1] == c.v[1]]
        along = [(c.u[0], (c.u[1] + c.v[1]) / 2) for c in swaps if c.u[0] == c.v[0]]
        for marks, spots in [(vertex_marks, vertex), (across_marks, across), (along_marks, along)]:
            marks.set_data([x for x, _ in spots], [y for _, y in spots])
        now = sum(1 for c in conflicts if c.t == t)
        ax.set_title(f"{title}   t = {t}   collisions at t: {now}, so far: {len(happened)}")

    fps = 3
    # the last frame is repeated for 3 seconds, so the final state can be seen before the loop
    frames = list(range(horizon)) + [horizon - 1] * (3 * fps)
    animation = FuncAnimation(fig, draw, frames=frames, interval=1000 / fps)
    if save:
        animation.save(save, writer=PillowWriter(fps=fps))
    if show:
        plt.show()
    plt.close(fig)
