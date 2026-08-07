"""Matplotlib animation of a MAPF solution, one frame per timestep; collisions are red.

Each agent is a numbered circle, its goal a square with the same number and colour, and the
line in its colour is the part of its path already travelled, from its start to where it is.
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
        labels.append(ax.text(*path[0], str(a), ha="center", va="center", fontsize=font, zorder=4))

    def draw(t: int):
        colliding = {a for c in conflicts if c.t == t for a in (c.a1, c.a2)}
        for a, path in enumerate(paths):
            travelled = path[: t + 1]  # the whole path once the agent has arrived
            lines[a].set_data([x for x, _ in travelled], [y for _, y in travelled])
            circles[a].center = position(path, t)
            circles[a].set_facecolor("red" if a in colliding else colors[a % len(colors)])
            labels[a].set_position(position(path, t))
        ax.set_title(f"{title}   t = {t}   collisions at t: {len(colliding) // 2}")

    animation = FuncAnimation(fig, draw, frames=horizon, interval=300)
    if save:
        animation.save(save, writer=PillowWriter(fps=3))
    if show:
        plt.show()
    plt.close(fig)
