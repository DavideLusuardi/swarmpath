"""Matplotlib animation of a MAPF solution, one frame per timestep; collisions are red."""

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

    # tab20 colours without its two reds, which are kept for collisions
    colors = [plt.get_cmap("tab20")(i) for i in range(20) if i not in (6, 7)]
    circles = []
    for a in range(instance.num_agents):
        color = colors[a % len(colors)]
        gx, gy = instance.goals[a]
        ax.add_patch(Rectangle((gx - 0.3, gy - 0.3), 0.6, 0.6, color=color, alpha=0.5))
        circles.append(ax.add_patch(Circle(instance.starts[a], 0.4, facecolor=color, edgecolor="black")))

    def draw(t: int):
        colliding = {a for c in conflicts if c.t == t for a in (c.a1, c.a2)}
        for a, circle in enumerate(circles):
            circle.center = position(paths[a], t)
            circle.set_facecolor("red" if a in colliding else colors[a % len(colors)])
        ax.set_title(f"{title}   t = {t}   collisions at t: {len(colliding) // 2}")

    animation = FuncAnimation(fig, draw, frames=horizon, interval=300)
    if save:
        animation.save(save, writer=PillowWriter(fps=3))
    if show:
        plt.show()
    plt.close(fig)
