"""GIF / video export of algorithm execution."""

from __future__ import annotations

import io
from pathlib import Path
from typing import Generator, List, Optional, Tuple

import numpy as np
from matplotlib.figure import Figure

from algorithms.base_planner import BasePlanner, PlannerState
from environment.grid_env import GridEnvironment
import config


def export_gif(
    env: GridEnvironment,
    planners: dict[str, BasePlanner],
    start: Tuple[float, float],
    goal: Tuple[float, float],
    output_path: str = "animation.gif",
    steps_per_frame: int = 5,
    fps: int = 15,
    max_frames: int = 500,
    dpi: int = 100,
) -> str:
    """Run planners and save an animated GIF of the dual visualisation.

    Parameters
    ----------
    env : GridEnvironment
    planners : dict mapping name → BasePlanner (expects "A*" and/or "RRT")
    start, goal : coordinates
    output_path : file path for the GIF
    steps_per_frame : algorithm iterations per animation frame
    fps : frames per second in the output GIF
    max_frames : safety cap on total frames
    dpi : resolution of each frame

    Returns
    -------
    str : absolute path of the saved file
    """
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from PIL import Image

    # ── Setup figure ──────────────────────────────────────────────────
    fig = Figure(figsize=(12, 5.5), dpi=dpi, facecolor="#F8F9FA")
    fig.subplots_adjust(left=0.02, right=0.98, top=0.92, bottom=0.02, wspace=0.08)
    canvas = FigureCanvasAgg(fig)

    axes = {}
    titles = list(planners.keys())
    n = len(titles)
    for i, name in enumerate(titles):
        ax = fig.add_subplot(1, n, i + 1)
        ax.set_title(name, fontsize=11, fontweight="bold")
        ax.set_xlim(-0.5, env.cols - 0.5)
        ax.set_ylim(env.rows - 0.5, -0.5)
        ax.set_aspect("equal")
        ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
        axes[name] = ax

    # ── Generators ────────────────────────────────────────────────────
    gens = {name: p.step_generator(start, goal) for name, p in planners.items()}
    done = {name: False for name in planners}
    last_state = {name: PlannerState() for name in planners}

    frames: List[Image.Image] = []

    for frame_num in range(max_frames):
        # Advance each planner
        for name in planners:
            if done[name]:
                continue
            for _ in range(steps_per_frame):
                try:
                    state = next(gens[name])
                    last_state[name] = state
                    if state.done:
                        done[name] = True
                        break
                except StopIteration:
                    done[name] = True
                    break

        # ── Draw frame ────────────────────────────────────────────────
        for name, ax in axes.items():
            ax.clear()
            ax.set_title(name, fontsize=11, fontweight="bold")
            ax.set_xlim(-0.5, env.cols - 0.5)
            ax.set_ylim(env.rows - 0.5, -0.5)
            ax.set_aspect("equal")
            ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

            # Grid
            display = np.where(env.get_grid(), 0.15, 1.0)
            ax.imshow(
                display, cmap="gray", vmin=0, vmax=1,
                extent=(-0.5, env.cols - 0.5, env.rows - 0.5, -0.5),
                interpolation="nearest",
            )

            state = last_state[name]

            # Closed set
            if state.closed_set:
                pts = np.array(state.closed_set)
                ax.scatter(pts[:, 1], pts[:, 0], s=4, c=config.COLOR_CLOSED_SET, zorder=1)

            # Open set
            if state.open_set:
                pts = np.array(state.open_set)
                ax.scatter(pts[:, 1], pts[:, 0], s=6, c=config.COLOR_OPEN_SET, zorder=2)

            # Tree edges
            if state.tree_edges:
                xs, ys = [], []
                for (r1, c1), (r2, c2) in state.tree_edges:
                    xs.extend([c1, c2, None])
                    ys.extend([r1, r2, None])
                ax.plot(xs, ys, color=config.COLOR_RRT_TREE, linewidth=0.5, alpha=0.5, zorder=2)

            # Path
            if state.path:
                rows = [p[0] for p in state.path]
                cols = [p[1] for p in state.path]
                ax.plot(cols, rows, color=config.COLOR_PATH, linewidth=2.5, zorder=5)

            # Current
            if state.current:
                ax.scatter([state.current[1]], [state.current[0]],
                           s=25, c="#E67E22", marker="D", zorder=4)

            # Start / Goal
            ax.scatter([start[1]], [start[0]], s=60, c=config.COLOR_START, marker="^", zorder=6)
            ax.scatter([goal[1]], [goal[0]], s=60, c=config.COLOR_GOAL, marker="*", zorder=6)

        # Rasterize to PIL Image
        canvas.draw()
        buf = canvas.buffer_rgba()
        img = Image.frombuffer("RGBA", canvas.get_width_height(), buf, "raw", "RGBA", 0, 1)
        frames.append(img.copy())

        if all(done.values()):
            # Add a few extra frames so the final path is visible
            for _ in range(int(fps * 1.5)):
                frames.append(img.copy())
            break

    # ── Save GIF ──────────────────────────────────────────────────────
    out = Path(output_path).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    frames[0].save(
        str(out),
        save_all=True,
        append_images=frames[1:],
        duration=int(1000 / fps),
        loop=0,
        optimize=True,
    )

    return str(out)
