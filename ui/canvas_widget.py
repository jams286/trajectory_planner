"""Matplotlib canvas widget embedded in Tkinter with mouse interaction."""

from __future__ import annotations

import tkinter as tk
from typing import Callable, List, Optional, Tuple

import matplotlib
matplotlib.use("TkAgg")

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import config
from algorithms.base_planner import PlannerState
from environment.grid_env import GridEnvironment

# Drawing modes
MODE_DRAW = "draw"
MODE_DRAW_CELL = "draw_cell"
MODE_ERASE = "erase"
MODE_START = "start"
MODE_GOAL = "goal"


class PlannerCanvas:
    """Single Matplotlib axes showing one algorithm's progress."""

    def __init__(self, ax: matplotlib.axes.Axes, title: str = "") -> None:
        self.ax = ax
        self.title = title
        self._grid_img = None
        self._grid_lines_h = []
        self._grid_lines_v = []
        self._show_grid = False
        self._open_scatter = None
        self._closed_scatter = None
        self._current_scatter = None
        self._path_line = None
        self._tree_lines = None
        self._start_scatter = None
        self._goal_scatter = None

    def init_plot(self, env: GridEnvironment) -> None:
        self.ax.clear()
        self.ax.set_title(self.title, fontsize=11, fontweight="bold")
        self.ax.set_xlim(-0.5, env.cols - 0.5)
        self.ax.set_ylim(env.rows - 0.5, -0.5)
        self.ax.set_aspect("equal")
        self.ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

        # Grid image
        display = np.where(env.get_grid(), 0.15, 1.0)
        self._grid_img = self.ax.imshow(
            display, cmap="gray", vmin=0, vmax=1,
            extent=(-0.5, env.cols - 0.5, env.rows - 0.5, -0.5),
            interpolation="nearest",
        )

        # Grid lines (hidden by default)
        self._grid_lines_h = []
        self._grid_lines_v = []
        for r in range(env.rows + 1):
            line, = self.ax.plot(
                [-0.5, env.cols - 0.5], [r - 0.5, r - 0.5],
                color=config.COLOR_GRID_LINE, linewidth=0.4, zorder=0.5,
                visible=self._show_grid,
            )
            self._grid_lines_h.append(line)
        for c in range(env.cols + 1):
            line, = self.ax.plot(
                [c - 0.5, c - 0.5], [-0.5, env.rows - 0.5],
                color=config.COLOR_GRID_LINE, linewidth=0.4, zorder=0.5,
                visible=self._show_grid,
            )
            self._grid_lines_v.append(line)

        # Scatter / line placeholders
        self._open_scatter = self.ax.scatter([], [], s=12, c=config.COLOR_OPEN_SET, zorder=2)
        self._closed_scatter = self.ax.scatter([], [], s=8, c=config.COLOR_CLOSED_SET, zorder=1)
        self._current_scatter = self.ax.scatter([], [], s=30, c="#E67E22", marker="D", zorder=4)
        self._path_line, = self.ax.plot([], [], color=config.COLOR_PATH, linewidth=2.5, zorder=5)
        self._tree_lines = None  # created on demand
        self._start_scatter = self.ax.scatter([], [], s=80, c=config.COLOR_START, marker="^", zorder=6)
        self._goal_scatter = self.ax.scatter([], [], s=80, c=config.COLOR_GOAL, marker="*", zorder=6)

    def update_grid(self, env: GridEnvironment) -> None:
        if self._grid_img is not None:
            display = np.where(env.get_grid(), 0.15, 1.0)
            self._grid_img.set_data(display)

    def draw_start_goal(self, start: Tuple[float, float], goal: Tuple[float, float]) -> None:
        self._start_scatter.set_offsets([[start[1], start[0]]])
        self._goal_scatter.set_offsets([[goal[1], goal[0]]])

    def update_state(self, state: PlannerState) -> None:
        # Open set
        if state.open_set:
            pts = np.array(state.open_set)
            self._open_scatter.set_offsets(pts[:, ::-1])  # (row,col) → (col,row)
        else:
            self._open_scatter.set_offsets(np.empty((0, 2)))

        # Closed set
        if state.closed_set:
            pts = np.array(state.closed_set)
            self._closed_scatter.set_offsets(pts[:, ::-1])
        else:
            self._closed_scatter.set_offsets(np.empty((0, 2)))

        # Current node
        if state.current:
            self._current_scatter.set_offsets([[state.current[1], state.current[0]]])
        else:
            self._current_scatter.set_offsets(np.empty((0, 2)))

        # RRT tree edges
        if state.tree_edges:
            if self._tree_lines is not None:
                self._tree_lines.remove()
            xs, ys = [], []
            for (r1, c1), (r2, c2) in state.tree_edges:
                xs.extend([c1, c2, None])
                ys.extend([r1, r2, None])
            self._tree_lines, = self.ax.plot(
                xs, ys, color=config.COLOR_RRT_TREE, linewidth=0.6, alpha=0.6, zorder=2,
            )

        # Path
        if state.path:
            rows = [p[0] for p in state.path]
            cols = [p[1] for p in state.path]
            self._path_line.set_data(cols, rows)
        else:
            self._path_line.set_data([], [])

    def set_grid_visible(self, visible: bool) -> None:
        self._show_grid = visible
        for line in self._grid_lines_h + self._grid_lines_v:
            line.set_visible(visible)

    def clear_state(self) -> None:
        self.update_state(PlannerState())
        if self._tree_lines is not None:
            self._tree_lines.remove()
            self._tree_lines = None


class CanvasWidget:
    """Tkinter frame containing dual Matplotlib canvases + mouse handlers."""

    def __init__(
        self,
        parent: tk.Frame,
        env: GridEnvironment,
        on_start_changed: Optional[Callable] = None,
        on_goal_changed: Optional[Callable] = None,
        on_obstacle_drawn: Optional[Callable] = None,
        on_obstacle_erased: Optional[Callable] = None,
    ) -> None:
        self.parent = parent
        self.env = env
        self.mode = MODE_DRAW

        # Callbacks
        self._on_start = on_start_changed
        self._on_goal = on_goal_changed
        self._on_draw = on_obstacle_drawn
        self._on_erase = on_obstacle_erased

        # Create matplotlib figure with two subplots
        self.fig = Figure(figsize=(12, 5.5), dpi=100, facecolor="#F8F9FA")
        self.fig.subplots_adjust(left=0.02, right=0.98, top=0.92, bottom=0.02, wspace=0.08)

        self.ax_astar = self.fig.add_subplot(1, 2, 1)
        self.ax_rrt = self.fig.add_subplot(1, 2, 2)

        self.canvas_astar = PlannerCanvas(self.ax_astar, "A*")
        self.canvas_rrt = PlannerCanvas(self.ax_rrt, "RRT")

        # Embed in Tkinter
        self.tk_canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.tk_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Mouse state
        self._drag_start: Optional[Tuple[int, int]] = None
        self._drag_rect = None

        # Connect mouse events
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)

    def init_plots(self, start: Tuple[float, float], goal: Tuple[float, float]) -> None:
        for canvas in (self.canvas_astar, self.canvas_rrt):
            canvas.init_plot(self.env)
            canvas.draw_start_goal(start, goal)
        self.tk_canvas.draw_idle()

    def refresh_grid(self) -> None:
        self.canvas_astar.update_grid(self.env)
        self.canvas_rrt.update_grid(self.env)
        self.tk_canvas.draw_idle()

    def draw(self) -> None:
        self.tk_canvas.draw_idle()

    def set_mode(self, mode: str) -> None:
        self.mode = mode

    def toggle_grid(self) -> None:
        show = not self.canvas_astar._show_grid
        self.canvas_astar.set_grid_visible(show)
        self.canvas_rrt.set_grid_visible(show)
        self.tk_canvas.draw_idle()
        return show

    # ------------------------------------------------------------------
    # Mouse event handlers
    # ------------------------------------------------------------------

    def _pixel_to_cell(self, event) -> Optional[Tuple[int, int]]:
        """Convert matplotlib event to (row, col). Uses the left axes."""
        if event.inaxes not in (self.ax_astar, self.ax_rrt):
            return None
        col = int(round(event.xdata))
        row = int(round(event.ydata))
        if 0 <= row < self.env.rows and 0 <= col < self.env.cols:
            return (row, col)
        return None

    def _on_press(self, event) -> None:
        cell = self._pixel_to_cell(event)
        if cell is None:
            return

        if self.mode == MODE_START:
            if self._on_start:
                self._on_start(cell)
        elif self.mode == MODE_GOAL:
            if self._on_goal:
                self._on_goal(cell)
        elif self.mode == MODE_DRAW:
            self._drag_start = cell
        elif self.mode == MODE_DRAW_CELL:
            # Single-cell drawing mode
            if self._on_draw:
                self._on_draw(cell[1], cell[0], 1, 1)
        elif self.mode == MODE_ERASE:
            if self._on_erase:
                self._on_erase(cell)

    def _on_motion(self, event) -> None:
        if self.mode == MODE_DRAW_CELL:
            # Continuous cell painting while dragging
            if event.button == 1:
                cell = self._pixel_to_cell(event)
                if cell is not None and self._on_draw:
                    self._on_draw(cell[1], cell[0], 1, 1)
        elif self.mode == MODE_ERASE:
            # Continuous erasing while dragging
            if event.button == 1:
                cell = self._pixel_to_cell(event)
                if cell is not None and self._on_erase:
                    self._on_erase(cell)
        elif self.mode == MODE_DRAW and self._drag_start is not None:
            cell = self._pixel_to_cell(event)
            if cell is None:
                return
            # Preview rectangle (optional future enhancement)

    def _on_release(self, event) -> None:
        if self.mode == MODE_DRAW and self._drag_start is not None:
            cell = self._pixel_to_cell(event)
            if cell is None:
                cell = self._drag_start
            r1, c1 = self._drag_start
            r2, c2 = cell
            top = min(r1, r2)
            left = min(c1, c2)
            h = abs(r2 - r1) + 1
            w = abs(c2 - c1) + 1
            if self._on_draw:
                self._on_draw(left, top, w, h)
            self._drag_start = None
