"""Main application window — orchestrates environment, algorithms, and UI."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Generator, Optional

import config
from algorithms.astar import AStarPlanner
from algorithms.base_planner import PlannerState
from algorithms.rrt import RRTPlanner
from environment.grid_env import GridEnvironment
from environment.map_presets import PRESETS
from environment.obstacle import StaticObstacle
from metrics.evaluator import MetricsCollector
from ui.canvas_widget import CanvasWidget
from ui.control_panel import ControlPanel
from ui.metrics_panel import MetricsPanel


class MainApp:
    """Top-level application class."""

    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("Trajectory Planner — A* vs RRT")
        self.root.geometry("1280x780")
        self.root.minsize(1000, 600)

        # ── State ─────────────────────────────────────────────────────
        self.start = config.DEFAULT_START
        self.goal = config.DEFAULT_GOAL
        self.env = GridEnvironment()
        self.metrics = MetricsCollector()

        self._astar_gen: Optional[Generator[PlannerState, None, None]] = None
        self._rrt_gen: Optional[Generator[PlannerState, None, None]] = None
        self._astar_done = False
        self._rrt_done = False
        self._running = False
        self._after_id: Optional[str] = None

        # ── Layout ────────────────────────────────────────────────────
        #  ┌──────────┬──────────────────────────────────┐
        #  │ Controls │         Canvas (dual)            │
        #  │          │                                  │
        #  │          ├──────────────────────────────────┤
        #  │          │        Metrics panel             │
        #  └──────────┴──────────────────────────────────┘

        left = ttk.Frame(self.root)
        left.pack(side=tk.LEFT, fill=tk.Y)

        right = ttk.Frame(self.root)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        canvas_frame = ttk.Frame(right)
        canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        metrics_frame = ttk.Frame(right)
        metrics_frame.pack(side=tk.BOTTOM, fill=tk.X)

        # ── Widgets ───────────────────────────────────────────────────
        self.control = ControlPanel(left, callbacks={
            "preset": self._load_preset,
            "mode": self._change_mode,
            "run": self._run,
            "step": self._step,
            "pause": self._pause,
            "reset": self._reset,
            "clear_obstacles": self._clear_obstacles,
        })
        self.control.pack(fill=tk.Y, expand=True)

        self.canvas = CanvasWidget(
            canvas_frame, self.env,
            on_start_changed=self._set_start,
            on_goal_changed=self._set_goal,
            on_obstacle_drawn=self._add_obstacle,
            on_obstacle_erased=self._erase_obstacle,
        )

        self.metrics_panel = MetricsPanel(metrics_frame)
        self.metrics_panel.pack(fill=tk.X)

        # ── Initial render ────────────────────────────────────────────
        self._load_preset("Maze")

    # ==================================================================
    # Preset / environment management
    # ==================================================================

    def _load_preset(self, name: str) -> None:
        self._stop_animation()
        factory = PRESETS.get(name)
        if factory is None:
            return
        obstacles = factory()
        self.env = GridEnvironment(obstacles=obstacles)
        self.canvas.env = self.env
        self.canvas.init_plots(self.start, self.goal)
        self.metrics.reset()
        self.metrics_panel.clear()

    def _clear_obstacles(self) -> None:
        self._stop_animation()
        self.env.clear_obstacles()
        self.canvas.init_plots(self.start, self.goal)
        self.metrics.reset()
        self.metrics_panel.clear()

    def _add_obstacle(self, x: int, y: int, w: int, h: int) -> None:
        self.env.add_obstacle(StaticObstacle(x=x, y=y, width=w, height=h))
        self.canvas.init_plots(self.start, self.goal)

    def _erase_obstacle(self, cell: tuple) -> None:
        row, col = cell
        self.env.remove_obstacles_at(row, col)
        self.canvas.init_plots(self.start, self.goal)

    # ==================================================================
    # Start / goal
    # ==================================================================

    def _set_start(self, cell: tuple) -> None:
        self.start = cell
        self.canvas.canvas_astar.draw_start_goal(self.start, self.goal)
        self.canvas.canvas_rrt.draw_start_goal(self.start, self.goal)
        self.canvas.draw()

    def _set_goal(self, cell: tuple) -> None:
        self.goal = cell
        self.canvas.canvas_astar.draw_start_goal(self.start, self.goal)
        self.canvas.canvas_rrt.draw_start_goal(self.start, self.goal)
        self.canvas.draw()

    def _change_mode(self, mode: str) -> None:
        self.canvas.set_mode(mode)

    # ==================================================================
    # Algorithm execution
    # ==================================================================

    def _create_generators(self) -> None:
        astar = AStarPlanner(self.env, heuristic=self.control.heuristic_var.get())
        rrt = RRTPlanner(
            self.env,
            step_size=self.control.step_size_var.get(),
            max_iter=self.control.max_iter_var.get(),
            goal_bias=self.control.goal_bias_var.get(),
            goal_threshold=self.control.goal_thresh_var.get(),
        )
        self._astar_gen = astar.step_generator(self.start, self.goal)
        self._rrt_gen = rrt.step_generator(self.start, self.goal)
        self._astar_done = False
        self._rrt_done = False

        self.metrics.reset()
        self.metrics.start_timer("A*")
        self.metrics.start_timer("RRT")

    def _run(self) -> None:
        """Start animated execution of both algorithms."""
        self._stop_animation()
        # Clear previous visualisation
        self.canvas.canvas_astar.clear_state()
        self.canvas.canvas_rrt.clear_state()
        self.canvas.init_plots(self.start, self.goal)

        self._create_generators()
        self._running = True
        self._tick()

    def _step(self) -> None:
        """Advance one frame manually."""
        if self._astar_gen is None:
            self._stop_animation()
            self.canvas.canvas_astar.clear_state()
            self.canvas.canvas_rrt.clear_state()
            self.canvas.init_plots(self.start, self.goal)
            self._create_generators()
        self._running = False
        self._advance_frame()

    def _pause(self) -> None:
        if self._running:
            self._running = False
            if self._after_id:
                self.root.after_cancel(self._after_id)
                self._after_id = None
        else:
            # Resume
            self._running = True
            self._tick()

    def _reset(self) -> None:
        self._stop_animation()
        self.env.reset()
        self._astar_gen = None
        self._rrt_gen = None
        self.canvas.canvas_astar.clear_state()
        self.canvas.canvas_rrt.clear_state()
        self.canvas.init_plots(self.start, self.goal)
        self.metrics.reset()
        self.metrics_panel.clear()

    def _stop_animation(self) -> None:
        self._running = False
        if self._after_id:
            self.root.after_cancel(self._after_id)
            self._after_id = None

    # ------------------------------------------------------------------
    # Animation loop
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if not self._running:
            return
        self._advance_frame()
        if not (self._astar_done and self._rrt_done):
            interval = self.control.interval_var.get()
            self._after_id = self.root.after(interval, self._tick)
        else:
            self._running = False

    def _advance_frame(self) -> None:
        steps = self.control.steps_per_frame_var.get()

        # Update dynamic obstacles
        self.env.update_dynamic_obstacles()
        self.canvas.refresh_grid()

        # ── A* steps ──────────────────────────────────────────────────
        if self._astar_gen and not self._astar_done:
            for _ in range(steps):
                try:
                    state = next(self._astar_gen)
                    self.metrics.track_step("A*")
                    self.canvas.canvas_astar.update_state(state)
                    if state.done:
                        self._astar_done = True
                        self.metrics.stop_timer("A*", state.path)
                        break
                except StopIteration:
                    self._astar_done = True
                    self.metrics.stop_timer("A*")
                    break

        # ── RRT steps ─────────────────────────────────────────────────
        if self._rrt_gen and not self._rrt_done:
            for _ in range(steps):
                try:
                    state = next(self._rrt_gen)
                    self.metrics.track_step("RRT")
                    self.canvas.canvas_rrt.update_state(state)
                    if state.done:
                        self._rrt_done = True
                        self.metrics.stop_timer("RRT", state.path)
                        break
                except StopIteration:
                    self._rrt_done = True
                    self.metrics.stop_timer("RRT")
                    break

        # ── Refresh ───────────────────────────────────────────────────
        self.canvas.draw()
        self.metrics_panel.update_metrics(self.metrics.comparison_dict())

    # ==================================================================
    # Run
    # ==================================================================

    def run(self) -> None:
        self.root.mainloop()
