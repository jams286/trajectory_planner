"""Sidebar control panel with algorithm parameters, action buttons and speed slider."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Callable, Dict, Optional

import config
from environment.map_presets import PRESETS
from ui.canvas_widget import MODE_DRAW, MODE_DRAW_CELL, MODE_ERASE, MODE_GOAL, MODE_START


class ControlPanel(ttk.Frame):
    """Left sidebar with all user controls."""

    def __init__(
        self,
        parent: tk.Widget,
        callbacks: Dict[str, Callable],
    ) -> None:
        super().__init__(parent, padding=10)
        self._callbacks = callbacks
        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        row = 0

        # ── Title ─────────────────────────────────────────────────────
        ttk.Label(self, text="Trajectory Planner", font=("Segoe UI", 14, "bold")).grid(
            row=row, column=0, columnspan=2, pady=(0, 12), sticky="w",
        )
        row += 1

        # ── Map preset ────────────────────────────────────────────────
        ttk.Label(self, text="Map Preset").grid(row=row, column=0, sticky="w")
        self.preset_var = tk.StringVar(value="Maze")
        preset_cb = ttk.Combobox(
            self, textvariable=self.preset_var,
            values=list(PRESETS.keys()), state="readonly", width=14,
        )
        preset_cb.grid(row=row, column=1, sticky="ew", padx=(4, 0))
        preset_cb.bind("<<ComboboxSelected>>", lambda _: self._on_preset())
        row += 1

        ttk.Separator(self, orient="horizontal").grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=8,
        )
        row += 1

        # ── Mouse mode ────────────────────────────────────────────────
        ttk.Label(self, text="Mouse Mode", font=("Segoe UI", 10, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w",
        )
        row += 1

        self.mode_var = tk.StringVar(value=MODE_DRAW)
        for label, mode in [("Draw Obstacle", MODE_DRAW), ("Draw Cell", MODE_DRAW_CELL),
                            ("Erase", MODE_ERASE),
                            ("Set Start", MODE_START), ("Set Goal", MODE_GOAL)]:
            rb = ttk.Radiobutton(self, text=label, variable=self.mode_var, value=mode,
                                 command=self._on_mode_changed)
            rb.grid(row=row, column=0, columnspan=2, sticky="w", padx=8)
            row += 1

        ttk.Separator(self, orient="horizontal").grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=8,
        )
        row += 1

        # ── A* settings ──────────────────────────────────────────────
        ttk.Label(self, text="A* Settings", font=("Segoe UI", 10, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w",
        )
        row += 1

        ttk.Label(self, text="Heuristic").grid(row=row, column=0, sticky="w")
        self.heuristic_var = tk.StringVar(value=config.DEFAULT_HEURISTIC)
        h_cb = ttk.Combobox(
            self, textvariable=self.heuristic_var,
            values=list(config.ASTAR_HEURISTICS), state="readonly", width=14,
        )
        h_cb.grid(row=row, column=1, sticky="ew", padx=(4, 0))
        row += 1

        ttk.Separator(self, orient="horizontal").grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=8,
        )
        row += 1

        # ── RRT settings ─────────────────────────────────────────────
        ttk.Label(self, text="RRT Settings", font=("Segoe UI", 10, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w",
        )
        row += 1

        # RRT variant selector
        ttk.Label(self, text="Variant").grid(row=row, column=0, sticky="w")
        self.rrt_variant_var = tk.StringVar(value="RRT")
        rrt_cb = ttk.Combobox(
            self, textvariable=self.rrt_variant_var,
            values=["RRT", "RRT*"], state="readonly", width=14,
        )
        rrt_cb.grid(row=row, column=1, sticky="ew", padx=(4, 0))
        row += 1

        # Step size
        ttk.Label(self, text="Step Size").grid(row=row, column=0, sticky="w")
        self.step_size_var = tk.DoubleVar(value=config.RRT_STEP_SIZE)
        self._step_lbl = ttk.Label(self, text=f"{config.RRT_STEP_SIZE:.1f}")
        self._step_lbl.grid(row=row, column=1, sticky="e")
        row += 1
        step_scale = ttk.Scale(self, from_=0.5, to=5.0, variable=self.step_size_var,
                               command=lambda v: self._step_lbl.config(text=f"{float(v):.1f}"))
        step_scale.grid(row=row, column=0, columnspan=2, sticky="ew")
        row += 1

        # Max iterations
        ttk.Label(self, text="Max Iterations").grid(row=row, column=0, sticky="w")
        self.max_iter_var = tk.IntVar(value=config.RRT_MAX_ITER)
        max_iter_spin = ttk.Spinbox(self, from_=500, to=20000, increment=500,
                                    textvariable=self.max_iter_var, width=8)
        max_iter_spin.grid(row=row, column=1, sticky="ew", padx=(4, 0))
        row += 1

        # Goal bias
        ttk.Label(self, text="Goal Bias").grid(row=row, column=0, sticky="w")
        self.goal_bias_var = tk.DoubleVar(value=config.RRT_GOAL_BIAS)
        self._bias_lbl = ttk.Label(self, text=f"{config.RRT_GOAL_BIAS:.2f}")
        self._bias_lbl.grid(row=row, column=1, sticky="e")
        row += 1
        bias_scale = ttk.Scale(self, from_=0.0, to=0.5, variable=self.goal_bias_var,
                               command=lambda v: self._bias_lbl.config(text=f"{float(v):.2f}"))
        bias_scale.grid(row=row, column=0, columnspan=2, sticky="ew")
        row += 1

        # Goal threshold
        ttk.Label(self, text="Goal Threshold").grid(row=row, column=0, sticky="w")
        self.goal_thresh_var = tk.DoubleVar(value=config.RRT_GOAL_THRESHOLD)
        self._thresh_lbl = ttk.Label(self, text=f"{config.RRT_GOAL_THRESHOLD:.1f}")
        self._thresh_lbl.grid(row=row, column=1, sticky="e")
        row += 1
        thresh_scale = ttk.Scale(self, from_=0.5, to=5.0, variable=self.goal_thresh_var,
                                 command=lambda v: self._thresh_lbl.config(text=f"{float(v):.1f}"))
        thresh_scale.grid(row=row, column=0, columnspan=2, sticky="ew")
        row += 1

        ttk.Separator(self, orient="horizontal").grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=8,
        )
        row += 1

        # ── Animation speed ──────────────────────────────────────────
        ttk.Label(self, text="Animation", font=("Segoe UI", 10, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w",
        )
        row += 1

        ttk.Label(self, text="Steps/Frame").grid(row=row, column=0, sticky="w")
        self.steps_per_frame_var = tk.IntVar(value=config.STEPS_PER_FRAME)
        self._spf_lbl = ttk.Label(self, text=str(config.STEPS_PER_FRAME))
        self._spf_lbl.grid(row=row, column=1, sticky="e")
        row += 1
        spf_scale = ttk.Scale(self, from_=1, to=50, variable=self.steps_per_frame_var,
                              command=lambda v: self._spf_lbl.config(text=str(int(float(v)))))
        spf_scale.grid(row=row, column=0, columnspan=2, sticky="ew")
        row += 1

        ttk.Label(self, text="Interval (ms)").grid(row=row, column=0, sticky="w")
        self.interval_var = tk.IntVar(value=config.ANIMATION_INTERVAL_MS)
        self._int_lbl = ttk.Label(self, text=str(config.ANIMATION_INTERVAL_MS))
        self._int_lbl.grid(row=row, column=1, sticky="e")
        row += 1
        int_scale = ttk.Scale(self, from_=5, to=200, variable=self.interval_var,
                              command=lambda v: self._int_lbl.config(text=str(int(float(v)))))
        int_scale.grid(row=row, column=0, columnspan=2, sticky="ew")
        row += 1

        ttk.Separator(self, orient="horizontal").grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=8,
        )
        row += 1

        # ── Action buttons ───────────────────────────────────────────
        btn_frame = ttk.Frame(self)
        btn_frame.grid(row=row, column=0, columnspan=2, sticky="ew")
        row += 1

        self.run_btn = ttk.Button(btn_frame, text="▶ Run Both", command=self._on_run)
        self.run_btn.pack(fill="x", pady=2)

        self.step_btn = ttk.Button(btn_frame, text="⏭ Step", command=self._on_step)
        self.step_btn.pack(fill="x", pady=2)

        self.pause_btn = ttk.Button(btn_frame, text="⏸ Pause", command=self._on_pause)
        self.pause_btn.pack(fill="x", pady=2)

        self.reset_btn = ttk.Button(btn_frame, text="↺ Reset", command=self._on_reset)
        self.reset_btn.pack(fill="x", pady=2)

        self.clear_obs_btn = ttk.Button(btn_frame, text="🗑 Clear Obstacles", command=self._on_clear_obs)
        self.clear_obs_btn.pack(fill="x", pady=2)

        self.grid_btn = ttk.Button(btn_frame, text="▦ Show Grid", command=self._on_toggle_grid)
        self.grid_btn.pack(fill="x", pady=2)

        self.export_btn = ttk.Button(btn_frame, text="📷 Export GIF", command=self._on_export_gif)
        self.export_btn.pack(fill="x", pady=2)

        # ── Replanning toggle ────────────────────────────────────────
        self.replan_var = tk.BooleanVar(value=False)
        self.replan_check = ttk.Checkbutton(
            btn_frame, text="🔄 Online Replanning",
            variable=self.replan_var,
        )
        self.replan_check.pack(fill="x", pady=2)

        # Make column 1 expand
        self.columnconfigure(1, weight=1)

    # ------------------------------------------------------------------
    # Internal callbacks
    # ------------------------------------------------------------------

    def _on_preset(self) -> None:
        cb = self._callbacks.get("preset")
        if cb:
            cb(self.preset_var.get())

    def _on_mode_changed(self) -> None:
        cb = self._callbacks.get("mode")
        if cb:
            cb(self.mode_var.get())

    def _on_run(self) -> None:
        cb = self._callbacks.get("run")
        if cb:
            cb()

    def _on_step(self) -> None:
        cb = self._callbacks.get("step")
        if cb:
            cb()

    def _on_pause(self) -> None:
        cb = self._callbacks.get("pause")
        if cb:
            cb()

    def _on_reset(self) -> None:
        cb = self._callbacks.get("reset")
        if cb:
            cb()

    def _on_clear_obs(self) -> None:
        cb = self._callbacks.get("clear_obstacles")
        if cb:
            cb()

    def _on_toggle_grid(self) -> None:
        cb = self._callbacks.get("toggle_grid")
        if cb:
            visible = cb()
            self.grid_btn.config(text="▦ Hide Grid" if visible else "▦ Show Grid")

    def _on_export_gif(self) -> None:
        cb = self._callbacks.get("export_gif")
        if cb:
            cb()
