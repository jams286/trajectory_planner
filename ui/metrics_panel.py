"""Metrics display panel with comparison table and bar charts."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Dict

import matplotlib
matplotlib.use("TkAgg")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import config


class MetricsPanel(ttk.Frame):
    """Bottom panel showing comparison table + bar charts."""

    def __init__(self, parent: tk.Widget) -> None:
        super().__init__(parent, padding=6)
        self._build_ui()

    def _build_ui(self) -> None:
        # ── Table ─────────────────────────────────────────────────────
        table_frame = ttk.LabelFrame(self, text="Comparison", padding=4)
        table_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=(0, 6))

        columns = ("metric", "astar", "rrt")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=4)
        self.tree.heading("metric", text="Metric")
        self.tree.heading("astar", text="A*")
        self.tree.heading("rrt", text="RRT")
        self.tree.column("metric", width=120)
        self.tree.column("astar", width=100, anchor="center")
        self.tree.column("rrt", width=100, anchor="center")
        self.tree.pack(fill=tk.BOTH, expand=True)

        # ── Bar charts ────────────────────────────────────────────────
        chart_frame = ttk.LabelFrame(self, text="Charts", padding=4)
        chart_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.chart_fig = Figure(figsize=(7, 1.8), dpi=90, facecolor="#F8F9FA")
        self.chart_fig.subplots_adjust(left=0.06, right=0.98, top=0.85, bottom=0.22, wspace=0.35)
        self.ax_length = self.chart_fig.add_subplot(1, 3, 1)
        self.ax_time = self.chart_fig.add_subplot(1, 3, 2)
        self.ax_nodes = self.chart_fig.add_subplot(1, 3, 3)

        self.chart_canvas = FigureCanvasTkAgg(self.chart_fig, master=chart_frame)
        self.chart_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update_metrics(self, data: Dict[str, Dict[str, str]]) -> None:
        """Refresh table and charts from MetricsCollector.comparison_dict()."""
        # Determine the RRT variant name from the data keys
        rrt_key = "RRT"
        for k in data:
            if k.startswith("RRT"):
                rrt_key = k
                break

        # ── Update table heading ──────────────────────────────────────
        self.tree.heading("rrt", text=rrt_key)

        # ── Update table ──────────────────────────────────────────────
        for item in self.tree.get_children():
            self.tree.delete(item)

        astar_data = data.get("A*", {})
        rrt_data = data.get(rrt_key, {})
        metrics = ["Path Length", "Compute Time", "Nodes Explored", "Path Found"]
        for m in metrics:
            self.tree.insert("", tk.END, values=(m, astar_data.get(m, "–"), rrt_data.get(m, "–")))

        # ── Update bar charts ─────────────────────────────────────────
        names = ["A*", rrt_key]
        colors = [config.COLOR_OPEN_SET, config.COLOR_RRT_TREE]

        def _safe_float(d: Dict[str, str], key: str) -> float:
            v = d.get(key, "0")
            try:
                return float(v.replace(" s", ""))
            except (ValueError, AttributeError):
                return 0.0

        length_vals = [_safe_float(astar_data, "Path Length"), _safe_float(rrt_data, "Path Length")]
        time_vals = [_safe_float(astar_data, "Compute Time"), _safe_float(rrt_data, "Compute Time")]
        nodes_vals = [_safe_float(astar_data, "Nodes Explored"), _safe_float(rrt_data, "Nodes Explored")]

        for ax, vals, title in [
            (self.ax_length, length_vals, "Path Length"),
            (self.ax_time, time_vals, "Time (s)"),
            (self.ax_nodes, nodes_vals, "Nodes"),
        ]:
            ax.clear()
            ax.bar(names, vals, color=colors, edgecolor="#2C3E50", linewidth=0.5)
            ax.set_title(title, fontsize=9, fontweight="bold")
            ax.tick_params(labelsize=8)

        self.chart_canvas.draw_idle()

    def clear(self) -> None:
        for item in self.tree.get_children():
            self.tree.delete(item)
        for ax in (self.ax_length, self.ax_time, self.ax_nodes):
            ax.clear()
        self.chart_canvas.draw_idle()
