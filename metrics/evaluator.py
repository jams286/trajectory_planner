"""Metrics collection and comparison for path-planning algorithms."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from algorithms.base_planner import BasePlanner, PlannerState


@dataclass
class PlannerMetrics:
    """Metrics for a single algorithm run."""

    algorithm: str = ""
    path_length: float = 0.0
    compute_time: float = 0.0
    nodes_explored: int = 0
    path_found: bool = False


class MetricsCollector:
    """Collects and compares metrics across algorithm runs."""

    def __init__(self) -> None:
        self.results: Dict[str, PlannerMetrics] = {}

    def evaluate(
        self,
        name: str,
        planner: BasePlanner,
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> PlannerMetrics:
        """Run planner to completion, measure metrics, and store results."""
        metrics = PlannerMetrics(algorithm=name)

        t0 = time.perf_counter()
        step_count = 0
        final_state = PlannerState()

        for state in planner.step_generator(start, goal):
            step_count += 1
            final_state = state
            if state.done:
                break

        metrics.compute_time = time.perf_counter() - t0
        metrics.nodes_explored = step_count

        if final_state.path:
            metrics.path_found = True
            metrics.path_length = _path_length(final_state.path)

        self.results[name] = metrics
        return metrics

    def track_step(self, name: str) -> None:
        """Increment node count for an ongoing animated run."""
        if name not in self.results:
            self.results[name] = PlannerMetrics(algorithm=name)
        self.results[name].nodes_explored += 1

    def start_timer(self, name: str) -> None:
        if name not in self.results:
            self.results[name] = PlannerMetrics(algorithm=name)
        self.results[name]._t0 = time.perf_counter()  # type: ignore[attr-defined]

    def stop_timer(self, name: str, path: List[Tuple[float, float]] | None = None) -> None:
        m = self.results.get(name)
        if m is None:
            return
        t0 = getattr(m, "_t0", None)
        if t0 is not None:
            m.compute_time = time.perf_counter() - t0
        if path:
            m.path_found = True
            m.path_length = _path_length(path)

    def reset(self, name: str | None = None) -> None:
        if name:
            self.results.pop(name, None)
        else:
            self.results.clear()

    def comparison_dict(self) -> Dict[str, Dict[str, str]]:
        """Return a dict-of-dicts suitable for display in the UI."""
        out: Dict[str, Dict[str, str]] = {}
        for name, m in self.results.items():
            out[name] = {
                "Path Length": f"{m.path_length:.2f}" if m.path_found else "N/A",
                "Compute Time": f"{m.compute_time:.4f} s",
                "Nodes Explored": str(m.nodes_explored),
                "Path Found": "Yes" if m.path_found else "No",
            }
        return out


def _path_length(path: List[Tuple[float, float]]) -> float:
    length = 0.0
    for i in range(1, len(path)):
        length += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
    return length
