"""Rapidly-exploring Random Tree (RRT) path-planning algorithm."""

from __future__ import annotations

import math
import random
from typing import Dict, Generator, List, Optional, Tuple

from algorithms.base_planner import BasePlanner, PlannerState
from environment.grid_env import GridEnvironment

Point = Tuple[float, float]


class RRTPlanner(BasePlanner):
    """Basic RRT in continuous 2-D space."""

    def __init__(
        self,
        env: GridEnvironment,
        step_size: float = 1.5,
        max_iter: int = 5000,
        goal_bias: float = 0.1,
        goal_threshold: float = 1.5,
    ) -> None:
        super().__init__(env)
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_bias = goal_bias
        self.goal_threshold = goal_threshold

    # ------------------------------------------------------------------

    def step_generator(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> Generator[PlannerState, None, None]:
        nodes: List[Point] = [start]
        parent: Dict[int, int] = {}  # child_idx → parent_idx
        edges: List[Tuple[Point, Point]] = []

        for iteration in range(self.max_iter):
            # ── Sample random point (with goal bias) ──────────────────
            if random.random() < self.goal_bias:
                sample = goal
            else:
                sample = (
                    random.uniform(0, self.env.rows),
                    random.uniform(0, self.env.cols),
                )

            # ── Find nearest node ─────────────────────────────────────
            nearest_idx = _nearest(nodes, sample)
            nearest = nodes[nearest_idx]

            # ── Steer toward sample ───────────────────────────────────
            new_point = _steer(nearest, sample, self.step_size)

            # ── Collision check ────────────────────────────────────────
            # Swap (row, col) → (x, y) for segment check (col=x, row=y)
            p1_xy = (nearest[1], nearest[0])
            p2_xy = (new_point[1], new_point[0])
            if not self.env.is_segment_free(p1_xy, p2_xy):
                yield PlannerState(tree_edges=list(edges))
                continue
            if not self.env.is_point_free(new_point[1], new_point[0]):
                yield PlannerState(tree_edges=list(edges))
                continue

            # ── Add node ──────────────────────────────────────────────
            new_idx = len(nodes)
            nodes.append(new_point)
            parent[new_idx] = nearest_idx
            edges.append((nearest, new_point))

            # ── Yield snapshot ────────────────────────────────────────
            yield PlannerState(
                tree_edges=list(edges),
                current=new_point,
            )

            # ── Check goal ────────────────────────────────────────────
            if _dist(new_point, goal) <= self.goal_threshold:
                path = _trace_path(nodes, parent, new_idx)
                yield PlannerState(
                    tree_edges=list(edges),
                    current=new_point,
                    path=path,
                    done=True,
                )
                return

        # Max iterations reached — no path found
        yield PlannerState(tree_edges=list(edges), done=True)


# ======================================================================
# Utility functions
# ======================================================================

def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _nearest(nodes: List[Point], target: Point) -> int:
    best_idx = 0
    best_d = _dist(nodes[0], target)
    for i in range(1, len(nodes)):
        d = _dist(nodes[i], target)
        if d < best_d:
            best_d = d
            best_idx = i
    return best_idx


def _steer(from_pt: Point, to_pt: Point, step: float) -> Point:
    d = _dist(from_pt, to_pt)
    if d <= step:
        return to_pt
    ratio = step / d
    return (
        from_pt[0] + ratio * (to_pt[0] - from_pt[0]),
        from_pt[1] + ratio * (to_pt[1] - from_pt[1]),
    )


def _trace_path(
    nodes: List[Point],
    parent: Dict[int, int],
    goal_idx: int,
) -> List[Point]:
    path: List[Point] = []
    idx: Optional[int] = goal_idx
    while idx is not None:
        path.append(nodes[idx])
        idx = parent.get(idx)
    path.reverse()
    return path
