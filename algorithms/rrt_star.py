"""RRT* path-planning algorithm with asymptotic optimality via rewiring."""

from __future__ import annotations

import math
import random
from typing import Dict, Generator, List, Optional, Set, Tuple

from algorithms.base_planner import BasePlanner, PlannerState
from environment.grid_env import GridEnvironment

Point = Tuple[float, float]


class RRTStarPlanner(BasePlanner):
    """RRT* — optimal rewiring variant of RRT.

    After adding a new node, RRT* searches a neighbourhood of radius
    ``r_rewire`` for cheaper connections, and rewires existing nodes
    through the newcomer if this reduces their cost-to-come.
    """

    def __init__(
        self,
        env: GridEnvironment,
        step_size: float = 1.5,
        max_iter: int = 5000,
        goal_bias: float = 0.1,
        goal_threshold: float = 1.5,
        rewire_radius: float | None = None,
    ) -> None:
        super().__init__(env)
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_bias = goal_bias
        self.goal_threshold = goal_threshold
        # If not given, compute from the asymptotic formula (2D):
        #   r = γ * (log(n)/n)^(1/d)  with γ chosen empirically
        self._rewire_radius_fixed = rewire_radius

    def _rewire_radius(self, n: int) -> float:
        if self._rewire_radius_fixed is not None:
            return self._rewire_radius_fixed
        # Asymptotic optimal radius for 2-D free space of area W*H
        area = self.env.rows * self.env.cols
        gamma = 2.0 * math.sqrt(area / math.pi)
        if n < 2:
            return self.step_size * 3
        return min(gamma * math.sqrt(math.log(n) / n), self.step_size * 3)

    # ------------------------------------------------------------------

    def step_generator(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> Generator[PlannerState, None, None]:
        nodes: List[Point] = [start]
        parent: Dict[int, int] = {}
        cost: Dict[int, float] = {0: 0.0}
        children: Dict[int, Set[int]] = {0: set()}

        best_goal_idx: Optional[int] = None
        best_goal_cost = math.inf

        for iteration in range(self.max_iter):
            # ── Sample ────────────────────────────────────────────────
            if random.random() < self.goal_bias:
                sample = goal
            else:
                sample = (
                    random.uniform(0, self.env.rows),
                    random.uniform(0, self.env.cols),
                )

            # ── Nearest ───────────────────────────────────────────────
            nearest_idx = _nearest(nodes, sample)
            nearest = nodes[nearest_idx]

            # ── Steer ─────────────────────────────────────────────────
            new_point = _steer(nearest, sample, self.step_size)

            # ── Collision check ────────────────────────────────────────
            p1_xy = (nearest[1], nearest[0])
            p2_xy = (new_point[1], new_point[0])
            if not self.env.is_segment_free(p1_xy, p2_xy):
                yield PlannerState(tree_edges=_build_edges(nodes, parent))
                continue
            if not self.env.is_point_free(new_point[1], new_point[0]):
                yield PlannerState(tree_edges=_build_edges(nodes, parent))
                continue

            # ── Find near neighbours for rewiring ─────────────────────
            new_idx = len(nodes)
            r = self._rewire_radius(new_idx)
            near_indices = _near(nodes, new_point, r)

            # Choose best parent from neighbourhood
            best_parent = nearest_idx
            best_cost = cost[nearest_idx] + _dist(nearest, new_point)

            for ni in near_indices:
                c = cost[ni] + _dist(nodes[ni], new_point)
                if c < best_cost:
                    ni_xy = (nodes[ni][1], nodes[ni][0])
                    if self.env.is_segment_free(ni_xy, p2_xy):
                        best_cost = c
                        best_parent = ni

            # ── Add node ──────────────────────────────────────────────
            nodes.append(new_point)
            parent[new_idx] = best_parent
            cost[new_idx] = best_cost
            children[new_idx] = set()
            children.setdefault(best_parent, set()).add(new_idx)

            # ── Rewire neighbours through new node ────────────────────
            for ni in near_indices:
                if ni == best_parent:
                    continue
                new_cost_via = best_cost + _dist(new_point, nodes[ni])
                if new_cost_via < cost[ni]:
                    ni_xy = (nodes[ni][1], nodes[ni][0])
                    np_xy = (new_point[1], new_point[0])
                    if self.env.is_segment_free(np_xy, ni_xy):
                        # Remove from old parent's children
                        old_parent = parent.get(ni)
                        if old_parent is not None and old_parent in children:
                            children[old_parent].discard(ni)
                        parent[ni] = new_idx
                        children[new_idx].add(ni)
                        _propagate_cost(ni, nodes, parent, children, cost)

            # ── Yield snapshot ────────────────────────────────────────
            edges = _build_edges(nodes, parent)

            # ── Check goal ────────────────────────────────────────────
            d_goal = _dist(new_point, goal)
            if d_goal <= self.goal_threshold and best_cost + d_goal < best_goal_cost:
                best_goal_idx = new_idx
                best_goal_cost = best_cost

            if best_goal_idx is not None:
                path = _trace_path(nodes, parent, best_goal_idx)
                yield PlannerState(
                    tree_edges=edges,
                    current=new_point,
                    path=path,
                )
            else:
                yield PlannerState(
                    tree_edges=edges,
                    current=new_point,
                )

        # Done — yield final state
        edges = _build_edges(nodes, parent)
        if best_goal_idx is not None:
            path = _trace_path(nodes, parent, best_goal_idx)
            yield PlannerState(tree_edges=edges, path=path, done=True)
        else:
            yield PlannerState(tree_edges=edges, done=True)


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


def _near(nodes: List[Point], target: Point, radius: float) -> List[int]:
    return [i for i, n in enumerate(nodes) if _dist(n, target) <= radius]


def _steer(from_pt: Point, to_pt: Point, step: float) -> Point:
    d = _dist(from_pt, to_pt)
    if d <= step:
        return to_pt
    ratio = step / d
    return (
        from_pt[0] + ratio * (to_pt[0] - from_pt[0]),
        from_pt[1] + ratio * (to_pt[1] - from_pt[1]),
    )


def _trace_path(nodes: List[Point], parent: Dict[int, int], idx: int) -> List[Point]:
    path: List[Point] = []
    current: Optional[int] = idx
    while current is not None:
        path.append(nodes[current])
        current = parent.get(current)
    path.reverse()
    return path


def _build_edges(
    nodes: List[Point], parent: Dict[int, int],
) -> List[Tuple[Point, Point]]:
    return [(nodes[parent[i]], nodes[i]) for i in parent]


def _propagate_cost(
    idx: int,
    nodes: List[Point],
    parent: Dict[int, int],
    children: Dict[int, Set[int]],
    cost: Dict[int, float],
) -> None:
    """Recursively update costs after a rewire."""
    p = parent[idx]
    cost[idx] = cost[p] + _dist(nodes[p], nodes[idx])
    for child in children.get(idx, set()):
        _propagate_cost(child, nodes, parent, children, cost)
