"""Grid-based 2-D environment with obstacle rasterization and collision checking."""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

import config
from environment.base_env import BaseEnvironment, Obstacle
from environment.obstacle import (
    DynamicCircularObstacle,
    DynamicLinearObstacle,
    StaticObstacle,
)

_SQRT2 = math.sqrt(2)

# 8-connected neighbour offsets: (Δrow, Δcol, cost)
_NEIGHBORS_8 = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2),
]


class GridEnvironment(BaseEnvironment):
    """Concrete grid environment.

    Obstacles are stored in continuous coordinates and rasterized onto a
    NumPy boolean grid for A*.  Segment collision checks operate in
    continuous space for RRT.
    """

    def __init__(
        self,
        rows: int = config.GRID_ROWS,
        cols: int = config.GRID_COLS,
        obstacles: List[Obstacle] | None = None,
    ) -> None:
        self.rows = rows
        self.cols = cols
        self.obstacles: List[Obstacle] = list(obstacles) if obstacles else []
        self._grid = np.zeros((rows, cols), dtype=bool)
        self._rebuild_grid()

    # ------------------------------------------------------------------
    # Grid helpers
    # ------------------------------------------------------------------

    def _rebuild_grid(self) -> None:
        self._grid[:] = False
        for obs in self.obstacles:
            self._grid |= obs.rasterize(self.rows, self.cols)

    def get_grid(self) -> np.ndarray:
        return self._grid

    # ------------------------------------------------------------------
    # Query
    # ------------------------------------------------------------------

    def is_free(self, row: int, col: int) -> bool:
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return not self._grid[row, col]
        return False

    def is_segment_free(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        for obs in self.obstacles:
            if obs.intersects_segment(p1, p2):
                return False
        return True

    def is_point_free(self, x: float, y: float) -> bool:
        for obs in self.obstacles:
            if obs.contains_point(x, y):
                return False
        if x < 0 or x >= self.cols or y < 0 or y >= self.rows:
            return False
        return True

    def get_neighbors(self, row: int, col: int) -> List[Tuple[int, int, float]]:
        result: List[Tuple[int, int, float]] = []
        for dr, dc, cost in _NEIGHBORS_8:
            nr, nc = row + dr, col + dc
            if self.is_free(nr, nc):
                # For diagonal moves, also require both cardinal neighbours free
                # to prevent corner-cutting through obstacles.
                if dr != 0 and dc != 0:
                    if not self.is_free(row + dr, col) or not self.is_free(row, col + dc):
                        continue
                result.append((nr, nc, cost))
        return result

    # ------------------------------------------------------------------
    # Obstacle management
    # ------------------------------------------------------------------

    def add_obstacle(self, obs: Obstacle) -> None:
        self.obstacles.append(obs)
        self._rebuild_grid()

    def remove_obstacles_at(self, row: int, col: int) -> None:
        """Remove every static obstacle whose rasterization covers (row, col)."""
        to_remove = []
        for obs in self.obstacles:
            if isinstance(obs, StaticObstacle) and obs.contains_point(col, row):
                to_remove.append(obs)
        for obs in to_remove:
            self.obstacles.remove(obs)
        if to_remove:
            self._rebuild_grid()

    def clear_obstacles(self) -> None:
        self.obstacles.clear()
        self._rebuild_grid()

    # ------------------------------------------------------------------
    # Dynamic obstacles
    # ------------------------------------------------------------------

    def update_dynamic_obstacles(self) -> None:
        any_dynamic = False
        for obs in self.obstacles:
            if isinstance(obs, (DynamicLinearObstacle, DynamicCircularObstacle)):
                obs.update(self.cols, self.rows)
                any_dynamic = True
        if any_dynamic:
            self._rebuild_grid()

    def reset(self) -> None:
        for obs in self.obstacles:
            if isinstance(obs, (DynamicLinearObstacle, DynamicCircularObstacle)):
                obs.reset()
        self._rebuild_grid()
