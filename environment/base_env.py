"""Abstract base class for 2-D environments."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Tuple

import numpy as np

from environment.obstacle import (
    DynamicCircularObstacle,
    DynamicLinearObstacle,
    StaticObstacle,
)

Obstacle = StaticObstacle | DynamicLinearObstacle | DynamicCircularObstacle


class BaseEnvironment(ABC):
    """Interface that every concrete environment must implement."""

    @abstractmethod
    def is_free(self, row: int, col: int) -> bool:
        """Return True if the grid cell (row, col) is traversable."""

    @abstractmethod
    def is_segment_free(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        """Return True if the straight-line segment p1→p2 is collision-free."""

    @abstractmethod
    def get_grid(self) -> np.ndarray:
        """Return the occupancy grid (rows×cols) — True = obstacle."""

    @abstractmethod
    def get_neighbors(self, row: int, col: int) -> List[Tuple[int, int, float]]:
        """Return walkable 8-connected neighbours as (r, c, step_cost)."""

    @abstractmethod
    def update_dynamic_obstacles(self) -> None:
        """Advance dynamic obstacles by one tick."""

    @abstractmethod
    def reset(self) -> None:
        """Restore the environment to its initial state."""
