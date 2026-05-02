"""Obstacle classes: static and dynamic (linear / circular)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Tuple

import numpy as np


@dataclass
class StaticObstacle:
    """Axis-aligned rectangular obstacle defined in continuous coordinates."""

    x: float          # left edge
    y: float          # bottom edge
    width: float
    height: float

    # ------------------------------------------------------------------
    @property
    def rect(self) -> Tuple[float, float, float, float]:
        return (self.x, self.y, self.width, self.height)

    def contains_point(self, px: float, py: float) -> bool:
        return self.x <= px < self.x + self.width and self.y <= py < self.y + self.height

    def intersects_segment(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        """Return True if the line segment p1→p2 intersects this rectangle."""
        return _segment_rect_intersect(p1, p2, self.x, self.y, self.width, self.height)

    def rasterize(self, rows: int, cols: int) -> np.ndarray:
        """Return a boolean mask (rows×cols) with True for occupied cells."""
        mask = np.zeros((rows, cols), dtype=bool)
        r_min = max(0, int(math.floor(self.y)))
        r_max = min(rows, int(math.ceil(self.y + self.height)))
        c_min = max(0, int(math.floor(self.x)))
        c_max = min(cols, int(math.ceil(self.x + self.width)))
        mask[r_min:r_max, c_min:c_max] = True
        return mask


@dataclass
class DynamicLinearObstacle:
    """Rectangular obstacle that bounces linearly between two limits."""

    x: float
    y: float
    width: float
    height: float
    dx: float = 0.0          # velocity in x per tick
    dy: float = 0.0          # velocity in y per tick
    _initial_x: float = field(init=False, repr=False)
    _initial_y: float = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._initial_x = self.x
        self._initial_y = self.y

    def update(self, bounds_w: float, bounds_h: float) -> None:
        self.x += self.dx
        self.y += self.dy
        # Bounce off world edges
        if self.x < 0 or self.x + self.width > bounds_w:
            self.dx = -self.dx
            self.x = max(0, min(self.x, bounds_w - self.width))
        if self.y < 0 or self.y + self.height > bounds_h:
            self.dy = -self.dy
            self.y = max(0, min(self.y, bounds_h - self.height))

    def reset(self) -> None:
        self.x = self._initial_x
        self.y = self._initial_y

    # Delegate collision helpers to the same logic as StaticObstacle
    def contains_point(self, px: float, py: float) -> bool:
        return self.x <= px < self.x + self.width and self.y <= py < self.y + self.height

    def intersects_segment(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        return _segment_rect_intersect(p1, p2, self.x, self.y, self.width, self.height)

    def rasterize(self, rows: int, cols: int) -> np.ndarray:
        mask = np.zeros((rows, cols), dtype=bool)
        r_min = max(0, int(math.floor(self.y)))
        r_max = min(rows, int(math.ceil(self.y + self.height)))
        c_min = max(0, int(math.floor(self.x)))
        c_max = min(cols, int(math.ceil(self.x + self.width)))
        mask[r_min:r_max, c_min:c_max] = True
        return mask


@dataclass
class DynamicCircularObstacle:
    """Rectangular obstacle that orbits around a centre point."""

    width: float
    height: float
    center_x: float        # orbit centre
    center_y: float
    orbit_radius: float
    angular_speed: float   # radians per tick
    _angle: float = field(default=0.0, init=False)

    @property
    def x(self) -> float:
        return self.center_x + self.orbit_radius * math.cos(self._angle) - self.width / 2

    @property
    def y(self) -> float:
        return self.center_y + self.orbit_radius * math.sin(self._angle) - self.height / 2

    def update(self, bounds_w: float, bounds_h: float) -> None:
        self._angle += self.angular_speed

    def reset(self) -> None:
        self._angle = 0.0

    def contains_point(self, px: float, py: float) -> bool:
        x, y = self.x, self.y
        return x <= px < x + self.width and y <= py < y + self.height

    def intersects_segment(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        return _segment_rect_intersect(p1, p2, self.x, self.y, self.width, self.height)

    def rasterize(self, rows: int, cols: int) -> np.ndarray:
        mask = np.zeros((rows, cols), dtype=bool)
        x, y = self.x, self.y
        r_min = max(0, int(math.floor(y)))
        r_max = min(rows, int(math.ceil(y + self.height)))
        c_min = max(0, int(math.floor(x)))
        c_max = min(cols, int(math.ceil(x + self.width)))
        mask[r_min:r_max, c_min:c_max] = True
        return mask


# ======================================================================
# Utility: segment–rectangle intersection (Liang–Barsky)
# ======================================================================

def _segment_rect_intersect(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    rx: float,
    ry: float,
    rw: float,
    rh: float,
) -> bool:
    """Liang–Barsky clipping to test if segment p1→p2 enters the AABB."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    p = [-dx, dx, -dy, dy]
    q = [
        p1[0] - rx,
        (rx + rw) - p1[0],
        p1[1] - ry,
        (ry + rh) - p1[1],
    ]

    t_enter = 0.0
    t_exit = 1.0

    for pi, qi in zip(p, q):
        if pi == 0:
            if qi < 0:
                return False
        else:
            t = qi / pi
            if pi < 0:
                t_enter = max(t_enter, t)
            else:
                t_exit = min(t_exit, t)
        if t_enter > t_exit:
            return False

    return t_enter <= t_exit
