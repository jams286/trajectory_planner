"""Predefined map configurations."""

from __future__ import annotations

from typing import List

import config
from environment.obstacle import (
    DynamicCircularObstacle,
    DynamicLinearObstacle,
    StaticObstacle,
)

Obstacle = StaticObstacle | DynamicLinearObstacle | DynamicCircularObstacle


def empty_map() -> List[Obstacle]:
    """No obstacles — useful for baseline tests."""
    return []


def maze_map() -> List[Obstacle]:
    """Hand-crafted maze with corridors and clear gaps for passage."""
    walls: List[Obstacle] = [
        # Vertical walls with gaps
        StaticObstacle(x=10, y=0, width=1, height=20),   # gap at rows 20-24
        StaticObstacle(x=10, y=25, width=1, height=15),   # continues after gap
        StaticObstacle(x=20, y=10, width=1, height=18),   # gap at rows 0-9 and 28-49
        StaticObstacle(x=30, y=0, width=1, height=15),    # gap at rows 15-19
        StaticObstacle(x=30, y=20, width=1, height=20),   # continues after gap
        StaticObstacle(x=40, y=5, width=1, height=20),    # gap at rows 0-4 and 25-49
        # Horizontal walls with gaps
        StaticObstacle(x=0, y=15, width=8, height=1),     # gap at cols 8-9
        StaticObstacle(x=22, y=30, width=7, height=1),    # partial wall
        StaticObstacle(x=35, y=20, width=10, height=1),   # gap at cols 45-49
    ]
    return walls


def random_map(
    n_obstacles: int = 25,
    seed: int = 42,
) -> List[Obstacle]:
    """Generate random rectangular obstacles."""
    import numpy as np

    rng = np.random.RandomState(seed)
    obstacles: List[Obstacle] = []
    rows, cols = config.GRID_ROWS, config.GRID_COLS

    for _ in range(n_obstacles):
        w = rng.randint(1, 5)
        h = rng.randint(1, 5)
        x = rng.randint(0, cols - w)
        y = rng.randint(0, rows - h)

        # Avoid blocking start / goal areas
        sr, sc = config.DEFAULT_START
        gr, gc = config.DEFAULT_GOAL
        if abs(x - sc) < 4 and abs(y - sr) < 4:
            continue
        if abs(x - gc) < 4 and abs(y - gr) < 4:
            continue

        obstacles.append(StaticObstacle(x=x, y=y, width=w, height=h))

    return obstacles


def dynamic_demo_map() -> List[Obstacle]:
    """Map with static walls plus a couple of dynamic obstacles."""
    obstacles: List[Obstacle] = [
        StaticObstacle(x=15, y=10, width=2, height=15),
        StaticObstacle(x=30, y=20, width=2, height=15),
        DynamicLinearObstacle(x=20, y=5, width=3, height=3, dx=0.3, dy=0),
        DynamicLinearObstacle(x=35, y=40, width=3, height=3, dx=0, dy=-0.3),
        DynamicCircularObstacle(
            width=2, height=2,
            center_x=25, center_y=25,
            orbit_radius=8,
            angular_speed=0.05,
        ),
    ]
    return obstacles


# Registry for easy lookup from UI
PRESETS = {
    "Empty": empty_map,
    "Maze": maze_map,
    "Random": random_map,
    "Dynamic Demo": dynamic_demo_map,
}
