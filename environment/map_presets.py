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


def micromouse_map() -> List[Obstacle]:
    """Maze inspired by micromouse competition layouts (16x16 cells on 50x50 grid).

    Uses DFS maze generation with a fixed seed for reproducibility.
    Extra walls are removed to create multiple paths, mimicking real
    IEEE micromouse competition mazes.
    """
    import numpy as np

    rng = np.random.RandomState(2026)
    n = 16  # 16×16 micromouse cells

    # h_walls[i][j] = horizontal wall on top edge of cell row i, column j
    # v_walls[i][j] = vertical wall on left edge of cell row i, column j
    h_walls = [[True] * n for _ in range(n + 1)]
    v_walls = [[True] * (n + 1) for _ in range(n)]

    # DFS perfect maze generation — guarantees every cell is reachable
    visited = [[False] * n for _ in range(n)]
    stack = [(0, 0)]
    visited[0][0] = True

    while stack:
        r, c = stack[-1]
        neighbors = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < n and 0 <= nc < n and not visited[nr][nc]:
                neighbors.append((nr, nc, dr, dc))
        if neighbors:
            nr, nc, dr, dc = neighbors[rng.randint(len(neighbors))]
            # Remove wall between current cell and neighbor
            if dr == -1:
                h_walls[r][c] = False
            elif dr == 1:
                h_walls[r + 1][c] = False
            elif dc == -1:
                v_walls[r][c] = False
            elif dc == 1:
                v_walls[r][c + 1] = False
            visited[nr][nc] = True
            stack.append((nr, nc))
        else:
            stack.pop()

    # Remove extra walls to create multiple paths (real micromouse mazes
    # are not perfect mazes — they have loops)
    for _ in range(30):
        r = rng.randint(1, n)
        c = rng.randint(0, n)
        h_walls[r][c] = False
    for _ in range(30):
        r = rng.randint(0, n)
        c = rng.randint(1, n)
        v_walls[r][c] = False

    # Always keep outer boundary walls
    for c in range(n):
        h_walls[0][c] = True
        h_walls[n][c] = True
    for r in range(n):
        v_walls[r][0] = True
        v_walls[r][n] = True

    # Render onto a grid bitmap
    cs = 3  # each micromouse cell = 3 grid units (2 open + 1 wall)
    gs = n * cs + 1  # 49

    grid = [[False] * gs for _ in range(gs)]

    # Posts at every intersection
    for i in range(n + 1):
        for j in range(n + 1):
            y, x = i * cs, j * cs
            if y < gs and x < gs:
                grid[y][x] = True

    # Horizontal wall segments (fill between posts)
    for i in range(n + 1):
        for j in range(n):
            if h_walls[i][j]:
                y = i * cs
                for k in range(1, cs):
                    x = j * cs + k
                    if y < gs and x < gs:
                        grid[y][x] = True

    # Vertical wall segments (fill between posts)
    for i in range(n):
        for j in range(n + 1):
            if v_walls[i][j]:
                x = j * cs
                for k in range(1, cs):
                    y = i * cs + k
                    if y < gs and x < gs:
                        grid[y][x] = True

    # Merge into horizontal-run obstacles for efficiency
    walls: List[Obstacle] = []
    for y in range(gs):
        x = 0
        while x < gs:
            if grid[y][x]:
                sx = x
                while x < gs and grid[y][x]:
                    x += 1
                walls.append(StaticObstacle(x=sx, y=y, width=x - sx, height=1))
            else:
                x += 1

    return walls


# Registry for easy lookup from UI
PRESETS = {
    "Empty": empty_map,
    "Maze": maze_map,
    "Micromouse": micromouse_map,
    "Random": random_map,
    "Dynamic Demo": dynamic_demo_map,
}
