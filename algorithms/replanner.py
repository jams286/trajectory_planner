"""Online replanning: monitors a path and triggers re-computation when
dynamic obstacles invalidate part of it."""

from __future__ import annotations

from typing import List, Optional, Tuple

from environment.grid_env import GridEnvironment

Point = Tuple[float, float]


class OnlineReplanner:
    """Watches a path and detects collisions with the current grid state.

    Usage in the animation loop:
        1. After ``env.update_dynamic_obstacles()`` call ``check(env, path)``
        2. If it returns an invalidation index, trigger re-planning from that
           point (or from the agent's current position) to the goal.
    """

    @staticmethod
    def path_is_valid(env: GridEnvironment, path: List[Point]) -> bool:
        """Return True if *every* segment of ``path`` is collision-free."""
        for i in range(len(path) - 1):
            p1 = (path[i][1], path[i][0])      # (col, row) → (x, y) for env
            p2 = (path[i + 1][1], path[i + 1][0])
            if not env.is_segment_free(p1, p2):
                return False
        return True

    @staticmethod
    def first_collision_index(env: GridEnvironment, path: List[Point]) -> Optional[int]:
        """Return the index of the first path segment that collides, or None."""
        for i in range(len(path) - 1):
            p1 = (path[i][1], path[i][0])
            p2 = (path[i + 1][1], path[i + 1][0])
            if not env.is_segment_free(p1, p2):
                return i
        return None

    @staticmethod
    def replan_start(path: List[Point], collision_idx: int) -> Point:
        """Return the safe point just before the collision to use as new start."""
        return path[max(0, collision_idx)]
