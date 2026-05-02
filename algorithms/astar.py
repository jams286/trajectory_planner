"""A* path-planning algorithm with configurable heuristics."""

from __future__ import annotations

import heapq
import math
from typing import Dict, Generator, List, Set, Tuple

from algorithms.base_planner import BasePlanner, PlannerState
from environment.grid_env import GridEnvironment

Node = Tuple[int, int]


# ======================================================================
# Heuristic functions
# ======================================================================

def _manhattan(a: Node, b: Node) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _euclidean(a: Node, b: Node) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _diagonal(a: Node, b: Node) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)


HEURISTIC_MAP = {
    "Manhattan": _manhattan,
    "Euclidean": _euclidean,
    "Diagonal": _diagonal,
}


class AStarPlanner(BasePlanner):
    """A* search on an 8-connected grid."""

    def __init__(self, env: GridEnvironment, heuristic: str = "Euclidean") -> None:
        super().__init__(env)
        if heuristic not in HEURISTIC_MAP:
            raise ValueError(f"Unknown heuristic '{heuristic}'. Choose from {list(HEURISTIC_MAP)}")
        self.heuristic = HEURISTIC_MAP[heuristic]
        self.heuristic_name = heuristic

    # ------------------------------------------------------------------

    def step_generator(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> Generator[PlannerState, None, None]:
        start_node: Node = (int(start[0]), int(start[1]))
        goal_node: Node = (int(goal[0]), int(goal[1]))

        g_score: Dict[Node, float] = {start_node: 0.0}
        came_from: Dict[Node, Node] = {}

        counter = 0  # tie-breaker for heap stability
        open_heap: List[Tuple[float, int, Node]] = []
        heapq.heappush(open_heap, (self.heuristic(start_node, goal_node), counter, start_node))

        open_set: Set[Node] = {start_node}
        closed_set: Set[Node] = set()

        while open_heap:
            _f, _cnt, current = heapq.heappop(open_heap)
            if current in closed_set:
                continue

            open_set.discard(current)
            closed_set.add(current)

            # ── Yield snapshot ────────────────────────────────────────
            yield PlannerState(
                open_set=list(open_set),
                closed_set=list(closed_set),
                current=current,
            )

            # ── Goal reached ──────────────────────────────────────────
            if current == goal_node:
                path = _reconstruct(came_from, current)
                yield PlannerState(
                    open_set=list(open_set),
                    closed_set=list(closed_set),
                    current=current,
                    path=path,
                    done=True,
                )
                return

            # ── Expand neighbours ─────────────────────────────────────
            for nr, nc, step_cost in self.env.get_neighbors(current[0], current[1]):
                neighbor: Node = (nr, nc)
                if neighbor in closed_set:
                    continue
                tentative_g = g_score[current] + step_cost
                if tentative_g < g_score.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal_node)
                    counter += 1
                    heapq.heappush(open_heap, (f, counter, neighbor))
                    open_set.add(neighbor)

        # No path found
        yield PlannerState(closed_set=list(closed_set), done=True)


def _reconstruct(came_from: Dict[Node, Node], current: Node) -> List[Tuple[float, float]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return [(float(r), float(c)) for r, c in path]
