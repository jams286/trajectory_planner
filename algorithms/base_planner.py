"""Abstract base class for path-planning algorithms."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Generator, List, Tuple

from environment.grid_env import GridEnvironment


class PlannerState:
    """Snapshot of planner progress yielded each step for visualisation."""

    __slots__ = ("open_set", "closed_set", "current", "path", "tree_edges", "done")

    def __init__(
        self,
        open_set: List[Tuple[float, float]] | None = None,
        closed_set: List[Tuple[float, float]] | None = None,
        current: Tuple[float, float] | None = None,
        path: List[Tuple[float, float]] | None = None,
        tree_edges: List[Tuple[Tuple[float, float], Tuple[float, float]]] | None = None,
        done: bool = False,
    ) -> None:
        self.open_set = open_set or []
        self.closed_set = closed_set or []
        self.current = current
        self.path = path or []
        self.tree_edges = tree_edges or []
        self.done = done


class BasePlanner(ABC):
    """Every planner must provide a step generator and a blocking run."""

    def __init__(self, env: GridEnvironment) -> None:
        self.env = env

    @abstractmethod
    def step_generator(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> Generator[PlannerState, None, None]:
        """Yield one PlannerState per algorithm iteration."""

    def run(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> PlannerState:
        """Run to completion and return the final state."""
        state = PlannerState()
        for state in self.step_generator(start, goal):
            if state.done:
                break
        return state
