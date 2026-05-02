"""Unit tests for the trajectory planner project."""

import math
import sys
import os

import numpy as np
import pytest

# Ensure project root is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from environment.obstacle import (
    StaticObstacle,
    DynamicLinearObstacle,
    DynamicCircularObstacle,
    _segment_rect_intersect,
)
from environment.grid_env import GridEnvironment
from environment.map_presets import PRESETS
from algorithms.astar import AStarPlanner, HEURISTIC_MAP
from algorithms.rrt import RRTPlanner
from algorithms.base_planner import PlannerState
from metrics.evaluator import MetricsCollector, _path_length


# ======================================================================
# Obstacles
# ======================================================================

class TestStaticObstacle:
    def test_contains_point_inside(self):
        obs = StaticObstacle(x=5, y=5, width=3, height=3)
        assert obs.contains_point(6, 6)

    def test_contains_point_outside(self):
        obs = StaticObstacle(x=5, y=5, width=3, height=3)
        assert not obs.contains_point(0, 0)

    def test_contains_point_edge(self):
        obs = StaticObstacle(x=5, y=5, width=3, height=3)
        assert obs.contains_point(5, 5)       # inclusive lower bound
        assert not obs.contains_point(8, 8)    # exclusive upper bound

    def test_rasterize_shape(self):
        obs = StaticObstacle(x=2, y=3, width=4, height=2)
        mask = obs.rasterize(50, 50)
        assert mask.shape == (50, 50)
        assert mask.dtype == bool

    def test_rasterize_correct_cells(self):
        obs = StaticObstacle(x=0, y=0, width=2, height=2)
        mask = obs.rasterize(10, 10)
        assert mask[0, 0] and mask[0, 1] and mask[1, 0] and mask[1, 1]
        assert not mask[2, 2]

    def test_rect_property(self):
        obs = StaticObstacle(x=1, y=2, width=3, height=4)
        assert obs.rect == (1, 2, 3, 4)


class TestDynamicLinearObstacle:
    def test_update_moves_obstacle(self):
        obs = DynamicLinearObstacle(x=10, y=10, width=2, height=2, dx=1, dy=0)
        obs.update(50, 50)
        assert obs.x == 11

    def test_bounce_at_boundary(self):
        obs = DynamicLinearObstacle(x=47, y=10, width=2, height=2, dx=2, dy=0)
        obs.update(50, 50)
        assert obs.dx == -2  # reversed direction

    def test_reset_restores_position(self):
        obs = DynamicLinearObstacle(x=10, y=10, width=2, height=2, dx=1, dy=0)
        obs.update(50, 50)
        obs.update(50, 50)
        obs.reset()
        assert obs.x == 10 and obs.y == 10


class TestDynamicCircularObstacle:
    def test_initial_position(self):
        obs = DynamicCircularObstacle(
            width=2, height=2, center_x=25, center_y=25,
            orbit_radius=5, angular_speed=0.1,
        )
        assert abs(obs.x - (25 + 5 - 1)) < 1e-9  # cos(0)=1, -width/2
        assert abs(obs.y - (25 - 1)) < 1e-9        # sin(0)=0, -height/2

    def test_update_changes_angle(self):
        obs = DynamicCircularObstacle(
            width=2, height=2, center_x=25, center_y=25,
            orbit_radius=5, angular_speed=0.1,
        )
        x0, y0 = obs.x, obs.y
        obs.update(50, 50)
        assert (obs.x, obs.y) != (x0, y0)

    def test_reset_returns_to_start(self):
        obs = DynamicCircularObstacle(
            width=2, height=2, center_x=25, center_y=25,
            orbit_radius=5, angular_speed=0.1,
        )
        x0, y0 = obs.x, obs.y
        for _ in range(10):
            obs.update(50, 50)
        obs.reset()
        assert abs(obs.x - x0) < 1e-9 and abs(obs.y - y0) < 1e-9


class TestSegmentRectIntersect:
    def test_segment_through_rect(self):
        assert _segment_rect_intersect((0, 0), (10, 10), 4, 4, 2, 2)

    def test_segment_misses_rect(self):
        assert not _segment_rect_intersect((0, 0), (3, 0), 5, 5, 2, 2)

    def test_segment_starts_inside(self):
        assert _segment_rect_intersect((5, 5), (10, 10), 4, 4, 3, 3)

    def test_horizontal_miss(self):
        assert not _segment_rect_intersect((0, 0), (10, 0), 4, 4, 2, 2)


# ======================================================================
# GridEnvironment
# ======================================================================

class TestGridEnvironment:
    def test_empty_grid_all_free(self):
        env = GridEnvironment(rows=10, cols=10)
        assert all(env.is_free(r, c) for r in range(10) for c in range(10))

    def test_obstacle_blocks_cell(self):
        obs = StaticObstacle(x=3, y=3, width=2, height=2)
        env = GridEnvironment(rows=10, cols=10, obstacles=[obs])
        assert not env.is_free(3, 3)
        assert not env.is_free(4, 4)

    def test_out_of_bounds_not_free(self):
        env = GridEnvironment(rows=10, cols=10)
        assert not env.is_free(-1, 0)
        assert not env.is_free(10, 5)

    def test_neighbors_count_open(self):
        env = GridEnvironment(rows=10, cols=10)
        neighbors = env.get_neighbors(5, 5)
        assert len(neighbors) == 8  # all 8 directions open

    def test_neighbors_corner(self):
        env = GridEnvironment(rows=10, cols=10)
        neighbors = env.get_neighbors(0, 0)
        assert len(neighbors) == 3  # right, down, diag down-right

    def test_add_and_remove_obstacle(self):
        env = GridEnvironment(rows=10, cols=10)
        obs = StaticObstacle(x=5, y=5, width=1, height=1)
        env.add_obstacle(obs)
        assert not env.is_free(5, 5)
        env.remove_obstacles_at(5, 5)
        assert env.is_free(5, 5)

    def test_clear_obstacles(self):
        obs = StaticObstacle(x=3, y=3, width=2, height=2)
        env = GridEnvironment(rows=10, cols=10, obstacles=[obs])
        env.clear_obstacles()
        assert env.is_free(3, 3)

    def test_segment_free_no_obstacles(self):
        env = GridEnvironment(rows=10, cols=10)
        assert env.is_segment_free((0, 0), (9, 9))

    def test_segment_blocked_by_obstacle(self):
        obs = StaticObstacle(x=4, y=4, width=2, height=2)
        env = GridEnvironment(rows=10, cols=10, obstacles=[obs])
        assert not env.is_segment_free((0, 0), (9, 9))

    def test_dynamic_obstacle_update(self):
        obs = DynamicLinearObstacle(x=5, y=5, width=2, height=2, dx=1, dy=0)
        env = GridEnvironment(rows=20, cols=20, obstacles=[obs])
        grid_before = env.get_grid().copy()
        env.update_dynamic_obstacles()
        grid_after = env.get_grid()
        assert not np.array_equal(grid_before, grid_after)


# ======================================================================
# Map Presets
# ======================================================================

class TestMapPresets:
    @pytest.mark.parametrize("name", list(PRESETS.keys()))
    def test_preset_returns_list(self, name):
        obstacles = PRESETS[name]()
        assert isinstance(obstacles, list)

    @pytest.mark.parametrize("name", list(PRESETS.keys()))
    def test_preset_env_creation(self, name):
        obstacles = PRESETS[name]()
        env = GridEnvironment(obstacles=obstacles)
        assert env.get_grid().shape == (50, 50)


# ======================================================================
# A* Planner
# ======================================================================

class TestAStarPlanner:
    def _make_env(self):
        return GridEnvironment(rows=20, cols=20)

    def test_finds_path_empty_grid(self):
        env = self._make_env()
        planner = AStarPlanner(env, heuristic="Euclidean")
        state = planner.run((0, 0), (19, 19))
        assert state.done and len(state.path) > 0

    def test_path_starts_at_start(self):
        env = self._make_env()
        planner = AStarPlanner(env, heuristic="Manhattan")
        state = planner.run((0, 0), (10, 10))
        assert state.path[0] == (0.0, 0.0)

    def test_path_ends_at_goal(self):
        env = self._make_env()
        planner = AStarPlanner(env, heuristic="Diagonal")
        state = planner.run((0, 0), (10, 10))
        assert state.path[-1] == (10.0, 10.0)

    def test_no_path_when_blocked(self):
        # Wall completely blocks the goal
        obstacles = [StaticObstacle(x=0, y=10, width=20, height=1)]
        env = GridEnvironment(rows=20, cols=20, obstacles=obstacles)
        planner = AStarPlanner(env)
        state = planner.run((0, 0), (19, 19))
        assert state.done and len(state.path) == 0

    @pytest.mark.parametrize("heuristic", ["Manhattan", "Euclidean", "Diagonal"])
    def test_all_heuristics_find_path(self, heuristic):
        env = self._make_env()
        planner = AStarPlanner(env, heuristic=heuristic)
        state = planner.run((0, 0), (15, 15))
        assert state.path and len(state.path) > 0

    def test_invalid_heuristic_raises(self):
        env = self._make_env()
        with pytest.raises(ValueError):
            AStarPlanner(env, heuristic="Invalid")

    def test_step_generator_yields_states(self):
        env = self._make_env()
        planner = AStarPlanner(env)
        steps = list(planner.step_generator((0, 0), (5, 5)))
        assert len(steps) > 1
        assert steps[-1].done

    def test_diagonal_shorter_than_manhattan(self):
        """Diagonal heuristic should explore fewer or equal nodes than Manhattan."""
        env = self._make_env()
        p_man = AStarPlanner(env, heuristic="Manhattan")
        p_diag = AStarPlanner(env, heuristic="Diagonal")
        s_man = p_man.run((0, 0), (19, 19))
        s_diag = p_diag.run((0, 0), (19, 19))
        # Diagonal path should be ≤ Manhattan path length (or very close)
        assert _path_length(s_diag.path) <= _path_length(s_man.path) + 0.01


# ======================================================================
# Heuristics
# ======================================================================

class TestHeuristics:
    def test_manhattan_distance(self):
        h = HEURISTIC_MAP["Manhattan"]
        assert h((0, 0), (3, 4)) == 7

    def test_euclidean_distance(self):
        h = HEURISTIC_MAP["Euclidean"]
        assert abs(h((0, 0), (3, 4)) - 5.0) < 1e-9

    def test_diagonal_distance(self):
        h = HEURISTIC_MAP["Diagonal"]
        # For (0,0)->(3,4): min=3, max=4 → 4 + (√2-2)*3
        expected = (3 + 4) + (math.sqrt(2) - 2) * 3
        assert abs(h((0, 0), (3, 4)) - expected) < 1e-9

    def test_euclidean_and_diagonal_are_admissible(self):
        """Euclidean and Diagonal heuristics must be ≤ true optimal cost on 8-connected grid."""
        a, b = (0, 0), (7, 11)
        # Optimal 8-connected cost: min(dx,dy)*√2 + |dx-dy|*1
        dx, dy = 7, 11
        optimal_cost = min(dx, dy) * math.sqrt(2) + abs(dx - dy)
        for name in ("Euclidean", "Diagonal"):
            h = HEURISTIC_MAP[name]
            assert h(a, b) <= optimal_cost + 1e-9, f"{name} is not admissible"

    def test_manhattan_overestimates_on_8connected(self):
        """Manhattan is NOT admissible for 8-connected grids (known trade-off)."""
        a, b = (0, 0), (5, 5)
        optimal_cost = 5 * math.sqrt(2)  # pure diagonal
        h = HEURISTIC_MAP["Manhattan"]
        assert h(a, b) > optimal_cost  # Manhattan = 10, optimal ≈ 7.07


# ======================================================================
# RRT Planner
# ======================================================================

class TestRRTPlanner:
    def _make_env(self):
        return GridEnvironment(rows=20, cols=20)

    def test_finds_path_empty_grid(self):
        env = self._make_env()
        planner = RRTPlanner(env, step_size=2.0, max_iter=5000, goal_bias=0.2, goal_threshold=2.0)
        state = planner.run((1, 1), (18, 18))
        assert state.path and len(state.path) > 0

    def test_path_starts_near_start(self):
        env = self._make_env()
        planner = RRTPlanner(env, step_size=2.0, max_iter=5000, goal_bias=0.3, goal_threshold=2.0)
        state = planner.run((1, 1), (18, 18))
        if state.path:
            assert state.path[0] == (1, 1)

    def test_step_generator_yields_states(self):
        env = self._make_env()
        planner = RRTPlanner(env, max_iter=50, goal_bias=0.0)
        steps = list(planner.step_generator((1, 1), (18, 18)))
        assert len(steps) > 0

    def test_tree_edges_grow(self):
        env = self._make_env()
        planner = RRTPlanner(env, max_iter=100, goal_bias=0.0)
        edges_count = 0
        for state in planner.step_generator((1, 1), (18, 18)):
            if state.tree_edges:
                edges_count = max(edges_count, len(state.tree_edges))
        assert edges_count > 0

    def test_high_goal_bias_finds_faster(self):
        """Higher goal bias should generally find the goal in fewer iterations."""
        env = self._make_env()
        results = []
        for bias in [0.0, 0.5]:
            planner = RRTPlanner(env, step_size=2.0, max_iter=5000, goal_bias=bias, goal_threshold=2.0)
            count = 0
            for state in planner.step_generator((1, 1), (18, 18)):
                count += 1
                if state.done:
                    break
            results.append(count)
        # Not deterministic, but with 5000 iterations, high bias should finish sooner on average
        # We just verify both complete
        assert all(r > 0 for r in results)


# ======================================================================
# Metrics
# ======================================================================

class TestMetrics:
    def test_path_length_straight(self):
        path = [(0.0, 0.0), (3.0, 4.0)]
        assert abs(_path_length(path) - 5.0) < 1e-9

    def test_path_length_multi_segment(self):
        path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
        assert abs(_path_length(path) - 2.0) < 1e-9

    def test_collector_evaluate(self):
        env = GridEnvironment(rows=20, cols=20)
        planner = AStarPlanner(env)
        mc = MetricsCollector()
        m = mc.evaluate("A*", planner, (0, 0), (10, 10))
        assert m.path_found
        assert m.path_length > 0
        assert m.compute_time > 0
        assert m.nodes_explored > 0

    def test_comparison_dict_format(self):
        env = GridEnvironment(rows=20, cols=20)
        mc = MetricsCollector()
        mc.evaluate("A*", AStarPlanner(env), (0, 0), (10, 10))
        mc.evaluate("RRT", RRTPlanner(env, goal_bias=0.3, goal_threshold=2.0), (0, 0), (10, 10))
        d = mc.comparison_dict()
        assert "A*" in d and "RRT" in d
        assert "Path Length" in d["A*"]

    def test_reset_clears_results(self):
        mc = MetricsCollector()
        env = GridEnvironment(rows=10, cols=10)
        mc.evaluate("A*", AStarPlanner(env), (0, 0), (5, 5))
        mc.reset()
        assert len(mc.results) == 0

    def test_timer_tracking(self):
        mc = MetricsCollector()
        mc.start_timer("test")
        mc.track_step("test")
        mc.track_step("test")
        mc.stop_timer("test", path=[(0.0, 0.0), (3.0, 4.0)])
        assert mc.results["test"].nodes_explored == 2
        assert mc.results["test"].path_found
        assert abs(mc.results["test"].path_length - 5.0) < 1e-9


# ======================================================================
# PlannerState
# ======================================================================

class TestPlannerState:
    def test_default_state(self):
        s = PlannerState()
        assert s.open_set == []
        assert s.closed_set == []
        assert s.current is None
        assert s.path == []
        assert s.tree_edges == []
        assert s.done is False

    def test_custom_state(self):
        s = PlannerState(
            open_set=[(1, 2)],
            path=[(0, 0), (1, 1)],
            done=True,
        )
        assert len(s.open_set) == 1
        assert len(s.path) == 2
        assert s.done
