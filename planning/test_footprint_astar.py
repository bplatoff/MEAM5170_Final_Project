"""Regression tests for footprint-aware A* initialization."""

import unittest
import numpy as np

from planning.footprint_astar import astar_three_circle_path
from planning.planner_helpers import astar_initial_path, sample_path_heading
from planning.robot_footprint import three_circle_centers, three_circle_parameters
from planning.mock_environment import rectangle, MockEnvironment


class TestFootprintAstar(unittest.TestCase):
    def setUp(self):
        self.start = np.array([0.5, 1.5, 0.0, 0.0])
        self.goal = np.array([11.5, 1.5, 0.0, 0.0])
        self.bounds = (0.0, 12.0, 0.0, 3.0)
        self.length = 0.7
        self.width = 0.31
        self.safety_margin = 0.05
        self.extra = 0.01

    def narrow_corridor(self, opening):
        low = 1.5 - opening / 2
        high = 1.5 + opening / 2
        return [rectangle(3, 8, 0, low), rectangle(3, 8, high, 3)]

    def run_astar(self, obstacles, **kwargs):
        params = {"resolution": .1, "extra_clearance": self.extra, **kwargs}
        return astar_three_circle_path(
            self.start, self.goal, self.bounds, obstacles,
            self.length, self.width, self.safety_margin, **params,
        )

    def assert_corridor_path_safe(self, path, obstacles):
        from planning.planner_helpers import point_in_inflated_polygon
        _, radius = three_circle_parameters(
            self.length, self.width, self.safety_margin)
        for t in np.linspace(0.0, 1.0, 251):
            from planning.planner_helpers import sample_path
            p = sample_path(t, path)
            yaw = sample_path_heading(t, path)
            for center in three_circle_centers(p, yaw, self.length):
                self.assertGreater(center[0] - radius, self.bounds[0])
                self.assertLess(center[0] + radius, self.bounds[1])
                self.assertGreater(center[1] - radius, self.bounds[2])
                self.assertLess(center[1] + radius, self.bounds[3])
                for obstacle in obstacles:
                    self.assertFalse(point_in_inflated_polygon(center, obstacle, radius),
                                     f"Collision at fraction {t:.3f}")

    def test_old_disc_rejects_gap_three_circles_use_it(self):
        obstacles = self.narrow_corridor(.7)
        disc_radius = np.hypot(self.length/2, self.width/2) + self.safety_margin
        with self.assertRaises(RuntimeError):
            astar_initial_path(self.start, self.goal, self.bounds, obstacles,
                               disc_radius, extra_clearance=self.extra)
        path = self.run_astar(obstacles)
        np.testing.assert_allclose(path[0], self.start[:2])
        np.testing.assert_allclose(path[-1], self.goal[:2])
        self.assertTrue(np.allclose(path[:, 1], 1.5))
        self.assert_corridor_path_safe(path, obstacles)

    def test_rejects_gap_smaller_than_three_circle_width(self):
        with self.assertRaises(RuntimeError):
            self.run_astar(self.narrow_corridor(.4))

    def test_yaw_mode_matters_for_vertical_corridor(self):
        # Vertical travel: longitudinal robot axis must also point up.
        self.bounds = (0.0, 3.0, 0.0, 5.0)
        self.start = np.array([1.5, 0.5, 0, 0])
        self.goal = np.array([1.5, 4.5, 0, 0])
        obs = [rectangle(0, 1.15, 1.0, 4.0),
               rectangle(1.85, 3.0, 1.0, 4.0)]
        path = self.run_astar(obs, yaw_mode="path_tangent")
        self.assertTrue(np.allclose(path[:, 0], 1.5))
        with self.assertRaises(RuntimeError):
            self.run_astar(obs, yaw_mode="fixed", fixed_yaw=0.0)

    def test_mock_multi_obstacle_route(self):
        from planning.mock_environment import make_route_choice_environment
        env = make_route_choice_environment()
        path = self.run_astar(env.obstacles)
        self.assertGreater(len(path), 2)
        self.assertTrue(np.all(np.isfinite(path)))
        self.assert_corridor_path_safe(path, env.obstacles)

    def test_collocation_can_use_narrow_gap(self):
        """End-to-end regression: A* seed must not reject an NLP-feasible gap."""
        from planning.direct_collocation import plan_trajectory
        from planning.test_planner import validate_result
        obstacles = self.narrow_corridor(.7)
        env = MockEnvironment(obstacles=obstacles)
        cfg = {
            "N": 30, "T": 12.0, "max_speed": 1.0,
            "smoothness_weight": 1.0, "robot_length": self.length,
            "robot_width": self.width, "r_safety": self.safety_margin,
            "astar_resolution": .1, "astar_clearance": self.extra,
            "footprint_model": "three_circle", "footprint_yaw_mode": "path_tangent",
            "footprint_yaw": 0.0,
        }
        result = plan_trajectory(self.start, self.goal, env, cfg)
        self.assertTrue(result["success"], result["return_status"])
        self.assertIn("astar_path", result)
        np.testing.assert_allclose(result["astar_path"][0], self.start[:2])
        validate_result(result, self.start, self.goal, env, cfg)

    def test_rejects_body_outside_course_wall(self):
        self.start = np.array([.1, 1.5, 0, 0])
        with self.assertRaises(RuntimeError):
            self.run_astar([])

    def test_validation_of_parameters(self):
        with self.assertRaises(ValueError):
            self.run_astar([], resolution=-.1)
        with self.assertRaises(ValueError):
            self.run_astar([], yaw_mode="unsupported")


if __name__ == "__main__":
    unittest.main()
