"""End-to-end planner smoke tests with opt-in three-circle constraints."""
import unittest
import numpy as np

from planning.direct_collocation import plan_trajectory
from planning.mock_environment import (
    make_empty_environment,
    make_single_obstacle_environment,
    make_route_choice_environment,
    default_start_goal,
)
from planning.test_planner import validate_result
from planning.planner_helpers import sample_path_heading


class TestThreeCirclePlanner(unittest.TestCase):
    def test_heading_sampling(self):
        path = np.array([[0., 0.], [1., 0.], [2., 0.]])
        for t in np.linspace(0, 1, 9):
            self.assertAlmostEqual(sample_path_heading(t, path), 0.0)
        vertical_path = np.array([[0., 0.], [0., 1.]])
        self.assertAlmostEqual(sample_path_heading(.5, vertical_path), np.pi/2)

    def test_three_circle_empty_fixed_yaw(self):
        self.run_scenario(make_empty_environment(), "fixed")

    def test_three_circle_single_path_yaw(self):
        self.run_scenario(make_single_obstacle_environment(), "path_tangent")

    def test_three_circle_multi_path_yaw(self):
        self.run_scenario(make_route_choice_environment(), "path_tangent")

    def run_scenario(self, env, yaw_mode):
        start, goal = default_start_goal()
        cfg = {
            "N": 30, "T": 12.0, "max_speed": 1.,
            "smoothness_weight": 1., "robot_length": .7,
            "robot_width": .31, "r_safety": .05,
            "astar_resolution": .1, "astar_clearance": .01,
            "footprint_model": "three_circle",
            "footprint_yaw_mode": yaw_mode,
            "footprint_yaw": 0.,
        }
        result = plan_trajectory(start, goal, env, cfg)
        validate_result(result, start, goal, env, cfg)
        self.assertTrue(result["success"], result["return_status"])
        self.assertEqual(result["yaw_ref"].shape, (31,))
        self.assertTrue(np.all(np.isfinite(result["yaw_ref"])))
        if yaw_mode == "fixed":
            np.testing.assert_allclose(result["yaw_ref"], 0.0)
        else:
            self.assertGreater(np.max(np.abs(result["yaw_ref"])), 0.1)


if __name__ == "__main__":
    unittest.main()
