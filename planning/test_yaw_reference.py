"""Bounded-rate yaw checks for the three-circle planning reference."""
import unittest
import numpy as np

from planning.yaw_reference import make_rate_limited_yaw_reference
from planning.direct_collocation import plan_trajectory
from planning.mock_environment import default_start_goal, make_route_choice_environment
from planning.test_planner import validate_result


class TestYawReference(unittest.TestCase):
    def test_turn_is_rate_limited_and_starts_at_measured_heading(self):
        path = np.array([[0., 0.], [1., 0.], [1., 1.]])
        fraction = np.linspace(0, 1, 101)
        yaw, angles = make_rate_limited_yaw_reference(
            path, fraction, horizon=2.0, max_yaw_rate=0.3, initial_yaw=0.0
        )
        self.assertAlmostEqual(yaw(0.), 0.)
        self.assertLessEqual(np.max(np.abs(np.diff(angles))) / .02, .3 + 1e-10)
        self.assertTrue(np.all(np.diff(angles) >= -1e-10))
        self.assertGreater(yaw(1.), 0.25)  # Turns only during the later part of the path.
        self.assertLess(yaw(1.), np.pi/2)  # Cannot reach the full requested turn yet.

    def test_wraparound_follows_short_direction(self):
        # World-frame path tangent near -pi; start yaw near +pi.
        path = np.array([[0., 0.], [-1., -0.01]])
        yaw, angles = make_rate_limited_yaw_reference(
            path, np.linspace(0, 1, 11), 2., .6, initial_yaw=3.13
        )
        self.assertTrue(np.all(np.abs(np.diff(angles)) < .13))
        self.assertGreater(yaw(1.), 3.)

    def test_rejects_invalid_parameters(self):
        path = np.array([[0.,0.],[1.,0.]])
        with self.assertRaises(ValueError):
            make_rate_limited_yaw_reference(path, [0.,0.,1.], 1., .6)
        with self.assertRaises(ValueError):
            make_rate_limited_yaw_reference(path, [0.,1.], 1., 0.)

    def test_collocation_respects_bounded_yaw(self):
        env = make_route_choice_environment()
        start, goal = default_start_goal()
        cfg = {
            "N": 30, "T": 12.0, "max_speed": 1.0,
            "smoothness_weight": 1., "robot_length": .7,
            "robot_width": .31, "r_safety": .05,
            "astar_resolution": .1, "astar_clearance": .01,
            "footprint_model": "three_circle", "footprint_yaw_mode": "path_tangent",
            "footprint_yaw": 0., "footprint_max_yaw_rate": .6,
        }
        result = plan_trajectory(start, goal, env, cfg)
        self.assertTrue(result["success"], result["return_status"])
        yaw=np.unwrap(result['yaw_ref'])
        rate=np.diff(yaw)/np.diff(result['t'])
        self.assertLessEqual(float(np.max(np.abs(rate))), .6+1e-8)
        self.assertAlmostEqual(yaw[0],0.)
        validate_result(result, start, goal, env, cfg)


if __name__ == '__main__':
    unittest.main()
