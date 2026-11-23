"""Checks for per-run yaw initialization and Jacob's yaw-rate cap."""
import math
import unittest

from planning.start_yaw_config import (
    MAX_REQUESTED_YAW_RATE_RAD_S,
    config_for_initial_yaw,
)


class TestStartYawConfig(unittest.TestCase):
    def setUp(self):
        self.base = {"footprint_model": "three_circle",
                     "footprint_yaw_mode": "path_tangent",
                     "footprint_yaw": 0.0}

    def test_uses_nonzero_start_heading_and_default_half_rad_per_sec_cap(self):
        cfg = config_for_initial_yaw(self.base, initial_yaw_rad=math.radians(15))
        self.assertAlmostEqual(cfg["footprint_yaw"], math.pi / 12)
        self.assertEqual(cfg["footprint_max_yaw_rate"], 0.5)
        self.assertEqual(MAX_REQUESTED_YAW_RATE_RAD_S, 0.5)

    def test_does_not_mutate_base_or_leak_between_seeded_runs(self):
        a = config_for_initial_yaw(self.base, initial_yaw_rad=math.radians(15))
        b = config_for_initial_yaw(self.base, initial_yaw_rad=math.radians(-15))
        self.assertEqual(self.base["footprint_yaw"], 0.0)
        self.assertNotIn("footprint_max_yaw_rate", self.base)
        self.assertAlmostEqual(a["footprint_yaw"], -b["footprint_yaw"])

    def test_preserves_lower_configured_cap(self):
        self.base["footprint_max_yaw_rate"] = 0.3
        cfg = config_for_initial_yaw(self.base, initial_yaw_rad=0.2)
        self.assertEqual(cfg["footprint_max_yaw_rate"], 0.3)

    def test_rejects_cap_above_half_rad_per_sec(self):
        self.base["footprint_max_yaw_rate"] = 0.6
        with self.assertRaisesRegex(ValueError, "0.5"):
            config_for_initial_yaw(self.base, initial_yaw_rad=0)

    def test_rejects_omitted_or_invalid_yaw(self):
        for val in (float('nan'), float('inf'), None, 'not a yaw'):
            with self.subTest(yaw=val), self.assertRaises(ValueError):
                config_for_initial_yaw(self.base, initial_yaw_rad=val)
        with self.assertRaises(TypeError):
            config_for_initial_yaw(self.base)

    def test_rejects_invalid_rate(self):
        for rate in (None, -0.1, 0, float('nan'), float('inf'), 'bad'):
            with self.subTest(rate=rate):
                self.base["footprint_max_yaw_rate"] = rate
                with self.assertRaises(ValueError):
                    config_for_initial_yaw(self.base, initial_yaw_rad=0)

    def test_rejects_wrong_footprint_or_yaw_mode(self):
        with self.assertRaises(ValueError):
            config_for_initial_yaw({"footprint_model": "single_circle"}, initial_yaw_rad=0)
        self.base["footprint_yaw_mode"] = "fixed"
        with self.assertRaises(ValueError):
            config_for_initial_yaw(self.base, initial_yaw_rad=0)

    def test_does_not_assume_yaw_must_be_in_minus_pi_to_pi(self):
        cfg = config_for_initial_yaw(self.base, initial_yaw_rad=3.3)
        self.assertEqual(cfg["footprint_yaw"], 3.3)


class TestStartYawRealPlanner(unittest.TestCase):
    def test_path_tangent_plan_starts_at_supplied_yaw_and_is_rate_limited(self):
        # Real IPOPT integration test; only runs where CasADi is installed.
        try:
            import numpy as np
            import casadi  # noqa: F401
            from planning.direct_collocation import plan_trajectory
            from planning.mock_environment import make_empty_environment, default_start_goal
        except ImportError as exc:
            self.skipTest(f"planner dependency unavailable: {exc}")

        start, goal = default_start_goal()
        config = {
            "N": 30, "T": 12.0, "max_speed": 1.0,
            "smoothness_weight": 1.0, "robot_length": 0.7,
            "robot_width": 0.31, "r_safety": 0.05,
            "astar_resolution": 0.05, "astar_clearance": 0.01,
            "footprint_model": "three_circle", "footprint_yaw_mode": "path_tangent",
        }
        theta0 = math.radians(15)
        cfg = config_for_initial_yaw(config, initial_yaw_rad=theta0)
        result = plan_trajectory(start, goal, make_empty_environment(), cfg)
        self.assertTrue(result["success"], result["return_status"])
        self.assertAlmostEqual(float(result["yaw_ref"][0]), theta0, places=10)
        rate = np.diff(np.unwrap(result["yaw_ref"])) / np.diff(result["t"])
        self.assertLessEqual(float(np.max(np.abs(rate))), 0.5 + 1e-8)


if __name__ == '__main__':
    unittest.main()
