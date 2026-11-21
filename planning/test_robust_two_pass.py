"""Two-pass planner tests. Contact schedules are illustrative, not Go2 data."""

import unittest
from unittest.mock import patch

import numpy as np

from planning.direct_collocation import plan_trajectory
from planning.mock_environment import MockEnvironment, rectangle
from planning.robust_control.tube_error_model import design_tube_error_model
from planning.robust_two_pass import plan_two_pass_friction_tube
from planning.test_planner import validate_result


class TestTwoPassFrictionTube(unittest.TestCase):
    @staticmethod
    def config():
        return {
            "N": 30, "T": 12.0, "max_speed": 1.0,
            "smoothness_weight": 1.0,
            "robot_length": 0.7, "robot_width": 0.31,
            "r_safety": 0.05, "astar_resolution": 0.1,
            "astar_clearance": 0.01,
            "footprint_model": "three_circle", "footprint_yaw_mode": "path_tangent",
            "footprint_yaw": 0.0,
        }

    @staticmethod
    def contacts(count, friction_lower=0.5):
        # Illustrative total normal force m*g shared across all four stance feet.
        # NOT a real Go2 contact forecast or a validated disturbance bound.
        return {
            "stance_mask": np.ones((count, 4), dtype=int),
            "normal_forces": np.full((count, 4), 20.0 * 9.81 / 4),
            "mu_plan": 0.5,
            "mu_lo": friction_lower,
            "mass": 20.0,
        }

    def test_calls_nominal_then_robust_with_meter_radius(self):
        config = self.config()
        model = design_tube_error_model(0.02)
        fake_result = {"success": True, "t": np.linspace(0, 12, 31)}
        with patch('planning.robust_two_pass.plan_trajectory', return_value=fake_result) as planner:
            outcome = plan_two_pass_friction_tube(
                [1, 1.5, 0, 0], [11, 1.5, 0, 0], MockEnvironment(),
                config, lambda nominal: self.contacts(600, friction_lower=0.3),
                model, 0.02,
            )
        self.assertEqual(planner.call_count, 2)
        self.assertEqual(planner.call_args_list[0].args[3]["tube_position_margin_m"], 0.0)
        self.assertAlmostEqual(planner.call_args_list[1].args[3]["tube_position_margin_m"],
                               max(outcome["tube_position_radii_m"]))
        self.assertGreater(outcome["tube_position_margin_m"], 0.2)
        self.assertEqual(len(outcome["tube_position_radii_m"]), 601)
        self.assertNotIn("tube_position_margin_m", config)  # no in-place mutation

    def test_empty_friction_gap_and_zero_residual_give_zero_tube(self):
        config = self.config()
        model = design_tube_error_model(0.02)
        fake_result = {"success": True, "t": np.linspace(0, 12, 31)}
        with patch('planning.robust_two_pass.plan_trajectory', return_value=fake_result):
            outcome = plan_two_pass_friction_tube(
                [1,1.5,0,0], [11,1.5,0,0], MockEnvironment(), config,
                lambda nominal: self.contacts(600, friction_lower=0.5), model, 0.02,
            )
        self.assertEqual(outcome["tube_position_margin_m"], 0.0)

    def test_rejects_timestep_mismatch(self):
        config = self.config()
        model = design_tube_error_model(0.4)  # NOT the 0.02-s error model
        fake_result = {"success": True, "t": np.linspace(0, 12, 31)}
        with patch('planning.robust_two_pass.plan_trajectory', return_value=fake_result):
            with self.assertRaisesRegex(ValueError, "inconsistent with error_dt"):
                plan_two_pass_friction_tube(
                    [1,1.5,0,0], [11,1.5,0,0], MockEnvironment(), config,
                    lambda nominal: self.contacts(600), model, 0.02,
                )
            with self.assertRaisesRegex(ValueError, "cover entire nominal horizon"):
                plan_two_pass_friction_tube(
                    [1,1.5,0,0], [11,1.5,0,0], MockEnvironment(), config,
                    lambda nominal: self.contacts(30), design_tube_error_model(0.02), 0.02,
                )

    def test_bad_margin_rejected(self):
        base = self.config()
        start = np.array([1.0, 1.5, 0, 0])
        goal = np.array([11.0, 1.5, 0, 0])
        for val in (-0.1, np.nan, np.inf):
            with self.subTest(val=val):
                cfg = dict(base, tube_position_margin_m=val)
                with self.assertRaises(ValueError):
                    plan_trajectory(start, goal, MockEnvironment(), cfg)

    def test_end_to_end_small_position_tube_clearance(self):
        # The 0.70m corridor fits a small tube; this exercises both *actual*
        # collocation solves.  It is not a validation of the contact predictor.
        low, high = 1.15, 1.85
        env = MockEnvironment(obstacles=[
            rectangle(3, 8, 0, low),
            rectangle(3, 8, high, 3),
        ])
        config = self.config()
        start = np.array([1., 1.5, 0., 0.])
        goal = np.array([11., 1.5, 0., 0.])
        model = design_tube_error_model(0.02)
        # Initial P-norm tube radii correspond to 0.02 meters position radius;
        # with no disturbance afterwards the tube decays, so max stays 0.02.
        initial_scale = 0.02 / np.sqrt(max(np.linalg.eigvalsh(
            np.eye(4)[:2] @ np.linalg.solve(model["P"], np.eye(4)[:2].T))))
        outcome = plan_two_pass_friction_tube(
            start, goal, env, config,
            lambda nominal: self.contacts(600, friction_lower=0.5),
            model, 0.02, alpha_0=initial_scale,
        )
        self.assertAlmostEqual(outcome["tube_position_margin_m"], 0.02, places=6)
        self.assertTrue(outcome["robust"]["success"])
        validate_result(outcome["robust"], start, goal, env, outcome["robust_config"])


if __name__ == '__main__':
    unittest.main()
