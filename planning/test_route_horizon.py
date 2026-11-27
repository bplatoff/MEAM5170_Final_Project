"""Regression tests for route-length fixed-horizon choice."""
import unittest
from unittest.mock import patch

import numpy as np

from planning.route_horizon import choose_planning_horizon


PATH = np.array([[0., 0.], [3., 0.], [3., 4.]])  # L = 7 m, endpoint distance = 5 m


class TestRouteHorizon(unittest.TestCase):
    def test_defaults_use_full_astar_polyline_not_endpoint_distance(self):
        T, L = choose_planning_horizon({"horizon_mode": "astar_length"}, PATH)
        self.assertAlmostEqual(L, 7.0)
        self.assertAlmostEqual(T, 7.0 / 0.4 + 2.0)

    def test_cruising_speed_and_buffer_are_configurable(self):
        T, _ = choose_planning_horizon({"horizon_mode": "astar_length", "cruise_speed_mps": 0.7,
                                        "time_buffer_s": 1.5}, PATH)
        self.assertAlmostEqual(T, 11.5)

    def test_quantum_rounds_up_to_tube_grid(self):
        cfg = {"horizon_mode": "astar_length", "cruise_speed_mps": 0.6,
               "time_buffer_s": 0.0, "horizon_quantum_s": 0.02}
        T, _ = choose_planning_horizon(cfg, PATH)
        self.assertAlmostEqual(T, 11.68)
        self.assertAlmostEqual(T / 0.02, round(T / 0.02))

    def test_fixed_mode_retains_existing_T(self):
        T, L = choose_planning_horizon({"T": 12.0}, PATH)
        self.assertEqual(T, 12.0)
        self.assertEqual(L, 7.0)

    def test_does_not_modify_config(self):
        cfg = {"horizon_mode": "astar_length", "cruise_speed_mps": 0.4}
        before = cfg.copy()
        choose_planning_horizon(cfg, PATH)
        self.assertEqual(cfg, before)

    def test_rejects_bad_path_and_parameters(self):
        with self.assertRaises(ValueError):
            choose_planning_horizon({"horizon_mode": "astar_length"}, [[0., 0.], [0., 0.]])
        with self.assertRaises(ValueError):
            choose_planning_horizon({"horizon_mode": "astar_length"}, [[0., 0., 0.], [1., 1., 1.]])
        for changes in ({"cruise_speed_mps": 0.}, {"cruise_speed_mps": np.nan},
                        {"time_buffer_s": -1.}, {"horizon_quantum_s": 0.},
                        {"horizon_quantum_s": np.inf}):
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    choose_planning_horizon({"horizon_mode": "astar_length", **changes}, PATH)
        for bad in (0, -2, np.inf):
            with self.subTest(fixed_T=bad):
                with self.assertRaises(ValueError):
                    choose_planning_horizon({"T": bad}, PATH)
        with self.assertRaises(ValueError):
            choose_planning_horizon({"horizon_mode": "bad"}, PATH)


class TestRouteHorizonTwoPass(unittest.TestCase):
    def test_two_pass_reuses_nominal_horizon_and_tube_grid(self):
        from planning.mock_environment import MockEnvironment
        from planning.robust_control.tube_error_model import design_tube_error_model
        from planning.robust_two_pass import plan_two_pass_friction_tube

        original = {"horizon_mode": "astar_length", "cruise_speed_mps": 0.4,
                    "time_buffer_s": 2.0, "footprint_model": "three_circle"}
        configs, durations = [], []

        def fake_planner(start, goal, env, cfg):
            T, _ = choose_planning_horizon(cfg, PATH)
            configs.append(dict(cfg))
            durations.append(T)
            return {"success": True, "t": np.linspace(0.0, T, 31)}

        def contacts(plan):
            steps = int(round(plan["t"][-1] / .02))
            return {"stance_mask": np.ones((steps, 4), dtype=int),
                    "normal_forces": np.full((steps, 4), 20.0 * 9.81 / 4),
                    "mu_plan": .5, "mu_lo": .5, "mass": 20.0}

        with patch("planning.robust_two_pass.plan_trajectory", side_effect=fake_planner):
            result = plan_two_pass_friction_tube(
                [0., 0., 0., 0.], [3., 4., 0., 0.], MockEnvironment(),
                original, contacts, design_tube_error_model(.02), .02)
        self.assertTrue(result["success"])
        self.assertEqual(len(configs), 2)
        self.assertEqual(configs[0]["horizon_mode"], "astar_length")
        self.assertAlmostEqual(configs[0]["horizon_quantum_s"], .02)
        self.assertEqual(configs[1]["horizon_mode"], "fixed")
        self.assertEqual(durations[0], durations[1])
        self.assertAlmostEqual(configs[1]["T"], durations[0])
        self.assertEqual(original["horizon_mode"], "astar_length")
        self.assertNotIn("T", original)


if __name__ == "__main__":
    unittest.main()
