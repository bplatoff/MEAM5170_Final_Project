"""Regression tests for the default A* grid resolution and failure diagnostics."""

import unittest
from unittest.mock import patch

import numpy as np

from planning.footprint_astar import astar_three_circle_path
from planning.mock_environment import rectangle


class TestAstarResolution(unittest.TestCase):
    def setUp(self):
        self.centerline = 1.55
        self.start = np.array([0.5, self.centerline, 0.0, 0.0])
        self.goal = np.array([11.5, self.centerline, 0.0, 0.0])
        self.bounds = (0.0, 12.0, 0.0, 3.0)

    def passage(self, width):
        low = self.centerline - width / 2
        high = self.centerline + width / 2
        return [rectangle(3.0, 8.0, 0.0, low),
                rectangle(3.0, 8.0, high, 3.0)]

    def astar(self, width, **kwargs):
        return astar_three_circle_path(
            self.start, self.goal, self.bounds, self.passage(width),
            length=0.7, width=0.31, safety_margin=0.05,
            extra_clearance=0.01, **kwargs,
        )

    def test_default_005_reaches_offgrid_centerline(self):
        path = self.astar(0.60)
        np.testing.assert_allclose(path[0], self.start[:2])
        np.testing.assert_allclose(path[-1], self.goal[:2])
        self.assertTrue(np.allclose(path[:, 1], self.centerline))

    def test_explicit_010_override_still_rejects_offgrid_centerline(self):
        with self.assertRaisesRegex(RuntimeError, r"resolution=0.1 m"):
            self.astar(0.60, resolution=0.10)

    def test_failure_diagnostic_does_not_claim_continuous_infeasibility(self):
        with self.assertRaises(RuntimeError) as observed:
            self.astar(0.40)
        self.assertIn("grid", str(observed.exception))
        self.assertIn("resolution=0.05 m", str(observed.exception))
        self.assertIn("does not establish", str(observed.exception))

    def test_direct_collocation_fallback_default_is_005(self):
        from planning.direct_collocation import plan_trajectory
        from planning.mock_environment import make_empty_environment, default_start_goal
        start, goal = default_start_goal()
        config = dict(N=15, T=18.0, max_speed=1.5,
                      smoothness_weight=1.0, robot_length=0.7,
                      robot_width=0.31, r_safety=0.05,
                      astar_clearance=0.01,
                      footprint_model="three_circle", footprint_yaw_mode="fixed",
                      footprint_yaw=0.0)
        # Verify actual call argument, avoiding an extra IPOPT solve.
        from planning import direct_collocation
        original_astar = direct_collocation.astar_three_circle_path
        with patch.object(direct_collocation, "astar_three_circle_path", wraps=original_astar) as astar:
            result = plan_trajectory(start, goal, make_empty_environment(), config)
        self.assertTrue(result["success"], result["return_status"])
        self.assertAlmostEqual(astar.call_args.kwargs["resolution"], 0.05)


if __name__ == "__main__":
    unittest.main()
