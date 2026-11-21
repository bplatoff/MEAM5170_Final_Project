"""Regression tests for passing command-line options into the planner."""

import sys
import unittest
from unittest.mock import patch

from planning import test_planner


class TestPlannerCLI(unittest.TestCase):
    def test_three_circle_flags_reach_planner(self):
        """Command-line geometry/yaw choices must survive config creation."""
        for yaw_mode, angle in (("path_tangent", "0.0"), ("fixed", "0.35")):
            with self.subTest(yaw_mode=yaw_mode):
                argv = [
                    "planning.test_planner",
                    "--scenario", "multi",
                    "--footprint-model", "three_circle",
                    "--yaw-mode", yaw_mode,
                    "--fixed-yaw", angle,
                    "--no-plot",
                ]
                # Replace the optimization and validation calls with mocks.
                # We are testing CLI wiring, not running IPOPT again.
                with (
                    patch.object(sys, "argv", argv),
                    patch.object(test_planner, "planner_adapter") as adapter,
                    patch.object(test_planner, "validate_result"),
                ):
                    test_planner.main()

                config = adapter.call_args.kwargs["config"]
                self.assertEqual(config["footprint_model"], "three_circle")
                self.assertEqual(config["footprint_yaw_mode"], yaw_mode)
                self.assertAlmostEqual(config["footprint_yaw"], float(angle))


if __name__ == "__main__":
    unittest.main()
