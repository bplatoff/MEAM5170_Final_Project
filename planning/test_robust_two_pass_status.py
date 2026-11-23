"""Integration-contract tests for two-pass expected planning failures.

Planner results and tube radii are mocked: these test safe failure handling,
not real robot disturbance bounds or IPOPT convergence.
"""

import unittest
from unittest.mock import Mock, patch

import numpy as np

from planning.robust_two_pass import plan_two_pass_friction_tube


class TestTwoPassStatuses(unittest.TestCase):
    def setUp(self):
        self.cfg = {"footprint_model": "three_circle"}
        self.environment = object()
        self.dt = 0.02
        self.model = {"B": np.vstack((0.5*self.dt**2*np.eye(2),
                                       self.dt*np.eye(2))),
                      "P": np.eye(4), "rho": 0.5}
        self.nominal = {"success": True, "t": np.linspace(0., 12., 31),
                        "x": np.zeros((31, 4)), "return_status": "Solve_Succeeded"}
        self.robust = {"success": True, "t": np.linspace(0., 12., 31),
                       "return_status": "Solve_Succeeded"}
        self.tube = np.full(601, 0.21)
        self.contacts = {
            "stance_mask": np.ones((600, 4), dtype=int),
            "normal_forces": np.ones((600, 4)),
            "mu_plan": 0.5, "mu_lo": 0.3, "mass": 20.0,
        }

    def invoke(self, plans, *, contacts=None, expected_count=None):
        predictor = Mock(return_value=self.contacts if contacts is None else contacts)
        with (patch("planning.robust_two_pass.plan_trajectory", side_effect=plans) as planner,
              patch("planning.robust_two_pass.compute_friction_tube_position_radii",
                    return_value=self.tube) as tube):
            result = plan_two_pass_friction_tube(
                [1., 1.5, 0., 0.], [11., 1.5, 0., 0.], self.environment,
                self.cfg, predictor, self.model, self.dt)
        if expected_count is not None:
            self.assertEqual(planner.call_count, expected_count)
        return result, predictor, tube, planner

    def test_success_preserves_both_results_and_top_level_status(self):
        result, predictor, tube, planner = self.invoke(
            [self.nominal, self.robust], expected_count=2)
        self.assertTrue(result["success"])
        self.assertIsNone(result["failed_stage"])
        self.assertEqual(result["planning_status"], "success")
        self.assertEqual(result["return_status"], "Solve_Succeeded")
        self.assertIs(result["nominal"], self.nominal)
        self.assertIs(result["robust"], self.robust)
        self.assertEqual(result["failure_diagnostics"], {})
        self.assertAlmostEqual(result["tube_position_margin_m"], 0.21)
        self.assertAlmostEqual(planner.call_args_list[0].args[3]["tube_position_margin_m"], 0)
        self.assertAlmostEqual(planner.call_args_list[1].args[3]["tube_position_margin_m"], .21)
        self.assertNotIn("tube_position_margin_m", self.cfg)
        predictor.assert_called_once_with(self.nominal)
        tube.assert_called_once()

    def test_nominal_structured_failure_stops_before_prediction(self):
        failed = {"success": False, "planning_status": "planning_failed",
                  "return_status": "Three-circle A*: center search exhausted"}
        result, predictor, tube, planner = self.invoke([failed], expected_count=1)
        self.assertFalse(result["success"])
        self.assertEqual(result["failed_stage"], "nominal")
        self.assertEqual(result["planning_status"], "planning_failed")
        self.assertEqual(result["return_status"], failed["return_status"])
        self.assertIsNone(result["nominal"])
        self.assertIsNone(result["robust"])
        self.assertIsNone(result["tube_position_margin_m"])
        predictor.assert_not_called()
        tube.assert_not_called()

    def test_robust_uncertified_preserves_nominal_only_for_diagnostics(self):
        failed = {"success": False, "planning_status": "uncertified",
                  "return_status": "Solve_Succeeded", "repair_attempts": 4,
                  "certification": {"certified": False}}
        with self.assertLogs("planning.robust_two_pass", level="WARNING") as logs:
            result, _, _, _ = self.invoke([self.nominal, failed], expected_count=2)
        self.assertIn("stage=robust", logs.output[0])
        self.assertIn("status=uncertified", logs.output[0])
        self.assertFalse(result["success"])
        self.assertEqual(result["failed_stage"], "robust")
        self.assertEqual(result["planning_status"], "uncertified")
        self.assertIs(result["nominal"], self.nominal)
        self.assertIsNone(result["robust"])
        self.assertAlmostEqual(result["tube_position_margin_m"], 0.21)
        self.assertEqual(result["failure_diagnostics"]["repair_attempts"], 4)
        self.assertFalse(result["failure_diagnostics"]["certification"]["certified"])

    def test_legacy_ipopt_failure_is_classified_without_inventing_infeasible(self):
        failed = {"success": False, "return_status": "Maximum_Iterations_Exceeded"}
        result, _, _, _ = self.invoke([self.nominal, failed], expected_count=2)
        self.assertEqual(result["failed_stage"], "robust")
        self.assertEqual(result["planning_status"], "solver_failed")
        self.assertEqual(result["return_status"], "Maximum_Iterations_Exceeded")
        self.assertNotEqual(result["planning_status"], "no_feasible_plan")

    def test_legacy_astar_exception_is_structured_at_nominal_stage(self):
        result, predictor, _, _ = self.invoke(
            [RuntimeError("Three-circle A*: center circle cannot reach the goal on this grid")],
            expected_count=1)
        self.assertFalse(result["success"])
        self.assertEqual(result["planning_status"], "planning_failed")
        self.assertIn("center circle", result["return_status"])
        predictor.assert_not_called()

    def test_new_astar_diagnostic_is_preserved_during_robust_stage(self):
        msg = ("Three-circle A* initialization failed (heading/edge/turn search exhausted; "
               "resolution=0.05 m). A grid-search failure does not establish that no "
               "continuous collision-free path exists.")
        result, _, _, _ = self.invoke([self.nominal, RuntimeError(msg)], expected_count=2)
        self.assertFalse(result["success"])
        self.assertEqual(result["failed_stage"], "robust")
        self.assertEqual(result["planning_status"], "planning_failed")
        self.assertIn("resolution=0.05 m", result["return_status"])
        self.assertIsNone(result["robust"])

    def test_collocation_solver_exception_from_pending_pr_is_classified(self):
        class _CollocationSolverError(RuntimeError):
            pass
        result, _, _, _ = self.invoke([
            self.nominal, _CollocationSolverError("IPOPT internal failure")
        ], expected_count=2)
        self.assertEqual(result["planning_status"], "solver_failed")
        self.assertEqual(result["return_status"], "IPOPT internal failure")

    def test_known_casadi_exception_is_classified(self):
        result, _, _, _ = self.invoke([
            self.nominal, RuntimeError("Error in Function::call for 'solver' [IpoptInterface]")
        ], expected_count=2)
        self.assertEqual(result["planning_status"], "solver_failed")

    def test_unexpected_runtime_error_is_not_swallowed(self):
        with self.assertRaisesRegex(RuntimeError, "programming bug"):
            self.invoke([RuntimeError("programming bug")])

    def test_configuration_errors_are_not_swallowed(self):
        with self.assertRaisesRegex(ValueError, "invalid dimensions"):
            self.invoke([ValueError("invalid dimensions")])

    def test_invalid_contact_predictor_data_remains_exception(self):
        with self.assertRaisesRegex(TypeError, "predict_contacts must return"):
            self.invoke([self.nominal], contacts="bad return")

    def test_warning_truncates_long_message_but_result_preserves_full_reason(self):
        reason = "CasADi error: " + "details " * 100
        with self.assertLogs("planning.robust_two_pass", level="WARNING") as logs:
            result, _, _, _ = self.invoke([
                self.nominal, {"success": False, "return_status": reason}
            ], expected_count=2)
        self.assertEqual(result["return_status"], reason)
        self.assertLess(len(logs.output[0]), 360)

    def test_missing_status_retains_truthful_message(self):
        result, _, _, _ = self.invoke([self.nominal, {"success": False}], expected_count=2)
        self.assertFalse(result["success"])
        self.assertEqual(result["planning_status"], "solver_failed")
        self.assertIn("without a return_status", result["return_status"])


if __name__ == "__main__":
    unittest.main()
