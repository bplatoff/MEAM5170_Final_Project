"""Certification and bounded targeted-repair tests (no changes to controller)."""
import unittest
from unittest.mock import patch

import numpy as np

from planning.continuous_clearance import verify_continuous_clearance
from planning.direct_collocation import plan_trajectory
from planning.mock_environment import MockEnvironment, rectangle, make_single_obstacle_environment, default_start_goal


def config(**overrides):
    cfg = {
        "N": 30, "T": 12.0, "max_speed": 1.0,
        "smoothness_weight": 1.0,
        "robot_length": 0.7, "robot_width": 0.31, "r_safety": 0.05,
        "astar_resolution": 0.1, "astar_clearance": 0.01,
        "footprint_model": "three_circle", "footprint_yaw_mode": "fixed",
        "footprint_yaw": 0.0, "continuous_clearance": True,
        "clearance_max_repairs": 2,
    }
    cfg.update(overrides)
    return cfg


class TestContinuousClearance(unittest.TestCase):
    def test_safe_straight_motion_is_certified_without_dense_sampling(self):
        env = MockEnvironment()
        r = {
            "t": np.array([0.0, 2.0]),
            "x": np.array([[1., 1.5, .2, 0.], [1.4, 1.5, .2, 0.]]),
            "u": np.array([[0., 0.]]),
        }
        c = verify_continuous_clearance(r, env, config())
        self.assertTrue(c["certified"], c["findings"])
        self.assertLess(c["checks"], 30)

    def test_detects_true_midinterval_collision_with_safe_endpoints(self):
        env = MockEnvironment(x_min=0., x_max=6., y_min=0., y_max=3.,
                              obstacles=[rectangle(2.8, 3.2, 0., .4)])
        # p_y(t) = 1-t+0.5*t^2, so p_y(0)=p_y(2)=1,
        # yet p_y(1)=0.5, penetrating the circle-inflated obstacle.
        r = {"t": np.array([0., 2.]),
             "x": np.array([[3., 1., 0., -1.], [3., 1., 0., 1.]]),
             "u": np.array([[0., 1.]])}
        c = verify_continuous_clearance(r, env, config())
        self.assertFalse(c["certified"])
        self.assertEqual(c["findings"][0]["status"], "collision")
        self.assertEqual(c["findings"][0]["feature"], ("obstacle", 0))
        self.assertAlmostEqual(c["findings"][0]["time_s"], 1.0)
        self.assertLess(c["findings"][0]["sampled_clearance_m"], 0.)

    def test_detects_midinterval_wall_penetration(self):
        # y(t) = 1 - 1.6*t + 0.8*t^2: two safe endpoints, wall collision at t=1.
        r = {"t": np.array([0., 2.]),
             "x": np.array([[3., 1., 0., -1.6], [3., 1., 0., 1.6]]),
             "u": np.array([[0., 1.6]])}
        c = verify_continuous_clearance(r, MockEnvironment(), config())
        self.assertFalse(c["certified"])
        self.assertEqual(c["findings"][0]["status"], "collision")
        self.assertEqual(c["findings"][0]["feature"], ("wall", "bottom"))

    def test_inconclusive_is_not_certified(self):
        env = MockEnvironment(obstacles=[rectangle(1.0, 1.5, 1.78, 2.5)])
        r = {"t": np.array([0., 2.]),
             "x": np.array([[1., 1.5, .2, 0.], [1.4, 1.5, .2, 0.]]),
             "u": np.zeros((1, 2))}
        c = verify_continuous_clearance(r, env, config(), max_depth=0)
        self.assertFalse(c["certified"])
        self.assertEqual(c["findings"][0]["status"], "inconclusive")

    def test_rejects_invalid_dynamics(self):
        r = {"t": np.array([0., 2.]),
             "x": np.array([[1., 1.5, 0., 0.], [2., 1.5, 0., 0.]]),
             "u": np.zeros((1, 2))}
        c = verify_continuous_clearance(r, MockEnvironment(), config())
        self.assertFalse(c["certified"])
        self.assertEqual(c["status"], "dynamics_mismatch")

    def test_requires_bounded_yaw(self):
        start, goal = default_start_goal()
        with self.assertRaisesRegex(ValueError, "footprint_max_yaw_rate"):
            plan_trajectory(start, goal, MockEnvironment(),
                            config(footprint_yaw_mode="path_tangent"))

    def test_repair_cycle_adds_witness_and_successfully_stops(self):
        from planning import direct_collocation as dc
        start, goal = default_start_goal()
        r = {"success": True, "return_status": "Solve_Succeeded",
             "_decision_variables": np.array([0., 1.])}
        fail = {"certified":False,"findings":[{"time_s":1.234}]}
        passed = {"certified":True,"findings":[]}
        with (patch.object(dc, "_solve_collocation_once", side_effect=[r.copy(), r.copy()]) as solve,
              patch("planning.continuous_clearance.verify_continuous_clearance",
                    side_effect=[fail, passed])):
            out = plan_trajectory(start, goal, MockEnvironment(), config())
        self.assertTrue(out["success"])
        self.assertEqual(out["planning_status"], "success")
        self.assertEqual(out["repair_attempts"], 1)
        self.assertEqual(solve.call_count, 2)
        self.assertEqual(solve.call_args_list[1].kwargs["repair_times"], [1.234])
        np.testing.assert_allclose(solve.call_args_list[1].kwargs["warm_start"], [0., 1.])

    def test_finite_retry_limit_fails_closed(self):
        from planning import direct_collocation as dc
        start, goal = default_start_goal()
        r = {"success":True, "return_status":"Solve_Succeeded",
             "_decision_variables": np.array([0., 1.])}
        fails = [{"certified":False,"findings":[{"time_s":t}]}
                 for t in (1.234, 1.555, 1.999)]
        with (patch.object(dc, "_solve_collocation_once", side_effect=[r.copy() for _ in range(3)]) as solve,
              patch("planning.continuous_clearance.verify_continuous_clearance",
                    side_effect=fails)):
            out = plan_trajectory(start,goal,MockEnvironment(),config())
        self.assertFalse(out["success"])
        self.assertEqual(out["planning_status"], "uncertified")
        self.assertEqual(out["repair_attempts"], 2)
        self.assertEqual(solve.call_count, 3)

    def test_solver_failure_is_not_called_infeasible(self):
        from planning import direct_collocation as dc
        start, goal = default_start_goal()
        with patch.object(dc, "_solve_collocation_once", return_value={
            "success":False, "return_status":"Restoration_Failed",
            "_decision_variables":np.array([1.])}):
            out = plan_trajectory(start,goal,MockEnvironment(),config())
        self.assertFalse(out["success"])
        self.assertEqual(out["planning_status"], "solver_failed")
        self.assertNotEqual(out["planning_status"], "no_feasible_plan")

    def test_ipopt_runtime_exception_is_classified_as_solver_failure(self):
        from planning import direct_collocation as dc
        start, goal = default_start_goal()
        with patch.object(dc.ca, "nlpsol", side_effect=RuntimeError("IPOPT crashed")):
            out = plan_trajectory(start, goal, MockEnvironment(), config())
        self.assertFalse(out["success"])
        self.assertEqual(out["planning_status"], "solver_failed")
        self.assertIn("IPOPT crashed", out["return_status"])
        self.assertEqual(out["repair_attempts"], 0)

    def test_ipopt_exception_during_repair_is_classified_as_solver_failure(self):
        from planning import direct_collocation as dc
        start, goal = default_start_goal()
        first = {"success": True, "return_status": "Solve_Succeeded",
                 "_decision_variables": np.array([0., 1.])}
        finding = {"certified": False, "findings": [{"time_s": 1.234}]}
        with (patch.object(dc, "_solve_collocation_once",
                           side_effect=[first, dc._CollocationSolverError("IPOPT repair crashed")]),
              patch("planning.continuous_clearance.verify_continuous_clearance",
                    return_value=finding)):
            out = plan_trajectory(start, goal, MockEnvironment(), config())
        self.assertFalse(out["success"])
        self.assertEqual(out["planning_status"], "solver_failed")
        self.assertEqual(out["repair_attempts"], 1)
        self.assertEqual(len(out["repair_history"]), 1)

    def test_astar_failure_is_not_called_infeasible(self):
        from planning import direct_collocation as dc
        start, goal = default_start_goal()
        with patch.object(dc, "_solve_collocation_once",
                          side_effect=RuntimeError("No grid route")):
            out = plan_trajectory(start,goal,MockEnvironment(),config())
        self.assertFalse(out["success"])
        self.assertEqual(out["planning_status"], "planning_failed")

    def test_original_planner_behavior_remains_opt_in(self):
        start, goal = default_start_goal()
        out = plan_trajectory(start, goal, MockEnvironment(), config(continuous_clearance=False))
        self.assertTrue(out["success"])
        self.assertNotIn("certification", out)

    def test_real_ipopt_resolves_with_added_targeted_constraint(self):
        # Force one repair pass on a benign course to exercise the *actual*
        # CasADi constraint-generation branch, not only a mocked retry loop.
        from planning import continuous_clearance as cc
        start, goal = default_start_goal()
        orig = cc.verify_continuous_clearance
        n = [0]

        def first_uncertain_then_real(result, env, cfg, **kwargs):
            n[0] += 1
            if n[0] == 1:
                return {"certified":False,"status":"uncertified",
                        "checks":1,"min_sampled_clearance_m":.2,
                        "findings":[{"time_s":5.123,"status":"inconclusive"}]}
            return orig(result, env, cfg, **kwargs)

        with patch.object(cc, "verify_continuous_clearance",
                          side_effect=first_uncertain_then_real):
            out = plan_trajectory(start, goal, MockEnvironment(), config())
        self.assertEqual(out["planning_status"], "success")
        self.assertTrue(out["certification"]["certified"])
        self.assertEqual(out["repair_attempts"], 1)
        self.assertEqual(n[0], 2)

    def test_actual_single_obstacle_planner_certifies(self):
        start, goal = default_start_goal()
        cfg = config(footprint_yaw_mode="path_tangent", footprint_max_yaw_rate=0.6)
        out = plan_trajectory(start, goal, make_single_obstacle_environment(), cfg)
        self.assertTrue(out["success"], out.get("planning_status"))
        self.assertEqual(out["planning_status"], "success")
        self.assertTrue(out["certification"]["certified"])


if __name__ == "__main__":
    unittest.main()
