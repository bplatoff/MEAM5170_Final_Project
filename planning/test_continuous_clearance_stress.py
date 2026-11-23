"""Reproducible, real-optimizer stress tests for continuous-clearance repair.

These intentionally exercise actual IPOPT/A* rather than mocking the optimizer or
injecting fake certification findings. They complement, rather than replace, the
unit tests in test_continuous_clearance.py.

Run from the repository root:
    python -m unittest planning.test_continuous_clearance_stress -v

No production planner implementation is modified by this test file.
"""

import unittest
import numpy as np

from planning.direct_collocation import plan_trajectory, _solve_collocation_once
from planning.continuous_clearance import verify_continuous_clearance, _nominal_yaw_function
from planning.mock_environment import MockEnvironment, rectangle
from planning.robot_footprint import three_circle_parameters


class TestContinuousClearanceStress(unittest.TestCase):
    def setUp(self):
        # Alternating obstacles create a turn and change in nominal orientation.
        # Coarse collocation can satisfy the constraints at sampled times while
        # the rear or front footprint circle clips an obstacle between samples.
        self.env = MockEnvironment(
            x_max=9.0,
            obstacles=[
                rectangle(2.4, 3.3, 0.0, 1.55),
                rectangle(5.0, 5.9, 1.45, 3.0),
            ],
        )
        self.start = np.array([0.6, 1.5, 0.0, 0.0])
        self.goal = np.array([8.4, 1.5, 0.0, 0.0])

    @staticmethod
    def config(**overrides):
        params = {
            "N": 12, "T": 11.0, "max_speed": 1.3,
            "smoothness_weight": 1.0,
            "robot_length": 0.7, "robot_width": 0.31,
            "r_safety": 0.05, "astar_resolution": 0.1,
            "astar_clearance": 0.01,
            "footprint_model": "three_circle",
            "footprint_yaw_mode": "path_tangent",
            "footprint_yaw": 0.0,
            "footprint_max_yaw_rate": 0.5,
            "continuous_clearance": True,
            "clearance_max_repairs": 4,
        }
        params.update(overrides)
        return params

    @staticmethod
    def dense_rectangle_clearance(plan, environment, config, samples_per_interval=300):
        """Independent dense check of axis-aligned rectangles and course walls.

        Diagnostic only: dense sampling is not a mathematical certificate.
        The actual guarantee is evaluated by verify_continuous_clearance.
        """
        yaw_at, _ = _nominal_yaw_function(plan, config)
        offsets, radius = three_circle_parameters(
            config["robot_length"], config["robot_width"],
            config["r_safety"] + config.get("tube_position_margin_m", 0.0),
        )
        xmin, xmax, ymin, ymax = environment.bounds
        obstacle_boxes = []
        for polygon in environment.obstacles:
            # The stress environment uses axis-aligned rectangles.
            vertices = np.asarray(polygon)
            obstacle_boxes.append((np.min(vertices[:, 0]),
                                   np.max(vertices[:, 0]),
                                   np.min(vertices[:, 1]),
                                   np.max(vertices[:, 1])))
        t = np.asarray(plan["t"])
        x = np.asarray(plan["x"])
        u = np.asarray(plan["u"])
        min_clearance = np.inf
        for k in range(len(u)):
            h = t[k+1] - t[k]
            # Endpoint for all but last segment is visited as the next start.
            for tau in np.linspace(0.0, h, samples_per_interval, endpoint=False):
                p = x[k, :2] + tau*x[k, 2:] + .5*tau*tau*u[k]
                yaw = yaw_at((t[k] - t[0] + tau)/(t[-1] - t[0]))
                direction = np.array([np.cos(yaw), np.sin(yaw)])
                for offset in offsets:
                    c = p + offset*direction
                    min_clearance = min(min_clearance,
                                        c[0]-xmin-radius, xmax-c[0]-radius,
                                        c[1]-ymin-radius, ymax-c[1]-radius)
                    for xlo, xhi, ylo, yhi in obstacle_boxes:
                        dx = max(xlo-c[0], 0.0, c[0]-xhi)
                        dy = max(ylo-c[1], 0.0, c[1]-yhi)
                        min_clearance = min(min_clearance, np.hypot(dx,dy)-radius)
        return float(min_clearance)

    def test_natural_collisions_are_detected_and_repaired(self):
        for N in (8, 12):
            with self.subTest(N=N):
                cfg = self.config(N=N)
                # First solve: real IPOPT result, no mocked collision/failure.
                raw = _solve_collocation_once(self.start, self.goal, self.env, cfg)
                self.assertTrue(raw["success"], raw["return_status"])
                initial = verify_continuous_clearance(raw, self.env, cfg)
                self.assertFalse(initial["certified"])
                self.assertTrue(any(f["status"] == "collision" for f in initial["findings"]))
                self.assertLess(self.dense_rectangle_clearance(raw, self.env, cfg), -0.003)

                # Actual planner with enabled repair: should add a targeted
                # constraint, re-solve using IPOPT, and certify the new path.
                repaired = plan_trajectory(self.start, self.goal, self.env, cfg)
                self.assertTrue(repaired["success"], repaired.get("planning_status"))
                self.assertEqual(repaired["planning_status"], "success")
                self.assertTrue(repaired["certification"]["certified"])
                self.assertGreaterEqual(repaired["repair_attempts"], 1)
                self.assertTrue(any(
                    len(step["constraint_times_s"]) > 0
                    for step in repaired["repair_history"][1:]
                ))
                self.assertGreater(self.dense_rectangle_clearance(repaired, self.env, cfg), -1e-5)

    def test_actual_collision_with_no_retry_fails_closed(self):
        cfg = self.config(clearance_max_repairs=0)
        result = plan_trajectory(self.start, self.goal, self.env, cfg)
        self.assertFalse(result["success"])
        self.assertEqual(result["planning_status"], "uncertified")
        self.assertEqual(result["repair_attempts"], 0)
        self.assertFalse(result["certification"]["certified"])
        self.assertTrue(any(
            item["status"] == "collision"
            for item in result["certification"]["findings"]
        ))

    def test_physically_narrow_gap_returns_planning_failed_not_proven_infeasible(self):
        blocked = MockEnvironment(x_max=9.0, obstacles=[
            rectangle(2.0, 6.0, 0.0, 1.3),
            rectangle(2.0, 6.0, 1.7, 3.0),
        ])
        result = plan_trajectory(self.start, self.goal, blocked, self.config())
        self.assertFalse(result["success"])
        self.assertEqual(result["planning_status"], "planning_failed")
        self.assertNotEqual(result["planning_status"], "no_feasible_plan")

    def test_unachievable_travel_time_returns_solver_failed(self):
        result = plan_trajectory(self.start, self.goal, self.env,
                                 self.config(N=8, T=2.0, clearance_max_repairs=1))
        self.assertFalse(result["success"])
        self.assertEqual(result["planning_status"], "solver_failed")
        self.assertNotEqual(result["planning_status"], "no_feasible_plan")

    def test_yaw_limits_and_fixed_yaw_never_bypass_certification(self):
        for kwargs in (
            {"footprint_max_yaw_rate": 0.25},
            {"footprint_max_yaw_rate": 0.8},
            {"footprint_yaw_mode": "fixed"},
        ):
            with self.subTest(kwargs=kwargs):
                cfg = self.config(**kwargs)
                result = plan_trajectory(self.start, self.goal, self.env, cfg)
                if result["success"]:
                    self.assertEqual(result["planning_status"], "success")
                    self.assertTrue(result["certification"]["certified"])
                    self.assertGreater(self.dense_rectangle_clearance(result,self.env,cfg), -1e-5)
                else:
                    self.assertNotEqual(result.get("planning_status"), "success")

    def test_insufficient_verification_budget_never_returns_success(self):
        # The depth budget is deliberately unusable for this chicane. The
        # real optimizer is retried and can no longer add novel times.
        cfg = self.config(clearance_max_depth=0, clearance_max_repairs=2)
        result = plan_trajectory(self.start, self.goal, self.env, cfg)
        self.assertFalse(result["success"])
        self.assertEqual(result["planning_status"], "uncertified")
        self.assertGreaterEqual(result["repair_attempts"], 1)
        self.assertFalse(result["certification"]["certified"])
        self.assertTrue(any(
            item["status"] == "inconclusive"
            for item in result["certification"]["findings"]
        ))


if __name__ == "__main__":
    unittest.main()
