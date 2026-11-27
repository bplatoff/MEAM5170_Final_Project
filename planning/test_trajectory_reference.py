"""Pure and end-to-end checks for time-indexed MPC reference evaluation."""
import unittest

import casadi as ca
import numpy as np

from planning.trajectory_reference import TrajectoryReference, evaluate_trajectory
from planning.yaw_reference import make_rate_limited_yaw_reference


def synthetic_plan(accelerations=((0.4, 0.0), (0.0, 0.2), (-0.1, -0.2)),
                   horizon=1.2, start_velocity=(0.3, -0.2), path=None,
                   yaw_mode="fixed", initial_yaw=0.3, max_rate=0.6):
    """Construct an exact double-integrator result without running IPOPT."""
    acc = np.array(accelerations, dtype=float)
    n = len(acc)
    times = np.linspace(0, horizon, n + 1)
    states = np.zeros((n + 1, 4))
    states[0, 2:] = start_velocity
    for k in range(n):
        dt = times[k + 1] - times[k]
        states[k + 1] = states[k]
        states[k + 1, :2] += dt * states[k, 2:] + 0.5 * dt**2 * acc[k]
        states[k + 1, 2:] += dt * acc[k]
    if path is None:
        path = np.array([[0., 0.], [1., 0.], [1., 1.]])
    config = {"footprint_model": "three_circle", "footprint_yaw_mode": yaw_mode,
              "footprint_yaw": initial_yaw, "footprint_max_yaw_rate": max_rate}
    plan = {"success": True, "t": times, "x": states, "u": acc,
            "astar_path": np.asarray(path, dtype=float)}
    if yaw_mode == "fixed":
        plan["yaw_ref"] = np.full(n + 1, initial_yaw)
    elif yaw_mode == "path_tangent" and max_rate is not None:
        tau = ca.collocation_points(3, "legendre")
        fractions = [0.0]
        for k in range(n):
            fractions.extend((k + float(s)) / n for s in tau)
            fractions.append((k + 1) / n)
        fn, _ = make_rate_limited_yaw_reference(
            path, fractions, horizon, max_rate, initial_yaw=initial_yaw)
        plan["yaw_ref"] = np.array([fn(k / n) for k in range(n + 1)])
    else:
        plan["yaw_ref"] = np.zeros(n + 1)
    return plan, config


class TestTrajectoryReference(unittest.TestCase):
    def test_exact_knots_and_endpoints(self):
        plan, config = synthetic_plan()
        out = evaluate_trajectory(plan, plan["t"], config)
        np.testing.assert_array_equal(out["x"], plan["x"])
        np.testing.assert_array_equal(out["u"], np.vstack([plan["u"], plan["u"][-1]]))
        np.testing.assert_allclose(out["yaw"], 0.3)
        np.testing.assert_allclose(out["yaw_rate"], 0.)

    def test_inside_interval_uses_constant_acceleration(self):
        plan, config = synthetic_plan()
        ref = TrajectoryReference(plan, config)
        t = 0.25
        x0, a0 = plan["x"][0], plan["u"][0]
        expected = x0.copy()
        expected[:2] += t * x0[2:] + 0.5 * t*t*a0
        expected[2:] += t*a0
        state = ref.evaluate(t)
        np.testing.assert_allclose(state["x"], expected)
        np.testing.assert_allclose(state["u"], a0)
        self.assertEqual(state["t"], t)
        self.assertIsInstance(state["yaw"], float)

    def test_right_continuity_of_acceleration(self):
        plan, cfg = synthetic_plan()
        ref = TrajectoryReference(plan, cfg)
        np.testing.assert_array_equal(ref.evaluate(plan["t"][1])["u"], plan["u"][1])
        np.testing.assert_array_equal(ref.evaluate(plan["t"][-1])["u"], plan["u"][-1])

    def test_sample_unsorted_times_and_duplicate_queries(self):
        plan, cfg = synthetic_plan()
        ref = TrajectoryReference(plan, cfg)
        q = np.array([0.6, 0.0, 0.6, 1.2, 0.13])
        out = ref.evaluate(q)
        self.assertEqual(out["x"].shape, (5, 4))
        self.assertEqual(out["u"].shape, (5, 2))
        np.testing.assert_array_equal(out["t"], q)
        np.testing.assert_array_equal(out["x"][0], out["x"][2])

    def test_fixed_yaw_with_nonzero_start(self):
        plan, cfg = synthetic_plan(initial_yaw=-2.9)
        out = TrajectoryReference(plan, cfg).evaluate(np.linspace(0, 1.2, 13))
        np.testing.assert_allclose(out["yaw"], -2.9)
        np.testing.assert_array_equal(out["yaw_rate"], 0.)

    def test_bounded_turn_matches_internal_collocation_yaw(self):
        path = np.array([[0., 0.], [1., 0.], [1., 1.]])
        plan, cfg = synthetic_plan(accelerations=((0., 0.),) * 4,
                                   horizon=2., start_velocity=(0., 0.),
                                   path=path, yaw_mode="path_tangent",
                                   initial_yaw=0., max_rate=.4)
        ref = TrajectoryReference(plan, cfg)
        tau = ca.collocation_points(3, "legendre")
        n = len(plan["u"])
        fractions = [0.]
        for k in range(n):
            fractions.extend((k + float(s)) / n for s in tau)
            fractions.append((k + 1) / n)
        fn, _ = make_rate_limited_yaw_reference(
            path, fractions, 2., .4, initial_yaw=0.)
        requested = np.array([0., .3, .74, 1., 1.35, 1.91, 2.])
        out = ref.evaluate(requested)
        np.testing.assert_allclose(out["yaw"], [fn(t/2.) for t in requested], atol=1e-12)
        self.assertLessEqual(np.max(np.abs(out["yaw_rate"])), .4 + 1e-10)
        np.testing.assert_allclose(ref.evaluate(plan["t"])["yaw"], plan["yaw_ref"])

    def test_yaw_not_naively_interpolated_between_knots(self):
        path = np.array([[0., 0.], [1., 0.], [1., 1.]])
        plan, cfg = synthetic_plan(accelerations=((0., 0.),) * 4,
                                   horizon=2., start_velocity=(0., 0.),
                                   path=path, yaw_mode="path_tangent",
                                   initial_yaw=0., max_rate=.4)
        ref = TrajectoryReference(plan, cfg)
        fine_t = ref.yaw_times[1:-1]
        actual = ref.evaluate(fine_t)["yaw"]
        coarse = np.interp(fine_t, plan["t"], plan["yaw_ref"])
        self.assertGreater(np.max(np.abs(actual - coarse)), 1e-4)

    def test_unwrapped_turn_across_pi(self):
        path = np.array([[0., 0.], [-1., -.01]])
        plan, cfg = synthetic_plan(accelerations=((0., 0.),) * 4,
                                   horizon=2., start_velocity=(0., 0.), path=path,
                                   yaw_mode="path_tangent", initial_yaw=3.13,
                                   max_rate=.6)
        ref = TrajectoryReference(plan, cfg)
        out = ref.evaluate(np.linspace(0, 2, 101))
        self.assertLessEqual(np.max(np.abs(out["yaw_rate"])), .6 + 1e-10)
        self.assertTrue(np.all(np.abs(np.diff(out["yaw"])) < .02))
        self.assertTrue(np.all(out["yaw"] > 3.))

    def test_rejects_failed_plan(self):
        plan, cfg = synthetic_plan()
        plan["success"] = False
        with self.assertRaisesRegex(ValueError, "successful"):
            TrajectoryReference(plan, cfg)

    def test_rejects_unbounded_tangent_yaw(self):
        plan, cfg = synthetic_plan(yaw_mode="path_tangent", max_rate=None)
        with self.assertRaisesRegex(ValueError, "Unbounded path-tangent"):
            TrajectoryReference(plan, cfg)

    def test_rejects_yaw_mismatch(self):
        plan, cfg = synthetic_plan()
        cfg["footprint_yaw"] += 0.1
        with self.assertRaisesRegex(ValueError, "Yaw knots disagree"):
            TrajectoryReference(plan, cfg)

    def test_rejects_malformed_time_or_arrays(self):
        plan, cfg = synthetic_plan()
        plan["t"][2] = plan["t"][1]
        with self.assertRaises(ValueError):
            TrajectoryReference(plan, cfg)
        plan, cfg = synthetic_plan()
        plan["u"] = plan["u"][:1]
        with self.assertRaises(ValueError):
            TrajectoryReference(plan, cfg)

    def test_rejects_dynamics_mismatch(self):
        plan, cfg = synthetic_plan()
        plan["x"][2, 0] += .03
        with self.assertRaisesRegex(ValueError, "Knot states"):
            TrajectoryReference(plan, cfg)

    def test_rejects_out_of_horizon_nonfinite_or_wrong_dimension_queries(self):
        plan, cfg = synthetic_plan()
        ref = TrajectoryReference(plan, cfg)
        for t in (-.01, 1.21, np.nan, np.inf, [], [[.2, .3]]):
            with self.subTest(t=t), self.assertRaises(ValueError):
                ref.evaluate(t)

    def test_rejects_single_circle_mode(self):
        plan, cfg = synthetic_plan()
        cfg["footprint_model"] = "single_circle"
        with self.assertRaisesRegex(ValueError, "three-circle"):
            TrajectoryReference(plan, cfg)

    def test_cache_does_not_alias_source_arrays(self):
        plan, cfg = synthetic_plan()
        ref = TrajectoryReference(plan, cfg)
        expected = ref.evaluate(.2)["x"].copy()
        plan["x"][:] = 100.
        np.testing.assert_allclose(ref.evaluate(.2)["x"], expected)

    def test_two_pass_wrapper_is_not_an_executable_plan(self):
        plan, cfg = synthetic_plan()
        wrapper = {"success": True, "nominal": plan, "robust": plan}
        with self.assertRaises(ValueError):
            TrajectoryReference(wrapper, cfg)


class TestPlannerIntegration(unittest.TestCase):
    def test_real_ipopt_result_can_be_sampled(self):
        # Run against the real planner in the team checkout. The lightweight
        # unit-test workspace used to author this patch has no full planner.
        try:
            from planning.direct_collocation import plan_trajectory
            from planning.mock_environment import make_empty_environment, default_start_goal
        except ImportError as exc:
            self.skipTest(f"Full planning package not available: {exc}")
        cfg = {"N": 30, "T": 12.0, "max_speed": 1.0,
               "smoothness_weight": 1.0, "robot_length": 0.7,
               "robot_width": 0.31, "r_safety": 0.05,
               "astar_resolution": 0.1, "astar_clearance": 0.01,
               "footprint_model": "three_circle", "footprint_yaw_mode": "fixed",
               "footprint_yaw": 0.0}
        start, goal = default_start_goal()
        plan = plan_trajectory(start, goal, make_empty_environment(), cfg)
        self.assertTrue(plan["success"], plan["return_status"])
        ref = TrajectoryReference(plan, cfg)
        knots = ref.evaluate(plan["t"])
        np.testing.assert_allclose(knots["x"], plan["x"], atol=1e-8)
        samples = ref.evaluate(np.linspace(0, cfg["T"], 241))
        self.assertEqual(samples["x"].shape, (241, 4))
        np.testing.assert_allclose(samples["yaw_rate"], 0.0)
        self.assertTrue(np.all(np.isfinite(samples["x"])))


if __name__ == "__main__":
    unittest.main()
