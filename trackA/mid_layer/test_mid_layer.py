"""Unit tests for the mid layer. No simulator needed.

Run from the repo root:  python -m unittest trackA.mid_layer.test_mid_layer -v
"""
import math
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from planning.robust_control.friction_tube import compute_friction_tube_position_radii
from planning.robust_control.tube_error_model import design_tube_error_model
from trackA.mid_layer.config import MidLayerConfig
from trackA.mid_layer.controller import TubeMidLayer
from trackA.mid_layer.feedback import lqr_acceleration, velocity_command, yaw_rate_command
from trackA.mid_layer.nominal_qp import NominalQP
from trackA.mid_layer.planner_adapter import (Plan, CourseGeometry, clearance, constant_tube_radius, load_course,
                                              planner_friction)
from trackA.mid_layer.reference import PlannerReference, waypoint_yaw_reference, wrap_to_pi
from trackA.mid_layer.tube import TubeTracker
from trackC.course import Course

CFG = MidLayerConfig()
COURSE = Path(__file__).resolve().parents[1] / "courses" / "midlayer_test_v1.yaml"


def straight_reference(speed=0.4, T=12.0, h=0.4, ramp=1.0):
    """Trapezoidal speed along +x with piecewise-constant acceleration on the knot grid."""
    K = int(round(T / h))
    u = np.zeros((K, 2))
    n = int(round(ramp / h))
    u[:n, 0], u[-n:, 0] = speed / (n * h), -speed / (n * h)
    x = np.zeros((K + 1, 4))
    for k in range(K):
        x[k + 1, :2] = x[k, :2] + h * x[k, 2:] + 0.5 * h * h * u[k]
        x[k + 1, 2:] = x[k, 2:] + h * u[k]
    return PlannerReference(np.arange(K + 1) * h, x, u, np.zeros(K + 1))


def open_geometry(obstacles=(), bounds=(-2.0, 20.0, -3.0, 3.0)):
    course = Course(dict(width=6.0, lower_flat=dict(start=-2.0, length=7.0),
                         ramp=dict(length_along_slope=7.0, angle_deg=0), upper_flat=dict(length=8.0)))
    return CourseGeometry("test", course, bounds, list(obstacles), np.array([0.0, 0.0]))


def make_plan(reference, geometry=None):
    geometry = geometry if geometry is not None else open_geometry()
    return Plan(reference, geometry, constant_tube_radius(reference.t_end + 3.0, CFG), CFG.robot_radius,
                reference.t_end, 0.1, 1)


def rect(xa, xb, ya, yb):
    return np.array([[xa, ya], [xb, ya], [xb, yb], [xa, yb]], dtype=float)


class TestReference(unittest.TestCase):
    def test_hits_the_knots_and_holds_the_end(self):
        r = straight_reference()
        for k in (0, 3, 10, 29):
            np.testing.assert_allclose(r.state(r.t[k]), r.x[k], atol=1e-12)
        np.testing.assert_allclose(r.state(r.t_end + 5.0), np.r_[r.x[-1, :2], 0, 0], atol=1e-12)
        np.testing.assert_allclose(r.accel(r.t_end + 1.0), [0, 0])

    def test_yaw_is_rate_limited_and_starts_at_measured_yaw(self):
        xy = np.column_stack((np.zeros(20), np.linspace(0, 4, 20)))     # heading +90 deg
        yaw = waypoint_yaw_reference(xy, 0.4, 0.0, 0.5)
        self.assertEqual(yaw[0], 0.0)
        self.assertLessEqual(np.abs(np.diff(yaw) / 0.4).max(), 0.5 + 1e-9)
        self.assertAlmostEqual(yaw[-1], np.pi / 2, places=6)

    def test_yaw_holds_on_short_steps(self):
        xy = np.array([[0, 0], [1, 0], [1.001, 0.001], [1.002, -0.001]], float)
        yaw = waypoint_yaw_reference(xy, 1.0, 0.0, 10.0)
        np.testing.assert_allclose(yaw, 0.0, atol=1e-12)

    def test_yaw_takes_the_short_way_across_pi(self):
        xy = np.array([[0, 0], [-1, -0.05], [-2, -0.1]], float)
        yaw = waypoint_yaw_reference(xy, 1.0, 3.1, 10.0)
        self.assertGreater(yaw[1], 3.1)     # crosses +pi instead of turning almost a full circle


class TestTube(unittest.TestCase):
    def test_matches_track_b_functions(self):
        model = design_tube_error_model(CFG.dt_fb)
        tube = TubeTracker(CFG, model)
        n = 200
        expected = compute_friction_tube_position_radii(np.ones((n, 4), int), np.full((n, 4), CFG.gravity / 4),
                                                        CFG.mu_plan, CFG.mu_lo, 1.0, model["B"], model["P"],
                                                        model["rho"])
        for _ in range(n):
            tube.step()
        self.assertAlmostEqual(tube.radius(tube.alpha), expected[-1], places=9)

    def test_steady_radius(self):
        self.assertAlmostEqual(TubeTracker(CFG).steady_radius, 0.263, places=3)

    def test_stage_radii_are_increasing_and_bounded(self):
        tube = TubeTracker(CFG)
        r = tube.stage_radii(CFG.horizon, CFG.qp_substeps)
        self.assertEqual(r[0], 0.0)
        self.assertTrue(np.all(np.diff(r) > 0))
        self.assertLess(r[-1], tube.steady_radius)

    def test_rejects_a_model_with_another_dt(self):
        with self.assertRaises(ValueError):
            TubeTracker(CFG, design_tube_error_model(0.05))


class TestFeedback(unittest.TestCase):
    def test_lqr_pushes_back_toward_the_nominal(self):
        K = design_tube_error_model(CFG.dt_fb)["K"]
        a = lqr_acceleration([0, 0.10, 0, 0.05], [0, 0, 0, 0], [0, 0], K)
        self.assertAlmostEqual(a[0], 0.0)
        self.assertLess(a[1], 0.0)

    def test_lag_inverse_lands_on_the_requested_velocity(self):
        for yaw in (0.0, 0.7, -2.1):
            v, acc = np.array([0.2, -0.1]), np.array([0.2, 0.1])
            cmd, clipped = velocity_command(v, acc, yaw, CFG)
            c, s = np.cos(yaw), np.sin(yaw)
            rot = np.array([[c, -s], [s, c]])
            v_b, cmd_b = rot.T @ v, rot.T @ cmd
            af, al = math.exp(-CFG.dt_fb / CFG.tau_forward), math.exp(-CFG.dt_fb / CFG.tau_lateral)
            next_b = np.array([af * v_b[0] + (1 - af) * CFG.k_forward * cmd_b[0],
                               al * v_b[1] + (1 - al) * CFG.k_lateral * cmd_b[1]])
            self.assertFalse(clipped)
            np.testing.assert_allclose(rot @ next_b, v + CFG.dt_fb * acc, atol=1e-12)

    def test_b3_conversion(self):
        cmd, _ = velocity_command([0.2, 0.0], [1.0, 0.0], 0.3, replace(CFG, conversion="b3"))
        np.testing.assert_allclose(cmd, [0.2 + CFG.dt_fb, 0.0])

    def test_command_is_clipped(self):
        cmd, clipped = velocity_command([0.0, 0.0], [100.0, 0.0], 0.0, CFG)
        self.assertTrue(clipped)
        self.assertAlmostEqual(np.linalg.norm(cmd), CFG.v_cmd_max)

    def test_sideways_command_has_its_own_limit(self):
        cmd, clipped = velocity_command([0.0, 0.0], [0.0, 100.0], 0.0, CFG)
        self.assertTrue(clipped)
        np.testing.assert_allclose(cmd, [0.0, CFG.v_cmd_max_lateral], atol=1e-12)

    def test_ramp_uses_its_own_gain_and_limit(self):
        flat, _ = velocity_command([0.15, 0.0], [0.0, 0.0], 0.0, CFG)
        ramp, _ = velocity_command([0.15, 0.0], [0.0, 0.0], 0.0, CFG, on_ramp=True)
        self.assertAlmostEqual(flat[0], 0.15 / CFG.k_forward)
        self.assertAlmostEqual(ramp[0], 0.15 / CFG.k_forward_ramp)
        capped, clipped = velocity_command([0.0, 0.0], [100.0, 0.0], 0.0, CFG, on_ramp=True)
        self.assertTrue(clipped)
        self.assertAlmostEqual(np.linalg.norm(capped), CFG.v_cmd_max_ramp)

    def test_yaw_command_wraps_and_clips(self):
        w = yaw_rate_command(np.pi - 0.05, 0.0, -np.pi + 0.05, CFG)
        self.assertLess(w, 0.0)         # shortest way is negative
        self.assertAlmostEqual(w, CFG.k_yaw * float(wrap_to_pi(-0.1)), places=9)
        self.assertEqual(yaw_rate_command(2.0, 0.0, 0.0, CFG), CFG.w_max)


class TestAdapter(unittest.TestCase):
    def test_planner_friction(self):
        self.assertAlmostEqual(planner_friction(0.0, CFG), 0.25)
        self.assertAlmostEqual(planner_friction(math.radians(10), CFG), 0.12)

    def test_course_loads_with_counter_clockwise_obstacles(self):
        g = load_course(COURSE)
        self.assertEqual(len(g.obstacles), 5)
        for o in g.obstacles:
            x, y = o[:, 0], o[:, 1]
            self.assertGreater(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y), 0.0)

    def test_every_opening_fits_the_inflated_circle(self):
        self.assertGreater(1.7, 2 * (CFG.robot_radius + constant_tube_radius(60.0, CFG)))

    def test_clearance_reports_a_violation(self):
        r = straight_reference()
        g = open_geometry([rect(2.0, 3.0, -0.2, 0.2)])
        self.assertLess(clearance(r, g, 0.3), 0.0)
        self.assertGreater(clearance(r, open_geometry(), 0.3), 0.0)


class TestNominalQP(unittest.TestCase):
    def horizon(self, r, t0=2.0):
        s = t0 + np.arange(CFG.horizon + 1) * CFG.dt_qp
        return (np.array([r.state(v) for v in s]), np.array([r.accel(v) for v in s[:-1]]),
                np.array([r.yaw(v) for v in s]))

    def test_returns_a_feasible_reference(self):
        r = straight_reference()
        x, a, yaw = self.horizon(r)
        sol = NominalQP(CFG, (-2, 20, -3, 3), []).solve(x[0], x, a, yaw, CFG.robot_radius, np.full(CFG.horizon + 1, .1))
        self.assertTrue(sol.success)
        self.assertLess(np.abs(sol.x - x).max(), 1e-3)
        self.assertLess(sol.slack.max(), 1e-4)

    def test_recovers_from_an_offset_nominal_and_ends_on_the_reference(self):
        r = straight_reference()
        x, a, yaw = self.horizon(r)
        x0 = x[0] + np.array([0.0, 0.05, 0.0, 0.0])
        sol = NominalQP(CFG, (-2, 20, -3, 3), []).solve(x0, x, a, yaw, CFG.robot_radius, np.full(CFG.horizon + 1, .1))
        self.assertTrue(sol.success)
        np.testing.assert_allclose(sol.x[-1], x[-1], atol=1e-3)

    def test_keeps_clear_of_an_obstacle_the_reference_grazes(self):
        r = straight_reference()
        x, a, yaw = self.horizon(r, t0=4.0)
        # Top face at y = -0.13 with radius 0.10 + tube 0.05: the reference (y = 0) is 0.02 m inside the inflated face.
        # The obstacle is short, so the horizon's end (which must stay on the reference) is clear of it.
        ob = rect(x[9, 0], x[11, 0], -1.5, -0.13)
        radii = np.full(CFG.horizon + 1, 0.05)
        sol = NominalQP(CFG, (-2, 20, -3, 3), [ob]).solve(x[0], x, a, yaw, 0.10, radii)
        self.assertTrue(sol.success)
        self.assertLess(sol.slack.max(), 1e-3)
        beside = (sol.x[:, 0] >= ob[0, 0]) & (sol.x[:, 0] <= ob[1, 0])
        self.assertTrue(beside.any())
        self.assertGreaterEqual(sol.x[beside, 1].min(), -0.13 + 0.15 - 2e-3)

    def test_respects_the_uphill_limit(self):
        r = straight_reference()
        x, a, yaw = self.horizon(r)
        sol = NominalQP(CFG, (-2, 20, -3, 3), [], slope_at=lambda _x: math.radians(10)).solve(
            x[0], x, a, yaw, CFG.robot_radius, np.full(CFG.horizon + 1, .1))
        self.assertTrue(sol.success)
        self.assertLessEqual(sol.a[:, 0].max(), CFG.uphill_limit(math.radians(10)) + 1e-3)


class TestClosedLoop(unittest.TestCase):
    """Mid layer on an ideal plant: first-order velocity lag with the identified tau and gain."""

    def simulate(self, cfg, push=1.962, noise=0.0):
        r = straight_reference()
        mid = TubeMidLayer(make_plan(r), cfg)
        dt = 0.002
        tau, gain = np.array([cfg.tau_forward, cfg.tau_lateral]), np.array([cfg.k_forward, cfg.k_lateral])
        a, c = np.exp(-dt / tau), tau * (1 - np.exp(-dt / tau))
        p, v = np.zeros(2), np.zeros(2)
        tube, worst, reached = mid.tube.steady_radius, 0.0, False
        for i in range(int((r.t_end + 3.0) / dt)):
            t = i * dt
            vm = v + noise * np.sin(2 * np.pi * 1.4 * t)
            cmd, _, done = mid.update(p, vm, 0.0, dt)
            if done:
                reached = True
                break
            worst = max(worst, np.linalg.norm(p - mid.ref.state(max(t - cfg.dt_fb, 0.0))[:2]))
            d = np.array([0.0, -push]) if 4.0 <= t < 5.5 else np.zeros(2)
            p = p + c * v + (dt - c) * gain * cmd[:2] + 0.5 * d * dt * dt
            v = a * v + (1 - a) * gain * cmd[:2] + d * dt
        return reached, worst, tube, mid

    def test_lqr_only_stays_in_the_tube(self):
        reached, worst, tube, mid = self.simulate(replace(CFG, use_qp=False))
        self.assertTrue(reached)
        self.assertLess(worst, tube)
        self.assertEqual(mid.counters["tube_violations"], 0)

    def test_qp_matches_lqr_on_a_valid_reference(self):
        _, worst_lqr, _, _ = self.simulate(replace(CFG, use_qp=False))
        reached, worst, tube, mid = self.simulate(CFG)
        self.assertTrue(reached)
        self.assertLess(worst, tube)
        self.assertAlmostEqual(worst, worst_lqr, delta=0.01)
        self.assertEqual(mid.counters["qp_fallbacks"], 0)
        self.assertLess(mid.max_slack, 1e-3)

    def test_b3_is_worse_on_a_lagging_plant(self):
        _, worst, tube, _ = self.simulate(replace(CFG, use_qp=False))
        _, worst_b3, _, _ = self.simulate(replace(CFG, use_qp=False, conversion="b3"))
        self.assertGreater(worst_b3, worst)


if __name__ == "__main__":
    unittest.main()
