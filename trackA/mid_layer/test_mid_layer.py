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

CFG = MidLayerConfig(footprint="single_circle", qp_start="carried")     # the committed baseline
THREE = MidLayerConfig(footprint="three_circle", qp_start="carried")
MEASURED = MidLayerConfig(footprint="three_circle", qp_start="measured")
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


def make_plan(reference, geometry=None, cfg=CFG):
    geometry = geometry if geometry is not None else open_geometry()
    return Plan(reference, geometry, constant_tube_radius(reference.t_end + 3.0, cfg), cfg.robot_radius,
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


class TestThreeCircles(unittest.TestCase):
    def horizon(self, r, t0=2.0):
        s = t0 + np.arange(THREE.horizon + 1) * THREE.dt_qp
        return (np.array([r.state(v) for v in s]), np.array([r.accel(v) for v in s[:-1]]),
                np.array([r.yaw(v) for v in s]))

    def test_footprint_values(self):
        np.testing.assert_allclose(THREE.circle_offsets, [-0.7 / 3, 0.0, 0.7 / 3])
        self.assertAlmostEqual(THREE.robot_radius, math.hypot(0.7 / 6, 0.31 / 2) + 0.05)
        self.assertAlmostEqual(THREE.heading_margin, 0.7 / 3 * 0.10)
        self.assertAlmostEqual(CFG.robot_radius, 0.4828, places=3)
        np.testing.assert_allclose(CFG.circle_offsets, [0.0])
        self.assertEqual(CFG.heading_margin, 0.0)

    def test_circles_cover_the_body_at_any_yaw(self):
        r = math.hypot(0.7 / 6, 0.31 / 2)
        for yaw in np.linspace(-np.pi, np.pi, 13):
            h, n = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])
            for a in np.linspace(-0.35, 0.35, 29):
                for b in np.linspace(-0.155, 0.155, 11):
                    point = a * h + b * n
                    self.assertLessEqual(min(np.linalg.norm(point - s * h) for s in THREE.circle_offsets), r + 1e-9)

    def test_clearance_checks_the_front_circle(self):
        r = straight_reference()
        ob = rect(r.x[-1, 0] + 0.30, r.x[-1, 0] + 0.50, -0.5, 0.5)      # just ahead of the final center position
        g = open_geometry([ob], bounds=(-2.0, 30.0, -3.0, 3.0))
        self.assertGreater(clearance(r, g, 0.1), 0.0)
        self.assertLess(clearance(r, g, 0.1, circle_offsets=THREE.circle_offsets), 0.0)

    def test_qp_keeps_each_circle_clear_with_its_heading_margin(self):
        h, N = 0.4, 30
        u = np.zeros((N, 2))
        x = np.zeros((N + 1, 4))
        x[:, 0], x[:, 2] = 0.2 * h * np.arange(N + 1), 0.2
        yaw = np.full(N + 1, np.pi / 2)                     # moving along +x while facing +y, so the circles spread in y
        ref = PlannerReference(h * np.arange(N + 1), x, u, yaw)
        xr, a, yr = self.horizon(ref)
        ob = rect(xr[8, 0], xr[12, 0], -1.0, -0.70)         # front/rear circles need y >= -0.70 + R + 0.233
        radii = np.full(THREE.horizon + 1, 0.05)
        qp = NominalQP(THREE, (-2, 20, -3, 3), [ob], offsets=THREE.circle_offsets)
        sol = qp.solve(xr[0], xr, a, yr, THREE.robot_radius, radii)
        self.assertTrue(sol.success)
        self.assertLess(sol.slack.max(), 1e-3)
        beside = (sol.x[:, 0] >= ob[0, 0]) & (sol.x[:, 0] <= ob[1, 0])
        need = -0.70 + THREE.robot_radius + THREE.heading_margin + 0.05 + 0.7 / 3
        self.assertGreaterEqual(sol.x[beside, 1].min(), need - 2e-3)

    def test_wall_limit_uses_the_tightest_circle(self):
        r = straight_reference()
        x, a, yaw = self.horizon(r)
        yaw = np.full_like(yaw, np.pi / 2)
        qp = NominalQP(THREE, (-2, 20, -0.45, 3), [], offsets=THREE.circle_offsets)
        sol = qp.solve(x[0], x, a, yaw, THREE.robot_radius, np.full(THREE.horizon + 1, 0.02))
        self.assertTrue(sol.success)
        need = -0.45 + THREE.robot_radius + THREE.heading_margin + 0.02 + 0.7 / 3
        self.assertGreater(need, 0.0)                       # the reference (y = 0) violates it, so slack must appear
        self.assertAlmostEqual(sol.slack.max(), need, delta=2e-3)


class TestMeasuredQP(unittest.TestCase):
    def setUp(self):
        self.ref = straight_reference(speed=0.3, T=20.0)
        self.tube = TubeTracker(MEASURED)
        self.radii = self.tube.stage_radii(MEASURED.horizon, MEASURED.qp_substeps, alpha0=0.0)
        s = 6.0 + np.arange(MEASURED.horizon + 1) * MEASURED.dt_qp
        self.x = np.array([self.ref.state(v) for v in s])
        self.a = np.array([self.ref.accel(v) for v in s[:-1]])
        self.yaw = np.zeros(MEASURED.horizon + 1)

    def qp(self, obstacles=()):
        return NominalQP(MEASURED, (-2, 20, -3, 3), list(obstacles), offsets=MEASURED.circle_offsets)

    def side_wall(self, gap):
        """An obstacle whose top face is gap beyond the body margin of the reference line."""
        top = -(MEASURED.robot_radius + MEASURED.heading_margin + gap)
        return rect(self.x[0, 0] - 2.0, self.x[-1, 0] + 2.0, top - 0.5, top)

    def test_slope_friction_couples_uphill_and_sideways(self):
        slope, g = math.radians(10), MEASURED.gravity
        offset = self.x[0] - np.array([0.3, 0.3, 0.0, 0.0])        # behind and beside: wants both directions
        on_slope = NominalQP(MEASURED, (-2, 20, -3, 3), [], offsets=MEASURED.circle_offsets,
                             slope_at=lambda _x: slope).solve(offset, self.x, self.a, self.yaw, MEASURED.robot_radius,
                                                              self.radii)
        self.assertTrue(on_slope.success)
        demand = np.hypot(on_slope.a[:, 0] + g * math.sin(slope), on_slope.a[:, 1])
        self.assertLessEqual(demand.max(), MEASURED.mu_lo * g * math.cos(slope) / math.cos(math.pi / 16) + 1e-3)
        self.assertLessEqual(on_slope.a[:, 0].max(), MEASURED.uphill_limit(slope) + 1e-3)
        self.assertGreater(on_slope.a[0, 1], 1.0)                   # still corrects sideways
        flat = self.qp().solve(offset, self.x, self.a, self.yaw, MEASURED.robot_radius, self.radii)
        self.assertGreater(flat.a[0, 0], on_slope.a[0, 0] + 0.5)   # flat ground is unchanged and less limited
        self.assertGreater(flat.a[0, 1], on_slope.a[0, 1] + 0.5)

    def test_tube_restarts_from_zero(self):
        np.testing.assert_allclose(self.radii[[1, 5, 10, 20]], [0.032, 0.127, 0.193, 0.244], atol=1e-3)
        self.tube.alpha = self.tube.steady_alpha
        np.testing.assert_allclose(self.tube.stage_radii(20, 5, alpha0=0.0), self.radii)
        self.assertGreater(self.tube.stage_radii(20, 5)[1], 0.25)

    def test_unconstrained_gain_matches_the_lqr(self):
        np.testing.assert_allclose(self.qp().K_qp, design_tube_error_model(MEASURED.dt_fb)["K"], atol=1e-5)

    def test_on_reference_it_returns_the_reference_with_no_slack(self):
        sol = self.qp().solve(self.x[0], self.x, self.a, self.yaw, MEASURED.robot_radius, self.radii)
        self.assertTrue(sol.success)
        self.assertLess(np.abs(sol.x - self.x).max(), 1e-3)
        self.assertLess(max(sol.slack_tube.max(), sol.slack_body.max(), sol.slack_speed.max()), 1e-4)

    def test_it_plans_back_to_the_reference_from_a_measured_offset(self):
        x0 = self.x[0] + np.array([0.0, 0.20, 0.0, 0.05])
        sol = self.qp().solve(x0, self.x, self.a, self.yaw, MEASURED.robot_radius, self.radii)
        self.assertTrue(sol.success)
        np.testing.assert_allclose(sol.x[0], x0)
        self.assertLess(sol.a[0, 1], -0.5)
        self.assertLess(abs(sol.x[-1, 1] - self.x[-1, 1]), 0.02)

    def test_first_step_margin_is_the_growing_tube(self):
        """Clear of the 3.2 cm first-step tube but inside the 26 cm steady tube: no violation."""
        ob = self.side_wall(0.10)
        x0 = self.x[0].copy()
        sol = self.qp([ob]).solve(x0, self.x, self.a, self.yaw, MEASURED.robot_radius, self.radii)
        self.assertTrue(sol.success)
        self.assertLess(sol.slack[0], 1e-3)

    def test_soft_matches_hard_when_hard_is_feasible(self):
        ob = self.side_wall(0.12)
        x0 = self.x[0] + np.array([0.0, 0.02, 0.0, 0.0])
        soft = self.qp([ob]).solve(x0, self.x, self.a, self.yaw, MEASURED.robot_radius, self.radii)
        hard = self.qp([ob]).solve(x0, self.x, self.a, self.yaw, MEASURED.robot_radius, self.radii, hard=True)
        self.assertTrue(hard.success)
        self.assertTrue(soft.success)
        self.assertLess(soft.slack.max(), 1e-3)
        self.assertLess(np.abs(soft.x - hard.x).max(), 1e-3)

    def test_it_solves_from_a_shove_where_the_hard_problem_cannot(self):
        ob = self.side_wall(0.05)
        x0 = self.x[0] + np.array([0.0, -0.05, 0.0, -0.5])    # at the body margin, moving toward the obstacle
        hard = self.qp([ob]).solve(x0, self.x, self.a, self.yaw, MEASURED.robot_radius, self.radii, hard=True)
        soft = self.qp([ob]).solve(x0, self.x, self.a, self.yaw, MEASURED.robot_radius, self.radii)
        self.assertFalse(hard.success)
        self.assertTrue(soft.success)
        self.assertGreater(soft.slack.max(), 1e-3)
        self.assertGreater(soft.a[0, 1], 2.0)               # it brakes away from the obstacle

    def test_it_uses_tube_slack_before_body_slack(self):
        ob = self.side_wall(0.0)
        x0 = self.x[0] + np.array([0.0, 0.0, 0.0, -0.1])
        sol = self.qp([ob]).solve(x0, self.x, self.a, self.yaw, MEASURED.robot_radius, self.radii)
        self.assertTrue(sol.success)
        self.assertGreater(sol.slack_tube.max(), 1e-3)
        used_body = sol.slack_body > 1e-3
        np.testing.assert_array_less(self.radii[1:][used_body] - 2e-3, sol.slack_tube[used_body])

    def test_it_solves_from_an_extreme_state(self):
        ob = self.side_wall(0.0)
        x0 = self.x[0] + np.array([0.0, -0.40, -0.8, -1.2])   # inside the obstacle, faster than the speed limit
        sol = self.qp([ob]).solve(x0, self.x, self.a, self.yaw, MEASURED.robot_radius, self.radii)
        self.assertTrue(sol.success)
        self.assertGreater(sol.slack_body.max(), 0.01)
        self.assertGreater(sol.slack_speed.max(), 0.01)
        polygon_corner = MEASURED.accel_limit / math.cos(math.pi / 16)      # the 16-sided limit is outside the circle
        self.assertLessEqual(np.linalg.norm(sol.a, axis=1).max(), polygon_corner + 1e-3)


class TestClosedLoop(unittest.TestCase):
    """Mid layer on an ideal plant: first-order velocity lag with the identified tau and gain."""

    def simulate(self, cfg, push=1.962, noise=0.0):
        r = straight_reference()
        mid = TubeMidLayer(make_plan(r, cfg=cfg), cfg)
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

    def test_measured_start_tracks_and_resets_the_nominal(self):
        reached, worst, tube, mid = self.simulate(MEASURED)
        self.assertTrue(reached)
        self.assertLess(worst, tube)
        self.assertEqual(mid.counters["qp_fallbacks"], 0)
        self.assertEqual(mid.counters["body_slack_solves"], 0)
        self.assertLess(mid.tube.alpha, 5 * mid.tube.r_w + 1e-9)

    def test_three_circle_carried_qp_matches_lqr(self):
        _, worst_lqr, _, _ = self.simulate(replace(THREE, use_qp=False))
        reached, worst, _, mid = self.simulate(THREE)
        self.assertTrue(reached)
        self.assertAlmostEqual(worst, worst_lqr, delta=0.01)
        self.assertEqual(mid.counters["qp_fallbacks"], 0)

    def test_b3_is_worse_on_a_lagging_plant(self):
        _, worst, tube, _ = self.simulate(replace(CFG, use_qp=False))
        _, worst_b3, _, _ = self.simulate(replace(CFG, use_qp=False, conversion="b3"))
        self.assertGreater(worst_b3, worst)


if __name__ == "__main__":
    unittest.main()
