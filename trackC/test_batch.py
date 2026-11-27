"""Tests for the batch runner's job matrix and the per-run metrics, on made-up logs with known answers.

    python -m unittest trackC.test_batch -v
"""
import unittest

import numpy as np

from trackC.batch import CONTROLLERS, build_jobs, parse_gaps
from trackC.course import Course
from trackC.course_spec import load_course_cfg
from trackC.metrics import CIRCLE_RADIUS, PLAN_SAFETY_MARGIN, run_metrics, slip_windows

FLAT = "trackC/courses/flat_v1.yaml"
RAMP = "trackC/courses/ramp_mu03_v1.yaml"
JOB = dict(cone_mu=0.3, tube_margin=0.286)


def fake_log(n=500, x=(1.0, 6.0), y=1.5, yaw=0.0, outcome="goal", hits=(), dt=0.002):
    """A straight run along x with all four feet planted and still, no commanded force."""
    t = np.arange(n) * dt
    pos = np.zeros((n, 3))
    pos[:, 0], pos[:, 1] = np.linspace(*x, n), y
    feet = np.repeat(pos[:, None, :], 4, axis=1)
    return dict(t=t, base_pos=pos, yaw=np.full(n, yaw), summary_outcome=np.array(outcome),
                wall_hit_barrier=np.array(list(hits), dtype=str), feet_contact=np.ones((n, 4), bool),
                feet_pos=feet, feet_vel=np.zeros((n, 4, 3)), feet_grf=np.tile([0, 0, 40.0], (n, 4, 1)),
                mpc_grf=np.zeros((n, 4, 3)))


def signals(n, vt, fn=40.0, r=0.3, cfn=40.0, cr=0.45):
    full = lambda v: np.broadcast_to(np.asarray(v, float), (n,)).copy()  # noqa: E731
    return dict(vt=full(vt), fn=full(fn), r=full(r), cfn=full(cfn), cr=full(cr), c_axis=full(cr))


class Matrix(unittest.TestCase):
    def test_gaps(self):
        self.assertEqual(parse_gaps("0.6:1.1:0.1"), [0.6, 0.7, 0.8, 0.9, 1.0, 1.1])
        self.assertEqual(parse_gaps("0.8,1.0"), [0.8, 1.0])
        self.assertEqual(parse_gaps(None), [None])

    def test_every_controller_gets_the_same_seeds_and_ids_are_unique(self):
        jobs = build_jobs(["nominal", "robust_follower"], ["low", "high"], [0.8, 1.0], range(20))
        self.assertEqual(len(jobs), 2 * 2 * 2 * 20)
        self.assertEqual(len({j["id"] for j in jobs}), len(jobs))
        by_ctrl = {c: sorted((j["variant"], j["gap"], j["seed"]) for j in jobs if j["controller"] == c)
                   for c in ("nominal", "robust_follower")}
        self.assertEqual(by_ctrl["nominal"], by_ctrl["robust_follower"])

    def test_controller_settings_match_the_decisions(self):
        self.assertEqual(CONTROLLERS["nominal"]["cone_mu"], 0.5)
        self.assertEqual(CONTROLLERS["robust_follower"]["cone_mu"], 0.3)          # mu_lo prior (B2, B4)
        self.assertAlmostEqual(CONTROLLERS["robust_follower"]["tube_margin"], 0.263 + 0.0233, places=3)

    def test_unavailable_controller_is_refused(self):
        with self.assertRaises(SystemExit):
            build_jobs(["robust"], [None], [None], [0])


class Outcome(unittest.TestCase):
    def test_outcome_order(self):
        cfg = load_course_cfg(FLAT)
        self.assertEqual(run_metrics(fake_log(), cfg, JOB)["outcome"], "success")
        self.assertEqual(run_metrics(fake_log(outcome="timeout"), cfg, JOB)["outcome"], "timeout")
        self.assertEqual(run_metrics(fake_log(outcome="terminated"), cfg, JOB)["outcome"], "fall")
        m = run_metrics(fake_log(hits=["wall_left_lower"]), cfg, JOB)        # touched a wall but reached the goal
        self.assertEqual(m["outcome"], "collision")
        self.assertTrue(m["reached_goal"])
        self.assertIsNone(m["time_to_goal"])

    def test_ramp_time(self):
        cfg = load_course_cfg(RAMP)
        c = Course(cfg["course"])
        log = fake_log(n=5001, x=(1.0, 11.0))                                  # 10 m in 10 s, 1 m/s
        self.assertAlmostEqual(run_metrics(log, cfg, JOB)["ramp_time"], c.x2 - c.x1, places=2)


class Clearance(unittest.TestCase):
    def test_body_and_circle_clearance_to_a_side_wall(self):
        cfg = load_course_cfg(FLAT)
        y = 3.0 - 0.155 - 0.2                     # body edge 0.2 m from the left wall, facing along x
        m = run_metrics(fake_log(y=y), cfg, JOB)
        self.assertAlmostEqual(m["body_clearance"], 0.2, places=6)
        slack = (3.0 - y) - (CIRCLE_RADIUS + PLAN_SAFETY_MARGIN)
        self.assertAlmostEqual(m["circle_slack"], slack, places=6)
        self.assertAlmostEqual(m["tube_used"], 1 - slack / JOB["tube_margin"], places=6)

    def test_obstacle_counts(self):
        cfg = load_course_cfg(FLAT)
        cfg["obstacles"] = [{"id": "post", "center": [5.0, 2.5], "size": [0.2, 0.2], "height": 0.4}]
        m = run_metrics(fake_log(x=(2.0, 6.0), y=1.5), cfg, JOB)               # post edge at y = 2.4
        self.assertAlmostEqual(m["body_clearance"], 2.4 - (1.5 + 0.155), places=6)


class Slips(unittest.TestCase):
    def test_landing_slide_is_not_a_slip(self):
        n = 100
        vt = 0.9 * np.exp(-np.arange(n) / 10)                                  # coasting to a stop
        self.assertEqual(slip_windows(signals(n, vt, r=0.3, cr=0.2), np.ones(n, bool)), [])

    def test_force_driven_slip_is_found_with_the_measured_ratio(self):
        n = 100
        vt = np.r_[np.zeros(30), np.full(70, 0.12)]                            # at rest, then pushed into a slide
        found = slip_windows(signals(n, vt, r=0.3, cr=0.45), np.ones(n, bool))
        self.assertEqual(len(found), 1)
        self.assertEqual(found[0][0], 30)
        self.assertAlmostEqual(found[0][1], 0.3)

    def test_slide_from_landing_that_the_controller_keeps_pushing_is_a_slip(self):
        n = 100
        vt = np.full(n, 0.15)                                                  # never comes to rest
        self.assertEqual(len(slip_windows(signals(n, vt, r=0.3, cr=0.42), np.ones(n, bool))), 1)

    def test_sliding_without_overload_or_without_commanded_stance_is_not_a_slip(self):
        n = 100
        vt = np.full(n, 0.15)
        self.assertEqual(slip_windows(signals(n, vt, r=0.3, cr=0.29), np.ones(n, bool)), [])  # ground keeps up
        self.assertEqual(slip_windows(signals(n, vt, r=0.3, cr=0.45, cfn=3.0), np.ones(n, bool)), [])  # no stance
        self.assertEqual(slip_windows(signals(n, vt, r=0.3, cr=0.45, fn=2.0), np.ones(n, bool)), [])  # liftoff

    def test_one_slip_per_stance(self):
        n = 200
        contact = np.r_[np.ones(90, bool), np.zeros(20, bool), np.ones(90, bool)]
        found = slip_windows(signals(n, np.full(n, 0.12)), contact)
        self.assertEqual([k for k, _ in found], [0, 110])


if __name__ == "__main__":
    unittest.main()
