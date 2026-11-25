"""Tests for the course loader, friction rules, mid-ramp gap, planner converter and prior friction map.

    python -m unittest trackC.test_course_spec -v
"""
import unittest

import numpy as np

from trackC.course_spec import (FRICTION_CLASSES, FRICTION_SET, CourseSpecError, load_course_cfg,
                                resolve_friction, scene_name)
from trackC.friction_map import PriorFrictionMap, query
from trackC.planner_env import planner_problem
from trackC.terrain_gen import build

MIXED = "trackC/courses/mixed_v1.yaml"
COURSES = ["trackC/courses/flat_v1.yaml", "trackC/courses/ramp_v1.yaml", "trackC/courses/debug_v03.yaml", MIXED]


class Friction(unittest.TestCase):
    def test_every_course_and_seed_stays_in_the_set(self):
        for path in COURSES:
            cfg = load_course_cfg(path)
            for seed in [None] + list(range(20)):
                for sid, mu in resolve_friction(cfg, seed).items():
                    self.assertIn(mu, FRICTION_SET, f"{path} seed {seed} {sid}")
                    self.assertGreaterEqual(mu, 0.3)

    def test_unseeded_uses_class_nominal_and_base_is_never_drawn(self):
        cfg = load_course_cfg(MIXED)
        mu = resolve_friction(cfg)
        for r in cfg["regions"]:
            self.assertEqual(mu[r["id"]], FRICTION_CLASSES[r["class"]][0])
        for seed in range(20):
            self.assertEqual(resolve_friction(cfg, seed)["base"], 0.5)

    def test_same_seed_same_course_and_seeds_differ(self):
        cfg = load_course_cfg(MIXED)
        self.assertEqual(resolve_friction(cfg, 7), resolve_friction(cfg, 7))
        draws = {tuple(sorted(resolve_friction(cfg, s).items())) for s in range(20)}
        self.assertGreater(len(draws), 1)

    def test_high_draws_only_07_or_08(self):
        cfg = load_course_cfg(MIXED)
        seen = {resolve_friction(cfg, s)["upper_high"] for s in range(50)}
        self.assertEqual(seen, {0.7, 0.8})

    def test_old_schema_and_bad_values_are_rejected(self):
        old = load_course_cfg("trackC/courses/flat_v1.yaml")
        old["friction_classes"] = {"low": 0.2}
        with self.assertRaises(CourseSpecError):
            load_course_cfg(old)
        bad = load_course_cfg("trackC/courses/flat_v1.yaml")
        bad["regions"] = [{"id": "ice", "mu": 0.2, "polygon": [[1, 1], [2, 1], [2, 2]]}]
        with self.assertRaises(CourseSpecError):
            load_course_cfg(bad)


class MidRampGap(unittest.TestCase):
    def test_barrier_leaves_the_gap_at_the_left_wall(self):
        for gap in (0.6, 0.8, 1.1):
            cfg = load_course_cfg(MIXED, gap=gap)
            b = next(o for o in cfg["obstacles"] if o["id"] == "barrier")
            y_lo, y_hi = b["center"][1] - b["size"][1] / 2, b["center"][1] + b["size"][1] / 2
            self.assertAlmostEqual(y_lo, 0.0)                        # attached to the right wall
            self.assertAlmostEqual(cfg["course"]["width"] - y_hi, gap)

    def test_variant_sets_the_patch_class(self):
        for variant, values in (("low", {0.3}), ("high", {0.7, 0.8})):
            cfg = load_course_cfg(MIXED, variant=variant)
            seen = {resolve_friction(cfg, s)["gap_patch"] for s in range(30)}
            self.assertEqual(seen, values)

    def test_loading_twice_does_not_add_a_second_barrier(self):
        cfg = load_course_cfg(load_course_cfg(MIXED))
        self.assertEqual(sum(o["id"] == "barrier" for o in cfg["obstacles"]), 1)

    def test_scene_names(self):
        self.assertEqual(scene_name(load_course_cfg(MIXED)), "mixed_v1")
        cfg = load_course_cfg(MIXED, gap=0.8, variant="high")
        self.assertEqual(scene_name(cfg, seed=3, overridden=True), "mixed_v1_g0.80_high_s3")
        with self.assertRaises(CourseSpecError):
            load_course_cfg("trackC/courses/flat_v1.yaml", gap=0.8)  # no mid-ramp gap to override
        with self.assertRaises(CourseSpecError):
            load_course_cfg(MIXED, gap=3.5)

    def test_every_swept_variant_builds(self):
        for gap in np.round(np.arange(0.6, 1.11, 0.1), 2):
            for variant in ("low", "high"):
                _, info = build(MIXED, seed=1, gap=float(gap), variant=variant)
                self.assertEqual(info["name"], f"mixed_v1_g{gap:.2f}_{variant}_s1")


class Planner(unittest.TestCase):
    def test_converter_matches_the_course(self):
        p = planner_problem(MIXED, gap=0.8)
        env = p.environment
        self.assertEqual(env.bounds, (0.0, p.course.x3, 0.0, 3.0))
        self.assertEqual(len(env.obstacles), len(p.cfg["obstacles"]))           # 4 posts + the barrier
        self.assertEqual(env.friction_mu, 0.5)
        np.testing.assert_allclose(p.start_state, [0.8, 1.5, 0, 0])
        np.testing.assert_allclose(p.goal_state, [11.1, 1.5, 0, 0])
        barrier = env.obstacles[-1]
        self.assertAlmostEqual(barrier[:, 1].max(), 3.0 - 0.8)                   # gap edge
        for poly in env.obstacles:                                               # counter-clockwise
            x, y = poly[:, 0], poly[:, 1]
            self.assertGreater(0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y), 0)

    def test_start_override(self):
        p = planner_problem(MIXED, start=(1.0, 1.2))
        np.testing.assert_allclose(p.start_state, [1.0, 1.2, 0, 0])


class Prior(unittest.TestCase):
    def test_prior_values(self):
        self.assertEqual(query(3.0, 1.5), (0.5, 0.3))
        mu_plan, mu_lo = PriorFrictionMap().query_many(np.zeros((7, 2)))
        np.testing.assert_array_equal(mu_plan, np.full(7, 0.5))
        np.testing.assert_array_equal(mu_lo, np.full(7, 0.3))
        with self.assertRaises(ValueError):
            PriorFrictionMap(mu_plan=0.3, mu_lo=0.5)


if __name__ == "__main__":
    unittest.main()
