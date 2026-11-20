"""Unit tests for the reference conversion layer. No simulator needed.

Run from the repo root:  python -m unittest trackA.reference.test_path_follower -v

The constructor and wrap_to_pi tests pass now. The `update` tests describe the expected behavior and fail
(NotImplementedError) until PathFollower.update is written.
"""
import unittest

import numpy as np

from trackA.reference.path_follower import PathFollower, wrap_to_pi

DT = 0.002  # simulation step [s]


def settle(follower, pos_xy, yaw, seconds=3.0, dt=DT):
    """Call update repeatedly at a fixed pose so the acceleration limit has converged; return the last output."""
    out = None
    for _ in range(int(seconds / dt)):
        out = follower.update(np.asarray(pos_xy, float), yaw, dt)
    return out


class TestConstructor(unittest.TestCase):
    def test_segment_geometry(self):
        f = PathFollower([[0, 0], [3, 0], [3, 4]])
        self.assertEqual(f.n_segments, 2)
        np.testing.assert_allclose(f.seg_len, [3, 4])
        np.testing.assert_allclose(f.seg_dir, [[1, 0], [0, 1]])
        np.testing.assert_allclose(f.seg_heading, [0, np.pi / 2])
        np.testing.assert_allclose(f.goal, [3, 4])

    def test_drops_consecutive_duplicates(self):
        f = PathFollower([[0, 0], [0, 0], [1, 0], [1, 0], [2, 0]])
        self.assertEqual(f.n_segments, 2)

    def test_rejects_bad_input(self):
        for bad in ([[0, 0]], [[1, 1], [1, 1]], [0, 1, 2], [[0, 0, 0], [1, 0, 0]]):
            with self.assertRaises(ValueError):
                PathFollower(bad)

    def test_reset_state(self):
        f = PathFollower([[0, 0], [1, 0]])
        self.assertEqual(f.seg_idx, 0)
        self.assertFalse(f.done)
        np.testing.assert_array_equal(f.prev_lin_vel, np.zeros(3))


class TestWrapToPi(unittest.TestCase):
    def test_wrap(self):
        self.assertAlmostEqual(float(wrap_to_pi(0.0)), 0.0)
        self.assertAlmostEqual(float(wrap_to_pi(3 * np.pi / 2)), -np.pi / 2)
        self.assertAlmostEqual(float(wrap_to_pi(-3 * np.pi / 2)), np.pi / 2)
        self.assertAlmostEqual(float(wrap_to_pi(2 * np.pi + 0.1)), 0.1)


class TestUpdate(unittest.TestCase):
    def setUp(self):
        self.f = PathFollower([[0, 0], [10, 0]], v_ref=0.4, k_p=1.0, v_corr_max=0.2, k_psi=1.5, w_max=0.6,
                              goal_tol=0.1, slow_radius=0.5, a_max=0.5)

    def test_output_format(self):
        lin, ang, done = self.f.update(np.array([0.0, 0.0]), 0.0, DT)
        self.assertEqual(np.shape(lin), (3,))
        self.assertEqual(np.shape(ang), (3,))
        self.assertEqual(lin[2], 0.0)
        self.assertEqual(ang[0], 0.0)
        self.assertEqual(ang[1], 0.0)
        self.assertIsInstance(done, (bool, np.bool_))

    def test_on_path_moves_along_it_at_v_ref(self):
        lin, ang, done = settle(self.f, [1.0, 0.0], 0.0)
        np.testing.assert_allclose(lin, [0.4, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(ang, [0.0, 0.0, 0.0], atol=1e-6)
        self.assertFalse(done)

    def test_offset_steers_back_and_speed_is_bounded(self):
        lin, _, _ = settle(self.f, [1.0, 0.3], 0.0)   # 0.3 m to the left of the path
        self.assertGreater(lin[0], 0.0)                # still makes progress
        self.assertLess(lin[1], 0.0)                   # heads back toward y = 0
        self.assertLessEqual(np.linalg.norm(lin), np.hypot(0.4, 0.2) + 1e-9)
        self.assertAlmostEqual(self.f.cross_track_error, 0.3, places=6)   # left of the path -> positive
        settle(self.f, [1.0, -0.3], 0.0, seconds=0.01)
        self.assertAlmostEqual(self.f.cross_track_error, -0.3, places=6)

    def test_correction_is_capped(self):
        lin_far, _, _ = settle(self.f, [1.0, 1.0], 0.0)
        self.assertAlmostEqual(lin_far[1], -0.2, places=6)   # k_p * 1.0 = 1.0 would exceed v_corr_max

    def test_yaw_rate_turns_toward_path_heading_and_saturates(self):
        _, ang, _ = settle(self.f, [1.0, 0.0], -0.1)
        self.assertAlmostEqual(ang[2], 1.5 * 0.1, places=6)   # P control, unsaturated
        _, ang, _ = settle(self.f, [1.0, 0.0], -np.pi / 2)
        self.assertAlmostEqual(ang[2], 0.6, places=6)         # saturated at w_max

    def test_yaw_error_is_wrapped(self):
        f = PathFollower([[0, 0], [-10, 0.001]])   # heading just under +pi
        _, ang, _ = settle(f, [-1.0, 0.0], -np.pi + 0.01)
        self.assertLess(abs(ang[2]), 1.5 * 0.05)            # small error across the +-pi seam, not ~2*pi

    def test_acceleration_limit_from_rest(self):
        lin, _, _ = self.f.update(np.array([1.0, 0.0]), 0.0, DT)
        self.assertLessEqual(np.linalg.norm(lin), 0.5 * DT + 1e-12)

    def test_switches_segment_at_corner(self):
        f = PathFollower([[0, 0], [1, 0], [1, 2]])
        lin, _, _ = settle(f, [1.05, 0.0], np.pi / 2)   # just past the end of segment 0
        self.assertEqual(f.seg_idx, 1)
        self.assertGreater(lin[1], 0.0)                  # now heading +y
        self.assertLess(abs(lin[0]), 0.2 + 1e-9)         # only correction in x

    def test_segment_index_never_goes_back(self):
        f = PathFollower([[0, 0], [1, 0], [1, 2]])
        settle(f, [1.05, 0.5], np.pi / 2, seconds=0.1)
        self.assertEqual(f.seg_idx, 1)
        settle(f, [0.5, 0.0], np.pi / 2, seconds=0.1)    # back near segment 0
        self.assertEqual(f.seg_idx, 1)

    def test_slows_near_goal(self):
        lin, _, done = settle(self.f, [9.75, 0.0], 0.0)  # 0.25 m from the goal, inside slow_radius
        self.assertFalse(done)
        self.assertGreater(lin[0], 0.0)
        self.assertLess(lin[0], 0.4)

    def test_done_at_goal_and_latched(self):
        lin, ang, done = self.f.update(np.array([9.95, 0.0]), 0.0, DT)
        self.assertTrue(done)
        np.testing.assert_array_equal(lin, np.zeros(3))
        np.testing.assert_array_equal(ang, np.zeros(3))
        lin, ang, done = self.f.update(np.array([9.5, 0.0]), 0.0, DT)   # drifted back out: stays done
        self.assertTrue(done)
        np.testing.assert_array_equal(lin, np.zeros(3))

    def test_reset_clears_done(self):
        self.f.update(np.array([9.95, 0.0]), 0.0, DT)
        self.f.reset()
        self.assertFalse(self.f.done)
        self.assertEqual(self.f.seg_idx, 0)


if __name__ == "__main__":
    unittest.main()
