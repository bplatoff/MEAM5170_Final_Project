"""Geometry and constraint-level tests; does not require a full NLP solve."""
import unittest

import casadi as ca
import numpy as np

from planning.mock_environment import rectangle
from planning.robot_footprint import (
    add_three_circle_constraints,
    three_circle_centers,
    three_circle_centers_symbolic,
    three_circle_parameters,
)


class TestThreeCircleFootprint(unittest.TestCase):
    def test_nominal_dimensions(self):
        offsets, radius = three_circle_parameters(0.7, 0.31, 0.05)
        np.testing.assert_allclose(offsets, [-0.7 / 3, 0, 0.7 / 3])
        self.assertAlmostEqual(radius, np.hypot(0.7 / 6, 0.31 / 2) + 0.05)

    def test_centers_rotate_with_yaw(self):
        centers_0 = three_circle_centers([2.0, 1.0], 0.0, 0.7)
        np.testing.assert_allclose(centers_0[:, 1], [1.0] * 3)
        np.testing.assert_allclose(centers_0[:, 0],
                                   [2 - 0.7 / 3, 2, 2 + 0.7 / 3])
        centers_90 = three_circle_centers([2.0, 1.0], np.pi / 2, 0.7)
        np.testing.assert_allclose(centers_90[:, 0], [2.0] * 3, atol=1e-12)
        np.testing.assert_allclose(centers_90[:, 1],
                                   [1 - 0.7 / 3, 1, 1 + 0.7 / 3])

    def test_three_circles_cover_entire_body_rectangle(self):
        length, width = 0.7, 0.31
        _, radius = three_circle_parameters(length, width)
        for yaw in [0.0, np.pi / 6, np.pi / 2]:
            centers = three_circle_centers([2.0, 1.0], yaw, length)
            c, s = np.cos(yaw), np.sin(yaw)
            for x in np.linspace(-length / 2, length / 2, 41):
                for y in np.linspace(-width / 2, width / 2, 21):
                    point = np.array([2 + c * x - s * y, 1 + s * x + c * y])
                    self.assertLessEqual(
                        np.min(np.linalg.norm(centers - point, axis=1)),
                        radius + 1e-12,
                    )

    def test_casadi_matches_numeric_centers(self):
        x = ca.MX.sym("x", 2)
        yaw = ca.MX.sym("yaw")
        expression = three_circle_centers_symbolic(x, yaw, 0.7)
        function = ca.Function("centers", [x, yaw], [expression])
        numeric = np.asarray(function([2.0, 1.0], 0.41))
        np.testing.assert_allclose(numeric, three_circle_centers([2, 1], 0.41, 0.7))

    def test_wall_constraints_are_yaw_dependent(self):
        # At x=.3 the +x/-x oriented rear circle crosses the left wall;
        # when rotated by 90 degrees, every circle stays inside the wall.
        _, fun = self._constraint_function([])
        facing_x = np.asarray(fun([0.3, 1.5], 0.0)).flatten()
        facing_y = np.asarray(fun([0.3, 1.5], np.pi / 2)).flatten()
        self.assertLess(np.min(facing_x), 0)
        self.assertGreater(np.min(facing_y), 0)

    def test_polygon_constraints_reject_collision(self):
        obstacle = rectangle(1.0, 2.0, 1.0, 2.0)
        _, fun = self._constraint_function([obstacle])
        colliding = np.asarray(fun([1.5, 1.5], 0.0)).flatten()
        safely_outside = np.asarray(fun([3.0, 1.5], 0.0)).flatten()
        self.assertLess(np.min(colliding), 0)
        self.assertGreater(np.min(safely_outside), 0)

    def test_constraints_have_finite_gradients(self):
        obstacle = rectangle(1.0, 2.0, 1.0, 2.0)
        pos, yaw = ca.MX.sym("p", 2), ca.MX.sym("theta")
        g, lo, hi = [], [], []
        add_three_circle_constraints(pos, yaw, [obstacle], (0, 4, 0, 3),
                                     0.7, 0.31, 0.05, 100, g, lo, hi)
        jac = ca.jacobian(ca.vertcat(*g), ca.vertcat(pos, yaw))
        fn = ca.Function("footprint_jac", [pos, yaw], [jac])
        self.assertTrue(np.all(np.isfinite(np.asarray(fn([3.0, 1.5], 0.1)))))

    @staticmethod
    def _constraint_function(obstacles):
        pos, yaw = ca.MX.sym("p", 2), ca.MX.sym("theta")
        g, lo, hi = [], [], []
        add_three_circle_constraints(pos, yaw, obstacles, (0, 4, 0, 3),
                                     0.7, 0.31, 0.05, 100, g, lo, hi)
        expr = ca.vertcat(*g)
        return expr, ca.Function("g", [pos, yaw], [expr])


if __name__ == "__main__":
    unittest.main()
