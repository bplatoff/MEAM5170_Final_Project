
import unittest
import numpy as np
from planning.robust_control.friction_tube import compute_friction_tube
from planning.robust_control.friction_tube import (
    friction_acceleration_bound,
    p_norm_disturbance_bound,
    propagate_tube,
    p_norm_to_position_radius,
    compute_friction_tube_position_radii,
)

from planning.robust_control.friction_tube import (
    friction_acceleration_bound,
    p_norm_disturbance_bound,
    propagate_tube,
)
from planning.robust_control.tube_error_model import (
    design_tube_error_model,
)


class TestFrictionTube(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.model = design_tube_error_model()
        cls.B = cls.model["B"]
        cls.P = cls.model["P"]
        cls.rho = cls.model["rho"]

    # -------------------------------------------------------
    # Friction acceleration bounds
    # -------------------------------------------------------

    def test_contact_mask(self):
        """Only feet in stance contribute to disturbance."""
        stance = np.array([[1, 1, 0, 0]])
        forces = np.array([[100, 100, 50, 50]])

        delta_a = friction_acceleration_bound(
            stance, forces, 0.5, 0.3, 20.0
        )

        np.testing.assert_allclose(delta_a, [2.0])
        self.assertEqual(delta_a.shape, (1,))

    def test_zero_disturbance(self):
        """No contact or no friction gap produces zero."""
        forces = np.full((3, 4), 50.0)

        no_contact = np.zeros((3, 4))
        all_contact = np.ones((3, 4))

        cases = [
            (no_contact, 0.5, 0.2),
            (all_contact, 0.5, 0.5),
            (all_contact, 0.3, 0.5),
        ]

        for stance, mu_plan, mu_lo in cases:
            with self.subTest(mu_plan=mu_plan, mu_lo=mu_lo):
                result = friction_acceleration_bound(
                    stance, forces, mu_plan, mu_lo, 20.0
                )
                np.testing.assert_allclose(result, np.zeros(3))

    def test_mass_scaling(self):
        """Doubling mass halves the acceleration bound."""
        stance = np.ones((2, 4))
        forces = np.full((2, 4), 50.0)

        a1 = friction_acceleration_bound(
            stance, forces, 0.5, 0.3, 20.0
        )
        a2 = friction_acceleration_bound(
            stance, forces, 0.5, 0.3, 40.0
        )

        np.testing.assert_allclose(a2, 0.5 * a1)

    def test_variable_friction_across_horizon(self):
        """Per-foot, per-step friction values are supported."""
        stance = np.array([
            [1, 1, 1, 1],
            [1, 1, 0, 0],
            [1, 1, 1, 1],
        ])

        forces = np.array([
            [50, 50, 50, 50],
            [100, 100, 0, 0],
            [50, 50, 50, 50],
        ])

        mu_plan = np.full((3, 4), 0.5)
        mu_lo = np.array([
            [0.5, 0.5, 0.5, 0.5],
            [0.3, 0.3, 0.5, 0.5],
            [0.5, 0.5, 0.5, 0.5],
        ])

        delta_a = friction_acceleration_bound(
            stance, forces, mu_plan, mu_lo, 20.0
        )

        np.testing.assert_allclose(delta_a, [0.0, 2.0, 0.0])

    # -------------------------------------------------------
    # P-norm conversion
    # -------------------------------------------------------

    def test_p_norm_largest_eigenvalue(self):
        """Use the maximum, not all eigenvalues."""
        B = np.array([
            [1.0, 0.0],
            [0.0, 2.0],
            [0.0, 0.0],
            [0.0, 0.0],
        ])
        P = np.eye(4)
        delta_a = np.array([0.0, 1.0, 2.0])

        result = p_norm_disturbance_bound(delta_a, B, P)

        np.testing.assert_allclose(result, [0.0, 2.0, 4.0])
        self.assertEqual(result.shape, (3,))

    def test_p_norm_bounds_disturbance_directions(self):
        """Computed bound contains disturbances in different directions."""
        magnitudes = np.array([0.1, 0.5, 2.0])

        directions = np.array([
            [1.0, 0.0],
            [0.0, 1.0],
            [1.0, 1.0],
        ])
        directions /= np.linalg.norm(
            directions, axis=1, keepdims=True
        )

        bounds = p_norm_disturbance_bound(
            magnitudes, self.B, self.P
        )

        for a, direction, bound in zip(
            magnitudes, directions, bounds
        ):
            disturbance = a * direction
            w = self.B @ disturbance
            actual_norm = np.sqrt(w.T @ self.P @ w)

            self.assertLessEqual(
                actual_norm, bound + 1e-10
            )

    # -------------------------------------------------------
    # Tube propagation
    # -------------------------------------------------------

    def test_tube_propagation(self):
        """Disturbance increases tube; zero disturbance contracts it."""
        r_w = np.array([0.2, 0.0, 0.0])

        alpha = propagate_tube(
            r_w, rho=0.5, alpha_0=0.1
        )

        np.testing.assert_allclose(
            alpha, [0.1, 0.25, 0.125, 0.0625]
        )
        self.assertEqual(alpha.shape, (4,))

    def test_complete_pipeline(self):
        """Friction inputs -> P-norm bounds -> tube trajectory."""
        stance = np.array([
            [1, 1, 1, 1],
            [1, 1, 0, 0],
            [1, 1, 1, 1],
        ])

        forces = np.array([
            [50, 50, 50, 50],
            [100, 100, 0, 0],
            [50, 50, 50, 50],
        ])

        mu_plan = np.full((3, 4), 0.5)
        mu_lo = np.array([
            [0.5, 0.5, 0.5, 0.5],
            [0.3, 0.3, 0.5, 0.5],
            [0.5, 0.5, 0.5, 0.5],
        ])

        delta_a = friction_acceleration_bound(
            stance, forces, mu_plan, mu_lo, 20.0
        )
        r_w = p_norm_disturbance_bound(
            delta_a, self.B, self.P
        )
        alpha = propagate_tube(
            r_w, self.rho, alpha_0=0.0
        )

        self.assertEqual(delta_a.shape, (3,))
        self.assertEqual(r_w.shape, (3,))
        self.assertEqual(alpha.shape, (4,))

        np.testing.assert_allclose(delta_a, [0, 2, 0])
        np.testing.assert_allclose(r_w[[0, 2]], [0, 0])

        # Disturbance occurs in interval 1.
        self.assertAlmostEqual(alpha[0], 0.0)
        self.assertAlmostEqual(alpha[1], 0.0)
        self.assertGreater(alpha[2], 0.0)

        # The next interval has no disturbance.
        self.assertAlmostEqual(
            alpha[3], self.rho * alpha[2]
        )


    def test_residual_disturbance(self):
        """Residual uncertainty is added to the P-norm bound."""
        delta_a = np.array([0.0, 1.0, 2.0])
        r0 = 0.1

        baseline = p_norm_disturbance_bound(
            delta_a, self.B, self.P
        )

        with_residual = p_norm_disturbance_bound(
            delta_a, self.B, self.P, r0=r0
        )

        np.testing.assert_allclose(
            with_residual, baseline + r0
        )

        self.assertAlmostEqual(with_residual[0], r0)


    def test_negative_residual_rejected(self):
        """Negative uncertainty bounds are invalid."""
        delta_a = np.array([0.0, 1.0])

        with self.assertRaises(ValueError):
            p_norm_disturbance_bound(
                delta_a, self.B, self.P, r0=-0.1
            )


    def test_invalid_mass(self):
        """Mass must be finite and strictly positive."""
        stance = np.ones((2, 4))
        forces = np.full((2, 4), 50.0)

        for mass in [0.0, -10.0, np.nan, np.inf]:
            with self.subTest(mass=mass):
                with self.assertRaises(ValueError):
                    friction_acceleration_bound(
                        stance, forces, 0.5, 0.3, mass
                    )


    def test_invalid_input_shapes(self):
        """Contact masks and forces must have matching (N,4) shapes."""
        cases = [
            (np.ones((3, 4)), np.ones((2, 4))),
            (np.ones((3, 3)), np.ones((3, 3))),
            (np.ones(4), np.ones(4)),
        ]

        for stance, forces in cases:
            with self.subTest(shape=stance.shape):
                with self.assertRaises(ValueError):
                    friction_acceleration_bound(
                        stance, forces, 0.5, 0.3, 20.0
                    )


    def test_invalid_normal_forces(self):
        """Negative or nonfinite normal forces are rejected."""
        stance = np.ones((1, 4))

        for invalid_value in [-10.0, np.nan, np.inf]:
            forces = np.array([
                [50.0, 50.0, invalid_value, 50.0]
            ])

            with self.subTest(value=invalid_value):
                with self.assertRaises(ValueError):
                    friction_acceleration_bound(
                        stance, forces, 0.5, 0.3, 20.0
                    )


    def test_invalid_stance_mask(self):
        """Contact indicators must be binary."""
        forces = np.full((1, 4), 50.0)

        for invalid_value in [0.5, -1.0, 2.0, np.nan]:
            stance = np.array([
                [1.0, 1.0, invalid_value, 0.0]
            ])

            with self.subTest(value=invalid_value):
                with self.assertRaises(ValueError):
                    friction_acceleration_bound(
                        stance, forces, 0.5, 0.3, 20.0
                    )



    def test_invalid_acceleration_bounds(self):
        """Acceleration bounds must be 1D, finite, and nonnegative."""
        cases = [
            np.array([-1.0, 2.0]),
            np.array([np.nan, 1.0]),
            np.array([np.inf, 1.0]),
            np.ones((2, 2)),
        ]

        for delta_a in cases:
            with self.subTest(delta_a=delta_a):
                with self.assertRaises(ValueError):
                    p_norm_disturbance_bound(
                        delta_a, self.B, self.P
                    )


    def test_invalid_B(self):
        """B must be a finite 4x2 matrix."""
        cases = [
            np.ones((2, 4)),
            np.ones((4, 3)),
            np.full((4, 2), np.nan),
        ]

        for B in cases:
            with self.subTest(shape=B.shape):
                with self.assertRaises(ValueError):
                    p_norm_disturbance_bound(
                        np.array([1.0]), B, self.P
                    )


    def test_invalid_P(self):
        """P must be finite, symmetric, and positive definite."""
        cases = [
            np.eye(3),
            np.diag([1.0, 1.0, 1.0, -1.0]),
            np.diag([1.0, 1.0, 1.0, 0.0]),
            np.full((4, 4), np.nan),
        ]

        nonsymmetric = np.eye(4)
        nonsymmetric[0, 1] = 0.5
        cases.append(nonsymmetric)

        for P in cases:
            with self.subTest(shape=P.shape):
                with self.assertRaises(ValueError):
                    p_norm_disturbance_bound(
                        np.array([1.0]), self.B, P
                    )


    def test_nonfinite_residual_rejected(self):
        """Residual uncertainty must be a finite scalar."""
        for r0 in [np.nan, np.inf, -np.inf]:
            with self.subTest(r0=r0):
                with self.assertRaises(ValueError):
                    p_norm_disturbance_bound(
                        np.array([1.0]),
                        self.B,
                        self.P,
                        r0=r0,
                    )


    def test_invalid_tube_disturbances(self):
        """Tube disturbances must be 1D, finite, and nonnegative."""
        cases = [
            np.array([-0.1, 0.2]),
            np.array([np.nan, 0.2]),
            np.array([np.inf, 0.2]),
            np.ones((2, 2)),
        ]

        for r_w in cases:
            with self.subTest(r_w=r_w):
                with self.assertRaises(ValueError):
                    propagate_tube(r_w, rho=0.9)


    def test_invalid_contraction_factor(self):
        """Contraction factor must satisfy 0 <= rho < 1."""
        r_w = np.array([0.1, 0.2])

        for rho in [-0.1, 1.0, 1.2, np.nan, np.inf]:
            with self.subTest(rho=rho):
                with self.assertRaises(ValueError):
                    propagate_tube(r_w, rho=rho)


    def test_invalid_initial_radius(self):
        """Initial radius must be finite and nonnegative."""
        r_w = np.array([0.1, 0.2])

        for alpha_0 in [-0.1, np.nan, np.inf]:
            with self.subTest(alpha_0=alpha_0):
                with self.assertRaises(ValueError):
                    propagate_tube(
                        r_w, rho=0.9, alpha_0=alpha_0
                    )



    def test_compute_friction_tube(self):
        """Convenience function matches the full three-stage pipeline."""

        stance = np.array([
            [1, 1, 1, 1],
            [1, 1, 0, 0],
            [1, 1, 1, 1],
        ])

        forces = np.array([
            [50, 50, 50, 50],
            [100, 100, 0, 0],
            [50, 50, 50, 50],
        ])

        mu_plan = 0.5
        mu_lo = np.array([
            [0.5, 0.5, 0.5, 0.5],
            [0.3, 0.3, 0.5, 0.5],
            [0.5, 0.5, 0.5, 0.5],
        ])

        mass = 20.0
        r0 = 0.05
        alpha_0 = 0.1

        # Independently compute expected result.
        delta_a = friction_acceleration_bound(
            stance, forces, mu_plan, mu_lo, mass
        )

        r_w = p_norm_disturbance_bound(
            delta_a, self.B, self.P, r0=r0
        )

        expected = propagate_tube(
            r_w, self.rho, alpha_0=alpha_0
        )

        # Test the single-call interface.
        actual = compute_friction_tube(
            stance_mask=stance,
            normal_forces=forces,
            mu_plan=mu_plan,
            mu_lo=mu_lo,
            mass=mass,
            B=self.B,
            P=self.P,
            rho=self.rho,
            r0=r0,
            alpha_0=alpha_0,
        )

        np.testing.assert_allclose(actual, expected)
        self.assertEqual(actual.shape, (4,))
        self.assertAlmostEqual(actual[0], alpha_0)

        # Residual uncertainty is present even without a friction gap.
        self.assertGreater(actual[1], 0.0)


    def test_position_radius_diagonal_P(self):
        """Converts a simple weighted ellipsoid into meters."""
        P = np.diag([4.0, 9.0, 1.0, 1.0])
        alpha = np.array([0.0, 1.0, 2.0])

        result = p_norm_to_position_radius(alpha, P)

        np.testing.assert_allclose(result, [0.0, 0.5, 1.0])

    def test_position_radius_coupled_P(self):
        """Accounts for position-velocity coupling in P."""
        P = np.array([
            [2.0, 0.0, 1.0, 0.0],
            [0.0, 2.0, 0.0, 1.0],
            [1.0, 0.0, 2.0, 0.0],
            [0.0, 1.0, 0.0, 2.0],
        ])
        alpha = np.array([0.0, 0.5, 1.0])

        result = p_norm_to_position_radius(alpha, P)

        np.testing.assert_allclose(
            result, alpha * np.sqrt(2.0 / 3.0)
        )


    def test_position_radius_full_pipeline(self):
        """Public interface returns position radii in meters."""
        stance = np.ones((3, 4))
        forces = np.full((3, 4), 50.0)

        args = dict(
            stance_mask=stance,
            normal_forces=forces,
            mu_plan=0.5,
            mu_lo=0.3,
            mass=20.0,
            B=self.B,
            P=self.P,
            rho=self.rho,
        )

        alpha = compute_friction_tube(**args)

        radii_m = compute_friction_tube_position_radii(**args)

        expected = p_norm_to_position_radius(alpha, self.P)

        np.testing.assert_allclose(radii_m, expected)
        self.assertEqual(radii_m.shape, (4,))
        self.assertAlmostEqual(radii_m[0], 0.0)
        self.assertTrue(np.all(radii_m >= 0.0))







if __name__ == "__main__":
    unittest.main(verbosity=2)
