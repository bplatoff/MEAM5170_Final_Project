import unittest
import numpy as np

from planning.robust_control.tube_error_model import (
    design_tube_error_model,
)


def simulate_error_recovery(A, B, K, e0, steps=100):
        """
        Simulate disturbance-free closed-loop error recovery. 

        Args:
            A: Discrete state transition matrix
            B: Discrete input matrix
            K: Feedback gain
            e0: Initial error state [ex, ey, evx, evy]
            steps: Number of simulation timesteps

        Returns:
            states: Error history, shape (steps+1, 4)
            controls: Corrective acceleration history, shape (steps, 2)
        """
        states = np.zeros((steps + 1, 4))
        controls = np.zeros((steps, 2))

        states[0] = e0

        # For each timestep, calculate the feedback correction
        # and propagate the error state using your dynamics.
        for i in range(steps):
            controls[i] = K @ states[i]
            states[i+1] = A @ states[i] + B @ controls[i]

        return states, controls


class TestTubeErrorModel(unittest.TestCase):

    def setUp(self):
        self.model = design_tube_error_model()
        self.A = self.model["A"]
        self.B = self.model["B"]
        self.K = self.model["K"]
        self.P = self.model["P"]
        self.rho = self.model["rho"]
        self.Acl = self.model["Acl"]

    def test_stability_and_certificate(self):
        self.assertTrue(
            np.all(np.abs(np.linalg.eigvals(self.Acl)) < 1)
        )
        self.assertTrue(
            np.all(np.linalg.eigvalsh(self.P) > 0)
        )
        self.assertLess(self.rho, 1.0)

        Q = np.diag([400, 400, 25, 25])
        residual = self.Acl.T @ self.P @ self.Acl - self.P + Q
        self.assertTrue(np.allclose(residual, 0))

    def test_error_recovery_and_contraction(self):
        cases = {
            "position": [0.05, 0, 0, 0],
            "velocity": [0, 0, 0.2, 0],
            "combined": [0.05, 0, 0.2, 0],
            "lateral": [0, 0.05, 0, 0],
        }

        for name, e0 in cases.items():
            with self.subTest(case=name):
                states, controls = simulate_error_recovery(
                    self.A, self.B, self.K, e0
                )

                # Compute P-weighted error norm at each timestep
                norms = np.array([
                    np.sqrt(e @ self.P @ e)
                    for e in states
                ])

                # Check contraction at every timestep
                self.assertTrue(np.all(
                    norms[1:] <= self.rho * norms[:-1] + 1e-10
                ))

                # Check recovery after 100 timesteps
                self.assertLess(norms[-1], 0.1 * norms[0])

                # Report required acceleration
                peak_acc = np.max(
                    np.linalg.norm(controls, axis=1)
                )

                print(
                    f"{name}: peak acceleration = "
                    f"{peak_acc:.3f} m/s^2"
                )

    def test_configurable_timestep(self):
        for dt in [0.01, 0.02, 0.04]:
            with self.subTest(dt=dt):
                model = design_tube_error_model(dt=dt)
                self.assertLess(model["rho"], 1.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
