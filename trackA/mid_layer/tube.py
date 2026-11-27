"""Tube width along the mid layer's horizon, using Track B's error model and tube functions."""
import numpy as np

from planning.robust_control.friction_tube import (
    p_norm_disturbance_bound,
    p_norm_to_position_radius,
    propagate_tube,
)
from planning.robust_control.tube_error_model import design_tube_error_model


class TubeTracker:
    """Holds the current P-norm width alpha and propagates it on the LQR grid.

    Before learning the friction gap is the same everywhere, so r_w is constant.
    """

    def __init__(self, cfg, model=None):
        self.cfg = cfg
        self.model = model if model is not None else design_tube_error_model(cfg.dt_fb)
        expected_b = np.vstack((0.5 * cfg.dt_fb ** 2 * np.eye(2), cfg.dt_fb * np.eye(2)))
        if not np.allclose(self.model["B"], expected_b):
            raise ValueError("the error model's dt does not match cfg.dt_fb")
        self.P, self.K, self.rho = self.model["P"], self.model["K"], float(self.model["rho"])
        delta_a = max(0.0, cfg.mu_plan - cfg.mu_lo) * cfg.gravity
        self.r_w = float(p_norm_disturbance_bound(np.array([delta_a]), self.model["B"], self.P, cfg.r0)[0])
        self.alpha = 0.0

    @property
    def steady_alpha(self) -> float:
        return self.r_w / (1.0 - self.rho)

    @property
    def steady_radius(self) -> float:
        return self.radius(self.steady_alpha)

    def radius(self, alpha) -> float:
        return float(p_norm_to_position_radius(np.array([alpha], dtype=float), self.P)[0])

    def error_norm(self, e) -> float:
        e = np.asarray(e, dtype=float)
        return float(np.sqrt(e @ self.P @ e))

    def reset(self, e0) -> None:
        self.alpha = self.error_norm(e0)

    def step(self) -> None:
        self.alpha = self.rho * self.alpha + self.r_w

    def stage_radii(self, horizon: int, substeps: int, alpha0=None) -> np.ndarray:
        """Position radius [m] for QP stages 0..horizon: the largest value in the LQR steps leading up to each stage.

        The tube starts from the current alpha, or from alpha0 if given (0 for a solve that starts at the measured state).
        """
        start = self.alpha if alpha0 is None else float(alpha0)
        alpha = propagate_tube(np.full(horizon * substeps, self.r_w), self.rho, start)
        radii = p_norm_to_position_radius(alpha, self.P)
        out = np.empty(horizon + 1)
        out[0] = radii[0]
        for k in range(1, horizon + 1):
            out[k] = radii[(k - 1) * substeps:k * substeps + 1].max()
        return out
