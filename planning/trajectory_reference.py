"""Evaluate a solved three-circle CoM trajectory at arbitrary times.

The global planner returns positions/velocities at mesh knots, constant
accelerations on intervals, and a separately prescribed yaw reference. This
module reconstructs both without changing the collocation problem or its mesh.

Only a *reference* is returned: tracking, dynamic yaw feasibility, and actual
robot collision safety must be validated by the downstream controller.
"""

import casadi as ca
import numpy as np

from planning.yaw_reference import make_rate_limited_yaw_reference


class TrajectoryReference:
    """Cache a successful plan for repeated reference requests from the MPC.

    Args:
        plan: `plan_trajectory()` result, or the `robust` child of a successful
            `plan_two_pass_friction_tube()` result (not the two-pass wrapper).
        config: The **same** planning configuration used to produce `plan`.
            Three-circle `fixed` yaw or bounded-rate `path_tangent` yaw is
            required for this full position/velocity/yaw interface.

    ``evaluate(t)`` accepts a scalar time or a one-dimensional collection of
    times and returns t, x=[px,py,vx,vy], u=[ax,ay], yaw, and yaw_rate.
    Yaw is unwrapped. At a control/yaw interval boundary the right-hand
    acceleration/yaw-rate value is returned, except at the final time where
    the left-hand value is returned. No extrapolation or smoothing is applied.
    """

    def __init__(self, plan, config):
        if not isinstance(plan, dict) or not plan.get("success", False):
            raise ValueError("Reference requires a successful planner result")
        if not isinstance(config, dict):
            raise TypeError("config must be the configuration used to plan")
        if config.get("footprint_model") != "three_circle":
            raise ValueError("Reference requires a three-circle planner result")

        try:
            t = np.asarray(plan["t"], dtype=float)
            x = np.asarray(plan["x"], dtype=float)
            u = np.asarray(plan["u"], dtype=float)
            yaw_knots = np.asarray(plan["yaw_ref"], dtype=float)
        except KeyError as exc:
            raise ValueError(f"Planner result missing required field {exc}") from exc
        if (t.ndim != 1 or len(t) < 2 or x.shape != (len(t), 4) or
                u.shape != (len(t) - 1, 2) or yaw_knots.shape != t.shape):
            raise ValueError("Invalid t, x, u, or yaw_ref shapes")
        if not all(np.all(np.isfinite(v)) for v in (t, x, u, yaw_knots)):
            raise ValueError("Trajectory values must be finite")
        h = np.diff(t)
        if np.any(h <= 0) or not np.allclose(h, h[0], atol=1e-9, rtol=1e-9):
            raise ValueError("The current collocation/yaw interface requires a uniform time grid")
        horizon = float(t[-1] - t[0])
        if horizon <= 0:
            raise ValueError("Trajectory horizon must be positive")

        # The solved point-mass model has a constant acceleration on each
        # interval. Validate its discrete dynamics before using it as a
        # reference; this also avoids silently exporting malformed test data.
        predicted = x[:-1].copy()
        predicted[:, :2] += h[:, None] * x[:-1, 2:] + 0.5 * h[:, None]**2 * u
        predicted[:, 2:] += h[:, None] * u
        if not np.allclose(predicted, x[1:], atol=1e-5, rtol=0.0):
            raise ValueError("Knot states are inconsistent with constant-acceleration dynamics")

        yaw_mode = config.get("footprint_yaw_mode", "path_tangent")
        initial_yaw = float(config.get("footprint_yaw", 0.0))
        if not np.isfinite(initial_yaw):
            raise ValueError("footprint_yaw must be finite")
        if yaw_mode == "fixed":
            yaw_times = np.array([t[0], t[-1]])
            yaw_values = np.array([initial_yaw, initial_yaw])
            predicted_knots = np.full(len(t), initial_yaw)
        elif yaw_mode == "path_tangent":
            max_rate = config.get("footprint_max_yaw_rate")
            if max_rate is None:
                raise ValueError(
                    "Unbounded path-tangent yaw has discontinuous turns and no finite "
                    "yaw-rate reference; set footprint_max_yaw_rate or use fixed yaw"
                )
            try:
                max_rate = float(max_rate)
            except (TypeError, ValueError) as exc:
                raise ValueError("footprint_max_yaw_rate must be positive and finite") from exc
            if not np.isfinite(max_rate) or max_rate <= 0:
                raise ValueError("footprint_max_yaw_rate must be positive and finite")
            if "astar_path" not in plan:
                raise ValueError("Missing A* path required to reconstruct planner yaw")

            # EXACTLY the normalized grid used in direct_collocation.py and
            # PR #9's continuous-clearance certificate: start, all degree-3
            # Legendre points, and the end of every interval.
            n = len(u)
            tau = ca.collocation_points(3, "legendre")
            fractions = [0.0]
            for k in range(n):
                fractions.extend((k + float(s)) / n for s in tau)
                fractions.append((k + 1) / n)
            fractions = np.asarray(fractions, dtype=float)
            yaw_fn, yaw_values = make_rate_limited_yaw_reference(
                plan["astar_path"], fractions, horizon, max_rate,
                initial_yaw=initial_yaw,
            )
            yaw_times = t[0] + horizon * fractions
            predicted_knots = np.array([yaw_fn(k / n) for k in range(n + 1)])
        else:
            raise ValueError("Unsupported footprint_yaw_mode")

        if not np.allclose(predicted_knots, yaw_knots, atol=1e-8, rtol=0.0):
            raise ValueError("Yaw knots disagree with the supplied planning configuration")
        if np.any(np.diff(yaw_times) <= 0):
            raise ValueError("Yaw reference times must be strictly increasing")

        # Defensive copies ensure later changes to the planner dictionaries
        # cannot silently mutate a cached MPC reference.
        self.t = t.copy()
        self.x = x.copy()
        self.u = u.copy()
        self.yaw_times = yaw_times.copy()
        self.yaw_values = yaw_values.copy()
        self.yaw_rates = np.diff(yaw_values) / np.diff(yaw_times)

    def evaluate(self, times):
        """Return the modeled trajectory at queried times; never extrapolate.

        Scalar input yields scalar t/yaw/yaw_rate and vectors x(4), u(2).
        1-D input yields t(M), x(M,4), u(M,2), yaw(M), yaw_rate(M).
        """
        requested = np.asarray(times, dtype=float)
        scalar = requested.ndim == 0
        if scalar:
            requested = requested.reshape(1)
        if requested.ndim != 1 or requested.size == 0:
            raise ValueError("times must be a scalar or nonempty 1-D array")
        if not np.all(np.isfinite(requested)):
            raise ValueError("times must be finite")
        if np.any(requested < self.t[0]) or np.any(requested > self.t[-1]):
            raise ValueError("Requested times lie outside the planned horizon")

        k = np.clip(np.searchsorted(self.t, requested, side="right") - 1,
                    0, len(self.u) - 1)
        dt = requested - self.t[k]
        state = self.x[k].copy()
        state[:, :2] += dt[:, None] * self.x[k, 2:] + 0.5 * dt[:, None]**2 * self.u[k]
        state[:, 2:] += dt[:, None] * self.u[k]
        state[requested == self.t[-1]] = self.x[-1]
        acceleration = self.u[k].copy()

        j = np.clip(np.searchsorted(self.yaw_times, requested, side="right") - 1,
                    0, len(self.yaw_rates) - 1)
        yaw_rate = self.yaw_rates[j].copy()
        yaw = self.yaw_values[j] + yaw_rate * (requested - self.yaw_times[j])

        if scalar:
            return {"t": float(requested[0]), "x": state[0], "u": acceleration[0],
                    "yaw": float(yaw[0]), "yaw_rate": float(yaw_rate[0])}
        return {"t": requested.copy(), "x": state, "u": acceleration,
                "yaw": yaw, "yaw_rate": yaw_rate}


def evaluate_trajectory(plan, times, config):
    """One-shot convenience wrapper; reuse TrajectoryReference for MPC ticks."""
    return TrajectoryReference(plan, config).evaluate(times)
