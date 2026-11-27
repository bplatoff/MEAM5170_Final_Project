"""Time-indexed reference from the global planner: state, acceleration, yaw and yaw rate at any time."""
import numpy as np


def wrap_to_pi(angle):
    return (np.asarray(angle) + np.pi) % (2 * np.pi) - np.pi


def waypoint_yaw_reference(xy, h, start_yaw, max_rate, min_step=0.02):
    """Yaw at each knot: the heading to the next knot, rate limited, starting from the measured yaw.

    The heading is held across steps shorter than min_step, where the direction between two points is mostly noise.
    The result is unwrapped, so it can be interpolated directly.
    """
    xy = np.asarray(xy, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 2:
        raise ValueError("xy must have shape (K+1, 2) with K >= 1")
    if h <= 0 or max_rate <= 0:
        raise ValueError("h and max_rate must be positive")
    yaw = np.empty(len(xy))
    yaw[0] = start_yaw
    target = float(start_yaw)
    for k in range(len(xy) - 1):
        d = xy[k + 1] - xy[k]
        if np.linalg.norm(d) >= min_step:
            target = float(np.arctan2(d[1], d[0]))
        step = float(wrap_to_pi(target - yaw[k]))
        yaw[k + 1] = yaw[k] + np.clip(step, -max_rate * h, max_rate * h)
    return yaw


class PlannerReference:
    """Planner knots t (K+1), x (K+1, 4), piecewise-constant u (K, 2), unwrapped yaw (K+1).

    Inside a knot interval the state follows the exact constant-acceleration solution. After the last knot the
    reference holds the final state with zero acceleration.
    """

    def __init__(self, t, x, u, yaw):
        self.t = np.asarray(t, dtype=float)
        self.x = np.asarray(x, dtype=float)
        self.u = np.asarray(u, dtype=float)
        self.yaw_knots = np.asarray(yaw, dtype=float)
        K = len(self.u)
        if self.t.shape != (K + 1,) or self.x.shape != (K + 1, 4) or self.u.shape != (K, 2) \
                or self.yaw_knots.shape != (K + 1,):
            raise ValueError("expected t (K+1), x (K+1, 4), u (K, 2), yaw (K+1)")
        if K < 1 or np.any(np.diff(self.t) <= 0):
            raise ValueError("t must be strictly increasing")
        self.t_end = float(self.t[-1])

    def _interval(self, s):
        k = int(np.searchsorted(self.t, s + 1e-9, side="right") - 1)
        return min(max(k, 0), len(self.u) - 1)

    def state(self, s):
        if s >= self.t_end:
            return np.r_[self.x[-1, :2], 0.0, 0.0]
        s = max(s, self.t[0])
        k = self._interval(s)
        r, a = s - self.t[k], self.u[k]
        return np.r_[self.x[k, :2] + r * self.x[k, 2:] + 0.5 * r * r * a, self.x[k, 2:] + r * a]

    def accel(self, s):
        if s >= self.t_end:
            return np.zeros(2)
        return self.u[self._interval(max(s, self.t[0]))].copy()

    def yaw(self, s):
        return float(np.interp(s, self.t, self.yaw_knots))

    def yaw_rate(self, s):
        if s >= self.t_end:
            return 0.0
        k = self._interval(max(s, self.t[0]))
        return float((self.yaw_knots[k + 1] - self.yaw_knots[k]) / (self.t[k + 1] - self.t[k]))
