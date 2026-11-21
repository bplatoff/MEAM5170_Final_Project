"""Build time-indexed, bounded-rate nominal yaw references for the global planner.

This does not optimize robot yaw dynamics or prove the low-level controller tracks
these values. It merely prevents the reference from requesting yaw rates above
a specified kinematic limit at the planner's sampled times.
"""

import numpy as np

from planning.planner_helpers import sample_path_heading


def make_rate_limited_yaw_reference(
    path, sample_fractions, horizon, max_yaw_rate, initial_yaw=0.0,
):
    """Return yaw(fraction) and the unwrapped time-sampled reference.

    `sample_fractions` must contain 0 and 1 and be strictly increasing.
    Yaw follows the shortest unwrapped variation of the A* path heading,
    constrained by `max_yaw_rate` in rad/s. It may temporarily point away
    from the path tangent; the collocation obstacle checks use the SAME
    rate-limited orientation, so those differences are handled geometrically.
    """
    fractions = np.asarray(sample_fractions, dtype=float)
    if (fractions.ndim != 1 or len(fractions) < 2 or
            not np.all(np.isfinite(fractions)) or
            abs(fractions[0]) > 1e-12 or abs(fractions[-1] - 1.0) > 1e-12 or
            np.any(np.diff(fractions) <= 0)):
        raise ValueError("sample_fractions must be increasing from 0 to 1")
    if (not np.isfinite(horizon) or horizon <= 0 or
            not np.isfinite(max_yaw_rate) or max_yaw_rate <= 0 or
            not np.isfinite(initial_yaw)):
        raise ValueError("horizon, yaw-rate limit, and initial yaw must be valid")

    raw = np.array([sample_path_heading(float(alpha), path, fallback_yaw=initial_yaw)
                    for alpha in fractions])
    # Unwrap around the actual starting orientation to avoid an arbitrary 2pi jump.
    unwrapped = np.unwrap(np.concatenate(([initial_yaw], raw)))[1:]
    limits = max_yaw_rate * horizon * np.diff(fractions)
    limited = np.empty_like(unwrapped)
    limited[0] = initial_yaw
    for k in range(1, len(fractions)):
        limited[k] = limited[k - 1] + np.clip(
            unwrapped[k] - limited[k - 1], -limits[k-1], limits[k-1]
        )

    def yaw_at(fraction):
        if not np.isfinite(fraction):
            raise ValueError("fraction must be finite")
        return float(np.interp(fraction, fractions, limited))

    return yaw_at, limited
