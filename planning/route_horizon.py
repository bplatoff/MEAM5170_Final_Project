"""Pick a fixed collocation horizon from the geometric A* initializer.

This does not optimize time inside IPOPT. `horizon_mode='fixed'` preserves
legacy callers; `horizon_mode='astar_length'` chooses T once, before the NLP.
"""

import math

import numpy as np


DEFAULT_CRUISE_SPEED_MPS = 0.4  # conservative initial PathFollower speed, configurable
DEFAULT_TIME_BUFFER_S = 2.0    # heuristic allowance for starting/stopping and turns


def choose_planning_horizon(config, astar_path):
    """Return (T_seconds, A_star_length_meters) for a fixed-T NLP.

    In route-length mode: T = path_length / cruise_speed_mps + time_buffer_s.
    Optional horizon_quantum_s rounds UP to a control/tube time grid.
    In fixed mode the caller's explicit config['T'] is used unchanged.
    """
    if not isinstance(config, dict):
        raise TypeError("config must be a dictionary")
    path = np.asarray(astar_path, dtype=float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2 or not np.all(np.isfinite(path)):
        raise ValueError("astar_path must be a finite polyline with shape (K>=2, 2)")
    length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    if not math.isfinite(length):
        raise ValueError("A* path length must be finite")

    mode = config.get("horizon_mode", "fixed")
    if mode == "fixed":
        try:
            T = float(config["T"])
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError("fixed horizon mode requires a positive finite T") from exc
        if not math.isfinite(T) or T <= 0:
            raise ValueError("fixed horizon mode requires a positive finite T")
        return T, length
    if mode != "astar_length":
        raise ValueError("horizon_mode must be 'fixed' or 'astar_length'")
    if length <= 0:
        raise ValueError("A* path must have positive length in astar_length mode")

    try:
        speed = float(config.get("cruise_speed_mps", DEFAULT_CRUISE_SPEED_MPS))
        buffer = float(config.get("time_buffer_s", DEFAULT_TIME_BUFFER_S))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("cruise_speed_mps and time_buffer_s must be finite numbers") from exc
    if not math.isfinite(speed) or speed <= 0 or not math.isfinite(buffer) or buffer < 0:
        raise ValueError("cruise_speed_mps must be positive and time_buffer_s nonnegative")
    raw_T = length / speed + buffer
    if not math.isfinite(raw_T):
        raise ValueError("computed route horizon is not finite")
    quantum = config.get("horizon_quantum_s")
    if quantum is None:
        return raw_T, length
    try:
        quantum = float(quantum)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("horizon_quantum_s must be positive and finite") from exc
    if not math.isfinite(quantum) or quantum <= 0:
        raise ValueError("horizon_quantum_s must be positive and finite")
    count = math.ceil(raw_T / quantum - 1e-10)
    return max(count, 1) * quantum, length
