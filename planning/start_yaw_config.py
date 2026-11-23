"""Configure the offline global planner for a known start pose.

This is used ONCE PER RUN, before planning, with the yaw from the run's
initial robot pose (e.g., the seeded MuJoCo spawn pose). It does not access a
live sensor or change the collocation dynamics.
"""

from math import isfinite


# Jacob's currently requested maximum reference rate, not a demonstrated
# hardware tracking guarantee. Physical heading error is checked in Task 2.5.
MAX_REQUESTED_YAW_RATE_RAD_S = 0.5


def config_for_initial_yaw(base_config, *, initial_yaw_rad):
    """Return a copy of `base_config` with start yaw and a safe yaw-rate cap.

    Only the path-tangent three-circle mode is supported because that is the
    mode needing a rate-limited changing heading. This does not change the
    robot's position/velocity `start_state=[px,py,vx,vy]`.

    `initial_yaw_rad` must be the actual, known INITIAL body heading in world
    coordinates for this trial. It is intentionally required; silently assuming
    zero yaw would be incorrect for the randomized-start trials.

    An omitted `footprint_max_yaw_rate` defaults to 0.5 rad/s. An explicitly
    configured smaller positive limit is honored. An explicit None, invalid,
    or larger limit is rejected rather than silently changing caller intent.
    """
    if not isinstance(base_config, dict):
        raise TypeError("base_config must be a dictionary")
    if base_config.get("footprint_model") != "three_circle":
        raise ValueError("start-heading configuration requires three_circle footprint")
    if base_config.get("footprint_yaw_mode", "path_tangent") != "path_tangent":
        raise ValueError("start-heading configuration requires path_tangent yaw mode")

    try:
        yaw = float(initial_yaw_rad)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("initial_yaw_rad must be a finite angle in radians") from exc
    if not isfinite(yaw):
        raise ValueError("initial_yaw_rad must be a finite angle in radians")

    raw_rate = base_config.get("footprint_max_yaw_rate", MAX_REQUESTED_YAW_RATE_RAD_S)
    try:
        rate = float(raw_rate)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("footprint_max_yaw_rate must be finite and in (0, 0.5]") from exc
    if not isfinite(rate) or rate <= 0 or rate > MAX_REQUESTED_YAW_RATE_RAD_S:
        raise ValueError("footprint_max_yaw_rate must be finite and in (0, 0.5]")

    config = base_config.copy()
    # For path-tangent mode the solver uses footprint_yaw as INITIAL yaw,
    # not as a fixed heading for the entire trajectory.
    config["footprint_yaw"] = yaw
    config["footprint_max_yaw_rate"] = rate
    return config
