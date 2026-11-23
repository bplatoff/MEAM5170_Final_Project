"""Two-pass global planner with position-error tube clearance.

Pass 1 finds the nominal (un-tightened) collocation trajectory. A caller's
contact predictor then supplies the stance schedule, predicted surface-normal
forces, and friction bounds. We propagate the existing LQR error tube at its
OWN time step (usually 0.02 s), convert it to a horizontal position radius in
meters, and replan with the maximum radius along the full route.

The contact predictor is deliberately external: the 2D global planner does
not predict quadruped footholds or contact forces. Without physically valid
predictions or a validated residual disturbance bound, this is only a modeled
robust clearance, not a full-robot collision guarantee.
"""

import logging

import numpy as np

from planning.direct_collocation import plan_trajectory
from planning.robust_control.friction_tube import compute_friction_tube_position_radii

_LOG = logging.getLogger(__name__)


def _run_planning_pass(start_state, goal_state, environment, config):
    """Normalize known planning failures; leave programming/config errors visible.

    The direct planner returns structured failures when continuous clearance is
    enabled (PR #9), but legacy A* and CasADi paths can still raise RuntimeError.
    Only recognized A*/solver exceptions are converted into expected failures.
    """
    try:
        return plan_trajectory(start_state, goal_state, environment, config)
    except RuntimeError as exc:
        message = str(exc)
        if message.startswith((
            "Three-circle A*", "Heading-aware three-circle A*",
            "A* could not find a collision-free path",
        )):
            status = "planning_failed"
        elif (type(exc).__name__ == "_CollocationSolverError"
              or "IpoptInterface" in message
              or "Error in Function::call for 'solver'" in message):
            status = "solver_failed"
        else:
            raise  # Unexpected runtime error: do not hide a potential bug.
        return {"success": False, "planning_status": status,
                "return_status": message}


def _failed_outcome(stage, failed_plan, *, nominal=None, radii=None,
                    margin=None, robust_config=None):
    """Never offer a failed plan in the executable `robust` slot."""
    status = failed_plan.get("planning_status") or "solver_failed"
    reason = failed_plan.get("return_status")
    if reason is None:
        reason = "Planner returned success=False without a return_status"
    diagnostics = {key: failed_plan[key] for key in (
        "repair_attempts", "certification", "repair_history",
    ) if key in failed_plan}
    concise_reason = " ".join(str(reason).split())
    if len(concise_reason) > 240:
        concise_reason = concise_reason[:237] + "..."
    _LOG.warning("Two-pass planning failed: stage=%s, status=%s, reason=%s",
                 stage, status, concise_reason)
    return {
        "success": False,
        "failed_stage": stage,
        "planning_status": status,
        "return_status": str(reason),
        "nominal": nominal,  # Only a successfully completed nominal trajectory.
        "robust": None,      # Never substitute a nominal plan for the robust plan.
        "tube_position_radii_m": radii,
        "tube_position_margin_m": margin,
        "robust_config": robust_config,
        "failure_diagnostics": diagnostics,
    }


def plan_two_pass_friction_tube(
    start_state, goal_state, environment, config,
    predict_contacts, error_model, error_dt,
    *, r0=0.0, alpha_0=0.0,
):
    """Run the nominal and tube-inflated global planner.

    `predict_contacts(nominal_plan)` must return a dict with:
      stance_mask: (M, 4), 0 or 1
      normal_forces: (M, 4), surface-normal force per foot, newtons
      mu_plan, mu_lo: scalars or arrays broadcastable to (M,4)
      mass: positive quadruped mass, kilograms

    `error_model` is from design_tube_error_model(dt=error_dt), containing
    A, B, K, P, rho. M*error_dt must cover the full global-planner horizon.

    The max radius is used at *all* positions, including start and goal,
    per the team's global-planner upper-bound rule. The midlayer can use
    time-dependent radii along its own prediction horizon.

    Returns success=True only when both passes succeed. Expected A*/solver/
    certification failures return success=False, failed_stage, planning_status,
    and return_status. On robust-pass failure the valid nominal trajectory is
    retained for diagnostics only; robust is None. Consumers MUST check the
    top-level success flag before using a reference. Input/config errors raise.
    """
    if config.get("footprint_model") != "three_circle":
        raise ValueError("two-pass planner requires footprint_model='three_circle'")
    if not callable(predict_contacts):
        raise TypeError("predict_contacts must be callable")
    if not np.isscalar(error_dt) or not np.isfinite(error_dt) or error_dt <= 0:
        raise ValueError("error_dt must be a positive finite scalar")
    if config.get("tube_position_margin_m", 0.0) != 0.0:
        raise ValueError("omit tube_position_margin_m in two-pass config; it is computed here")

    # First pass: footprint + geometric safety margin, with no tube.
    nominal_config = dict(config)
    nominal_config["tube_position_margin_m"] = 0.0
    nominal = _run_planning_pass(start_state, goal_state, environment, nominal_config)
    if not nominal["success"]:
        return _failed_outcome("nominal", nominal)

    predictions = predict_contacts(nominal)
    if not isinstance(predictions, dict):
        raise TypeError("predict_contacts must return a dictionary")
    required = {"stance_mask", "normal_forces", "mu_plan", "mu_lo", "mass"}
    if not required.issubset(predictions):
        raise ValueError(f"contact prediction missing: {sorted(required - predictions.keys())}")

    stance = np.asarray(predictions["stance_mask"])
    normal = np.asarray(predictions["normal_forces"])
    if (stance.ndim != 2 or stance.shape[1] != 4 or normal.shape != stance.shape):
        raise ValueError("contact predictions must have matching (M, 4) shapes")
    horizon = float(nominal["t"][-1] - nominal["t"][0])
    if not np.isclose(stance.shape[0] * error_dt, horizon, rtol=0, atol=1e-7):
        raise ValueError("contact predictions must cover entire nominal horizon at error_dt")

    # Prevent accidental mixing of a 0.02-s tube certificate with 0.4-s
    # planner knots, which would invalidate the contraction and disturbance map.
    B = np.asarray(error_model["B"], dtype=float)
    expected_B = np.vstack((0.5 * error_dt**2 * np.eye(2), error_dt * np.eye(2)))
    if B.shape != (4, 2) or not np.allclose(B, expected_B, rtol=1e-8, atol=1e-12):
        raise ValueError("error_model B is inconsistent with error_dt")

    radii_m = compute_friction_tube_position_radii(
        stance_mask=stance,
        normal_forces=normal,
        mu_plan=predictions["mu_plan"],
        mu_lo=predictions["mu_lo"],
        mass=predictions["mass"],
        B=B,
        P=error_model["P"],
        rho=error_model["rho"],
        r0=r0,
        alpha_0=alpha_0,
    )
    if (radii_m.shape != (stance.shape[0] + 1,) or
            not np.all(np.isfinite(radii_m)) or np.any(radii_m < 0)):
        raise ValueError("tube prediction returned invalid meter-valued position radii")
    margin_m = float(np.max(radii_m))

    # Second pass: identical environment/reference, but enlarge each of the
    # three footprint circles and both A* and NLP collision constraints.
    robust_config = dict(config)
    robust_config["tube_position_margin_m"] = margin_m
    robust = _run_planning_pass(start_state, goal_state, environment, robust_config)
    if not robust["success"]:
        return _failed_outcome(
            "robust", robust, nominal=nominal, radii=radii_m,
            margin=margin_m, robust_config=robust_config,
        )

    return {
        "success": True,
        "failed_stage": None,
        "planning_status": "success",
        "return_status": robust.get("return_status"),
        "failure_diagnostics": {},
        "nominal": nominal,
        "robust": robust,
        "tube_position_radii_m": radii_m,
        "tube_position_margin_m": margin_m,
        "robust_config": robust_config,
    }
