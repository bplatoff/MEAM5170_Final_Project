"""Continuous-time collision certificate for the planned three-circle footprint.

Conservatively certifies the *modeled* path, not physical robot tracking.
Each nominal segment has exact double-integrator translation and a prescribed
piecewise-linear, rate-limited yaw reference. The center-to-obstacle distance
is 1-Lipschitz, so a clearance at the midpoint larger than an upper bound on
center displacement certifies every instant in the interval.

A leaf that cannot be certified is reported as inconclusive, NEVER as safe.
The verifier works with the physical polygon distance (not the NLP's
conservative smooth-max separating-plane approximation).
"""

from dataclasses import dataclass
import numpy as np
import casadi as ca

from planning.robot_footprint import three_circle_parameters, three_circle_centers
from planning.yaw_reference import make_rate_limited_yaw_reference


@dataclass(frozen=True)
class _Obstacle:
    vertices: np.ndarray
    starts: np.ndarray
    edges: np.ndarray
    squared_lengths: np.ndarray
    normals: np.ndarray


def _prepare_polygon(poly):
    p = np.asarray(poly, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.all(np.isfinite(p)):
        raise ValueError("Obstacle must have >=3 finite [x,y] vertices")
    e = np.roll(p, -1, axis=0) - p
    length2 = np.sum(e * e, axis=1)
    if np.any(length2 <= 0):
        raise ValueError("Obstacle polygon has zero-length edge")
    normals = np.column_stack((e[:, 1], -e[:, 0]))
    return _Obstacle(p, p, e, length2, normals)


def _point_to_polygon_distance(c, obs):
    # Convex CCW polygon: inside is a collision for any positive circle radius.
    if np.all(np.sum(obs.normals * (c - obs.starts), axis=1) <= 0):
        return 0.0
    rel = c - obs.starts
    weights = np.clip(np.sum(rel * obs.edges, axis=1) / obs.squared_lengths, 0.0, 1.0)
    closest = obs.starts + weights[:, None] * obs.edges
    return float(np.min(np.linalg.norm(closest - c, axis=1)))


def _nominal_yaw_function(result, config):
    mode = config.get("footprint_yaw_mode", "path_tangent")
    initial = float(config.get("footprint_yaw", 0.0))
    if not np.isfinite(initial):
        raise ValueError("footprint_yaw must be finite")
    if mode == "fixed":
        return lambda fraction: initial, 0.0
    if mode != "path_tangent":
        raise ValueError("Unsupported yaw mode for certification")
    yaw_rate = config.get("footprint_max_yaw_rate")
    if yaw_rate is None or not np.isfinite(yaw_rate) or yaw_rate <= 0:
        raise ValueError("Continuous certification requires footprint_max_yaw_rate > 0")
    if "astar_path" not in result:
        raise ValueError("Missing the A* path used to construct nominal yaw")

    N = len(result["u"])
    tau = ca.collocation_points(3, "legendre")
    fractions = [0.0]
    for k in range(N):
        fractions.extend((k + float(s)) / N for s in tau)
        fractions.append((k+1)/N)
    total_T = float(result["t"][-1] - result["t"][0])
    fn, _ = make_rate_limited_yaw_reference(
        result["astar_path"], fractions, total_T, float(yaw_rate),
        initial_yaw=initial,
    )
    if "yaw_ref" in result:
        predicted = [fn(float(k)/N) for k in range(N+1)]
        if not np.allclose(predicted, result["yaw_ref"], atol=1e-8, rtol=0.0):
            raise ValueError("Yaw knots disagree with the yaw function being certified")
    return fn, float(yaw_rate)


def verify_continuous_clearance(
    result, environment, config, *, max_depth=17,
    clearance_tolerance_m=1e-6, max_checks=120000,
    max_findings=6,
):
    """Certify continuous clearance, or return witnessed/conservative failures.

    Returns {certified, status, min_sampled_clearance_m, checks, findings}.
    `findings` have status = 'collision' or 'inconclusive'. An inconclusive
    leaf is unsafe to ACCEPT but is not proof that a collision exists.
    `clearance_tolerance_m` reserves space for numerical error; it is NOT a
    replacement for the configured geometric or tube safety margin.
    """
    if config.get("footprint_model") != "three_circle":
        raise ValueError("Three-circle footprint is required for certification")
    if (not isinstance(max_depth, int) or max_depth < 0 or
            not isinstance(max_checks, int) or max_checks <= 0 or
            not isinstance(max_findings, int) or max_findings <= 0 or
            not np.isfinite(clearance_tolerance_m) or clearance_tolerance_m < 0):
        raise ValueError("Invalid certification limits or tolerance")
    times = np.asarray(result["t"], dtype=float)
    x = np.asarray(result["x"], dtype=float)
    u = np.asarray(result["u"], dtype=float)
    if (times.ndim != 1 or x.shape != (len(times), 4) or
            u.shape != (len(times)-1, 2) or
            not np.all(np.isfinite(x)) or not np.all(np.isfinite(u)) or
            not np.all(np.diff(times) > 0)):
        raise ValueError("Invalid trajectory arrays")
    T = float(times[-1] - times[0])
    yaw_at, yaw_rate = _nominal_yaw_function(result, config)
    length = float(config["robot_length"])
    safety = float(config["r_safety"] + config.get("tube_position_margin_m", 0.0))
    offsets, radius = three_circle_parameters(length, float(config["robot_width"]), safety)
    xmin, xmax, ymin, ymax = environment.bounds
    obstacles = [_prepare_polygon(obs) for obs in environment.obstacles]
    overall_min = np.inf
    checks = 0
    findings = []

    def clearance_and_kind(k, local_dt):
        nonlocal overall_min, checks
        checks += 1
        p = x[k, :2] + local_dt * x[k, 2:] + 0.5 * local_dt**2 * u[k]
        yaw = yaw_at((times[k] - times[0] + local_dt) / T)
        centers = three_circle_centers(p, yaw, length)
        best = (np.inf, -1, "none")
        for j, c in enumerate(centers):
            wall_margin = [c[0]-xmin-radius, xmax-c[0]-radius,
                           c[1]-ymin-radius, ymax-c[1]-radius]
            side = int(np.argmin(wall_margin))
            if wall_margin[side] < best[0]:
                best = (float(wall_margin[side]), j,
                        ("wall", ("left", "right", "bottom", "top")[side]))
            for oi, obs in enumerate(obstacles):
                margin = _point_to_polygon_distance(c, obs) - radius
                if margin < best[0]:
                    best = (float(margin), j, ("obstacle", oi))
        overall_min = min(overall_min, best[0])
        return best

    # Check node consistency first; certificates rely on exact interval dynamics.
    for k in range(len(u)):
        h = times[k+1]-times[k]
        next_x = x[k].copy()
        next_x[:2] += h * x[k,2:] + .5*h*h*u[k]
        next_x[2:] += h*u[k]
        if not np.allclose(next_x, x[k+1], atol=1e-5, rtol=0):
            return {
                "certified": False, "status": "dynamics_mismatch",
                "checks": checks, "min_sampled_clearance_m": float("nan"),
                "findings": [],
            }

    for k in range(len(u)):
        h = float(times[k+1]-times[k])
        stack = [(0.0, h, 0)]
        failed_interval = False
        while stack:
            a, b, depth = stack.pop()
            midpoint = .5 * (a+b)
            dist, ci, kind = clearance_and_kind(k, midpoint)
            global_t = float(times[k] + midpoint)
            v0 = x[k,2:] + a*u[k]
            v1 = x[k,2:] + b*u[k]
            vmax = max(np.linalg.norm(v0), np.linalg.norm(v1))
            max_center_motion = (.5*(b-a) *
                                 (vmax + np.max(np.abs(offsets)) * yaw_rate))
            if dist > max_center_motion + clearance_tolerance_m:
                continue  # The full interval is certified, not just midpoint.
            if dist < -clearance_tolerance_m:
                outcome = "collision"
            elif depth >= max_depth or checks >= max_checks:
                outcome = "inconclusive"
            else:
                stack.append((midpoint, b, depth+1))
                stack.append((a, midpoint, depth+1))
                continue
            findings.append({
                "status": outcome, "time_s": global_t,
                "interval": k, "circle": ci,
                "feature": kind, "sampled_clearance_m": dist,
                "displacement_bound_m": float(max_center_motion),
            })
            failed_interval = True
            break
        if len(findings) >= max_findings or checks >= max_checks:
            break
        if failed_interval:
            continue

    certified = not findings and checks < max_checks
    if not certified and not findings:
        findings.append({"status":"inconclusive", "time_s":float(times[min(k+1,len(times)-1)]),
                         "interval":int(k), "circle":0, "feature":("unknown",-1),
                         "sampled_clearance_m":float("nan"),
                         "displacement_bound_m":float("nan")})
    return {
        "certified": certified,
        "status": "certified" if certified else "uncertified",
        "checks": checks,
        "min_sampled_clearance_m": float(overall_min),
        "findings": findings,
    }
