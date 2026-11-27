"""Per-run evaluation metrics for the batch runner (task 2.3f, decisions A2.4, A3.5, C2).

Everything is computed after the run from the run_waypoints log (.npz), the expanded course dict and the job
settings, so metrics can be recomputed for old runs without simulating again.

    outcome            no_feasible_plan / collision / fall / timeout / success (first that applies, in that order).
                       A run that touched a wall or obstacle is a collision even if it reached the goal.
    time_to_goal       simulated time of a successful run; ramp_time from the bottom to the top crease
    body_clearance     smallest distance from the bare 0.7 x 0.31 m body rectangle to an obstacle or wall (A2.4)
    circle_slack       smallest distance from the three footprint circles to an obstacle or wall, minus
                       radius + safety margin. Below 0 the robot used up its safety margin. tube_used = 1 - slack /
                       (tube + heading allowance) is the share of the reserved buffer used at the tightest moment.
    slips              force-driven slips (slip_windows below): the foot slides while the controller asks the ground
                       for more friction than it delivers. Landing slides are not slips.
    limit_fraction     share of commanded-stance time a foot's commanded force is at its friction pyramid limit
                       (decision B4's assumption that this is rare; checked in task 2.5)
"""
import math

import numpy as np

from trackC.course import Course
from trackC.start_pose import CIRCLE_OFFSETS, CIRCLE_RADIUS, PLAN_SAFETY_MARGIN, ROBOT_LENGTH, ROBOT_WIDTH

# Slip rule (see the README, "Slips"). All quantities are ones a real robot measures or commands.
SLIP_WINDOW_STEPS = 10      # 20 ms at the 2 ms simulation step
V_SLIP = 0.05               # [m/s] foot speed along the surface that counts as moving
CMD_FN_MIN = 10.0           # [N] commanded normal force that means the controller treats the foot as in stance
FN_MIN = 5.0                # [N] measured normal force below which ratios are not trusted (impact, liftoff)
RATIO_GAP = 0.02            # commanded ratio must exceed the measured ratio by this much: the ground gave out
DECEL_TOL = 0.02            # [m/s] over one window; a foot coasting on momentum slows far more (about 0.18)
LIMIT_FRACTION_OF_CONE = 0.95
CLEARANCE_EVERY = 10        # evaluate clearances every 10 steps (20 ms)


# ---------------------------------------------------------------- geometry helpers
def surface_frame(course: Course, x: float):
    """Unit normal n and tangents (t1 along the course, t2 across it) of the surface at plan position x."""
    th = course.theta if course.x1 < x < course.x2 else 0.0
    n = np.array([-math.sin(th), 0.0, math.cos(th)])
    t1 = np.array([math.cos(th), 0.0, math.sin(th)])
    return n, t1, np.array([0.0, 1.0, 0.0])


def _point_box_distance(px, py, o):
    (cx, cy), (L, W) = o["center"], o["size"]
    dx = np.maximum(np.abs(px - cx) - L / 2, 0.0)
    dy = np.maximum(np.abs(py - cy) - W / 2, 0.0)
    return np.hypot(dx, dy)


def _rect_corners(x, y, yaw, length=ROBOT_LENGTH, width=ROBOT_WIDTH):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(length / 2, width / 2), (-length / 2, width / 2), (-length / 2, -width / 2), (length / 2, -width / 2)]
    return [(x + c * a - s * b, y + s * a + c * b) for a, b in pts]


def body_clearance(course: Course, obstacles, x, y, yaw) -> float:
    """Distance from the body rectangle to the nearest wall face or obstacle (0 if touching or overlapping)."""
    from shapely.geometry import Polygon, box
    corners = _rect_corners(x, y, yaw)
    wall = min(float(course.wall_clearance(px, py)) for px, py in corners)
    body = Polygon(corners)
    obs = [body.distance(box(o["center"][0] - o["size"][0] / 2, o["center"][1] - o["size"][1] / 2,
                             o["center"][0] + o["size"][0] / 2, o["center"][1] + o["size"][1] / 2)) for o in obstacles]
    return max(0.0, min([wall] + obs)) if wall > 0 else wall


def circle_slack(course: Course, obstacles, x, y, yaw) -> float:
    """Smallest distance from a footprint circle center to a wall or obstacle, minus radius + safety margin."""
    cx = x + CIRCLE_OFFSETS * math.cos(yaw)
    cy = y + CIRCLE_OFFSETS * math.sin(yaw)
    d = np.min(course.wall_clearance(cx, cy))
    for o in obstacles:
        d = min(d, float(np.min(_point_box_distance(cx, cy, o))))
    return float(d) - (CIRCLE_RADIUS + PLAN_SAFETY_MARGIN)


# ---------------------------------------------------------------- slips
def _stances(contact):
    """(start, end) index pairs of each contact phase of one foot; end is exclusive."""
    c = np.asarray(contact, dtype=bool)
    edges = np.flatnonzero(np.diff(c.astype(int))) + 1
    starts = list(edges[c[edges]]) + ([0] if c[0] else [])
    out = []
    for s in sorted(starts):
        e = s
        while e < len(c) and c[e]:
            e += 1
        out.append((int(s), int(e)))
    return out


def foot_signals(log, course: Course, leg: int):
    """Per step for one foot: speed along the surface, measured and commanded normal force and friction ratio,
    and the commanded force's largest per-axis ratio (what the friction pyramid limits)."""
    pos, vel = log["feet_pos"][:, leg], log["feet_vel"][:, leg]
    grf, cmd = log["feet_grf"][:, leg], log["mpc_grf"][:, leg]
    n_steps = len(pos)
    out = {k: np.full(n_steps, np.nan) for k in ("vt", "fn", "r", "cfn", "cr", "c_axis")}
    for k in range(n_steps):
        n, t1, t2 = surface_frame(course, pos[k, 0])
        v = vel[k]
        out["vt"][k] = np.linalg.norm(v - (v @ n) * n)
        fn = grf[k] @ n
        out["fn"][k] = fn
        if fn > FN_MIN:
            out["r"][k] = np.linalg.norm(grf[k] - fn * n) / fn
        cfn = cmd[k] @ n
        out["cfn"][k] = cfn
        if cfn > CMD_FN_MIN:
            out["cr"][k] = np.linalg.norm(cmd[k] - cfn * n) / cfn
            out["c_axis"][k] = max(abs(cmd[k] @ t1), abs(cmd[k] @ t2)) / cfn
    return out


def slip_windows(sig, contact):
    """Force-driven slips of one foot: list of (step index of onset, measured ratio over the first window).

    A window of SLIP_WINDOW_STEPS steps is a slip when, for every step, the foot is in contact, the controller
    commands stance (normal force > CMD_FN_MIN), the measured normal force is above FN_MIN and the foot moves faster
    than V_SLIP along the surface; and, over the window, the commanded friction ratio exceeds the measured one by
    RATIO_GAP (the ground gave less than asked) and the foot did not slow by more than DECEL_TOL (not coasting on
    landing momentum). One slip at most per stance: its onset and the measured ratio, which is the friction label.
    """
    w = SLIP_WINDOW_STEPS
    found = []
    for s, e in _stances(contact):
        for k in range(s, e - w + 1):
            sl = slice(k, k + w)
            if not (np.all(sig["vt"][sl] > V_SLIP) and np.all(sig["cfn"][sl] > CMD_FN_MIN)
                    and np.all(sig["fn"][sl] > FN_MIN)):
                continue
            r_meas, r_cmd = np.nanmean(sig["r"][sl]), np.nanmean(sig["cr"][sl])
            slowing = sig["vt"][k + w - 1] - sig["vt"][k]
            if r_cmd - r_meas > RATIO_GAP and slowing > -DECEL_TOL:
                found.append((k, float(r_meas)))
                break
    return found


# ---------------------------------------------------------------- one run
def run_metrics(log, cfg: dict, job: dict) -> dict:
    """Metrics for one simulated run. log: the loaded run_waypoints .npz (or a dict with the same keys)."""
    course = Course(cfg["course"])
    obstacles = cfg.get("obstacles", [])
    t = np.asarray(log["t"])
    pos, yaw = np.asarray(log["base_pos"]), np.asarray(log["yaw"])
    raw = str(log["summary_outcome"])
    hits = [str(b) for b in np.asarray(log["wall_hit_barrier"]).ravel()]
    if hits:
        outcome = "collision"
    elif raw == "terminated":
        outcome = "fall"
    elif raw == "goal":
        outcome = "success"
    else:
        outcome = "timeout"

    m = dict(outcome=outcome, reached_goal=raw == "goal", sim_time=float(t[-1] - t[0]),
             time_to_goal=float(t[-1] - t[0]) if outcome == "success" else None,
             collision_steps=len(hits), collided_with=sorted(set(hits)))
    up = np.flatnonzero(pos[:, 0] >= course.x1)
    top = np.flatnonzero(pos[:, 0] >= course.x2)
    m["ramp_time"] = float(t[top[0]] - t[up[0]]) if len(up) and len(top) and course.theta > 0 else None

    idx = range(0, len(t), CLEARANCE_EVERY)
    m["body_clearance"] = float(min(body_clearance(course, obstacles, pos[k, 0], pos[k, 1], yaw[k]) for k in idx))
    slack = min(circle_slack(course, obstacles, pos[k, 0], pos[k, 1], yaw[k]) for k in idx)
    m["circle_slack"] = float(slack)
    buffer = job.get("tube_margin", 0.0)
    m["tube_used"] = float(1.0 - slack / buffer) if buffer > 0 else None

    slips, limit_steps, stance_steps, slip_labels = 0, 0, 0, []
    contact = np.asarray(log["feet_contact"])
    cone = job["cone_mu"]
    for leg in range(4):
        sig = foot_signals(log, course, leg)
        found = slip_windows(sig, contact[:, leg])
        slips += len(found)
        slip_labels += [r for _, r in found]
        commanded = contact[:, leg] & (sig["cfn"] > CMD_FN_MIN)
        stance_steps += int(commanded.sum())
        limit_steps += int(np.sum(commanded & (sig["c_axis"] >= LIMIT_FRACTION_OF_CONE * cone)))
    m["slips"] = slips
    m["slip_ratio_median"] = float(np.median(slip_labels)) if slip_labels else None
    m["limit_fraction"] = float(limit_steps / stance_steps) if stance_steps else None
    return m


def no_plan_metrics(reason: str) -> dict:
    return dict(outcome="no_feasible_plan", reached_goal=False, plan_failure=reason)
