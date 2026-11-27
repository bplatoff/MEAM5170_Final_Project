"""Seeded start poses (decision A1.1) and footprint clearance to the walls.

Start pose. Each course file has a `start_zone` on the lower flat:

    start_zone: {x: [0.8, 1.2], y: [0.9, 2.1], yaw_deg: 15}

`sample_start(cfg, seed)` draws x, y and yaw uniformly from it. The start seed is its own number, separate from the
friction seed, so the batch runner can vary one while holding the other. The zone is checked once when the course is
built (terrain_gen), so every draw is valid and no draw is rejected: the distribution stays uniform.

The zone spans the 1.2 m gateway's opening (y 0.9 to 2.1) and stays at least 1.2 m behind it. A wider first try
(x up to 1.4, y 0.6 to 2.4) passed every check here, but from a start tucked behind a gateway post the robust
planner's A* (full tube, yaw-rate limit) found no way to line up with the opening. All four corners of this zone at
+-15 deg plan successfully with the full tube on mixed_mu03_v1. The build checks cannot see A* reachability.

Footprint clearance. The robust planner keeps each of the three footprint circles (A2.1) at least
ROBUST_CIRCLE_CLEARANCE from every wall:

    circle radius 0.194 + safety margin 0.05 + tube 0.263 + heading allowance 0.023  =  0.53 m

The circles sit at 0 and +-0.233 m along the body axis, so with the robot facing along x the CoM must be
0.233 + 0.53 = 0.763 m from the back and end walls, and 0.53 m from the side walls. Start zones and goals are
checked against this for every yaw they can have. The tube and heading terms are provisional until task 2.5.
"""
import math
from dataclasses import dataclass

import numpy as np

from planning.robot_footprint import three_circle_parameters

ROBOT_LENGTH, ROBOT_WIDTH = 0.7, 0.31                 # same box as the planner's footprint
PLAN_SAFETY_MARGIN = 0.05                             # A2.3
ROBUST_TUBE_RADIUS = 0.263                            # constant tube at mu_plan 0.5, mu_lo 0.3 (Jacob's 2.4 runs)
HEADING_ALLOWANCE = 0.233 * 0.10                      # 0.233 m lever arm x eps_psi 0.10 rad, until 2.5 measures it
CIRCLE_OFFSETS, CIRCLE_RADIUS = three_circle_parameters(ROBOT_LENGTH, ROBOT_WIDTH)
ROBUST_CIRCLE_CLEARANCE = CIRCLE_RADIUS + PLAN_SAFETY_MARGIN + ROBUST_TUBE_RADIUS + HEADING_ALLOWANCE
GOAL_YAW_DEG = 15.0                                   # headings the robot may arrive at the goal with


@dataclass(frozen=True)
class StartPose:
    x: float
    y: float
    yaw: float          # [rad], world frame
    seed: int


def footprint_wall_clearance(course, x, y, yaw) -> float:
    """Smallest distance from any footprint circle's edge (radius + margin + tube + heading) to a wall face.
    Negative means the inflated footprint crosses a wall."""
    cx = x + CIRCLE_OFFSETS * math.cos(yaw)
    cy = y + CIRCLE_OFFSETS * math.sin(yaw)
    return float(np.min(course.wall_clearance(cx, cy))) - ROBUST_CIRCLE_CLEARANCE


def _box_distance(xa, xb, ya, yb, o) -> float:
    """Plan-view distance between the rectangle [xa, xb] x [ya, yb] and an obstacle box (0 if they overlap)."""
    (cx, cy), (L, W) = o["center"], o["size"]
    dx = max(cx - L / 2 - xb, xa - (cx + L / 2), 0.0)
    dy = max(cy - W / 2 - yb, ya - (cy + W / 2), 0.0)
    return math.hypot(dx, dy)


def check_start_and_goal(cfg, course) -> list:
    """Errors for a start zone or goal that breaks A1.1 / A3.3 or that the robust planner cannot use."""
    c = cfg["course"]
    errors = []
    z = c.get("start_zone")
    if z is None:
        return [f"course '{cfg['name']}' has no course.start_zone (decision A1.1)"]
    (xa, xb), (ya, yb), yaw_max = z["x"], z["y"], math.radians(z["yaw_deg"])
    if not (xa <= xb and ya <= yb and 0.0 <= yaw_max):
        errors.append(f"start_zone {z} is empty or has a negative yaw range")
    if not (course.x0 <= xa and xb <= course.x1):
        errors.append(f"start_zone x {z['x']} is not on the lower flat [{course.x0}, {course.x1}]")
    corners = [(x, y) for x in (xa, xb) for y in (ya, yb)]
    wall_min = min(float(course.wall_clearance(x, y)) for x, y in corners)
    if wall_min < c["wall_clearance"]:
        errors.append(f"start_zone comes {wall_min:.2f} m from a wall; need >= {c['wall_clearance']} m")
    for o in cfg.get("obstacles", []):
        d = _box_distance(xa, xb, ya, yb, o)
        if d < c.get("obstacle_clearance", 1.0):
            errors.append(f"start_zone comes {d:.2f} m from obstacle {o['id']}; need >= {c.get('obstacle_clearance', 1.0)} m")
    if c.get("robust_planning", True):
        # Clearance is smallest at a zone corner and at an extreme or zero yaw, so these cases cover the whole zone.
        for name, pts, yaw_lim in (("start_zone", corners, yaw_max), ("goal", [tuple(c["goal"])], math.radians(GOAL_YAW_DEG))):
            worst = min(footprint_wall_clearance(course, x, y, s * yaw_lim) for x, y in pts for s in (-1, 0, 1))
            if worst < 0.0:
                errors.append(f"{name}: the robust footprint (three circles, {ROBUST_CIRCLE_CLEARANCE:.3f} m each) "
                              f"crosses a wall by {-worst:.3f} m; move it inward")
    return errors


def sample_start(cfg, seed: int) -> StartPose:
    """Uniform draw from the course's start zone. The same seed always gives the same pose.

    The generator is seeded with (seed, a fixed tag), so a start seed never reproduces the friction draws of the
    same number."""
    z = cfg["course"]["start_zone"]
    rng = np.random.default_rng([int(seed), 0xA11])
    x = rng.uniform(*z["x"])
    y = rng.uniform(*z["y"])
    yaw = rng.uniform(-1.0, 1.0) * math.radians(z["yaw_deg"])
    return StartPose(float(x), float(y), float(yaw), int(seed))
