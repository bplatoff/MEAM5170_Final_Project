"""Conservative planar three-circle footprint for the quadruped body.

The circles cover the entire length x width rectangle in the robot body
frame, plus an optional safety margin. The caller supplies the robot yaw;
the current direct-collocation state does not include yaw yet.
"""

import casadi as ca
import numpy as np

from planning.planner_helpers import polygon_halfspaces


def three_circle_offsets(length):
    """Longitudinal body-frame positions of rear, center and front circles."""
    if not np.isfinite(length) or length <= 0:
        raise ValueError("length must be finite and strictly positive")
    return np.array([-length / 3.0, 0.0, length / 3.0])


def three_circle_parameters(length, width, safety_margin=0.0):
    """Return longitudinal circle offsets [m] and common radius [m].

    Three equally spaced centers at -L/3, 0, +L/3 guarantee that every
    point in the L-by-W body rectangle is within at least one circle of
    radius hypot(L/6, W/2). Adding safety_margin inflates each circle.
    """
    values = np.asarray([length, width, safety_margin], dtype=float)
    if not np.all(np.isfinite(values)) or length <= 0 or width <= 0 or safety_margin < 0:
        raise ValueError("length and width must be positive; safety_margin must be nonnegative")

    offsets = three_circle_offsets(length)
    radius = float(np.hypot(length / 6.0, width / 2.0) + safety_margin)
    return offsets, radius


def three_circle_centers(position_xy, yaw, length):
    """Return shape (3, 2) numeric circle-center coordinates for yaw [rad]."""
    position = np.asarray(position_xy, dtype=float)
    if position.shape != (2,) or not np.all(np.isfinite(position)):
        raise ValueError("position_xy must be a finite [x, y] vector")
    if not np.isscalar(yaw) or not np.isfinite(yaw):
        raise ValueError("yaw must be a finite scalar in radians")
    offsets = three_circle_offsets(length)
    direction = np.array([np.cos(yaw), np.sin(yaw)])
    return position[None, :] + offsets[:, None] * direction[None, :]


def three_circle_centers_symbolic(position_xy, yaw, length):
    """Return a CasADi (3,2) matrix with differentiable circle centers."""
    offsets = three_circle_offsets(length)
    direction = ca.vertcat(ca.cos(yaw), ca.sin(yaw))
    columns = [ca.vertcat(position_xy[0], position_xy[1]) + offset * direction
               for offset in offsets]
    return ca.horzcat(*columns).T


def add_three_circle_constraints(
    position_xy, yaw, obstacles, bounds, length, width,
    safety_margin, beta, g, lbg, ubg,
):
    """Append wall and convex-polygon avoidance constraints for all circles.

    Inputs position_xy and yaw can be CasADi expressions. Obstacles must
    be CCW convex polygons. Each collision constraint is >= 0; the
    shifted smooth maximum underestimates the true maximum of the
    polygon's separating halfspaces, providing conservative avoidance.

    WARNING: The caller must supply a physically meaningful robot yaw.
    The existing four-state direct-collocation planner has no yaw state,
    so this function is intentionally not wired into that NLP yet.
    """
    if not np.isfinite(beta) or beta <= 0:
        raise ValueError("beta must be a finite positive scalar")
    if len(bounds) != 4 or not np.all(np.isfinite(bounds)):
        raise ValueError("bounds must be four finite numbers")
    x_min, x_max, y_min, y_max = bounds
    if x_min >= x_max or y_min >= y_max:
        raise ValueError("bounds must have increasing extents")

    _, radius = three_circle_parameters(length, width, safety_margin)
    centers = three_circle_centers_symbolic(position_xy, yaw, length)
    for i in range(3):
        cx, cy = centers[i, 0], centers[i, 1]
        for clearance in (
            cx - x_min - radius,
            x_max - radius - cx,
            cy - y_min - radius,
            y_max - radius - cy,
        ):
            g.append(clearance)
            lbg.append([0.0])
            ubg.append([np.inf])

        for obstacle in obstacles:
            normals, offsets = polygon_halfspaces(obstacle, radius)
            violations = ca.vertcat(*(
                normal[0] * cx + normal[1] * cy - b
                for normal, b in zip(normals, offsets)
            ))
            scaled = beta * violations
            shift = ca.mmax(scaled)
            conservative_smooth_max = (
                shift + ca.log(ca.sum1(ca.exp(scaled - shift)))
                - np.log(len(offsets))
            ) / beta
            g.append(conservative_smooth_max)
            lbg.append([0.0])
            ubg.append([np.inf])
