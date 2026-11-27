"""Temporary bridge from a course file to Track B's collocation planner, so full runs work before stage 2 integration.

The patches here replace work that Track B and Track C still own, without editing their code:
  * one tube-inflated pass with the three-circle footprint, instead of the two-pass planner
  * the three-circle settings passed explicitly, because the planner's default is still one circle
  * the yaw-error margin added to the planner's tube margin, because the planner has no heading term
  * a constant tube computed directly, instead of predict_contacts
  * acceleration limited through environment.friction_mu
  * one speed limit for the whole course, set by the ramp, because the planner has no per-section speed limit
  * T from the route length, retried if the solve fails
  * every plan checked on a 0.02 s grid, with the margin raised if it cuts a constraint between samples
  * yaw from the heading between knots, only with the one-circle comparison footprint
  * "no feasible plan" returned as a status
  * bounds and obstacles read from the course file in whatever frame it uses
"""
import math
import os
import sys
from contextlib import contextmanager
from dataclasses import dataclass, field

import numpy as np
import yaml

from planning.direct_collocation import plan_trajectory
from planning.footprint_astar import astar_three_circle_path
from planning.planner_helpers import astar_initial_path, polygon_halfspaces
from planning.robust_control.friction_tube import compute_friction_tube_position_radii
from planning.robust_control.tube_error_model import design_tube_error_model
from trackA.mid_layer.reference import PlannerReference, waypoint_yaw_reference
from trackC.course import Course

KNOT_DT = 0.4           # [s] a multiple of the QP step, so the reference is exact on the QP grid
NOMINAL_SPEED = 0.30    # [m/s] average speed used to pick T on a flat course
NOMINAL_SPEED_RAMP = 0.15   # [m/s] same, on a course with a ramp, where the low level is much slower
RETRY_TIME = 4.0        # [s] added to T when a solve fails
MAX_ATTEMPTS = 4
CHECK_DT = 0.02         # [s]


class NoFeasiblePlan(RuntimeError):
    """The course has no route for this footprint and tube."""


@dataclass
class CourseGeometry:
    """What the planner needs from a course file. Has the attributes plan_trajectory reads from an environment."""
    name: str
    course: Course
    bounds: tuple
    obstacles: list
    goal: np.ndarray
    friction_mu: float = 0.5

    def slope_at(self, x: float) -> float:
        return self.course.theta if self.course.x1 <= x <= self.course.x2 else 0.0


@dataclass
class Plan:
    reference: PlannerReference
    geometry: CourseGeometry
    tube_radius: float
    circle_radius: float
    T: float
    min_clearance: float          # smallest slack to the inflated constraints on the 0.02 s grid [m]
    attempts: int
    info: dict = field(default_factory=dict)


def load_course(path) -> CourseGeometry:
    with open(path) as f:
        cfg = yaml.safe_load(f)
    c = cfg["course"]
    course = Course(c)
    obstacles = []
    for o in cfg.get("obstacles", []):
        (x, y), (length, width) = o["center"], o["size"]
        xa, xb, ya, yb = x - length / 2, x + length / 2, y - width / 2, y + width / 2
        obstacles.append(np.array([[xa, ya], [xb, ya], [xb, yb], [xa, yb]], dtype=float))  # counter-clockwise
    bounds = (course.x0, course.x3, -course.width / 2, course.width / 2)
    return CourseGeometry(cfg["name"], course, bounds, obstacles, np.asarray(c["goal"], dtype=float))


def planner_friction(theta: float, cfg) -> float:
    """friction_mu that makes ||a|| <= mu g satisfy the turn limit and the uphill limit everywhere."""
    mu = min(cfg.mu_lo - cfg.delta, cfg.mu_lo * math.cos(theta) - math.sin(theta))
    if mu <= 0.0:
        raise NoFeasiblePlan(f"no acceleration margin at mu_lo = {cfg.mu_lo} on a {math.degrees(theta):.1f} deg slope")
    return math.floor(mu * 100 + 1e-9) / 100


def constant_tube_radius(T: float, cfg, model=None) -> float:
    """Largest tube radius [m] over T for the constant friction gap. Total normal force is m g, so the mass cancels."""
    model = model if model is not None else design_tube_error_model(cfg.dt_fb)
    steps = max(1, int(round(T / cfg.dt_fb)))
    mass = 1.0
    radii = compute_friction_tube_position_radii(
        stance_mask=np.ones((steps, 4), dtype=int),
        normal_forces=np.full((steps, 4), mass * cfg.gravity / 4),
        mu_plan=cfg.mu_plan, mu_lo=cfg.mu_lo, mass=mass,
        B=model["B"], P=model["P"], rho=model["rho"], r0=cfg.r0,
    )
    return float(radii.max())


def clearance(reference: PlannerReference, geometry: CourseGeometry, radius: float, dt: float = CHECK_DT,
              circle_offsets=(0.0,)) -> float:
    """Smallest slack of every footprint circle to the walls and obstacles inflated by radius, sampled every dt."""
    x_lo, x_hi, y_lo, y_hi = geometry.bounds
    faces = [polygon_halfspaces(o, radius) for o in geometry.obstacles]
    times = np.arange(0.0, reference.t_end + 1e-9, dt)
    p = np.array([reference.state(s)[:2] for s in times])
    yaw = np.array([reference.yaw(s) for s in times])
    along = np.column_stack((np.cos(yaw), np.sin(yaw)))
    best = np.inf
    for s in np.asarray(circle_offsets, dtype=float):
        c = p + s * along
        best = min(best, float(np.min([c[:, 0] - x_lo, x_hi - c[:, 0], c[:, 1] - y_lo, y_hi - c[:, 1]]) - radius))
        for normals, offsets in faces:
            best = min(best, float(np.min(np.max(c @ normals.T - offsets, axis=1))))
    return best


@contextmanager
def _quiet():
    """Silence IPOPT, which prints from C."""
    sys.stdout.flush()
    saved, null = os.dup(1), os.open(os.devnull, os.O_WRONLY)
    os.dup2(null, 1)
    try:
        yield
    finally:
        sys.stdout.flush()
        os.dup2(saved, 1)
        os.close(null)
        os.close(saved)


def route_length(start, goal, geometry: CourseGeometry, tube: float, cfg) -> float:
    """Length of the A* route for the configured footprint, with the tube and the yaw margin."""
    start_state, goal_state = np.r_[start, 0.0, 0.0], np.r_[goal, 0.0, 0.0]
    if cfg.footprint == "three_circle":
        path = astar_three_circle_path(
            start_state, goal_state, geometry.bounds, geometry.obstacles, cfg.robot_length, cfg.robot_width,
            safety_margin=cfg.r_safety + tube + cfg.heading_margin, resolution=0.1, extra_clearance=0.01,
        )
    else:
        inflation = cfg.robot_radius + tube
        x_lo, x_hi, y_lo, y_hi = geometry.bounds
        path = astar_initial_path(
            start_state=start_state, goal_state=goal_state,
            bounds=(x_lo + inflation, x_hi - inflation, y_lo + inflation, y_hi - inflation),
            obstacles=geometry.obstacles, inflation_radius=inflation, resolution=0.1, extra_clearance=0.01,
        )
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def plan_for_course(course_path, start_xy, start_yaw, cfg, model=None) -> Plan:
    """Plan from the measured start to the course goal. Raises NoFeasiblePlan if no route is found."""
    geometry = load_course(course_path)
    geometry.friction_mu = planner_friction(geometry.course.theta, cfg)
    start, goal = np.asarray(start_xy, dtype=float), geometry.goal
    model = model if model is not None else design_tube_error_model(cfg.dt_fb)
    r_extra = 0.0

    try:
        L = route_length(start, goal, geometry, constant_tube_radius(60.0, cfg, model), cfg)
    except (ValueError, RuntimeError) as err:
        raise NoFeasiblePlan(f"no route on {geometry.name}: {err}") from err
    has_ramp = geometry.course.theta > 0.0
    speed_limit = cfg.v_ramp if has_ramp else cfg.v_max
    T = math.ceil(L / (NOMINAL_SPEED_RAMP if has_ramp else NOMINAL_SPEED) / KNOT_DT) * KNOT_DT

    last = "not attempted"
    for attempt in range(1, MAX_ATTEMPTS + 1):
        tube = constant_tube_radius(T, cfg, model)
        planner_cfg = {
            "N": int(round(T / KNOT_DT)), "T": T, "max_speed": speed_limit, "smoothness_weight": 1.0,
            "robot_length": cfg.robot_length, "robot_width": cfg.robot_width, "r_safety": cfg.r_safety + r_extra,
            "astar_resolution": 0.1, "astar_clearance": 0.01, "footprint_model": cfg.footprint,
            "tube_position_margin_m": tube + cfg.heading_margin,
        }
        if cfg.footprint == "three_circle":
            planner_cfg.update(footprint_yaw_mode="path_tangent", footprint_yaw=float(start_yaw),
                               footprint_max_yaw_rate=cfg.yaw_ref_rate)
        try:
            with _quiet():
                result = plan_trajectory(np.r_[start, 0.0, 0.0], np.r_[goal, 0.0, 0.0], geometry, planner_cfg)
        except (ValueError, RuntimeError) as err:
            raise NoFeasiblePlan(f"planner rejected {geometry.name}: {err}") from err
        if not result["success"]:
            last = result["return_status"]
            T += RETRY_TIME
            continue

        h = result["t"][1] - result["t"][0]
        if cfg.footprint == "three_circle":     # the yaw the planner used for its own collision checks
            yaw = np.unwrap(result["yaw_ref"])
        else:
            yaw = waypoint_yaw_reference(result["x"][:, :2], h, start_yaw, cfg.yaw_ref_rate)
        reference = PlannerReference(result["t"], result["x"], result["u"], yaw)
        circle = cfg.robot_radius
        slack = clearance(reference, geometry, circle + tube + cfg.heading_margin, circle_offsets=cfg.circle_offsets)
        if slack < 0.0:                         # cut a constraint between samples: replan with more margin
            r_extra += -slack + 0.01
            last = f"cut the inflated constraints by {100 * -slack:.2f} cm between samples"
            continue
        return Plan(reference, geometry, tube, circle, T, slack, attempt,
                    {"status": result["return_status"], "r_extra": r_extra, "friction_mu": geometry.friction_mu})
    raise NoFeasiblePlan(f"no valid plan on {geometry.name} after {MAX_ATTEMPTS} attempts: {last}")
