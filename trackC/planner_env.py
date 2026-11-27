"""Course file -> global planner input (task 2.3e, decision A1).

The planner (planning/direct_collocation.py) works in plan view in the world frame and takes a MockEnvironment:
bounds (the inner wall faces), obstacle polygons and one friction value for its acceleration limit. This module
builds that from a course YAML, through the same loader as the terrain generator, so the planner sees exactly the
obstacles MuJoCo builds, including the mid-ramp barrier for a given --gap.

    from trackC.planner_env import planner_problem
    p = planner_problem("trackC/courses/mixed_v1.yaml", gap=0.8)
    result = plan_trajectory(p.start_state, p.goal_state, p.environment, config)

Obstacles on the ramp are used in plan view, like everything else in the planner. Walls are not obstacles here:
the planner keeps the footprint inside the bounds.
"""
from dataclasses import dataclass

import numpy as np

from planning.mock_environment import MockEnvironment, rectangle
from trackC.course import Course
from trackC.course_spec import MU_NOMINAL, load_course_cfg


@dataclass
class PlannerProblem:
    environment: MockEnvironment
    start_state: np.ndarray        # [px, py, vx, vy], at rest
    goal_state: np.ndarray         # [px, py, vx, vy], at rest
    course: Course                 # geometry: sections, slope, wall faces
    cfg: dict                      # the expanded course dict (barrier and gap patch included)


def obstacle_polygons(cfg: dict) -> list:
    """Each obstacle box as a counter-clockwise (4, 2) polygon in plan view."""
    out = []
    for o in cfg.get("obstacles", []):
        (x, y), (L, W) = o["center"], o["size"]
        out.append(rectangle(x - L / 2, x + L / 2, y - W / 2, y + W / 2))
    return out


def planner_environment(cfg: dict, mu=MU_NOMINAL) -> MockEnvironment:
    """MockEnvironment for an already-loaded course dict. mu: friction for the planner's acceleration limit (A5:
    0.5 for the robust baseline)."""
    course = Course(cfg["course"])
    return MockEnvironment(x_min=course.x0, x_max=course.x3, y_min=course.y0, y_max=course.y1,
                           friction_mu=mu, obstacles=obstacle_polygons(cfg), name=cfg["name"])


def planner_problem(source, gap=None, variant=None, start=None, mu=MU_NOMINAL) -> PlannerProblem:
    """Everything the planner needs for one run.

    source: course YAML path or dict. gap, variant: mid-ramp overrides (as in terrain_gen). start: (x, y) to plan
    from, e.g. the robot's actual spawn position; defaults to the course file's start.
    """
    cfg = load_course_cfg(source, gap=gap, variant=variant)
    c = cfg["course"]
    sx, sy = start if start is not None else c["start"]
    gx, gy = c["goal"]
    return PlannerProblem(environment=planner_environment(cfg, mu),
                          start_state=np.array([sx, sy, 0.0, 0.0], dtype=float),
                          goal_state=np.array([gx, gy, 0.0, 0.0], dtype=float),
                          course=Course(c), cfg=cfg)
