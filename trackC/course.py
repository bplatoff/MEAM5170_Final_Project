"""Course geometry: the flat -> ramp -> flat profile from the parameter sheet (v0.3).

World frame (decision A1), shared by every module: the origin is the back-right inner wall corner on the
lower-flat surface. x runs forward along the course (horizontal distance, also on the ramp), y to the left, z up.
So the inner wall faces are x = 0 (back), x = x3 (end), y = 0 (right) and y = width (left), and the robot starts
wherever the course file's `start` says, not at the origin.

    side profile                                                      plan view (from above)
    z                                                                 y
    ^                                     upper flat (z = H)          ^  left wall, y = width
    |                            x2 ___________________ x3            |  +----------------------------+
    |                             /                                   |  |  start ->            goal  |
    |                   ramp    /   slope angle theta                 |  +----------------------------+
    |                         /                                       |  right wall, y = 0
    |  lower flat (z = 0)    /                                        +--x0 = 0 -------------- x3 --> x
    +---x0 = 0 --------- x1 ------------------------------------> x

Every point on the course belongs to exactly one SECTION. Each section is a plane, so a friction region drawn in
plan view is "lifted" onto the course by giving each of its vertices the height z(x) of the section it lies in.
A region that crosses x1 or x2 (a crease) is cut there, and each piece is lifted onto its own plane.
"""
import math
from dataclasses import dataclass

import numpy as np

from shapely.geometry import box


@dataclass(frozen=True)
class Section:
    name: str      # "lower", "ramp" or "upper"
    xa: float      # plan-view start of the section
    xb: float      # plan-view end of the section


class Course:
    def __init__(self, c: dict):
        if "start" in c["lower_flat"]:
            raise ValueError("lower_flat.start is from the old spawn-at-origin frame; the course now always begins at "
                             "x = 0 (decision A1). Remove it and shift start, goal, regions and obstacles instead.")
        self.width = c["width"]
        self.theta = math.radians(c["ramp"]["angle_deg"])
        self.x0 = 0.0                       # back wall face
        self.y0, self.y1 = 0.0, self.width  # right and left wall faces
        self.yc = self.width / 2            # centerline
        self.x1 = self.x0 + c["lower_flat"]["length"]
        run = c["ramp"]["length_along_slope"] * math.cos(self.theta)     # horizontal run of the ramp
        self.H = c["ramp"]["length_along_slope"] * math.sin(self.theta)  # height gained by the ramp
        self.x2 = self.x1 + run
        self.x3 = self.x2 + c["upper_flat"]["length"]
        self.sections = [Section("lower", self.x0, self.x1),
                         Section("ramp", self.x1, self.x2),
                         Section("upper", self.x2, self.x3)]

    # ---- heights -------------------------------------------------------------------------------------------------
    def z(self, x: float) -> float:
        """Surface height at plan-view position x."""
        if x <= self.x1:
            return 0.0
        if x >= self.x2:
            return self.H
        return (x - self.x1) * math.tan(self.theta)

    def section_at(self, x: float) -> Section:
        for s in self.sections:
            if s.xa <= x <= s.xb:
                return s
        raise ValueError(f"x = {x} is outside the course [{self.x0}, {self.x3}]")

    # ---- plan-view shapes ---------------------------------------------------------------------------------------
    def section_rect(self, s: Section):
        """The plan-view rectangle covered by one section (a shapely polygon)."""
        return box(s.xa, self.y0, s.xb, self.y1)

    def course_rect(self):
        return box(self.x0, self.y0, self.x3, self.y1)

    def wall_clearance(self, x, y):
        """Plan-view distance from (x, y) to the nearest inner wall face; negative means past that face.

        Works on scalars or numpy arrays of equal shape."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        return np.minimum.reduce([x - self.x0, self.x3 - x, y - self.y0, self.y1 - y])

    def summary(self) -> str:
        return (f"lower flat x [{self.x0:.2f}, {self.x1:.2f}] | ramp x [{self.x1:.2f}, {self.x2:.2f}] "
                f"rising {self.H:.3f} m at {math.degrees(self.theta):.1f} deg | "
                f"upper flat x [{self.x2:.2f}, {self.x3:.2f}] | y [{self.y0:.2f}, {self.y1:.2f}]")
