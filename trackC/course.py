"""Course geometry: the flat -> ramp -> flat profile from the parameter sheet (v0.3).

Coordinates are plan view: x runs along the course (start to goal), y runs across it, z is up.
The robot spawns at the origin, so the lower flat starts a little behind x = 0 (the back-wall margin).

    z
    ^                                     upper flat (z = H)
    |                            x2 ___________________ x3
    |                             /
    |                   ramp    /   slope angle theta
    |                         /
    |  lower flat (z = 0)    /
    +---x0 ------------- x1 ------------------------------------> x

Every point on the course belongs to exactly one SECTION. Each section is a plane, so a friction region drawn in
plan view is "lifted" onto the course by giving each of its vertices the height z(x) of the section it lies in.
A region that crosses x1 or x2 (a crease) is cut there, and each piece is lifted onto its own plane.
"""
import math
from dataclasses import dataclass

from shapely.geometry import box


@dataclass(frozen=True)
class Section:
    name: str      # "lower", "ramp" or "upper"
    xa: float      # plan-view start of the section
    xb: float      # plan-view end of the section


class Course:
    def __init__(self, c: dict):
        self.width = c["width"]
        self.theta = math.radians(c["ramp"]["angle_deg"])
        self.x0 = c["lower_flat"]["start"]
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
        return box(s.xa, -self.width / 2, s.xb, self.width / 2)

    def course_rect(self):
        return box(self.x0, -self.width / 2, self.x3, self.width / 2)

    def summary(self) -> str:
        return (f"lower flat x [{self.x0:.2f}, {self.x1:.2f}] | ramp x [{self.x1:.2f}, {self.x2:.2f}] "
                f"rising {self.H:.3f} m at {math.degrees(self.theta):.1f} deg | "
                f"upper flat x [{self.x2:.2f}, {self.x3:.2f}] | width {self.width} m")
