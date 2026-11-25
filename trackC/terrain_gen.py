"""Generate the course from the parameter sheet (v0.3) as a MuJoCo scene for gym-quadruped / Quadruped-PyMPC.

Usage (from the repo root):
    python -m trackC.terrain_gen trackC/courses/debug_v03.yaml            # class values 0.3 / 0.5 / 0.8
    python -m trackC.terrain_gen trackC/courses/debug_v03.yaml --seed 7    # each class region draws from its set, seed 7
    python -m trackC.terrain_gen trackC/courses/mixed_v1.yaml --gap 0.8 --variant high   # override the mid-ramp gap
Then set Quadruped-PyMPC  simulation_params['scene'] = '<scene name printed by this script>'.

Pipeline (each step is one function below):
  0. load_course_cfg    (course_spec) check the friction rules, apply --gap / --variant, add the barrier and gap patch
  1. resolve_friction   (course_spec) each region's mu: its class value, an explicit mu, or a seeded draw from its class
  2. plan_pieces        in plan view, cut the course into non-overlapping pieces: (region or base) x (section)
  3. triangulate        split every piece into triangles (pieces can be concave or have holes; triangles are convex)
  4. prism_xml          lift each triangle onto its section's plane and extrude it 5 cm down: a convex mesh geom
  5. walls / obstacles  enclose all four sides; place obstacle boxes
  6. checks             ramp friction above tan(theta), start/goal clearance to walls and obstacles, full coverage

Rules carried over from the tests on the first starter (see README):
  * Terrain geoms are named "surf_<region>__<k>". All pieces of one region share <region>, so seams between them
    (triangle edges, creases) are not mistaken for friction boundaries by the contact logger.
  * Terrain geoms have priority="2" so their friction, not the foot's, is used in contacts.
  * The whole course is tiled; MuJoCo's floor plane sits 10 cm below the lower flat and is never touched.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from shapely import constrained_delaunay_triangles
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

from trackC.course import Course
from trackC.course_spec import load_course_cfg, resolve_friction, scene_name

TILE_DEPTH = 0.05   # [m] tiles are prisms extruded this far below the surface

HEADER = """<mujoco model="{name}">
  <statistic center="{cx:.2f} {cy:.2f} 0.5" extent="6"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.99 0.99 0.99" rgb2="0.99 0.99 0.99" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.93 0.93 0.93" rgb2="1 1 1"
             markrgb="1 1 1" width="200" height="200"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.05"/>
{meshes}  </asset>
  <worldbody>
    <light pos="{cx:.2f} {cy:.2f} 4" dir="0 0 -1" directional="true"/>
    <geom name="floor" pos="0 0 -0.1" size="0 0 0.05" type="plane" material="groundplane"/>
"""
FOOTER = """  </worldbody>
</mujoco>
"""


# ---------------------------------------------------------------- 1. friction
# Friction values come from trackC.course_spec.resolve_friction (decision C3: discrete set, lower limit 0.3).


# ---------------------------------------------------------------- 2. plan-view pieces
def plan_pieces(cfg, course: Course):
    """Cut the course into non-overlapping plan-view pieces, one list entry per (surface, section).

    Regions later in the YAML win where regions overlap. The base surface is whatever is left. Clipping every
    surface against each section rectangle is what splits a region at a crease.
    """
    course_rect = course.course_rect()
    claimed = Polygon()                               # area already given to a later region
    surfaces = []
    for r in reversed(cfg["regions"]):
        shape = Polygon(r["polygon"]).intersection(course_rect).difference(claimed)
        claimed = unary_union([claimed, shape])
        surfaces.append((r["id"], shape))
    surfaces.append(("base", course_rect.difference(claimed)))
    pieces = []
    for sid, shape in surfaces:
        for sec in course.sections:
            part = shape.intersection(course.section_rect(sec))
            if part.area > 1e-9:
                pieces.append((sid, sec, part))
    return pieces


# ---------------------------------------------------------------- 3. triangles
def triangulate(shape):
    """Constrained Delaunay triangulation: triangles that exactly cover `shape`, respecting its edges and holes."""
    tris = constrained_delaunay_triangles(shape)
    return [list(t.exterior.coords)[:3] for t in tris.geoms if t.area > 1e-10]


# ---------------------------------------------------------------- 4. prisms
def prism_xml(name, tri, course: Course, mu):
    """One triangle -> one convex mesh: top face lies on the course surface, bottom face TILE_DEPTH below it.

    Vertices are written in world coordinates relative to the triangle centroid, and the geom is placed at the
    centroid. (MuJoCo re-centers mesh vertices on their centroid, so this keeps world positions exact.)
    """
    top = [(x, y, course.z(x)) for x, y in tri]
    pts = top + [(x, y, z - TILE_DEPTH) for x, y, z in top]
    c = np.mean(pts, axis=0)
    verts = " ".join(f"{x - c[0]:.6f} {y - c[1]:.6f} {z - c[2]:.6f}" for x, y, z in pts)
    mesh = f'    <mesh name="m_{name}" vertex="{verts}"/>\n'
    geom = (f'    <geom name="{name}" type="mesh" mesh="m_{name}" pos="{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}" '
            f'priority="2" friction="{mu} 0.005 0.0001" rgba="{_rgba(mu)}"/>\n')
    return mesh, geom


def _rgba(mu):
    """Color by friction: pale blue = slippery, dark blue = grippy."""
    t = max(0.0, min(1.0, (mu - 0.1) / 0.8))
    return f"{0.85 - 0.6*t:.2f} {0.9 - 0.45*t:.2f} {1.0 - 0.2*t:.2f} 1"


# ---------------------------------------------------------------- 5. walls and obstacles
def _box(name, pos, half, quat=(1, 0, 0, 0), rgba="0.55 0.55 0.6 0.35"):
    return (f'    <geom name="{name}" type="box" pos="{pos[0]:.5f} {pos[1]:.5f} {pos[2]:.5f}" '
            f'size="{half[0]:.5f} {half[1]:.5f} {half[2]:.5f}" quat="{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} '
            f'{quat[3]:.6f}" rgba="{rgba}"/>\n')


def walls_xml(cfg, course: Course):
    """Four-sided enclosure. Each side wall is one box per section; the ramp's boxes are pitched to follow it.

    A wall spans 0.1 m below the surface to `height` above it. Flat-section walls are extended 0.15 m past the
    crease so the wedge-shaped gap where a flat wall meets a pitched one is closed.
    """
    h, t = cfg["course"]["walls"]["height"], cfg["course"]["walls"]["thickness"]
    out = []
    for side, y in (("left", course.y1 + t / 2), ("right", course.y0 - t / 2)):
        for sec in course.sections:
            if sec.name == "ramp":
                L = (sec.xb - sec.xa) / math.cos(course.theta)
                cx, cz0 = (sec.xa + sec.xb) / 2, course.z((sec.xa + sec.xb) / 2)
                nx, nz = -math.sin(course.theta), math.cos(course.theta)    # unit normal of the ramp
                off = (h - 0.1) / 2                                         # box center above the surface, along n
                q = (math.cos(-course.theta / 2), 0.0, math.sin(-course.theta / 2), 0.0)
                out.append(_box(f"wall_{side}_ramp", (cx + nx * off, y, cz0 + nz * off), (L / 2, t / 2, (h + 0.1) / 2), q))
            else:
                xa, xb = sec.xa - (0.15 if sec.name == "upper" else 0.0), sec.xb + (0.15 if sec.name == "lower" else 0.0)
                zs = course.z((sec.xa + sec.xb) / 2)
                out.append(_box(f"wall_{side}_{sec.name}", ((xa + xb) / 2, y, zs + (h - 0.1) / 2),
                                ((xb - xa) / 2, t / 2, (h + 0.1) / 2)))
    span = course.width / 2 + t     # half-length across the course, covering both side walls' ends
    out.append(_box("wall_back", (course.x0 - t / 2, course.yc, (h - 0.1) / 2), (t / 2, span, (h + 0.1) / 2)))
    out.append(_box("wall_end", (course.x3 + t / 2, course.yc, course.H + (h - 0.1) / 2), (t / 2, span, (h + 0.1) / 2)))
    return "".join(out)


def obstacles_xml(cfg, course: Course):
    """Axis-aligned vertical boxes. The bottom is buried 0.1 m below the lowest surface point under the box and the
    top sits `height` above the highest one, so obstacles on the ramp still reach the surface everywhere."""
    out = []
    for o in cfg.get("obstacles", []):
        (x, y), (L, W) = o["center"], o["size"]
        zlo, zhi = course.z(x - L / 2) - 0.1, course.z(x + L / 2) + o["height"]
        out.append(_box(f"obstacle_{o['id']}", (x, y, (zlo + zhi) / 2), (L / 2, W / 2, (zhi - zlo) / 2),
                        rgba="0.35 0.35 0.38 1"))
    return "".join(out)


# ---------------------------------------------------------------- 6. checks
def checks(cfg, course: Course, mu, pieces):
    c = cfg["course"]
    errors = []
    covered = unary_union([p for _, _, p in pieces])
    if abs(covered.area - course.course_rect().area) > 1e-6:
        errors.append(f"pieces cover {covered.area:.4f} m^2 but the course is {course.course_rect().area:.4f} m^2")
    tan_t = math.tan(course.theta)
    for sid, sec, _ in pieces:
        if sec.name == "ramp" and mu[sid] <= tan_t:
            errors.append(f"region {sid} on the ramp: mu {mu[sid]} <= tan(theta) {tan_t:.3f}; the robot cannot stand")
    margin, ob_margin = c["wall_clearance"], c.get("obstacle_clearance", 1.0)
    for label, (x, y) in (("start", c["start"]), ("goal", c["goal"])):
        gap = float(course.wall_clearance(x, y))
        if gap < margin:
            errors.append(f"{label} at ({x}, {y}) is {gap:.2f} m from a wall; need >= {margin} m")
        for o in cfg.get("obstacles", []):
            d = obstacle_distance(o, x, y)
            if d < ob_margin:
                errors.append(f"{label} at ({x}, {y}) is {d:.2f} m from obstacle {o['id']}; need >= {ob_margin} m")
    return errors


def obstacle_distance(o, x, y) -> float:
    """Plan-view distance from (x, y) to an axis-aligned obstacle box (0 inside it)."""
    (cx, cy), (L, W) = o["center"], o["size"]
    dx, dy = max(abs(x - cx) - L / 2, 0.0), max(abs(y - cy) - W / 2, 0.0)
    return float((dx * dx + dy * dy) ** 0.5)


# ---------------------------------------------------------------- assemble
def build(cfg, seed=None, gap=None, variant=None):
    """cfg: a course dict or a path. gap, variant: override the course's mid-ramp gap (see course_spec)."""
    cfg = load_course_cfg(cfg, gap=gap, variant=variant)
    course = Course(cfg["course"])
    mu = resolve_friction(cfg, seed)
    pieces = plan_pieces(cfg, course)
    errs = checks(cfg, course, mu, pieces)
    if errs:
        raise ValueError("course checks failed:\n  " + "\n  ".join(errs))
    meshes, geoms, count = [], [], {}
    for sid, sec, shape in pieces:
        for tri in triangulate(shape):
            k = count.get(sid, 0)
            count[sid] = k + 1
            m, g = prism_xml(f"surf_{sid}__{k}", tri, course, mu[sid])
            meshes.append(m)
            geoms.append(g)
    name = scene_name(cfg, seed, overridden=cfg.get("_overridden", False))
    xml = (HEADER.format(name=name, meshes="".join(meshes), cx=course.x3 / 2, cy=course.yc) + "".join(geoms)
           + walls_xml(cfg, course) + obstacles_xml(cfg, course) + FOOTER)
    info = dict(name=name, course=course, mu=mu, pieces=pieces, n_tiles=len(geoms), cfg=cfg)
    return xml, info


def install(name, xml) -> Path:
    import gym_quadruped
    out = Path(gym_quadruped.__file__).parent / "robot_model" / f"scene_{name}.xml"
    out.write_text(xml)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("config")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--gap", type=float, default=None, help="mid-ramp gap width [m] (courses with mid_ramp_gap)")
    ap.add_argument("--variant", choices=["low", "high"], default=None, help="friction class of the gap patch")
    ap.add_argument("--out", help="write the XML here instead of installing it into gym_quadruped")
    a = ap.parse_args()
    xml, info = build(a.config, a.seed, gap=a.gap, variant=a.variant)
    print(info["course"].summary())
    print(f"{len(info['mu'])} surfaces, {info['n_tiles']} tiles; friction: "
          + ", ".join(f"{k}={v}" for k, v in info["mu"].items()))
    path = Path(a.out) if a.out else install(info["name"], xml)
    if a.out:
        path.write_text(xml)
    print(f"wrote {path}\nset Quadruped-PyMPC simulation_params['scene'] = '{info['name']}'")
