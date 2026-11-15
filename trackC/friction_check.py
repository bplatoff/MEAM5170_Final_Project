"""Measure the friction MuJoCo actually applies on every surface piece of a course, flat or ramp.

How it works: drop a small block (no priority, friction 0.6 like the Go2 feet) on a point inside the piece, slowly
ramp a force on it until it slides, and convert the force at slip into a friction coefficient.

  flat ground:  slip when F = mu m g                          ->  mu = F / (m g)
  ramp (theta): push DOWN the slope; slip when
                F + m g sin(theta) = mu m g cos(theta)        ->  mu = (F + m g sin(theta)) / (m g cos(theta))

A block is used instead of a foot-sized sphere because a sphere rolls, which hides sliding. Friction combination in
MuJoCo depends only on the two geoms' friction and priority, so the result applies to the feet. If priorities were
wrong, every surface would measure 0.6 (the block's own friction) instead of its set value.

Usage: python friction_check.py courses/debug_v03.yaml [--seed N]
"""
import argparse
import math

import mujoco
import numpy as np
import yaml

from terrain_gen import build

S, M, G = 0.02, 1.0, 9.81      # probe half-size [m], mass [kg], gravity


def measure(scene_xml, course, x, y):
    sec = course.section_at(x)
    th = course.theta if sec.name == "ramp" else 0.0
    n = np.array([-math.sin(th), 0.0, math.cos(th)])          # surface normal
    down = np.array([-math.cos(th), 0.0, -math.sin(th)]) if th else np.array([1.0, 0.0, 0.0])
    p = np.array([x, y, course.z(x)]) + n * (S + 0.002)
    q = (math.cos(-th / 2), 0.0, math.sin(-th / 2), 0.0)       # pitch the block to sit flat on the ramp
    probe = (f'<body name="probe" pos="{p[0]} {p[1]} {p[2]}" quat="{q[0]} {q[1]} {q[2]} {q[3]}"><freejoint/>'
             f'<geom type="box" size="{S} {S} {S}" mass="{M}" friction="0.6 0.005 0.0001" margin="0.001"/></body>\n')
    xml = scene_xml.replace("  </worldbody>", "    " + probe + "  </worldbody>")
    m = mujoco.MjModel.from_xml_string(xml)
    m.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    d = mujoco.MjData(m)
    bid = m.body("probe").id
    for _ in range(300):                                      # settle
        mujoco.mj_step(m, d)
    if np.linalg.norm(d.qvel[:3]) > 0.01:                     # already sliding under gravity alone
        return float("nan"), "slides under gravity"
    t0 = d.time
    while d.time - t0 < 30:
        F = 0.5 * (d.time - t0)                                # [N] force ramp, 0.5 N/s
        d.xfrc_applied[bid, :3] = F * down
        mujoco.mj_step(m, d)
        if np.dot(d.qvel[:3], down) > 0.01:
            return (F + M * G * math.sin(th)) / (M * G * math.cos(th)), sec.name
    return float("nan"), "no slip within 30 s"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("config")
    ap.add_argument("--seed", type=int, default=None)
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    xml, info = build(cfg, a.seed)
    course, mu = info["course"], info["mu"]
    print(f"{'surface':<14}{'section':<9}{'mu set':>8}{'measured':>10}")
    for sid, sec, shape in info["pieces"]:
        pt = shape.representative_point()                     # a point guaranteed to lie inside the piece
        got, where = measure(xml, course, pt.x, pt.y)
        flag = "" if abs(got - mu[sid]) < 0.05 else "   <-- check"
        print(f"{sid:<14}{sec.name:<9}{mu[sid]:>8.2f}{got:>10.3f}{flag}")
