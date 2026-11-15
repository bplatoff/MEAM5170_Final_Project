"""Smoke tests for the course and the logger, without the MPC.

The Go2 is held in its standing pose with joint PD control, placed somewhere on the course, and optionally pushed.
Each test prints what the logger measured next to what physics says it should be:

  1. flat ice, pushed sideways  -> sliding force ratio r ~ mu of the ice (0.20)
  2. standing still on the ramp -> summed over all four feet, r ~ tan(10 deg) = 0.176: friction must hold the
                                   robot against gravity. This only comes out right if r is measured in the
                                   SURFACE frame. Individual feet read higher (about 0.24 even on flat ground)
                                   because the legs push against each other; those internal forces cancel in the
                                   sum. A single foot's ratio is still a valid lower bound on mu: it did not slip.
  3. ramp snow, pushed downhill -> sliding force ratio r ~ mu of the snow (0.30)
  4. next to a wall, pushed     -> the logger records a wall collision

Usage (from the repo root): python -m trackC.smoke_test trackC/courses/debug_v03.yaml
"""
import math
import sys

import numpy as np
import yaml
from gym_quadruped.quadruped_env import QuadrupedEnv

from trackC.contact_logger import ContactLogger
from trackC.terrain_gen import build, install


def run(env, course, x, y, push=(0, 0, 0), t_push=(0.8, 1.8), T=2.4):
    """Place the robot at plan position (x, y) on the surface, hold the stance, push the base during t_push."""
    env.reset(random=False)
    d, m = env.mjData, env.mjModel
    sec = course.section_at(x)
    th = course.theta if sec.name == "ramp" else 0.0
    d.qpos[0:3] = [x, y, course.z(x) + 0.27 / math.cos(th)]
    d.qpos[3:7] = [math.cos(-th / 2), 0, math.sin(-th / 2), 0]       # pitch the body to match the ramp
    d.qvel[:] = 0
    q_stand = m.key_qpos[0][7:19].copy()
    base = m.body("base").id
    log = ContactLogger(env)
    for _ in range(int(T / env.simulation_dt)):
        tau = 60.0 * (q_stand - d.qpos[7:19]) - 2.0 * d.qvel[6:18]
        on = t_push[0] < env.simulation_time < t_push[1]
        d.xfrc_applied[base, :3] = push if on else (0, 0, 0)
        env.step(action=tau)
        log.record(env)
    return log


def ratio(log, sliding, t_window=None):
    rows = [s for leg in log.steps for s in log.steps[leg]
            if s["contact"] and not np.isnan(s["r"]) and (s["v_t"] > 0.05) == sliding
            and (t_window is None or t_window[0] < s["t"] < t_window[1])]
    return np.median([s["r"] for s in rows]) if rows else float("nan"), len(rows)


def total_ratio(log, t0=0.8):
    """Force ratio of the whole robot: sum the four foot forces first, then split into normal and tangential."""
    out = []
    for i, s in enumerate(log.steps["FL"]):
        rows = [log.steps[leg][i] for leg in log.steps]
        if s["t"] < t0 or not all(r["contact"] for r in rows):
            continue
        F = sum(np.array([r["fx"], r["fy"], r["fz"]]) for r in rows)
        n = np.array([s["nx"], s["ny"], s["nz"]])
        fn = F @ n
        out.append(np.linalg.norm(F - fn * n) / fn)
    return float(np.median(out))


if __name__ == "__main__":
    cfg = yaml.safe_load(open(sys.argv[1]))
    xml, info = build(cfg)
    install(info["name"], xml)
    course = info["course"]
    env = QuadrupedEnv(robot="go2", scene=info["name"], ground_friction_coeff=0.5, ref_base_lin_vel=0.0)

    log = run(env, course, 1.2, -0.8, push=(0, 50, 0))
    r, n = ratio(log, sliding=True)
    print(f"1. flat ice (mu 0.20), pushed: sliding r = {r:.3f} over {n} foot-steps")
    ev = log.events()
    kinds = sorted({e["kind"] for e in ev})
    leaked = set().union(*(e.keys() for e in log.learner_events())) & {"surface", "straddle", "mu_true"}
    print(f"   stance events: {len(ev)}, kinds {kinds} (only hold/slip allowed), "
          f"straddles flagged {sum(e['straddle'] for e in ev)}, eval fields visible to learner: {sorted(leaked) or 'none'}")

    log = run(env, course, 4.6, 1.0, T=1.5)
    r_foot, _ = ratio(log, sliding=False, t_window=(0.8, 1.5))
    print(f"2. standing on dry ramp (mu 0.80): all feet r = {total_ratio(log):.3f}, expected tan(10 deg) = "
          f"{math.tan(course.theta):.3f} (single foot median {r_foot:.3f})")

    downhill = 60 * np.array([-math.cos(course.theta), 0, -math.sin(course.theta)])
    log = run(env, course, 6.3, -1.0, push=downhill)
    r, n = ratio(log, sliding=True)
    print(f"3. ramp snow (mu 0.30), pushed downhill: sliding r = {r:.3f} over {n} foot-steps")

    log = run(env, course, -0.3, 1.15, push=(0, 80, 0))
    walls = sorted({name for _, name in log.collisions})
    print(f"4. pushed toward the left wall: collisions with {walls}")
