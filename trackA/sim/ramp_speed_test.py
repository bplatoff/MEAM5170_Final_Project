"""Measure the forward speed Quadruped-PyMPC actually reaches on the 10 degree ramp for each commanded speed.

The robot walks to the ramp, then holds each command for several gait cycles while a small loop keeps y and yaw near 0.
The steady speed is averaged over whole gait cycles.

Run from the repo root, inside the pixi env:
    python -m trackA.sim.ramp_speed_test
"""
import argparse
import json
from datetime import datetime
from pathlib import Path

import numpy as np

from trackA.sim.run_midlayer import ensure_scene

COURSE = "trackA/courses/midlayer_ramp_v1.yaml"
GAIT_PERIOD = 1.0 / 1.4
RAMP_START, RAMP_END = 2.3, 7.7     # [m] measure only well inside the ramp
COMMANDS = (0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60)
HOLD_CYCLES = 7
SETTLE_CYCLES = 3                   # cycles discarded at the start of each hold


class RampCommand:
    log_keys = ("ramp_cmd", "ramp_level")

    def __init__(self, spawn_xy):
        self.waypoints = np.array([spawn_xy, [RAMP_END, 0.0]])
        self.seg_idx, self.cross_track_error = 0, 0.0
        self.t_ramp, self.cmd, self.level = None, 0.3, -1

    def log_row(self):
        return {"ramp_cmd": self.cmd, "ramp_level": self.level}

    def update(self, pos_xy, vel_xy, yaw, dt):
        if self.t_ramp is None and pos_xy[0] >= RAMP_START:
            self.t_ramp = 0.0
        self.level, self.cmd = -1, 0.3
        if self.t_ramp is not None:
            self.level = int(self.t_ramp // (HOLD_CYCLES * GAIT_PERIOD))
            self.t_ramp += dt
            if self.level >= len(COMMANDS):
                return np.zeros(3), np.zeros(3), True
            self.cmd = COMMANDS[self.level]
        done = bool(pos_xy[0] >= RAMP_END)
        vy = float(np.clip(-1.0 * pos_xy[1], -0.15, 0.15))
        w = float(np.clip(-1.5 * yaw, -0.6, 0.6))
        return np.array([self.cmd, vy, 0.0]), np.array([0.0, 0.0, w]), done


def analyze(path):
    d = np.load(path, allow_pickle=False)
    dt = float(d["sim_dt"])
    level, x, v = d["ramp_level"].astype(int), d["base_pos"][:, 0], d["base_lin_vel"][:, 0]
    skip, rows = int(round(SETTLE_CYCLES * GAIT_PERIOD / dt)), []
    period = int(round(GAIT_PERIOD / dt))
    for k, c in enumerate(COMMANDS):
        idx = np.flatnonzero(level == k)
        if len(idx) <= skip + period:
            continue
        idx = idx[skip:]
        idx = idx[:len(idx) - len(idx) % period]    # whole gait cycles
        if x[idx].max() > RAMP_END:
            continue
        rows.append(dict(cmd=c, speed=float(v[idx].mean()), ratio=float(v[idx].mean() / c)))
    return rows


if __name__ == "__main__":
    from trackA.sim.run_waypoints import run
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mu", nargs="+", type=float, default=[0.3, 0.5])
    ap.add_argument("--out", default="trackA/results/step_test/ramp_speed.json")
    a = ap.parse_args()
    ensure_scene(COURSE)
    folder = Path("data/ramp_speed_test") / f"{datetime.now():%Y%m%d_%H%M%S}"
    result = {}
    for mu in a.mu:
        s = run(COURSE, timeout=90.0, render=False, log_dir=str(folder), mpc_kind="perfoot", mu_feet=[mu] * 4,
                tag=f"ramp_mu{int(round(mu * 100)):02d}", follower_factory=lambda xy, yaw: RampCommand(xy))
        result[f"mu_{mu:g}"] = dict(outcome=s["outcome"], levels=analyze(s["log"]))
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(result, indent=1))
    print(json.dumps(result, indent=1))
