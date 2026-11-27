"""Step J2 of the mid-layer plan: measure how Quadruped-PyMPC responds to velocity commands.

Commands steps in forward velocity, lateral velocity and yaw rate on flat_v1, bypassing PathFollower, and fits
v_dot = (K v_cmd - v) / tau on each step. Both signals are averaged over one gait cycle first, which removes the trot
oscillation and leaves a linear fit unchanged because the same filter is applied to both.

Between steps the robot walks back to a home point away from the walls. Yaw is held at 0 during linear steps, so the
world x and y axes are the body's forward and lateral axes. Each size is commanded in both directions: half the
difference of the two steady speeds gives the gain with any constant drift removed.

Run from the repo root, inside the pixi env:
    python -m trackA.sim.step_test
    python -m trackA.sim.step_test --analyze data/step_test/<timestamp>     # re-run only the fits

Logs go to data/step_test/<timestamp>/ (ignored by git). The summary goes to trackA/results/step_test/.
"""
import argparse
import json
from datetime import datetime
from pathlib import Path

import numpy as np

GAIT_PERIOD = 1.0 / 1.4         # [s] trot step frequency in the Quadruped-PyMPC config
HOME = np.array([3.8, 1.5])     # [m] on flat_v1 (world frame, centerline), at least 1.5 m from every wall
WALK_HOME = 14.0                # [s] time to reach HOME before the first step
RECENTER_CYCLES = 4             # gait cycles walking back to HOME after a step
QUIET_CYCLES = 2                # gait cycles of zero command before the next step
K_HOME, V_HOME = 1.0, 0.3       # [1/s], [m/s]
K_YAW, W_HOLD = 1.5, 0.6        # [1/s], [rad/s]
AXES = {
    "forward": dict(index=0, sizes=(0.1, 0.2, 0.3, 0.4), hold_cycles=6),
    "lateral": dict(index=1, sizes=(0.1, 0.2, 0.3), hold_cycles=4),
    "yaw": dict(index=2, sizes=(0.2, 0.4, 0.6), hold_cycles=6),
}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def build_schedule():
    """List of (t_start, t_end, axis, value), each size commanded + then -."""
    t, out = WALK_HOME, []
    rest = (RECENTER_CYCLES + QUIET_CYCLES) * GAIT_PERIOD
    for axis, spec in AXES.items():
        hold = spec["hold_cycles"] * GAIT_PERIOD
        for size in spec["sizes"]:
            for value in (size, -size):
                out.append((t, t + hold, axis, value))
                t += hold + rest
    return out, t


class StepCommand:
    """Velocity-command source for run_waypoints: plays the schedule, recentering between steps."""
    log_keys = ("step_cmd", "step_active")

    def __init__(self, spawn_xy, schedule, t_end):
        self.schedule, self.t_end = schedule, t_end
        self.waypoints = np.array([spawn_xy, HOME])
        self.seg_idx, self.cross_track_error = 0, 0.0
        self.t = 0.0
        self.cmd = np.zeros(3)
        self.active = False

    def log_row(self):
        return {"step_cmd": self.cmd.copy(), "step_active": self.active}

    def _quiet(self):
        return any(t0 - QUIET_CYCLES * GAIT_PERIOD <= self.t < t0 for t0, _, _, _ in self.schedule) \
            or self.t >= self.schedule[-1][1] + RECENTER_CYCLES * GAIT_PERIOD

    def update(self, pos_xy, vel_xy, yaw, dt):
        self.cmd = np.zeros(3)
        self.active = False
        hold_yaw = float(np.clip(K_YAW * wrap(0.0 - yaw), -W_HOLD, W_HOLD))
        step = next(((axis, v) for t0, t1, axis, v in self.schedule if t0 <= self.t < t1), None)
        if step is not None:
            axis, value = step
            self.active = True
            self.cmd[AXES[axis]["index"]] = value
            if axis != "yaw":
                self.cmd[2] = hold_yaw
        elif not self._quiet():
            to_home = K_HOME * (HOME - np.asarray(pos_xy))
            speed = np.linalg.norm(to_home)
            self.cmd[:2] = to_home * min(1.0, V_HOME / speed) if speed > 1e-9 else 0.0
            self.cmd[2] = hold_yaw
        self.t += dt
        return np.array([self.cmd[0], self.cmd[1], 0.0]), np.array([0.0, 0.0, self.cmd[2]]), self.t >= self.t_end


def run_tests(out_dir: Path, mus, timeout_margin=2.0):
    from trackA.sim.run_waypoints import run
    schedule, t_end = build_schedule()
    results = {}
    for mu in mus:
        tag = f"step_mu{int(round(mu * 100)):02d}"
        s = run("trackC/courses/flat_v1.yaml", timeout=t_end + timeout_margin, render=False, log_dir=str(out_dir),
                mpc_kind="perfoot", mu_feet=[mu] * 4, tag=tag,
                follower_factory=lambda xy, yaw: StepCommand(xy, schedule, t_end))
        results[tag] = dict(outcome=s["outcome"], wall_contact_steps=s["wall_contact_steps"])
    (out_dir / "schedule.json").write_text(json.dumps(dict(schedule=schedule, t_end=t_end, mus=list(mus))))
    return results


# ---------------------------------------------------------------- fitting
def moving_average(y, n):
    kernel = np.ones(n) / n
    pad = (np.full(n, y[0]), y, np.full(n, y[-1]))
    return np.convolve(np.concatenate(pad), kernel, mode="same")[n:-n]


def lag_response(u, y0, tau, dt):
    a = np.exp(-dt / tau)
    y = np.empty_like(u)
    y[0] = y0
    for k in range(len(u) - 1):
        y[k + 1] = a * y[k] + (1 - a) * u[k]
    return y


def fit_step(t, cmd, meas, t0, t1, dt):
    """Fit tau and K on one step, from one gait cycle before it until the command ends."""
    decim = max(1, int(round(0.01 / dt)))
    n_avg = max(1, int(round(GAIT_PERIOD / (dt * decim))))
    start, stop = int(np.searchsorted(t, t0 - GAIT_PERIOD)), int(np.searchsorted(t, t1))
    u, y = cmd[start:stop:decim], meas[start:stop:decim]
    h = dt * decim
    y_avg = moving_average(y, n_avg)
    best = None
    for tau in np.arange(0.02, 1.2001, 0.01):
        base = moving_average(lag_response(u, 0.0, tau, h), n_avg)
        if base @ base < 1e-12:
            continue
        gain = float(base @ y_avg / (base @ base))
        err = float(np.mean((gain * base - y_avg) ** 2))
        if best is None or err < best[0]:
            best = (err, float(tau), gain)
    err, tau, gain = best
    total = float(np.mean((y_avg - y_avg.mean()) ** 2))
    tail = (t >= t1 - 2 * GAIT_PERIOD) & (t < t1)          # last two whole gait cycles of the hold
    return dict(tau=tau, gain=gain, r2=1.0 - err / total if total > 0 else 0.0, steady=float(meas[tail].mean()))


def analyze(log_dir: Path, out_dir: Path) -> dict:
    meta = json.loads((log_dir / "schedule.json").read_text())
    rows, report = [], {}
    for mu in meta["mus"]:
        d = np.load(log_dir / f"step_mu{int(round(mu * 100)):02d}.npz", allow_pickle=False)
        dt = float(d["sim_dt"])
        t = d["t"] - d["t"][0] + dt
        yaw, v = d["yaw"], d["base_lin_vel"]
        c, s = np.cos(yaw), np.sin(yaw)
        forward, lateral = c * v[:, 0] + s * v[:, 1], -s * v[:, 0] + c * v[:, 1]
        meas = np.column_stack((forward, lateral, d["base_ang_vel"][:, 2]))
        cmd = d["step_cmd"]
        cmd_b = np.column_stack((c * cmd[:, 0] + s * cmd[:, 1], -s * cmd[:, 0] + c * cmd[:, 1], cmd[:, 2]))
        for t0, t1, axis, value in meta["schedule"]:
            i = AXES[axis]["index"]
            rows.append(dict(mu=mu, axis=axis, cmd=value, **fit_step(t, cmd_b[:, i], meas[:, i], t0, t1, dt)))
        hits = d["wall_hit_barrier"]
        key = f"mu_{mu:g}"
        report[key] = dict(outcome=str(d["summary_outcome"]), wall_contact_steps=int(len(hits)),
                           max_yaw_during_linear=float(np.abs(yaw[(cmd[:, 2] != 0) | (cmd[:, 0] != 0)]).max()))
        for axis, spec in AXES.items():
            sel = [r for r in rows if r["mu"] == mu and r["axis"] == axis]
            good = [r for r in sel if r["r2"] >= 0.8]
            pairs = {}
            for size in spec["sizes"]:
                plus = next(r for r in sel if r["cmd"] == size)["steady"]
                minus = next(r for r in sel if r["cmd"] == -size)["steady"]
                pairs[f"{size:g}"] = dict(gain=(plus - minus) / (2 * size), bias=(plus + minus) / 2)
            gains = [p["gain"] for p in pairs.values()]
            report[key][axis] = dict(
                tau_median=float(np.median([r["tau"] for r in good])) if good else None,
                tau_range=[float(min(r["tau"] for r in good)), float(max(r["tau"] for r in good))] if good else None,
                n_fits=len(good), gain_median=float(np.median(gains)), gain_range=[min(gains), max(gains)],
                by_size=pairs,
            )
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "fits.json").write_text(json.dumps(dict(summary=report, steps=rows), indent=1))
    lines = ["# Step test: Quadruped-PyMPC velocity response", "",
             f"Logs: `{log_dir}`. `v_dot = (K v_cmd - v) / tau`, body frame, gait-cycle-averaged. `tau` uses fits with "
             "R^2 >= 0.8. `K` is half the difference of the steady speeds for + and - commands of each size.", "",
             "| cone mu | axis | tau median [s] | tau range [s] | K median | K range | fits | wall contacts |",
             "|---|---|---|---|---|---|---|---|"]
    for key, rep in report.items():
        for axis in AXES:
            r = rep[axis]
            tau = f"{r['tau_median']:.3f}" if r["tau_median"] is not None else "n/a"
            span = f"{r['tau_range'][0]:.2f} to {r['tau_range'][1]:.2f}" if r["tau_range"] else "n/a"
            lines.append(f"| {key[3:]} | {axis} | {tau} | {span} | {r['gain_median']:.3f} | {r['gain_range'][0]:.3f} to "
                         f"{r['gain_range'][1]:.3f} | {r['n_fits']} | {rep['wall_contact_steps']} |")
    lines += ["", "## Gain by command size", "", "| cone mu | axis | size | K | constant drift |", "|---|---|---|---|---|"]
    for key, rep in report.items():
        for axis in AXES:
            for size, p in rep[axis]["by_size"].items():
                lines.append(f"| {key[3:]} | {axis} | {size} | {p['gain']:.3f} | {p['bias']:+.4f} |")
    lines += ["", "## Every step", "", "| cone mu | axis | command | tau [s] | K fit | steady | R^2 |",
              "|---|---|---|---|---|---|---|"]
    lines += [f"| {r['mu']:g} | {r['axis']} | {r['cmd']:+.2f} | {r['tau']:.2f} | {r['gain']:.3f} | {r['steady']:+.3f} | "
              f"{r['r2']:.3f} |" for r in rows]
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n")
    return report


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mu", nargs="+", type=float, default=[0.5, 0.3], help="friction used by the MPC's cones")
    ap.add_argument("--analyze", default=None, help="existing log folder: only redo the fits")
    ap.add_argument("--out", default="trackA/results/step_test")
    a = ap.parse_args()
    if a.analyze:
        folder = Path(a.analyze)
    else:
        folder = Path("data/step_test") / f"{datetime.now():%Y%m%d_%H%M%S}"
        folder.mkdir(parents=True, exist_ok=True)
        print(run_tests(folder, a.mu))
    print(json.dumps(analyze(folder, Path(a.out)), indent=1))
