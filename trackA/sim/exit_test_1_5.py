"""Step 1.5: stage 1 exit test. 20 runs on flat ground (10 straight, 10 zigzag), logs saved, then analyzed.

Pass criterion (project plan): at least 19 of 20 runs reach the goal.

Each run is a separate `python -m trackA.sim.run_waypoints` process (headless), so a crash in one run cannot take the
others down. Runs differ only by their seed, which jitters the spawn by +-10 cm in x, y and +-15 deg in yaw (without
it every run of a path would be identical: MuJoCo and acados are deterministic). Runs are sequential: every process
regenerates and compiles the acados solver into the same folder, so parallel runs would race on it.

Run from the repo root, inside the pixi env:
    python -m trackA.sim.exit_test_1_5                       # 20 runs on flat_v1, then the analysis
    python -m trackA.sim.exit_test_1_5 --n 2 --timeout 20    # quick dry run of the pipeline
    python -m trackA.sim.analyze_1_5 data/exit_test_1_5/<timestamp>   # re-run only the analysis

Output folder data/exit_test_1_5/<timestamp>/ (ignored by git):
    manifest.json                    settings, seeds, commits, course hash, per-run return codes
    run_XX_<path>_sYY.npz / .png     per-step log and plan-view plot of each run (see run_waypoints.py)
    run_XX_<path>_sYY_contacts.npz   stance events (holds / slips) from the Track C contact logger
    run_XX_<path>_sYY.log            the run's console output
    summary.md, summary.csv, *.png   written by analyze_1_5
"""
import argparse
import hashlib
import json
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from quadruped_pympc import config as cfg

from trackA.sim import analyze_1_5
from trackA.sim.run_waypoints import JITTER_XY, JITTER_YAW

PATHS = ("straight", "zigzag")


def git_rev(path: str = ".") -> str:
    try:
        rev = subprocess.run(["git", "-C", path, "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout
        dirty = subprocess.run(["git", "-C", path, "status", "--porcelain"], capture_output=True, text=True).stdout
        return rev.strip() + ("-dirty" if dirty.strip() else "")
    except OSError:
        return "unknown"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", default="trackC/courses/flat_v1.yaml")
    ap.add_argument("--n", type=int, default=10, help="runs per path (seeds 0..n-1)")
    ap.add_argument("--v-ref", type=float, default=0.4)
    ap.add_argument("--timeout", type=float, default=90.0, help="seconds of simulated time per run")
    ap.add_argument("--out", default=None, help="output folder (default data/exit_test_1_5/<timestamp>)")
    a = ap.parse_args()

    out = Path(a.out or f"data/exit_test_1_5/{datetime.now():%Y%m%d_%H%M%S}")
    out.mkdir(parents=True, exist_ok=True)
    runs = [dict(tag=f"run_{k:02d}_{p}_s{s:02d}", path=p, seed=s)
            for k, (p, s) in enumerate((p, s) for p in PATHS for s in range(a.n))]

    manifest = dict(
        created=datetime.now().isoformat(timespec="seconds"),
        step="1.5 stage 1 exit test",
        pass_criterion="at least 19 of 20 runs reach the goal (scaled: >= 95% of runs)",
        course_file=a.course,
        course_sha256=hashlib.sha256(Path(a.course).read_bytes()).hexdigest(),
        git_commit=git_rev("."), quadruped_pympc_commit=git_rev("Quadruped-PyMPC"),
        v_ref=a.v_ref, timeout=a.timeout, mpc="upstream nominal (no MPC walls)",
        spawn_jitter=dict(xy_m=JITTER_XY, yaw_deg=float(np.rad2deg(JITTER_YAW))),
        mpc_params={k: cfg.mpc_params[k] for k in ("type", "horizon", "dt", "frequency", "mu", "use_RTI",
                                                     "num_qp_iterations", "use_foothold_constraints")},
        sim_dt=cfg.simulation_params["dt"],
        runs=runs,
    )
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str))

    print(f"step 1.5: {len(runs)} runs -> {out}")
    t_all = time.time()
    for k, r in enumerate(runs):
        cmd = [sys.executable, "-m", "trackA.sim.run_waypoints", "--course", a.course, "--path", r["path"],
               "--seed", str(r["seed"]), "--v-ref", str(a.v_ref), "--timeout", str(a.timeout), "--no-render",
               "--plot", "--log-dir", str(out), "--tag", r["tag"]]
        t0 = time.time()
        with open(out / f"{r['tag']}.log", "w") as f:
            rc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT).returncode
        r.update(returncode=rc, wall_time_s=round(time.time() - t0, 1))
        result = "crashed" if rc else _outcome(out / f"{r['tag']}.npz")
        print(f"[{k + 1:2d}/{len(runs)}] {r['tag']}: {result} ({r['wall_time_s']:.0f} s)", flush=True)
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str))

    print(f"all runs done in {(time.time() - t_all) / 60:.1f} min\n")
    analyze_1_5.analyze(out)


def _outcome(npz_path: Path) -> str:
    try:
        d = np.load(npz_path, allow_pickle=True)
        return f"{d['summary_outcome']} after {float(d['summary_time']):.1f} s"
    except (OSError, KeyError):
        return "no log"


if __name__ == "__main__":
    main()
