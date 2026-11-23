"""Step J7 of the mid-layer plan: full-course batch comparing the controllers on the same seeds.

Each run is its own `python -m trackA.sim.run_midlayer` process, so a crash cannot take the others down.

Run from the repo root, inside the pixi env:
    python -m trackA.sim.compare_midlayer                       # the full matrix below
    python -m trackA.sim.compare_midlayer --seeds 2 --pushed-seeds 1 --workers 3    # quick dry run
    python -m trackA.sim.compare_midlayer --resume data/compare_midlayer/<timestamp>   # run only what is missing
    python -m trackA.sim.compare_midlayer --analyze data/compare_midlayer/<timestamp>

Logs go to data/compare_midlayer/<timestamp>/ (ignored by git). The summary goes to trackA/results/mid_layer/.
"""
import argparse
import csv
import json
import queue
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

import numpy as np

COURSE = "trackA/courses/midlayer_test_v1.yaml"
PUSHES = (1.0, 1.25)    # [m/s^2] sideways push sizes


def build_matrix(seeds, pushed_seeds):
    """The follower runs ahead of the plan's clock, so plan-timed pushes would miss the obstacles: no pushed follower."""
    jobs = []
    for controller in ("follower", "lqr", "tube"):
        jobs += [dict(controller=controller, seed=s, push=0.0, conversion="lag_inverse") for s in range(seeds)]
    for push in PUSHES:
        for controller in ("lqr", "tube"):
            jobs += [dict(controller=controller, seed=s, push=push, conversion="lag_inverse")
                     for s in range(pushed_seeds)]
    for push in (0.0, *PUSHES):                 # decision B3's conversion, for the team's comparison
        jobs += [dict(controller="lqr", seed=s, push=push, conversion="b3") for s in range(pushed_seeds)]
    return jobs


def job_name(job):
    return f"{job['controller']}_{job['conversion']}_p{int(round(job['push'] * 100)):03d}_s{job['seed']:02d}"


def run_job(job, folder, workers):
    worker = workers.get()
    name = job_name(job)
    try:
        cmd = [sys.executable, "-m", "trackA.sim.run_midlayer", "--course", COURSE, "--controller", job["controller"],
               "--seed", str(job["seed"]), "--conversion", job["conversion"], "--no-render", "--log-dir", str(folder),
               "--tag", name, "--json", str(folder / f"{name}.json"),
               "--solver-dir", str(folder / "solvers" / f"worker_{worker}")]
        if job["push"] > 0.0:
            cmd += ["--push", "worst", "--push-accel", str(job["push"])]
        with open(folder / f"{name}.log", "w") as f:
            try:
                code = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=1200).returncode
            except subprocess.TimeoutExpired:
                code = -1
        print(f"{name}: exit {code}", flush=True)
        return name, code
    finally:
        workers.put(worker)


def _values(rows, key):
    return [r[key] for r in rows if r.get(key) is not None]


def _fmt(x, digits=1, scale=1.0):
    return "n/a" if x is None else f"{x * scale:.{digits}f}"


def analyze(folder: Path, out_dir: Path) -> list:
    rows = []
    for path in sorted(folder.glob("*.json")):
        if path.name == "manifest.json":
            continue
        r = json.loads(path.read_text())
        r["name"] = path.stem
        if r["controller"] == "follower" and r.get("push_accel", 0.0) > 0.0:
            continue                            # its pushes were timed by the plan and missed the obstacles
        rows.append(r)
    groups = {}
    for r in rows:
        groups.setdefault((r["controller"], r["conversion"], r.get("push_accel", 0.0)), []).append(r)
    order = {"follower": 0, "lqr": 1, "tube": 2}
    table = []
    for key in sorted(groups, key=lambda k: (k[2], k[1] != "lag_inverse", order[k[0]])):
        g = groups[key]
        reached = [r for r in g if r["outcome"] == "goal"]
        err = _values(g, "max_pos_error")
        clear = _values(g, "min_bare_clearance")
        table.append(dict(
            controller=key[0], conversion=key[1], push=key[2], runs=len(g), goal=len(reached),
            falls=sum(r["outcome"] == "terminated" for r in g),
            contact_runs=sum(r.get("wall_contact_steps", 0) > 0 for r in g),
            mean_time=float(np.mean([r["time"] for r in reached])) if reached else None,
            worst_error=max(err) if err else None, mean_max_error=float(np.mean(err)) if err else None,
            worst_over_radius=max(_values(g, "max_pos_over_radius"), default=None),
            tube_violation_runs=sum(r.get("tube_violations", 0) > 0 for r in g),
            saturated_updates=sum(r.get("saturated", 0) for r in g),
            min_clearance=min(clear) if clear else None,
            qp_solves=sum(r.get("qp_solves", 0) for r in g), qp_fallbacks=sum(r.get("qp_fallbacks", 0) for r in g),
            qp_slack_solves=sum(r.get("slack_solves", 0) for r in g),
            qp_overruns=sum(r.get("qp_overruns", 0) for r in g),
            qp_p99_ms=max(_values(g, "qp_ms_p99"), default=None), qp_max_ms=max(_values(g, "qp_ms_max"), default=None),
            low_level_failed=sum(r.get("low_level_failed_solves", 0) for r in g),
        ))
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / "runs.csv", "w", newline="") as f:
        keys = ["name", "controller", "conversion", "push_accel", "seed", "outcome", "time", "final_dist",
                "wall_contact_steps", "min_bare_clearance", "max_cross_track", "max_pos_error", "rms_pos_error",
                "max_pos_over_radius", "max_e_over_alpha", "tube_violations", "saturated", "qp_solves", "qp_fallbacks",
                "slack_solves", "max_slack_m", "qp_ms_median", "qp_ms_p99", "qp_ms_max", "low_level_failed_solves"]
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    lines = ["# Mid-layer comparison on `midlayer_test_v1`", "",
             f"Logs: `{folder}`. One 0.483 m circle plus the 0.263 m tube, friction cones at 0.3, 1.7 m openings. "
             "Push = 1.5 s sideways force toward the nearest obstacle at each of the three obstacle groups. "
             "Tracking error is measured from the nominal, so it is not available for `follower`. QP times were "
             "measured with several runs in parallel.", "",
             "| controller | conversion | push [m/s²] | goal | falls | runs with contact | mean time [s] | "
             "worst error [cm] | worst error / tube | runs outside tube | saturated updates | min body clearance [m] |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for t in table:
        lines.append(f"| {t['controller']} | {t['conversion']} | {t['push']:g} | {t['goal']}/{t['runs']} | {t['falls']} | "
                     f"{t['contact_runs']} | {_fmt(t['mean_time'])} | {_fmt(t['worst_error'], 1, 100)} | "
                     f"{_fmt(t['worst_over_radius'], 2)} | "
                     f"{t['tube_violation_runs'] if t['controller'] != 'follower' else 'n/a'} | "
                     f"{t['saturated_updates'] if t['controller'] != 'follower' else 'n/a'} | "
                     f"{_fmt(t['min_clearance'], 3)} |")
    lines += ["", "## Nominal QP", "", "| push [m/s²] | solves | fallbacks | solves with slack > 1 mm | over 100 ms | "
              "worst p99 [ms] | worst [ms] |", "|---|---|---|---|---|---|---|"]
    for t in table:
        if t["controller"] == "tube":
            lines.append(f"| {t['push']:g} | {t['qp_solves']} | {t['qp_fallbacks']} | {t['qp_slack_solves']} | "
                         f"{t['qp_overruns']} | {_fmt(t['qp_p99_ms'])} | {_fmt(t['qp_max_ms'])} |")
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n")
    return table


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--pushed-seeds", type=int, default=5)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--analyze", default=None)
    ap.add_argument("--resume", default=None, help="existing run folder: run only the jobs without a result")
    ap.add_argument("--out", default="trackA/results/mid_layer")
    a = ap.parse_args()
    if a.analyze:
        run_dir = Path(a.analyze)
    else:
        if a.resume:
            run_dir = Path(a.resume)
            full = build_matrix(a.seeds, a.pushed_seeds)
            (run_dir / "manifest.json").write_text(json.dumps(dict(course=COURSE, jobs=full), indent=1))
            matrix = [j for j in full if not (run_dir / f"{job_name(j)}.json").exists()]
            print(f"resuming: {len(matrix)} runs left", flush=True)
        else:
            run_dir = Path("data/compare_midlayer") / f"{datetime.now():%Y%m%d_%H%M%S}"
            run_dir.mkdir(parents=True, exist_ok=True)
            matrix = build_matrix(a.seeds, a.pushed_seeds)
            (run_dir / "manifest.json").write_text(json.dumps(dict(course=COURSE, jobs=matrix), indent=1))
        pool = queue.Queue()
        for i in range(a.workers):
            pool.put(i)
        with ThreadPoolExecutor(a.workers) as ex:
            results = list(ex.map(lambda j: run_job(j, run_dir, pool), matrix))
        failed = [n for n, c in results if c != 0]
        print(f"{len(results)} runs, {len(failed)} process failures: {failed}", flush=True)
    for row in analyze(run_dir, Path(a.out)):
        print(json.dumps(row))
