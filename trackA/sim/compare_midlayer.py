"""Steps J7 to J9 of the mid-layer plan: full-course batch comparing the controllers on the same seeds.

Each run is its own `python -m trackA.sim.run_midlayer` process, so a crash cannot take the others down.

Run from the repo root, inside the pixi env:
    python -m trackA.sim.compare_midlayer                       # the full matrix below
    python -m trackA.sim.compare_midlayer --seeds 2 --pushed-seeds 1 --narrow-seeds 1 --workers 3    # quick dry run
    python -m trackA.sim.compare_midlayer --resume data/compare_midlayer/<timestamp>   # run only what is missing
    python -m trackA.sim.compare_midlayer --analyze data/compare_midlayer/<timestamp>

Logs go to data/compare_midlayer/<timestamp>/ (ignored by git). The summary goes to trackA/results/mid_layer_j9/.
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

COURSES = {"wide": "trackA/courses/midlayer_test_v1.yaml",      # 1.7 m openings
           "narrow": "trackA/courses/midlayer_gap125_v1.yaml"}  # 1.25 m openings
PUSHES = (1.0, 1.25)    # [m/s^2] sideways push sizes
FOOTPRINT = "three_circle"
VERSIONS = (("lqr", None), ("tube", "carried"), ("tube", "measured"))


def build_matrix(seeds, pushed_seeds, narrow_seeds, first_seed=0):
    """Every version on the same seeds. The follower runs ahead of the plan's clock, so it gets no plan-timed pushes."""
    def seed_range(n):
        return range(first_seed, first_seed + n)

    jobs = [dict(course="wide", controller="follower", qp_start=None, seed=s, push=0.0) for s in seed_range(seeds)]
    for controller, qp_start in VERSIONS:
        jobs += [dict(course="wide", controller=controller, qp_start=qp_start, seed=s, push=0.0) for s in seed_range(seeds)]
        for push in PUSHES if controller == "tube" else ():     # LQR alone matched the carried QP in the last batch
            jobs += [dict(course="wide", controller=controller, qp_start=qp_start, seed=s, push=push)
                     for s in seed_range(pushed_seeds)]
    for controller, qp_start in VERSIONS[1:]:
        for push in (0.0, 1.0):
            jobs += [dict(course="narrow", controller=controller, qp_start=qp_start, seed=s, push=push)
                     for s in seed_range(narrow_seeds)]
    return jobs


def job_name(job):
    version = job["controller"] if job["qp_start"] is None else f"{job['controller']}-{job['qp_start']}"
    return f"{job['course']}_{version}_p{int(round(job['push'] * 100)):03d}_s{job['seed']:02d}"


def run_job(job, folder, workers):
    worker = workers.get()
    name = job_name(job)
    try:
        cmd = [sys.executable, "-m", "trackA.sim.run_midlayer", "--course", COURSES[job["course"]],
               "--controller", job["controller"], "--footprint", FOOTPRINT, "--seed", str(job["seed"]), "--no-render",
               "--log-dir", str(folder), "--tag", name, "--json", str(folder / f"{name}.json"),
               "--solver-dir", str(folder / "solvers" / f"worker_{worker}"), "--plan-cache", str(folder / "plans")]
        if job["qp_start"]:
            cmd += ["--qp-start", job["qp_start"]]
        if job["push"] > 0.0:
            cmd += ["--push", "worst", "--push-accel", str(job["push"])]
        with open(folder / f"{name}.log", "w") as f:
            try:
                code = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=1800).returncode
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


def _version(r):
    if r["controller"] != "tube":
        return r["controller"]
    return f"QP {r.get('qp_start') or 'carried'}"


def analyze(folder: Path, out_dir: Path) -> list:
    rows = []
    for path in sorted(folder.glob("*.json")):
        if path.name == "manifest.json":
            continue
        r = json.loads(path.read_text())
        r["name"] = path.stem
        r["course_name"] = Path(r["course"]).stem
        r["version"] = _version(r)
        rows.append(r)
    groups = {}
    for r in rows:
        groups.setdefault((r["course_name"], r["version"], r.get("push_accel", 0.0)), []).append(r)
    order = {"follower": 0, "lqr": 1, "QP carried": 2, "QP measured": 3}
    table = []
    for key in sorted(groups, key=lambda k: (k[0] != "midlayer_test_v1", k[2], order[k[1]])):
        g = groups[key]
        reached = [r for r in g if r["outcome"] == "goal"]
        err = _values(g, "max_ref_error")
        clear = _values(g, "min_bare_clearance")
        table.append(dict(
            course=key[0], version=key[1], push=key[2], runs=len(g), goal=len(reached),
            falls=sum(r["outcome"] == "terminated" for r in g), no_plan=sum(r["outcome"] == "no feasible plan" for r in g),
            contact_runs=sum(r.get("wall_contact_steps", 0) > 0 for r in g),
            mean_time=float(np.mean([r["time"] for r in reached])) if reached else None,
            worst_error=max(err) if err else None, mean_max_error=float(np.mean(err)) if err else None,
            worst_over_tube=max(_values(g, "max_ref_over_tube"), default=None),
            outside_tube_runs=sum(r.get("max_ref_over_tube", 0.0) > 1.0 for r in g),
            worst_yaw_error=max(_values(g, "max_yaw_error"), default=None),
            saturated_updates=sum(r.get("saturated", 0) for r in g),
            min_clearance=min(clear) if clear else None,
            qp_solves=sum(r.get("qp_solves", 0) for r in g), qp_fallbacks=sum(r.get("qp_fallbacks", 0) for r in g),
            tube_slack_solves=sum(r.get("tube_slack_solves", 0) for r in g),
            body_slack_solves=sum(r.get("body_slack_solves", 0) for r in g),
            speed_slack_solves=sum(r.get("speed_slack_solves", 0) for r in g),
            max_body_slack=max(_values(g, "max_body_slack_m"), default=None),
            qp_overruns=sum(r.get("qp_overruns", 0) for r in g),
            qp_p99_ms=max(_values(g, "qp_ms_p99"), default=None), qp_max_ms=max(_values(g, "qp_ms_max"), default=None),
            low_level_failed=sum(r.get("low_level_failed_solves", 0) for r in g),
        ))
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / "runs.csv", "w", newline="") as f:
        keys = ["name", "course_name", "version", "footprint", "push_accel", "seed", "outcome", "time", "final_dist",
                "wall_contact_steps", "min_bare_clearance", "max_ref_error", "rms_ref_error", "max_ref_over_tube",
                "time_outside_plan_tube", "max_yaw_error", "time_over_eps_yaw", "saturated", "qp_solves", "qp_fallbacks",
                "tube_slack_solves", "body_slack_solves", "speed_slack_solves", "max_tube_slack_m", "max_body_slack_m",
                "max_speed_slack", "qp_ms_median", "qp_ms_p99", "qp_ms_max", "low_level_failed_solves", "plan_T",
                "plan_clearance"]
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    lines = ["# Three-circle and measured-state MPC comparison (J8, J9)", "",
             f"Logs: `{folder}`. Three-circle footprint, friction cones at 0.3. `midlayer_test_v1` has 1.7 m openings and "
             "`midlayer_gap125_v1` has 1.25 m openings. Push = 1.5 s sideways force toward the nearest obstacle at each "
             "of the three obstacle groups. **Error is the distance from the planner's reference at the same time**, for "
             "every version. \"Outside tube\" means that error exceeded the planner's full tube (0.263 m). "
             "QP times were measured with several runs in parallel.", "",
             "| course | version | push [m/s²] | goal | falls | runs with contact | mean time [s] | worst error [cm] | "
             "worst error / tube | runs outside tube | worst yaw error [rad] | saturated updates | min body clearance [m] |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for t in table:
        lines.append(f"| {t['course']} | {t['version']} | {t['push']:g} | {t['goal']}/{t['runs']} | {t['falls']} | "
                     f"{t['contact_runs']} | {_fmt(t['mean_time'])} | {_fmt(t['worst_error'], 1, 100)} | "
                     f"{_fmt(t['worst_over_tube'], 2)} | {t['outside_tube_runs'] if t['version'] != 'follower' else 'n/a'} | "
                     f"{_fmt(t['worst_yaw_error'], 3)} | "
                     f"{t['saturated_updates'] if t['version'] != 'follower' else 'n/a'} | {_fmt(t['min_clearance'], 3)} |")
    lines += ["", "## QP", "", "| course | version | push [m/s²] | solves | fallbacks | tube slack solves | body slack solves | "
              "speed slack solves | max body slack [cm] | over 100 ms | worst p99 [ms] | worst [ms] |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for t in table:
        if t["version"].startswith("QP"):
            lines.append(f"| {t['course']} | {t['version']} | {t['push']:g} | {t['qp_solves']} | {t['qp_fallbacks']} | "
                         f"{t['tube_slack_solves']} | {t['body_slack_solves']} | {t['speed_slack_solves']} | "
                         f"{_fmt(t['max_body_slack'], 2, 100)} | {t['qp_overruns']} | {_fmt(t['qp_p99_ms'])} | "
                         f"{_fmt(t['qp_max_ms'])} |")
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n")
    return table


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--pushed-seeds", type=int, default=5)
    ap.add_argument("--narrow-seeds", type=int, default=5)
    ap.add_argument("--first-seed", type=int, default=0, help="run seeds first_seed, first_seed + 1, ...")
    ap.add_argument("--pushed-only", action="store_true", help="skip the runs without a push")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--analyze", default=None)
    ap.add_argument("--resume", default=None, help="existing run folder: run only the jobs without a result")
    ap.add_argument("--out", default="trackA/results/mid_layer_j9")
    a = ap.parse_args()
    if a.analyze:
        run_dir = Path(a.analyze)
    else:
        if a.resume:
            run_dir = Path(a.resume)
            full = build_matrix(a.seeds, a.pushed_seeds, a.narrow_seeds, a.first_seed)
            full = [j for j in full if j["push"] > 0.0] if a.pushed_only else full
            (run_dir / "manifest.json").write_text(json.dumps(dict(courses=COURSES, jobs=full), indent=1))
            matrix = [j for j in full if not (run_dir / f"{job_name(j)}.json").exists()]
            print(f"resuming: {len(matrix)} runs left", flush=True)
        else:
            run_dir = Path("data/compare_midlayer") / f"{datetime.now():%Y%m%d_%H%M%S}"
            run_dir.mkdir(parents=True, exist_ok=True)
            matrix = build_matrix(a.seeds, a.pushed_seeds, a.narrow_seeds, a.first_seed)
            matrix = [j for j in matrix if j["push"] > 0.0] if a.pushed_only else matrix
            (run_dir / "manifest.json").write_text(json.dumps(dict(courses=COURSES, jobs=matrix), indent=1))
        pool = queue.Queue()
        for i in range(a.workers):
            pool.put(i)
        with ThreadPoolExecutor(a.workers) as ex:
            results = list(ex.map(lambda j: run_job(j, run_dir, pool), matrix))
        failed = [n for n, c in results if c != 0]
        print(f"{len(results)} runs, {len(failed)} process failures: {failed}", flush=True)
    for row in analyze(run_dir, Path(a.out)):
        print(json.dumps(row))
