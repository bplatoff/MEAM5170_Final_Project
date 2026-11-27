"""Batch runner (task 2.3g, decision C2): seeded trials over courses, gap widths, variants and controllers.

    python -m trackC.batch --course trackC/courses/mixed_v1.yaml --gaps 0.6:1.1:0.1 --variants low high \\
        --seeds 20 --controllers nominal robust_follower --workers 3
    python -m trackC.batch --resume data/batch/<timestamp>          # run only the jobs without a result
    python -m trackC.batch --summarize data/batch/<timestamp> --publish results/batch/<name>

One job = one controller on one course variant (gap, patch class) with one seed. Seed i draws both the friction
values and the start pose, from separate generators (A1.1); every controller gets the same seeds, so they face
identical terrain and starts. Steps:

  1. scenes   build each (gap, variant, seed) scene once, in this process, and install it only if it changed
  2. plans    plan each (planning profile, gap, seed) once, in parallel: footprint and start yaw per A2.3 and
              Austin's config_for_initial_yaw, inflated by the controller's tube. No route -> "no feasible plan"
  3. runs     one process per job (trackC.batch_sim), a few at a time; each writes results/<job>.json
  4. summary  runs.csv, summary.csv / summary.md by controller x variant x gap, two plots (A3.5)

Run folders hold logs and are ignored by git (data/). Small summaries can be copied into the repo with --publish (C4).
"""
import argparse
import csv
import hashlib
import json
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, as_completed
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

from trackC.course_spec import MU_NOMINAL, load_course_cfg
from trackC.friction_map import PriorFrictionMap
from trackC.metrics import no_plan_metrics
from trackC.start_pose import (HEADING_ALLOWANCE, PLAN_SAFETY_MARGIN, ROBOT_LENGTH, ROBOT_WIDTH, ROBUST_TUBE_RADIUS,
                               sample_start)

REPO = Path(__file__).resolve().parents[1]

# Controllers. `available: False` ones are listed so the matrix and summaries already know them; they raise a clear
# error until their pieces land. cone_mu is the friction every foot's MPC cone uses; tube_margin inflates the plan.
CONTROLLERS = {
    "nominal": dict(
        available=True, cone_mu=MU_NOMINAL, tube_margin=0.0, v_ref=0.4,
        note="stage 1: per-foot MPC with cones at 0.5, path follower on a plan with no tube"),
    "robust_follower": dict(
        available=True, cone_mu=PriorFrictionMap().mu_lo, tube_margin=ROBUST_TUBE_RADIUS + HEADING_ALLOWANCE,
        v_ref=0.2,
        note="interim robust: plan inflated by the full tube, cones at mu_lo = 0.3, path follower. Stands in for "
             "'robust' until the mid layer is merged; it has no tube feedback"),
    "robust": dict(available=False, needs="the mid-layer tube MPC (task 2.4, branch trackA-2-4) in run()"),
    "learned": dict(available=False, needs="the certificate layer's friction map (task 3.3)"),
    "oracle": dict(available=False, needs="per-foot true friction in run() (Jacob's run() interface)"),
}

PLANNER_BASE = dict(
    N=75, horizon_mode="astar_length", cruise_speed_mps=0.3, time_buffer_s=2.0, max_speed=0.5,
    friction_mu=MU_NOMINAL,                       # A5: the robust baseline's acceleration limit
    smoothness_weight=1.0, robot_length=ROBOT_LENGTH, robot_width=ROBOT_WIDTH, r_safety=PLAN_SAFETY_MARGIN,
    astar_resolution=0.05, astar_clearance=0.01, footprint_model="three_circle", footprint_yaw_mode="path_tangent")


# ---------------------------------------------------------------- job matrix
def parse_gaps(text):
    """'0.6:1.1:0.1' -> [0.6, 0.7, ..., 1.1]; '0.8,1.0' -> [0.8, 1.0]; None -> [None] (course without a gap)."""
    if text is None:
        return [None]
    if ":" in text:
        a, b, s = (float(v) for v in text.split(":"))
        return [round(float(g), 3) for g in np.arange(a, b + s / 2, s)]
    return [round(float(g), 3) for g in text.split(",")]


def build_jobs(controllers, variants, gaps, seeds) -> list:
    """Every combination, in a stable order. Job ids are readable and unique."""
    jobs = []
    for ctrl in controllers:
        spec = CONTROLLERS[ctrl]
        if not spec["available"]:
            raise SystemExit(f"controller '{ctrl}' is not available yet: needs {spec['needs']}")
        for variant in variants:
            for gap in gaps:
                for seed in seeds:
                    gid = "nogap" if gap is None else f"g{gap:.2f}"
                    vid = variant or "file"
                    jobs.append(dict(id=f"{ctrl}_{vid}_{gid}_s{seed:03d}", controller=ctrl, variant=variant, gap=gap,
                                     seed=seed, cone_mu=spec["cone_mu"], tube_margin=spec["tube_margin"],
                                     v_ref=spec["v_ref"]))
    return jobs


# ---------------------------------------------------------------- scenes and plans
def ensure_scene(course_path, variant, gap, seed) -> dict:
    """Build one scene and install it only if it is missing or changed. Returns terrain_gen's info (with the
    expanded course dict under 'cfg' and the scene name under 'name')."""
    import gym_quadruped
    from trackC.terrain_gen import build, install
    xml, info = build(course_path, seed, gap=gap, variant=variant)
    target = Path(gym_quadruped.__file__).parent / "robot_model" / f"scene_{info['name']}.xml"
    if not target.exists() or target.read_text() != xml:
        install(info["name"], xml)
    return info


def plan_job(args):
    """Plan one (profile, gap, seed). Returns (key, path xy or None, info)."""
    key, course_path, variant, gap, seed, tube_margin, plan_file = args
    from planning.direct_collocation import plan_trajectory
    from planning.start_yaw_config import config_for_initial_yaw
    from trackC.planner_env import planner_problem
    cfg = load_course_cfg(course_path, gap=gap, variant=variant)
    pose = sample_start(cfg, seed)
    problem = planner_problem(cfg, start=(pose.x, pose.y))
    config = dict(PLANNER_BASE)
    if tube_margin > 0:
        config["tube_position_margin_m"] = tube_margin
    config = config_for_initial_yaw(config, initial_yaw_rad=pose.yaw)
    t0 = time.time()
    try:
        result = plan_trajectory(problem.start_state, problem.goal_state, problem.environment, config)
        if not result["success"]:
            return key, None, dict(reason=f"NLP did not converge: {result.get('return_status')}",
                                   seconds=time.time() - t0)
    except Exception as e:                                   # A* found no route, or the NLP failed to build
        return key, None, dict(reason=f"{type(e).__name__}: {str(e)[:300]}", seconds=time.time() - t0)
    xy = np.asarray(result["x"])[:, :2]
    np.savez(plan_file, xy=xy, t=np.asarray(result["t"]))
    return key, plan_file, dict(seconds=time.time() - t0, T=float(result["t"][-1]))


# ---------------------------------------------------------------- run folder
def git_state():
    rev = subprocess.run(["git", "-C", str(REPO), "rev-parse", "--short", "HEAD"], capture_output=True, text=True)
    dirty = subprocess.run(["git", "-C", str(REPO), "status", "--porcelain"], capture_output=True, text=True)
    return dict(commit=rev.stdout.strip(), dirty=bool(dirty.stdout.strip()))


def prepare(run_dir: Path, a) -> list:
    """Write the manifest, build scenes, plan, and write one job file per job. Returns the jobs."""
    for sub in ("jobs", "courses", "plans", "results", "logs", "stdout"):
        (run_dir / sub).mkdir(parents=True, exist_ok=True)
    course_path = str(Path(a.course))
    seeds = list(range(a.seeds)) if a.seed_list is None else a.seed_list
    gaps = parse_gaps(a.gaps)
    variants = a.variants or [None]
    jobs = build_jobs(a.controllers, variants, gaps, seeds)
    manifest = dict(created=datetime.now().isoformat(timespec="seconds"), course=course_path,
                    course_sha256=hashlib.sha256(Path(course_path).read_bytes()).hexdigest(), gaps=gaps,
                    variants=variants, seeds=seeds, controllers={c: CONTROLLERS[c] for c in a.controllers},
                    timeout=a.timeout, planner=PLANNER_BASE, git=git_state(), jobs=len(jobs))
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=1, default=str))

    print(f"{len(jobs)} jobs; building scenes")
    scenes = {}
    for v in variants:
        for g in gaps:
            for s in seeds:
                info = ensure_scene(course_path, v, g, s)
                cfg = info["cfg"]
                cfg["name"] = info["name"]                         # run_waypoints loads the scene by this name
                scenes[(v, g, s)] = cfg

    print("planning")
    plans, tasks = {}, {}
    for j in jobs:
        key = (j["tube_margin"], j["gap"], j["seed"])              # the friction variant does not change the plan
        if key not in tasks:
            gid = "nogap" if j["gap"] is None else f"g{j['gap']:.2f}"
            plan_file = run_dir / "plans" / f"tube{j['tube_margin']:.3f}_{gid}_s{j['seed']:03d}.npz"
            tasks[key] = (key, course_path, j["variant"], j["gap"], j["seed"], j["tube_margin"], str(plan_file))
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        for fut in as_completed([pool.submit(plan_job, t) for t in tasks.values()]):
            key, plan_file, info = fut.result()
            plans[key] = (plan_file, info)
            print(f"  plan tube {key[0]:.3f} gap {key[1]} seed {key[2]}: "
                  f"{'ok' if plan_file else 'no route'} ({info['seconds']:.0f} s)", flush=True)

    for j in jobs:
        cfg = scenes[(j["variant"], j["gap"], j["seed"])]
        pose = sample_start(cfg, j["seed"])
        plan_file, plan_info = plans[(j["tube_margin"], j["gap"], j["seed"])]
        course_cfg = dict(cfg)
        course_cfg["course"] = dict(cfg["course"], start=[pose.x, pose.y])
        course_file = run_dir / "courses" / f"{j['id']}.yaml"
        course_file.write_text(yaml.safe_dump(course_cfg, sort_keys=False))
        j.update(start_pose=dict(x=pose.x, y=pose.y, yaw=pose.yaw), course_file=str(course_file),
                 plan_file=plan_file, plan_info=plan_info, timeout=a.timeout, log_dir=str(run_dir / "logs"),
                 result_file=str(run_dir / "results" / f"{j['id']}.json"))
        (run_dir / "jobs" / f"{j['id']}.json").write_text(json.dumps(j, indent=1, default=str))
    return jobs


def run_one(job: dict, run_dir: Path, wall_timeout: float) -> str:
    result_file = Path(job["result_file"])
    if job["plan_file"] is None:                                   # no route: nothing to simulate
        r = no_plan_metrics(job["plan_info"]["reason"])
        r.update({k: job[k] for k in ("id", "controller", "variant", "gap", "seed", "cone_mu", "tube_margin")})
        r["start_pose"] = job["start_pose"]
        result_file.write_text(json.dumps(r, indent=1, default=str))
        return r["outcome"]
    with open(run_dir / "stdout" / f"{job['id']}.txt", "w") as out:
        try:
            subprocess.run([sys.executable, "-m", "trackC.batch_sim", str(run_dir / "jobs" / f"{job['id']}.json")],
                           cwd=REPO, stdout=out, stderr=subprocess.STDOUT, timeout=wall_timeout)
        except subprocess.TimeoutExpired:
            pass
    if not result_file.exists():
        r = dict(outcome="crashed", reached_goal=False, error="no result written (wall-clock timeout or hard crash)")
        r.update({k: job[k] for k in ("id", "controller", "variant", "gap", "seed", "cone_mu", "tube_margin")})
        result_file.write_text(json.dumps(r, indent=1, default=str))
    return json.loads(result_file.read_text())["outcome"]


def run_all(jobs, run_dir: Path, workers: int, wall_timeout: float):
    todo = [j for j in jobs if not Path(j["result_file"]).exists()]
    print(f"running {len(todo)} of {len(jobs)} jobs with {workers} worker(s)")
    t0 = time.time()
    with ThreadPoolExecutor(max_workers=workers) as pool:
        futs = {pool.submit(run_one, j, run_dir, wall_timeout): j for j in todo}
        for k, fut in enumerate(as_completed(futs), 1):
            print(f"  [{k}/{len(todo)}] {futs[fut]['id']}: {fut.result()} ({time.time() - t0:.0f} s elapsed)",
                  flush=True)


# ---------------------------------------------------------------- summary
ROW_KEYS = ("id", "controller", "variant", "gap", "seed", "outcome", "reached_goal", "time_to_goal", "ramp_time",
            "collision_steps", "body_clearance", "circle_slack", "tube_used", "slips", "slip_ratio_median",
            "limit_fraction", "sim_time", "plan_failure", "error")
PASS_RATE_TARGET = 0.8


def _mean(vals):
    vals = [v for v in vals if v is not None and v == v]
    return float(np.mean(vals)) if vals else None


def summarize(run_dir: Path, publish: Path | None = None):
    rows = [json.loads(p.read_text()) for p in sorted((run_dir / "results").glob("*.json"))]
    if not rows:
        raise SystemExit(f"no results in {run_dir / 'results'}")
    with open(run_dir / "runs.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=ROW_KEYS, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)

    groups = {}
    for r in rows:
        groups.setdefault((r["controller"], r["variant"] or "-", r["gap"]), []).append(r)
    summary = []
    for (ctrl, variant, gap), rs in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1], kv[0][2] or 0)):
        ok = [r for r in rs if r["outcome"] == "success"]
        counts = {o: sum(r["outcome"] == o for r in rs) for o in
                  ("success", "collision", "fall", "timeout", "no_feasible_plan", "crashed")}
        summary.append(dict(
            controller=ctrl, variant=variant, gap=gap, runs=len(rs), pass_rate=len(ok) / len(rs), **counts,
            time_to_goal_mean=_mean([r.get("time_to_goal") for r in ok]),
            time_to_goal_median=float(np.median([r["time_to_goal"] for r in ok])) if ok else None,
            ramp_time_mean=_mean([r.get("ramp_time") for r in ok]),
            body_clearance_min=min((r["body_clearance"] for r in rs if r.get("body_clearance") is not None),
                                   default=None),
            tube_used_max=max((r["tube_used"] for r in rs if r.get("tube_used") is not None), default=None),
            slips_mean=_mean([r.get("slips") for r in rs]),
            limit_fraction_mean=_mean([r.get("limit_fraction") for r in rs])))
    with open(run_dir / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0]))
        w.writeheader()
        w.writerows(summary)

    smallest = {}
    for s in summary:
        if s["gap"] is not None and s["pass_rate"] >= PASS_RATE_TARGET:
            k = (s["controller"], s["variant"])
            smallest[k] = min(smallest.get(k, 9.9), s["gap"])
    manifest = json.loads((run_dir / "manifest.json").read_text())
    fmt = lambda v, d=2: "n/a" if v is None else (f"{v:.{d}f}" if isinstance(v, float) else str(v))  # noqa: E731
    lines = [f"# Batch summary: {run_dir.name}", "",
             f"Course `{manifest['course']}`, commit {manifest['git']['commit']}"
             f"{' (uncommitted changes)' if manifest['git']['dirty'] else ''}, timeout {manifest['timeout']} s.", ""]
    for c, spec in manifest["controllers"].items():
        lines.append(f"- **{c}**: {spec.get('note', '')}")
    lines += ["", "| controller | variant | gap [m] | runs | pass | success / collision / fall / timeout / no plan / "
              "crashed | time to goal [s] | ramp [s] | min body clearance [m] | max tube used | slips / run | "
              "time at limit |", "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for s in summary:
        lines.append(
            f"| {s['controller']} | {s['variant']} | {fmt(s['gap'])} | {s['runs']} | {s['pass_rate']:.0%} | "
            f"{s['success']} / {s['collision']} / {s['fall']} / {s['timeout']} / {s['no_feasible_plan']} / "
            f"{s['crashed']} | {fmt(s['time_to_goal_mean'], 1)} | {fmt(s['ramp_time_mean'], 1)} | "
            f"{fmt(s['body_clearance_min'], 3)} | {fmt(s['tube_used_max'])} | {fmt(s['slips_mean'], 1)} | "
            f"{fmt(s['limit_fraction_mean'])} |")
    if smallest:
        lines += ["", f"Smallest gap cleared in at least {PASS_RATE_TARGET:.0%} of runs (A3.5):", ""]
        lines += [f"- {c}, {v} patch: {g:.2f} m" for (c, v), g in sorted(smallest.items())]
    (run_dir / "summary.md").write_text("\n".join(lines) + "\n")
    plot_summary(summary, run_dir)
    if publish:
        publish.mkdir(parents=True, exist_ok=True)
        for name in ("summary.md", "summary.csv", "runs.csv", "manifest.json", "pass_rate.png", "time_to_goal.png"):
            if (run_dir / name).exists():
                (publish / name).write_bytes((run_dir / name).read_bytes())
    print((run_dir / "summary.md").read_text())


def plot_summary(summary, run_dir: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    series = {}
    for s in summary:
        if s["gap"] is not None:
            series.setdefault((s["controller"], s["variant"]), []).append(s)
    if not series:
        return
    for key, ylabel, fname in (("pass_rate", "pass rate", "pass_rate.png"),
                               ("time_to_goal_mean", "mean time to goal [s]", "time_to_goal.png")):
        fig, ax = plt.subplots(figsize=(6.5, 4))
        for (ctrl, variant), ss in sorted(series.items()):
            ss = sorted(ss, key=lambda s: s["gap"])
            xs = [s["gap"] for s in ss if s[key] is not None]
            ys = [s[key] for s in ss if s[key] is not None]
            ax.plot(xs, ys, "o-", label=f"{ctrl}, {variant} patch")
        if key == "pass_rate":
            ax.axhline(PASS_RATE_TARGET, color="0.6", ls="--", lw=1)
            ax.set_ylim(-0.05, 1.05)
        ax.set_xlabel("mid-ramp gap [m]")
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(run_dir / fname, dpi=150)
        plt.close(fig)


# ---------------------------------------------------------------- CLI
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", default="trackC/courses/mixed_v1.yaml")
    ap.add_argument("--gaps", default=None, help="'0.6:1.1:0.1' or '0.8,1.0'; omit for courses without a gap")
    ap.add_argument("--variants", nargs="*", choices=["low", "high"], default=None)
    ap.add_argument("--seeds", type=int, default=20, help="seeds 0..N-1 (20 per gap width, as decided)")
    ap.add_argument("--seed-list", type=int, nargs="*", default=None, help="explicit seeds instead of --seeds")
    ap.add_argument("--controllers", nargs="+", default=["nominal", "robust_follower"], choices=list(CONTROLLERS))
    ap.add_argument("--timeout", type=float, default=150.0, help="simulated seconds per run")
    ap.add_argument("--wall-timeout", type=float, default=3600.0, help="wall-clock seconds before a job is killed")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--out", default="data/batch")
    ap.add_argument("--resume", default=None, help="existing run folder: run only jobs without a result")
    ap.add_argument("--summarize", default=None, help="existing run folder: only rebuild the summary")
    ap.add_argument("--publish", default=None, help="copy the summary files to this folder (e.g. results/batch/x)")
    a = ap.parse_args()

    publish = Path(a.publish) if a.publish else None
    if a.summarize:
        return summarize(Path(a.summarize), publish)
    if a.resume:
        run_dir = Path(a.resume)
        jobs = [json.loads(p.read_text()) for p in sorted((run_dir / "jobs").glob("*.json"))]
    else:
        run_dir = Path(a.out) / datetime.now().strftime("%Y%m%d_%H%M%S")
        jobs = prepare(run_dir, a)
    run_all(jobs, run_dir, a.workers, a.wall_timeout)
    summarize(run_dir, publish)


if __name__ == "__main__":
    main()
