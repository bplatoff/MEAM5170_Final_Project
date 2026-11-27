"""One full run with the global planner and the mid layer (task 2.4).

Controllers, all following the same plan with friction cones at mu_lo:
    follower   the stage 1 PathFollower on the planned path (baseline)
    lqr        planner reference tracked by LQR, QP off
    tube       QP plus LQR; --qp-start measured (default) or carried

Run from the repo root, inside the pixi env:
    python -m trackA.sim.run_midlayer --controller tube                     # viewer on
    python -m trackA.sim.run_midlayer --controller lqr --no-render --seed 3
    python -m trackA.sim.run_midlayer --controller tube --no-render --push worst

Logs go to data/run_midlayer/ (ignored by git).
"""
import argparse
import fcntl
import hashlib
import json
import pickle
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml
from shapely.geometry import Polygon, box

from trackA.mid_layer.config import MidLayerConfig
from trackA.mid_layer.controller import TubeMidLayer
from trackA.mid_layer.planner_adapter import NoFeasiblePlan, load_course, plan_for_course
from trackA.reference.path_follower import PathFollower

DEFAULT_COURSE = "trackA/courses/midlayer_test_v1.yaml"
WORST_ACCEL = 1.962     # [m/s^2] (mu_plan - mu_lo) g, the largest shortfall the tube is sized for
PUSH_DURATION = 1.5     # [s]
PUSH_LEAD = 0.3         # [m] the push starts when the plan is this far before the obstacle
PUSH_CHOICES = ("none", "worst")
CONTROLLERS = ("follower", "lqr", "tube")


class PlannedPathFollower:
    """PathFollower on the plan's knots, with the runner's follower_factory interface."""

    def __init__(self, plan, v_ref=0.35):
        xy = plan.reference.x[:, :2]
        idx = np.unique(np.r_[np.arange(0, len(xy), 4), len(xy) - 1])
        self.inner = PathFollower(xy[idx], v_ref=v_ref)
        self.waypoints = self.inner.waypoints

    @property
    def seg_idx(self):
        return self.inner.seg_idx

    @property
    def cross_track_error(self):
        return self.inner.cross_track_error

    def update(self, pos_xy, vel_xy, yaw, dt):
        return self.inner.update(pos_xy, yaw, dt)


def ensure_scene(course_path):
    """Install the course's MuJoCo scene if it is missing or out of date."""
    import gym_quadruped
    from trackC.terrain_gen import build
    with open(course_path) as f:
        xml, info = build(yaml.safe_load(f))
    target = Path(gym_quadruped.__file__).parent / "robot_model" / f"scene_{info['name']}.xml"
    if not target.exists() or target.read_text() != xml:
        target.write_text(xml)


def cached_plan(course, spawn_xy, spawn_yaw, cfg, cache_dir=None):
    """Plan once per course, spawn pose and planner settings, so every controller on a seed follows the same plan."""
    if cache_dir is None:
        return plan_for_course(course, spawn_xy, spawn_yaw, cfg)
    folder = Path(cache_dir)
    folder.mkdir(parents=True, exist_ok=True)
    planner_cfg = replace(cfg, conversion="lag_inverse", use_qp=True, qp_start="measured")
    text = f"{Path(course).read_text()}|{np.round(spawn_xy, 9).tolist()}|{spawn_yaw:.9f}|{planner_cfg}"
    key = hashlib.sha256(text.encode()).hexdigest()[:24]
    with open(folder / f"{key}.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        path = folder / f"{key}.pkl"
        if path.exists():
            return pickle.loads(path.read_bytes())
        plan = plan_for_course(course, spawn_xy, spawn_yaw, cfg)    # NoFeasiblePlan is not cached
        temp = path.with_suffix(".tmp")
        temp.write_bytes(pickle.dumps(plan))
        temp.replace(path)
        return plan


def obstacle_pushes(plan, accel=WORST_ACCEL):
    """One sideways push toward the nearest obstacle as the plan reaches each obstacle group: (t0, t1, a_x, a_y)."""
    ref, groups = plan.reference, {}
    for o in plan.geometry.obstacles:
        groups.setdefault(round(float(o[:, 0].mean()), 2), []).append(o)
    times = np.arange(0.0, ref.t_end, 0.02)
    xs = np.array([ref.state(s)[0] for s in times])
    out = []
    for x_ob, members in sorted(groups.items()):
        hit = np.flatnonzero(xs >= x_ob - PUSH_LEAD)
        if not hit.size:
            continue
        t0 = float(times[hit[0]])
        y = ref.state(t0)[1]
        gaps = [(min(abs(o[:, 1].min() - y), abs(o[:, 1].max() - y)), np.sign(o[:, 1].mean() - y)) for o in members]
        out.append((t0, t0 + PUSH_DURATION, 0.0, float(min(gaps)[1]) * accel))
    return out


def footprint(x, y, yaw, length, width):
    c, s = np.cos(yaw), np.sin(yaw)
    corners = [(length / 2, width / 2), (-length / 2, width / 2), (-length / 2, -width / 2), (length / 2, -width / 2)]
    return Polygon([(x + c * a - s * b, y + s * a + c * b) for a, b in corners])


def time_outside(values, limit, dt) -> float:
    """Total time [s] that values exceed limit."""
    return float(np.sum(np.asarray(values) > limit) * dt)


def bare_clearance(log, geometry, cfg, every=25) -> float:
    """Smallest distance from the bare body rectangle to any obstacle or wall (decision A2.4). Negative if inside."""
    x_lo, x_hi, y_lo, y_hi = geometry.bounds
    interior = box(x_lo, y_lo, x_hi, y_hi)
    obstacles = [Polygon(o) for o in geometry.obstacles]
    best = np.inf
    for (x, y, _), yaw in zip(log["base_pos"][::every], log["yaw"][::every]):
        body = footprint(float(x), float(y), float(yaw), cfg.robot_length, cfg.robot_width)
        if not interior.contains(body):
            return min(best, -float(body.difference(interior).area > 0))
        wall = interior.exterior.distance(body)
        near = min((o.distance(body) for o in obstacles), default=np.inf)
        if any(o.intersects(body) for o in obstacles):
            return -1.0
        best = min(best, wall, near)
    return float(best)


def run_once(course=DEFAULT_COURSE, controller="tube", seed=None, push="none", conversion="lag_inverse", timeout=120.0,
             render=False, log_dir="data/run_midlayer", tag=None, plot=False, cfg=None, solver_dir=None,
             push_accel=WORST_ACCEL, footprint=None, qp_start=None, plan_cache=None) -> dict:
    if solver_dir:                  # a private acados build folder, so runs can go in parallel
        import controllers.centroidal_nmpc_nominal_1_4 as low_level
        Path(solver_dir).mkdir(parents=True, exist_ok=True)
        low_level.CODE_EXPORT_DIR = Path(solver_dir).resolve()
    from trackA.sim.run_waypoints import run
    if controller not in CONTROLLERS:
        raise ValueError(f"controller must be one of {CONTROLLERS}")
    cfg = cfg if cfg is not None else MidLayerConfig()
    cfg = replace(cfg, conversion=conversion, use_qp=controller == "tube", footprint=footprint or cfg.footprint,
                  qp_start=qp_start or cfg.qp_start)
    ensure_scene(course)
    store = {}

    def factory(spawn_xy, spawn_yaw):
        plan = cached_plan(course, spawn_xy, spawn_yaw, cfg, plan_cache)
        store["plan"] = plan
        store["ctrl"] = PlannedPathFollower(plan) if controller == "follower" else TubeMidLayer(plan, cfg)
        store["pushes"] = obstacle_pushes(plan, push_accel) if push == "worst" else []
        return store["ctrl"]

    def push_now(t):
        for t0, t1, ax, ay in store.get("pushes", ()):
            if t0 <= t < t1:
                return ax, ay
        return 0.0, 0.0

    tag = tag or f"{Path(course).stem}_{controller}_s{'x' if seed is None else seed}_{push}"
    base = dict(course=str(course), controller=controller, seed=seed, push=push, push_accel=push_accel if push != "none" else 0.0,
                conversion=conversion, footprint=cfg.footprint, qp_start=cfg.qp_start if controller == "tube" else None)
    try:
        summary = run(course, timeout=timeout, render=render, log_dir=log_dir, plot=plot, mpc_kind="perfoot",
                      mu_feet=[cfg.mu_lo] * 4, seed=seed, tag=tag, follower_factory=factory, push=push_now if push != "none" else None)
    except NoFeasiblePlan as err:
        return dict(base, outcome="no feasible plan", detail=str(err))

    plan, ctrl = store["plan"], store["ctrl"]
    log = np.load(summary["log"], allow_pickle=False)
    status = log["mpc_status"]
    out = dict(base, outcome=summary["outcome"], time=summary["time"], final_dist=summary["final_dist_to_goal"],
               wall_contact_steps=int(summary["wall_contact_steps"]), max_cross_track=summary["max_abs_cross_track"],
               low_level_failed_solves=int(np.sum((status != 0) & (status != 2))),
               min_bare_clearance=bare_clearance(log, plan.geometry, cfg), plan_T=plan.T,
               pushes=store.get("pushes", []), plan_attempts=plan.attempts, plan_clearance=plan.min_clearance, tube_radius=plan.tube_radius,
               log=summary["log"])
    if isinstance(ctrl, TubeMidLayer):
        err, alpha, pnorm = log["mid_pos_error"], log["mid_tube_alpha"], log["mid_e_pnorm"]
        radius = log["mid_tube_radius"]
        ref_err, yaw_err = log["mid_ref_error"], log["mid_yaw_error"]
        sim_dt = float(summary.get("sim_dt", 0.002))
        running = alpha > 0
        out.update(max_pos_error=float(err.max()), rms_pos_error=float(np.sqrt(np.mean(err ** 2))),
                   max_e_over_alpha=float((pnorm[running] / alpha[running]).max()) if running.any() else 0.0,
                   max_pos_over_radius=float((err[running] / radius[running]).max()) if running.any() else 0.0,
                   max_ref_error=float(ref_err.max()), rms_ref_error=float(np.sqrt(np.mean(ref_err ** 2))),
                   max_ref_over_tube=float(ref_err.max() / plan.tube_radius),
                   time_outside_plan_tube=time_outside(ref_err, plan.tube_radius, sim_dt),
                   max_yaw_error=float(yaw_err.max()),
                   time_over_eps_yaw=time_outside(yaw_err, cfg.eps_yaw, sim_dt),
                   **ctrl.summary())
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", default=DEFAULT_COURSE)
    ap.add_argument("--controller", default="tube", choices=CONTROLLERS)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--push", default="none", choices=PUSH_CHOICES)
    ap.add_argument("--push-accel", type=float, default=WORST_ACCEL, help="push size [m/s^2]")
    ap.add_argument("--v-lat-max", type=float, default=None, help="override the sideways command limit [m/s]")
    ap.add_argument("--conversion", default="lag_inverse", choices=("lag_inverse", "b3"))
    ap.add_argument("--footprint", default=None, choices=("three_circle", "single_circle"))
    ap.add_argument("--qp-start", default=None, choices=("measured", "carried"))
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--log-dir", default="data/run_midlayer")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--json", default=None, help="also write the result to this file")
    ap.add_argument("--solver-dir", default=None, help="private acados build folder (for parallel runs)")
    ap.add_argument("--plan-cache", default=None, help="folder of saved plans, shared by runs on the same seed")
    a = ap.parse_args()
    result = run_once(a.course, a.controller, a.seed, a.push, a.conversion, a.timeout, render=not a.no_render,
                      log_dir=a.log_dir, tag=a.tag, plot=a.plot, solver_dir=a.solver_dir,
                      push_accel=a.push_accel, footprint=a.footprint, qp_start=a.qp_start, plan_cache=a.plan_cache,
                      cfg=replace(MidLayerConfig(), v_cmd_max_lateral=a.v_lat_max) if a.v_lat_max else None)
    text = json.dumps(result, indent=1, default=float)
    print(text)
    if a.json:
        Path(a.json).write_text(text)
