"""Step 1.1: walk the Go2 from start to goal on one of our courses, driven by the reference conversion layer.

Adapted from Quadruped-PyMPC's simulation/simulation.py (pinned commit 025eab4; authors Giulio Turrisi and
Daniel Ordonez). Changes from upstream:
  * The velocity command comes from trackA.reference.path_follower.PathFollower instead of env.target_base_vel()
    (the keyboard in the upstream demo).
  * One episode on one of our courses; it ends when the goal is reached, the robot falls, or the timeout passes.
  * Each run saves a log to data/run_waypoints/ (ignored by git) and prints a summary.
  * Removed: multi-episode reset, h5 recording, heightmaps (our config runs the MPC blind).

Run from the repo root, inside the pixi env (see SETUP.md):
    python -m trackC.terrain_gen trackC/courses/flat_v1.yaml       # once per environment: installs the scene
    python -m trackA.sim.run_waypoints                                     # straight line, flat_v1, viewer on
    python -m trackA.sim.run_waypoints --path zigzag --plot
    python -m trackA.sim.run_waypoints --no-render --timeout 90
"""
import argparse
import copy
import time
from datetime import datetime
from pathlib import Path

import mujoco
import numpy as np
import yaml
from gym_quadruped.quadruped_env import QuadrupedEnv
from gym_quadruped.utils.mujoco.visual import render_line, render_sphere, render_vector
from gym_quadruped.utils.quadruped_utils import LegsAttr
from quadruped_pympc import config as cfg
from quadruped_pympc.helpers.quadruped_utils import plot_swing_mujoco
from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper

from trackA.reference.path_follower import PathFollower
from trackC.course import Course

LEGS_ORDER = ("FL", "FR", "RL", "RR")
RENDER_FREQ = 30  # [Hz]
FRICTION = 0.5    # MuJoCo floor plane only (10 cm below our course, never touched); fixed for repeatable runs

# Path drawing in the viewer (RGBA)
PATH_LIFT = 0.01                                   # [m] draw this far above the course surface
WAYPOINT_COLOR = np.array([1.0, 0.55, 0.0, 0.9])   # orange spheres
GOAL_COLOR = np.array([1.0, 0.85, 0.0, 0.9])       # yellow sphere at the last waypoint
SEGMENT_COLOR = np.array([0.2, 0.2, 0.9, 0.6])     # blue: segments not being followed
ACTIVE_COLOR = np.array([0.0, 0.85, 1.0, 0.9])     # cyan: segment the follower is tracking now


def draw_path(viewer, waypoints: np.ndarray, course: Course, active_seg: int, geom_ids: dict) -> dict:
    """Draw the waypoints (spheres) and the segments between them (lines) on the course surface.

    The segment PathFollower is currently tracking is drawn thicker and in cyan. Segments that cross the bottom
    or top of the ramp are split there, so the line lies on the surface instead of cutting through the ramp.
    geom_ids carries the viewer geom slots between calls so each frame updates the same geoms.
    """
    def lift(xy):
        return np.array([xy[0], xy[1], course.z(xy[0]) + PATH_LIFT])

    for k, xy in enumerate(waypoints):
        color = GOAL_COLOR if k == len(waypoints) - 1 else WAYPOINT_COLOR
        geom_ids[("wp", k)] = render_sphere(viewer, lift(xy), diameter=0.06, color=color,
                                            geom_id=geom_ids.get(("wp", k), -1))
    for k in range(len(waypoints) - 1):
        a, b = waypoints[k], waypoints[k + 1]
        # split points: the segment's ends plus any crease (ramp bottom x1, ramp top x2) strictly between them
        ts = [0.0, 1.0] + [(xc - a[0]) / (b[0] - a[0]) for xc in (course.x1, course.x2)
                           if b[0] != a[0] and 0.0 < (xc - a[0]) / (b[0] - a[0]) < 1.0]
        ts = sorted(ts)
        active = k == active_seg
        for j in range(len(ts) - 1):
            p0, p1 = lift(a + ts[j] * (b - a)), lift(a + ts[j + 1] * (b - a))
            geom_ids[("seg", k, j)] = render_line(viewer, p0, p1, width=0.012 if active else 0.006,
                                                  color=ACTIVE_COLOR if active else SEGMENT_COLOR,
                                                  geom_id=geom_ids.get(("seg", k, j), -1))
    return geom_ids


# ---------------------------------------------------------------- waypoints
def make_waypoints(kind: str, start, goal, amplitude: float = 1.0, n_legs: int = 5) -> np.ndarray:
    """Hand-placed waypoint paths for step 1.1. Later the planner's path replaces these: result["x"][:, :2].

    straight: start -> goal.
    zigzag:   start -> alternating +-amplitude in y -> goal, n_legs segments evenly spaced in x. With the
              3 m wide course, amplitude 1.0 keeps the path 0.5 m from the walls (the wall-clearance rule).
    """
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    if kind == "straight":
        return np.array([start, goal])
    if kind == "zigzag":
        xs = np.linspace(start[0], goal[0], n_legs + 1)
        ys = [start[1]] + [amplitude * (1 if k % 2 == 0 else -1) for k in range(n_legs - 1)] + [goal[1]]
        return np.column_stack([xs, ys])
    raise ValueError(f"unknown path kind: {kind}")


# ---------------------------------------------------------------- simulation
def run(course_path: str, path_kind: str = "straight", v_ref: float = 0.4, timeout: float = 60.0,
        render: bool = True, log_dir: str = "data/run_waypoints", plot: bool = False) -> dict:
    course_cfg = yaml.safe_load(open(course_path))
    scene = course_cfg["name"]
    start, goal = course_cfg["course"]["start"], course_cfg["course"]["goal"]

    # Settings go on the config module from here; Quadruped-PyMPC's own files are never edited (SETUP.md).
    cfg.simulation_params["scene"] = scene
    if cfg.locomotion_params["visual_foothold_adaptation"] != "blind":
        raise RuntimeError("run_waypoints.py has no heightmaps; set visual_foothold_adaptation to 'blind'")
    sim_dt = cfg.simulation_params["dt"]

    env = QuadrupedEnv(
        robot=cfg.robot,
        scene=scene,
        sim_dt=sim_dt,
        ref_base_lin_vel=0.0,             # the env's own command is unused: PathFollower replaces it
        ref_base_ang_vel=0.0,
        ground_friction_coeff=FRICTION,
        base_vel_command_type="forward",  # not "human": no keyboard input
        state_obs_names=(),
    )
    env.mjModel.opt.gravity[2] = -cfg.gravity_constant
    if cfg.qpos0_js is not None:  # some robots need a different zero joint configuration
        env.mjModel.qpos0 = np.concatenate((env.mjModel.qpos0[:7], cfg.qpos0_js))
    env.reset(random=False)

    spawn_xy = env.base_pos[:2].copy()
    if np.linalg.norm(spawn_xy - np.asarray(start)) > 0.2:
        print(f"warning: robot spawned at {spawn_xy}, course start is {start}")

    waypoints = make_waypoints(path_kind, start, goal)
    follower = PathFollower(waypoints, v_ref=v_ref)
    course = Course(course_cfg["course"])   # surface height z(x), so the path is drawn on ramps too
    path_geom_ids = {}

    if render:
        env.render()
        env.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        env.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
        path_geom_ids = draw_path(env.viewer, follower.waypoints, course, follower.seg_idx, path_geom_ids)
        env.render()

    # Controller -------------------------------------------------------------------------------------------
    tau = LegsAttr(*[np.zeros((env.mjModel.nv, 1)) for _ in range(4)])
    tau_soft_limits_scalar = 0.9
    tau_limits = LegsAttr(
        FL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FL] * tau_soft_limits_scalar,
        FR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FR] * tau_soft_limits_scalar,
        RL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RL] * tau_soft_limits_scalar,
        RR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RR] * tau_soft_limits_scalar,
    )
    feet_traj_geom_ids, feet_GRF_geom_ids = None, LegsAttr(FL=-1, FR=-1, RL=-1, RR=-1)
    wrapper = QuadrupedPyMPC_Wrapper(
        initial_feet_pos=env.feet_pos,
        legs_order=LEGS_ORDER,
        feet_geom_id=env._feet_geom_id,
        quadrupedpympc_observables_names=("ref_feet_pos", "nmpc_footholds", "swing_time", "lift_off_positions"),
    )

    # Loop ---------------------------------------------------------------------------------------------------
    log = {k: [] for k in ("t", "base_pos", "yaw", "base_lin_vel", "ref_lin_vel", "ref_ang_vel",
                           "cross_track_error", "seg_idx")}
    outcome = "timeout"
    last_render_time = time.time()
    n_steps = int(timeout / sim_dt)

    for _ in range(n_steps):
        feet_pos = env.feet_pos(frame="world")
        feet_vel = env.feet_vel(frame="world")
        hip_pos = env.hip_positions(frame="world")
        base_lin_vel = env.base_lin_vel(frame="world")
        base_ang_vel = env.base_ang_vel(frame="base")
        base_ori_euler_xyz = env.base_ori_euler_xyz
        base_pos = copy.deepcopy(env.base_pos)
        com_pos = copy.deepcopy(env.com)

        # Reference conversion: replaces `env.target_base_vel()` from the upstream loop ------------------
        ref_base_lin_vel, ref_base_ang_vel, done = follower.update(base_pos[:2], base_ori_euler_xyz[2], sim_dt)

        log["t"].append(env.simulation_time)
        log["base_pos"].append(base_pos.copy())
        log["yaw"].append(base_ori_euler_xyz[2])
        log["base_lin_vel"].append(base_lin_vel.copy())
        log["ref_lin_vel"].append(np.array(ref_base_lin_vel, dtype=float))
        log["ref_ang_vel"].append(np.array(ref_base_ang_vel, dtype=float))
        log["cross_track_error"].append(follower.cross_track_error)
        log["seg_idx"].append(follower.seg_idx)

        if done:
            outcome = "goal"
            break

        if cfg.mpc_params["use_inertia_recomputation"]:
            inertia = env.get_base_inertia().flatten()
        else:
            inertia = cfg.inertia.flatten()

        qpos, qvel = env.mjData.qpos, env.mjData.qvel
        legs_qvel_idx = env.legs_qvel_idx
        legs_qpos_idx = env.legs_qpos_idx
        joints_pos = LegsAttr(FL=legs_qvel_idx.FL, FR=legs_qvel_idx.FR, RL=legs_qvel_idx.RL, RR=legs_qvel_idx.RR)

        tau = wrapper.compute_actions(
            com_pos, base_pos, base_lin_vel, base_ori_euler_xyz, base_ang_vel,
            feet_pos, hip_pos, joints_pos,
            None,  # heightmaps (blind)
            list(LEGS_ORDER), sim_dt,
            ref_base_lin_vel, ref_base_ang_vel,
            env.step_num, qpos, qvel,
            env.feet_jacobians(frame="world", return_rot_jac=False),
            env.feet_jacobians_dot(frame="world", return_rot_jac=False),
            feet_vel,
            env.legs_qfrc_passive, env.legs_qfrc_bias, env.legs_mass_matrix,
            legs_qpos_idx, legs_qvel_idx,
            tau, inertia, env.mjData.contact,
        )
        for leg in LEGS_ORDER:
            tau[leg] = np.clip(tau[leg], tau_limits[leg][:, 0], tau_limits[leg][:, 1])

        action = np.zeros(env.mjModel.nu)
        for leg in LEGS_ORDER:
            action[env.legs_tau_idx[leg]] = tau[leg]
        _, _, is_terminated, _, info = env.step(action=action)

        if is_terminated:  # body (not leg) touched the world, or left the terrain bounds
            outcome = "terminated"
            print(f"terminated: {list(info.get('invalid_contacts', {}).keys())}")
            break

        if render and (time.time() - last_render_time > 1.0 / RENDER_FREQ or env.step_num == 1):
            ctrl_state = wrapper.get_obs()
            _, _, feet_GRF = env.feet_contact_state(ground_reaction_forces=True)
            feet_traj_geom_ids = plot_swing_mujoco(
                viewer=env.viewer,
                swing_traj_controller=wrapper.wb_interface.stc,
                swing_period=wrapper.wb_interface.stc.swing_period,
                swing_time=LegsAttr(**{leg: ctrl_state["swing_time"][i] for i, leg in enumerate(LEGS_ORDER)}),
                lift_off_positions=ctrl_state["lift_off_positions"],
                nmpc_footholds=ctrl_state["nmpc_footholds"],
                ref_feet_pos=ctrl_state["ref_feet_pos"],
                early_stance_detector=wrapper.wb_interface.esd,
                geom_ids=feet_traj_geom_ids,
            )
            for leg in LEGS_ORDER:
                feet_GRF_geom_ids[leg] = render_vector(
                    env.viewer, vector=feet_GRF[leg], pos=feet_pos[leg],
                    scale=np.linalg.norm(feet_GRF[leg]) * 0.005,
                    color=np.array([0, 1, 0, 0.5]), geom_id=feet_GRF_geom_ids[leg],
                )
            path_geom_ids = draw_path(env.viewer, follower.waypoints, course, follower.seg_idx, path_geom_ids)
            env.render()
            last_render_time = time.time()

    env.close()

    # Summary and log ----------------------------------------------------------------------------------------
    arrays = {k: np.asarray(v) for k, v in log.items()}
    e = arrays["cross_track_error"]
    final_xy = arrays["base_pos"][-1, :2]
    summary = dict(
        outcome=outcome,
        time=float(arrays["t"][-1] - arrays["t"][0]),
        final_dist_to_goal=float(np.linalg.norm(final_xy - np.asarray(goal))),
        max_abs_cross_track=float(np.max(np.abs(e))),
        rms_cross_track=float(np.sqrt(np.mean(e ** 2))),
    )
    print(f"\n{scene} / {path_kind}: {outcome} after {summary['time']:.2f} s | "
          f"final distance to goal {summary['final_dist_to_goal']:.3f} m | "
          f"cross-track max {summary['max_abs_cross_track']:.3f} m, rms {summary['rms_cross_track']:.3f} m")

    out_dir = Path(log_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = out_dir / f"{scene}_{path_kind}_{datetime.now():%Y%m%d_%H%M%S}"
    np.savez(f"{stem}.npz", waypoints=waypoints, goal=np.asarray(goal), v_ref=v_ref,
             **arrays, **{f"summary_{k}": v for k, v in summary.items()})
    print(f"log: {stem}.npz")

    if plot:
        plot_run(arrays, waypoints, f"{stem}.png", title=f"{scene} / {path_kind}: {outcome}",
                 half_width=course_cfg["course"]["width"] / 2)
        print(f"plot: {stem}.png")
    return summary


def plot_run(arrays: dict, waypoints: np.ndarray, out_path: str, title: str = "", half_width: float = 1.5):
    """Plan view of the path against the robot's base track, and the cross-track error over time.

    The plan view always spans the course width (+-half_width, wall faces dashed), so a straight run with
    centimeter-level error doesn't collapse the equal-aspect axes.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), gridspec_kw=dict(height_ratios=[2, 1]))
    ax1.plot(waypoints[:, 0], waypoints[:, 1], "o--", color="0.5", label="waypoint path")
    ax1.plot(arrays["base_pos"][:, 0], arrays["base_pos"][:, 1], color="C0", label="base")
    for y in (-half_width, half_width):
        ax1.axhline(y, color="0.3", ls=":", lw=1)
    ax1.set_ylim(-half_width - 0.2, half_width + 0.2)
    ax1.set_aspect("equal")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.legend()
    ax1.set_title(title)
    ax2.plot(arrays["t"], arrays["cross_track_error"], color="C1")
    ax2.axhline(0, color="0.7", lw=0.8)
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel("cross-track error [m]")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", default="trackC/courses/flat_v1.yaml")
    ap.add_argument("--path", default="straight", choices=["straight", "zigzag"])
    ap.add_argument("--v-ref", type=float, default=0.4)
    ap.add_argument("--timeout", type=float, default=60.0, help="seconds of simulated time")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--plot", action="store_true", help="save a path / cross-track plot next to the log")
    a = ap.parse_args()
    run(a.course, a.path, a.v_ref, a.timeout, render=not a.no_render, plot=a.plot)
