"""Run one batch job in its own process: simulate, then compute and save the metrics.

    python -m trackC.batch_sim <run folder>/jobs/<job id>.json

Called by trackC.batch, one process per job, so a crashed or hung simulation never takes the sweep down.

TEMPORARY SHIM. trackA.sim.run_waypoints.run() does not yet accept a start pose (x, y, yaw) or a path array (the
interface agreed with Jacob). Until it does, this module swaps two of run_waypoints' helpers for this process only:
  * spawn_with_jitter: places the robot at the job's sampled start pose instead of the course start plus jitter
  * make_waypoints:    returns the planner's path instead of a straight line or zigzag
and it holds a file lock while the scene and acados solvers are built, so parallel jobs don't build at the same time
(the same lock file as Jacob's acquire_build_lock on trackA-2-4). Nothing in run_waypoints.py is edited. When
run() takes these arguments, `simulate` below becomes a plain call and the shim goes away.
"""
import fcntl
import json
import sys
import tempfile
import traceback
from pathlib import Path

import numpy as np
import yaml

from trackC.course import Course
from trackC.metrics import run_metrics

BUILD_LOCK = Path(tempfile.gettempdir()) / "meam5170_acados_build.lock"


def simulate(job: dict) -> dict:
    import trackA.sim.run_waypoints as rw
    from scipy.spatial.transform import Rotation

    pose = job["start_pose"]
    plan_xy = np.load(job["plan_file"])["xy"]

    def spawn_at_pose(env, start, course: Course, seed):
        env.reset(random=False)
        qpos, qvel = env.mjData.qpos.copy(), np.zeros_like(env.mjData.qvel)
        qpos[0], qpos[1] = pose["x"], pose["y"]
        qpos[2] += course.z(pose["x"])
        rot = Rotation.from_euler("z", pose["yaw"]) * Rotation.from_quat(np.roll(qpos[3:7], -1))
        qpos[3:7] = np.roll(rot.as_quat(), 1)
        env.reset(qpos=qpos, qvel=qvel)
        return 0.0, 0.0, 0.0

    lock = open(BUILD_LOCK, "w")
    fcntl.flock(lock, fcntl.LOCK_EX)

    class ReleasingSolveRecorder(rw.SolveRecorder):
        """run() builds SolveRecorder right after the MPC solvers are built: release the build lock there."""
        def __init__(self, *a, **k):
            lock.close()
            super().__init__(*a, **k)

    rw.spawn_with_jitter = spawn_at_pose
    rw.make_waypoints = lambda kind, start, goal, **kw: plan_xy
    rw.SolveRecorder = ReleasingSolveRecorder
    try:
        return rw.run(job["course_file"], path_kind="straight", v_ref=job["v_ref"], timeout=job["timeout"],
                      render=False, log_dir=job["log_dir"], plot=False, mpc_kind="perfoot",
                      mu_feet=[job["cone_mu"]] * 4, tag=job["id"])
    finally:
        if not lock.closed:
            lock.close()


def main(job_path: str):
    job = json.loads(Path(job_path).read_text())
    out = Path(job["result_file"])
    try:
        summary = simulate(job)
        with open(job["course_file"]) as f:
            cfg = yaml.safe_load(f)
        result = run_metrics(np.load(summary["log"], allow_pickle=True), cfg, job)
        result["log"] = summary["log"]
    except Exception as e:                        # a crash is a result too: record it and move on
        result = dict(outcome="crashed", reached_goal=False, error=f"{type(e).__name__}: {e}",
                      traceback=traceback.format_exc(limit=5))
    result.update({k: job[k] for k in ("id", "controller", "variant", "gap", "seed", "cone_mu", "tube_margin")})
    result["start_pose"] = job["start_pose"]
    out.write_text(json.dumps(result, indent=1, default=str))


if __name__ == "__main__":
    main(sys.argv[1])
