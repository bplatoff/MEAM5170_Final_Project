"""Mid-layer tube controller (task 2.4): nominal QP at 10 Hz, LQR feedback and yaw loop at 50 Hz.

update() has the same outputs as PathFollower.update(), plus it takes the measured velocity:
    ref_lin_vel, ref_ang_vel, done = mid_layer.update(base_pos[:2], base_lin_vel[:2], yaw, sim_dt)

With cfg.use_qp = False the nominal is the planner's reference and only LQR acts.
"""
import numpy as np

from trackA.mid_layer.feedback import lqr_acceleration, velocity_command, yaw_rate_command
from trackA.mid_layer.nominal_qp import NominalQP
from trackA.mid_layer.tube import TubeTracker

SLACK_TOL = 1e-3        # [m] above this the solve no longer supports the guarantee
QP_BUDGET = 0.10        # [s]
RADIUS_TOL = 1e-3       # [m]

LOG_KEYS = ("mid_x_nom", "mid_a_nom", "mid_a_cmd", "mid_v_cmd", "mid_w_cmd", "mid_yaw_ref", "mid_tube_alpha",
            "mid_tube_radius", "mid_e_pnorm", "mid_pos_error", "mid_qp_time", "mid_qp_slack", "mid_qp_fallback",
            "mid_qp_overrun", "mid_tube_violation", "mid_cmd_saturated")


class TubeMidLayer:
    log_keys = LOG_KEYS

    def __init__(self, plan, cfg, model=None):
        self.plan, self.cfg, self.ref = plan, cfg, plan.reference
        self.tube = TubeTracker(cfg, model)
        self.K = self.tube.K
        self.A_fb, self.B_fb = self.tube.model["A"], self.tube.model["B"]
        self.goal = plan.reference.x[-1, :2].copy()

        # Same attributes as PathFollower, used by the runner's logging and viewer
        self.waypoints = plan.reference.x[:, :2].copy()
        self.seg_idx = 0
        self.cross_track_error = 0.0
        self.done = False

        self.qp = None
        if cfg.use_qp:
            self.qp = NominalQP(cfg, plan.geometry.bounds, plan.geometry.obstacles, slope_at=plan.geometry.slope_at)
        self.step_count = 0
        self.fb_count = 0
        self.fb_every = self.qp_every = None
        self.t_nom = 0.0
        self.x_nom = self.ref.state(0.0)
        self.a_nom = self.ref.accel(0.0)
        self.plan_x = self.plan_a = None
        self.v_cmd, self.w_cmd = np.zeros(2), 0.0
        self.counters = dict(qp_solves=0, qp_fallbacks=0, qp_overruns=0, slack_solves=0, tube_violations=0,
                             saturated=0, updates=0)
        self.qp_times = []
        self.max_slack = 0.0
        self._row = self._empty_row()

    def _empty_row(self):
        return dict(mid_x_nom=self.x_nom.copy(), mid_a_nom=np.zeros(2), mid_a_cmd=np.zeros(2), mid_v_cmd=np.zeros(2),
                    mid_w_cmd=0.0, mid_yaw_ref=self.ref.yaw(0.0), mid_tube_alpha=0.0, mid_tube_radius=0.0,
                    mid_e_pnorm=0.0, mid_pos_error=0.0, mid_qp_time=0.0, mid_qp_slack=0.0, mid_qp_fallback=False,
                    mid_qp_overrun=False, mid_tube_violation=False, mid_cmd_saturated=False)

    def log_row(self):
        return self._row

    # ------------------------------------------------------------------ main loop
    def update(self, pos_xy, vel_xy, yaw, sim_dt):
        if self.fb_every is None:
            self._set_rates(sim_dt)
            self.tube.reset(np.r_[pos_xy, vel_xy] - self.x_nom)

        if self.done or (np.linalg.norm(np.asarray(pos_xy) - self.goal) <= self.cfg.goal_tol
                         and np.linalg.norm(self.x_nom[:2] - self.goal) <= self.cfg.goal_tol):
            self.done = True
            self.v_cmd, self.w_cmd = np.zeros(2), 0.0
            return np.zeros(3), np.zeros(3), True

        if self.step_count % self.qp_every == 0:
            self._update_nominal_plan()
        if self.step_count % self.fb_every == 0:
            self._feedback(np.asarray(pos_xy, float), np.asarray(vel_xy, float), float(yaw))
        self.step_count += 1
        return np.r_[self.v_cmd, 0.0], np.array([0.0, 0.0, self.w_cmd]), False

    def _set_rates(self, sim_dt):
        for name, period in (("dt_fb", self.cfg.dt_fb), ("dt_qp", self.cfg.dt_qp)):
            ratio = period / sim_dt
            if abs(ratio - round(ratio)) > 1e-6:
                raise ValueError(f"{name} = {period} is not a whole number of simulation steps of {sim_dt}")
        self.fb_every = int(round(self.cfg.dt_fb / sim_dt))
        self.qp_every = int(round(self.cfg.dt_qp / sim_dt))

    # ------------------------------------------------------------------ nominal plan
    def _reference_horizon(self):
        N, dt = self.cfg.horizon, self.cfg.dt_qp
        times = self.t_nom + np.arange(N + 1) * dt
        return (np.array([self.ref.state(s) for s in times]), np.array([self.ref.accel(s) for s in times[:-1]]),
                np.array([self.ref.yaw(s) for s in times]))

    def _update_nominal_plan(self):
        if self.qp is None or self.t_nom >= self.ref.t_end:
            self.x_nom, self.a_nom = self.ref.state(self.t_nom), self.ref.accel(self.t_nom)
            return
        x_ref, a_ref, yaw_ref = self._reference_horizon()
        radii = self.tube.stage_radii(self.cfg.horizon, self.cfg.qp_substeps)
        allowed = self.plan.tube_radius + self.plan.info.get("r_extra", 0.0) + RADIUS_TOL
        if radii.max() > allowed:
            raise ValueError(f"tube radius {radii.max():.4f} m exceeds the planner's inflation {allowed:.4f} m")
        prev = None
        if self.plan_x is not None:
            prev = np.vstack((self.plan_x[1:, :2], x_ref[-1, :2]))
        sol = self.qp.solve(self.x_nom, x_ref, a_ref, yaw_ref, self.plan.circle_radius, radii, prev)
        self.counters["qp_solves"] += 1
        self.qp_times.append(sol.solve_time)
        overrun = sol.solve_time > QP_BUDGET
        self.counters["qp_overruns"] += int(overrun)
        slack = float(sol.slack.max()) if sol.success else 0.0
        if sol.success:
            self.plan_x, self.plan_a = sol.x, sol.a
            self.a_nom = sol.a[0].copy()
            self.max_slack = max(self.max_slack, slack)
            self.counters["slack_solves"] += int(slack > SLACK_TOL)
        else:
            self.counters["qp_fallbacks"] += 1
            if self.plan_a is not None:                         # keep the previous plan, one stage later
                self.plan_x = np.vstack((self.plan_x[1:], x_ref[-1]))
                self.plan_a = np.vstack((self.plan_a[1:], a_ref[-1]))
                self.a_nom = self.plan_a[0].copy()
            else:
                self.a_nom = a_ref[0].copy()
        self._row.update(mid_qp_time=sol.solve_time, mid_qp_slack=slack, mid_qp_fallback=not sol.success,
                         mid_qp_overrun=overrun)

    # ------------------------------------------------------------------ feedback
    def _feedback(self, pos, vel, yaw):
        if self.qp is None or self.t_nom >= self.ref.t_end:
            self.x_nom, self.a_nom = self.ref.state(self.t_nom), self.ref.accel(self.t_nom)
        x_meas = np.r_[pos, vel]
        e = x_meas - self.x_nom
        a_cmd = lqr_acceleration(x_meas, self.x_nom, self.a_nom, self.K)
        self.v_cmd, saturated = velocity_command(vel, a_cmd, yaw, self.cfg, on_ramp=self.plan.geometry.slope_at(pos[0]) > 0.0)
        yaw_ref = self.ref.yaw(self.t_nom)
        self.w_cmd = yaw_rate_command(yaw_ref, self.ref.yaw_rate(self.t_nom), yaw, self.cfg)

        e_norm = self.tube.error_norm(e)
        violation = e_norm > self.tube.alpha + 1e-9
        self.counters["tube_violations"] += int(violation)
        self.counters["saturated"] += int(saturated)
        self.counters["updates"] += 1
        speed = np.linalg.norm(self.x_nom[2:])
        if speed > 0.05:
            left = np.array([-self.x_nom[3], self.x_nom[2]]) / speed
            self.cross_track_error = float(e[:2] @ left)
        else:
            self.cross_track_error = float(np.linalg.norm(e[:2]))
        self._row.update(mid_x_nom=self.x_nom.copy(), mid_a_nom=self.a_nom.copy(), mid_a_cmd=a_cmd.copy(),
                         mid_v_cmd=self.v_cmd.copy(), mid_w_cmd=self.w_cmd, mid_yaw_ref=yaw_ref,
                         mid_tube_alpha=self.tube.alpha, mid_tube_radius=self.tube.radius(self.tube.alpha),
                         mid_e_pnorm=e_norm, mid_pos_error=float(np.linalg.norm(e[:2])),
                         mid_tube_violation=violation, mid_cmd_saturated=saturated)

        self.x_nom = self.A_fb @ self.x_nom + self.B_fb @ self.a_nom
        self.tube.step()
        self.fb_count += 1
        self.t_nom = self.fb_count * self.cfg.dt_fb

    def summary(self):
        times = np.asarray(self.qp_times) * 1e3
        out = dict(self.counters, max_slack_m=self.max_slack)
        if times.size:
            out.update(qp_ms_median=float(np.median(times)), qp_ms_p99=float(np.percentile(times, 99)),
                       qp_ms_max=float(times.max()))
        return out
