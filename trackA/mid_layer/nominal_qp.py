"""The mid layer's QP: plan the robot's next 2 s around the global reference.

Variables  z = [x_1 .. x_N, a_0 .. a_{N-1}, s_t,1..N, s_b,1..N, s_v,1..N], with x = [p_x, p_y, v_x, v_y].

Two start modes (cfg.qp_start):
  measured  starts at x_meas. Acceleration limits are hard. Obstacles, walls and speed are soft, so a = 0 with enough
            slack is always a solution. The tube margin (s_t) is given up before the body margin (s_b). A terminal cost
            pulls the plan back to the reference.
  carried   starts at the carried nominal, with hard speed limits, a terminal equality and one obstacle slack (s_b).

Staying outside an obstacle is an OR across its faces. Before each solve one face is chosen per circle, obstacle and
stage interval, from a predicted trajectory, and imposed at both ends of the interval.
"""
import time
from dataclasses import dataclass

import casadi as ca
import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.optimize import least_squares

from planning.planner_helpers import polygon_halfspaces
from planning.robust_control.tube_error_model import design_tube_error_model

N_DIRS = 16                     # polygon approximation of the speed and acceleration circles
CARRIED_SLACK_LINEAR = 1e4
CARRIED_SLACK_QUADRATIC = 1e2
ACTIVATION = 2.0                # [m] obstacles farther than this from the prediction are left out
MIN_TURN_SPEED = 0.05           # [m/s] below this the path normal is undefined
INF = np.inf


def weights_matching_gain(dt, target_gain):
    """Position and velocity weights (with R = 1) whose LQR gain at step dt equals target_gain = [k_p, k_v] per axis.

    A measured-start MPC acts as feedback at its own step, so with no active constraints this makes it as stiff as the
    50 Hz LQR. With the LQR's own weights, the 0.1 s gain is 27% weaker in position.
    """
    A, B = np.array([[1.0, dt], [0.0, 1.0]]), np.array([[0.5 * dt * dt], [dt]])

    def gain(log_q):
        P = solve_discrete_are(A, B, np.diag(np.exp(log_q)), np.eye(1))
        return -np.linalg.solve(np.eye(1) + B.T @ P @ B, B.T @ P @ A).ravel()

    fit = least_squares(lambda w: gain(w) - target_gain, np.log([400.0, 25.0]), xtol=1e-12, ftol=1e-12, gtol=1e-12)
    if np.abs(fit.fun).max() > 1e-6:
        raise ValueError(f"no QP weights match the LQR gain {target_gain}")
    return np.exp(fit.x)


@dataclass
class QPSolution:
    x: np.ndarray               # (N+1, 4)
    a: np.ndarray               # (N, 2)
    slack: np.ndarray           # (N,) total obstacle and wall slack, s_t + s_b
    success: bool
    status: str
    solve_time: float
    slack_tube: np.ndarray = None   # (N,)
    slack_body: np.ndarray = None   # (N,)
    slack_speed: np.ndarray = None  # (N,)


class NominalQP:
    def __init__(self, cfg, bounds, obstacles, offsets=(0.0,), slope_at=None):
        self.cfg, self.N, self.dt = cfg, cfg.horizon, cfg.dt_qp
        self.measured = cfg.qp_start == "measured"
        self.bounds = bounds
        self.offsets = np.asarray(offsets, dtype=float)
        self.heading_margins = np.abs(self.offsets) * cfg.eps_yaw
        self.slope_at = slope_at if slope_at is not None else (lambda x: 0.0)
        self.faces = [polygon_halfspaces(np.asarray(o, float), 0.0) for o in obstacles]

        I, dt = np.eye(2), self.dt
        self.A = np.block([[I, dt * I], [np.zeros((2, 2)), I]])
        self.B = np.vstack((0.5 * dt * dt * I, dt * I))
        self.Q = np.array([400.0, 400.0, 25.0, 25.0])     # same scales as the LQR design
        self.R = np.array([1.0, 1.0])
        if self.measured:
            K_fb = np.asarray(design_tube_error_model(cfg.dt_fb)["K"])
            q_p, q_v = weights_matching_gain(dt, K_fb[0, [0, 2]])
            self.Q = np.array([q_p, q_p, q_v, q_v])
        self.P_N = solve_discrete_are(self.A, self.B, np.diag(self.Q), np.diag(self.R))
        self.P_N = 0.5 * (self.P_N + self.P_N.T)
        self.K_qp = -np.linalg.solve(np.diag(self.R) + self.B.T @ self.P_N @ self.B, self.B.T @ self.P_N @ self.A)

        N = self.N
        self.nz = 4 * N + 2 * N + 3 * N
        self._build_constraints()

        hessian = np.zeros((self.nz, self.nz))
        for k in range(1, N):
            i = self._ix(k)
            hessian[range(i, i + 4), range(i, i + 4)] = 2.0 * self.Q
        if self.measured:
            i = self._ix(N)
            hessian[i:i + 4, i:i + 4] = 2.0 * self.P_N
        for k in range(N):
            i = self._iu(k)
            hessian[range(i, i + 2), range(i, i + 2)] = 2.0 * self.R
        q_t, q_b, q_v = self._quadratic_weights()
        for k in range(1, N + 1):
            hessian[self._ist(k), self._ist(k)] = 2.0 * q_t
            hessian[self._isb(k), self._isb(k)] = 2.0 * q_b
            hessian[self._isv(k), self._isv(k)] = 2.0 * q_v
        self.H = ca.sparsify(ca.DM(hessian))
        self.H_sparsity = self.H.sparsity()
        osqp = {"verbose": False, "eps_abs": 1e-5, "eps_rel": 1e-5, "max_iter": 30000, "polish": True}
        if self.measured:       # OSQP's stopping test scales with the large linear penalties, so it must be tighter
            osqp.update(eps_rel=1e-6, rho=0.01)
        self.solver = ca.conic("nominal_qp", "osqp", {"h": self.H_sparsity, "a": self.A_sparsity}, {
            "warm_start_primal": True, "warm_start_dual": True, "error_on_fail": False, "osqp": osqp,
        })
        self._warm = None

    def _quadratic_weights(self):
        if self.measured:
            return self.cfg.q_tube, self.cfg.q_body, self.cfg.q_speed
        return CARRIED_SLACK_QUADRATIC, CARRIED_SLACK_QUADRATIC, CARRIED_SLACK_QUADRATIC

    def _linear_weights(self):
        if self.measured:
            return self.cfg.l_tube, self.cfg.l_body, self.cfg.l_speed
        return CARRIED_SLACK_LINEAR, CARRIED_SLACK_LINEAR, CARRIED_SLACK_LINEAR

    # ------------------------------------------------------------------ indices
    def _ix(self, k):           # state k, k = 1..N
        return 4 * (k - 1)

    def _iu(self, k):           # acceleration k, k = 0..N-1
        return 4 * self.N + 2 * k

    def _ist(self, k):          # tube slack k, k = 1..N
        return 6 * self.N + k - 1

    def _isb(self, k):          # body slack k, k = 1..N
        return 7 * self.N + k - 1

    def _isv(self, k):          # speed slack k, k = 1..N
        return 8 * self.N + k - 1

    # ------------------------------------------------------------------ constraint matrix (fixed sparsity)
    def _build_constraints(self):
        N = self.N
        rows, cols, vals = [], [], []

        def add(r, c, v):
            rows.append(r), cols.append(c), vals.append(float(v))
            return len(vals) - 1

        row = 0
        self.r_dyn = row                                        # x_{k+1} - A x_k - B a_k = b
        for k in range(N):
            for i in range(4):
                add(row + i, self._ix(k + 1) + i, 1.0)
                if k > 0:
                    for j in range(4):
                        if self.A[i, j] != 0.0:
                            add(row + i, self._ix(k) + j, -self.A[i, j])
                for j in range(2):
                    if self.B[i, j] != 0.0:
                        add(row + i, self._iu(k) + j, -self.B[i, j])
            row += 4

        angles = np.arange(N_DIRS) * 2 * np.pi / N_DIRS
        self.r_acc = row
        for k in range(N):
            for th in angles:
                add(row, self._iu(k), np.cos(th)), add(row, self._iu(k) + 1, np.sin(th))
                row += 1
        self.r_speed = row                                      # d . v_k - s_v,k <= v_max
        for k in range(1, N + 1):
            for th in angles:
                add(row, self._ix(k) + 2, np.cos(th)), add(row, self._ix(k) + 3, np.sin(th))
                add(row, self._isv(k), -1.0)
                row += 1

        self.r_fric = row                                       # d . a_k <= mu_lo g cos(theta) - d . [g sin(theta), 0]
        if self.measured:                                       # friction on a slope: uphill and sideways share it
            for k in range(N):
                for th in angles:
                    add(row, self._iu(k), np.cos(th)), add(row, self._iu(k) + 1, np.sin(th))
                    row += 1

        self.r_turn = row                                       # n_k . a_k
        self.turn_idx = []
        for k in range(N):
            self.turn_idx.append((add(row, self._iu(k), 0.0), add(row, self._iu(k) + 1, 0.0)))
            row += 1

        self.r_wall = row                                       # p_x + s, p_x - s, p_y + s, p_y - s
        for k in range(1, N + 1):
            for axis in (0, 1):
                for sign in (1.0, -1.0):
                    add(row, self._ix(k) + axis, 1.0)
                    add(row, self._ist(k), sign), add(row, self._isb(k), sign)
                    row += 1

        self.r_obs = row                                        # n . p_k + s_t,k + s_b,k >= rhs
        self.obs_rows = []                                      # (interval, stage, obstacle, circle, row, ix, iy)
        for interval in range(N):
            for stage in (interval, interval + 1):
                if stage == 0:
                    continue
                for j in range(len(self.faces)):
                    for m in range(len(self.offsets)):
                        ix = add(row, self._ix(stage), 0.0)
                        iy = add(row, self._ix(stage) + 1, 0.0)
                        add(row, self._ist(stage), 1.0), add(row, self._isb(stage), 1.0)
                        self.obs_rows.append((interval, stage, j, m, row, ix, iy))
                        row += 1
        self.nc = row

        rows, cols = np.asarray(rows), np.asarray(cols)
        self.A_vals = np.asarray(vals)
        self.A_order = np.lexsort((rows, cols))
        colind = np.searchsorted(cols[self.A_order], np.arange(self.nz + 1))
        self.A_sparsity = ca.Sparsity(self.nc, self.nz, colind.tolist(), rows[self.A_order].tolist())

    # ------------------------------------------------------------------ prediction used to choose obstacle faces
    def rollout(self, x0, x_ref, a_ref) -> np.ndarray:
        """x0 rolled forward with LQR tracking the reference, within the acceleration limit: (N+1, 4)."""
        x = np.empty((self.N + 1, 4))
        x[0] = x0
        for k in range(self.N):
            a = a_ref[k] + self.K_qp @ (x[k] - x_ref[k])
            size = np.linalg.norm(a)
            if size > self.cfg.accel_limit:
                a *= self.cfg.accel_limit / size
            x[k + 1] = self.A @ x[k] + self.B @ a
        return x

    # ------------------------------------------------------------------ one solve
    def solve(self, x0, x_ref, a_ref, yaw_ref, circle_radius, tube_radii, prev_xy=None, hard=False) -> QPSolution:
        """x_ref (N+1, 4), a_ref (N, 2), yaw_ref (N+1,), tube_radii (N+1,). prev_xy (N+1, 2): prediction for faces.

        hard=True removes every slack, which can make the problem infeasible. It exists to check the soft problem.
        """
        cfg, N = self.cfg, self.N
        x0, x_ref, a_ref = np.asarray(x0, float), np.asarray(x_ref, float), np.asarray(a_ref, float)
        if prev_xy is None:
            prev_xy = self.rollout(x0, x_ref, a_ref)[:, :2] if self.measured else x_ref[:, :2]
        else:
            prev_xy = np.asarray(prev_xy, float)

        g = np.zeros(self.nz)
        for k in range(1, N):
            g[self._ix(k):self._ix(k) + 4] = -2.0 * self.Q * x_ref[k]
        for k in range(N):
            g[self._iu(k):self._iu(k) + 2] = -2.0 * self.R * a_ref[k]
        l_t, l_b, l_v = self._linear_weights()
        g[self._ist(1):self._ist(1) + N] = l_t
        g[self._isb(1):self._isb(1) + N] = l_b
        g[self._isv(1):self._isv(1) + N] = l_v

        lbx, ubx = -INF * np.ones(self.nz), INF * np.ones(self.nz)
        lbx[self._ist(1):] = 0.0
        if hard:
            ubx[self._ist(1):] = 0.0
        elif self.measured:
            ubx[self._ist(1):self._ist(1) + N] = tube_radii[1:]
        else:
            ubx[self._ist(1):self._ist(1) + N] = 0.0            # one slack only
            ubx[self._isv(1):self._isv(1) + N] = 0.0            # hard speed limit

        if self.measured:
            g[self._ix(N):self._ix(N) + 4] = -2.0 * self.P_N @ x_ref[N]
        else:
            lbx[self._ix(N):self._ix(N) + 4] = x_ref[N]         # end on the global plan
            ubx[self._ix(N):self._ix(N) + 4] = x_ref[N]
        slope_theta = np.zeros(N)
        for k in range(N):                                      # uphill limit (the course climbs in +x)
            theta = self.slope_at(float(x_ref[k, 0]))
            if self.measured:                                   # the robot may be away from the reference
                theta = max(theta, self.slope_at(float(prev_xy[k, 0])))
            if theta > 0.0:
                ubx[self._iu(k)] = cfg.uphill_limit(theta)
                slope_theta[k] = theta

        lba, uba = -INF * np.ones(self.nc), INF * np.ones(self.nc)
        if self.measured:
            # The uphill and turn limits each assume the other direction uses no friction. On a slope this polygon
            # couples them: ||(a_x + g sin(theta), a_y)|| <= mu_lo g cos(theta). Its +x face is the uphill limit.
            d_x = np.cos(np.arange(N_DIRS) * 2 * np.pi / N_DIRS)
            for k in np.flatnonzero(slope_theta > 0.0):
                grav, th = cfg.gravity, slope_theta[k]
                r = self.r_fric + N_DIRS * k
                uba[r:r + N_DIRS] = cfg.mu_lo * grav * np.cos(th) - d_x * grav * np.sin(th)
        lba[self.r_dyn:self.r_dyn + 4 * N] = 0.0
        uba[self.r_dyn:self.r_dyn + 4 * N] = 0.0
        b0 = self.A @ x0
        lba[self.r_dyn:self.r_dyn + 4], uba[self.r_dyn:self.r_dyn + 4] = b0, b0
        uba[self.r_acc:self.r_acc + N_DIRS * N] = cfg.accel_limit
        uba[self.r_speed:self.r_speed + N_DIRS * N] = cfg.v_max

        vals = self.A_vals.copy()
        for k in range(N):                                      # turn limit across the reference velocity
            v = x_ref[k, 2:]
            speed = np.linalg.norm(v)
            ix, iy = self.turn_idx[k]
            if speed > MIN_TURN_SPEED:
                vals[ix], vals[iy] = -v[1] / speed, v[0] / speed
                lba[self.r_turn + k], uba[self.r_turn + k] = -cfg.turn_limit, cfg.turn_limit

        heading = np.column_stack((np.cos(yaw_ref), np.sin(yaw_ref)))
        x_lo, x_hi, y_lo, y_hi = self.bounds
        for k in range(1, N + 1):                               # walls, tightest over the circles
            R = circle_radius + self.heading_margins + tube_radii[k]
            cx, cy = self.offsets * heading[k, 0], self.offsets * heading[k, 1]
            r = self.r_wall + 4 * (k - 1)
            lba[r], uba[r + 1] = x_lo + np.max(R - cx), x_hi - np.max(R + cx)
            lba[r + 2], uba[r + 3] = y_lo + np.max(R - cy), y_hi - np.max(R + cy)

        chosen = {}
        for interval, stage, j, m, row, ix, iy in self.obs_rows:   # obstacle faces
            key = (interval, j, m)
            if key not in chosen:
                chosen[key] = self._choose_face(interval, j, m, prev_xy, heading, circle_radius, tube_radii)
            face = chosen[key]
            if face is None:
                continue
            normals, offsets = self.faces[j]
            n = normals[face]
            vals[ix], vals[iy] = n[0], n[1]
            lba[row] = offsets[face] + circle_radius + self.heading_margins[m] + tube_radii[stage] \
                - self.offsets[m] * (n @ heading[stage])

        args = dict(h=self.H, g=g, a=ca.DM(self.A_sparsity, ca.DM(vals[self.A_order])),
                    lbx=lbx, ubx=ubx, lba=lba, uba=uba)
        if self._warm is not None:
            args.update(x0=self._warm[0], lam_x0=self._warm[1], lam_a0=self._warm[2])
        t0 = time.perf_counter()
        out = self.solver(**args)
        elapsed = time.perf_counter() - t0
        stats = self.solver.stats()
        success = bool(stats.get("success", False))
        z = np.asarray(out["x"]).ravel()
        if success and np.all(np.isfinite(z)):
            self._warm = (out["x"], out["lam_x"], out["lam_a"])
        else:
            success, self._warm = False, None
        x = np.vstack([x0] + [z[self._ix(k):self._ix(k) + 4] for k in range(1, N + 1)])
        a = np.array([z[self._iu(k):self._iu(k) + 2] for k in range(N)])
        s_t = np.maximum(z[self._ist(1):self._ist(1) + N], 0.0)
        s_b = np.maximum(z[self._isb(1):self._isb(1) + N], 0.0)
        s_v = np.maximum(z[self._isv(1):self._isv(1) + N], 0.0)
        return QPSolution(x, a, s_t + s_b, success, str(stats.get("return_status", "")), elapsed, s_t, s_b, s_v)

    def _choose_face(self, interval, j, m, prev_xy, heading, circle_radius, tube_radii):
        """Face with the best worst-end margin over the interval, or None if the obstacle is far away."""
        normals, offsets = self.faces[j]
        s = self.offsets[m]
        R = circle_radius + self.heading_margins[m]
        margins = []
        for k in (interval, interval + 1):
            c = prev_xy[k] + s * heading[k]
            margins.append(normals @ c - offsets - R - tube_radii[k])
        score = np.minimum(margins[0], margins[1])
        face = int(np.argmax(score))
        return None if score[face] > ACTIVATION else face
