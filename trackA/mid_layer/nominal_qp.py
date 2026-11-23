"""The mid layer's nominal QP: plan the ideal robot's next 2 s around the global reference.

Variables  z = [x_1 .. x_N, a_0 .. a_{N-1}, s_1 .. s_N], with x = [p_x, p_y, v_x, v_y].

Staying outside an obstacle is an OR across its faces. Before each solve one face is chosen per circle, obstacle and
stage interval, from the previous plan, and imposed at both ends of the interval. A half-space is convex, so that also
covers the segment between them. Slack s_k keeps the solver returning an answer; s_k > 1 mm voids the guarantee.
"""
import time
from dataclasses import dataclass

import casadi as ca
import numpy as np

from planning.planner_helpers import polygon_halfspaces

N_DIRS = 16                     # polygon approximation of the speed and acceleration circles
SLACK_LINEAR = 1e4
SLACK_QUADRATIC = 1e2
ACTIVATION = 2.0                # [m] obstacles farther than this from the plan are left out
MIN_TURN_SPEED = 0.05           # [m/s] below this the path normal is undefined
INF = np.inf


@dataclass
class QPSolution:
    x: np.ndarray               # (N+1, 4)
    a: np.ndarray               # (N, 2)
    slack: np.ndarray           # (N,)
    success: bool
    status: str
    solve_time: float


class NominalQP:
    def __init__(self, cfg, bounds, obstacles, offsets=(0.0,), slope_at=None):
        self.cfg, self.N, self.dt = cfg, cfg.horizon, cfg.dt_qp
        self.bounds = bounds
        self.offsets = np.asarray(offsets, dtype=float)
        self.slope_at = slope_at if slope_at is not None else (lambda x: 0.0)
        self.faces = [polygon_halfspaces(np.asarray(o, float), 0.0) for o in obstacles]

        I, dt = np.eye(2), self.dt
        self.A = np.block([[I, dt * I], [np.zeros((2, 2)), I]])
        self.B = np.vstack((0.5 * dt * dt * I, dt * I))
        self.Q = np.array([400.0, 400.0, 25.0, 25.0])     # same scales as the LQR design
        self.R = np.array([1.0, 1.0])

        N = self.N
        self.nz = 4 * N + 2 * N + N
        self._build_constraints()

        h = np.zeros(self.nz)
        for k in range(1, N):
            h[self._ix(k):self._ix(k) + 4] = 2.0 * self.Q
        for k in range(N):
            h[self._iu(k):self._iu(k) + 2] = 2.0 * self.R
        h[self._is(1):] = 2.0 * SLACK_QUADRATIC
        self.H_sparsity = ca.Sparsity.diag(self.nz)
        self.H = ca.DM(self.H_sparsity, ca.DM(h))
        self.solver = ca.conic("nominal_qp", "osqp", {"h": self.H_sparsity, "a": self.A_sparsity}, {
            "warm_start_primal": True, "warm_start_dual": True, "error_on_fail": False,
            "osqp": {"verbose": False, "eps_abs": 1e-5, "eps_rel": 1e-5, "max_iter": 30000, "polish": True},
        })
        self._warm = None

    # ------------------------------------------------------------------ indices
    def _ix(self, k):           # state k, k = 1..N
        return 4 * (k - 1)

    def _iu(self, k):           # acceleration k, k = 0..N-1
        return 4 * self.N + 2 * k

    def _is(self, k):           # slack k, k = 1..N
        return 6 * self.N + k - 1

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
        self.r_speed = row
        for k in range(1, N + 1):
            for th in angles:
                add(row, self._ix(k) + 2, np.cos(th)), add(row, self._ix(k) + 3, np.sin(th))
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
                    add(row, self._ix(k) + axis, 1.0), add(row, self._is(k), sign)
                    row += 1

        self.r_obs = row                                        # n . p_k + s_k >= rhs
        self.obs_rows = []                                      # (interval, stage, obstacle, circle, row, ix, iy)
        for interval in range(N):
            for stage in (interval, interval + 1):
                if stage == 0:
                    continue
                for j in range(len(self.faces)):
                    for m in range(len(self.offsets)):
                        ix = add(row, self._ix(stage), 0.0)
                        iy = add(row, self._ix(stage) + 1, 0.0)
                        add(row, self._is(stage), 1.0)
                        self.obs_rows.append((interval, stage, j, m, row, ix, iy))
                        row += 1
        self.nc = row

        rows, cols = np.asarray(rows), np.asarray(cols)
        self.A_vals = np.asarray(vals)
        self.A_order = np.lexsort((rows, cols))
        colind = np.searchsorted(cols[self.A_order], np.arange(self.nz + 1))
        self.A_sparsity = ca.Sparsity(self.nc, self.nz, colind.tolist(), rows[self.A_order].tolist())

    # ------------------------------------------------------------------ one solve
    def solve(self, x0, x_ref, a_ref, yaw_ref, circle_radius, tube_radii, prev_xy=None) -> QPSolution:
        """x_ref (N+1, 4), a_ref (N, 2), yaw_ref (N+1,), tube_radii (N+1,). prev_xy (N+1, 2): last plan, shifted."""
        cfg, N = self.cfg, self.N
        x0, x_ref, a_ref = np.asarray(x0, float), np.asarray(x_ref, float), np.asarray(a_ref, float)
        prev_xy = x_ref[:, :2] if prev_xy is None else np.asarray(prev_xy, float)

        g = np.zeros(self.nz)
        for k in range(1, N):
            g[self._ix(k):self._ix(k) + 4] = -2.0 * self.Q * x_ref[k]
        for k in range(N):
            g[self._iu(k):self._iu(k) + 2] = -2.0 * self.R * a_ref[k]
        g[self._is(1):] = SLACK_LINEAR

        lbx, ubx = -INF * np.ones(self.nz), INF * np.ones(self.nz)
        lbx[self._is(1):] = 0.0
        lbx[self._ix(N):self._ix(N) + 4] = x_ref[N]             # end on the global plan
        ubx[self._ix(N):self._ix(N) + 4] = x_ref[N]
        for k in range(N):                                      # uphill limit (the course climbs in +x)
            theta = self.slope_at(float(x_ref[k, 0]))
            if theta > 0.0:
                ubx[self._iu(k)] = cfg.uphill_limit(theta)

        lba, uba = -INF * np.ones(self.nc), INF * np.ones(self.nc)
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
            R = circle_radius + tube_radii[k]
            cx, cy = self.offsets * heading[k, 0], self.offsets * heading[k, 1]
            r = self.r_wall + 4 * (k - 1)
            lba[r], uba[r + 1] = x_lo + R - cx.min(), x_hi - R - cx.max()
            lba[r + 2], uba[r + 3] = y_lo + R - cy.min(), y_hi - R - cy.max()

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
            lba[row] = offsets[face] + circle_radius + tube_radii[stage] - self.offsets[m] * (n @ heading[stage])

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
        return QPSolution(x, a, np.maximum(z[self._is(1):], 0.0), success, str(stats.get("return_status", "")), elapsed)

    def _choose_face(self, interval, j, m, prev_xy, heading, circle_radius, tube_radii):
        """Face with the best worst-end margin over the interval, or None if the obstacle is far away."""
        normals, offsets = self.faces[j]
        s = self.offsets[m]
        margins = []
        for k in (interval, interval + 1):
            c = prev_xy[k] + s * heading[k]
            margins.append(normals @ c - offsets - circle_radius - tube_radii[k])
        score = np.minimum(margins[0], margins[1])
        face = int(np.argmax(score))
        return None if score[face] > ACTIVATION else face
