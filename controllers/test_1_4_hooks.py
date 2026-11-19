"""Step 1.4 check: per-foot friction and CoM walls change the MPC solution as expected (solver only, no sim).

Run from the repo root inside the pixi env:
    python -m controllers.test_1_4_hooks

Each test builds one MPC input (Go2 standing, all four feet in stance), solves, and inspects the solution.
  1. Friction per foot: FL at mu = 0.05 caps FL's force ratio at 0.05; the other feet take more sideways force.
  2. Friction per timestep: FL at 0.05 on only half the horizon caps only those stages.
  3. Wall: a wall in front of the robot (world frame, robot not at the origin) stops the predicted CoM.
  4. Slopes: surface_frame is orthonormal and matches the closed forms; the cone rows reject standing on a slope
     steeper than atan(mu); in the solver, the world-vertical cone breaks the true (surface-frame) friction limit
     when pushing uphill and the slope cone does not, for a ramp in x and one in y.
  5. Default: with no set_friction / set_walls / set_slope the solution matches upstream's nominal MPC.
"""
import time

import casadi as cs
import numpy as np

from quadruped_pympc import config as cfg

cfg.mpc_params["use_RTI"] = False  # one full SQP solve per call, so each test sees a complete solution

from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_nominal import Acados_NMPC_Nominal  # noqa: E402

from controllers.centroidal_nmpc_nominal_1_4 import PerFootFrictionWallNMPC, surface_frame  # noqa: E402

N = cfg.mpc_params["horizon"]
HEIGHT = 0.27  # [m] Go2 nominal CoM height
FEET = {"FL": (0.19, 0.14), "FR": (0.19, -0.14), "RL": (-0.19, 0.14), "RR": (-0.19, -0.14)}
TOL = 1e-3


def make_problem(p0=(0.0, 0.0), v_ref=(0.0, 0.0)):
    """State at rest at p0 with all feet down, and a reference asking for velocity v_ref."""
    p = np.array([p0[0], p0[1], HEIGHT])
    feet = {k: np.array([p[0] + dx, p[1] + dy, 0.0]) for k, (dx, dy) in FEET.items()}
    state = {
        "position": p, "linear_velocity": np.zeros(3), "orientation": np.zeros(3), "angular_velocity": np.zeros(3),
        **{f"foot_{k}": v.copy() for k, v in feet.items()},
    }
    reference = {
        "ref_position": p.copy(), "ref_linear_velocity": np.array([v_ref[0], v_ref[1], 0.0]),
        "ref_orientation": np.zeros(3), "ref_angular_velocity": np.zeros(3),
        **{f"ref_foot_{k}": v.reshape(1, 3).copy() for k, v in feet.items()},
    }
    contact_sequence = np.ones((4, N))
    return state, reference, contact_sequence


def solve(mpc, p0=(0.0, 0.0), v_ref=(0.0, 0.0), n_solves=10):
    """Solve repeatedly from the same input, return GRFs per stage, shape (N, 4, 3).

    Upstream runs one SQP iteration per call (num_qp_iterations = 1), warm-started from the previous call, so one
    call still carries the previous test's solution. Ten calls converge it (checked: changes below 1e-3 N after 10).
    """
    for _ in range(n_solves):
        state, reference, contact_sequence = make_problem(p0, v_ref)
        mpc.compute_control(state, reference, contact_sequence)
    return np.array([mpc.acados_ocp_solver.get(j, "u")[12:24].reshape(4, 3) for j in range(N)])


def ratio(grf):
    """Friction-pyramid ratio max(|f_x|, |f_y|) / f_z per stage and foot, shape (N, 4)."""
    return np.max(np.abs(grf[..., :2]), axis=-1) / np.maximum(grf[..., 2], 1e-6)


def frame(sx, sy):
    """surface_frame as three numpy 3-vectors."""
    return [np.array(v, dtype=float).ravel() for v in surface_frame(sx, sy)]


def surface_ratio(grf, slope):
    """True friction ratio max(|f.t|, |f.b|) / f.n in each foot's surface frame, shape (N, 4). slope: (4, 2)."""
    out = np.zeros(grf.shape[:2])
    for leg in range(4):
        n, t, b = frame(*slope[leg])
        f = grf[:, leg]
        out[:, leg] = np.maximum(np.abs(f @ t), np.abs(f @ b)) / np.maximum(f @ n, 1e-6)
    return out


def report(name, ok, detail):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    return ok


def main():
    mpc = PerFootFrictionWallNMPC()
    results = []

    # 1. friction per foot: strong sideways demand, FL slippery on every stage
    v_side = (0.0, 1.5)
    mpc.set_friction(0.8)
    g_high = solve(mpc, v_ref=v_side)
    mpc.set_friction([0.05, 0.8, 0.8, 0.8])
    g_low = solve(mpc, v_ref=v_side)
    side_others = lambda g: np.abs(g[:, 1:, 1]).sum()  # noqa: E731  total |f_y| of FR, RL, RR over the horizon
    results.append(report(
        "per-foot friction",
        ratio(g_high)[:, 0].max() > 0.05 + TOL and ratio(g_low)[:, 0].max() <= 0.05 + TOL
        and side_others(g_low) > side_others(g_high),
        f"FL max ratio {ratio(g_high)[:, 0].max():.3f} (mu 0.8) -> {ratio(g_low)[:, 0].max():.3f} (mu 0.05); "
        f"other feet total |f_y| {side_others(g_high):.1f} -> {side_others(g_low):.1f} N",
    ))

    # 2. friction per timestep: with mu 0.8 everywhere FL uses up to 0.8 late in the horizon.
    #    Slippery only early: early stages capped, late stages still free.  Slippery only late: late stages capped.
    early, late = slice(0, N // 2), slice(N // 2, N)
    r_free = ratio(g_high)[:, 0]
    mu = np.full((N, 4), 0.8)
    mu[early, 0] = 0.05
    mpc.set_friction(mu)
    r_early = ratio(solve(mpc, v_ref=v_side))[:, 0]
    mu = np.full((N, 4), 0.8)
    mu[late, 0] = 0.05
    mpc.set_friction(mu)
    r_late = ratio(solve(mpc, v_ref=v_side))[:, 0]
    results.append(report(
        "per-timestep friction",
        r_free[late].max() > 0.05 + TOL
        and r_early[early].max() <= 0.05 + TOL and r_early[late].max() > 0.05 + TOL
        and r_late[late].max() <= 0.05 + TOL,
        f"FL max ratio [stages 0-{N // 2 - 1} | {N // 2}-{N - 1}]: "
        f"mu 0.8 everywhere [{r_free[early].max():.3f} | {r_free[late].max():.3f}], "
        f"0.05 early [{r_early[early].max():.3f} | {r_early[late].max():.3f}], "
        f"0.05 late [{r_late[early].max():.3f} | {r_late[late].max():.3f}]",
    ))
    mpc.set_friction(None)

    # 3. wall: robot at (3, 1) world, asked to walk +x; wall halfway to where it would get
    p0, v_fwd = (3.0, 1.0), (1.0, 0.0)
    mpc.set_walls(None)
    solve(mpc, p0=p0, v_ref=v_fwd)
    x_free = mpc.predicted_com()[:, 0]
    x_wall = p0[0] + 0.5 * (x_free.max() - p0[0])
    mpc.set_walls([[1.0, 0.0, x_wall]])  # x <= x_wall
    solve(mpc, p0=p0, v_ref=v_fwd)
    x_con = mpc.predicted_com()[:, 0]
    # stage 0 is the measured state; walls act from stage 1 to N-1 (no terminal constraint)
    results.append(report(
        "CoM wall",
        x_free.max() > x_wall + TOL and x_con[1:N].max() <= x_wall + TOL,
        f"wall x <= {x_wall:.4f}; predicted max x without wall {x_free.max():.4f}, "
        f"with wall {x_con[1:N].max():.4f}",
    ))

    # a wall the robot is far from must not change anything
    mpc.set_walls([[1.0, 0.0, p0[0] + 5.0]])
    solve(mpc, p0=p0, v_ref=v_fwd)
    x_far = mpc.predicted_com()[:, 0]
    results.append(report("far wall inactive", np.allclose(x_far, x_free, atol=1e-4),
                          f"max |dx| vs no wall {np.abs(x_far - x_free).max():.2e}"))
    mpc.set_walls(None)

    # 4a. surface frame: orthonormal with n up, and the closed forms for a ramp in x and a ramp in y
    th = np.deg2rad(10.0)
    s, c = np.sin(th), np.cos(th)
    expected = {
        (0.0, 0.0): ([0, 0, 1], [1, 0, 0], [0, 1, 0]),
        (np.tan(th), 0.0): ([-s, 0, c], [c, 0, s], [0, 1, 0]),
        (0.0, np.tan(th)): ([0, -s, c], [1, 0, 0], [0, c, s]),
    }
    err = 0.0
    for (sx, sy), want in expected.items():
        err = max(err, max(np.abs(got - np.array(w)).max() for got, w in zip(frame(sx, sy), want)))
    for sx, sy in [(0.3, -0.2), (-0.5, 0.4), (1.0, 1.0)]:   # diagonal slopes: check the frame properties
        n, t, b = frame(sx, sy)
        M = np.stack([t, b, n], axis=1)
        err = max(err, np.abs(M.T @ M - np.eye(3)).max(), abs(np.linalg.det(M) - 1), abs(n @ [1, 0, sx]),
                  abs(n @ [0, 1, sy]))
    results.append(report("surface frame", err < 1e-9 and all(frame(sx, sy)[0][2] > 0 for sx, sy in expected),
                          f"max error vs closed forms / orthonormality {err:.1e}"))

    # 4b. cone rows, evaluated directly: a purely vertical force F on a 10 deg slope (tan 10deg = 0.176) is only
    #     allowed when mu >= 0.176. Uphill face = row 0 (+t) for an x slope, row 1 (+b) for a y slope.
    expr, ubu, lbu = mpc.create_friction_cone_constraints()
    m = mpc.centroidal_model
    rows = cs.Function("cone_rows", [m.inputs, m.mu_feet, m.slope_feet], [expr])
    u = np.zeros(24)
    u[12:24] = np.tile([0.0, 0.0, 30.0], 4)   # 30 N straight up on every foot
    ok, lines = True, []
    for name, slope, row in (("x", (np.tan(th), 0.0), 0), ("y", (0.0, np.tan(th)), 1)):
        for mu_val, should_hold in ((0.15, False), (0.25, True)):
            h = np.array(rows(u, mu_val * np.ones(4), np.tile(slope, 4))).ravel()
            holds = bool(np.all(h <= ubu + 1e-9) and np.all(h >= lbu - 1e-9))
            ok &= holds == should_hold and (h[row] > 0) != should_hold
            lines.append(f"{name}-slope mu {mu_val}: row {row} = {h[row]:+.2f} ({'ok' if holds else 'violated'})")
    h_flat = np.array(rows(u, 0.15 * np.ones(4), np.zeros(8))).ravel()
    ok &= bool(np.all(h_flat <= ubu + 1e-9) and np.all(h_flat >= lbu - 1e-9))   # flat ground: always fine
    results.append(report("slope cone rows", ok, "; ".join(lines)))

    # 4c. in the solver: robot asked to accelerate uphill on a 10 deg slope with mu 0.3. The world-vertical cone
    #     (no set_slope) lets the force lean further uphill than the slope allows; the slope cone keeps it inside.
    mu_val = 0.3
    mpc.set_friction(mu_val)
    for name, slope_xy, v_up in (("x", (np.tan(th), 0.0), (1.5, 0.0)), ("y", (0.0, np.tan(th)), (0.0, 1.5))):
        slope = np.tile(slope_xy, (4, 1))
        mpc.set_slope(None)
        r_flat = surface_ratio(solve(mpc, v_ref=v_up), slope).max()
        mpc.set_slope(slope)
        r_slope = surface_ratio(solve(mpc, v_ref=v_up), slope).max()
        results.append(report(
            f"slope cone in solver ({name} ramp)",
            r_flat > mu_val + TOL and r_slope <= mu_val + TOL,
            f"max true friction ratio, mu {mu_val}: world-vertical cone {r_flat:.3f}, slope cone {r_slope:.3f}",
        ))
    mpc.set_slope(None)
    mpc.set_friction(None)

    # 5. default behaviour equals upstream (same config mu on all feet, no walls, flat)
    upstream = Acados_NMPC_Nominal()
    g_up = solve(upstream, v_ref=v_side, n_solves=30)  # fully converged: ours still carries the wall tests' history
    g_ours = solve(mpc, v_ref=v_side, n_solves=30)
    results.append(report("defaults match upstream", np.allclose(g_up, g_ours, atol=1e-2),
                          f"max |dGRF| {np.abs(g_up - g_ours).max():.2e} N"))

    # solve time
    times = []
    for _ in range(50):
        state, reference, contact_sequence = make_problem(v_ref=v_side)
        t0 = time.perf_counter()
        mpc.compute_control(state, reference, contact_sequence)
        times.append(time.perf_counter() - t0)
    print(f"compute_control time: mean {1e3 * np.mean(times):.2f} ms, max {1e3 * np.max(times):.2f} ms "
          f"(budget {1e3 / cfg.mpc_params['frequency']:.1f} ms at {cfg.mpc_params['frequency']} Hz)")

    print(f"\n{sum(results)}/{len(results)} checks passed")
    raise SystemExit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
