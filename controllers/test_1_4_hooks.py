"""Step 1.4 check: per-foot friction and CoM walls change the MPC solution as expected (solver only, no sim).

Run from the repo root inside the pixi env:
    python -m controllers.test_1_4_hooks

Each test builds one MPC input (Go2 standing, all four feet in stance), solves, and inspects the solution.
  1. Friction per foot: FL at mu = 0.05 caps FL's force ratio at 0.05; the other feet take more sideways force.
  2. Friction per timestep: FL at 0.05 on only half the horizon caps only those stages.
  3. Wall: a wall in front of the robot (world frame, robot not at the origin) stops the predicted CoM.
  4. Default: with no set_friction / set_walls the solution matches upstream's nominal MPC.
"""
import time

import numpy as np

from quadruped_pympc import config as cfg

cfg.mpc_params["use_RTI"] = False  # one full SQP solve per call, so each test sees a complete solution

from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_nominal import Acados_NMPC_Nominal  # noqa: E402

from controllers.centroidal_nmpc_nominal_1_4 import PerFootFrictionWallNMPC  # noqa: E402

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

    # 4. default behaviour equals upstream (same config mu on all feet, no walls)
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
