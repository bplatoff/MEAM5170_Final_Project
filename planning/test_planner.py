"""Development harness for collocation_planner.py."""

import argparse
import numpy as np
import matplotlib.pyplot as plt

from mock_environment import (
    make_empty_environment,
    make_single_obstacle_environment,
    make_route_choice_environment,
    default_start_goal,
)


def planner_adapter(start_state, goal_state, environment, config):
    """Edit only this function if your planner uses a different interface."""
    try:
        from direct_collocation import plan_trajectory
    except ImportError as exc:
        raise ImportError(
            "Could not import collocation_planner.py. "
            "Put it in the same folder as this tester."
        ) from exc

    return plan_trajectory(
        start_state=start_state,
        goal_state=goal_state,
        environment=environment,
        config=config,
    )


def get_scenario(name, mu):
    if name == "empty":
        return make_empty_environment(mu)
    if name == "single":
        return make_single_obstacle_environment(mu)
    if name == "multi":
        return make_route_choice_environment(mu)
    raise ValueError(name)


def validate_result(result, start_state, goal_state, environment, config):
    required = {"t", "x", "u"}
    missing = required.difference(result.keys())
    assert not missing, f"Planner result missing keys: {sorted(missing)}"

    assert result["success"], (
    f"Solver failed: {result['return_status']}"
    )

    t = np.asarray(result["t"], dtype=float)
    x = np.asarray(result["x"], dtype=float)
    u = np.asarray(result["u"], dtype=float)

    max_allowed_accel = environment.friction_mu * 9.81
    accel_mag = np.linalg.norm(u, axis=1)

    assert np.max(accel_mag) <= max_allowed_accel + 1e-4, (
        "Trajectory violates friction acceleration limit"
    )

    assert t.ndim == 1
    assert x.ndim == 2 and x.shape[1] == 4, f"x shape = {x.shape}"
    assert u.ndim == 2 and u.shape[1] == 2, f"u shape = {u.shape}"
    assert len(t) == len(x)
    assert len(u) == len(x) - 1
    assert np.all(np.diff(t) > 0)

    assert np.linalg.norm(x[0] - start_state) <= 1e-3
    assert np.linalg.norm(x[-1] - goal_state) <= 2e-2

    for k, state in enumerate(x):
        assert environment.contains_point(state[:2]), (
            f"State {k} left course boundary: {state[:2]}"
        )

    # independently check returned knot states against inflated obstacles
    robot_length = config["robot_length"]
    robot_width = config["robot_width"]
    r_safety = config["r_safety"]

    r_robot = np.sqrt(
        (robot_length / 2)**2 +
        (robot_width / 2)**2
    ) + r_safety

    for obs_idx, obstacle in enumerate(environment.obstacles):

        x_min_obs = np.min(obstacle[:, 0]) - r_robot
        x_max_obs = np.max(obstacle[:, 0]) + r_robot
        y_min_obs = np.min(obstacle[:, 1]) - r_robot
        y_max_obs = np.max(obstacle[:, 1]) + r_robot

        for k, state in enumerate(x):
            px, py = state[:2]

            inside = (
                x_min_obs <= px <= x_max_obs
                and
                y_min_obs <= py <= y_max_obs
            )

            assert not inside, (
                f"State {k} intersects inflated obstacle {obs_idx}: "
                f"position = ({px:.3f}, {py:.3f})"
            )

    print("Basic planner checks passed.")
    print(f"Nodes:     {len(x)}")
    print(f"Horizon:   {t[-1] - t[0]:.3f} s")
    print(f"Max speed: {np.max(np.linalg.norm(x[:, 2:4], axis=1)):.3f} m/s")
    print(f"Max accel: {np.max(np.linalg.norm(u, axis=1)):.3f} m/s^2")


def plot_result(result, start_state, goal_state, environment):
    t = np.asarray(result["t"], dtype=float)
    x = np.asarray(result["x"], dtype=float)
    u = np.asarray(result["u"], dtype=float)

    ax = environment.plot()
    ax.plot(x[:, 0], x[:, 1], "-o", markersize=3, label="planned CoM path")
    ax.plot(start_state[0], start_state[1], "o", markersize=8, label="start")
    ax.plot(goal_state[0], goal_state[1], "x", markersize=10, label="goal")
    ax.legend()
    plt.tight_layout()

    plt.figure(figsize=(8, 4))
    plt.plot(t, x[:, 2], label="vx")
    plt.plot(t, x[:, 3], label="vy")
    plt.xlabel("time [s]")
    plt.ylabel("velocity [m/s]")
    plt.title("Planned velocity")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(8, 4))
    plt.step(t[:-1], u[:, 0], where="post", label="ax")
    plt.step(t[:-1], u[:, 1], where="post", label="ay")
    plt.xlabel("time [s]")
    plt.ylabel("acceleration [m/s^2]")
    plt.title("Planned control")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()

    plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=["empty", "single", "multi"], default="empty")
    parser.add_argument("--mu", type=float, default=0.50)
    parser.add_argument("--N", type=int, default=30)
    parser.add_argument("--T", type=float, default=12.0)
    parser.add_argument("--max-speed", type=float, default=1.0)
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--smoothness_weight", type=float, default = 1.0)
    args = parser.parse_args()

    environment = get_scenario(args.scenario, args.mu)
    start_state, goal_state = default_start_goal()

    config = {
        "N": args.N,
        "T": args.T,
        "max_speed": args.max_speed,
        "friction_mu": args.mu,
        "smoothness_weight": args.smoothness_weight,
        "robot_length": 0.7,
        "robot_width": 0.31,
        "r_safety": 0.05,
        "astar_resolution": 0.1,
        "astar_clearance": 0.01
    }

    print(f"Scenario: {environment.name}")
    print(f"Bounds:   {environment.bounds}")
    print(f"mu:       {environment.friction_mu}")
    print(f"Start:    {start_state}")
    print(f"Goal:     {goal_state}")

    result = planner_adapter(
        start_state=start_state,
        goal_state=goal_state,
        environment=environment,
        config=config,
    )

    validate_result(result, start_state, goal_state, environment, config)

    if not args.no_plot:
        plot_result(result, start_state, goal_state, environment)

        


if __name__ == "__main__":
    main()
