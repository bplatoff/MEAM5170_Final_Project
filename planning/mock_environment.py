"""Mock environments for developing the direct-collocation planner."""

from dataclasses import dataclass, field
from typing import List, Sequence
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as PolygonPatch


@dataclass
class MockEnvironment:
    x_min: float = 0.0
    x_max: float = 12.0
    y_min: float = 0.0
    y_max: float = 3.0
    friction_mu: float = 0.5
    obstacles: List[np.ndarray] = field(default_factory=list)
    name: str = "mock_environment"

    @property
    def bounds(self):
        return self.x_min, self.x_max, self.y_min, self.y_max

    def contains_point(self, point: Sequence[float], margin: float = 0.0) -> bool:
        x, y = float(point[0]), float(point[1])

        return (
            self.x_min + margin <= x <= self.x_max - margin
            and
            self.y_min + margin <= y <= self.y_max - margin
        )

    def plot(self, ax=None):
        if ax is None:
            _, ax = plt.subplots(figsize=(10, 3.5))

        ax.plot(
            [
                self.x_min,
                self.x_max,
                self.x_max,
                self.x_min,
                self.x_min,
            ],
            [
                self.y_min,
                self.y_min,
                self.y_max,
                self.y_max,
                self.y_min,
            ],
            linewidth=2,
        )

        for obstacle in self.obstacles:
            ax.add_patch(
                PolygonPatch(
                    obstacle,
                    closed=True,
                    alpha=0.35,
                    edgecolor="black",
                )
            )

        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(self.x_min - 0.5, self.x_max + 0.5)
        ax.set_ylim(self.y_min - 0.5, self.y_max + 0.5)

        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(self.name)

        ax.grid(True, alpha=0.25)

        return ax


def rectangle(x_min, x_max, y_min, y_max):
    """
    Create a counter-clockwise axis-aligned rectangular polygon.
    """

    return np.array(
        [
            [x_min, y_min],
            [x_max, y_min],
            [x_max, y_max],
            [x_min, y_max],
        ],
        dtype=float,
    )


def make_empty_environment(mu=0.50):
    return MockEnvironment(
        friction_mu=mu,
        obstacles=[],
        name="Milestone A — Empty Course",
    )


def make_single_obstacle_environment(mu=0.50):
    obstacle = rectangle(
        5.0,
        7.0,
        0.9,
        2.1,
    )

    return MockEnvironment(
        friction_mu=mu,
        obstacles=[obstacle],
        name="Milestone B — Single Blocking Obstacle",
    )


def make_route_choice_environment(mu=0.50):
    """
    Multi-obstacle environment requiring an alternating route.

    Obstacle 1 blocks the lower portion of the course.
    Obstacle 2 blocks the upper portion.
    Obstacle 3 blocks the lower portion again.

    The longitudinal spacing is intentionally large enough for the
    inflated Go2 footprint to transition between the corridors.
    """

    obstacles = [
        # lower obstacle -> robot must pass above
        rectangle(
            2.0,
            3.0,
            0.0,
            1.30,
        ),

        # upper obstacle -> robot must pass below
        rectangle(
            5.0,
            6.0,
            1.70,
            3.0,
        ),

        # lower obstacle -> robot must pass above again
        rectangle(
            8.0,
            9.0,
            0.0,
            1.30,
        ),
    ]

    return MockEnvironment(
        friction_mu=mu,
        obstacles=obstacles,
        name="Milestone C — Multi-Obstacle Route Choice",
    )


def default_start_goal():
    # State convention: [px, py, vx, vy]

    start_state = np.array(
        [0.5, 1.5, 0.0, 0.0],
        dtype=float,
    )

    goal_state = np.array(
        [11.5, 1.5, 0.0, 0.0],
        dtype=float,
    )

    return start_state, goal_state


if __name__ == "__main__":
    scenarios = [
        make_empty_environment(),
        make_single_obstacle_environment(),
        make_route_choice_environment(),
    ]

    start, goal = default_start_goal()

    for env in scenarios:
        ax = env.plot()

        ax.plot(
            start[0],
            start[1],
            "o",
            label="start",
        )

        ax.plot(
            goal[0],
            goal[1],
            "x",
            markersize=10,
            label="goal",
        )

        ax.legend()
        plt.show()