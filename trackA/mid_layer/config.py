"""Settings of the mid-layer tube MPC (task 2.4). See trackA/plans/mid_lvl_MPC_24.md."""
from dataclasses import dataclass

import numpy as np

from planning.robot_footprint import three_circle_parameters


@dataclass(frozen=True)
class MidLayerConfig:
    # Friction (decisions B2, B4). The tube is sized by mu_plan - mu_lo; delta is grip reserved for the LQR correction.
    mu_plan: float = 0.5
    mu_lo: float = 0.3
    delta: float = 0.05
    gravity: float = 9.81
    r0: float = 0.0                 # residual disturbance in P-norm units; 0 until task 2.5 calibrates it

    # Timing [s]. Both periods must be whole multiples of the simulation step.
    dt_fb: float = 0.02             # LQR, yaw and command update; must match the error model's dt
    dt_qp: float = 0.10             # nominal QP period and prediction step
    horizon: int = 20               # QP stages (2 s)

    # Footprint. "three_circle" is the default; "single_circle" is one circle around the whole body, for comparison.
    footprint: str = "three_circle"
    robot_length: float = 0.7
    robot_width: float = 0.31
    r_safety_three: float = 0.05    # each of the three circles: 0.194 + 0.05 = 0.244 m
    r_safety_one: float = 0.10      # the one circle: 0.383 + 0.10 = 0.483 m
    eps_yaw: float = 0.10           # [rad] yaw-tracking allowance; placeholder until task 2.5 measures it

    # Limits
    v_max: float = 0.5              # [m/s] nominal speed
    v_cmd_max: float = 0.8          # [m/s] command sent to Quadruped-PyMPC
    v_cmd_max_lateral: float = 0.4  # [m/s] sideways command
    v_ramp: float = 0.18            # [m/s] reference speed on a course with a ramp: at cones of 0.3 the robot saturates
                                    # near 0.24 m/s on the 10 deg ramp (trackA/results/step_test/ramp_speed.json)
    v_cmd_max_ramp: float = 0.45    # [m/s] commanding more than this on the ramp gains almost no speed
    w_max: float = 0.6              # [rad/s] yaw-rate command
    yaw_ref_rate: float = 0.5       # [rad/s] leaves 0.1 rad/s for yaw correction
    k_yaw: float = 1.5              # [1/s] yaw gain, same as PathFollower

    # Response of Quadruped-PyMPC to a velocity command, v_dot = (k v_cmd - v) / tau, in the body frame.
    # Measured by trackA/sim/step_test.py (trackA/results/step_test/summary.md). The lateral gain grows with speed,
    # from 0.62 at 0.1 m/s to 0.77 at 0.3 m/s; this is its median.
    tau_forward: float = 0.085
    k_forward: float = 0.885
    k_forward_ramp: float = 0.62    # on the 10 deg ramp with cones at 0.3
    tau_lateral: float = 0.10
    k_lateral: float = 0.707
    k_yaw_rate: float = 0.867
    conversion: str = "lag_inverse" # "lag_inverse" or "b3" (v_cmd = v_meas + dt * a_cmd)

    use_qp: bool = True             # False: the nominal is the planner reference and only LQR acts
    qp_start: str = "measured"      # "measured" (default, Jacob, Nov 26): each solve starts at x_meas. "carried": it
                                    # starts at the carried nominal (no contact in J9's pushed runs, but more falls)

    # Measured-start QP penalties, l s + q s^2. l_body > l_tube + 2 q_tube r_tube makes it use the tube margin first.
    l_tube: float = 1e4
    q_tube: float = 1e2
    l_body: float = 1e5
    q_body: float = 1e3
    l_speed: float = 1e3
    q_speed: float = 1e2

    goal_tol: float = 0.10          # [m]

    def __post_init__(self):
        if self.conversion not in ("lag_inverse", "b3"):
            raise ValueError("conversion must be 'lag_inverse' or 'b3'")
        if self.footprint not in ("three_circle", "single_circle"):
            raise ValueError("footprint must be 'three_circle' or 'single_circle'")
        if self.qp_start not in ("measured", "carried"):
            raise ValueError("qp_start must be 'measured' or 'carried'")
        if self.eps_yaw < 0.0:
            raise ValueError("eps_yaw must be nonnegative")
        if not 0.0 <= self.mu_lo <= self.mu_plan:
            raise ValueError("need 0 <= mu_lo <= mu_plan")
        if self.delta < 0.0 or self.delta >= self.mu_plan:
            raise ValueError("need 0 <= delta < mu_plan")
        ratio = self.dt_qp / self.dt_fb
        if abs(ratio - round(ratio)) > 1e-9 or round(ratio) < 1:
            raise ValueError("dt_qp must be a whole multiple of dt_fb")

    @property
    def qp_substeps(self) -> int:
        return int(round(self.dt_qp / self.dt_fb))

    def tau_eff(self, tau: float) -> float:
        """Time constant that makes a command held for dt_fb land exactly on v + dt_fb * a at the end of the step."""
        return self.dt_fb / (1.0 - np.exp(-self.dt_fb / tau))

    @property
    def r_safety(self) -> float:
        return self.r_safety_three if self.footprint == "three_circle" else self.r_safety_one

    @property
    def circle_offsets(self) -> np.ndarray:
        """Circle centers along the body axis [m]."""
        if self.footprint == "single_circle":
            return np.zeros(1)
        return three_circle_parameters(self.robot_length, self.robot_width)[0]

    @property
    def robot_radius(self) -> float:
        """Radius of each footprint circle, including r_safety [m]."""
        if self.footprint == "single_circle":
            return float(np.hypot(self.robot_length / 2, self.robot_width / 2) + self.r_safety)
        return three_circle_parameters(self.robot_length, self.robot_width, self.r_safety)[1]

    @property
    def heading_margin(self) -> float:
        """Largest sideways shift of a circle center from a yaw error of eps_yaw [m]."""
        return float(np.max(np.abs(self.circle_offsets)) * self.eps_yaw)

    @property
    def accel_limit(self) -> float:
        return (self.mu_plan - self.delta) * self.gravity

    @property
    def turn_limit(self) -> float:
        return (self.mu_lo - self.delta) * self.gravity

    def uphill_limit(self, theta: float) -> float:
        return (self.mu_lo * np.cos(theta) - np.sin(theta)) * self.gravity
