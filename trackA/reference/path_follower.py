"""Reference conversion layer (step 1.1): waypoint path -> velocity and yaw-rate commands for Quadruped-PyMPC.

Sits between the global path (hand-placed waypoints now, the collocation planner's path later) and the MPC.
Every control tick it reads the robot's planar pose and returns the same thing `env.target_base_vel()` returns,
so its output goes straight into `QuadrupedPyMPC_Wrapper.compute_actions(...)`:

    ref_base_lin_vel = [v_x, v_y, 0]      world frame [m/s]
    ref_base_ang_vel = [0, 0, yaw_rate]   world frame [rad/s]

Control law (per segment p1 -> p2 of the waypoint polyline):

    p_hat    = (p2 - p1) / ||p2 - p1||           unit vector along the segment
    d        = p - p1                             robot position relative to the segment start
    s        = d . p_hat                          progress along the segment
    delta    = d - s * p_hat                      perpendicular offset, pointing from the path to the robot
    v        = p_hat * v_ref  -  clip(k_p * delta, v_corr_max)
    yaw_rate = clip(k_psi * wrap(psi_path - yaw), w_max),   psi_path = atan2(p_hat_y, p_hat_x)

plus segment switching (s >= segment length), slowdown near the goal, a stop inside goal_tol, and an
acceleration limit on the commanded velocity. See `update` for the step list.

The MPC does not track x-y position (its position reference only sets height), so the correction term here is
the only thing in the stack that pulls the robot back onto the path.
"""
import numpy as np


def wrap_to_pi(angle):
    """Wrap an angle (or array of angles) to [-pi, pi)."""
    return (np.asarray(angle) + np.pi) % (2 * np.pi) - np.pi


class PathFollower:
    def __init__(
        self,
        waypoints,
        v_ref: float = 0.4,        # [m/s]   cruise speed along the path
        k_p: float = 1.0,          # [1/s]   gain on the perpendicular offset delta
        v_corr_max: float = 0.2,   # [m/s]   cap on the back-to-path correction ||k_p * delta||
        k_psi: float = 1.5,        # [1/s]   yaw gain
        w_max: float = 0.6,        # [rad/s] yaw-rate limit
        goal_tol: float = 0.1,     # [m]     done when within this distance of the last waypoint
        slow_radius: float = 0.5,  # [m]     start slowing down this far from the goal
        a_max: float = 0.5,        # [m/s^2] limit on how fast the commanded velocity may change
    ):
        """
        Args:
            waypoints: (N, 2) array-like of world-frame (x, y) points, N >= 2. The first point is normally the
                robot's start and the last is the goal. Consecutive duplicates are dropped. Later, the planner's
                path plugs in as `result["x"][:, :2]`.
        """
        wp = np.asarray(waypoints, dtype=float)
        if wp.ndim != 2 or wp.shape[1] != 2:
            raise ValueError(f"waypoints must have shape (N, 2), got {wp.shape}")
        keep = np.ones(len(wp), dtype=bool)
        keep[1:] = np.linalg.norm(np.diff(wp, axis=0), axis=1) > 1e-9
        wp = wp[keep]
        if len(wp) < 2:
            raise ValueError("need at least two distinct waypoints")

        # Path geometry, precomputed once. Segment i runs from waypoints[i] to waypoints[i + 1].
        self.waypoints = wp
        self.n_segments = len(wp) - 1
        self.seg_vec = wp[1:] - wp[:-1]                               # (M, 2)
        self.seg_len = np.linalg.norm(self.seg_vec, axis=1)           # (M,)
        self.seg_dir = self.seg_vec / self.seg_len[:, None]           # (M, 2) unit vectors p_hat
        self.seg_heading = np.arctan2(self.seg_dir[:, 1], self.seg_dir[:, 0])  # (M,) psi_path per segment
        self.goal = wp[-1].copy()

        # Gains and limits
        self.v_ref = v_ref
        self.k_p = k_p
        self.v_corr_max = v_corr_max
        self.k_psi = k_psi
        self.w_max = w_max
        self.goal_tol = goal_tol
        self.slow_radius = slow_radius
        self.a_max = a_max

        self.reset()

    def reset(self):
        """Back to the first segment, at rest. Call between runs."""
        self.seg_idx = 0                       # active segment; only ever moves forward
        self.done = False                      # latched True once the goal is reached
        self.prev_lin_vel = np.zeros(3)        # last commanded velocity, for the acceleration limit
        self.cross_track_error = 0.0           # signed ||delta|| from the last update (+ = left of path), for logging

    def update(self, pos_xy, yaw: float, dt: float) -> tuple[np.ndarray, np.ndarray, bool]:
        """One control tick.

        Args:
            pos_xy: (2,) robot base position (x, y), world frame [m]. From `env.base_pos[:2]`.
            yaw:    robot yaw, world frame [rad]. From `env.base_ori_euler_xyz[2]`.
            dt:     time since the last call [s], used by the acceleration limit.

        Returns:
            ref_lin_vel: (3,) [v_x, v_y, 0] world frame [m/s]
            ref_ang_vel: (3,) [0, 0, yaw_rate] world frame [rad/s]
            done:        True once the robot is within goal_tol of the goal (then both commands are zero)

        Steps:
            1. If already done, return zeros and True.
            2. If within goal_tol of self.goal: set self.done, return zeros and True.
            3. Segment switching: while not on the last segment and the progress s along the active segment
               is >= its length, advance self.seg_idx.
            4. On the active segment: p_hat = self.seg_dir[i], d = p - p1, s = d . p_hat, delta = d - s * p_hat.
               Store the signed offset in self.cross_track_error (for logging): +||delta|| when the robot is
               left of the path direction (p_hat_x * d_y - p_hat_y * d_x > 0), -||delta|| when right.
            5. Speed: v_ref, reduced near the goal, e.g. v_ref * min(1, dist_to_goal / slow_radius).
            6. v_xy = p_hat * speed - (k_p * delta clipped to norm v_corr_max).
            7. Yaw: yaw_rate = clip(k_psi * wrap_to_pi(self.seg_heading[i] - yaw), -w_max, w_max).
            8. Acceleration limit: move from self.prev_lin_vel toward [v_x, v_y, 0] by at most a_max * dt;
               store the result in self.prev_lin_vel.
            9. Return (lin_vel (3,), [0, 0, yaw_rate], False).
        """
        p = np.asarray(pos_xy, dtype=float)[:2]
        dist_to_goal = np.linalg.norm(self.goal - p)

        # 1-2. Goal reached (latched): stop and stay stopped
        if not self.done and dist_to_goal <= self.goal_tol:
            self.done = True
        if self.done:
            self.prev_lin_vel = np.zeros(3)
            return np.zeros(3), np.zeros(3), True

        # 3. Segment switching: advance once the robot crosses the line through the segment's end point,
        #    perpendicular to the segment (progress s >= segment length). Never moves back.
        while self.seg_idx < self.n_segments - 1:
            i = self.seg_idx
            s = (p - self.waypoints[i]) @ self.seg_dir[i]
            if s < self.seg_len[i]:
                break
            self.seg_idx += 1

        # 4. Geometry on the active segment i (from waypoint p_i to p_{i+1})
        i = self.seg_idx
        p_i = self.waypoints[i]
        p_hat = self.seg_dir[i]
        d = p - p_i                       # robot relative to the segment start
        s = d @ p_hat                     # progress along the segment
        delta = d - s * p_hat             # perpendicular offset, path -> robot
        side = p_hat[0] * d[1] - p_hat[1] * d[0]   # 2D cross product: > 0 when left of the path direction
        self.cross_track_error = float(np.copysign(np.linalg.norm(delta), side))

        # 5. Along-path speed, slowing down linearly inside slow_radius of the goal
        speed = self.v_ref * min(1.0, dist_to_goal / self.slow_radius)

        # 6. Velocity: along the path, minus the capped correction back toward it
        correction = self.k_p * delta
        corr_norm = np.linalg.norm(correction)
        if corr_norm > self.v_corr_max:
            correction *= self.v_corr_max / corr_norm
        target = np.array([*(p_hat * speed - correction), 0.0])

        # 7. Yaw: P control toward the segment heading, wrapped and saturated
        yaw_err = wrap_to_pi(self.seg_heading[i] - yaw)
        yaw_rate = float(np.clip(self.k_psi * yaw_err, -self.w_max, self.w_max))

        # 8. Acceleration limit: the commanded velocity moves toward the target by at most a_max * dt per tick
        step = target - self.prev_lin_vel
        step_norm = np.linalg.norm(step)
        max_step = self.a_max * dt
        if step_norm > max_step:
            step *= max_step / step_norm
        self.prev_lin_vel = self.prev_lin_vel + step

        # 9. Outputs in the format of env.target_base_vel(): world-frame (3,) vectors
        return self.prev_lin_vel.copy(), np.array([0.0, 0.0, yaw_rate]), False
