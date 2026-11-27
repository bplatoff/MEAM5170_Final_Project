"""LQR correction around the nominal, conversion to the velocity command, and the yaw loop."""
import numpy as np

from trackA.mid_layer.reference import wrap_to_pi


def lqr_acceleration(x_meas, x_nom, a_nom, K):
    """a_cmd = a_nom + K (x_meas - x_nom). K already contains the minus sign (A_cl = A + B K)."""
    return np.asarray(a_nom, dtype=float) + np.asarray(K) @ (np.asarray(x_meas, float) - np.asarray(x_nom, float))


def velocity_command(v_meas, a_cmd, yaw, cfg, on_ramp=False):
    """World-frame velocity command for the next dt_fb, and whether it was clipped.

    lag_inverse inverts v_dot = (k v_cmd - v) / tau over a held step, per body axis, because the robot responds
    differently forward and sideways, and more weakly forward on the ramp. b3 is decision B3, v_meas + dt a_cmd.
    """
    v_meas, a_cmd = np.asarray(v_meas, float), np.asarray(a_cmd, float)
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s], [s, c]])                           # body to world
    if cfg.conversion == "lag_inverse":
        v_body, a_body = rot.T @ v_meas, rot.T @ a_cmd
        k_forward = cfg.k_forward_ramp if on_ramp else cfg.k_forward
        cmd_body = np.array([(cfg.tau_eff(cfg.tau_forward) * a_body[0] + v_body[0]) / k_forward,
                             (cfg.tau_eff(cfg.tau_lateral) * a_body[1] + v_body[1]) / cfg.k_lateral])
    else:
        cmd_body = rot.T @ (v_meas + cfg.dt_fb * a_cmd)
    limits = np.array([cfg.v_cmd_max_ramp if on_ramp else cfg.v_cmd_max, cfg.v_cmd_max_lateral])
    clipped = np.clip(cmd_body, -limits, limits)
    return rot @ clipped, bool(np.any(np.abs(cmd_body) > limits))


def yaw_rate_command(yaw_ref, yaw_rate_ref, yaw_meas, cfg):
    """Feedforward plus proportional yaw correction, clipped to +-w_max."""
    w = yaw_rate_ref / cfg.k_yaw_rate + cfg.k_yaw * float(wrap_to_pi(yaw_ref - yaw_meas))
    return float(np.clip(w, -cfg.w_max, cfg.w_max))
