"""Step 1.4: per-foot, per-timestep friction and CoM half-space (wall) constraints in the acados problem.

Subclasses the upstream nominal NMPC (Quadruped-PyMPC pinned at 025eab4) without editing it, as SETUP.md asks.

What changes compared to upstream:
  * Parameter vector: two blocks are appended AFTER the upstream 29 entries, so every fixed index the upstream
    dynamics read (param[5], param[13:16], param[19:28], param[28]) stays valid:
        mu_feet     (4)            friction of FL, FR, RL, RR
        slope_feet  (8)            terrain slope (dz/dx, dz/dy) under FL, FR, RL, RR, flattened [sx_FL, sy_FL, ...]
        walls       (3 * K_WALLS)  K half-spaces [a_x, a_y, b] meaning  a_x * x + a_y * y <= b  on the CoM
    The upstream scalar mu (param[4]) is still filled but no longer used by the friction cone.
  * Friction cone: rows 0-4 use mu_feet[0] (FL), 5-9 mu_feet[1] (FR), 10-14 mu_feet[2] (RL), 15-19 mu_feet[3] (RR).
    Parameters are per shooting stage, so mu can change along the horizon.
  * Slopes: each foot's pyramid is built in the surface frame (n, t, b) of the terrain under it (surface_frame()),
    not in the world axes, so it is correct on a ramp tilted in x, y or both. Zero slope gives back the world axes,
    i.e. upstream's cone exactly. The GRFs are world-frame, so the frame does not depend on the robot's yaw.
  * Walls: K extra rows a^T p_xy - b <= 0 on stages 1..N-1 (stage 0 is the fixed measured state). They are soft
    with a large exact penalty, so an unreachable wall degrades the solution instead of failing the solve.
    An unused slot is a = 0, b = INACTIVE_B, which is always satisfied.

Frames: upstream's perform_scaling moves the CoM to the origin before every solve, so the solver works in a frame
centered on the current CoM. set_walls() takes walls in the WORLD frame; they are shifted here at solve time
(b_rel = b - a^T p0). Friction needs no shift.

Use:
    from controllers.centroidal_nmpc_nominal_1_4 import PerFootFrictionWallNMPC
    wrapper = QuadrupedPyMPC_Wrapper(...)
    mpc = PerFootFrictionWallNMPC()
    wrapper.srbd_controller_interface.controller = mpc
    mpc.set_friction([0.5, 0.5, 0.2, 0.5])            # or shape (N, 4) for per-timestep values
    mpc.set_slope([[0.176, 0.0]] * 4)                 # all feet on a 10 deg ramp rising in +x, or shape (N, 4, 2)
    mpc.set_walls([[0.0, 1.0, 1.2]])                  # y <= 1.2 (world), or shape (N, k, 3)
    # stage 2.4 will pass b_j - r_robot - alpha_t * ||a_j|| per stage through the same set_walls()
"""
import copy
import pathlib

import casadi as cs
import numpy as np
from acados_template import AcadosOcpSolver

import quadruped_pympc.config as config
from quadruped_pympc.controllers.gradient.nominal.centroidal_model_nominal import Centroidal_Model_Nominal
from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_nominal import ACADOS_INFTY, Acados_NMPC_Nominal

K_WALLS = 4                # number of wall slots in the generated solver (changing it regenerates the solver)
INACTIVE_B = 1e3           # [m] b of an unused wall slot: 0 <= 1e3 always holds
WALL_SLACK_LINEAR = 1e5    # exact (L1) penalty on wall violation, large so the wall acts hard when feasible
WALL_SLACK_QUADRATIC = 1e3
N_UPSTREAM_PARAMS = 29     # length of the parameter vector upstream builds in compute_control
MODEL_NAME = "centroidal_model_pf"  # differs from upstream's so the two generated libraries never clash
CODE_EXPORT_DIR = pathlib.Path(__file__).parent / "c_generated_code_per_foot"  # ignored by git (c_generated_code*/)


def surface_frame(sx, sy):
    """Unit normal n and tangents t, b (3x1 each) of a surface with slope sx = dz/dx, sy = dz/dy.

    n = (1, 0, sx) x (0, 1, sy), normalized: perpendicular to the two directions that lie in the surface; it points
    up, out of the ground. t is world x projected onto the surface, b = n x t. Zero slope gives the world axes.
    Works on numbers (returns casadi DM) and on casadi symbols (used inside the OCP).
    """
    n = cs.vertcat(-sx, -sy, 1) / cs.sqrt(1 + sx**2 + sy**2)
    t = cs.vertcat(1, 0, sx) / cs.sqrt(1 + sx**2)
    b = cs.cross(n, t)
    return n, t, b


class PerFootModel(Centroidal_Model_Nominal):
    """Upstream centroidal model with mu_feet, slope_feet and walls appended to the parameter vector."""

    def __init__(self) -> None:
        super().__init__()
        self.mu_feet = cs.SX.sym("mu_feet", 4, 1)
        self.slope_feet = cs.SX.sym("slope_feet", 8, 1)
        self.walls = cs.SX.sym("walls", 3 * K_WALLS, 1)

    def export_robot_model(self):
        acados_model = super().export_robot_model()  # builds self.param (upstream 29) and the dynamics
        self.param = cs.vertcat(self.param, self.mu_feet, self.slope_feet, self.walls)  # order = _extra_params
        acados_model.p = self.param
        acados_model.name = MODEL_NAME
        return acados_model


class _ParamAppendingSolver:
    """Wraps the AcadosOcpSolver so upstream code that sets 29 parameters per stage gets ours appended.

    Everything else is forwarded unchanged. Also extends stage bounds that upstream's set_stage_constraint writes
    (only used when foothold or stability constraints are on) with the wall bounds, so their sizes still match.
    """

    def __init__(self, solver: AcadosOcpSolver, owner: "PerFootFrictionWallNMPC"):
        self._solver = solver
        self._owner = owner

    def set(self, stage, field, value):
        if field == "p" and np.size(value) == N_UPSTREAM_PARAMS:
            value = np.concatenate((np.ravel(value), self._owner._extra_params(stage)))
        return self._solver.set(stage, field, value)

    def constraints_set(self, stage, field, value, *args, **kwargs):
        if field in ("uh", "lh") and stage > 0 and np.size(value) == self._owner.nh_upstream:
            wall_bound = np.zeros(K_WALLS) if field == "uh" else -ACADOS_INFTY * np.ones(K_WALLS)
            value = np.concatenate((np.ravel(value), wall_bound))
        return self._solver.constraints_set(stage, field, value, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._solver, name)


class PerFootFrictionWallNMPC(Acados_NMPC_Nominal):

    # ------------------------------------------------------------------ construction
    def __init__(self):
        """Copy of upstream Acados_NMPC_Nominal.__init__, with two changes marked CHANGED."""
        if config.mpc_params["use_DDP"]:
            raise NotImplementedError("PerFootFrictionWallNMPC adds state constraints, which upstream DDP does not support")

        # Our per-stage values. None means "use config.mpc_params['mu'] on every foot", read at every solve
        self._mu_feet = None                      # (N, 4) or None
        self._slope_feet = None                   # (N, 4, 2) or None (flat under every foot)
        self._walls = self._inactive_walls()      # (N, K_WALLS, 3), world frame

        self.horizon = config.mpc_params['horizon']
        self.dt = config.mpc_params['dt']
        self.T_horizon = self.horizon * self.dt
        self.use_RTI = config.mpc_params["use_RTI"]
        self.use_warm_start = config.mpc_params["use_warm_start"]
        self.use_riccati_feedback = config.mpc_params.get("use_riccati_feedback", False)
        self.riccati_K = None
        self.riccati_x = None
        self.use_foothold_constraints = config.mpc_params["use_foothold_constraints"]

        self.use_static_stability = config.mpc_params["use_static_stability"]
        self.use_zmp_stability = config.mpc_params["use_zmp_stability"]
        self.use_stability_constraints = self.use_static_stability or self.use_zmp_stability

        self.use_DDP = config.mpc_params["use_DDP"]

        self.verbose = config.mpc_params["verbose"]

        self.previous_status = -1
        self.previous_contact_sequence = np.zeros((4, self.horizon))
        self.optimal_next_state = np.zeros((24,))
        self.previous_optimal_GRF = np.zeros((12,))

        self.initial_base_position = np.array([0, 0, 0])

        # CHANGED: our model, with mu_feet and walls in the parameter vector
        self.centroidal_model = PerFootModel()
        acados_model = self.centroidal_model.export_robot_model()
        self.states_dim = acados_model.x.size()[0]
        self.inputs_dim = acados_model.u.size()[0]

        self.ocp = self.create_ocp_solver_description(acados_model)

        # CHANGED: our own export folder, so upstream's generated solver is never overwritten
        self.ocp.code_export_directory = str(CODE_EXPORT_DIR)

        self.acados_ocp_solver = _ParamAppendingSolver(
            AcadosOcpSolver(self.ocp, json_file=self.ocp.code_export_directory + "/centroidal_nmpc" + ".json"),
            self,
        )

        for stage in range(self.horizon + 1):
            self.acados_ocp_solver.set(stage, "x", np.zeros((self.states_dim,)))
        for stage in range(self.horizon):
            self.acados_ocp_solver.set(stage, "u", np.zeros((self.inputs_dim,)))

        if self.use_RTI:
            self.acados_ocp_solver.options_set("rti_phase", 1)
            self.acados_ocp_solver.solve()

    def reset(self):
        super().reset()  # builds a fresh AcadosOcpSolver, so wrap it again
        self.acados_ocp_solver = _ParamAppendingSolver(self.acados_ocp_solver, self)

    # ------------------------------------------------------------------ OCP changes
    def create_friction_cone_constraints(self):
        """Upstream friction pyramid, with each leg's 5 rows using that leg's own mu and the surface frame of the
        terrain under that foot (both parameters per stage).

        Upstream fixes n = (0, 0, 1), t = (1, 0, 0), b = (0, 1, 0) (world axes), which is only right on flat ground.
        Here n, t, b come from surface_frame(sx, sy); with sx = sy = 0 they are those same world axes.
        """
        mu_feet = self.centroidal_model.mu_feet
        slope_feet = self.centroidal_model.slope_feet
        f_max = config.mpc_params["grf_max"]
        f_min = config.mpc_params["grf_min"]

        # Focchi et al., "High-slope terrain locomotion for torque-controlled quadruped robots"
        Jbu = cs.SX.zeros(20, 12)  # row = one inequality, 3 columns per foot = that foot's (fx, fy, fz)
        for leg in range(4):  # FL, FR, RL, RR
            r, c = 5 * leg, 3 * leg
            mu = mu_feet[leg]
            n, t, b = surface_frame(slope_feet[2 * leg], slope_feet[2 * leg + 1])  # 3x1 columns, hence the .T
            Jbu[r + 0, c:c + 3] = (-n * mu + t).T  # (t - mu n) . f <= 0  ->  f_t <= mu f_n   (+t face; uphill on a +x ramp)
            Jbu[r + 1, c:c + 3] = (-n * mu + b).T  # f_b <= mu f_n                               (mu f_n = max friction force)
            Jbu[r + 2, c:c + 3] = (n * mu + b).T   # f_b >= -mu f_n
            Jbu[r + 3, c:c + 3] = (n * mu + t).T   # f_t >= -mu f_n
            Jbu[r + 4, c:c + 3] = n.T              # f_min <= f_n <= f_max (normal force, = f_z on flat ground)

        Jbu = Jbu @ cs.vertcat(self.centroidal_model.inputs[12:24])

        lbu = np.tile([-ACADOS_INFTY, -ACADOS_INFTY, 0, 0, f_min], 4).astype(float)
        ubu = np.tile([0, 0, ACADOS_INFTY, ACADOS_INFTY, f_max], 4).astype(float)
        return Jbu, ubu, lbu

    def create_ocp_solver_description(self, acados_model):
        ocp = super().create_ocp_solver_description(acados_model)

        # Rows already in con_h_expr (friction, then optional foothold and stability). Walls go after them
        self.nh_upstream = ocp.model.con_h_expr.shape[0]

        p_xy = self.centroidal_model.states[0:2]
        walls = self.centroidal_model.walls
        expr_walls = cs.vertcat(*[
            walls[3 * k] * p_xy[0] + walls[3 * k + 1] * p_xy[1] - walls[3 * k + 2] for k in range(K_WALLS)
        ])
        ocp.model.con_h_expr = cs.vertcat(ocp.model.con_h_expr, expr_walls)
        ocp.constraints.uh = np.concatenate((ocp.constraints.uh, np.zeros(K_WALLS)))
        ocp.constraints.lh = np.concatenate((ocp.constraints.lh, -ACADOS_INFTY * np.ones(K_WALLS)))

        # Soft walls: append to upstream's slack set (empty unless foothold or stability constraints are on)
        wall_idx = np.arange(self.nh_upstream, self.nh_upstream + K_WALLS)
        ocp.constraints.idxsh = np.concatenate((np.asarray(ocp.constraints.idxsh, dtype=int), wall_idx))
        ocp.constraints.lsh = np.concatenate((np.asarray(ocp.constraints.lsh), np.zeros(K_WALLS)))
        ocp.constraints.ush = np.concatenate((np.asarray(ocp.constraints.ush), np.zeros(K_WALLS)))
        for name, w in (("zl", WALL_SLACK_LINEAR), ("zu", WALL_SLACK_LINEAR),
                        ("Zl", WALL_SLACK_QUADRATIC), ("Zu", WALL_SLACK_QUADRATIC)):
            setattr(ocp.cost, name, np.concatenate((np.asarray(getattr(ocp.cost, name)), w * np.ones(K_WALLS))))

        # Logging arrays upstream sizes by the number of h rows
        nh = self.nh_upstream + K_WALLS
        self.upper_bound = np.array([np.zeros((nh,)) for _ in range(self.horizon)], dtype=object)
        self.lower_bound = np.array([np.zeros((nh,)) for _ in range(self.horizon)], dtype=object)

        # Default values of the appended parameters
        ocp.parameter_values = np.concatenate((
            ocp.parameter_values,
            config.mpc_params["mu"] * np.ones(4),
            np.zeros(8),                            # flat ground under every foot
            self._inactive_walls()[0].ravel(),
        ))
        return ocp

    # ------------------------------------------------------------------ public API
    def set_friction(self, mu):
        """Friction per foot (FL, FR, RL, RR) used from the next solve on.

        mu: scalar, shape (4,) (same on every stage), or shape (N, 4) (stage j of the horizon, j = 0..N-1).
        None goes back to config.mpc_params['mu'] on every foot.
        """
        if mu is None:
            self._mu_feet = None
            return
        mu = np.asarray(mu, dtype=float)
        if mu.ndim == 0:
            mu = np.full(4, float(mu))
        if mu.shape == (4,):
            mu = np.tile(mu, (self.horizon, 1))
        if mu.shape != (self.horizon, 4):
            raise ValueError(f"mu must be a scalar, shape (4,) or ({self.horizon}, 4), got {mu.shape}")
        if np.any(mu <= 0):
            raise ValueError("friction must be positive")
        self._mu_feet = mu

    def set_slope(self, slope):
        """Terrain slope (dz/dx, dz/dy) under FL, FR, RL, RR, used from the next solve on.

        slope: shape (4, 2) (same on every stage) or (N, 4, 2) (stage j of the horizon). A 10 deg ramp rising in
        +x is (tan 10deg, 0) = (0.176, 0); downhill slopes are negative. None goes back to flat ground everywhere.
        """
        if slope is None:
            self._slope_feet = None
            return
        slope = np.asarray(slope, dtype=float)
        if slope.shape == (4, 2):
            slope = np.tile(slope, (self.horizon, 1, 1))
        if slope.shape != (self.horizon, 4, 2):
            raise ValueError(f"slope must have shape (4, 2) or ({self.horizon}, 4, 2), got {slope.shape}")
        if not np.all(np.isfinite(slope)):
            raise ValueError("slope must be finite")
        self._slope_feet = slope

    def set_walls(self, walls):
        """CoM half-spaces a_x * x + a_y * y <= b in the WORLD frame, used from the next solve on.

        walls: shape (k, 3) rows [a_x, a_y, b] (same on every stage), or (N, k, 3) per stage, with k <= K_WALLS.
        Each row is normalized so a is a unit vector (then a violation is in meters). None or [] removes all walls.
        """
        out = self._inactive_walls()
        if walls is None or np.size(walls) == 0:
            self._walls = out
            return
        walls = np.asarray(walls, dtype=float)
        if walls.ndim == 2:
            walls = np.broadcast_to(walls, (self.horizon,) + walls.shape)
        if walls.ndim != 3 or walls.shape[0] != self.horizon or walls.shape[2] != 3:
            raise ValueError(f"walls must have shape (k, 3) or ({self.horizon}, k, 3), got {walls.shape}")
        k = walls.shape[1]
        if k > K_WALLS:
            raise ValueError(f"at most {K_WALLS} walls per stage (K_WALLS), got {k}")
        norm = np.linalg.norm(walls[:, :, :2], axis=2, keepdims=True)
        if np.any(norm < 1e-9):
            raise ValueError("wall normal a = (a_x, a_y) must be nonzero")
        out[:, :k, :] = walls / norm
        self._walls = out

    def predicted_com(self):
        """Predicted CoM positions of the last solve in the WORLD frame, shape (N + 1, 3)."""
        return np.array([self.acados_ocp_solver.get(j, "x")[0:3] for j in range(self.horizon + 1)]) \
            + self.initial_base_position

    # ------------------------------------------------------------------ internals
    def _inactive_walls(self):
        walls = np.zeros((config.mpc_params["horizon"], K_WALLS, 3))
        walls[:, :, 2] = INACTIVE_B
        return walls

    def _extra_params(self, stage):
        """mu_feet, slope_feet and walls (shifted to the solver's CoM-centered frame) for one stage."""
        j = min(stage, self.horizon - 1)
        mu = np.full(4, float(config.mpc_params["mu"])) if self._mu_feet is None else self._mu_feet[j]
        slope = np.zeros(8) if self._slope_feet is None else self._slope_feet[j].ravel()  # [sx_FL, sy_FL, sx_FR, ...]

        # perform_scaling already ran for this solve, so initial_base_position is the CoM the solver subtracted
        walls = copy.deepcopy(self._walls[j])
        p0 = np.asarray(self.initial_base_position, dtype=float)[0:2]
        walls[:, 2] -= walls[:, 0:2] @ p0
        return np.concatenate((mu, slope, walls.ravel()))
