import casadi as ca
import numpy as np

from planning.planner_helpers import (
    polygon_halfspaces,
    astar_initial_path,
    sample_path,
    sample_path_heading)
from planning.robot_footprint import add_three_circle_constraints
from planning.footprint_astar import astar_three_circle_path
from planning.yaw_reference import make_rate_limited_yaw_reference


class _CollocationSolverError(RuntimeError):
    """Failure while building or executing the numerical NLP solver."""


def add_obstacle_constraints(state, obstacles, r_robot, beta, g, lbg, ubg):
    """Helper function to add obstacle constraints to the direct collocation problem """
    for obstacle in obstacles:
                normals, offsets = polygon_halfspaces(obstacle, r_robot)

                violations = []

                for normal, b in zip(normals, offsets):
                    s = normal[0]*state[0] + normal[1]*state[1] - b
                    violations.append(s)

                s_vec = ca.vertcat(*violations)

                # Numerically stable conservative smooth maximum
                scaled = beta * s_vec
                shift = ca.mmax(scaled)

                smooth_max = (shift + ca.log(ca.sum1(ca.exp(scaled - shift))) - np.log(len(violations))) / beta

                g.append(smooth_max)
                lbg.append([0])
                ubg.append([np.inf])


def _solve_collocation_once(
    start_state, goal_state, environment, config, *,
    repair_times=(), repair_clearance_m=0.0, warm_start=None,
):
    # degree of interpolating polynomial
    d = 3  

    # get collocation points
    tau_root = np.append(0, ca.collocation_points(d, 'legendre')) # normalized locations inside one time interval [0, 1] - w/ d = 3 gives 3 internal collocation points plus the start point at tou = 0

    # smoothness weight
    lambda_s = config["smoothness_weight"]

    # control discretization
    N = config['N']

    # time horizon
    T = config['T']

    # duration of each section
    h = T/N 

    mu = environment.friction_mu
    gravity = 9.81


    # get obstacles
    obstacles = environment.obstacles # list of polygons
    beta = 100

    # create inflation radius
    length = config['robot_length']
    width = config['robot_width']
    r_safety = config['r_safety']
    tube_margin_m = float(config.get("tube_position_margin_m", 0.0))
    if not np.isfinite(tube_margin_m) or tube_margin_m < 0.0:
        raise ValueError("tube_position_margin_m must be finite and nonnegative")
    # The tube is a horizontal position error in meters, not a P-norm radius.
    # Inflate every footprint circle and both the A* warm start and NLP constraints.
    effective_safety_margin = r_safety + tube_margin_m
    r_robot = np.sqrt((length/2)**2 + (width/2)**2) + effective_safety_margin

    # Explicit opt-in while the yaw reference and controller interface are validated.
    footprint_model = config.get("footprint_model", "single_circle")
    if footprint_model not in ("single_circle", "three_circle"):
        raise ValueError("footprint_model must be 'single_circle' or 'three_circle'")

    yaw_mode = config.get("footprint_yaw_mode", "path_tangent")
    if footprint_model == "three_circle" and yaw_mode not in ("path_tangent", "fixed"):
        raise ValueError("footprint_yaw_mode must be 'path_tangent' or 'fixed'")
    fixed_yaw = float(config.get("footprint_yaw", 0.0))
    if not np.isfinite(fixed_yaw):
        raise ValueError("footprint_yaw must be finite")

    # Keep the existing conservative single-disc A* seed for now. This
    # can reject some narrow paths which the three-circle NLP could use.
    x_min, x_max, y_min, y_max = environment.bounds
    astar_bounds = (x_min + r_robot, x_max - r_robot,
                    y_min + r_robot, y_max - r_robot)
    # The NLP can use the full map; three-circle constraints tighten each
    # wall independently based on the supplied yaw reference.
    state_bounds = astar_bounds if footprint_model == "single_circle" else environment.bounds
    sx_min, sx_max, sy_min, sy_max = state_bounds
    max_speed = config['max_speed']

    # get A* path for good initial guesses
    resolution = config["astar_resolution"]
    clearance = config["astar_clearance"]

    if footprint_model == "three_circle":
        astar_path = astar_three_circle_path(
            start_state=start_state, goal_state=goal_state,
            bounds=environment.bounds, obstacles=obstacles,
            length=length, width=width, safety_margin=effective_safety_margin,
            resolution=resolution, extra_clearance=clearance,
            yaw_mode=yaw_mode, fixed_yaw=fixed_yaw,
        )
    else:
        astar_path = astar_initial_path(
            start_state=start_state, goal_state=goal_state,
            bounds=astar_bounds, obstacles=obstacles,
            inflation_radius=r_robot, resolution=resolution,
            extra_clearance=clearance,
        )

    # Optional bounded-rate yaw reference: same orientation is used both
    # by the NLP collision constraints and in the returned yaw_ref.
    # Without this opt-in the original A*-tangent behavior is unchanged.
    max_yaw_rate = config.get("footprint_max_yaw_rate")
    yaw_at = None
    if footprint_model == "three_circle" and yaw_mode == "path_tangent" and max_yaw_rate is not None:
        sample_fractions = [0.0]
        for k in range(N):
            sample_fractions.extend((k + float(tau))/N for tau in tau_root[1:])
            sample_fractions.append((k+1)/N)
        yaw_at, _ = make_rate_limited_yaw_reference(
            astar_path, sample_fractions, T, float(max_yaw_rate), initial_yaw=fixed_yaw
        )

    def nominal_yaw(fraction):
        if yaw_mode == "fixed":
            return fixed_yaw
        if yaw_at is not None:
            return yaw_at(fraction)
        return sample_path_heading(fraction, astar_path, fallback_yaw=fixed_yaw)

    def add_collision_constraints(state, fraction, g, lbg, ubg):
        if footprint_model == "single_circle":
            add_obstacle_constraints(state, obstacles, r_robot, beta, g, lbg, ubg)
        else:
            add_three_circle_constraints(
                state[:2], nominal_yaw(fraction), obstacles,
                environment.bounds, length, width, effective_safety_margin, beta,
                g, lbg, ubg,
            )


    # coefficients of the collocation equation - eventually use to impose x_dot = f(x, u)
    C = np.zeros((d+1, d+1))

    # coefficients of the continuity equation - eventually tells where ending point Xk,end is and enforce end = start of next
    D = np.zeros(d+1)

    # coefficients of the quadrature function - tells how much cost evaluated at each collocation  contributes to cost integral
    B = np.zeros(d+1) 

    # Construct polynomial basis - lagrange interpolation
    for j in range(d+1):
        # Construct Lagrange polynomials to get the polynomial basis at the collocation point
        p = np.poly1d([1])
        for r in range(d+1):
            if r != j:
                p *= np.poly1d([1, -tau_root[r]]) / (tau_root[j]-tau_root[r])

        # Evaluate the polynomial at the final time to get the coefficients of the continuity equation
        D[j] = p(1.0) # coefficients for continuity constraints

        # Evaluate the time derivative of the polynomial at all collocation points to get the coefficients of the continuity equation
        pder = np.polyder(p)
        for r in range(d+1):
            C[j,r] = pder(tau_root[r]) # coefficients for dynamics constraints

        # Evaluate the integral of the polynomial to get the coefficients of the quadrature function
        pint = np.polyint(p)
        B[j] = pint(1.0) # cost integration


    # declare model variables
    px = ca.SX.sym('px')
    py = ca.SX.sym('py')
    # pz = ca.SX.sym('pz')
    vx = ca.SX.sym('vx')
    vy = ca.SX.sym('vy')
    # vz = ca.SX.sym('vz')

    x = ca.vertcat(px, py, vx, vy)

    ax = ca.SX.sym('ax')
    ay = ca.SX.sym('ay')

    u = ca.vertcat(ax, ay)

    xdot = ca.vertcat(vx, vy, ax, ay)

    # objective term
    R = np.eye(2)
    L = ca.mtimes([u.T, R, u])

    # dynamics + stage cost function
    f = ca.Function("f", [x, u], [xdot, L]) # include L to follow recommended CasADi structure using the quadrature coefficients B

    # initialize empty NLP
    z = []  # decision variable vector - ex: w = [X0, U0, X01, X02, X03, X1, U1, ...] - contains knot states Xk, controls Uk, and collocation states Xkj
    z0 = [] # initial guess for solver
    lbz = []
    ubz = [] # lower and upper bound on each optimization variable
    J = 0
    g = [] # constraint expressions
    lbg = [] 
    ubg = [] # lower and upper bounds on constraint expressions

    U_prev = None

    # for plotting
    x_plot = []
    u_plot = []


    # start constructing the NLP
    # lift the initial conditions
    Xk = ca.MX.sym('X0', 4) # x is 4x1
    z.append(Xk)
    lbz.append(start_state) # start state [px, py, vx, vy]
    ubz.append(start_state) 
    z0.append(start_state) 
    x_plot.append(Xk)
    # The start state is fixed, but its complete footprint still needs a
    # collision constraint in three-circle mode.
    if footprint_model == "three_circle":
        add_collision_constraints(Xk, 0.0, g, lbg, ubg)

    # formulate NLP
    for k in range(N):
        # construct new NLP variable for the control U_k
        Uk = ca.MX.sym('U_' + str(k), 2) # U_k = [axk, ayk]

        z.append(Uk)
        # lbz.append([-4, -4]) # control limits [ax, ay] - arbitrarily tentatively set a +- 4 m/s^2 
        # ubz.append([4, 4]) 
        lbz.append([-np.inf, -np.inf])
        ubz.append([np.inf, np.inf])
        z0.append([0, 0]) # control initial guess [ax, ay]
        u_plot.append(Uk)

        # friction/acceleration constraint - ax^2 + ay^2 <_ (mu*gravity)^2
        friction_expr = ca.sumsqr(Uk) 
        g.append(friction_expr)
        lbg.append([0])
        ubg.append([(mu*gravity)**2])

        # add additional cost to rapid changes in acceleration 
        if U_prev is not None:
            J += lambda_s*ca.sumsqr(Uk - U_prev)

        U_prev = Uk

        # Adaptive continuous-clearance repair: add only the offending times.
        # Translation is exact for the constant-acceleration double integrator.
        # The yaw is the SAME fixed/rate-limited nominal yaw used elsewhere
        # in this NLP. No additional state or grid points are introduced.
        for time_s in repair_times:
            time_s = float(time_s)
            if k * h - 1e-10 <= time_s < (k+1)*h - 1e-10:
                tau = np.clip(time_s - k*h, 0.0, h)
                pos_at_t = (Xk[:2] + tau * Xk[2:4] +
                            0.5 * tau**2 * Uk)
                add_three_circle_constraints(
                    pos_at_t, nominal_yaw(time_s / T), obstacles,
                    environment.bounds, length, width,
                    effective_safety_margin + repair_clearance_m,
                    beta, g, lbg, ubg,
                )

        # state at collocation points
        Xc = []
        # points between collocation points
        for j in range(d):
            Xkj = ca.MX.sym('X_' + str(k) + '_' + str(j), 4) # Xkj = [pxkj pykj, vxkj, vykj]
            Xc.append(Xkj)
            z.append(Xkj)
            lbz.append([sx_min, sy_min, -np.inf, -np.inf]) # enforce boundary constraint
            ubz.append([sx_max, sy_max, np.inf, np.inf])

            # add max_speed constraint
            Vx = Xkj[2]
            Vy = Xkj[3]
            velo_expr = Vx**2 + Vy**2
            g.append(velo_expr)
            lbg.append([0])
            ubg.append([max_speed**2])

            # Geometry is evaluated at the nominal yaw for this time index.
            alpha = (k + tau_root[j+1])/N
            add_collision_constraints(Xkj, alpha, g, lbg, ubg)

            # make initial guess for optimization variable - midpoint state between collocation points Xkj

            position_guess = sample_path(alpha, astar_path)

            z0.append([position_guess[0], position_guess[1], 0, 0]) # make initial guess proportional to how far along the trajectory we are at



        # loop over collocation points
        Xk_end = D[0]*Xk
        for j in range(1, d+1):
            # expression for the state derivative at the collocation point
            xp = C[0, j]*Xk 
            for r in range(d):
                # xp is the derivative implied by the interpolation polynomial at collocation point j
                xp = xp + C[r+1, j]*Xc[r]

            # append collocation equations
            fj, qj = f(Xc[j-1], Uk) # calculates f(Xkj, Uk)
            g.append(h*fj - xp) # enforce dynamics constraint 
            lbg.append([0, 0, 0, 0])
            ubg.append([0, 0, 0, 0])
            

            # add contribution to state at the end of the interval
            Xk_end = Xk_end + D[j]*Xc[j-1]

            # add contribution to quadrature function
            J = J + B[j]*qj*h

        # NLP variable for state at the end of the interval
        # collocation points
        Xk = ca.MX.sym('X_' + str(k+1), 4)
        z.append(Xk)
        lbz.append([sx_min, sy_min, -np.inf, -np.inf]) # enforce boundary constraint
        ubz.append([sx_max, sy_max, np.inf, np.inf])

        # add max_speed constraint
        Vx = Xk[2]
        Vy = Xk[3]
        velo_expr = Vx**2 + Vy**2
        g.append(velo_expr)
        lbg.append([0])
        ubg.append([max_speed**2])

        # make initial guess for optimization variable - collocation state Xk
        alpha = (k+1)/N 
        
        position_guess = sample_path(alpha, astar_path)

        z0.append([position_guess[0], position_guess[1], 0, 0]) # make initial guess proportional to how far along the trajectory we are at'

        x_plot.append(Xk)

        # Apply circle and wall constraints at each knot, too.
        add_collision_constraints(Xk, (k+1)/N, g, lbg, ubg)

        # add equality constraint
        g.append(Xk_end - Xk) # enforce continuity constraints - end of one section aligns with start of next
        lbg.append([0, 0, 0, 0])
        ubg.append([0, 0, 0, 0])

    # at final goal state termination constraint
    g.append(Xk - goal_state)
    lbg.append([0, 0, 0, 0])
    ubg.append([0, 0, 0, 0])

    # concatinate vectors
    z = ca.vertcat(*z)
    g = ca.vertcat(*g)
    x_plot = ca.horzcat(*x_plot)
    u_plot = ca.horzcat(*u_plot)
    z0 = np.concatenate(z0)
    lbz = np.concatenate(lbz)
    ubz = np.concatenate(ubz)
    lbg = np.concatenate(lbg)
    ubg = np.concatenate(ubg)

    if warm_start is not None:
        initial = np.asarray(warm_start, dtype=float).reshape(-1)
        if initial.shape == z0.shape and np.all(np.isfinite(initial)):
            z0 = initial

    # Keep solver failures distinguishable from A* initialization failures.
    # CasADi/IPOPT may raise RuntimeError instead of returning a failed status.
    prob = {'f': J, 'x': z, 'g': g}
    try:
        solver = ca.nlpsol('solver', 'ipopt', prob)
        trajectories = ca.Function('trajectories', [z], [x_plot, u_plot], ['z'], ['x', 'u'])
        sol = solver(x0=z0, lbx=lbz, ubx=ubz, lbg=lbg, ubg=ubg)
        x_opt, u_opt = trajectories(sol['x'])
        x_opt = x_opt.full()
        u_opt = u_opt.full()
        stats = solver.stats()
    except RuntimeError as exc:
        raise _CollocationSolverError(str(exc)) from exc

    t = np.linspace(0, T, N+1)

    result = {
        "t": t,
        "x": x_opt.T,
        "u": u_opt.T,
        "success": stats["success"],
        "return_status": stats["return_status"],
        "_decision_variables": np.asarray(sol['x'].full()).reshape(-1),
    }
    if footprint_model == "three_circle":
        result["astar_path"] = astar_path.copy()  # geometric warm start, useful for debugging
        # Downstream controllers must track, or conservatively bound error
        # relative to, this prescribed orientation reference.
        result["yaw_ref"] = np.array([nominal_yaw(k/N) for k in range(N+1)])
    return result




def plan_trajectory(start_state, goal_state, environment, config):
    """Solve the existing NLP, optionally certifying and repairing clearance.

    New behavior is explicitly opt-in via `continuous_clearance=True`.
    Requires three_circle and a finite rate limit for path_tangent yaw.
    A failed numerical solve or exhausted certificate is NOT a proof of
    geometric infeasibility, so this function never invents that status.
    """
    enabled = bool(config.get("continuous_clearance", False))
    if not enabled:
        result = _solve_collocation_once(start_state, goal_state, environment, config)
        result.pop("_decision_variables", None)
        return result

    from planning.continuous_clearance import verify_continuous_clearance

    if config.get("footprint_model") != "three_circle":
        raise ValueError("continuous_clearance requires footprint_model='three_circle'")
    yaw_mode = config.get("footprint_yaw_mode", "path_tangent")
    limit = config.get("footprint_max_yaw_rate")
    if yaw_mode == "path_tangent" and (
        limit is None or not np.isfinite(limit) or limit <= 0
    ):
        raise ValueError("Certification requires footprint_max_yaw_rate > 0")
    if yaw_mode not in ("fixed", "path_tangent"):
        raise ValueError("Unsupported yaw mode for certification")

    max_repairs = config.get("clearance_max_repairs", 4)
    if not isinstance(max_repairs, int) or max_repairs < 0:
        raise ValueError("clearance_max_repairs must be a nonnegative integer")
    repair_padding = float(config.get("clearance_repair_padding_m", 0.002))
    if not np.isfinite(repair_padding) or repair_padding < 0:
        raise ValueError("clearance_repair_padding_m must be finite and nonnegative")
    max_depth = config.get("clearance_max_depth", 17)
    repair_times = []
    warm_start = None
    history = []

    for attempt in range(max_repairs + 1):
        try:
            result = _solve_collocation_once(
                start_state, goal_state, environment, config,
                repair_times=repair_times,
                repair_clearance_m=repair_padding, warm_start=warm_start,
            )
        except _CollocationSolverError as exc:
            return {"success": False, "planning_status": "solver_failed",
                    "return_status": str(exc), "repair_attempts": attempt,
                    "repair_history": history}
        except RuntimeError as exc:
            # A* initialization or another planning-stage runtime failure.
            # Neither outcome establishes infeasibility.
            return {"success": False, "planning_status": "planning_failed",
                    "return_status": str(exc), "repair_attempts": attempt,
                    "repair_history": history}

        warm_start = result.pop("_decision_variables")
        if not result["success"]:
            result.update(success=False, planning_status="solver_failed",
                          repair_attempts=attempt, repair_history=history)
            return result

        certificate = verify_continuous_clearance(
            result, environment, config, max_depth=max_depth,
        )
        history.append({"attempt": attempt, "certificate": certificate,
                        "constraint_times_s": list(repair_times)})
        result.update(
            certification=certificate,
            repair_attempts=attempt,
            repair_history=history,
        )
        if certificate["certified"]:
            result["planning_status"] = "success"
            return result

        if attempt == max_repairs:
            result.update(success=False, planning_status="uncertified")
            return result
        # Add constraints at newly detected collision or uncertain locations.
        # Do not repeatedly re-add the same time; leave an explicit failure
        # status if refinement makes no progress.
        new_times = []
        for finding in certificate["findings"]:
            time_s = float(finding["time_s"])
            if (np.isfinite(time_s) and
                    all(abs(time_s - t) > 1e-6 for t in repair_times + new_times)):
                new_times.append(time_s)
        if not new_times:
            result.update(success=False, planning_status="uncertified")
            return result
        repair_times = sorted(repair_times + new_times)

    raise AssertionError("Unreachable finite repair loop")
