import casadi as ca
import numpy as np

from planner_helpers import (
    polygon_halfspaces,
    astar_initial_path,
    sample_path)


def add_obstacle_constraints(state, obstacles, r_robot, beta, g, lbg, ubg):
    """Helper function to add obstacle constraints to the direct collocation problem """
    for obstacle in obstacles:
                normals, offsets = polygon_halfspaces(obstacle, r_robot)

                violations = []

                for normal, b in zip(normals, offsets):
                    s = normal[0]*state[0] + normal[1]*state[1] - b
                    violations.append(s)

                s_vec = ca.vertcat(*violations)

                smooth_max = (
                    ca.log(ca.sum1(ca.exp(beta*s_vec))) / beta
                    - np.log(len(violations)) / beta)

                g.append(smooth_max)
                lbg.append([0])
                ubg.append([np.inf])


def plan_trajectory(start_state, goal_state, environment, config):
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
    beta = 20

    # create inflation radius
    length = config['robot_length']
    width = config['robot_width']
    r_safety = config['r_safety']
    r_robot = np.sqrt((length/2)**2 + (width/2)**2) + r_safety

    # extract bounds and speed limits
    x_min, x_max, y_min, y_max  = environment.bounds
    x_min += r_robot
    x_max -= r_robot
    y_min += r_robot
    y_max -= r_robot
    max_speed = config['max_speed']

    # get A* path for good initial guesses
    resolution = config["astar_resolution"]
    clearance = config["astar_clearance"]

    astar_path = astar_initial_path(start_state=start_state,
                                    goal_state=goal_state,
                                    bounds=(x_min, x_max, y_min, y_max),
                                    obstacles=obstacles,
                                    inflation_radius=r_robot,
                                    resolution=resolution,
                                    extra_clearance=clearance)


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

        # state at collocation points
        Xc = []
        # points between collocation points
        for j in range(d):
            Xkj = ca.MX.sym('X_' + str(k) + '_' + str(j), 4) # Xkj = [pxkj pykj, vxkj, vykj]
            Xc.append(Xkj)
            z.append(Xkj)
            lbz.append([x_min, y_min, -np.inf, -np.inf]) # enforce boundary constraint
            ubz.append([x_max, y_max, np.inf, np.inf])

            # add max_speed constraint
            Vx = Xkj[2]
            Vy = Xkj[3]
            velo_expr = Vx**2 + Vy**2
            g.append(velo_expr)
            lbg.append([0])
            ubg.append([max_speed**2])

            # add constraints for obstacle avoidance
            add_obstacle_constraints(Xkj, obstacles, r_robot, beta, g, lbg, ubg)

            # make initial guess for optimization variable - midpoint state between collocation points Xkj
            alpha = (k + tau_root[j+1])/N  

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
        lbz.append([x_min, y_min, -np.inf, -np.inf]) # enforce boundary constraint
        ubz.append([x_max, y_max, np.inf, np.inf])

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

        # add constraints for obstacle avoidance
        add_obstacle_constraints(Xk, obstacles, r_robot, beta, g, lbg, ubg)

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

    # create NLP solver
    prob = {'f': J, 'x': z, 'g': g}
    solver = ca.nlpsol('solver', 'ipopt', prob)

    # function to get x and u trajectories from z
    trajectories = ca.Function('trajectories', [z], [x_plot, u_plot], ['z'], ['x', 'u'])

    # solve the NLP
    sol = solver(x0 = z0, lbx = lbz, ubx = ubz, lbg = lbg, ubg = ubg)
    x_opt, u_opt = trajectories(sol['x']) # optimal trajectories
    x_opt = x_opt.full() # to numpy array
    u_opt = u_opt.full() # to numpy array


    t = np.linspace(0, T, N+1)

    stats = solver.stats()

    return {
        "t": t,
        "x": x_opt.T,
        "u": u_opt.T,
        "success": stats["success"],
        "return_status": stats["return_status"]
    }


