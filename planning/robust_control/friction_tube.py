import numpy as np

def friction_acceleration_bound(
    stance_mask,
    normal_forces,
    mu_plan,
    mu_lo,
    mass,
):
    """
    Calculate the friction-induced acceleration disturbance bound.

    Inputs:
        stance_mask: (N, 4) contact indicators
        normal_forces: (N, 4) surface-normal forces [N]
        mu_plan: planned friction coefficient
        mu_lo: conservative friction lower bound
        mass: robot mass [kg]

    Returns:
        delta_a: (N,) maximum acceleration disturbance [m/s²]
    """

    mu_plan = np.asarray(mu_plan)
    mu_lo = np.asarray(mu_lo)

    # input validation
    if np.ndim(mass) != 0 or mass <= 0 or not np.isfinite(np.asarray(mass)):
        raise ValueError("Mass must be finite and strictly positive")

    if stance_mask.ndim != 2 or normal_forces.ndim != 2:
        raise ValueError("stance mask and normal forces must have shape Nx4")

    if stance_mask.shape != normal_forces.shape or stance_mask.shape[1] != 4:
        raise ValueError("Stance mask and normal forces must have the same shape and be Nx4 matrices")

    if not np.all(normal_forces >= 0) or not np.all(np.isfinite(normal_forces)):
        raise ValueError("Normal Forces must only contain finite nonnegatives values")

    if not np.all((stance_mask == 0) | (stance_mask == 1)):
        raise ValueError("Stance mask must contain only zeros and ones")

    if not np.all(mu_plan >= 0) or not np.all(mu_lo >= 0):
        raise ValueError("mu_plan and mu_lo must be nonnegative")

    if not np.all(np.isfinite(mu_plan)) or not np.all(np.isfinite(mu_lo)):
        raise ValueError("mu_plan and mu_lo must be finite")

    try:
        np.broadcast_to(mu_lo, stance_mask.shape)
    except ValueError:
        raise ValueError("mu_lo must have compatible shape with stance mask")

    try:
        np.broadcast_to(mu_plan, stance_mask.shape)
    except ValueError:
        raise ValueError("mu_plan must have compatible shape with stance mask")


    delta_a = (1/mass)*np.sum((np.maximum(0, mu_plan - mu_lo)*normal_forces*stance_mask), axis = 1)


    return delta_a

def p_norm_disturbance_bound(delta_a, B, P, r0 = 0.0):
    """
    Convert acceleration disturbance bounds into P-norm bounds.

    Inputs:
        delta_a: (N,) acceleration bounds [m/s²]
        B: (4, 2) discrete acceleration input matrix
        P: (4, 4) positive-definite Lyapunov matrix
        r0: nonnegative residual uncertainty bound in P-norm units

    Returns:
        r_w: (N,) disturbance bounds in the P-norm
    """

    # input validation
    if not np.all(np.asarray(delta_a) >= 0.0) or delta_a.ndim != 1 or not np.all(np.isfinite(delta_a)):
        raise ValueError("delta_a must be 1D, finite, and nonnegative")

    if B.shape != (4,2) or not np.all(np.isfinite(B)):
        raise ValueError("B must have shape 4x2 and be finite")

    if P.shape != (4,4) or not np.all(np.isfinite(P)):
        raise ValueError("P must have shape 4x4 and be finite")

    if not np.allclose(P, P.T) or not np.all(np.linalg.eigvalsh(P) > 0.0):
        raise ValueError("P must be positive-definite and symmetric")

    if np.ndim(r0) != 0 or not np.isfinite(np.asarray(r0)) or r0 < 0.0:
        raise ValueError("r0 must be finite scalar nonnegative scalar")


    lambda_max = max(np.linalg.eigvalsh(B.T@P@B))

    r_w = np.sqrt(lambda_max)*delta_a + r0


    return r_w

def propagate_tube(r_w, rho, alpha_0=0.0):
    """
    Propagate the P-norm tracking-error bound.

    Inputs:
        r_w: (N,) P-norm disturbance bounds
        rho: closed-loop contraction factor
        alpha_0: initial P-norm error bound

    Returns:
        alpha: (N+1,) tube radii
    """

    # input validation
    if r_w.ndim != 1 or not np.all(np.isfinite(np.asarray(r_w))) or not np.all(r_w >= 0.0):
        raise ValueError("r_w must be 1D, finite, and nonnegative")

    if np.ndim(rho) != 0 or rho < 0 or rho >= 1 or not np.isfinite(np.asarray(rho)):
        raise ValueError("rho musta scalar between 0 and 1")

    if np.ndim(alpha_0) != 0 or not np.isfinite(np.asarray(alpha_0)) or alpha_0 < 0:
        raise ValueError("alpha_0 must be finite scalar and nonnegative")

    alpha = np.zeros(r_w.shape[0]+1)
    alpha[0] = alpha_0

    for i in range(len(r_w)):
        alpha[i+1] = rho*alpha[i] + r_w[i]

    return alpha

def compute_friction_tube(
    stance_mask,
    normal_forces,
    mu_plan,
    mu_lo,
    mass,
    B,
    P,
    rho,
    r0=0.0,
    alpha_0=0.0,
):
    """
    Return the tube radii over the entire planning horizon.

    Returns:
        alpha: (N+1,) P-norm tube radii
    """

    # get friction acceleration bound
    delta_a = friction_acceleration_bound(stance_mask, normal_forces, mu_plan, mu_lo, mass)

    # calculate disturbance bound in p-norm units
    r_w = p_norm_disturbance_bound(delta_a, B, P, r0)

    # propogate the tube
    alpha = propagate_tube(r_w, rho, alpha_0)

    return alpha
