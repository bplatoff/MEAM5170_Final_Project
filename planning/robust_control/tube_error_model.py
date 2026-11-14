
import numpy as np
from scipy.linalg import solve_discrete_are, solve_discrete_lyapunov, eigh


def discrete_error_dynamics(dt: float = 0.02):
    """
    Construct discrete horizontal CoM error dynamics.

    State: [ex, ey, evx, evy]
    Input: [delta_ax, delta_ay]

    Returns:
        A: State transition matrix (4, 4)
        B: Input matrix (4, 2)
    """
    if dt <= 0:
        raise ValueError("dt must be positive")

    I = np.eye(2)
    Z = np.zeros((2, 2))

    # Assemble A using 2x2 blocks
    A = np.block([[I, dt*I],
                  [Z, I]])

    # Assemble B using 2x2 blocks
    B = np.block([[0.5*(dt**2)*I], 
                  [dt*I]])

    # Return A and B
    return A, B


def lqr_weight_matrices(pos_scale: float = 0.05, vel_scale: float = 0.2, acc_scale: float = 1.0):

    """
    Construct LQR cost matrices using physical scales.

    State: [ex, ey, evx, evy]
    Input: [delta_ax, delta_ay]

    Returns:
        Q: State cost matrix (4, 4)
        R: Control cost matrix (2, 2)
    """
    if min(pos_scale, vel_scale, acc_scale) <= 0:
        raise ValueError("All scales must be positive")

    # Calculate the position, velocity, and acceleration penalty coefficients
    cp = 1/(pos_scale**2)
    cv = 1/(vel_scale**2)
    ca = 1/(acc_scale**2)

    # Assemble Q, with equal x/y weights
    Q = np.diag([cp, cp, cv, cv])

    # Assemble R, with equal x/y acceleration weights
    R = ca*np.eye(2)

    return Q, R

def lqr_feedback(A, B, Q, R):
    """
    Calculate the discrete LQR feedback gain.

    Returns:
        K: State feedback gain (2, 4)
        S: LQR cost-to-go matrix (4, 4)
    """

    # Calculate S using the discrete Riccati equation
    S = solve_discrete_are(A, B, Q, R)

    # Calculate K using S, A, B, and R
    K = np.linalg.solve(R + B.T @ S @ B, -B.T @ S @ A)

    return K, S


def lyapunov_certificate(A_cl, Q_L):
    """
    Compute a Lyapunov matrix and contraction factor.

    Args:
        A_cl: Closed-loop state transition matrix (4, 4)
        Q_L: Positive-definite Lyapunov weighting matrix (4, 4)

    Returns:
        P: Positive-definite Lyapunov matrix (4, 4)
        rho: Worst-case contraction factor in the P-norm
    """

    # Solve the discrete Lyapunov equation for P
    P = solve_discrete_lyapunov(A_cl.T, Q_L)

    # Construct the matrix A_cl.T @ P @ A_cl
    A_cl_P_A_cl = A_cl.T @ P @ A_cl

    # Find the generalized eigenvalues associated with (A_cl.T @ P @ A_cl, P)
    gen_eigs = eigh(A_cl_P_A_cl, P, eigvals_only=True)

    # Calculate rho from the largest eigenvalue
    rho = np.sqrt(gen_eigs[-1])

    assert np.all(np.linalg.eigvalsh(P) > 0), "The Lyapunov matrix must be positive definite."
    assert rho < 1, "The worst-case contraction factor must be less than 1 for stability."
    assert np.allclose(A_cl.T@P@A_cl - P + Q_L, 0)

    return P, rho


def design_tube_error_model(dt: float = 0.02, pos_scale: float = 0.05, vel_scale: float = 0.2, acc_scale: float = 1.0):

    """
    Design the horizontal CoM error model and feedback.

    Returns a dictionary containing the model,
    feedback gain, and contraction certificate.
    """

    # Construct A, B
    A, B = discrete_error_dynamics(dt)

    # Construct Q, R
    Q, R = lqr_weight_matrices(pos_scale, vel_scale, acc_scale)

    # Calculate K
    K, S = lqr_feedback(A, B, Q, R)

    # Construct the closed-loop matrix
    Acl = A + B@K

    # Compute P and rho
    P, rho = lyapunov_certificate(Acl, Q)

    return {
        "A": A,
        "B": B,
        "K": K,
        "Acl": Acl,
        "P": P,
        "rho": rho,
    }
