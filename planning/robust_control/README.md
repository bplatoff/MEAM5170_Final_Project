# Horizontal CoM Tracking-Error Model (Task 2.1)

This module defines a simplified horizontal center-of-mass (CoM) error model, an LQR feedback law, and a Lyapunov contraction bound for the robust MPC portion of the project. It is intended to provide the nominal error dynamics and contraction parameters needed for the disturbance/tube calculations in Task 2.2.

**Files**
- `tube_error_model.py` — model construction, LQR gain, Lyapunov certificate, and public design function.
- `test_tube_error_model.py` — standalone stability, contraction, and recovery tests.

## Model and assumptions

The error state and corrective input are

$$
e_k = [e_x,\ e_y,\ e_{v_x},\ e_{v_y}]^\top,
\qquad
u_k = [\Delta a_x,\ \Delta a_y]^\top.
$$

Errors are defined relative to the nominal/reference CoM position and velocity. The input represents an **additional horizontal acceleration correction**, not an MPC ground-reaction force or joint command. For the disturbance-free auxiliary model,

$$
e_{k+1}=A_de_k+B_du_k,\qquad u_k=Ke_k,
$$

with

$$
A_d=\begin{bmatrix}I_2&\Delta t I_2\\0&I_2\end{bmatrix},
\qquad
B_d=\begin{bmatrix}\tfrac12\Delta t^2 I_2\\\Delta t I_2\end{bmatrix}.
$$

This is a discrete planar double integrator with piecewise-constant corrective acceleration over each interval. The axes are modeled independently. It does not include yaw/turning coupling, contact changes, leg dynamics, actuator saturation, or friction constraints; those effects must be accounted for separately when relating the error model to the full quadruped simulation.

## Design choices

| Parameter | Default | Rationale |
|---|---:|---|
| Sample interval `dt` | 0.02 s | Matches the nominal MPC prediction-grid timestep used in the project configuration. This is not the MuJoCo physics timestep or necessarily the actual control-update period. |
| Position scale | 0.05 m | Represents a meaningful horizontal tracking offset for the planned route; it is a tuning scale, not a guaranteed clearance limit. |
| Velocity scale | 0.20 m/s | A velocity mismatch of this size would accumulate roughly 0.05 m of displacement in 0.25 s without correction. |
| Acceleration scale | 1.0 m/s² | Penalizes aggressive corrections without imposing a hard acceleration limit. |

The LQR costs are normalized by the squared physical scales:

$$
Q=\operatorname{diag}\left(\frac{1}{0.05^2},\frac{1}{0.05^2},\frac{1}{0.20^2},\frac{1}{0.20^2}\right)
=\operatorname{diag}(400,400,25,25),
\qquad R=I_2.
$$

Equal weights are used in the x and y directions because the planned path changes direction; the design does not assume that one horizontal axis is consistently more important. These weights specify relative penalties, **not bounds** on position error, velocity error, or acceleration.

The discrete algebraic Riccati equation is solved for the infinite-horizon LQR cost matrix `S`, followed by the signed feedback gain `K`. The closed-loop dynamics are `Acl = A + B @ K`.

## Lyapunov certificate and tube interface

With $A_{cl}=A_d+B_dK$, the certificate uses $Q_L=Q$ and solves

$$
A_{cl}^{\top}P A_{cl}-P=-Q_L.
$$

Using the same positive-definite weights as the LQR cost keeps the Lyapunov norm aligned with the chosen position/velocity scales without adding another set of tuning parameters. `P` is a separate Lyapunov matrix; it is not the Riccati matrix `S`.

Define $\|e\|_P=\sqrt{e^\top P e}$. The worst-case one-step contraction is computed from the largest generalized eigenvalue:

$$
\rho=\sqrt{\lambda_{\max}(A_{cl}^{\top}PA_{cl},\ P)},
\qquad \|e_{k+1}\|_P\le\rho\|e_k\|_P.
$$

For the default configuration:

- $K\approx\begin{bmatrix}-18.4514&0&-7.6277&0\\0&-18.4514&0&-7.6277\end{bmatrix}$
- Maximum closed-loop eigenvalue magnitude: **0.92257**
- Contraction factor: **$\rho=0.97397066$** per 0.02 s step
- Lyapunov residual norm: approximately **$1.2\times10^{-12}$**

The contraction factor is larger than the eigenvalue magnitude because it bounds the *worst-case single-step change in the $P$-norm*, not the asymptotic decay rate inferred from eigenvalues.

For subsequent tube calculations, an additive disturbance $w_k$ in the **state-error dynamics** would give the bound

$$
\alpha_{k+1}\le\rho\alpha_k+\|w_k\|_P,
\qquad \alpha_k\ge\|e_k\|_P.
$$

The disturbance term and its units must be defined consistently with the actual modeling error or friction mismatch. In particular, a disturbance specified as acceleration must first be mapped through the input matrix before taking its $P$-norm. A $P$-norm tube radius cannot be treated directly as a position clearance in meters without projecting it into the position coordinates.

## Usage

Run from the repository root with NumPy and SciPy installed:

```python
from planning.robust_control.tube_error_model import design_tube_error_model

model = design_tube_error_model(dt=0.02)
K = model["K"]
P = model["P"]
rho = model["rho"]
Acl = model["Acl"]
```

`design_tube_error_model()` also accepts `pos_scale`, `vel_scale`, and `acc_scale`, and returns a dictionary with `A`, `B`, `K`, `Acl`, `P`, and `rho`. It computes the design once; it does **not** run the robot controller or simulation loop. The underlying functions remain available for custom weights or separate analyses.

## Validation

From the repository root:

```bash
python -m unittest discover -s planning/robust_control -p "test_*.py" -v
```

**Current result: 3 tests passed.** The tests check closed-loop stability, positive definiteness and the Lyapunov equation, contraction along simulated disturbance-free trajectories, recovery over 100 steps (2 s), and repeatability of a stabilizing design for `dt = 0.01`, `0.02`, and `0.04` s.

For the default design, simulated peak corrective accelerations were:

| Initial error $[e_x,e_y,e_{v_x},e_{v_y}]$ | Case | Peak correction |
|---|---|---:|
| `[0.05, 0, 0, 0]` | Position | 0.923 m/s² |
| `[0, 0, 0.2, 0]` | Velocity | 1.526 m/s² |
| `[0.05, 0, 0.2, 0]` | Combined | 2.448 m/s² |
| `[0, 0.05, 0, 0]` | Lateral position | 0.923 m/s² |

In the combined case, the forward position error briefly increases above 0.05 m before recovering because the initial velocity error points away from the reference. The weighted norm still contracts as predicted. The larger acceleration requests are expected from **unconstrained** LQR and do not establish that the robot can generate those corrections on low-friction terrain.

## Integration status / limitations

This completes the standalone **Task 2.1 model and numerical certificate**. The following still need to be connected or verified as part of the robust MPC integration:

- Define how nominal and actual Go2 CoM states are extracted and compared in a consistent world frame, and how the auxiliary acceleration correction maps into the existing MPC/contact-force formulation.
- Bound friction/model mismatch as a disturbance in the error dynamics and propagate the tube radius (Task 2.2).
- Check actual corrective-force feasibility under contact and friction limits; the LQR acceleration penalty is **not** a saturation constraint.
- Validate any claimed tube/clearance bound against the full MuJoCo quadruped simulation. Stability of this simplified model alone is **not** a proof of collision avoidance or closed-loop stability of the full robot.
