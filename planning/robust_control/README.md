# Horizontal CoM Error Model and Friction Tube (Tasks 2.1–2.2)

This directory provides the simplified horizontal center-of-mass (CoM) tracking-error model and friction-uncertainty tube calculation for the robust MPC portion of the project. Task 2.1 designs the auxiliary LQR feedback and certifies its contraction in a Lyapunov norm. Task 2.2 uses a contact/friction schedule to bound additive disturbances and propagate a time-varying tracking-error tube. These are standalone planning/modeling utilities, not an integrated quadruped controller.

**Files**
- `tube_error_model.py` — model construction, LQR gain, Lyapunov certificate, and public design function.
- `test_tube_error_model.py` — standalone stability, contraction, and recovery tests.
- `friction_tube.py` — friction-gap acceleration bound, conversion to the Lyapunov $P$-norm, tube propagation, and single-call interface.
- `test_friction_tube.py` — disturbance, validation, propagation, and end-to-end interface tests.

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

For Task 2.2, an additive disturbance $w_k$ in the **state-error dynamics** gives the bound

$$
\alpha_{k+1}\le\rho\alpha_k+\|w_k\|_P,
\qquad \alpha_k\ge\|e_k\|_P.
$$

The disturbance term and its units must be defined consistently with the actual modeling error or friction mismatch. In particular, a disturbance specified as acceleration must first be mapped through the input matrix before taking its $P$-norm. A $P$-norm tube radius cannot be treated directly as a position clearance in meters without projecting it into the position coordinates.

## Friction-disturbance model and tube propagation (Task 2.2)

For each prediction interval $k$ and each of the four feet $i$, define:

- $s_{k,i}\in\{0,1\}$: stance indicator (1 means the foot is in contact).
- $f_{n,k,i}\ge 0$: **surface-normal** contact force in newtons (not necessarily world-vertical on sloped terrain).
- $\mu_{\mathrm{plan},k,i}$: friction coefficient used for planning.
- $\mu_{\mathrm{lo},k,i}$: conservative lower bound on available friction.
- $m>0$: robot mass in kilograms.

The modeled upper bound on missing *horizontal* acceleration is

$$
\Delta a_k = \frac{1}{m}\sum_{i=1}^{4} s_{k,i}
\max\bigl(0,\mu_{\mathrm{plan},k,i}-\mu_{\mathrm{lo},k,i}\bigr) f_{n,k,i}.
$$

`friction_acceleration_bound()` returns the nonnegative array `delta_a` of shape `(N,)`, where $N$ is the number of prediction intervals. Swing feet contribute nothing, even if their force array entries are nonzero. Friction values may be scalars or arrays broadcastable to `(N, 4)` (for example `(4,)` per foot, `(N, 1)` per interval, or `(N, 4)` per foot and interval).

Treat $\Delta a_k$ as a bound on the Euclidean norm of an unknown planar acceleration disturbance, $\lVert d_k\rVert_2\le\Delta a_k$. Under the same piecewise-constant-acceleration discretization as Task 2.1, $w_k=B_dd_k$ enters the four-dimensional state-error dynamics. The direction-independent bound is

$$
\begin{aligned}
c_P &= \sqrt{\lambda_{\max}(B_d^\top P B_d)}, \\
r_{w,k} &= c_P\Delta a_k+r_0, \\
\alpha_{k+1} &= \rho\alpha_k+r_{w,k},\qquad k=0,\ldots,N-1.
\end{aligned}
$$

Here $r_0\ge0$ is an **additional per-interval disturbance allowance already expressed in $P$-norm units**; it must not be interpreted as acceleration. The initial radius $\alpha_0\ge0$ describes the initially bounded tracking error. With `alpha_0=0`, the model assumes no initial error. The recursion returns `alpha` of shape `(N+1,)`, including the initial radius and the radius after each interval. With $r_{w,k}=0$ and $0\le\rho<1$, the tube contracts.

**Modeling assumptions and scope.** The friction-gap formula is a conservative *model* of unrealizable horizontal contact acceleration under the assumed normal-force and friction-limit interpretation. It is not, by itself, a verified bound on actual CoM acceleration error in MuJoCo. In particular, it does not infer the commanded tangential forces, model contact switching/transients, account for actuator limits or yaw/vertical coupling, or guarantee that horizontal-world-frame friction losses on ramps match the modeled planar disturbance. The stance schedule and normal-force estimates must come from a consistent nominal contact plan. The matrix $B_d$, certificate $P$, and contraction $\rho$ must all come from the **same** error model and prediction timestep. The residual $r_0$ requires justification/calibration (Task 2.5); setting it to zero disregards unmodeled disturbances.

The returned radii bound $\lVert e_k\rVert_P$ **under those assumptions**. They are **not distances in meters**. MPC obstacle tightening (Task 2.4) needs a justified projection from the four-dimensional $P$-norm tube to position-space margins, and must also check correction feasibility under the friction and actuator limits.

## Usage

Run from the repository root in the project Pixi environment (see the repository's `SETUP.md`):

```python
from planning.robust_control.tube_error_model import design_tube_error_model

model = design_tube_error_model(dt=0.02)
K = model["K"]
P = model["P"]
rho = model["rho"]
Acl = model["Acl"]
```

`design_tube_error_model()` also accepts `pos_scale`, `vel_scale`, and `acc_scale`, and returns a dictionary with `A`, `B`, `K`, `Acl`, `P`, and `rho`. It computes the design once; it does **not** run the robot controller or simulation loop. The underlying functions remain available for custom weights or separate analyses.

For Task 2.2, pass the Task 2.1 model parameters and the predicted contact schedule to `compute_friction_tube()`:

```python
import numpy as np
from planning.robust_control.tube_error_model import design_tube_error_model
from planning.robust_control.friction_tube import compute_friction_tube

model = design_tube_error_model(dt=0.02)
stance = np.array([
    [1, 1, 1, 1],
    [1, 1, 0, 0],
    [1, 1, 1, 1],
])
normal_forces = np.array([
    [50, 50, 50, 50],
    [100, 100, 0, 0],
    [50, 50, 50, 50],
], dtype=float)
mu_lo = np.array([
    [0.5, 0.5, 0.5, 0.5],
    [0.3, 0.3, 0.5, 0.5],
    [0.5, 0.5, 0.5, 0.5],
])

alpha = compute_friction_tube(
    stance_mask=stance,
    normal_forces=normal_forces,
    mu_plan=0.5,
    mu_lo=mu_lo,
    mass=20.0,
    B=model["B"],
    P=model["P"],
    rho=model["rho"],
    r0=0.0,       # Replace with a justified residual allowance
    alpha_0=0.0,  # Replace if initial tracking error is nonzero
)
assert alpha.shape == (stance.shape[0] + 1,)
```

This is an illustrative contact schedule, not a calibrated Go2 trajectory. The lower-level functions `friction_acceleration_bound()`, `p_norm_disturbance_bound()`, and `propagate_tube()` are also exposed for diagnostics or testing. Each validates its documented inputs and raises `ValueError` for invalid parameter values.

## Validation

From the repository root:

```bash
python -m unittest discover -s planning/robust_control -p "test_*.py" -v
```

**Latest reported test results:** 3 tests passed for Task 2.1, and 22 tests passed for Task 2.2 in the shared Pixi environment (25 tests across the two suites). The Task 2.1 tests check closed-loop stability, positive definiteness and the Lyapunov equation, contraction along simulated disturbance-free trajectories, recovery over 100 steps (2 s), and repeatability of a stabilizing design for `dt = 0.01`, `0.02`, and `0.04` s. The Task 2.2 tests exercise stance masking, zero and variable friction gaps, force/mass behavior, the $P$-norm bound for different disturbance directions, residual uncertainty $r_0$, tube recursion, input rejection, and the single-call `compute_friction_tube()` interface. These are numerical/implementation checks, not physical validation of the disturbance bound.

For the default design, simulated peak corrective accelerations were:

| Initial error $[e_x,e_y,e_{v_x},e_{v_y}]$ | Case | Peak correction |
|---|---|---:|
| `[0.05, 0, 0, 0]` | Position | 0.923 m/s² |
| `[0, 0, 0.2, 0]` | Velocity | 1.526 m/s² |
| `[0.05, 0, 0.2, 0]` | Combined | 2.448 m/s² |
| `[0, 0.05, 0, 0]` | Lateral position | 0.923 m/s² |

In the combined case, the forward position error briefly increases above 0.05 m before recovering because the initial velocity error points away from the reference. The weighted norm still contracts as predicted. The larger acceleration requests are expected from **unconstrained** LQR and do not establish that the robot can generate those corrections on low-friction terrain.

## Integration status / limitations

The standalone **Task 2.1 error model and numerical certificate** and **Task 2.2 friction-gap tube calculation** are implemented and tested. They are not yet integrated into the running quadruped MPC. The following remain for robust MPC integration and physical validation:

- Define how nominal and actual Go2 CoM states are extracted and compared in a consistent world frame, and how the auxiliary acceleration correction maps into the existing MPC/contact-force formulation.
- Connect planned footholds/contact indicators, surface-normal forces, and friction estimates to `compute_friction_tube()` at the MPC prediction horizon.
- Convert each $P$-norm tube radius into justified horizontal position/obstacle half-space tightening (Task 2.4); do not use $\alpha_k$ directly as a meter-valued clearance.
- Check actual corrective-force feasibility under contact, friction, and actuator limits; the LQR acceleration penalty is **not** a saturation constraint.
- Compare observed one-step CoM/model errors against the analytical disturbance bound in MuJoCo and calibrate $r_0$ or revise the model where necessary (Task 2.5).
- Verify any claimed closed-loop collision-avoidance guarantee against the full robot simulation. Stability of this simplified model alone is **not** proof of collision avoidance or stability of the full quadruped.
