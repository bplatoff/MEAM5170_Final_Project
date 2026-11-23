# Sampling the global plan for the midlayer MPC

`planning.trajectory_reference.TrajectoryReference` reconstructs the optimized
point-mass trajectory at any requested time. It does **not** rerun IPOPT or
increase the planner's optimization resolution.

```python
import numpy as np
from planning.trajectory_reference import TrajectoryReference

# Only after verifying the two-pass wrapper succeeded:
assert two_pass_result["success"]
plan = two_pass_result["robust"]
reference = TrajectoryReference(plan, config)  # config used for this robust pass

tick = reference.evaluate(0.62)    # scalar, for a live MPC tick
batch = reference.evaluate(np.arange(0.0, plan["t"][-1], 0.02))
```

Each returned dictionary includes:

- `t` (seconds): query time(s)
- `x` (`[px, py, vx, vy]`): world-frame position (m) and velocity (m/s)
- `u` (`[ax, ay]`): world-frame acceleration (m/s²)
- `yaw`: **unwrapped** nominal heading (rad)
- `yaw_rate`: nominal heading rate (rad/s)

Scalar input produces scalar `t`, `yaw`, and `yaw_rate`, with vectors `x(4)`
and `u(2)`. One-dimensional input of length M produces `t(M)`, `x(M,4)`,
`u(M,2)`, `yaw(M)`, `yaw_rate(M)`.

The position/velocity evaluation exactly uses the plan's constant-acceleration
interval model. At interval boundaries acceleration is right-continuous; the
last available interval control is reported at the final time.

For **three-circle** plans, yaw is reconstructed from the original A* warm
start and the **same degree-3 Legendre sample grid** as the planner and PR #9's
continuous-clearance checker. The supplied `config` must match the solved plan;
the resulting yaw at the optimizer knots is checked against `plan["yaw_ref"]`.
Fixed-yaw mode reports a constant heading and zero heading rate. Path-tangent
mode requires `footprint_max_yaw_rate` so that finite angular-velocity commands
exist; the legacy discontinuous/unbounded heading mode is deliberately rejected.
No extrapolation is performed.

This is an interface for a **desired** motion, not a guarantee that the low-level
Go2 controller can track the commanded yaw or position. Physical yaw-tracking
error and its contribution to three-circle clearance remain Issue #5B.
Use PR #9 certification when available to establish continuous clearance for
the **modeled** trajectory before executing it.

For repeated controller ticks, construct `TrajectoryReference` only once and
reuse `.evaluate(t)`; `evaluate_trajectory(plan, times, config)` is a convenience
one-shot function. This module does not modify Jacob's waypoint follower or
prescribe reference timing/retiming. The MPC can request the desired state at
its own control times.
