---
paths:
  - "trackA/**"
  - "controllers/**"
---
# Track A: simulation and MPC (owner Jacob)

- `trackA/sim/run_waypoints.py` is the standard runner and log format (C1). Keep its log keys stable; the batch
  runner (2.3) and the disturbance check (2.5) read them. Add keys rather than renaming.
- Commands to Quadruped-PyMPC are world-frame [v_x, v_y, 0] and [0, 0, yaw_rate], as in `PathFollower`.
- Per-foot friction goes through `set_friction`, walls through `set_walls`. Cones use μ_lo, not μ_plan (B4).
- Pending changes (see `docs/status.md`, Migration): absolute start pose, path array input, fall vs collision,
  per-foot μ from the friction map.
- The mid-layer MPC (2.4) uses Austin's `design_tube_error_model` for A, B, K, P, ρ; do not recompute K online.
