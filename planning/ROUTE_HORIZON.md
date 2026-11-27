# Route-length fixed horizon for the global collocation planner

The NLP **still has a fixed horizon**; `T` is not a decision variable. In `horizon_mode='astar_length'`, the planner first finds its standard footprint-aware A* geometric warm start and sets

```
T = astar_path_length_m / cruise_speed_mps + time_buffer_s
```

`cruise_speed_mps=0.4` is the **current conservative starting cruise-speed estimate**, not a measured maximum speed or a guarantee the quadruped can maintain it on a ramp. `time_buffer_s=2.0` is a configurable heuristic allowance for starting, stopping and turning; neither value is optimized or calibrated. `max_speed` is a separate, unchanged optimization constraint. `N` stays at the caller's chosen value: the collocation interval changes to `T/N`.

```python
config = {
    # ... existing planner keys, including footprint and yaw configuration
    'N': 30,
    'horizon_mode': 'astar_length',
    'cruise_speed_mps': 0.4,
    'time_buffer_s': 2.0,
    'horizon_quantum_s': 0.02,  # optional rounding UP to a time grid
}
plan = plan_trajectory(start_state, goal_state, environment, config)
assert plan['success']
print(plan['astar_path_length_m'], plan['t'][-1])
```

A legacy `config={'T':12.0, ...}` continues to mean fixed `T` by default. Or set `horizon_mode='fixed'` explicitly. In length mode, `T` (if present) is intentionally ignored. The harness accepts `--auto-horizon --cruise-speed 0.4 --time-buffer 2.0` and otherwise retains `--T 12.0`.

## Two-pass robustness and time alignment

When `plan_two_pass_friction_tube()` is called in length mode, the **nominal pass** selects `T` from its A* path and rounds it up to a multiple of the provided `error_dt`. The **robust pass** is forced to use that same `T`; changing the inflation can change the robust A* route but not the duration. This keeps the disturbance/contact prediction's `M*error_dt == T` assumption valid. `robust_config` in the returned result records the actual pinned duration. The contact predictor must allocate its samples from `nominal['t'][-1]`, not a hard-coded duration.

If the inflated A* route is substantially longer, a fixed nominal-pass duration could make the robust solve too aggressive or infeasible. Report planning failure rather than silently using the uninflated plan. A future speed adjustment may be justified by real track-A tracking/ramp data; do not assume 0.4 m/s is achievable everywhere.

The yaw-reference rate bound is time-based and is reconstructed with the chosen `T`. `TrajectoryReference(plan, config)` evaluates using the returned `t` grid as before. This change does not modify the LQR gain, friction disturbance equations, or tube recursion; it changes how many 0.02-s tube steps the nominal horizon covers. PR #9's continuous clearance must be retested after merging because it also modifies `planning/direct_collocation.py`.
