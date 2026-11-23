# Jacob item #5 — starting heading and reference yaw-rate limit

The global collocation planner is **offline**. It plans **before each trial**
using the trial's known initial pose; it does not continuously read heading
measurements or re-plan during locomotion.

The 4D collocation state remains `[px, py, vx, vy]`. The initial heading is
supplied **separately** in `config['footprint_yaw']`; with
`footprint_yaw_mode='path_tangent'` it means *initial* yaw, not fixed yaw.
The 0.5 rad/s rate cap is supplied in
`config['footprint_max_yaw_rate']`. The rate limit applies to the
**prescribed nominal yaw reference**, not guaranteed robot tracking.

For randomized starts (the project plan specifies initial headings ±15°),
use the **actual seeded spawn yaw for each run**, whether determined from the
spawn seed or read from the simulator once before that run begins:

```python
from planning.start_yaw_config import config_for_initial_yaw

# base_config includes N, T, friction/margin, robot dimensions, etc., and
# uses footprint_model='three_circle', footprint_yaw_mode='path_tangent'.
# spawn_yaw_rad comes from the spawn pose chosen for THIS seeded trial.
run_config = config_for_initial_yaw(
    base_config, initial_yaw_rad=spawn_yaw_rad
)

result = plan_trajectory(start_state, goal_state, environment, run_config)
# For two-pass planning, pass `run_config` to
# `plan_two_pass_friction_tube(...)` instead. Both passes get the same yaw.
```

This helper copies the config, replaces any stale `footprint_yaw` with the
current run's explicit starting heading, sets the rate cap to 0.5 rad/s when
absent, and rejects an explicit limit above 0.5 rad/s. A stricter limit is
preserved. Both planner calls remain unchanged. No IPOPT/mesh changes.

If planning a **single reusable path for multiple randomized start poses**,
there is no one heading that matches every spawn. To satisfy item #5, use a
per-run plan or demonstrate a robust heading-error bound for the variations.

**Remaining external dependency:** Jacob/Ben's runner must supply its actual
start yaw when invoking the planner. The physical yaw-tracking error bound and
its effect on three-circle clearance are part of official Task 2.5 and are not
claimed to be complete by this helper.
