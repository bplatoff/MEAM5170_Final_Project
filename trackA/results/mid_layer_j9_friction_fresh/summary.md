# Three-circle and measured-state MPC comparison (J8, J9)

Logs: `data/compare_midlayer/20261124_friction_fresh`. Three-circle footprint, friction cones at 0.3. `midlayer_test_v1` has 1.7 m openings and `midlayer_gap125_v1` has 1.25 m openings. Push = 1.5 s sideways force toward the nearest obstacle at each of the three obstacle groups. **Error is the distance from the planner's reference at the same time**, for every version. "Outside tube" means that error exceeded the planner's full tube (0.263 m). QP times were measured with several runs in parallel.

| course | version | push [m/s²] | goal | falls | runs with contact | mean time [s] | worst error [cm] | worst error / tube | runs outside tube | worst yaw error [rad] | saturated updates | min body clearance [m] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| midlayer_test_v1 | QP carried | 1 | 3/3 | 0 | 0 | 65.5 | 10.9 | 0.42 | 0 | 0.044 | 0 | 0.297 |
| midlayer_test_v1 | QP measured | 1 | 3/3 | 0 | 1 | 65.5 | 75.0 | 2.85 | 1 | 0.377 | 664 | 0.098 |
| midlayer_test_v1 | QP carried | 1.25 | 1/3 | 2 | 0 | 65.9 | 36.4 | 1.38 | 3 | 0.320 | 334 | 0.111 |
| midlayer_test_v1 | QP measured | 1.25 | 3/3 | 0 | 1 | 65.5 | 64.0 | 2.43 | 2 | 0.139 | 655 | 0.001 |
| midlayer_gap125_v1 | QP carried | 1 | 3/3 | 0 | 0 | 66.3 | 11.0 | 0.42 | 0 | 0.039 | 0 | 0.316 |
| midlayer_gap125_v1 | QP measured | 1 | 3/3 | 0 | 0 | 66.3 | 22.0 | 0.84 | 0 | 0.053 | 182 | 0.248 |

## QP

| course | version | push [m/s²] | solves | fallbacks | tube slack solves | body slack solves | speed slack solves | max body slack [cm] | over 100 ms | worst p99 [ms] | worst [ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| midlayer_test_v1 | QP carried | 1 | 1966 | 0 | 0 | 0 | 0 | 0.00 | 0 | 20.2 | 47.9 |
| midlayer_test_v1 | QP measured | 1 | 1965 | 0 | 45 | 8 | 48 | 4.46 | 28 | 225.9 | 693.8 |
| midlayer_test_v1 | QP carried | 1.25 | 1362 | 0 | 0 | 0 | 0 | 0.00 | 0 | 21.2 | 40.1 |
| midlayer_test_v1 | QP measured | 1.25 | 1965 | 0 | 31 | 10 | 48 | 9.76 | 28 | 216.4 | 299.5 |
| midlayer_gap125_v1 | QP carried | 1 | 1990 | 0 | 0 | 0 | 0 | 0.00 | 0 | 19.0 | 45.5 |
| midlayer_gap125_v1 | QP measured | 1 | 1990 | 0 | 0 | 0 | 0 | 0.00 | 0 | 27.8 | 64.3 |
