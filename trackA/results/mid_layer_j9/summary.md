# Three-circle and measured-state MPC comparison (J8, J9)

Logs: `data/compare_midlayer/20261122_230424`. Three-circle footprint, friction cones at 0.3. `midlayer_test_v1` has 1.7 m openings and `midlayer_gap125_v1` has 1.25 m openings. Push = 1.5 s sideways force toward the nearest obstacle at each of the three obstacle groups. **Error is the distance from the planner's reference at the same time**, for every version. "Outside tube" means that error exceeded the planner's full tube (0.263 m). QP times were measured with several runs in parallel.

| course | version | push [m/s²] | goal | falls | runs with contact | mean time [s] | worst error [cm] | worst error / tube | runs outside tube | worst yaw error [rad] | saturated updates | min body clearance [m] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| midlayer_test_v1 | follower | 0 | 10/10 | 0 | 0 | 42.2 | n/a | n/a | n/a | n/a | n/a | 0.406 |
| midlayer_test_v1 | lqr | 0 | 10/10 | 0 | 0 | 65.7 | 6.3 | 0.24 | 0 | 0.037 | 0 | 0.388 |
| midlayer_test_v1 | QP carried | 0 | 10/10 | 0 | 0 | 65.7 | 6.3 | 0.24 | 0 | 0.037 | 0 | 0.388 |
| midlayer_test_v1 | QP measured | 0 | 10/10 | 0 | 0 | 65.6 | 5.4 | 0.21 | 0 | 0.041 | 5 | 0.387 |
| midlayer_test_v1 | QP carried | 1 | 5/5 | 0 | 0 | 65.8 | 11.0 | 0.42 | 0 | 0.045 | 0 | 0.297 |
| midlayer_test_v1 | QP measured | 1 | 5/5 | 0 | 0 | 65.8 | 71.3 | 2.71 | 1 | 0.175 | 639 | 0.181 |
| midlayer_test_v1 | QP carried | 1.25 | 3/5 | 2 | 0 | 65.7 | 57.9 | 2.20 | 4 | 0.222 | 1035 | 0.065 |
| midlayer_test_v1 | QP measured | 1.25 | 3/5 | 2 | 0 | 66.1 | 75.2 | 2.86 | 3 | 0.277 | 540 | 0.059 |
| midlayer_gap125_v1 | QP carried | 0 | 5/5 | 0 | 0 | 66.3 | 5.8 | 0.22 | 0 | 0.033 | 0 | 0.406 |
| midlayer_gap125_v1 | QP measured | 0 | 5/5 | 0 | 0 | 66.3 | 4.6 | 0.18 | 0 | 0.037 | 0 | 0.403 |
| midlayer_gap125_v1 | QP carried | 1 | 5/5 | 0 | 0 | 66.3 | 23.6 | 0.90 | 0 | 0.101 | 152 | 0.302 |
| midlayer_gap125_v1 | QP measured | 1 | 5/5 | 0 | 0 | 66.3 | 30.1 | 1.15 | 1 | 0.187 | 619 | 0.254 |

## QP

| course | version | push [m/s²] | solves | fallbacks | tube slack solves | body slack solves | speed slack solves | max body slack [cm] | over 100 ms | worst p99 [ms] | worst [ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| midlayer_test_v1 | QP carried | 0 | 6571 | 0 | 0 | 0 | 0 | 0.00 | 0 | 20.4 | 36.2 |
| midlayer_test_v1 | QP measured | 0 | 6569 | 0 | 0 | 0 | 0 | 0.00 | 0 | 26.1 | 45.9 |
| midlayer_test_v1 | QP carried | 1 | 3291 | 0 | 0 | 0 | 0 | 0.00 | 0 | 17.4 | 32.8 |
| midlayer_test_v1 | QP measured | 1 | 3290 | 0 | 42 | 5 | 50 | 5.11 | 3 | 68.0 | 190.2 |
| midlayer_test_v1 | QP carried | 1.25 | 2687 | 0 | 0 | 0 | 0 | 0.00 | 0 | 18.6 | 28.1 |
| midlayer_test_v1 | QP measured | 1.25 | 2694 | 0 | 10 | 7 | 16 | 15.62 | 12 | 116.6 | 547.7 |
| midlayer_gap125_v1 | QP carried | 0 | 3316 | 0 | 0 | 0 | 0 | 0.00 | 0 | 39.5 | 58.7 |
| midlayer_gap125_v1 | QP measured | 0 | 3316 | 0 | 0 | 0 | 0 | 0.00 | 0 | 26.7 | 39.8 |
| midlayer_gap125_v1 | QP carried | 1 | 3316 | 0 | 0 | 0 | 0 | 0.00 | 0 | 43.7 | 85.0 |
| midlayer_gap125_v1 | QP measured | 1 | 3316 | 0 | 0 | 0 | 0 | 0.00 | 0 | 30.2 | 45.3 |
