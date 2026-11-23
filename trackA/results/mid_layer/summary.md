# Mid-layer comparison on `midlayer_test_v1`

Logs: `data/compare_midlayer/20261122_165808`. One 0.483 m circle plus the 0.263 m tube, friction cones at 0.3, 1.7 m openings. Push = 1.5 s sideways force toward the nearest obstacle at each of the three obstacle groups. Tracking error is measured from the nominal, so it is not available for `follower`. QP times were measured with several runs in parallel.

| controller | conversion | push [m/s²] | goal | falls | runs with contact | mean time [s] | worst error [cm] | worst error / tube | runs outside tube | saturated updates | min body clearance [m] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| follower | lag_inverse | 0 | 10/10 | 0 | 0 | 42.5 | n/a | n/a | n/a | n/a | 0.547 |
| lqr | lag_inverse | 0 | 10/10 | 0 | 0 | 66.9 | 2.9 | 0.11 | 0 | 0 | 0.547 |
| tube | lag_inverse | 0 | 10/10 | 0 | 0 | 66.9 | 2.9 | 0.11 | 0 | 0 | 0.547 |
| lqr | b3 | 0 | 5/5 | 0 | 0 | 67.8 | 19.5 | 0.74 | 0 | 0 | 0.537 |
| lqr | lag_inverse | 1 | 5/5 | 0 | 0 | 67.0 | 22.6 | 0.86 | 0 | 158 | 0.537 |
| tube | lag_inverse | 1 | 5/5 | 0 | 0 | 67.0 | 22.6 | 0.86 | 0 | 158 | 0.537 |
| lqr | b3 | 1 | 5/5 | 0 | 0 | 67.8 | 47.0 | 1.79 | 5 | 0 | 0.268 |
| lqr | lag_inverse | 1.25 | 3/5 | 2 | 0 | 66.7 | 143.0 | 5.44 | 4 | 1409 | 0.147 |
| tube | lag_inverse | 1.25 | 3/5 | 2 | 0 | 66.7 | 150.0 | 5.70 | 4 | 1427 | 0.147 |
| lqr | b3 | 1.25 | 5/5 | 0 | 0 | 67.8 | 63.6 | 2.42 | 5 | 116 | 0.126 |

## Nominal QP

| push [m/s²] | solves | fallbacks | solves with slack > 1 mm | over 100 ms | worst p99 [ms] | worst [ms] |
|---|---|---|---|---|---|---|
| 0 | 6694 | 0 | 0 | 0 | 5.7 | 10.5 |
| 1 | 3352 | 0 | 0 | 0 | 6.0 | 10.0 |
| 1.25 | 2758 | 0 | 0 | 0 | 5.6 | 8.4 |
