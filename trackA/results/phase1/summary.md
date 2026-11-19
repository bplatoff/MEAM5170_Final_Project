# Step 1.5 exit test: flat_v1_20runs

**PASS**: 20 of 20 runs reached the goal (need 19).

Course `trackC/courses/flat_v1.yaml`, v_ref 0.4 m/s, timeout 90.0 s, spawn jitter +-0.1 m / +-15 deg, MPC upstream nominal (no MPC walls), commit dfec012-dirty (Quadruped-PyMPC 025eab4).

## Per path type

| | goal | mean time to goal [s] | mean \|e\| [m] | mean \|e\| settled [m] | mean p95 \|e\| [m] | max \|e\| [m] | mean speed err [m/s] | min clearance: any wall / side walls [m] | wall-hit events (runs) | failed MPC solves | MPC mean / max [ms] | slips: logger / after touchdown (stances) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| straight | 10/10 | 31.5 | 0.007 | 0.005 | 0.025 | 0.098 | 0.057 | 0.715 / 1.402 | 0 (0) | 0 | 2.27 / 20.94 | 1759 / 0 (1768) |
| zigzag | 10/10 | 42.3 | 0.043 | 0.045 | 0.172 | 0.214 | 0.062 | 0.403 / 0.403 | 0 (0) | 0 | 2.35 / 108.21 | 2378 / 0 (2453) |
| all | 20/20 | 36.9 | 0.025 | 0.025 | 0.098 | 0.214 | 0.059 | 0.403 / 0.403 | 0 (0) | 0 | 2.31 / 108.21 | 4137 / 0 (4221) |

## Per run

| run | outcome | time [s] | final dist [m] | spawn dx, dy [m], dyaw [deg] | mean \|e\| | settled | p95 | max | min clearance any / side [m] | wall hits | MPC max [ms] | failed solves | slips (after touchdown) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| run_00_straight_s00 | goal | 31.5 | 0.100 | +0.027, -0.046, -13.8 | 0.007 | 0.005 | 0.017 | 0.049 | 0.802 / 1.451 | 0 | 14.22 | 0 | 176 (0) |
| run_01_straight_s01 | goal | 31.6 | 0.100 | +0.002, +0.090, -10.7 | 0.010 | 0.006 | 0.045 | 0.090 | 0.800 / 1.409 | 0 | 7.00 | 0 | 178 (0) |
| run_02_straight_s02 | goal | 31.7 | 0.100 | -0.048, -0.040, +9.4 | 0.006 | 0.005 | 0.010 | 0.045 | 0.750 / 1.455 | 0 | 14.97 | 0 | 177 (0) |
| run_03_straight_s03 | goal | 31.8 | 0.100 | -0.083, -0.053, +9.0 | 0.007 | 0.005 | 0.015 | 0.057 | 0.715 / 1.443 | 0 | 10.58 | 0 | 177 (0) |
| run_04_straight_s04 | goal | 31.3 | 0.100 | +0.089, +0.002, +14.3 | 0.005 | 0.005 | 0.010 | 0.014 | 0.802 / 1.486 | 0 | 5.00 | 0 | 174 (0) |
| run_05_straight_s05 | goal | 31.4 | 0.100 | +0.061, +0.062, +0.5 | 0.008 | 0.005 | 0.035 | 0.062 | 0.802 / 1.438 | 0 | 5.45 | 0 | 175 (0) |
| run_06_straight_s06 | goal | 31.6 | 0.100 | +0.008, -0.031, -3.9 | 0.006 | 0.005 | 0.011 | 0.035 | 0.802 / 1.465 | 0 | 13.08 | 0 | 175 (0) |
| run_07_straight_s07 | goal | 31.5 | 0.100 | +0.025, +0.079, +8.3 | 0.010 | 0.006 | 0.044 | 0.080 | 0.802 / 1.421 | 0 | 9.23 | 0 | 175 (0) |
| run_08_straight_s08 | goal | 31.7 | 0.100 | -0.035, +0.097, -5.4 | 0.010 | 0.006 | 0.049 | 0.098 | 0.763 / 1.402 | 0 | 20.94 | 0 | 177 (0) |
| run_09_straight_s09 | goal | 31.4 | 0.100 | +0.074, -0.043, +3.1 | 0.006 | 0.005 | 0.013 | 0.046 | 0.802 / 1.454 | 0 | 10.57 | 0 | 175 (0) |
| run_10_zigzag_s00 | goal | 42.4 | 0.100 | +0.027, -0.046, -13.8 | 0.043 | 0.044 | 0.171 | 0.207 | 0.410 / 0.410 | 0 | 17.82 | 0 | 238 (0) |
| run_11_zigzag_s01 | goal | 42.3 | 0.100 | +0.002, +0.090, -10.7 | 0.045 | 0.045 | 0.174 | 0.214 | 0.404 / 0.404 | 0 | 8.70 | 0 | 237 (0) |
| run_12_zigzag_s02 | goal | 42.5 | 0.100 | -0.048, -0.040, +9.4 | 0.041 | 0.044 | 0.170 | 0.205 | 0.409 / 0.409 | 0 | 14.86 | 0 | 239 (0) |
| run_13_zigzag_s03 | goal | 42.6 | 0.100 | -0.083, -0.053, +9.0 | 0.041 | 0.044 | 0.171 | 0.200 | 0.405 / 0.405 | 0 | 37.78 | 0 | 238 (0) |
| run_14_zigzag_s04 | goal | 42.1 | 0.100 | +0.089, +0.002, +14.3 | 0.043 | 0.045 | 0.171 | 0.211 | 0.403 / 0.403 | 0 | 88.29 | 0 | 238 (0) |
| run_15_zigzag_s05 | goal | 42.1 | 0.100 | +0.061, +0.062, +0.5 | 0.043 | 0.045 | 0.171 | 0.211 | 0.403 / 0.403 | 0 | 108.21 | 0 | 235 (0) |
| run_16_zigzag_s06 | goal | 42.4 | 0.100 | +0.008, -0.031, -3.9 | 0.042 | 0.044 | 0.171 | 0.207 | 0.411 / 0.411 | 0 | 59.90 | 0 | 238 (0) |
| run_17_zigzag_s07 | goal | 42.2 | 0.100 | +0.025, +0.079, +8.3 | 0.044 | 0.045 | 0.173 | 0.213 | 0.403 / 0.403 | 0 | 79.74 | 0 | 239 (0) |
| run_18_zigzag_s08 | goal | 42.3 | 0.100 | -0.035, +0.097, -5.4 | 0.045 | 0.045 | 0.171 | 0.207 | 0.409 / 0.409 | 0 | 27.28 | 0 | 239 (0) |
| run_19_zigzag_s09 | goal | 42.2 | 0.100 | +0.074, -0.043, +3.1 | 0.044 | 0.045 | 0.174 | 0.213 | 0.404 / 0.404 | 0 | 9.27 | 0 | 237 (0) |

## Wall hits

None.

## Notes

- Cross-track error e comes from the path follower (distance to the active segment). 'Settled' drops the first 3 s, which include recovering from the spawn jitter.
- Wall clearance is from the CoM to the nearest inner wall face in plan view; the body reaches about 0.3 m further. 'Any' includes the back wall, 0.8 m behind the spawn on flat_v1; 'side' is the two long walls only.
- A wall-hit event merges contacts with the same wall less than 0.1 s apart. Hits do not fail a run; a body hit ends the run as 'terminated'.
- Failed MPC solves: acados status other than 0 (converged) or 2 (iteration limit, normal with one SQP iteration per solve). Solve time is wall-clock time of compute_control, compared to the solve interval.
- Slips: the Track C detector also fires on the short slide right after touchdown; 'after touchdown' counts only slips starting more than 50 ms after touchdown.

Files: `summary.csv`, `trajectories.png`, `tracking.png`, per-run `*.png`, `windows/`.
