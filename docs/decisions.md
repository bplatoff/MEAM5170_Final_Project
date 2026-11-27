# Team decisions

Snapshot of the "MEAM 5170 Final Project Team Decisions" Google Sheet (shared drive), taken Nov 21, 2026.
All items are closed. The sheet is the source of truth; if it changes, update this file in the same PR as the code.
The plan (`docs/plan.md`) already reflects every decision below.

## A. Frame, course and planner

| # | Decision | Owner |
|:--|:--|:--|
| A1 | World frame: origin at the back-right inner wall corner on the lower-flat surface; x forward along the course, y left, z up. All data shared between modules uses this frame; the body frame is used only inside the controller. Start, goal and obstacles come from the course YAML; Ben writes the YAML-to-planner converter. | All |
| A1.1 | Start pose: seeded uniform draw from a box on the lower flat, at least 0.5 m from walls and 1.0 m from obstacles, heading within ±15°. The start seed is separate from the friction seed. | Ben |
| A2.1 | Footprint: three circles, adopted before M2. | Austin, Jacob |
| A2.3 | Planner: plan, propagate the tube along the path, replan with inflation = three-circle footprint + 0.05 m margin + largest tube radius on the route. Mid layer: inflates each predicted step k by footprint + margin + its own α_k, propagated from the current width with r_w from μ_lo at that step's footholds; α_k never exceeds the planner's inflation. Before learning both are equal (constant tube, after about 2 s of growth from zero). Walls are treated the same way. | Austin, Jacob |
| A2.4 | Collisions: actual robot contact with a wall or obstacle. Minimum clearance: bare footprint (no margin, no tube) to the nearest obstacle or wall, at the logged position and yaw. | Ben |
| A3.1 | Gateway at the bottom of the ramp and passage at the top, at least 1.2 m. Mid-ramp barrier attached to the right wall (y = 0) leaving gap g to the left wall; g swept by the batch runner, about 0.6 to 1.1 m in 0.1 m steps. Two variants: 0.3 patch or high patch (0.7 or 0.8) around the gap. Provisional minimum gaps: robust about 1.0 m, learned about 0.6 to 0.75 m. Below its minimum the planner finds no route, recorded as "no feasible plan". | All |
| A3.3 | One fixed goal per course, at least 1.0 m from obstacles and 0.5 m from walls. | All |
| A3.4 | Opening widths provisional; locked after task 2.5 measures the real tube. | All |
| A3.5 | Effectiveness: on the same seeds and both variants, (1) pass rate vs gap width and the smallest gap cleared in at least 80% of runs, (2) time to goal on successful runs, overall and on the ramp. The adaptive controller uses learned μ_lo in the planner (tube from the learned map) and the mid layer (speed, turn-rate and acceleration limits: v²κ ≤ (μ_lo − reserve)g, uphill (μ_lo cos θ − sin θ)g, capped by Quadruped-PyMPC's max walking speed). With μ_lo = 0.3 the same rules give the robust baseline. | All |
| A5 | Planner acceleration limit uses μ = 0.5 (robust baseline); the adaptive planner uses the learned map. | Austin |
| A6 | Global planner gives the mid layer desired position and velocity; the mid-layer robust MPC outputs desired velocity and yaw rate to Quadruped-PyMPC. | Austin, Jacob |

## B. Tube and friction

| # | Decision | Owner |
|:--|:--|:--|
| B1 | The tube function returns a position radius in meters per horizon step. Austin owns the unit conversions. | Austin, Ben |
| B2 | The certificate layer exposes `query(x, y) -> (mu_plan, mu_lo)`. Before learning: mu_plan = 0.5, mu_lo = 0.3 everywhere. Disagreement threshold ε below 0.1 (e.g. 0.05). μ_plan can move both ways, μ_lo ≤ μ_plan, μ_plan rises above 0.5 only where μ_lo has risen. Further learning choices after the robust parts are complete. | Ben, Austin |
| B3 | Feedback in the mid layer: a_cmd = a_nom + K (x_meas − x_nom), K from Austin's LQR (already includes the minus sign, A_cl = A + BK), computed once offline. Command (revised Nov 26, team): invert Quadruped-PyMPC's measured first-order velocity lag per body axis, v_cmd = (τ_eff · a_cmd + v_meas) / k with τ_eff = dt / (1 − e^(−dt/τ)), τ and k from step tests (lower forward gain on the ramp), then clip and rotate to the world frame. It replaces v_cmd = v_meas + dt · a_cmd (full runs: 2.9 cm vs 19.5 cm worst error), which stays available as `--conversion b3`. Checks: velocity-tracking time constant (recompute K and ρ if much longer than dt); low-pass v_meas over a gait cycle; bounded heading error; world-frame commands. | Jacob, Austin |
| B4 | Mid layer plans with mu_plan = 0.5; Quadruped-PyMPC cones use mu_lo per foot, so feet don't slip. Tube width α = r_w / (1 − ρ), r_w covering the shortfall (0.5 − mu_lo)·g plus tracking error. Assumes the low level is rarely at its friction limit; 2.5 checks time at the limit and drift against α. | All |

## C. Data, courses and results

| # | Decision | Owner |
|:--|:--|:--|
| C1 | Jacob's `run_waypoints` log is the standard: CoM, orientation, foot forces, MPC forces and contacts at the simulation rate. | Jacob, Austin, Ben |
| C2 | Batch runner (2.3), built on Jacob's runner. Inputs: course, variant, gap g (value or list), friction seed, start seed, controller, path source. Outputs per run and summary: outcome (success, collision, fall, timeout, no feasible plan), time to goal (overall and ramp), collisions, clearance, slips, fraction of time at the friction limit, drift against the tube. Summaries by controller and gap width. Builds the course scene itself. | Ben |
| C3 | Friction values from {0.3, 0.4, 0.5, 0.6, 0.7, 0.8}; nominal 0.5; lower limit 0.3 everywhere (ramp tests: planning for 0.3 climbs in 40 s, 0.2 stalls, planning for 0.5 on a 0.3 patch fell). 0.3 patches on the ramp, at turns and near obstacles; high patches 0.7 or 0.8. Seeds choose discrete values within each class. | All |
| C4 | Small summaries and plots in `results/` in the repo; logs and datasets on the shared Drive. | All |

## D. Repo conventions

| # | Decision |
|:--|:--|
| D1 | Package-style imports from the repo root; run everything from the root with `python -m`. |
| D2 | Project environment only (conda on macOS, pixi on Linux and WSL), never the system Python. |
| D3 | No generated files in git: no `__pycache__`, `c_generated_code`, logs or data. |
| D4 | `planning/robust_control/` stays where it is. |
| D5 | PR review: fetch the PR, check it merges cleanly, run the tests it touches plus the existing ones. Reported results come only from committed code. |
| D6 | Delete branches after their PR is merged. |
| D7 | `git config --global submodule.recurse true` on every machine. |

## E. Schedule

| # | Decision |
|:--|:--|
| E1 | M2 moved to Nov 27. |
| E2 | Plan and parameter sheet updated once every item is closed (done for the plan Nov 21). |
