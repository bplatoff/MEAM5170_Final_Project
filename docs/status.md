# Project status

Where the work stands against the plan (`docs/plan.md`, Development steps). Update this file in the same PR as
the work it describes: change the state, add the evidence (PR, test, results file), and bump "Last updated".

**Last updated:** Nov 27, 2026 (task 2.4, mid-layer robust MPC; tasks 2.3b to 2.3g)  

**States:** Done · In progress · Not started · Blocked (say on what)

## Milestones

| Milestone | Date | State | Notes |
|:--|:--|:--|:--|
| M0 kickoff | Nov 2 | Done | Parameter sheet v0.3, demo runs on every machine |
| M1 stage 1 | Nov 11 | Done | Merged Nov 20 (PR #5); exit test in `trackA/results/phase1/summary.md` |
| M2 stage 2 validated | Nov 27 (moved from Nov 24, E1) | At risk | 2.4 done. Still needs 2.3, 2.5, 2.6 and 2.7, so it cannot be validated by Nov 27. 2.7 also waits on the friction-limit fix (`trackA/plans/friction_limits.md`) |
| M3 stage 3 | Dec 3 | Not started | |
| Code freeze | Dec 5 | | |
| Submission | Dec 10 | | |

## Development steps

| ID | Step | Owner | Planned | State | Evidence / next action |
|:--|:--|:--|:--|:--|:--|
| 0.1 | Shared repo, install, demo | All | Oct 30 to Nov 1 | Done | SETUP.md |
| 0.2 | Read the acados problem | A | Oct 31 to Nov 3 | Done | Hooks found; used in 1.4 |
| 0.3 | Environment generator | C | Oct 31 to Nov 4 | Done, needs migration | `trackC/terrain_gen.py`; move to new frame and friction set (see Migration) |
| 0.4 | Parameter sheet | B | Oct 31 to Nov 2 | Done, needs v0.4 | Sheet v0.3; update for decisions (E2) |
| 1.1 | Quadruped-PyMPC on our course, waypoint reference | A | Nov 3 to 7 | Done | `trackA/sim/run_waypoints.py`, `trackA/reference/path_follower.py` (PR #5) |
| 1.2 | Contact logger and slip detection | C | Nov 4 to 8 | Done | `trackC/contact_logger.py`; open: touchdown false slips |
| 1.3 | Collocation planner | B | Nov 3 to 10 | Done | PRs #1, #2; three circles #7; footprint A* and two-pass tube planning #8 |
| 1.4 | Per-foot friction and CoM half-spaces in acados | A | Nov 7 to 11 | Done | `controllers/centroidal_nmpc_nominal_1_4.py` (`set_friction`, `set_walls`, slope cone), `controllers/test_1_4_hooks.py` |
| 1.5 | Stage 1 exit test, 20 runs | A | Nov 10 to 11 | Done | `trackA/results/phase1/summary.md` |
| 2.1 | Error model, LQR K, ellipsoid, ρ < 1 | B | Nov 10 to 13 | Done | `planning/robust_control/tube_error_model.py` (PR #3) |
| 2.2 | r_w from the shortfall, α_k recursion, radii in meters | B | Nov 12 to 16 | Done | `planning/robust_control/friction_tube.py` (PRs #4, #6); r0 still 0 until 2.5 |
| 2.3 | Mixed courses, batch runner, metrics | C | Nov 10 to 14 | In progress (late) | Breakdown below; 2.5 needs it by Nov 24 |
| 2.4 | Mid-layer robust MPC | A, with B | Nov 12 to 23 | Done | `trackA/mid_layer/` (QP + LQR, three circles; each QP solve starts from the measured state, `--qp-start carried` keeps the nominal instead), `trackA/sim/run_midlayer.py`, `trackA/sim/compare_midlayer.py`; 43 tests (`python -m unittest trackA.mid_layer.test_mid_layer`). No push: 10/10 goals on 1.7 m openings, at most 5.4 cm error, no contact; after the world-frame merge, seed 3 matches (66.6 s, 5.4 cm), worst QP solve 13 ms run alone (budget 100 ms). Sideways pushes: 8/8 goals at 1.0 and 1.25 m/s², but a front leg touched the ramp barrier in 3 of 24 pushed runs (at most 9 mm), and some pushed solves took over 100 ms with 6 runs in parallel; the carried version had no contact but fell in 4/8 at 1.25 m/s². Plan and results: `trackA/plans/mid_lvl_MPC_24.md`, `trackA/results/mid_layer_j9*/`. Next: friction-limit fix and push margin before 2.7 |
| 2.5 | Disturbance check (r_w, lag, time at limit, drift, heading) | B | Nov 17 to 24 | Not started | Needs 2.3 runs |
| 2.6 | End-to-end stage 2 | A | Nov 20 to 25 | Not started | 2.4 done. Needs `run()` to take a start pose and path array and run the mid layer (Jacob, next; replaces Ben's `batch_sim.py` shim), the friction-limit fix, and Ben's 2.3 branch |
| 2.7 | Stage 2 validation, 20 seeds per gap | B | Nov 23 to 27 | Not started | Needs 2.5, 2.6 |
| 3.1 | Emulated probe data | C | Nov 15 to 17 | Not started | |
| 3.2 | GP on equalities | C | Nov 17 to 21 | Not started | |
| 3.3 | Certificate layer, `query(x, y)` | C | Nov 20 to 24 | Not started | Interface and prior-only map done: `trackC/friction_map.py` (0.5 / 0.3) |
| 3.4 | Boundary unit tests | C | Nov 24 to 25 | Not started | |
| 3.5 | μ into r_w and α_k; μ_lo speed limits | B | Nov 23 to 25 | Not started | |
| 3.6 | Between-run loop | A | Nov 25 to Dec 1 | Not started | |
| 3.7 | Stage 3 results | All | Dec 1 to 3 | Not started | |
| SA | Stretch A, probing | A | Dec 1 to 6 | Not started | Only after M2 |
| SB | Stretch B, censored GP | C | Nov 27 to Dec 6 | Not started | Only after M2 |
| 4.1 | Report drafting | B | Nov 24 to Dec 7 | Not started | |
| 4.2 | Final batch runs | C | Dec 4 to 6 | Not started | |
| 4.3 | Figures and videos | B | Dec 3 to 7 | Not started | |
| 4.4 | Assemble and submit | All | Dec 7 to 10 | Not started | |

## Task 2.3 breakdown (Ben)

Done when: one command runs N seeded trials per gap width and outputs outcome, time to goal, clearance, slips and
time at the friction limit (C2).

| Step | Work | Done when | Target | State |
|:--|:--|:--|:--|:--|
| 2.3a | Move `course.py`, `terrain_gen.py`, `flat_v1`, `ramp_v1` to the new frame; spawn at the course start pose | Flat and ramp runs reach the goal; `friction_check` passes | Nov 22 | Done (branch `task-2.3a-world-frame`, in review) |
| 2.3b | Discrete friction set, ramp floor 0.3 | Same seed gives the same course; no value outside the set | Nov 22 | Done (`course_spec.py`, `test_course_spec`) |
| 2.3c | `mixed_v1.yaml` with `gap` and `variant`; generator takes `--gap`, `--variant` | Scenes build for g = 0.6 to 1.1 m, both variants; opening and goal checks pass | Nov 23 | Done (`mixed_v1.yaml`) |
| 2.3d | Start-pose sampler (A1.1) | 1000 draws valid and reproducible | Nov 23 | Done (`trackC/start_pose.py`; zones in every course file; robust plans from 3 sampled starts on mixed_v1) |
| 2.3e | Course YAML to planner converter; two-pass plan per gap; infeasible caught as "no feasible plan" | Every gap gives a path or a clean rejection | Nov 23 | Converter done (`planner_env.py`). Planner: no tube plans gaps ≥ 0.8 m; full tube plans gap 2.0 but A* fails at 1.1 (Austin's A* item) |
| 2.3f | `trackC/metrics.py`: outcome, time to goal (overall, ramp), clearance, slips, time at friction limit; tube drift placeholder | Unit tests on synthetic logs pass | Nov 24 | Done (`trackC/metrics.py`, `test_batch`; slip rule checked on recorded runs). Tube drift waits for the mid layer |
| 2.3g | `python -m trackC.batch`: builds each scene, one subprocess per run, writes `runs.csv` and a summary by controller and gap | 2 gaps × 3 seeds dry run with `nominal`; full-sweep time estimated | Nov 24 | Done with a temporary `run()` shim (`batch_sim.py`); dry run on `mixed_mu03_v1` |
| 2.3h | Controller slots `robust`, `learned`, `oracle` | `robust` runs once 2.4 exists | After 2.4 | `robust_follower` (tube plan, cones 0.3) runs now; `robust`, `learned`, `oracle` wait on 2.4, 3.3 and the `run()` interface |

To agree before 2.3a: `run()` signature (start pose, path array, scene, per-foot μ source) with Jacob; who writes
`predict_contacts` and the "no feasible plan" status with Austin; outcome definitions with both.

## Migration to the Nov 21 plan

Code and documents written before the decisions that still need changing.

| Item | Owner | State |
|:--|:--|:--|
| `trackC/course.py`: new frame | Ben | Done (2.3a) |
| `trackC/terrain_gen.py`: discrete friction set, floor 0.3, 1.0 m obstacle clearance check | Ben | Done (2.3b) |
| Course files: new frame (2.3a) and friction schema (2.3b); new `mixed_v1` (2.3c); `debug_v03` keeps its old layout for the logger tests; Jacob's `midlayer_*` courses converted to the friction schema with a `start_zone` (Nov 27; their scenes are unchanged) | Ben, Jacob | Done |
| `visualize_course.py`, `friction_check.py`, `smoke_test.py`: new frame | Ben | Done (2.3a) |
| `run_waypoints.run()`: absolute start pose and frame-aware drawing and plots (done in 2.3a; merged into `trackA-2-4` Nov 26). To do: start pose and path array as arguments and the mid layer as a controller (Ben's `batch_sim.py` shim until then); per-foot μ from the map. Fall vs collision: Ben's `trackC/metrics.py` (branch `task-2.3bce-friction-mixed`). Terrain bounds: no change needed, gym-quadruped sets no bounds for custom scenes | Jacob | Partly done |
| `robust_two_pass.py`: return "no feasible plan" instead of raising | Austin | Not started |
| `predict_contacts` for the two-pass planner (owner to agree) | Austin or Ben | Not started |
| Parameter sheet v0.4 | Ben | Not started |
| SETUP.md: PR review (D5), branch deletion (D6), `submodule.recurse` (D7), new course commands | Ben | Not started |
| `trackC/README.md`: new frame | Ben | Done (2.3a) |
