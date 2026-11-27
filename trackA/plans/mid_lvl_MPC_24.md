# Task 2.4 plan: mid-layer tube MPC and LQR feedback

*Updated Nov 26, 2026 (J9 results), on a branch still based on `master` at `ca7889a`. Owners: Jacob, with Austin.*

## Team summary

**Status (Nov 23):** The three-circle mid layer (QP + LQR) runs full ramp-and-obstacle courses on today's `master`, using temporary patches. Without pushes it reached the goal in **10 of 10** runs with 1.7 m openings and **5 of 5** with 1.25 m openings, with at most **6.3 cm** tracking error and no contact.

**Main open problem:** the tube assumes the robot can reject a 1.96 m/s² shortfall. In simulation it handled sideways pushes of **1.0 m/s² (8 of 8 runs on each course, inside the tube)**, but **fell in 4 of 8 runs at 1.25 m/s²** (three circles, carried-nominal QP).

**Footprint (Jacob's decision):** the mid-layer MPC and its planner call now use **three circles by default**, and Austin's planner default should match. With the full tube, this lowers the smallest passable opening from 1.49 m to about **1.06 m**. Narrower gaps should return "no feasible plan" unless task 2.5 shrinks the tube.

**MPC design (Jacob's decision):** the target MPC starts from the **measured state** and grows the tube from zero at every solve. Obstacles are **soft constraints with a large linear penalty**, so the robot still gets a recovery command after a shove beyond the tube. **It is the default since Nov 26 (Jacob's decision).** It passed the no-push checks and never fell at 1.25 m/s² (8 of 8 vs 4 of 8 for the carried-nominal QP), but it did worse at 1.0 m/s² (worst error 75 cm vs 11 cm) and a leg touched the ramp barrier in 3 pushed runs (two of them on seeds where the carried-nominal QP fell). The carried-nominal QP, the only version with no contact in any run, stays available with `--qp-start carried`. The friction-limit fix and a push margin target the contacts before 2.7.

**Needed from Austin and Ben by Wed Nov 25,** so validation can finish before M2 on Nov 27.

### TODO by person

**Jacob**
1. ~~Temporary planner adapter and wide-gap test course~~ Done
2. ~~Step tests for velocity lag `τ` and gain `K`~~ Done
3. ~~LQR tracking and nominal QP~~ Done
4. ~~`run_waypoints` integration and full-course runs~~ Done
5. ~~Push tests and logs for task 2.5~~ Done
6. ~~Switch the MPC and planner call to three circles (J8)~~ Done
7. ~~Measured-state MPC comparison (J9)~~ Done Nov 26: fails "no worse than carried" at 1.0 m/s², but never falls; Jacob made it the default anyway (Nov 26). Cross-track weighting is dropped as a fix for the contacts: in those runs the QP was already pushing sideways at its friction limit (section 0)
8. **Merge `master` (world frame, 2.3a) into `trackA-2-4`** and open the PR
9. **Friction limits** (plan: `trackA/plans/friction_limits.md`): replace the low-level pyramid, which allows √2·μ diagonally (Ben, Nov 25), with 8 faces (at least 0.924·μ in every direction); fit the mid layer's polygons inside their circles; subtract δ from the slope and uphill limits. Then try a push margin for the measured-start QP
10. Improve recovery on the ramp, then integrate teammates' work when it lands

**Austin (by Nov 25)**
1. **Make three circles the default:** change `footprint_model` to `"three_circle"` in `plan_trajectory` and the `test_planner.py` CLI, and keep `"single_circle"` as an explicit option. All 56 planning tests still pass with the `plan_trajectory` default changed
2. Fix the A* seed: it rejects robust gaps of 1.2 m or less that the NLP can solve (seeding pass 2 from pass 1 worked for 1.05 to 1.2 m). Also, A* turns in place, so a 1.25 m opening directly beside a side wall failed: the front circle at 45° crossed the wall bound by about 5 mm
3. Return "no feasible plan" instead of raising an exception
4. Make plans valid between samples (they currently cut into the inflated constraints by 0.8 to 2.9 cm)
5. Keep `h` a multiple of 0.1 s and choose `T` by searching for the shortest feasible time
6. **Add a per-section speed limit:** at `μ_lo = 0.3` the robot saturates near 0.24 m/s on the ramp, so plans should use about 0.18 m/s there and can go faster on the flats
7. **Make the rate-limited `yaw_ref` the default:** start at the measured yaw and limit its rate to 0.5 rad/s (`footprint_yaw` and `footprint_max_yaw_rate` already exist)
8. Add the friction reserve `δ`, uphill and turn limits, and heading-error inflation. The uphill limit should include `δ` (`trackA/plans/friction_limits.md`, team decision 1)
9. Run the disturbance check (2.5) on Jacob's logs

**Ben (by Nov 25)**
1. Move courses to the new frame and discrete friction set (2.3a, 2.3b)
2. Commit `ramp_v1` and build `mixed_v1` with `gap` and `variant` (2.3c)
3. Add the start-pose sampler and YAML-to-planner converter (2.3d, 2.3e)
4. Add metrics and the batch runner's `robust` slot (2.3f to 2.3h)
5. Add a prior-only `query(x, y)` that returns `(0.5, 0.3)`

### Decisions by person

| Who | Decision | Recommendation (used until decided) |
|---|---|---|
| **Jacob + Austin** | Replace B3's `v_cmd = v_meas + dt·a_cmd`? | Yes; the team agreed Nov 26 and B3 is revised in `docs/decisions.md`. In full runs, error was 2.9 cm with the measured-lag conversion and 19.5 cm with B3. At a 1.0 m/s² push: 22.6 cm (inside the tube) vs 47 cm (outside). Caveat: B3 did not fall at 1.25 m/s², but left the tube by up to 2.4× |
| **Jacob + Austin** | What disturbance does the tube claim to cover? | Keep 1.96 m/s² for planning for now, but state that only 1.0 m/s² is demonstrated until recovery improves or 2.5 shows the real shortfall is smaller |
| **Jacob + Austin** | Keep the gait-cycle filter on `v_meas`? | No, it adds about 0.36 s of delay. Full runs used raw `v_meas` and tracked to 2.9 cm without pushes |
| **Jacob + Austin** | Acceleration limits | `‖a‖ ≤ (μ_plan − δ)g`, with turn and uphill limits from `μ_lo` |
| **Jacob + Austin** | Heading-error allowance `ε_ψ` | 0.10 rad until 2.5 measures it |
| **Jacob + Austin** | What safety claim does the measured-state MPC support? | Claim no collision only when tube and body slack are zero and the disturbance is within the tube's bound. Log any slack as outside the guarantee |
| **Austin + Ben** | Who writes `predict_contacts`? | Austin. A constant version is enough for stage 2 |
| **Jacob + Ben** | `run()` interface for the batch runner | Start pose, path array, scene, per-foot `μ` source |
| **All** | Start and goal distance from walls | Raise from 0.5 m to **0.75 m**. The robust planner cannot start or finish at 0.5 m |

### After Austin and Ben finish (Jacob)

1. Replace the adapter's explicit three-circle settings with Austin's three-circle default, two-pass planner, and `yaw_ref`
2. Switch to Ben's courses, frame, converter, and batch runner
3. Apply the decisions above and rerun the mid-layer tests
4. Run end-to-end stage 2 (2.6), then validate with Austin (2.7)

---

# Detailed plan

## 0. Implementation status (Nov 26)

Built without changes to Track B or Track C code. J8 and J9 are committed and pushed on `trackA-2-4` at `6c15d04` (the one-circle version is `8b0ea34`), and are not merged to `master`. Merged with `master` (world frame) on Nov 26. The default is three circles with the measured-state QP (since Nov 26). Run the carried-nominal version with `--qp-start carried`.

| Piece | File |
|---|---|
| Mid-layer controller, QP, LQR, tube, reference, planner adapter | `trackA/mid_layer/` |
| One full run | `python -m trackA.sim.run_midlayer --controller tube` |
| Batch comparison | `python -m trackA.sim.compare_midlayer` |
| Step test and ramp speed test | `trackA/sim/step_test.py`, `trackA/sim/ramp_speed_test.py` |
| Test courses | `trackA/courses/` |
| Results | `trackA/results/mid_layer/summary.md`, `trackA/results/step_test/` |
| Runner hooks (default behavior unchanged) | `trackA/sim/run_waypoints.py` |

**Measured response of Quadruped-PyMPC** (`v̇ = (K·v_cmd − v)/τ`, cones at 0.3):

| Axis | `τ` | `K` |
|---|---|---|
| Forward, flat | 0.085 s | 0.885 |
| Lateral, flat | 0.10 s | 0.707 (0.62 at 0.1 m/s to 0.77 at 0.3 m/s) |
| Yaw rate | At or below the 0.02 s fit resolution | 0.867 |
| Forward, 10° ramp | Not fitted (flat value used) | 0.62, saturating near 0.24 m/s |

**One-circle baseline** on `midlayer_test_v1` (1.7 m openings, one circle plus tube, QP started from the carried nominal):

| Controller | Push | Reached goal | Worst error | Inside tube |
|---|---|---|---|---|
| LQR | None | 10/10 | 2.9 cm | 10/10 |
| LQR + QP | None | 10/10 | 2.9 cm | 10/10 |
| LQR, B3 conversion | None | 5/5 | 19.5 cm | 5/5 |
| LQR | 1.0 m/s² | 5/5 | 22.6 cm | 5/5 |
| LQR + QP | 1.0 m/s² | 5/5 | 22.6 cm | 5/5 |
| LQR, B3 conversion | 1.0 m/s² | 5/5 | 47.0 cm | 0/5 |
| LQR | 1.25 m/s² | **3/5** | 143 cm | 1/5 |
| LQR + QP | 1.25 m/s² | **3/5** | 150 cm | 1/5 |
| LQR, B3 conversion | 1.25 m/s² | 5/5 | 63.6 cm | 0/5 |

- The QP never failed or used slack above 1 mm in 12,804 solves. Its worst solve took 10.5 ms.
- **On a valid plan the QP matches LQR**, as expected, because the carried-nominal QP starts from the nominal state and never sees a push.
- Both falls happened 2 to 4 s after the ramp-barrier push began, on the 0.3 friction patch. Scaling LQR commands to the friction budget made recovery worse, so that change was removed.
- The `PathFollower` baseline reached the goal 10/10 without pushes, in 42.5 s versus 66.9 s for the mid layer. The mid layer is slower because one speed limit, set by the ramp, currently applies to the whole course.

**J8 and J9 results** (three circles, cones at 0.3, same seeds and plans for both QPs; full tables in `trackA/results/mid_layer_j9/summary.md`):

| Opening | Push | Carried QP: goal, worst error | Measured QP: goal, worst error |
|---|---|---|---|
| 1.7 m | None | 10/10, 6.3 cm | 10/10, 5.4 cm |
| 1.7 m | 1.0 m/s² | 5/5, 11.0 cm | 5/5, **71.3 cm** |
| 1.7 m | 1.25 m/s² | 3/5, 57.9 cm | 3/5, 75.2 cm |
| 1.25 m | None | 5/5, 5.8 cm | 5/5, 4.6 cm |
| 1.25 m | 1.0 m/s² | 5/5, 23.6 cm | 5/5, **30.1 cm** |

- **J8 passes.** All no-push runs had no contact, the QP returned the planner's reference at every controller update, and all 15 plans clear the inflated three-circle constraints by at least 0.43 cm on a 0.02 s check.
- **J9 does not pass.** It solved every shoved state (no failures in 19,185 solves), matched the hard QP in unit tests, and used no slack without pushes. It fails "no worse than carried": at 1.0 m/s² it has one 71 cm excursion on the 1.7 m course and is worse on 3 of 5 seeds on the 1.25 m course. At 1.25 m/s², both versions reach 3 of 5 goals, but different seeds fall.
- **Cause still open.** The largest measured-state errors occur at the ramp barrier, often with the QP at its 1.19 m/s² uphill limit. Raising that limit to what LQR alone would command did not fix it: of the five worst runs, one improved and two became much worse (one fell, one reached 212 cm). That change was removed.
- **Gain match:** the measured-state QP uses `Q ≈ diag(1033, 1033, 64.6, 64.6)` so its first-step gain equals the LQR gain. With `diag(400, 400, 25, 25)` it was 27% weaker. OSQP uses `eps_rel = 1e-6` and `rho = 0.01`, so large linear penalties do not loosen its stopping test.

**J9 with the slope friction limit (Nov 24 to 26).** The measured-state QP now limits `‖(a_x + g sin θ, a_y)‖ ≤ μ_lo g cos θ`, so uphill and sideways acceleration share one friction budget. Pushed runs only, original seeds 0 to 4 plus fresh seeds 5 to 7; full tables in `trackA/results/mid_layer_j9_friction/summary.md` and `trackA/results/mid_layer_j9_friction_fresh/summary.md`:

| Opening | Push | Carried QP: goal, worst error, runs outside tube, runs with contact | Measured QP: goal, worst error, runs outside tube, runs with contact |
|---|---|---|---|
| 1.7 m | 1.0 m/s² | 8/8, 11.0 cm, 0, 0 | 8/8, **75.0 cm**, 1, **1** |
| 1.7 m | 1.25 m/s² | **4/8** (4 falls), 57.9 cm, 7, 0 | **8/8**, 64.0 cm, 4, **2** |
| 1.25 m | 1.0 m/s² | 8/8, 23.6 cm, 0, 0 | 8/8, 22.0 cm, 0, 0 |

- **Measured never fell.** At 1.25 m/s² it reached 8 of 8 goals; the carried QP fell in 4 of 8. Both fresh-seed falls were on the 0.3 ramp patch, just before the mid-ramp barrier.
- **It still fails "no worse than carried".** At 1.0 m/s² on the 1.7 m course, fresh seed 6 left the tube (75 cm) and the front-right leg briefly touched the barrier (foot and calf, 9.4 mm). At 1.25 m/s², original seed 4 and fresh seed 5 also touched the barrier with the front-right leg. The carried QP had no contact in any run.
- **Slack and solve time.** In pushed runs on the 1.7 m course the measured QP used tube, body and speed slack (up to 9.8 cm of body slack), and 28 to 32 solves per group took over 100 ms (worst 694 ms). The carried QP never used slack and never exceeded 85 ms. Times were measured with 6 runs in parallel, so absolute values are inflated, but both versions ran under the same load.
- **What the contacts were (logs, Nov 26):** all three are the same event. The body was pushed 34 to 39 cm toward the barrier, past the 26.3 cm tube, and the front-right foot, about 21 cm to the body's right, touched the barrier's inner face (1 to 9 mm, at most 0.22 s). Two of the three are on seeds where the carried-nominal QP fell, so at 1.25 m/s² measured start mostly survived a slide that knocked the carried version over.
- **Why (partly known):**
  - During the push, the QP was at its friction limit and mostly pushing sideways; in both 1.25 m/s² runs, entirely sideways.
  - Its friction limit ignores the push, so it planned about 2.4 m/s² of sideways recovery when the feet could give about 1.35.
  - Fresh seed 6 at 1.0 m/s² is the only measured-only failure. The robot stumbled and backed down the ramp at up to 0.54 m/s, while the QP kept spending friction sideways and the velocity command followed the slide (−0.44 m/s). Why it stumbled there and the carried version did not is unknown, from one run.
  - Earlier offline re-solves of two runs without contact found the QP trading sideways push-back for along-track catch-up. That explains their larger error at 1.0 m/s², not the contacts.
  - Solve-time overruns are not a cause, because the simulation waits for each solve.
- **Decision (Jacob, Nov 26):** the measured-state QP is the default. It never fell, while the carried-nominal QP fell in 4 of 8 runs at 1.25 m/s². The cost is the contacts above: 2.7 requires no collisions, and at 1.0 m/s² the carried-nominal QP is the only version without contact. Cross-track weighting is dropped as a contact fix. Next comes the friction-limit fix (`trackA/plans/friction_limits.md`), then a push margin for the measured-state QP.

**Next for Jacob:** merge `master` (world frame) and open the PR. Then, both before 2.7 runs: the friction-limit fix (`trackA/plans/friction_limits.md`), and a start pose, path array and the mid-layer controller in `run_waypoints.run()`, so Ben's batch runner can drop its shim and run the `robust` controller. Then end-to-end stage 2 (2.6).

## 1. What Jacob can do now

### Temporary patches

These use the current repo and do not edit Austin's or Ben's code.

| Missing today | Patch for now | Replaced by |
|---|---|---|
| Robust three-circle plan | The adapter passes the three-circle settings explicitly (J8) | Austin's three-circle default and two-pass planner |
| Two-pass tube inflation | One pass with `tube_position_margin_m = 0.263`, because the tube is constant before learning | `plan_two_pass_friction_tube` |
| `predict_contacts` | Compute the constant tube directly with `compute_friction_tube_position_radii` | Shared predictor |
| Rate-limited `yaw_ref` | The adapter passes the measured start yaw and a 0.5 rad/s limit, then uses the planner's returned `yaw_ref` (J8) | Austin's default |
| Acceleration limits | `env.friction_mu = 0.12`, which limits `‖a_ref‖` to 1.18 m/s² and covers the ramp | Planner limits |
| Time search | `T` = route length / 0.30 m/s (0.15 m/s with a ramp), rounded up to a multiple of 0.4 s; add 4 s and retry on failure | Austin's search |
| Per-section speed limit | One limit for the whole course: 0.18 m/s if it has a ramp, otherwise 0.5 m/s | Austin's limits |
| Between-sample validity | Check the plan every 0.02 s; if it violates, raise `r_safety` by the shortfall plus 1 cm and replan | Planner check |
| "No feasible plan" | Catch planner exceptions in the adapter and return a status | Planner status |
| YAML-to-planner converter | Read bounds and obstacles from the course file, in whatever frame it uses | Ben's converter |
| Friction map | Constant `μ_plan = 0.5`, `μ_lo = 0.3` | `query(x, y)` |
| Course | Jacob's own test course (below) | `mixed_v1` |

### Test course

`trackA/courses/midlayer_test_v1.yaml`, using the existing course format and frame:

- 3 m flat, 6 m ramp at 10°, 3 m flat, 3 m wide
- Base `μ = 0.5`, with 0.3 patches at the gateway and mid-ramp barrier, and one 0.8 patch
- Gateway, mid-ramp barrier, and top passage, each with a **1.7 m opening**
- Start `(0, 0)`, goal `(10.2, 0)`

One circle plus the tube needs an opening of `2(0.483 + 0.263) = 1.49 m`. Three circles need `2(0.194 + 0.05 + 0.263 + 0.023) = 1.06 m`. This course **does not test that limit yet**. Openings of 1.2 m or less also wait for Austin's A* fix, and openings below about 1.06 m should return "no feasible plan" at the full tube.

### Jacob's work order

| Step | Target | Work | Done when |
|---|---|---|---|
| **J1. Adapter and course** | Nov 22 | Course file and `planner_adapter.py` with the patches above | Plan solves and passes the 0.02 s check |
| **J2. Step test** | Nov 22 | Bypass `PathFollower`. Step `v_x`, `v_y`, and `ω` at several sizes, with cones at 0.5 and 0.3, and fit over whole gait cycles | `τ` and `K_sys` repeat across runs |
| **J3. Core and unit tests** | Nov 23 | Reference, tube, LQR, conversion, yaw loop | Unit tests pass |
| **J4. LQR tracking** | Nov 23 | QP off, nominal equals planner reference | 10 of 10 seeded full runs reach the goal with no contact |
| **J5. Nominal QP** | Nov 23 to 24 | Offline against saved plans, then in the loop | Returns a valid reference to within 1 mm, p99 under 50 ms, no slack above 1 mm, no worse than J4 |
| **J6. Disturbance test** | Nov 24 | Apply a horizontal force of `m × 1.96 m/s²` to the base in timed windows | `‖e‖_P ≤ α`, no contact, logs saved for 2.5. **Not met yet:** passes at 1.0 m/s², falls in 2 of 5 runs at 1.25 m/s² |
| **J7. Comparison** | Nov 24 | `PathFollower`, LQR, and LQR + QP on the same seeds | Summary table in `trackA/results/` |
| **J8. Three circles** | Nov 23 | Switch the planner call and QP to three circles, then rerun J4 to J7 | 10 of 10 no-push runs with no contact; QP returns the valid reference; all three circles clear the inflated constraints on a 0.02 s check. **Passed Nov 23** |
| **J9. Measured-state MPC** | Nov 24 | Start the QP from `x_meas`, restart the tube from zero at each solve, and use soft obstacle and speed constraints with a terminal cost. Compare with the carried-nominal QP on the same seeds | Solves from shoved states; matches the hard QP within 1 mm when that QP is feasible; no slack and 10 of 10 goals without pushes; no worse than the carried-nominal QP at 1.0 and 1.25 m/s². **Not met:** the last check fails (section 0) |

**Fallback (in use since Nov 23):** if J9 is not ready or does worse, keep the carried-nominal QP that is already built. If time runs shorter, J4 (planner plus LQR) is a complete controller, and on a valid reference it has the same stage 2 guarantee as the carried-nominal QP.

### Three-circle footprint (J8)

**Decision:** the planner and MPC both use three circles by default. One circle stays available as an explicit option for comparison.

```
s_m   = {-L/3, 0, +L/3} = {-0.233, 0, +0.233} m
r     = hypot(L/6, W/2) + r_safety = 0.194 + 0.05 = 0.244 m
c̄_m,k = p̄_k + s_m [cos ψ_ref,k, sin ψ_ref,k]ᵀ
R_m,k = r + r_tube,k + |s_m|·ε_ψ
```

The MPC stays a QP because `ψ_ref` is known before the solve, so each circle center is linear in `p̄_k`. The planner and MPC must use the same `ψ_ref`.

| File | Change |
|---|---|
| `planner_adapter.py` | Pass `footprint_model="three_circle"`, `footprint_yaw_mode="path_tangent"`, `footprint_yaw=<measured yaw>`, `footprint_max_yaw_rate=0.5`, and `r_safety=0.05`. Use `tube_position_margin_m = r_tube + 0.233·ε_ψ` until Austin adds the heading term. Use the returned `yaw_ref`, and check all three circles every 0.02 s |
| `controller.py`, `nominal_qp.py` | Pass the three offsets and the radius to the QP. Add `abs(s_m)·ε_ψ` to each circle's wall and obstacle margin |
| `config.py` | Add the footprint model and `ε_ψ`, and use `r_safety = 0.05` for three circles |
| `test_mid_layer.py` | Test circle placement, margins at several yaws, and that the QP returns a valid three-circle plan |

Test on the 1.7 m course first, then on a 1.25 m version. Openings of 1.2 m or less need Austin's A* fix.

**Checked Nov 22:** with these settings, `master`'s planner solved `midlayer_test_v1` (`T = 68 s`) with 1.2 cm of slack to the inflated three-circle constraints on a 0.02 s grid and a peak yaw rate of 0.30 rad/s. This was a planner-only check. The three-circle MPC has since run in simulation (section 0).

### Measured-state MPC (J9)

**Decision:** the MPC starts from the measured state, so it can plan a recovery around obstacles after a shove, even when the robot has left the tube. The formulation is in section 2. **Status:** the default since Nov 26, although it failed J9's push comparison (section 0); `--qp-start carried` selects the carried-nominal QP.

| File | Change |
|---|---|
| `nominal_qp.py` | Start from `x_meas`. Replace the single slack with tube, body, and speed slacks. Replace the terminal equality with terminal cost `P_N`. Choose obstacle faces from a rollout of `x_meas` |
| `tube.py` | Add stage radii that start from `α = 0` at each solve |
| `controller.py` | After a successful solve, set `x̄ = x_meas` and restart the tube monitor. After a failed solve, keep the previous plan without resetting. Log all three slacks |
| `config.py` | Add `qp_start` (`measured` or `carried`), `P_N`, and the slack weights |
| `run_midlayer.py`, `compare_midlayer.py` | Add `--qp-start` and run both versions on the same seeds |
| `test_mid_layer.py` | Test that the QP solves from shoved states, soft matches hard when hard is feasible, tube slack is used before body slack, and the tube restarts from zero |

### What cannot be finished without teammates

- Passing openings of 1.2 m or less, until Austin's A* fix lands
- Final courses, batch runs, and metrics
- Task 2.5 sign-off on `r_w`, `r_0`, and the opening widths
- Changing closed team decisions

## 2. Design

```
Global planner      safe x_ref, a_ref; hard obstacle constraints inflated by the full tube
      │
      v
MPC (QP)            every 0.10 s: plans the next 2 s from the measured state
      │ x̄, ā
      v
LQR + conversion    every 0.02 s: corrects the real robot's error from the MPC's prediction
      │ v_cmd, ω_cmd (world frame)
      v
Quadruped-PyMPC     footholds and forces, friction cones at μ_lo
```

The safety argument:

1. The planner's reference clears every obstacle by the full tube, using hard constraints.
2. At each solve, the MPC starts from the measured state and keeps its tube, which grows from zero over the horizon, clear of obstacles whenever it can.
3. LQR keeps the real robot near the MPC's prediction between solves.
4. If no tube or body slack is used and the disturbance stays within `r_w`, the real robot does not collide.
5. If a shove makes that impossible, the MPC still returns the least-bad recovery, giving up the tube margin first and the body margin last. The guarantee does not apply to that solve.

This holds only if true friction is at least `μ_lo`, disturbances stay within `r_w`, and Quadruped-PyMPC delivers the commanded motion. Task 2.5 checks those conditions. A formal feasibility proof also needs speed and acceleration limits tightened for the LQR correction. **That derivation is not done.**

With no active constraints, the MPC should behave much like LQR. It matters near obstacles and limits, where it can plan a recovery that LQR cannot.

### Interfaces

**From the planner:** `t`, `x = [p_x, p_y, v_x, v_y]`, piecewise-constant `u`, `ψ_ref`, tube margin, and a status.

**Measured:** `base_pos`, `base_lin_vel`, yaw, all world frame. `base_pos` is within 0.5 cm of the CoM horizontally in our logs.

**To Quadruped-PyMPC:** the same return format as `PathFollower.update()`:

```python
ref_base_lin_vel, ref_base_ang_vel, done = mid_layer.update(base_pos[:2], base_lin_vel[:2], yaw, sim_dt)
```

Use the `perfoot` MPC with `set_friction(μ_lo)`. The tube is applied in the mid layer, so `controllers/tube_nmpc_2_4.py` is not part of this design.

### Timing

| Loop | Step | Rate |
|---|---|---|
| Simulation and leg control | 0.002 s | 500 Hz |
| Quadruped-PyMPC | 0.004 s | 250 Hz |
| **LQR, yaw, command** | **0.02 s** (every 10 sim steps) | **50 Hz** |
| **Nominal QP** | **0.10 s** (every 50 sim steps), `N = 20` | **10 Hz**, 2 s horizon |

### Parameters

| Symbol | One circle (explicit option) | Three circles (default) |
|---|---|---|
| Circle offsets `s_m` | `{0}` | `{-0.233, 0, +0.233}` m |
| Circle radius plus margin | 0.483 m | `0.194 + 0.05` m |
| Planner tube radius | 0.263 m | 0.263 m, then from the route after Austin's two-pass planner |
| Minimum opening | 1.49 m | About 1.06 m with the heading term |

| Symbol | Value | Source |
|---|---|---|
| `K` per axis | `[-18.45, -7.63]` | `design_tube_error_model(0.02)` |
| `ρ` | 0.974 | Same |
| `μ_plan`, `μ_lo`, `δ` | 0.5, 0.3, 0.05 | B2, B4 |
| `τ`, `K` forward (flat) | 0.085 s, 0.885 | Step test |
| `τ`, `K` lateral | 0.10 s, 0.707 | Step test |
| `K` forward (ramp) | 0.62 | Ramp speed test |
| `K` yaw rate | 0.867 | Step test |
| `τ_eff` | `Δt / (1 − exp(−Δt/τ))` per axis | Exact inverse over a held step |
| `v_max` | 0.5 m/s flat, 0.18 m/s with a ramp | Ramp speed test |
| Command limits | 0.8 m/s forward, 0.45 m/s on the ramp, 0.4 m/s lateral | Measured response |
| `ω_max`, `k_ψ` | 0.6 rad/s, 1.5 1/s | `PathFollower` |
| `ε_ψ` | 0.10 rad | *Placeholder* until 2.5 measures it. Used from J8 |
| `P_N` | Riccati solution for the QP's `A`, `B`, `Q`, `R` at 0.1 s | Terminal cost (J9) |
| Tube slack `ℓ_t`, `q_t` | `10⁴`, `10²` | *Starting value*, same as today's slack |
| Body slack `ℓ_b`, `q_b` | `10⁵`, `10³` | *Starting value*. Needs `ℓ_b > ℓ_t + 2 q_t r_tube,max` so tube slack is used first |
| Speed slack `ℓ_v`, `q_v` | `10³`, `10²` | *Starting value* |

### MPC (target design, J9)

*The default since Nov 26 (`qp_start="measured"`). `qp_start="carried"` keeps one slack and a terminal equality.*

```
Decide:  x̄_1..x̄_N,  ā_0..ā_{N-1},  s_t,k,  s_b,k,  s_v,k

min   Σ_{k=0}^{N-1} [ ‖x̄_k − x_ref,k‖²_Q + ‖ā_k − a_ref,k‖²_R ]  +  ‖x̄_N − x_ref,N‖²_{P_N}
      + Σ_{k=1}^{N} [ ℓ_t s_t,k + q_t s_t,k²  +  ℓ_b s_b,k + q_b s_b,k²  +  ℓ_v s_v,k + q_v s_v,k² ]

s.t.  x̄_0 = x_meas
      x̄_{k+1} = A x̄_k + B ā_k              k = 0..N-1

      Hard:
      dᵢᵀ ā_k ≤ (μ_plan − δ)g               acceleration, 16 directions
      |n_kᵀ ā_k| ≤ (μ_lo − δ)g              turn limit, when ‖v_ref,k‖ > 0.05
      ā_x,k ≤ (μ_lo cos θ_k − sin θ_k)g     uphill limit, on the ramp

      Soft:
      dᵢᵀ v̄_k ≤ v_max + s_v,k               speed, 16 directions

      For each circle m and k = 1..N:
      walls:      c̄_m,k stays inside the walls by R_body,m + r_tube,k − s_t,k − s_b,k
      obstacles:  n_jᵀ c̄_m,k ≥ b_j + R_body,m + r_tube,k − s_t,k − s_b,k
                  for the chosen face of each nearby obstacle

      0 ≤ s_t,k ≤ r_tube,k,   s_b,k ≥ 0,   s_v,k ≥ 0
```

```
A = [I₂  0.1·I₂]    B = [0.005·I₂]    Q ≈ diag(1033, 1033, 64.6, 64.6)    R = I₂
    [0   I₂    ]        [0.1·I₂  ]

c̄_m,k = p̄_k + s_m [cos ψ_ref,k, sin ψ_ref,k]ᵀ
R_body,m = circle radius + margin + |s_m|·ε_ψ
dᵢ    = [cos(iπ/8), sin(iπ/8)]ᵀ,   i = 0..15
n_k   = unit normal to v_ref,k
```

- **Measured start:** the MPC plans a recovery from where the robot really is, including outside the tube.
- **Always solvable:** only acceleration limits are hard, and `ā = 0` satisfies them. With enough slack, that is always a valid solution.
- **Linear penalty:** if `ℓ` exceeds the hard constraint's Lagrange multiplier, the slack is exactly zero and the QP matches the hard-constrained answer whenever that answer exists.
- **Quadratic penalty:** when a violation is unavoidable, it spreads the violation instead of concentrating it at one stage.
- **Two penalty levels:** the MPC gives up the tube margin first and enters the body margin only if it has no better option.
- **Terminal cost:** pulls the plan back to the reference without making a large shove infeasible, as a terminal equality could.
- **Time-indexed reference:** the reference does not wait for the robot. The MPC and LQR recover any lag.
- **16-sided limits outside the planner's circles:** the reference always satisfies the MPC's limits.
- **Slack logging:** any `s_t` or `s_b` above 1 mm is logged, and **the guarantee does not apply to that solve**.

**Face selection.** Staying outside an obstacle is an OR across its faces. Before each solve, roll `x_meas` forward with LQR tracking the reference. For each circle, nearby obstacle, and interval `[k, k+1]`, pick the face with the best smaller margin at the two endpoints, and impose it at **both** endpoints. Only include obstacles within 2 m of the predicted trajectory.

**Solver.** OSQP through CasADi `conic`, with warm starts. It is already in the pixi environment.

### Tube

**The planner** uses the constant full tube (0.263 m) with hard constraints, because it plans once before the run.

**The MPC** restarts the tube from zero at every solve, because the state is measured at `k = 0` and uncertainty only builds over the prediction. Propagate on the 0.02 s grid with Austin's functions:

```
r_w,i     = c_P · Δa_i + r_0,       Δa = (μ_plan − μ_lo)g = 1.96 m/s² before learning
α_{0|t}   = 0
α_{i+1|t} = ρ·α_{i|t} + r_w,i
r_tube    = p_norm_to_position_radius(α, P)      [m]
```

| Prediction time | 0.1 s | 0.2 s | 0.5 s | 1 s | 2 s |
|---|---|---|---|---|---|
| MPC tube radius | 3.2 cm | 6.1 cm | 12.7 cm | 19.3 cm | 24.4 cm |

Starting from `α = 0` assumes the measured state is exact, which holds in simulation. With measurement error, start from a bound on that error. QP stage `k` uses the largest radius in the 5 LQR steps before it. Raise an error if a radius exceeds the planner's margin. `α` is in P-norm units and must never be used directly as meters.

### LQR and command conversion

Every 0.02 s:

```
e     = x_meas − x̄(t)
a_cmd = ā(t) + K e
v_cmd,i = (τ_eff,i · a_cmd,i + v_meas,i) / K_i       for i = forward, lateral (body frame)
ω_cmd = clip( ω_ref(t)/K_ω + k_ψ · wrap(ψ_ref(t) − ψ_meas),  ±ω_max )
```

With the measured-state MPC, `e` resets to zero at each successful QP solve, and LQR corrects what builds up over the next 0.1 s. `K` in the LQR line already contains the minus sign, so the error must be **measured minus nominal, at the current time**. The conversion works in the body frame because the robot responds differently forward and sideways, then rotates back to the world frame. B3's conversion stays available with `--conversion b3`.

### Control loop

```python
every sim step:
    if step % 50 == 0:
        solve QP from x_meas
        on success: x̄ = x_meas; α = 0; store ā_0
        on failure: keep x̄ and α; shift the previous plan
    if step % 10 == 0:
        v_cmd, ω_cmd = feedback(x_meas, x̄, ā)    # held until the next update
        x̄ = A_fb x̄ + B_fb ā                      # 0.02 s model
        α = ρ α + r_w
    send held v_cmd, ω_cmd to Quadruped-PyMPC
```

After the plan ends, hold the nominal at the goal with `ā = 0`. Report `done` within 0.1 m of the goal.

| Event | Response | Logged as |
|---|---|---|
| QP fails | Keep the previous plan, shifted one stage | `mid_qp_fallback` |
| QP exceeds 100 ms | Use the result (simulation waits for it), and log it | `mid_qp_overrun` |
| Tube or body slack above 1 mm | Continue with the least-bad recovery, mark the guarantee void for that solve | `mid_qp_slack_tube`, `mid_qp_slack_body` |
| Speed slack above 1 mm | Continue | `mid_qp_slack_speed` |
| `‖e‖_P > α` | Continue, counts against the stage 2 exit | `tube_violation` |
| `v_cmd` clipped | Continue | `cmd_saturated` |
| Planner infeasible | Do not run | `no feasible plan` |

### Code layout

```
trackA/courses/midlayer_test_v1.yaml
trackA/mid_layer/
    config.py            Section 2 values and flags
    planner_adapter.py   Temporary patches; later calls Austin's two-pass planner
    reference.py         x, a, yaw, yaw rate at time t
    tube.py              Wraps Austin's tube functions
    nominal_qp.py        Sparse QP, face selection, warm start
    feedback.py          LQR, velocity conversion, yaw loop
    controller.py        TubeMidLayer.update()
    test_*.py            Unit tests, run from the repo root
trackA/sim/step_test.py  Identifies τ and K_sys
```

In `trackA/sim/run_waypoints.py`, add a controller option, a planner path source, and a timed push. Existing behavior and log keys stay unchanged, and new log keys are only added. Do not edit `Quadruped-PyMPC/`.

## 3. After teammates finish

| Trigger | Jacob's next step | Check |
|---|---|---|
| **Austin's planner fixes land** | Replace the adapter's patches, including its explicit three-circle settings, with Austin's default and `plan_two_pass_friction_tube` | J4 to J6 pass on a gap near 1.1 m |
| **Ben's frame and courses land** | Use his converter, start poses, and `mixed_v1` | Same seeds still reach the goal |
| **Decisions made** | Set the conversion, filter, limits, and `ε_ψ` flags | J6 rerun |
| **Batch runner lands** | Connect the `robust` controller slot | 2 gaps × 3 seeds dry run |
| **Task 2.5 results** | Update `r_0`, the tube, and opening widths | Measured errors stay within `r_w` |
| **All of the above** | End-to-end stage 2 (2.6), then validation with Austin (2.7) | 20 seeds per gap, no collisions, deviation inside the tube |

**Stage 2 uses the measured-state QP (Jacob, Nov 26),** although J9 found it worse at 1.0 m/s² pushes and the only version with contact. The friction-limit fix and a push margin (`trackA/plans/friction_limits.md`) are meant to remove the contacts before 2.7; the carried-nominal QP stays available as the fallback.

## 4. Repo status

**On `master` from Austin:** LQR error model; friction tube with radii in meters; three-circle footprint (optional, the default is still one circle); `yaw_ref` in three-circle mode; footprint-aware A*; two-pass planner.

**On other branches:** none of Austin's branches have unmerged commits. The only remote branch ahead of `master` is Ben's `docs/claude-context` (one commit, docs only). Local work and PRs from forks are not visible here.

## 5. Evidence and limits

All checked on `master` at `ca7889a`.

- **Full run with today's code:** planner with one 0.483 m circle plus the 0.263 m tube, followed by `PathFollower`, with cones at 0.3, on a draft of the test course. The robot **reached the goal in 43.2 s** with no wall or obstacle contact, 4.6 cm maximum cross-track error, and no failed Quadruped-PyMPC solves. This check was run before the mid layer was built.
- **A\* gaps:** on a 0.3 m barrier attached to one side wall, A* failed at every tested robust gap of 1.2 m or less. The NLP solved 1.05, 1.1, and 1.2 m with a usable seed, and reported 0.95 and 1.02 m infeasible.
- **Wall clearance:** with the robust margin, the three-circle A* failed for a start 0.5 m from a wall and for a goal 0.61 m from a wall. It succeeded at 0.75 m.
- **Between samples:** master's three-circle plans violated the inflated constraints by 0.8 cm at walking speed and 2.9 cm at the default `T = 12`, checked every 0.02 s with interpolated yaw.
- **QP prototype** (`Δt = 0.1`, `N = 20`, 30 s plan): 320 solves, none failed. Median 4.3 ms, p99 29 ms, worst 212 ms, so the fallback is required. It omitted the turn and uphill limits and the two-endpoint face rule.
- **Conversion comparison:** the MuJoCo full-course results in section 0 replace the earlier idealized estimate.
- **Why obstacles stay soft in J9 (calculation, not simulation):** with the growing tube, a robot braking at the QP's 4.41 m/s² limit needs 1.0, 4.0, and 7.3 cm beyond the body margin when moving toward an obstacle at 0, 0.3, and 0.5 m/s. At 2.45 m/s² it needs 2.0, 7.2, and 12.6 cm. Closer than that, a hard constraint has no solution. These are best cases that assume room to move directly away.
- **Limits of the full-course results:** they use test courses with 1.7 m and 1.25 m openings and external pushes as a stand-in for friction shortfall. They do not validate the real 0.6 to 1.1 m gaps or the final courses.
