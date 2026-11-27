# Friction limits: low-level pyramid and mid-layer limits

*Nov 26, 2026. Not started. Owner: Jacob. Separate branch and PR, after the 2.4 merge and PR. Affects Austin (uphill rule, tube) and Ben (shared low-level controller).*

## Why

1. **The low-level pyramid allows too much friction.** `create_friction_cone_constraints` in `controllers/centroidal_nmpc_nominal_1_4.py` limits the uphill and sideways foot forces separately, so diagonally it allows √2·μ (0.42 at μ = 0.3). The simulated ground is a true circle (`go2.xml` uses `cone="elliptic"`), so those forces slip. This breaks decision B4 ("feet don't slip").
2. **The mid layer's 16-sided polygons sit outside their circles.** Their faces touch the circle, so the corners allow 1.9% more.
3. **Two mid-layer limits skip the LQR reserve δ.** The overall and turn limits use μ − δ. The slope friction polygon (measured start) and the uphill limit (both start modes) use the full μ_lo. In the J9 contact runs, the QP sat at the slope limit, and with the LQR correction on top, the total command reached 3 to 5 m/s² against about 2.9 m/s² available.

## Changes

Numbers are for θ = 10°, μ_lo = 0.3, δ = 0.05.

| Limit | Where | Now | New |
|---|---|---|---|
| Low-level pyramid, per foot | `create_friction_cone_constraints` | 4 faces at μ: 0.3 straight, up to 0.42 diagonal | 8 faces with corners on the circle, faces aligned with uphill and sideways: 0.277 to 0.3 (at least 0.924·μ in every direction). 9 rows per leg instead of 5 |
| Overall acceleration, 16 sides | `nominal_qp.py`, `cfg.accel_limit` | Faces at 4.41 m/s² | Faces at 4.41·cos(π/16) = 4.33 m/s² |
| Slope friction polygon (measured start only) | `nominal_qp.py`, `r_fric` rows | Radius μ_lo·g·cos θ = 2.90 m/s² | Radius (μ_lo − δ)·g·cos θ·cos(π/16) = 2.37 m/s² |
| Uphill limit (both start modes) | `cfg.uphill_limit` | (μ_lo cos θ − sin θ)·g = 1.19 m/s² | The slope polygon's uphill face: 2.37 − g sin θ = 0.67 m/s² |
| Planner acceleration limit | `planner_adapter.planner_friction` | min(μ_lo − δ, μ_lo cos θ − sin θ)·g = 1.19 m/s² | Same helper as the uphill limit, 0.67 m/s², so the reference never asks for more than the QP allows |
| Turn limit | `cfg.turn_limit` | Slab, (μ_lo − δ)·g = 2.45 m/s² | Unchanged: a slab has no corners to fit inside a circle |
| Speed polygon | `nominal_qp.py`, `r_speed` | Faces at v_max | Unchanged: not a friction limit |

What this does:
- **The low level's uphill acceleration** on the 10° ramp falls from 1.19 to 0.97 m/s². The QP asks for at most 0.67, which leaves room for the LQR.
- **The measured-start QP's sideways limit** while holding on the slope falls from about 2.4 to 1.65 m/s².
- **Today's plans should not change.** Their reference accelerations are at most 0.024 m/s², far below either planner limit. Code step 4 checks this.
- **Pyramid orientation.** With faces aligned to uphill and sideways, straight uphill gets 0.277. Rotating the pyramid to put a corner uphill would give 0.3 straight uphill, but only about 0.26 once a 1 m/s² push adds sideways demand, which is the case being fixed.

## Code steps

1. **Low level:** for each foot, 8 face rows (cos φ_k·t + sin φ_k·b − μ cos(π/8)·n)·f ≤ 0 for φ_k = kπ/4, plus the normal-force row. Upstream sizes its bounds from the arrays this function returns, and the wall rows start after `nh_upstream`, so both follow the new row count. Confirm nothing else assumes 20 rows, then regenerate the acados solvers.
2. **`controllers/test_1_4_hooks.py`:** its friction check measures max(|f_t|, |f_b|)/f_n, the old square. Check the 8 faces instead, and add a diagonal-demand case that must stay within ‖f_tan‖ ≤ μ·f_n.
3. **Mid layer:** add one helper in `config.py` for the slope radius, and use it for the slope polygon, the uphill limit and `planner_friction`. Scale the overall acceleration bound by cos(π/16).
4. **Tests:**
   - Update `test_mid_layer.py` where it assumes the old values: the uphill limit test, the slope test's `/ cos(π/16)` bound, and `polygon_corner`.
   - Add checks that every polygon corner is on or inside its circle, and that the planner limit is at most the QP's uphill limit.
   - Confirm with the plan check script that the plans are unchanged.
5. **Batch runner (`compare_midlayer.py`):**
   - Record the git commit, and whether the tree had uncommitted changes, in `manifest.json` and `summary.md`.
   - Add a seed-list option, so runs 0 and 2 below cover only the seeds that matter.
   - Do not `--resume` a batch folder from before this change. The plan cache key comes from config fields, and this change adds none, so old plans would be reused.
6. **Run every test:** mid layer (43), path follower (17), 1.4 hooks. **Commit the code before any batch.**

## Runs

All runs use the 1.7 m course unless noted, with 6 workers. Each new batch folder replans every seed before running.

| Step | What | Runs | Time | Pass |
|---|---|---|---|---|
| 0 | **Reproduction check,** before the change, on the merged world-frame branch. Both start modes: seed 6 at 1.0 m/s², and seeds 3, 4, 5 and 7 at 1.25 m/s² (every contact and every carried fall). Without the seed-list option, run seeds 3 to 7 at both pushes instead (20 runs, about 1 h) | 10 | ~30 min | Same falls and contacts as the old-frame batches, with errors within about 2 cm. If so, the Nov 24 to 26 batches are the baseline. If not, rerun the baseline pushed batch in the world frame (about 2 h) |
| 1 | **Ramp climb, no push:** `PathFollower`, carried and measured, 3 seeds each | 9 | ~30 min | All reach the goal, with times within 5% of before (42.2 s follower, 65.7 s mid layer). If not, stop: a limit is too tight |
| 2 | **Quick read:** step 0's runs with the change | 10 | ~30 min | Check contacts, falls and sideways movement toward the barrier before spending 2 h |
| 3 | **Full batch:** both start modes, seeds 0 to 7, at 1.0 and 1.25 m/s² on the 1.7 m course and 1.0 m/s² on the 1.25 m course, plus 3 no-push seeds per mode | 54 | 2 to 2.5 h | See below |

If step 2 changes the outcome, an optional repeat of it with only the low-level change (mid-layer limits as before) shows which part made the difference. It takes about 30 min.

**Report** goals, falls, runs with contact, the worst sideways movement toward the barrier, runs outside the tube, backward slides on the ramp, QP slack and solve time, low-level solve time and failures, and time to goal.

**"Improved"** means fewer runs with a contact or a fall, no new contact or fall at 1.0 m/s², and no change without pushes. With 8 seeds per case, a difference of one or two runs is noise, so weigh the sideways movement and slides as well as the counts.

Then commit the results, a results section in this plan and `docs/status.md`, and open the PR. It holds the code commit and the results commit.

## Next: push margin (measured start)

- **Reserve a margin m of the slope polygon's sideways reach** for disturbances, starting with m = 0.5 m/s². Shrinking the whole polygon does not work: its uphill face (0.67 m/s²) would turn negative for m > 0.67 and force the robot downhill.
- **Why 0.5:** after this plan, the low level can give about 2.1 to 2.35 m/s² sideways while holding on the slope. A 1.0 m/s² push leaves 1.1 to 1.35, and the QP plans with 1.65, so m = 0.5 brings the QP to 1.15.
- **The real push is unknown,** so m is a fixed reserve like δ. It mainly affects measured start: the carried QP's limits rarely bind, and its LQR has no friction limit.
- **Runs:** step 2's quick read, then the full pushed batch for measured start only, about 1 h.

## Team decisions needed before the PR

1. **Uphill rule (Austin).** `docs/plan.md` sets a_up ≤ (μ_lo cos θ − sin θ)·g, and Austin's TODO 8 adds it to the planner. This plan adds δ and the 16-gon factor, giving 0.67 m/s² at 10°. Both controllers should follow the same rule. Otherwise Austin's planner may ask for up to 1.19 m/s² uphill, more than the QP allows.
2. **Tube size (Austin, task 2.2).** The tube assumes the low level delivers μ_lo = 0.3 in every direction, but the 8-face pyramid only guarantees 0.277. Sized for 0.277, the tube grows from 26.3 to 29.3 cm, and the smallest passable opening rises by about 6 cm, to roughly 1.12 m. Recommendation: keep 0.3 for M2, record the gap, and let task 2.5's measured drift decide.
3. **Shared low level (Ben).** The path follower and Ben's batch runner use the same low-level controller, so their results change too. Ben's ramp test stalled at μ = 0.2; step 1 rechecks the climb.

In the PR, update `docs/decisions.md` (a new entry for the 8-face pyramid, the uphill rule with δ and the tube choice), `docs/plan.md` (the uphill rule and the wording of B4) and `docs/status.md`.
