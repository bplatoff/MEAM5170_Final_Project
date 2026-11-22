# Task 2.4 plan: mid-layer tube MPC and LQR feedback

*Updated Nov 22, 2026, against `master` at `ca7889a`. Owners: Jacob, with Austin.*

## Team summary

**Status:** Jacob can build and test the mid layer on today's `master` without waiting on anyone. A full ramp-and-obstacle test run already reaches the goal using temporary patches.

**Needed from Austin and Ben by Wed Nov 25,** so validation can finish before M2 on Nov 27.

### TODO by person

**Jacob (starting now, needs nothing from anyone)**
1. Build a temporary planner adapter and a wide-gap test course (one 0.483 m circle plus tube)
2. Run step tests to measure the velocity lag `τ` and gain `K_sys`
3. Build LQR tracking, then add the nominal QP
4. Integrate into `run_waypoints` and run full courses
5. Inject worst-case pushes, check the robot stays in the tube, and save logs for task 2.5

**Austin (by Nov 25)**
1. Fix the A* seed: it rejects robust gaps of 1.2 m or less that the NLP can solve (seeding pass 2 from pass 1 worked for 1.05 to 1.2 m)
2. Return "no feasible plan" instead of raising an exception
3. Make plans valid between samples (they currently cut into the inflated constraints by 0.8 to 2.9 cm)
4. Keep `h` a multiple of 0.1 s and choose `T` by searching for the shortest feasible time
5. Start `yaw_ref` at the measured yaw and limit its rate to 0.5 rad/s
6. Add the friction reserve `δ`, uphill and turn limits, and heading-error inflation
7. Run the disturbance check (2.5) on Jacob's logs

**Ben (by Nov 25)**
1. Move courses to the new frame and discrete friction set (2.3a, 2.3b)
2. Commit `ramp_v1` and build `mixed_v1` with `gap` and `variant` (2.3c)
3. Add the start-pose sampler and YAML-to-planner converter (2.3d, 2.3e)
4. Add metrics and the batch runner's `robust` slot (2.3f to 2.3h)
5. Add a prior-only `query(x, y)` that returns `(0.5, 0.3)`

### Decisions by person

| Who | Decision | Recommendation (used until decided) |
|---|---|---|
| **Jacob + Austin** | Replace B3's `v_cmd = v_meas + dt·a_cmd`? | Yes, if the step test confirms the lag. In simulation, slip error was 79 cm with B3 and 10.7 cm with the lag-based conversion |
| **Jacob + Austin** | Keep the gait-cycle filter on `v_meas`? | No, it adds about 0.36 s of delay. Use raw, or a 1.4 Hz notch if needed |
| **Jacob + Austin** | Acceleration limits | `‖a‖ ≤ (μ_plan − δ)g`, with turn and uphill limits from `μ_lo` |
| **Jacob + Austin** | Heading-error allowance `ε_ψ` | 0.10 rad until 2.5 measures it |
| **Austin + Ben** | Who writes `predict_contacts`? | Austin. A constant version is enough for stage 2 |
| **Jacob + Ben** | `run()` interface for the batch runner | Start pose, path array, scene, per-foot `μ` source |
| **All** | Start and goal distance from walls | Raise from 0.5 m to **0.75 m**. The robust planner cannot start or finish at 0.5 m |

### After Austin and Ben finish (Jacob)

1. Switch to Austin's three-circle, two-pass planner and `yaw_ref`
2. Switch to Ben's courses, frame, converter, and batch runner
3. Apply the decisions above and rerun the mid-layer tests
4. Run end-to-end stage 2 (2.6), then validate with Austin (2.7)

---

# Detailed plan

## 1. What Jacob can do now

### Temporary patches

These use the current repo and do not edit Austin's or Ben's code.

| Missing today | Patch for now | Replaced by |
|---|---|---|
| Working robust three-circle plan | `footprint_model="single_circle"`, `r_safety=0.10` (radius 0.483 m) | Austin's two-pass planner |
| Two-pass tube inflation | One pass with `tube_position_margin_m = 0.263`, because the tube is constant before learning | `plan_two_pass_friction_tube` |
| `predict_contacts` | Compute the constant tube directly with `compute_friction_tube_position_radii` | Shared predictor |
| `yaw_ref` in single-circle mode | Heading between planner knots, indexed by nominal time, held on steps under 2 cm, limited to 0.5 rad/s | Austin's `yaw_ref` |
| Acceleration limits | `env.friction_mu = 0.12`, which limits `‖a_ref‖` to 1.18 m/s² and covers the ramp | Planner limits |
| Time search | `T` = route length / 0.30 m/s, rounded up to a multiple of 0.4 s; add 4 s and retry on failure | Austin's search |
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

One circle plus the tube needs an opening of `2(0.483 + 0.263) = 1.49 m`. So this course **cannot test the real 0.6 to 1.1 m gaps**. Those wait for Austin's three-circle planner.

### Jacob's work order

| Step | Target | Work | Done when |
|---|---|---|---|
| **J1. Adapter and course** | Nov 22 | Course file and `planner_adapter.py` with the patches above | Plan solves and passes the 0.02 s check |
| **J2. Step test** | Nov 22 | Bypass `PathFollower`. Step `v_x`, `v_y`, and `ω` at several sizes, with cones at 0.5 and 0.3, and fit over whole gait cycles | `τ` and `K_sys` repeat across runs |
| **J3. Core and unit tests** | Nov 23 | Reference, tube, LQR, conversion, yaw loop | Unit tests pass |
| **J4. LQR tracking** | Nov 23 | QP off, nominal equals planner reference | 10 of 10 seeded full runs reach the goal with no contact |
| **J5. Nominal QP** | Nov 23 to 24 | Offline against saved plans, then in the loop | Returns a valid reference to within 1 mm, p99 under 50 ms, no slack above 1 mm, no worse than J4 |
| **J6. Disturbance test** | Nov 24 | Apply a horizontal force of `m × 1.96 m/s²` to the base in timed windows | `‖e‖_P ≤ α`, no contact, logs saved for 2.5 |
| **J7. Comparison** | Nov 24 | `PathFollower`, LQR, and LQR + QP on the same seeds | Summary table in `trackA/results/` |

**Fallback:** if time runs short, J4 (planner plus LQR) is a complete controller. When the planner's reference is valid, it has the same stage 2 guarantee as the QP version.

### What cannot be finished without teammates

- Passing the real 0.6 to 1.1 m gaps
- Final courses, batch runs, and metrics
- Task 2.5 sign-off on `r_w`, `r_0`, and the opening widths
- Changing closed team decisions

## 2. Design

```
Global planner      safe x_ref, a_ref; obstacles inflated by the largest tube
      │
      v
Nominal QP          every 0.10 s: plans the nominal robot's next 2 s
      │ x̄, ā
      v
LQR + conversion    every 0.02 s: corrects the real robot's error from the nominal
      │ v_cmd, ω_cmd (world frame)
      v
Quadruped-PyMPC     footholds and forces, friction cones at μ_lo
```

The safety argument:

1. LQR keeps the real robot within the tube radius of the nominal.
2. The QP keeps the nominal far enough from obstacles that the whole tube is clear.
3. So the real robot does not collide.

This holds only if true friction is at least `μ_lo`, disturbances stay within `r_w`, and Quadruped-PyMPC delivers the commanded motion. Task 2.5 checks those conditions.

If the planner's reference satisfies every constraint, the QP returns that reference and LQR does the active correction. The QP matters when the reference needs repair on the finer grid, and later when friction or the tube changes during a run.

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

| Symbol | Now (one circle) | After Austin (three circles) |
|---|---|---|
| Circle offsets `s_m` | `{0}` | `{-0.233, 0, +0.233}` m |
| Circle radius plus margin | 0.483 m | `0.194 + 0.05` m |
| Tube radius | 0.263 m | From the route |
| Minimum opening | 1.49 m | About 1.06 m with the heading term |

| Symbol | Value | Source |
|---|---|---|
| `K` per axis | `[-18.45, -7.63]` | `design_tube_error_model(0.02)` |
| `ρ` | 0.974 | Same |
| `μ_plan`, `μ_lo`, `δ` | 0.5, 0.3, 0.05 | B2, B4 |
| `τ_v`, `K_sys,v` | 0.10 s, 0.87 | *Provisional until J2* |
| `τ_ω`, `K_sys,ω` | 0.05 s, 0.95 | *Provisional until J2* |
| `τ_eff` | 0.110 s | `Δt / (1 − exp(−Δt/τ_v))` |
| `v_max` | 0.5 m/s | Must match the planner |
| `ω_max`, `k_ψ` | 0.6 rad/s, 1.5 1/s | `PathFollower` |
| `ε_ψ` | 0.10 rad | *Placeholder*, unused with one circle |

### Nominal QP

```
Decide:  x̄_1..x̄_N,  ā_0..ā_{N-1},  s_1..s_N ≥ 0

min   Σ_{k=0}^{N-1} [ ‖x̄_k − x_ref,k‖²_Q + ‖ā_k − a_ref,k‖²_R ]  +  Σ_{k=1}^{N} (w₁ s_k + w₂ s_k²)

s.t.  x̄_0 = carried nominal state          (measured pose on the first solve)
      x̄_{k+1} = A x̄_k + B ā_k              k = 0..N-1
      x̄_N = x_ref,N

      dᵢᵀ ā_k ≤ (μ_plan − δ)g               acceleration, 16 directions
      dᵢᵀ v̄_k ≤ v_max                       speed, 16 directions
      |n_kᵀ ā_k| ≤ (μ_lo − δ)g              turn limit, when ‖v_ref,k‖ > 0.05
      ā_x,k ≤ (μ_lo cos θ_k − sin θ_k)g     uphill limit, on the ramp

      For each circle m and k = 1..N:
      walls:      c̄_m,k stays inside the walls by R_m,k − s_k
      obstacles:  n_jᵀ c̄_m,k ≥ b_j + R_m,k − s_k    for the chosen face of each nearby obstacle
```

```
A = [I₂  0.1·I₂]    B = [0.005·I₂]    Q = diag(400, 400, 25, 25)    R = I₂
    [0   I₂    ]        [0.1·I₂  ]

c̄_m,k = p̄_k + s_m [cos ψ_ref,k, sin ψ_ref,k]ᵀ
R_m,k = circle radius + margin + r_tube,k + |s_m|·ε_ψ
dᵢ    = [cos(iπ/8), sin(iπ/8)]ᵀ,   i = 0..15
n_k   = unit normal to v_ref,k
```

- **Carried nominal start:** measurement noise cannot make the QP infeasible.
- **Terminal equality:** the planner's trajectory is the safe continuation past the horizon.
- **Time-indexed reference:** the nominal cannot fall behind. LQR handles the real robot's lag.
- **16-sided limits outside the planner's circles:** the reference always stays feasible.
- **Slack:** keeps the solver returning an answer. Any `s_k > 1 mm` is logged, and **the guarantee is void for that solve**.

**Face selection.** Staying outside an obstacle is an OR across its faces. Before each solve, shift the previous nominal forward one stage. For each circle, nearby obstacle, and interval `[k, k+1]`, pick the face with the best smaller margin at the two endpoints, and impose it at **both** endpoints. Only include obstacles within 2 m of the predicted trajectory.

**Solver.** OSQP through CasADi `conic`, with warm starts. It is already in the pixi environment.

### Tube

Propagate on the 0.02 s grid with Austin's functions:

```
r_w,i   = c_P · Δa_i + r_0,       Δa = (μ_plan − μ_lo)g = 1.96 m/s² before learning
α_{i+1} = ρ·α_i + r_w,i
r_tube  = p_norm_to_position_radius(α, P)      [m]
```

Start at `α_0 = ‖x_meas(0) − x̄(0)‖_P`. QP stage `k` uses the largest radius in the 5 LQR steps before it. Raise an error if a radius exceeds the planner's margin. `α` is in P-norm units and must never be used directly as meters.

### LQR and command conversion

Every 0.02 s:

```
e     = x_meas − x̄(t)
a_cmd = ā(t) + K e
v_cmd = (τ_eff · a_cmd + v_meas) / K_sys,v
ω_cmd = clip( ω_ref(t)/K_sys,ω + k_ψ · wrap(ψ_ref(t) − ψ_meas),  ±ω_max )
```

`K` already contains the minus sign, so the error must be **measured minus nominal, at the current time**. Keep B3's conversion behind a config flag until the team decides.

### Control loop

```python
every sim step:
    if step % 50 == 0:  solve QP from x̄; store ā_0 (on failure, shift the previous plan)
    if step % 10 == 0:
        v_cmd, ω_cmd = feedback(x_meas, x̄, ā)    # held until the next update
        x̄ = A_fb x̄ + B_fb ā                      # 0.02 s model
        α = ρ α + r_w
    send held v_cmd, ω_cmd to Quadruped-PyMPC
```

After the plan ends, hold the nominal at the goal with `ā = 0`. Report `done` within 0.1 m of the goal.

| Event | Response | Logged as |
|---|---|---|
| QP fails or exceeds 100 ms | Keep the previous plan, shifted one stage | `qp_fallback` |
| Slack above 1 mm | Continue, mark the guarantee void | `qp_slack_max` |
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
| **Austin's planner fixes land** | Replace the adapter's patches with `plan_two_pass_friction_tube` and `yaw_ref`; turn on three circles | J4 to J6 pass on a gap near 1.1 m |
| **Ben's frame and courses land** | Use his converter, start poses, and `mixed_v1` | Same seeds still reach the goal |
| **Decisions made** | Set the conversion, filter, limits, and `ε_ψ` flags | J6 rerun |
| **Batch runner lands** | Connect the `robust` controller slot | 2 gaps × 3 seeds dry run |
| **Task 2.5 results** | Update `r_0`, the tube, and opening widths | Measured errors stay within `r_w` |
| **All of the above** | End-to-end stage 2 (2.6), then validation with Austin (2.7) | 20 seeds per gap, no collisions, deviation inside the tube |

**After M2 only:** compare a measured-state MPC (`x̄_0 = x_meas`, LQR cost-to-go as terminal cost) on low-friction patches and narrow gaps. It may help when constraints are active, but its guarantee is harder to prove.

## 4. Repo status

**On `master` from Austin:** LQR error model; friction tube with radii in meters; three-circle footprint; `yaw_ref` in three-circle mode; footprint-aware A*; two-pass planner.

**On other branches:** none of Austin's branches have unmerged commits. The only remote branch ahead of `master` is Ben's `docs/claude-context` (one commit, docs only). Local work and PRs from forks are not visible here.

## 5. Evidence and limits

All checked on `master` at `ca7889a`.

- **Full run with today's code:** planner with one 0.483 m circle plus the 0.263 m tube, followed by `PathFollower`, with cones at 0.3, on a draft of the test course. The robot **reached the goal in 43.2 s** with no wall or obstacle contact, 4.6 cm maximum cross-track error, and no failed Quadruped-PyMPC solves. The mid layer itself is not built yet.
- **A\* gaps:** on a 0.3 m barrier attached to one side wall, A* failed at every tested robust gap of 1.2 m or less. The NLP solved 1.05, 1.1, and 1.2 m with a usable seed, and reported 0.95 and 1.02 m infeasible.
- **Wall clearance:** with the robust margin, the three-circle A* failed for a start 0.5 m from a wall and for a goal 0.61 m from a wall. It succeeded at 0.75 m.
- **Between samples:** master's three-circle plans violated the inflated constraints by 0.8 cm at walking speed and 2.9 cm at the default `T = 12`, checked every 0.02 s with interpolated yaw.
- **QP prototype** (`Δt = 0.1`, `N = 20`, 30 s plan): 320 solves, none failed. Median 4.3 ms, p99 29 ms, worst 212 ms, so the fallback is required. It omitted the turn and uphill limits and the two-endpoint face rule.
- **Conversion comparison (79 cm vs 10.7 cm)** uses an idealized double integrator with first-order lag and the provisional `τ = 0.1`, `K_sys = 0.87`. The step test decides whether B3 changes.
