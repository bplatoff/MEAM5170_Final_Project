# MEAM 5170 final project: robust and adaptive MPC for a quadruped

A simulated Unitree Go2 in MuJoCo walks from a start pose to a goal over a 10° ramp course with friction patches
and obstacles. Stage 1 is nominal MPC, stage 2 adds a robust tube MPC, stage 3 learns friction to shrink the tube.
Due Dec 10, 2026. Team: Jacob (track A, sim and MPC), Austin (track B, planning and tube math), Ben (track C,
environment, data and learning).

- Full plan: `docs/plan.md` (exported from the docx on the shared drive; read the relevant section before design work)
- Closed decisions: @docs/decisions.md
- Where the work stands: @docs/status.md

## Architecture

1. **Global planner** (`planning/`): direct collocation on a point-mass model, three-circle footprint, two passes
   (plan, propagate the tube, replan with footprint + 0.05 m margin + largest tube radius). Outputs desired
   position and velocity.
2. **Mid-layer robust MPC** (task 2.4, not built yet): point mass, about 2 s horizon, keeps its nominal trajectory,
   LQR feedback a_cmd = a_nom + K (x_meas − x_nom), sends v_cmd = v_meas + dt · a_cmd and yaw rate.
3. **Low level**: Quadruped-PyMPC (`controllers/centroidal_nmpc_nominal_1_4.py` adds per-foot friction, wall
   half-spaces and slope-aware cones). Friction cones use μ_lo per foot.

## Fixed numbers and conventions

- World frame: origin at the back-right inner wall corner on the lower flat; x forward, y left, z up. All data
  between modules is in this frame. Course is 3 m wide; walls 0.5 m high.
- Friction values come from {0.3, 0.4, 0.5, 0.6, 0.7, 0.8}. Nominal 0.5, lower limit 0.3 everywhere. The classes and
  checks live in `trackC/course_spec.py`; every module loads courses through `load_course_cfg`.
- Before learning: μ_plan = 0.5, μ_lo = 0.3 everywhere (`trackC.friction_map.PriorFrictionMap`), so the tube has
  constant width α = r_w / (1 − ρ).
- Footprint: three circles of radius 0.194 m at 0 and ±0.233 m along the body axis. With margin, tube and heading
  allowance each needs 0.53 m to a wall, so the CoM needs 0.763 m to the back and end walls (`trackC/start_pose.py`).
- Start poses: `trackC.start_pose.sample_start(cfg, start_seed)`, uniform over the course's `start_zone`, seed separate
  from the friction seed (A1.1).
- Tube radii are in meters (B1). K is computed once, offline; its sign is built in (A_cl = A + BK).
- Evaluation: 20 seeds per controller per mid-ramp gap width (g from about 0.6 to 1.1 m).

## Rules

- Run everything from the repo root with `python -m package.module`; imports are package style
  (`from planning.direct_collocation import plan_trajectory`). Every code folder has an `__init__.py`.
- Use the project environment only: conda `qpympc` on macOS, pixi `humble-no-cuda` on Linux and WSL. Never the
  system Python. On macOS, scripts that open the MuJoCo viewer run with `mjpython -m`.
- Never edit files inside `Quadruped-PyMPC/` (a pinned submodule). Change behavior from our own modules.
- Do not commit generated files: `__pycache__`, `c_generated_code`, logs, `.npz`, `data/`. Run logs go to `data/`
  or the shared drive; small summaries and plots go to `results/`.
- New tests go next to the code as `test_*.py` and must run from the root.
- Results we report come only from committed code.
- When a task step changes state, update `docs/status.md` in the same change. If a change contradicts
  `docs/decisions.md`, stop and say so instead of working around it.

## Commands

```bash
python -m trackC.terrain_gen trackC/courses/flat_v1.yaml        # build and install a course scene
python -m trackC.friction_check trackC/courses/flat_v1.yaml     # every surface within about 0.01 of its friction
python -m trackC.smoke_test trackC/courses/debug_v03.yaml       # logger physics checks
python -m unittest trackC.test_course_spec trackC.test_batch     # course rules, gap, converter, metrics, batch matrix
python -m trackC.batch --course trackC/courses/mixed_mu03_v1.yaml --gaps 1.1 --variants low --seeds 2   # small sweep
python -m trackC.terrain_gen trackC/courses/mixed_v1.yaml --gap 0.8 --variant high --seed 3   # one swept course
python -m trackA.sim.run_waypoints --course trackC/courses/flat_v1.yaml --mpc perfoot --no-render
python -m trackA.reference.test_path_follower
python -m controllers.test_1_4_hooks
python -m planning.test_planner --scenario multi --no-plot
python -m unittest discover -s planning -t . -p "test_*.py"   # planner, footprint, tube (about 40 s)
```

Setup details and the PR review procedure are in `SETUP.md`.
