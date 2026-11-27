---
paths:
  - "trackC/**"
---
# Track C: environment, data and learning (owner Ben)

- Course files live in `trackC/courses/*.yaml`; the generator is `trackC/terrain_gen.py`, geometry in
  `trackC/course.py`. Regions are polygons in plan view, lifted onto each section's plane and tiled.
- World frame (A1): origin at the back-right inner wall corner, y from 0 (right wall) to 3 (left wall). `Course` exposes
  `x0, x1, x2, x3, y0, y1, yc` and `wall_clearance(x, y)`; use those instead of `width / 2`. Course files have no
  `lower_flat.start` (the course always begins at x = 0).
- Friction (C3) and the mid-ramp gap (A3.1) live in `course_spec.py`. Load courses with `load_course_cfg(path, gap,
  variant)`, never `yaml.safe_load` directly, so every module sees the barrier and the same friction rules. The planner
  gets its input from `planner_env.planner_problem`; friction lookups go through `friction_map` (`query(x, y)`).
- Terrain tiles are named `surf_<region>__<k>` and have higher contact priority than the feet; keep both.
- `trackC/contact_logger.py` classifies each stance as hold or slip from the surface-normal force ratio. Straddle
  flags are for evaluation only and never feed the learner.
- The batch runner (2.3) builds each scene itself so it never runs a stale course, and runs one subprocess per run.
- Learning (3.x) exposes `query(x, y) -> (mu_plan, mu_lo)` (B2).
