---
paths:
  - "trackC/**"
---
# Track C: environment, data and learning (owner Ben)

- Course files live in `trackC/courses/*.yaml`; the generator is `trackC/terrain_gen.py`, geometry in
  `trackC/course.py`. Regions are polygons in plan view, lifted onto each section's plane and tiled.
- Target frame and friction rules are in `docs/decisions.md` (A1, A1.1, A3.1, A3.3, C3). Existing course files still
  use the old spawn-at-origin frame until task 2.3a lands.
- Terrain tiles are named `surf_<region>__<k>` and have higher contact priority than the feet; keep both.
- `trackC/contact_logger.py` classifies each stance as hold or slip from the surface-normal force ratio. Straddle
  flags are for evaluation only and never feed the learner.
- The batch runner (2.3) builds each scene itself so it never runs a stale course, and runs one subprocess per run.
- Learning (3.x) exposes `query(x, y) -> (mu_plan, mu_lo)` (B2).
