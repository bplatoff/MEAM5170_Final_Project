# Track C starter v2: course generator, friction check, contact logger

Matches **parameter sheet v0.3** (the source of truth for course and friction values). Tested against the
gym-quadruped Go2 model in MuJoCo 3.x. The Quadruped-PyMPC controller itself has not been run with this code yet.

## Files

| File | What it does |
|---|---|
| `course.py` | Course geometry: lower flat, 10° ramp, upper flat. Height z(x) and which section a point is in |
| `terrain_gen.py` | Builds the MuJoCo scene from a course YAML: friction regions, triangulated tiles, walls, obstacles, checks |
| `courses/debug_v03.yaml` | One hand-designed example course: 8 regions (two crossing a crease), a gateway, a narrow passage |
| `visualize_course.py` | Plan view and side profile of a course, showing how regions were cut into tiles |
| `friction_check.py` | Measures the friction MuJoCo actually applies on every surface piece, flat and ramp |
| `contact_logger.py` | Per-step foot contacts, slip detection, wall and obstacle collisions, per-stance hold/slip events; straddle flag for evaluation only |
| `smoke_test.py` | Physics checks of the logger without the MPC |
| `course_debug_v03.png` | The example course, rendered by `visualize_course.py` |

Extra dependency over Quadruped-PyMPC: `pip install "shapely>=2.1" pyyaml matplotlib`.

## Run order

```bash
python visualize_course.py courses/debug_v03.yaml          # look at the course first
python terrain_gen.py courses/debug_v03.yaml               # install scene_debug_v03.xml into gym_quadruped
python friction_check.py courses/debug_v03.yaml            # every surface should measure its set value
python smoke_test.py courses/debug_v03.yaml                # logger physics checks
```

Seeded friction values: add `--seed N` to `terrain_gen.py`, `friction_check.py` or `visualize_course.py`. The
scene is then named `debug_v03_sN`, and the same seed always gives the same course.

In Quadruped-PyMPC: set `simulation_params['scene']` to the printed scene name, and pass a fixed float as
`friction_coeff` to `run_simulation` (the default range randomizes the floor, which our courses never touch, but
fixed values keep runs repeatable). The MPC's own friction assumption is `'mu': 0.5` in `config.py`, matching the
sheet's nominal controller μ.

## Verified results

Friction check on `debug_v03` (set → measured): every one of the 13 surface pieces measured within 0.011 of its
set value, including both pieces of `snow_crease` (lower flat and ramp) and `snow_top` (ramp and upper flat).

Smoke tests:

| Test | Expected | Measured |
|---|---|---|
| Flat ice (μ 0.20), pushed sideways | sliding ratio 0.20 | 0.200 |
| Standing still on the ramp, all four feet | tan 10° = 0.176 | 0.176 |
| Ramp snow (μ 0.30), pushed downhill | sliding ratio 0.30 | 0.300 |
| Pushed toward the left wall | wall collision | `wall_left_lower` |
| Event kinds (test 1) | only `hold` / `slip`; no evaluation field in `learner_events()` | 76 events, hold and slip only, none leaked |

Straddle flag, checked separately by pushing the robot from ice across the ice/base edge: 2 of 23 stances were
flagged as straddles, both classified `slip` at r = 0.20 from their measurements, with `mu_true_min/max` = 0.20/0.50.

Geometry: every tile vertex lies on the course surface (or exactly 5 cm below it) to within 0.001 mm, and a
downward ray grid over the whole course hits a terrain tile at the correct height everywhere.

## What changed from v1, and why

1. **Three-section course from the sheet.** 3 m lower flat, 6 m ramp along the slope at 10° (rises 1.04 m),
   3 m upper flat, 3 m wide. The robot spawns at the origin, 0.8 m in front of the back wall.
2. **Polygonal regions on any section.** Regions are drawn in plan view. The generator clips them to each
   section (this is what splits a region at a crease), cuts each piece into triangles, and builds every triangle
   as a thin prism lying on its section's plane. Triangles are used because MuJoCo collides with convex shapes,
   and any polygon, including concave ones or the base surface with holes, splits into triangles.
3. **Walls on all four sides,** 0.5 m above the local surface, pitched along the ramp. Robot contact with a wall
   or obstacle is logged as a collision.
4. **Friction classes and seeds.** `low / medium / high` = 0.20 / 0.50 / 0.80, or seeded draws within each class
   range. A region touching the ramp is never drawn below the 0.25 ramp floor, so one region keeps one friction
   value on every section. `ramp_stress_test: true` on a region allows going below the floor (down to tan θ).
5. **Checks that stop a bad course:** ramp friction at or below tan θ, start or goal closer than 0.5 m to a wall,
   or pieces that don't exactly cover the course.
6. **Logger fix: force ratio in the surface frame.** v1 computed |f_xy| / f_z, which is only right on flat
   ground. On the ramp, friction acts along the surface, so v2 splits each foot force into the part along the
   contact normal and the part tangent to it. Test 2 above is the check: world-frame math would give about 0.

7. **The learner sees only what a robot can measure.** v2 originally labeled a stance that touched two regions as
   `kind = "straddle"` and excluded it, which used simulator knowledge of where the boundaries are. Now every stance
   is a `hold` or a `slip`, decided from its force ratio and foot velocity alone. The straddle flag, the region
   names and the true friction are still recorded, but only as evaluation columns (`EVAL_FIELDS`).
   `learner_events()` strips them; feed the learner from that. Boundaries are found only by the disagreement test.
   For a straddle, `mu_true` is NaN and `mu_true_min` / `mu_true_max` give the two sides, so evaluation can count
   holds with `r > mu_true_min` (certificates that overstate the lower side).

## Two things the tests taught us

- **Single-foot ratios include internal forces.** Standing still, each foot reads a ratio of about 0.24 even on
  flat ground, because the legs push against each other; those forces cancel in the four-foot sum. For learning
  this is fine: a foot that held at ratio r still proves μ ≥ r. But slip detection and anything that compares a
  foot's ratio to tan θ must keep it in mind.
- **The straddle band is about 1 cm.** The foot sinks about 3 mm into the soft contact, so it can touch two tiles
  within roughly √(2 × 0.022 m × 0.003 m) ≈ 1 cm of a boundary.

## What is fixed and what varies

A course file fixes the region shapes and obstacle positions; `debug_v03` is one example layout. Friction values
vary with `--seed`. Obstacles can sit on any section, including the ramp: their base is buried below the slope along
their full length and their top is at least `height` above the highest surface point under them (checked: a 0.4 m
obstacle on the ramp stands 0.40 m above the surface on its uphill side and 0.50 m on its downhill side).

## Not done yet

- Seeded random **layouts** (the sheet's second phase): region shapes and obstacle positions drawn per seed,
  including obstacles on the ramp. Only friction values are seeded so far.
- Narrow-passage width is a 0.7 m placeholder until the robust tube radius is measured.
- Slip thresholds (`v_slip`, `k_slip`) need tuning on real walking runs with the MPC.
