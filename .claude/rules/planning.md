---
paths:
  - "planning/**"
---
# Planning and robust control (owner Austin)

- The planner already works in the world frame (x 0 to about 12 m, y 0 to 3 m); see `planning/mock_environment.py`.
- Footprint model is `three_circle` (`planning/robot_footprint.py`). The two-pass planner is
  `planning/robust_two_pass.py`; it needs a `predict_contacts` callable.
- Tube functions in `planning/robust_control/friction_tube.py` return position radii in meters (B1). The error
  model's B must match its own dt (usually 0.02 s); never mix it with planner knot spacing.
- Acceleration limit uses μ = 0.5 for the robust baseline (A5).
- Infeasible second pass should become a "no feasible plan" outcome for the batch runner (status: Migration).
- Tests: `python -m unittest discover -s planning -t . -p "test_*.py"` (about 40 s).
