# Task 1.3 — Direct Collocation Planner

This folder contains the global reference planner for Task 1.3 of the MEAM 5170 final project.

The planner generates a collision-free 2-D center-of-mass (CoM) reference from a start state to a goal state. It uses a point-mass direct-collocation model in CasADi, solves the nonlinear program with IPOPT, and uses A* only to provide a good geometric initial guess around obstacles.

This is the **global planning layer**, not the quadruped locomotion controller. The output is intended to be converted into a reference for Quadruped-PyMPC/acados, which handles the actual quadruped dynamics and contact forces.

## Files

- `direct_collocation.py`  
  Main planner. Builds and solves the direct-collocation NLP.

- `planner_helpers.py`  
  Geometry utilities, convex-polygon inflation/halfspaces, A* warm-start generation, and path sampling.

- `mock_environment.py`  
  Simple standalone environments used while developing and testing the planner.

- `test_planner.py`  
  Standalone test/plotting harness for the empty, single-obstacle, and multi-obstacle cases.

Only `direct_collocation.py` and `planner_helpers.py` are required by the planner itself. The mock environment and tester are included so the planner can be tested independently of the full simulation stack.

## Planner Interface

The main entry point is:

```python
from direct_collocation import plan_trajectory

result = plan_trajectory(
    start_state=start_state,
    goal_state=goal_state,
    environment=environment,
    config=config,
)
```

### State format

States use

```text
[px, py, vx, vy]
```

where position is in meters and velocity is in m/s.

Example:

```python
start_state = np.array([0.5, 1.5, 0.0, 0.0])
goal_state  = np.array([11.5, 1.5, 0.0, 0.0])
```

The terminal state is enforced, so the goal velocity is also specified by `goal_state`.

## Required Environment Interface

The environment passed to `plan_trajectory()` must provide:

```python
environment.bounds
```

with format

```text
(x_min, x_max, y_min, y_max)
```

and

```python
environment.obstacles
```

as a list of polygon arrays. Each polygon should have shape `(M, 2)`:

```python
np.array([
    [x1, y1],
    [x2, y2],
    ...
])
```

Obstacle vertices should be listed **counter-clockwise**. The current obstacle formulation assumes each obstacle is **convex**. A concave obstacle should be represented as multiple convex polygons.

The environment must also provide:

```python
environment.friction_mu
```

which is the nominal scalar friction coefficient used by the planner's acceleration constraint.

## Required Config Values

Current configuration fields are:

```python
config = {
    "N": 30,
    "T": 12.0,
    "max_speed": 1.0,
    "smoothness_weight": 1.0,
    "robot_length": 0.7,
    "robot_width": 0.31,
    "r_safety": 0.1,
    "astar_resolution": 0.1,
    "astar_clearance": 0.1,
}
```

`N`, `T`, and `max_speed` are development/tuning values rather than fixed project constants. They can be changed during integration based on the final course and desired Go2 gait speed.

## Model

The global planner uses a 2-D point-mass CoM model:

```text
x = [px, py, vx, vy]
u = [ax, ay]
```

with dynamics

```text
px_dot = vx
py_dot = vy
vx_dot = ax
vy_dot = ay
```

The direct-collocation transcription currently uses degree-3 Legendre collocation with three internal collocation points per interval.

The point-mass model is intentional: this planner only needs to generate a reasonable global CoM reference. The downstream MPC handles the quadruped/contact dynamics.

## Objective

The main running cost minimizes acceleration effort:

```text
L = uᵀu
```

A smoothness term also penalizes changes in acceleration between adjacent intervals:

```text
lambda_s * ||U_k - U_(k-1)||²
```

This discourages unnecessarily aggressive or rapidly changing references without adding another hard constraint.

## Constraints

The planner currently enforces:

### Start and goal

The initial state is fixed to `start_state`, and the final state is constrained to `goal_state`.

### Point-mass dynamics

The collocation equations enforce the continuous-time point-mass dynamics inside every interval, and continuity constraints connect consecutive intervals.

### Maximum speed

```text
vx² + vy² <= max_speed²
```

This limits total planar speed rather than limiting the x/y components independently.

### Friction-limited acceleration

```text
ax² + ay² <= (mu * g)²
```

This is a simple nominal planar friction constraint. The global planner does **not** know the true spatial friction map. Spatial friction uncertainty and adaptation are handled later by the robust/adaptive MPC and learning parts of the project.

### Course boundaries

The optimizer's usable rectangular course bounds are reduced by the conservative robot footprint radius so the CoM cannot run directly along a physical course edge while the robot body extends outside it.

### Obstacle avoidance

Obstacles are inflated by a conservative orientation-independent robot radius:

```text
r_robot = sqrt((length/2)² + (width/2)²) + r_safety
```

For the current Go2 development values this is approximately `0.483 m`.

Each convex obstacle is converted into half-space inequalities. A smooth log-sum-exp approximation is used to represent the condition that the CoM must remain outside at least one polygon face. Obstacle constraints are applied at both knot states and internal collocation states.

This supports convex polygons that are not axis-aligned. Rotated rectangles, triangles, etc. are fine as long as their vertices are ordered counter-clockwise.

## A* Warm Start

Obstacle avoidance makes the direct-collocation NLP nonconvex. A straight initial guess can put IPOPT inside an obstacle or in the wrong route topology, especially with multiple obstacles.

To improve solver reliability, `planner_helpers.py` runs an 8-connected A* search on a 2-D grid before the NLP solve.

A* is used **only as an initial guess**:

```text
environment
    -> inflated geometry
    -> A* path
    -> initial NLP state guesses
    -> IPOPT/direct collocation
    -> optimized trajectory
```

The final trajectory is still determined by the direct-collocation optimization and must satisfy all dynamics and constraints.

Current A* development defaults are:

```text
resolution = 0.10 m
extra initial-guess clearance = 0.10 m
```

The extra A* clearance only affects the warm start; it is not an additional final obstacle constraint.

A* also provides a useful geometric feasibility check before IPOPT is run.

## Output

`plan_trajectory()` returns:

```python
{
    "t": ...,
    "x": ...,
    "u": ...,
    "success": ...,
    "return_status": ...,
}
```

where:

- `t`: `(N+1,)` array of knot-state times
- `x`: `(N+1, 4)` array of states `[px, py, vx, vy]`
- `u`: `(N, 2)` array of interval controls `[ax, ay]`
- `success`: solver success flag
- `return_status`: IPOPT/CasADi solver status string

The integration code should check `success` before using the returned reference.

## Running the Standalone Tests

From the `planning/` directory:

```bash
python3 test_planner.py --scenario empty
python3 test_planner.py --scenario single
python3 test_planner.py --scenario multi
```

Useful options include:

```bash
--mu 0.5
--N 30
--T 12
--max-speed 1.0
--smoothness_weight 1.0
--no-plot
```

Example:

```bash
python3 test_planner.py --scenario multi --mu 0.5
```

The tester checks solver success, trajectory dimensions, start/goal states, nominal friction acceleration limits, course bounds, and collisions for the current rectangular test obstacles. It also plots the CoM path, velocity, and acceleration.

## Current Scope / Assumptions

- 2-D CoM global planner only.
- Full quadruped and contact dynamics are left to Quadruped-PyMPC.
- Friction is represented by one nominal scalar value in this planner.
- The planner does not use the learned spatial friction map.
- Convex polygon obstacles are supported directly.
- Concave obstacles should be decomposed into convex pieces.
- The robot footprint is approximated conservatively as a circle.
- The outer workspace is currently represented as a rectangle; more complicated course shapes can be represented by adding forbidden polygon regions as obstacles.
- Ramp/contact dynamics are intentionally left to the MPC layer. The global planner remains a 2-D reference planner.
- IPOPT is a local nonlinear optimizer. A* improves initialization but does not provide a global optimality guarantee.

## Stage 2 Integration Note

Task 1.3 produces the nominal collision-free reference.

Later in Stage 2, the robust tube radius will add another safety margin around obstacles. At that point the planner obstacle inflation can be extended from approximately

```text
r_robot
```

to

```text
r_robot + tube margin
```

without changing the basic direct-collocation architecture.

The immediate integration task is to convert the returned position/velocity trajectory into the reference format expected by Quadruped-PyMPC.
