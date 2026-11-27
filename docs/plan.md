<!-- Generated from MEAM5170_Final_Project_Plan.docx (shared drive), version of Nov 21, 2026.
The docx is the source of truth. Regenerate with:
pandoc MEAM5170_Final_Project_Plan.docx -t gfm --wrap=none -o docs/plan.md   (then fix figures and the two case equations) -->

# Robust and Adaptive MPC for Quadruped Locomotion: project plan

MEAM 5170 final project. Plan of October 29, 2026, updated November 21, 2026. Due December 10, 2026.

# Overview

We will make a simulated Unitree Go2 walk from a start position to a goal across terrain where friction varies by region and is unknown until contact, including ramps and obstacles. The project builds in three stages, each adding one capability, followed by two stretch goals that can be developed in parallel.

| **Stage** | **Capability**                                                                 | **Status**       |
|-----------|--------------------------------------------------------------------------------|------------------|
| 1         | Nominal MPC walking on flat, constant-friction ground                          | Core, complete   |
| 2         | Robust tube MPC (mid layer) on ramps and mixed-friction terrain near obstacles | Core             |
| 3         | Learned friction map from assumed probing data, used to shrink the tube        | Core             |
| Stretch A | Implement the probing sequence in simulation                                   | Parallel stretch |
| Stretch B | Learning with both equality and inequality friction observations               | Parallel stretch |

**Platform:** Quadruped-PyMPC (Python, acados single rigid body MPC), Unitree Go2 model, MuJoCo. Starting from an existing MPC lets the team spend its time on robustness and learning instead of rebuilding locomotion. Our layers sit above it: a global planner and a mid-layer robust MPC that sends Quadruped-PyMPC world-frame velocity and yaw-rate commands.

**Central claim:** the MPC itself carries the robustness guarantee, and learning friction measurably shrinks the safety margin it needs, so the robot can take tighter, faster paths as its friction knowledge improves.

# Environment

## Course layout

The course follows the parameter sheet (v0.3), which is the source of truth for course geometry and friction values. In summary:

- **Profile:** 3 m lower flat, then a 6 m ramp (measured along the slope) at 10°, rising 1.04 m, then a 3 m upper flat. The course is 3 m wide.

- **Walls:** on all four sides, 0.5 m above the local surface and 0.1 m thick. Any robot contact with a wall or an obstacle counts as a collision.

- **Start and goal:** each run draws its start pose from a seeded uniform box on the lower flat, at least 0.5 m from walls and 1.0 m from obstacles, heading within ±15°. The start seed is separate from the friction seed. The goal is one fixed point per course, the same for every run, at least 1.0 m from obstacles and 0.5 m from walls.

- **Friction:** every region takes one value from the discrete set {0.3, 0.4, 0.5, 0.6, 0.7, 0.8}. The nominal value is 0.5 and the lower limit is 0.3 everywhere, including the ramp. Low patches (0.3) sit on the ramp, at turns and near obstacles, where the robot needs the most grip; straight flat stretches would not make it slip. High patches are 0.7 or 0.8. Everything outside a region is base ground (0.5).

- **Obstacles:** a gateway at the bottom of the ramp and a passage at the top, each at least 1.2 m wide so the robust and learned controllers both pass. A mid-ramp barrier is one rectangle attached to the right wall (y = 0), leaving a gap g to the left wall. g is a course parameter that the batch runner sweeps (about 0.6 to 1.1 m in 0.1 m steps). Two course variants place either a 0.3 patch or a high-friction patch (0.7 or 0.8) around the gap.

- **Gap sizes (provisional):** with the three-circle footprint (radius 0.194 m), a 0.05 m margin and Austin’s current gains, the robust controller needs a gap of about 1.0 m and the learned controller about 0.6 to 0.75 m; the heading-error term is still to be added. Widths are locked after task 2.5 measures the real tube. Below a controller’s minimum the planner finds no route, which is recorded as its own outcome (no feasible plan).

## Coordinate system

> **Figure.** Course coordinate system. The world frame is fixed to the course: origin at the back-right inner wall corner on the lower-flat surface, x forward along the course (horizontal distance, also on the ramp), y to the left, z up. All data shared between modules uses this frame; the body frame is used only inside the controller. Start poses are drawn from a seeded start zone on the lower flat, and the goal is fixed per course. *(Image in the plan docx on the shared drive.)*

*Course coordinate system. The world frame is fixed to the course: origin at the back-right inner wall corner on the lower-flat surface, x forward along the course (horizontal distance, also on the ramp), y to the left, z up. All data shared between modules uses this frame; the body frame is used only inside the controller. Start poses are drawn from a seeded start zone on the lower flat, and the goal is fixed per course.*

## Example course design

> **Figure.** Example course design (debug_v03): one hand-designed layout. Top: plan view, with friction regions colored by coefficient, white edges showing how the generator cut each region into tiles, walls, obstacles, start and goal. Bottom: side profile of the lower flat, 10° ramp and upper flat. *(Image in the plan docx on the shared drive.)*

*Example course design (debug_v03): one hand-designed layout. Top: plan view, with friction regions colored by coefficient, white edges showing how the generator cut each region into tiles, walls, obstacles, start and goal. Bottom: side profile of the lower flat, 10° ramp and upper flat.*

The figure shows one example course, not the only course. What a course file fixes and what varies:

- **Region shapes and obstacle positions** are set by the course file. debug_v03 is the hand-designed layout the sheet calls for first: 8 regions, two of which cross a crease, a gateway on the lower flat, and a narrow passage on the upper flat whose 0.7 m width is a placeholder. It predates the layout above; the course files move to the new frame, friction set and mid-ramp gap.

- **Friction values vary by seed.** With a seed, each region draws a discrete value from its class. The same seed always gives the same course, so every controller faces identical terrain.

- **Seeded random layouts** (region shapes and obstacle positions also drawn per seed) are the sheet's second phase and are not built yet.

- **Obstacles may be placed on any section, including the ramp.** An obstacle's base is buried below the slope along its full length and its top stands at least its set height above the surface. Checked in simulation: a 0.4 m obstacle mid-ramp stands 0.40 m above the surface on its uphill side and 0.50 m on its downhill side.

## Terrain as friction polygons

Regions are drawn as polygons in plan view (x along the course, y across it). Implementation notes for MuJoCo, each verified in simulation with the starter code:

- **Tiled course:** each region and the base surface are clipped to each section, split into triangles, and each triangle is built as a thin convex prism lying on its section's plane. Tiles never overlap. Clipping at the section boundaries is what splits a region at a crease into same-name pieces.

- **No patches on a floor plane:** the Go2 foot sinks about 3 mm into the soft contact, so a patch laid on a floor plane lets the foot touch both and mixes their friction. MuJoCo's floor plane sits 10 cm below the course and is never touched.

- **Contact friction:** by default MuJoCo combines two contacting geoms' friction by taking the maximum. Every terrain tile has a higher priority than the feet, so the terrain value is used. Without it, every surface measured 0.61 (the foot's own value) in testing.

- **Naming:** tiles are named surf\_\<region\>\_\_\<k\>, so the contact logger can tell a seam between tiles of one region from a real friction boundary, and gym-quadruped's reset (which overwrites the friction of geoms named floor, ground, terrain or hfield) never touches them.

- **Sand** is represented only by its friction coefficient. MuJoCo contact is rigid, so sinkage and deformation are out of scope and stated as a simplification.

- **Check:** the measured friction on every surface piece of the example course, flat and ramp, matched its set value to within 0.011.

## Friction bounds and ramp angle

All friction values lie in an assumed interval. The robustness guarantee is stated conditional on this assumption:

$$\mu \in \left\lbrack \mu_{\min},\ \mu_{\max} \right\rbrack\quad \text{ everywhere on the course}$$

A stance foot on a slope of angle θ needs friction of at least tan θ just to hold position, so ramp angles and friction values must be chosen together, with margin for acceleration:

$$\mu_{min,ramp} > tan\theta$$

On the 10° ramp, tan θ ≈ 0.176. The course sets μ_min = 0.3 everywhere, flat and ramp, and μ_max = 0.8. Tests on the plain ramp course (ramp_v1) support this floor: with forces planned for 0.3 the robot climbs without slipping and reaches the goal in 40 s (35 s when planned for 0.5 on 0.5 ground); planning for 0.25 is slow and planning for 0.2 stalls. Planning for 0.5 on a 0.3 patch slips, and the robot fell. That is the failure the tube must cover.

# Stage 1: Nominal MPC on flat ground

**Goal:** Quadruped-PyMPC running the Go2 in our own MuJoCo environment, reaching a goal on flat ground with constant friction.

Tasks:

1.  Install Quadruped-PyMPC, run the Go2 trot demo, and confirm solve times.

2.  Swap in our environment generator with a single flat polygon.

3.  Replace the velocity-command reference with a goal-directed reference (waypoints first).

4.  Add logging of contact positions, contact forces, force ratio and foot slip velocity. Stage 3 uses this data.

5.  **Verify the hooks needed later:** that the acados problem can take friction per foot and per timestep as a parameter, and accept extra linear constraints on CoM position.

**Exit criteria:** the robot reaches the goal reliably over repeated runs, logs are recorded, and the two hooks in task 5 are confirmed or a workaround is identified.

**Status: complete.** The exit test (1.5) passes, and the hooks from 1.4 work: per-foot friction (set_friction), wall half-spaces (set_walls) and a slope-aware friction cone.

# Stage 2: Tube MPC on ramps and mixed friction

**Goal:** walk near obstacles over ramps and friction patches with a guarantee that slip within the assumed friction bounds cannot cause a collision.

## Three-layer architecture

- **Global planner (track B):** direct collocation on a point-mass model. It outputs the desired position and velocity along a route to the goal. Its acceleration limit uses μ = 0.5, the robust baseline; the adaptive version takes its limits from the learned map (stage 3).

- **Mid-layer robust MPC (track A with track B):** point-mass model, about 2 s horizon. It tracks the planner’s desired state, keeps its nominal trajectory between solves, applies the tube feedback and enforces the tightened obstacle and wall constraints. It outputs world-frame commands \[v_x, v_y, 0\] and \[0, 0, yaw rate\] to the low level.

- **Low level:** Quadruped-PyMPC chooses footholds and forces. Its friction cones use μ_lo at each foot, so the feet do not slip.

## Reference and obstacles

- Reference path from the global planner, built in two passes: plan, propagate the tube along the path, then replan with obstacles and walls inflated by the three-circle footprint (radius 0.194 m, centers at 0 and ±0.233 m along the body axis), a 0.05 m margin and the largest tube radius on the route.

- Obstacles and walls enter the mid-layer MPC as linear half-space constraints on each of the three circle centers, so the problem stays a QP-style solve.

## Tube formulation

The mid layer plans motion with μ_plan, while Quadruped-PyMPC limits foot forces to μ_lo. Where μ_lo \< μ_plan the robot cannot reach the planned acceleration. This shortfall, plus velocity-tracking error, is the bounded disturbance per mid-layer step Δt:

$$r_{w} = \Delta t\left( \mu_{plan} - \mu_{lo} \right)g + r_{0}$$

Here r_0 covers velocity-tracking error, from the logs. Before learning, query(x, y) in the certificate layer returns μ_plan = 0.5 and μ_lo = 0.3 everywhere, so r_w is the same across the course. Deviations are corrected by Austin’s LQR gain K, computed once offline, acting on the error between the measured state and the mid layer’s nominal state:

$$a_{cmd} = {\bar{a}}_{k} + K\left( x_{meas} - {\bar{x}}_{k} \right),\ \ \ \ \ v_{cmd} = \left( \tau_{eff}\ a_{cmd} + v_{meas} \right)/k$$

x̄ is the nominal position and velocity and ā the nominal acceleration. K already includes the minus sign (A_cl = A + BK), so this is negative feedback. The velocity command inverts Quadruped-PyMPC's measured first-order velocity lag (time constant τ and gain k per body axis, from step tests; τ_eff = Δt/(1 − e^(−Δt/τ))), so the robot reaches the commanded acceleration over one step and matches the double-integrator error model (B3, revised Nov 26; the original v_meas + Δt a_cmd stays available as an option). The tube width follows the homothetic tube recursion from the robust control lecture:

$$\alpha_{k + 1} = \rho\ \alpha_{k} + r_{w,k},\ \ \ \ \ \alpha_{\infty} = \frac{r_{w}}{1 - \rho}$$

With constant r_w the tube grows from zero over about 2 s at the start of a run to the constant width α\_∞. Each predicted step k of the mid layer uses its own α_k, propagated from the current width, and α_k never exceeds the planner’s inflation. Obstacle and wall constraints on the three nominal circle centers c̄\_m,k are tightened by the tube:

$$a_{j}^{\top}{\bar{c}}_{m,k} \leq b_{j} - \left( r_{c} + r_{s} + \alpha_{k} \right)\left\| a_{j} \right\|,\ \ \ \ \ m = 1,2,3$$

Here r_c = 0.194 m is the circle radius and r_s = 0.05 m the safety margin.

- **Later option:** make α a decision variable (homothetic tube MPC).

- **Checks (task 2.5):** measure Quadruped-PyMPC’s velocity-tracking time constant from the logs; if it is much longer than Δt, add the lag to the error model and recompute K and ρ. Low-pass filter v_meas over about one gait cycle before using it. Yaw rate comes from a separate loop, and its heading error must stay bounded for the three-circle footprint.

- ρ is the contraction factor of the closed-loop error dynamics. It must be below 1, which the Lyapunov ellipsoid template guarantees.

- **Assumption:** the low level is rarely at its friction limit. Task 2.5 measures the fraction of time feet sit at the limit and the drift against α.

**Exit criteria:** across randomized slip trials (20 seeds per gap width), measured CoM deviation stays inside the predicted tube and no collisions occur.

# Stage 3: Learning from assumed probing data

**Goal:** show that knowing friction better shrinks the tube and improves speed and path choice.

**Assumption for this stage:** a probing sequence exists and has already been run. It produces a normal path of footholds toward the goal where each foothold has a known friction value. In simulation these values are read from the ground truth, emulating a perfect probe. The probing sequence itself is not implemented in this stage (see Stretch A).

## Data used

| **Data**          | **Source in stage 3**                                           | **Type**               |
|-------------------|-----------------------------------------------------------------|------------------------|
| Probed footholds  | Assumed probing run, labeled from sim ground truth              | Equality: μ known at p |
| Non-slip contacts | Later runs, force ratio r = ‖f_t‖ / f_n (normal to the surface) | Lower bound: μ ≥ r     |

## Method

1.  **GP on equalities.** Fit a Gaussian process with an RBF kernel to the probed footholds. It returns a mean m(p) and standard deviation σ(p) anywhere on the course.

2.  **Lower bound from the GP:** μ_GP,lo(p) = m(p) − βσ(p).

3.  **Threshold radius for non-slip contacts.** No foot lands on exactly the same point twice, so a non-slip contact certifies its force ratio within a small radius ρ_c around it. Away from detected boundaries, the lower bound takes the larger of the two (the full rule, including slip caps and boundary zones, is in the next section):

$$\mu_{lo}(p) = max\left( m(p) - \beta\,\sigma(p),\mspace{6mu}\max_{j:\, \parallel p - p_{j} \parallel \leq \rho_{c}}r_{j}^{hold} \right)$$

4.  **Planning friction.** Plan with whichever estimate is higher, so the robot never plans with less friction than it has already proven:

$$\mu_{plan}(p) = max\left( m(p),\ \mu_{lo}(p) \right)$$

5.  **Feed the tube.** query(x, y) returns μ_plan and μ_lo at each planned foothold, and they go into r_w from stage 2. Where μ_lo rises toward μ_plan the shortfall shrinks, r_w drops toward r_0 and the tube narrows.

6.  **Update rules.** Before learning, μ_plan = 0.5 and μ_lo = 0.3. μ_plan can move both ways, μ_lo never exceeds μ_plan, and μ_plan rises above 0.5 only where μ_lo has risen. Further learning choices are decided after the robust parts are complete.

7.  **Use μ_lo for speed.** The adaptive controller uses the learned μ_lo in both upper layers. The planner’s tube comes from the learned map, and the mid layer’s speed, turn-rate and acceleration limits scale with μ_lo, capped by Quadruped-PyMPC’s maximum walking speed:

$$v^{2}\kappa \leq \left( \mu_{lo} - \delta \right)g,\ \ \ \ \ a_{up} \leq \left( \mu_{lo}cos\ \theta - sin\ \theta \right)g$$

κ is the path curvature, θ the slope and δ a reserve that keeps grip for the tube correction. With μ_lo = 0.3 the same rules give the robust baseline, so both controllers run under identical rules.

**Choosing ρ_c:** start at about 1 cm (trade-off explained in Handling friction boundaries in MuJoCo). The rules in the next section govern exactly how certificates and the GP are combined near friction boundaries, and they replace any cell-wide application of a bound.

**Exit criteria:** with the learned map, the tube is measurably narrower along the probed path, and the robot reaches the goal faster or through a gap the stage 2 controller could not use.

# Friction boundaries and point certificates

## The problem

A contact measures friction only at the point where the foot landed. Extending that measurement to a cell or a radius assumes no friction boundary lies inside that area. We deliberately do **not** assume boundaries are known from perception, because closed-form region boundaries are hard to obtain reliably in practice. The learning method therefore has to stay safe when an unseen boundary passes near the data.

Notation: regions Ω with constant friction μ, separated by boundaries Γ. A **hold** at p_h is a stance contact that did not slip while using force ratio r_h. A **slip** at p_s is a stance contact that slipped at ratio r_s:

$$hold\ at\ p_{h}:\ \ \mu\left( p_{h} \right) \geq r_{h},\quad\quad slip\ at\ p_{s}:\ \ \mu\left( p_{s} \right) \approx r_{s}$$

A slip is an equality, so it is also an upper bound: μ(p_s) ≤ r_s.

## Case 1: smoothing across a boundary

> **Figure.** Two-region boundary. (a) A slip and a hold on the low-friction side and a hold on the high-friction side, each with its certificate radius. (b) A stationary RBF GP fit to one point on each side, computed along a line crossing the boundary: it overestimates friction on the low side and underestimates it on the high side. *(Image in the plan docx on the shared drive.)*

*Two-region boundary. (a) A slip and a hold on the low-friction side and a hold on the high-friction side, each with its certificate radius. (b) A stationary RBF GP fit to one point on each side, computed along a line crossing the boundary: it overestimates friction on the low side and underestimates it on the high side.*

Two regions Ω_L and Ω_H with μ_L \< μ_H meet at Γ. A slip at p_s ∈ Ω_L gives r_s ≈ μ_L. Holds at p_1 ∈ Ω_L and p_2 ∈ Ω_H give r_1 ≤ μ_L and r_2 ≤ μ_H, so r_2 can exceed μ_L.

The GP posterior mean is a kernel-weighted combination of the data, with weights set only by distance:

$$m(p) = \mu_{0} + k(p)^{\top}\left( K + \sigma_{n}^{2}I \right)^{- 1}\left( y - \mu_{0} \right)$$

Near Γ the mean blends values from both sides:

- **High side:** the mean is pulled down by low-side data and can fall below the certified bound r_2. This is conservative, and the hold floor corrects it near p_2.

- **Low side:** the mean is pulled up by high-side data, so the GP overestimates friction. This is the unsafe direction, because it narrows the tube where friction is actually low.

The posterior standard deviation does not warn about this. For a stationary kernel, σ(p) depends only on the distances from p to the data, not on whether the data agree.

## Case 2: a corner between three regions

> **Figure.** Three regions meet at a corner inside one cell of radius R. Step 1 holds in the high-friction region, step 2 slips in the medium-friction region, and step 3 is a candidate foothold in the low-friction region. *(Image in the plan docx on the shared drive.)*

*Three regions meet at a corner inside one cell of radius R. Step 1 holds in the high-friction region, step 2 slips in the medium-friction region, and step 3 is a candidate foothold in the low-friction region.*

Regions Ω_H, Ω_M and Ω_L with μ_L \< μ_M \< μ_H meet inside one cell of radius R:

1.  Step 1 holds in Ω_H with ratio r_1, so r_1 ≤ μ_H, but r_1 may exceed μ_M.

2.  Step 2 slips in Ω_M, so r_2 ≈ μ_M.

3.  Step 3 is a candidate foothold in Ω_L.

Applying any single value to the whole cell fails:

- **Cell-wide hold maximum:** the cell’s lower bound becomes r_1. If r_1 \> μ_M, step 2 is planned with friction it does not have, so the guarantee is already violated at step 2.

- **Cell-wide slip value:** the cell’s estimate becomes μ_M. Step 3 then plans with μ_M on ground with μ_L \< μ_M, so slip at step 3 is likely and the tube is too narrow.

No rule can infer μ_L from data taken only in Ω_H and Ω_M. The data can show that a boundary is nearby, but not what lies across it.

## Cells, certificate radius and neighborhood radius

Three distances play different roles and must not be mixed:

| **Distance**           | **Typical size**                                        | **Role**                                                                     |
|------------------------|---------------------------------------------------------|------------------------------------------------------------------------------|
| Certificate radius ρ_c | About 1 cm (see Handling friction boundaries in MuJoCo) | Where a single hold or slip applies, measured from its exact contact point   |
| Neighborhood radius R  | About the GP length scale                               | Only used to check whether nearby data are consistent with a single surface  |
| GP cell h              | 10 to 25 cm                                             | Data reduction for GP training only (stretch B). Never used to apply a bound |

The sizes satisfy ρ_c \< h ≤ R.

## Data reduction

A trot produces roughly 8 contacts per second, nearly all of them holds, so hold data accumulate quickly. The data serve two jobs, and only one of them is reduced:

| **Job**                                           | **Data used**                                                     | **Why**                                                                                                                                                                                                                                                     |
|---------------------------------------------------|-------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Certificate layer (floor, cap, disagreement test) | **Every** contact, stored at its exact position                   | Each hold certifies only its own spot within ρ_c. Two holds 10 cm apart certify two different spots, so dropping either one loses a certificate. Lookups of contacts within ρ_c or R of a foothold are cheap with a KD-tree, so nothing needs to be dropped |
| GP training (stretch B only)                      | Per GP cell: the hold with the largest ratio r_h, plus every slip | Every hold in the GP is a training point, and exact GP cost grows as n³. Within a cell, a smaller hold next to a larger one changes the GP estimate very little, so dropping it mostly saves compute                                                        |

Reducing the GP data is safe because it only affects the GP’s accuracy. If a boundary passes through a GP cell, the reduced GP may be slightly off, but the certificate layer still holds every contact and still applies the floor, cap and boundary fallback on top of the GP. Data reduction changes cost and accuracy, never the safety guarantee.

## Rules

**Certified floor and slip cap.** Within ρ_c, holds set a floor and slips set a cap:

$$c(p) = \max_{h:\  \parallel p - p_{h} \parallel \leq \rho_{c}}r_{h},\quad\quad u(p) = \min_{s:\  \parallel p - p_{s} \parallel \leq \rho_{c}}r_{s}$$

with c(p) = −∞ and u(p) = +∞ when no certificate is in range.

**Disagreement test.** Two data points within R of each other that cannot come from one surface prove a boundary is nearby:

$$\parallel p_{h} - p_{s} \parallel \leq R\ and\ r_{s} < r_{h} - \epsilon\quad\quad or\quad\quad \parallel p_{s_{1}} - p_{s_{2}} \parallel \leq R\ and\ \left| r_{s_{1}} - r_{s_{2}} \right| > \epsilon$$

**ε covers measurement noise** and must stay below the 0.1 spacing between friction values; start at 0.05. Each disagreeing pair (a, b) adds the disks of radius R around both points to the heterogeneous zone Z.

**Lower bound and planning friction.** Outside Z, the GP is trusted together with the certificates. Inside Z, only direct certificates and the global minimum are trusted:

$$\mu_{lo}(p)=\begin{cases}\min\left(\max\left(\mu_{\min},\ c(p),\ m(p)-\beta\sigma(p)\right),\ u(p)\right) & p\notin Z\\ \min\left(\max\left(\mu_{\min},\ c(p)\right),\ u(p)\right) & p\in Z\end{cases}$$

$$\mu_{plan}(p)=\begin{cases}\min\left(\max\left(m(p),\ \mu_{lo}(p)\right),\ u(p)\right) & p\notin Z\\ \max\left(\mu_{lo}(p),\ \min\left(m(p),\ \min_{s:\,\|p-p_s\|\le R} r_s\right)\right) & p\in Z\end{cases}$$

Both values then feed the tube through r_w, as in stage 2.

## Applying the rules to the two cases

- **Case 1:** the slip at p_s and the hold at p_2 disagree (r_s \< r_2), so the area around Γ becomes heterogeneous. The overestimating GP mean on the low side is overridden: μ_lo falls back to μ_min there, except within ρ_c of p_1 and p_s. Near p_2 the floor r_2 still applies.

- **Case 2:** step 1 certifies only within ρ_c of its own point, so step 2 is planned from the GP and μ_min, not from r_1. The slip at step 2 disagrees with the hold at step 1, which marks the cell heterogeneous. Step 3 is not within ρ_c of any certificate, so μ_lo = μ_min and the tube widens locally. The robot behaves like the stage 2 controller there and stays safe.

## How each bound and disagreement is used

Both learning versions share the same certificate layer, placed between the GP and the MPC:

$$GP\ posterior\  \rightarrow \ certificate\ layer\ (floor,\ cap,\ zone\ fallback)\  \rightarrow \ \mu_{plan}(p),\ \mu_{lo}(p)\  \rightarrow \ tube\ MPC$$

| **Rule**                                                    | **Stage 3 (GP on equalities)**                                                                                          | **Stretch B (equalities and inequalities in the GP)**                                                                                                                                                        |
|-------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Hold gives a floor within ρ_c                               | Main channel for non-slip data, the only place hold information enters                                                  | Mostly redundant, since holds are in the posterior. Kept as an exact floor because the censored likelihood enforces the bound only probabilistically. Changes nothing when the posterior is already above it |
| Slip gives a cap within ρ_c                                 | Needed: caps GP blending near the slip                                                                                  | Still needed: the slip is a noisy equality in the posterior, and the cap guarantees the mean never exceeds it locally                                                                                        |
| Disagreement marks a heterogeneous zone with μ_min fallback | Needed. Probe points that disagree trigger it too, since two probes with different values imply a boundary between them | Still needed. Inequality data do not change the stationary kernel, which still smooths across boundaries and stays confident where data disagree                                                             |

Because both versions share this layer and the same interface to the MPC, any difference in tube width or time to goal comes from the learning model alone. Stretch B is expected to narrow the tube further in homogeneous areas, where hold data now lower σ. Near boundaries both fall back to μ_min and should perform the same.

## Assumptions and limitations

- Friction is constant within ρ_c of a contact point.

- In MuJoCo a foot touching a single terrain polygon makes one contact with one friction coefficient, so a hold or slip measures that polygon’s friction directly. A foot straddling a boundary measures a blended value, as it would on hardware. The learner cannot tell when this happens and treats the contact like any other; only the disagreement test can expose it (see the next section).

- Without known boundaries, no rule can certify an unvisited region next to a detected boundary. The design makes this cost speed, not safety: the worst case is a locally wider tube, and the stage 2 guarantee for μ ≥ μ_min still holds everywhere.

- The surface normal is read from the simulator. The learner uses it to split each foot force into normal and tangential parts. On hardware it would come from a slope estimate (leg kinematics or a terrain map); on this course’s planar sections the two agree.

- Possible extensions: bias foothold selection toward certified points or away from heterogeneous zones (Lecture 15 footstep MPC), or use a non-stationary kernel so σ itself grows near boundaries.

# Handling friction boundaries in MuJoCo

## How MuJoCo produces foot contacts

- **The foot is a small sphere.** In the gym-quadruped Go2 model each foot is a collision sphere of radius 2.2 cm with a 1 mm contact margin.

- Contacts are generated per pair of geoms. A foot sphere touching one terrain tile produces a single contact point with a single friction coefficient, set by the tile’s friction and the priority rule in the Environment section.

- **Friction is constant within a geom.** MuJoCo has no friction map inside one geom, so each friction polygon must be its own geom.

## Straddling contacts

The foot sinks about 3 mm into MuJoCo’s soft contact, so it touches a neighboring coplanar tile whenever it lands within roughly √(2 × foot radius × penetration) = √(2 × 22 mm × 3 mm) ≈ 1.1 cm of a polygon edge (measured with the track C starter code). In that case MuJoCo creates **two separate contacts**, one per geom, each with its own friction coefficient. MuJoCo never averages the coefficients, but the solver splits the foot’s total force between the two contacts.

The robot measures only the total force at each foot, so for a straddling foot the measured ratio mixes two surfaces:

$$r_{foot} = \frac{\parallel f_{t,1} + f_{t,2} \parallel}{f_{n,1} + f_{n,2}}$$

- A hold measured this way only proves that the force-weighted mix of the two coefficients exceeds the ratio, so it can overstate the lower-friction polygon.

- A slip measured this way lies between the two coefficients. A cap set by it is conservative for the higher-friction polygon.

## Rules

1.  **The learner never sees straddles.** A real robot measures only foot position, total foot force, foot velocity and contact state, and cannot tell which polygon a foot touched. Every stance therefore becomes a hold or a slip from its measured force ratio and foot velocity, straddling or not.

2.  **Boundaries come only from the disagreement test.** A straddling hold or slip enters the certificate layer like any other contact. If its blended value disagrees with a nearby contact by more than ε, the pair creates a heterogeneous zone with the μ_min fallback. If it does not disagree, it counts as an ordinary measurement.

3.  **Straddles are logged for evaluation only.** The logger still counts each stance foot’s contacts with distinct terrain regions (each tile carries its region id in its geom name) and stores a straddle flag per step and per stance. The flag measures how often blended contacts occur, how many became certificates and whether any led to an unsafe μ_lo. It is never an input to the learned pipeline.

4.  **Ground truth stays outside the learner.** The emulated probe labels in stage 3 come from simulator ground truth (a stated assumption), and evaluation compares against the true friction map. Neither feeds region membership or straddle flags back into the learner.

## Environment construction

- Tile the whole course: every friction polygon is cut into triangles and each triangle is built as a thin convex prism, with neighboring tiles sharing edges exactly and no floor plane in reach of the feet (see the Environment section). Small gaps or overlaps would create spurious contacts at the seams.

- Give terrain geoms higher contact priority than the feet so terrain friction always governs.

- Straddles occur only within about 1 cm of an edge, so they are uncommon, but over thousands of steps they will happen. Log the straddle count per run for evaluation.

## Assumption and residual risk

Because straddling contacts are not excluded, a hold within about 1 cm of an edge can carry a blended ratio that overstates the lower-friction side. Its certificate then reaches across the edge by up to ρ_c.

Two things limit this. A blended hold matters only if a later foothold lands within ρ_c of it on the lower side, and any slip or lower hold measured there disagrees with it and puts both inside a zone with the μ_min fallback. The residual risk is a foothold within ρ_c of a blended hold, on the lower side, before a disagreeing contact exists. It shrinks with ρ_c:

- **Smaller ρ_c (below the ~1 cm straddle band):** the assumption holds almost everywhere, but repeat landings that close are rare because foot placement error is on the order of a centimeter, so the floor rule rarely applies.

- **Larger ρ_c** (about 1 to 2 cm): certificates apply more often, with a small risk near edges.

We will start with ρ_c of about 1 cm. Using the logged straddle flags and ground truth (evaluation only), we will report how many certificates came from straddling contacts, how often a certificate was applied within ρ_c of a later-detected boundary, and how often μ_lo exceeded the true friction, and tighten ρ_c if those occur.

# Stretch goals (parallel)

These are independent and can be owned by different team members once stage 2 is stable. Both plug into the same interface: they change how μ_plan(p) and μ_lo(p) are produced, not the MPC.

## Stretch A: probing sequence in simulation

Replace the assumed probe data with real probing in sim.

- **Probe:** keep three feet planted and slowly ramp the probing foot’s tangential force ratio until slip is detected. The ratio at slip onset is the friction label.

- **Slip detection:** a scheduled-stance foot whose tangential velocity exceeds a threshold above MuJoCo’s baseline contact creep for several steps. Forces come from mj_contactForce.

- **Where to probe:** where σ(p) is high and the tube is limiting the plan, such as near an active obstacle constraint.

- **Safety:** probe only where the current tube leaves margin to obstacles, at low speed, and never on a ramp with friction close to tan θ.

- Unplanned slips during normal walking are also used as equality labels.

## Stretch B: learning with equalities and inequalities

Replace the threshold-radius rule by putting non-slip contacts directly into the GP through a censored (probit-style) likelihood:

$$p\left( hold\ at\ p_{j} \mid f \right) = \Phi\left( \frac{f\left( p_{j} \right) - r_{j}}{\sigma_{n}} \right),\quad\quad p\left( y_{j} \mid f \right) = N\left( y_{j};\ f\left( p_{j} \right),\ \sigma_{n}^{2} \right)$$

- Non-slip data then lower σ and raise the mean, not just set a floor. The certificate layer from the boundaries section stays in place as a safety wrapper.

- Keep the problem tractable by training the GP on the dominant hold per GP cell (the certificate layer still keeps every contact) and, if needed, using a sparse variational GP. Training happens between runs.

- Requires approximate inference (Laplace, expectation propagation or variational). GPyTorch supports custom likelihoods with variational inference.

- Compare against stage 3 on the same logged data: tube width versus number of contacts.

# Evaluation

All controllers run on the same courses and seeds: both mid-ramp variants (a 0.3 or a high-friction patch at the gap), gap widths from about 0.6 to 1.1 m in 0.1 m steps, and 20 seeds per controller per gap width. Friction values come from the discrete set between 0.3 and 0.8.

| **Controller**        | **Friction used**                                                                   | **Expected behavior**                                                |
|-----------------------|-------------------------------------------------------------------------------------|----------------------------------------------------------------------|
| Nominal (stage 1 MPC) | One assumed value (0.5), too high on 0.3 patches                                    | Slips and collides near obstacles                                    |
| Robust (stage 2)      | μ_plan = 0.5, μ_lo = 0.3 everywhere: constant tube                                  | Safe, slower, needs a gap of about 1.0 m                             |
| Learned (stage 3)     | Learned map from probe data plus non-slip bounds, used in the planner and mid layer | Safe, narrower tube, faster, clears gaps down to about 0.6 to 0.75 m |
| Oracle                | True friction map                                                                   | Upper bound on performance                                           |

**Metrics:** outcome per run (success, collision, fall, timeout or no feasible plan); pass rate against gap width and the smallest gap cleared in at least 80% of runs; time to goal on successful runs, overall and on the ramp; collisions and minimum clearance (bare footprint, with no margin or tube, to the nearest obstacle or wall at the logged position and yaw); slip events; fraction of time feet sit at their friction limit; tube radius over the run.

**Key figures:**

- Pass rate and time to goal against mid-ramp gap width, per controller and course variant (validates the learned controller).

- Measured deviation against predicted tube radius across all slip trials (validates stage 2).

- Tube width and time to goal against amount of friction data (validates stage 3).

- If stretch goals are completed: learning curves comparing assumed probing, simulated probing, and equality plus inequality learning.

# Schedule

## Roles

The work splits into three tracks that can run in parallel. Each person owns one track and helps on shared milestones.

| **Member** | **Track**                      | **Owns**                                                                                     |
|------------|--------------------------------|----------------------------------------------------------------------------------------------|
| A (Jacob)  | Simulation and MPC integration | Quadruped-PyMPC, the acados problem, closed-loop runs, Stretch A                             |
| B (Austin) | Planning and robust control    | Collocation planner, tube math, stage 2 validation, figures and report lead                  |
| C (Ben)    | Environment, data and learning | Environment generator, logging and detectors, batch runner, GP, certificate layer, Stretch B |

Jacob owns track A, Austin track B and Ben track C. The mid-layer robust MPC is built by Jacob with Austin.

## Timeline

> **Figure.** Gantt chart of all development steps, colored by owner, with milestones. *(Image in the plan docx on the shared drive.)*

*Gantt chart of all development steps, colored by owner, with milestones.*

**Milestones:**

- **M0, Nov 2:** kickoff, parameter sheet agreed, everyone runs the demo.

- **M1, Nov 11:** stage 1 complete, friction and constraint hooks working.

- **M2, Nov 27** (moved from Nov 24): stage 2 complete and validated. Stretch goals may start only after this.

- **M3, Dec 3:** stage 3 complete with figures.

- **Code freeze, Dec 5.** Submission **Dec 10**, with Dec 8 to 9 as buffer.

**Critical path:** 0.2 → 1.4 → 2.4 → 2.6 → 2.7 → 3.6 → 3.7 → 4.2 → 4.4. A delay on any of these moves the end date, so track A’s integration steps get priority help from the others when they slip. The learning chain 3.1 → 3.2 → 3.3 → 3.5 runs offline on logged data and must land by Nov 25 to feed the closed-loop runs in 3.6.

## Development steps

### Phase 0: Setup (Oct 30 to Nov 4)

| **ID** | **Step**                                                                                                      | **Owner** | **Dates**       | **Needs** | **Done when**                                                                                   |
|--------|---------------------------------------------------------------------------------------------------------------|-----------|-----------------|-----------|-------------------------------------------------------------------------------------------------|
| 0.1    | Shared repo, install Quadruped-PyMPC, MuJoCo and acados on every machine; run the Go2 trot demo               | All       | Oct 30 to Nov 1 | None      | All three members run the demo and record solve times                                           |
| 0.2    | Read Quadruped-PyMPC’s acados problem: locate the friction cone, the constraint list and the parameter vector | A         | Oct 31 to Nov 3 | 0.1       | Written note on how to pass per-foot friction and add linear CoM constraints, or the workaround |
| 0.3    | Environment generator: polygon config (vertices, μ, slope) to MJCF boxes with terrain priority over feet      | C         | Oct 31 to Nov 4 | 0.1       | Flat, patch and ramp courses load; single-foot test confirms terrain friction governs           |
| 0.4    | Choose μ_min, μ_max, μ_nom, ramp angles (μ_min \> tan θ with margin) and the course layout                    | B         | Oct 31 to Nov 2 | None      | One-page parameter sheet agreed at kickoff                                                      |

### Phase 1: Stage 1, nominal MPC on flat ground (Nov 3 to 11)

| **ID** | **Step**                                                                                                                                                        | **Owner** | **Dates**        | **Needs** | **Done when**                                                                          |
|--------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|------------------|-----------|----------------------------------------------------------------------------------------|
| 1.1    | Run Quadruped-PyMPC in our flat environment; goal-directed reference by converting waypoints to velocity commands                                               | A         | Nov 3 to Nov 7   | 0.2, 0.3  | Go2 walks from start to goal on flat ground                                            |
| 1.2    | Contact logger (position, surface-frame forces, foot velocity, contact state) with slip detection; straddle flag recorded for evaluation only                   | C         | Nov 4 to Nov 8   | 0.3       | Log file per run; detectors flag injected test slips and edge landings                 |
| 1.3    | Collocation planner (CasADi + IPOPT, point-mass CoM, acceleration limited by μ = 0.5, three-circle footprint, two-pass tube inflation) and reference conversion | B         | Nov 3 to Nov 10  | 0.4       | Collision-free desired position and velocity around obstacles, passed to the mid layer |
| 1.4    | Plumb per-foot, per-timestep friction and CoM half-space constraints into the acados problem, tested with fixed values                                          | A         | Nov 7 to Nov 11  | 0.2, 1.1  | Changing a foot’s μ or adding a wall constraint changes the solution as expected       |
| 1.5    | Stage 1 exit test: 20 runs on flat ground                                                                                                                       | A         | Nov 10 to Nov 11 | 1.1, 1.2  | At least 19 of 20 runs reach the goal; logs saved                                      |

### Phase 2: Stage 2, tube MPC on mixed terrain (Nov 10 to 27)

| **ID** | **Step**                                                                                                                                                                       | **Owner** | **Dates**        | **Needs** | **Done when**                                                                                                                     |
|--------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|------------------|-----------|-----------------------------------------------------------------------------------------------------------------------------------|
| 2.1    | Horizontal CoM error model, LQR gain K (computed once, offline), Lyapunov ellipsoid template, contraction factor ρ \< 1                                                        | B         | Nov 10 to Nov 13 | 1.3       | ρ and the ellipsoid computed and checked in simulation                                                                            |
| 2.2    | r_w from the acceleration shortfall (μ_plan − μ_lo)g plus tracking error; α_k recursion over the horizon                                                                       | B         | Nov 12 to Nov 16 | 2.1       | Function returning α_0..α_N in meters for given footholds and friction values                                                     |
| 2.3    | Mixed courses from the discrete friction set (0.3 to 0.8) with both mid-ramp variants; batch runner (gap sweep, friction and start seeds, controller, path source) and metrics | C         | Nov 10 to Nov 14 | 0.3, 1.2  | One command runs N seeded trials per gap width and outputs outcome, time to goal, clearance, slips and time at the friction limit |
| 2.4    | Mid-layer robust MPC: point mass, about 2 s horizon, LQR feedback, three-circle half-spaces tightened by α_k; sends velocity and yaw rate to Quadruped-PyMPC (cones at μ_lo)   | A, with B | Nov 12 to Nov 23 | 1.4, 2.2  | Mid layer solves within the real-time budget and drives Quadruped-PyMPC                                                           |
| 2.5    | Empirical disturbance check: one-step CoM errors against analytic r_w, velocity-tracking time constant, time at the friction limit, drift against α, heading error             | B         | Nov 17 to Nov 24 | 2.2, 2.3  | Measured errors stay below r_w, or r_w, K and ρ are recalibrated; opening widths locked                                           |
| 2.6    | End-to-end stage 2: planner inflated by footprint, margin and largest tube, plus the mid-layer tube MPC, on the full courses                                                   | A         | Nov 20 to Nov 25 | 1.3, 2.4  | Robot completes the obstacle courses with the tube active                                                                         |
| 2.7    | Stage 2 validation: 20 seeds per gap width, deviation against tube, robust minimum gap                                                                                         | B         | Nov 23 to Nov 27 | 2.5, 2.6  | No collisions; measured deviation inside the predicted tube; robust minimum gap measured                                          |

### Phase 3: Stage 3, learning from emulated probe data (Nov 15 to Dec 3)

| **ID** | **Step**                                                                                                                                         | **Owner** | **Dates**        | **Needs** | **Done when**                                                             |
|--------|--------------------------------------------------------------------------------------------------------------------------------------------------|-----------|------------------|-----------|---------------------------------------------------------------------------|
| 3.1    | Emulated probe data: footholds along the nominal path labeled with ground-truth μ                                                                | C         | Nov 15 to Nov 17 | 1.2, 2.3  | Probe dataset per course                                                  |
| 3.2    | GP on equalities (RBF kernel, fixed hyperparameters), query API returning m(p) and σ(p)                                                          | C         | Nov 17 to Nov 21 | 3.1       | Offline GP reproduces the course friction along the probed path           |
| 3.3    | Certificate layer: KD-tree of all contacts, floor and cap within ρ_c, disagreement test (ε = 0.05), zones Z; query(x, y) returns μ_plan and μ_lo | C         | Nov 20 to Nov 24 | 1.2, 3.2  | Layer returns μ_plan and μ_lo for any foothold                            |
| 3.4    | Unit tests reproducing the two-region and three-region boundary cases                                                                            | C         | Nov 24 to Nov 25 | 3.3       | Both cases fall back to μ_min where they should                           |
| 3.5    | Feed per-foothold μ_plan and μ_lo into r_w and α_k; μ_lo-scaled speed, turn-rate and acceleration limits in the planner and mid layer            | B         | Nov 23 to Nov 25 | 2.2, 3.3  | Tube narrows and speed limits rise where certificates or probe data exist |
| 3.6    | Between-run loop: run, log, refit, rerun; multi-run experiments on each course                                                                   | A         | Nov 25 to Dec 1  | 2.7, 3.5  | Several consecutive runs per course with the learned map                  |
| 3.7    | Stage 3 results: pass rate against gap width, smallest gap cleared in 80% of runs, time to goal; compare robust, learned and oracle              | All       | Dec 1 to Dec 3   | 3.6       | Stage 3 figures drafted                                                   |

### Stretch goals (Nov 27 to Dec 6, only after milestone M2)

| **ID** | **Step**                                                                                                                                  | **Owner** | **Dates**       | **Needs** | **Done when**                                             |
|--------|-------------------------------------------------------------------------------------------------------------------------------------------|-----------|-----------------|-----------|-----------------------------------------------------------|
| SA     | Stretch A: probing primitive (three feet planted, ramp one foot’s tangential force to slip onset), probe site selection and safety gating | A         | Dec 1 to Dec 6  | 2.7       | Probe labels replace emulated data on at least one course |
| SB     | Stretch B: censored-likelihood GP (GPyTorch, variational), dominant-hold data per GP cell, calibration against stage 3                    | C         | Nov 27 to Dec 6 | 3.3       | Comparison against stage 3 on the same logged data        |

### Phase 4: Evaluation, write-up and submission (Nov 24 to Dec 10)

| **ID** | **Step**                                                                          | **Owner** | **Dates**       | **Needs** | **Done when**                        |
|--------|-----------------------------------------------------------------------------------|-----------|-----------------|-----------|--------------------------------------|
| 4.1    | Report drafting, one section per owner as each stage closes                       | B         | Nov 24 to Dec 7 | None      | Full draft by Dec 7                  |
| 4.2    | Code freeze, final batch runs: nominal, robust, learned and oracle on all courses | C         | Dec 4 to Dec 6  | 3.7       | Final metrics table                  |
| 4.3    | Final figures and demo videos                                                     | B         | Dec 3 to Dec 7  | 4.2       | All figures and videos in the report |
| 4.4    | Assemble, review and submit; Dec 8 to 9 kept as buffer                            | All       | Dec 7 to Dec 10 | 4.1, 4.3  | Submitted Dec 10                     |

## Week by week

| **Week**                    | **A: sim and MPC**                                                   | **B: planning and robust control**                                       | **C: environment and learning**                                            |
|-----------------------------|----------------------------------------------------------------------|--------------------------------------------------------------------------|----------------------------------------------------------------------------|
| Oct 30 to Nov 4             | Install, read the acados problem (0.2)                               | Parameter sheet (0.4)                                                    | Environment generator (0.3)                                                |
| Nov 3 to 9                  | Our environment in the loop, goal reference (1.1)                    | Collocation planner (1.3)                                                | Logger and detectors (1.2)                                                 |
| Nov 10 to 16                | Friction and constraint hooks (1.4), **M1**, start tube in MPC (2.4) | Error model and ρ (2.1), r_w and α (2.2)                                 | Mixed courses and batch runner (2.3), probe data (3.1)                     |
| Nov 17 to 23                | Mid-layer robust MPC (2.4), start end-to-end stage 2 (2.6)           | Disturbance check (2.5), start validation (2.7), start μ into tube (3.5) | GP on equalities (3.2), certificate layer (3.3)                            |
| Nov 24 to 30 (Thanksgiving) | Finish end-to-end stage 2 (2.6), between-run loop (3.6)              | Validation (2.7), M2 on Nov 27, μ into tube (3.5), report drafting (4.1) | Finish certificate layer (3.3), boundary unit tests (3.4), start Stretch B |
| Dec 1 to 7                  | Stage 3 results (3.7), **M3**, Stretch A                             | Figures and videos (4.3), report                                         | Final batch runs (4.2), Stretch B                                          |
| Dec 8 to 10                 | Buffer, review, submit                                               | Buffer, review, submit                                                   | Buffer, review, submit                                                     |

## Checkpoints and fallbacks

- **M1, Nov 11:** if stage 1 is not walking reliably, drop ramps and use flat friction patches only. If per-foot friction cannot be passed to the solver (0.2), edit the acados model directly before anything else.

- **M2, Nov 27:** if stage 2 is not validated, stage 3 runs offline only, showing the learned map shrinking r_w on logged data, and both stretch goals are dropped.

- **M3, Dec 3:** if closed-loop stage 3 is unstable, report the offline results from 3.2 to 3.5.

- **Code freeze, Dec 5:** stretch results enter the report only if they work by then.

- **Thanksgiving week** runs at reduced capacity. M2 moved to Nov 27, so stage 2 validation (2.7) is the one critical-path step that finishes that week.

# Decisions

All other design questions were settled in the team’s decisions sheet in the shared drive, which records each question, the decision, its owner and when it closed. The sections above reflect those decisions. One item remains open:

1.  GP settings (library, kernel length scale, β, refit frequency, ρ_c) are still open; they are decided after the robust parts are complete.

# Risks

| **Risk**                                                                       | **Mitigation**                                                                                                                     |
|--------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------|
| Quadruped-PyMPC does not expose per-foot friction or position constraints      | Resolved in 1.4: per-foot friction and wall constraints work                                                                       |
| Go2 cannot hold position on the chosen ramps                                   | Choose ramp angle from μ_min with margin, test early                                                                               |
| Tube too wide, making the course infeasible                                    | Sweep the mid-ramp gap (0.6 to 1.1 m); record "no feasible plan" as its own outcome; openings at the ramp ends stay at least 1.2 m |
| Feet spend long periods at the μ_lo friction limit, so drift exceeds the tube  | 2.5 measures time at the limit and drift against α; raise r_0 or lower the mid-layer limits                                        |
| Quadruped-PyMPC lags the commanded velocity, breaking the error model behind K | Measure the time constant in 2.5; add the lag to the error model and recompute K and ρ                                             |
| MuJoCo friction combining overrides terrain values                             | Use geom priority, verify with a single-foot friction test                                                                         |
| Schedule slip around Thanksgiving                                              | Stage 3 offline fallback, stretch goals only after stage 2                                                                         |
