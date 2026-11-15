"""Per-step contact logging and slip detection for gym-quadruped / Quadruped-PyMPC.

Call `logger.record(env)` once per simulation step (after env.step). At the end, `logger.events()` returns one row per
stance phase (touchdown to liftoff) for each foot. Every stance is classified from measured quantities only (force
ratio and foot velocity), exactly as a real robot would:

  kind = "hold"  stance with no slip: certificate mu >= r (the largest force ratio used during the stance)
  kind = "slip"  stance that slipped: label mu ~= r (force ratio at slip onset)

Two kinds of columns, kept apart on purpose:

  LEARNER_FIELDS  what a real robot can measure: foot position, total foot force, foot velocity, contact state.
                  The surface normal used to split the force is read from the simulator; this stands in for a
                  slope estimate (stated assumption in the project plan).
  EVAL_FIELDS     simulator ground truth: which terrain region(s) the foot touched, the straddle flag, true mu.
                  Use only for evaluation and for the emulated probe labels of stage 3, never as learner input.

A foot that touches two regions at once (a "straddle", within about 1 cm of a boundary) still becomes a hold or a
slip with a blended ratio, because the robot cannot tell. Boundaries must be found by the disagreement test in the
certificate layer. The straddle flag exists only to measure afterwards how often this happened and whether it hurt.
`learner_events()` returns the events with the evaluation columns removed; feed the learner from that.
"""
from collections import defaultdict
from dataclasses import dataclass, field

import mujoco
import numpy as np

LEGS = ("FL", "FR", "RL", "RR")
LEARNER_FIELDS = ("leg", "t0", "t1", "x", "y", "z", "kind", "r", "slip_t", "fn_mean")
EVAL_FIELDS = ("surface", "straddle", "mu_true", "mu_true_min", "mu_true_max")


@dataclass
class _Stance:
    t0: float
    pos0: np.ndarray
    geoms: set = field(default_factory=set)
    surfaces: set = field(default_factory=set)
    r_max_noslip: float = 0.0
    slip_count: int = 0
    slip_r: float = np.nan
    slip_t: float = np.nan
    slipped: bool = False
    straddle: bool = False                                # evaluation only
    fn_sum: float = 0.0
    n: int = 0


class ContactLogger:
    def __init__(self, env, v_slip=0.05, k_slip=5, fz_min=5.0):
        """
        v_slip : [m/s] tangential foot speed that counts as sliding. Tune above MuJoCo's soft-contact creep.
        k_slip : consecutive steps above v_slip before a slip is declared (rejects touchdown transients).
        fz_min : [N] normal force below which the force ratio is not trusted (touchdown/liftoff).
        """
        self.env, self.v_slip, self.k_slip, self.fz_min = env, v_slip, k_slip, fz_min
        m = env.mjModel
        # Terrain tiles are named "surf_<region>__<k>" by terrain_gen.py. Map geom id -> region id so that an internal
        # seam between two tiles of the same region is not mistaken for a boundary.
        self.surface = {i: m.geom(i).name.split("__")[0] for i in range(m.ngeom)
                        if (m.geom(i).name or "").startswith("surf_")}
        self.terrain_ids = set(self.surface)
        # Walls and obstacles: any robot contact with these counts as a collision (parameter sheet v0.3).
        self.barrier = {i: m.geom(i).name for i in range(m.ngeom)
                        if (m.geom(i).name or "").startswith(("wall_", "obstacle_"))}
        self.collisions = []                                   # (time, barrier geom name), one entry per contact
        self.steps = defaultdict(list)
        self._open = {leg: None for leg in LEGS}
        self._events = []

    # ------------------------------------------------------------------ per step
    def record(self, env=None, extra: dict | None = None):
        env = env or self.env
        m, d = env.mjModel, env.mjData
        t = d.time
        in_contact, contacts, forces = env.feet_contact_state(ground_reaction_forces=True)
        pos, vel = env.feet_pos(frame="world"), env.feet_vel(frame="world")
        self._record_collisions(m, d)
        for leg in LEGS:
            geoms, normals = set(), []
            for c in contacts[leg]:
                for g in (c.geom1, c.geom2):
                    if g in self.terrain_ids:
                        geoms.add(int(g))
                        nrm = np.array(c.frame[:3])            # contact normal (first row of the contact frame)
                        normals.append(nrm if nrm[2] > 0 else -nrm)
            # Friction acts relative to the SURFACE, so split force and velocity into the part along the surface
            # normal n and the part tangent to it. On flat ground n = (0, 0, 1) and this reduces to |f_xy| / f_z;
            # on the 10 deg ramp the world-frame version would be wrong by roughly tan(10 deg) = 0.18.
            n = np.mean(normals, axis=0) if normals else np.array([0.0, 0.0, 1.0])
            n /= np.linalg.norm(n)
            f = np.asarray(forces[leg], dtype=float)
            fn = float(f @ n)                                  # normal force pressing the foot into the ground
            ft = float(np.linalg.norm(f - fn * n))             # tangential (friction) force
            r = ft / fn if fn > self.fz_min else np.nan
            v = np.asarray(vel[leg], dtype=float)
            v_t = float(np.linalg.norm(v - (v @ n) * n))       # foot speed along the surface
            p = np.asarray(pos[leg], dtype=float)
            # Ground truth for evaluation only: which regions the foot touched. Not used to classify the stance.
            surfaces = {self.surface[g] for g in geoms}
            row = dict(t=t, leg=leg, contact=bool(in_contact[leg]), geoms=tuple(sorted(geoms)), fx=f[0], fy=f[1],
                       fz=f[2], nx=n[0], ny=n[1], nz=n[2], fn=fn, ft=ft, r=r, v_t=v_t, x=p[0], y=p[1], z=p[2], surfaces=tuple(sorted(surfaces)),
                       straddle=len(surfaces) > 1,
                       mu_true=float(m.geom_friction[next(iter(geoms)), 0]) if len(surfaces) == 1 else np.nan)
            if extra:
                row.update(extra)
            self.steps[leg].append(row)
            self._update_stance(leg, row)

    def _record_collisions(self, m, d):
        """Scan MuJoCo's active contacts for robot-vs-barrier pairs. Static geoms belong to body 0 (the world), so a
        barrier contact whose other geom is on any other body is the robot touching a wall or an obstacle."""
        for i in range(d.ncon):
            c = d.contact[i]
            for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if a in self.barrier and m.geom_bodyid[b] != 0:
                    self.collisions.append((float(d.time), self.barrier[a]))

    def collided(self) -> bool:
        return len(self.collisions) > 0

    def _update_stance(self, leg, row):
        s = self._open[leg]
        if row["contact"] and s is None:                                   # touchdown
            s = self._open[leg] = _Stance(t0=row["t"], pos0=np.array([row["x"], row["y"], row["z"]]))
        if s is None:
            return
        if not row["contact"]:                                              # liftoff: close the event
            self._close(leg, s, row["t"])
            self._open[leg] = None
            return
        s.geoms |= set(row["geoms"])
        s.surfaces |= set(row["surfaces"])
        s.straddle |= row["straddle"]                                       # recorded, never used below
        s.n += 1
        s.fn_sum += row["fn"]
        sliding = row["v_t"] > self.v_slip and not np.isnan(row["r"])
        s.slip_count = s.slip_count + 1 if sliding else 0
        if s.slip_count >= self.k_slip and not s.slipped:                  # slip onset
            s.slipped, s.slip_t, s.slip_r = True, row["t"], row["r"]
        if not sliding and not s.slipped and not np.isnan(row["r"]):
            s.r_max_noslip = max(s.r_max_noslip, row["r"])

    def _close(self, leg, s, t1):
        # Classification uses measured data only: did the foot slide (velocity), and at what force ratio.
        kind = "slip" if s.slipped else "hold"
        # Ground truth for evaluation. For a straddle, mu_true is undefined; min/max give the two sides, so a hold
        # with r > mu_true_min is a certificate that overstates the lower-friction side.
        mus = [float(self.env.mjModel.geom_friction[g, 0]) for g in s.geoms]
        self._events.append(dict(
            leg=leg, t0=s.t0, t1=t1, x=s.pos0[0], y=s.pos0[1], z=s.pos0[2], kind=kind,
            r=s.slip_r if s.slipped else s.r_max_noslip, slip_t=s.slip_t, fn_mean=s.fn_sum / max(s.n, 1),
            surface="|".join(sorted(s.surfaces)), straddle=s.straddle,
            mu_true=mus[0] if len(s.surfaces) == 1 else np.nan,
            mu_true_min=min(mus) if mus else np.nan, mu_true_max=max(mus) if mus else np.nan))

    # ------------------------------------------------------------------ outputs
    def events(self):
        """List of dicts, one per completed stance phase, including the evaluation-only columns."""
        return list(self._events)

    def learner_events(self):
        """The same events with only LEARNER_FIELDS: everything the learning pipeline is allowed to see."""
        return [{k: e[k] for k in LEARNER_FIELDS} for e in self._events]

    def save(self, path):
        """Save events and per-step data to a compressed .npz (load with np.load(path, allow_pickle=True))."""
        np.savez_compressed(path, events=np.array(self._events, dtype=object),
                            collisions=np.array(self.collisions, dtype=object),
                            steps=np.array({k: v for k, v in self.steps.items()}, dtype=object))
