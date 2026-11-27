"""Friction lookup for the planner and the tube (decision B2).

Every consumer asks one question, `query(x, y) -> (mu_plan, mu_lo)`, at world-frame plan-view points (A1):

    mu_plan   friction the mid layer plans motion with
    mu_lo     certified lower bound; Quadruped-PyMPC's cones and the tube's r_w use it

PriorFrictionMap is what stage 2 runs on: before learning, mu_plan = 0.5 and mu_lo = 0.3 everywhere, so the tube
has one constant width. The stage 3 certificate layer (task 3.3) will be another class with the same two methods,
so callers do not change when learning is switched on.
"""
import numpy as np

from trackC.course_spec import MU_MIN, MU_NOMINAL

MU_PLAN_PRIOR = MU_NOMINAL    # 0.5: the middle of the friction set and the planner's nominal value
MU_LO_PRIOR = MU_MIN          # 0.3: the lower limit everywhere (decision C3)


class PriorFrictionMap:
    """Stage 2 friction map: the prior everywhere, before any contact data."""

    def __init__(self, mu_plan=MU_PLAN_PRIOR, mu_lo=MU_LO_PRIOR):
        if not 0.0 < mu_lo <= mu_plan:
            raise ValueError(f"need 0 < mu_lo <= mu_plan, got mu_lo {mu_lo}, mu_plan {mu_plan}")
        self.mu_plan, self.mu_lo = float(mu_plan), float(mu_lo)

    def query(self, x: float, y: float) -> tuple:
        """(mu_plan, mu_lo) at one point."""
        return self.mu_plan, self.mu_lo

    def query_many(self, xy) -> tuple:
        """Arrays (mu_plan, mu_lo), each shape (N,), for points xy of shape (N, 2): e.g. footholds over a horizon."""
        n = np.asarray(xy, dtype=float).reshape(-1, 2).shape[0]
        return np.full(n, self.mu_plan), np.full(n, self.mu_lo)


_PRIOR = PriorFrictionMap()


def query(x: float, y: float) -> tuple:
    """Module-level shortcut for the prior map: query(x, y) -> (0.5, 0.3)."""
    return _PRIOR.query(x, y)
