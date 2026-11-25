"""Course files: the team's friction rules, the parametric mid-ramp gap, and loading a course for any module.

Every module that reads a course YAML (terrain generator, planner converter, batch runner) goes through
`load_course_cfg`, so they all see the same expanded course: the same friction values, the same obstacles and the
same scene name.

Friction (decision C3). Every surface takes one value from FRICTION_SET. The lower limit 0.3 applies everywhere,
flat and ramp. Classes are fixed here for the whole team, not per course file:

    class    unseeded   seeded draw (uniform over the list)
    low      0.3        0.3
    medium   0.5        0.4, 0.5, 0.6
    high     0.8        0.7, 0.8

A region gives either `class` (drawn per seed) or an explicit `mu` (fixed, must be in FRICTION_SET). Everything
outside a region is base ground at `friction.base` (default 0.5, the nominal value), which seeds never change.

Mid-ramp gap (decision A3.1). A course may have a `mid_ramp_gap` block. It becomes one obstacle (the barrier,
attached to the right wall y = 0, leaving `gap` to the left wall) and one region (the patch around the gap, of
class `variant`). The batch runner sweeps `gap` and `variant` by passing overrides here; the scene name then
carries both so swept scenes never overwrite each other.
"""
import copy
import random

import yaml

FRICTION_SET = (0.3, 0.4, 0.5, 0.6, 0.7, 0.8)
MU_MIN, MU_MAX, MU_NOMINAL = 0.3, 0.8, 0.5
FRICTION_CLASSES = {                     # class: (unseeded value, values a seed can draw)
    "low": (0.3, (0.3,)),
    "medium": (0.5, (0.4, 0.5, 0.6)),
    "high": (0.8, (0.7, 0.8)),
}
GAP_VARIANTS = ("low", "high")           # friction class of the patch around the mid-ramp gap

_OLD_KEYS = ("friction_classes", "randomize_ranges")
_OLD_COURSE_KEYS = ("ramp_mu_floor", "base_class")


class CourseSpecError(ValueError):
    pass


def check_schema(cfg: dict):
    """Reject course files written for the old continuous friction ranges, with the fix in the message."""
    old = [k for k in _OLD_KEYS if k in cfg] + [f"course.{k}" for k in _OLD_COURSE_KEYS if k in cfg["course"]]
    if old:
        raise CourseSpecError(
            f"course '{cfg.get('name')}' uses the old friction keys {old}. Friction now follows decision C3: delete "
            "them, add `friction: {base: 0.5}`, and give each region a `class` (low / medium / high) or an `mu` from "
            f"{list(FRICTION_SET)}. See trackC/course_spec.py.")
    for r in cfg.get("regions", []):
        if ("class" in r) == ("mu" in r):
            raise CourseSpecError(f"region {r['id']}: give exactly one of `class` or `mu`")
        if "class" in r and r["class"] not in FRICTION_CLASSES:
            raise CourseSpecError(f"region {r['id']}: unknown class {r['class']!r}; use {list(FRICTION_CLASSES)}")
        if "mu" in r:
            check_mu(r["mu"], f"region {r['id']}")
    check_mu(cfg.get("friction", {}).get("base", MU_NOMINAL), "friction.base")


def check_mu(mu, where: str):
    if not any(abs(mu - v) < 1e-9 for v in FRICTION_SET):
        raise CourseSpecError(f"{where}: mu {mu} is not in the friction set {list(FRICTION_SET)} (decision C3)")


def resolve_friction(cfg: dict, seed=None) -> dict:
    """{surface id: mu} for "base" and every region. The same seed always gives the same values.

    Draws go through one random.Random(seed) in region order, so adding a region to the end of a course file does
    not change the values the earlier regions get.
    """
    rng = random.Random(seed)
    out = {"base": cfg.get("friction", {}).get("base", MU_NOMINAL)}
    for r in cfg.get("regions", []):
        if "mu" in r:
            out[r["id"]] = r["mu"]
        else:
            nominal, choices = FRICTION_CLASSES[r["class"]]
            out[r["id"]] = nominal if seed is None else rng.choice(choices)
    return out


def expand_mid_ramp_gap(cfg: dict) -> dict:
    """Turn a `mid_ramp_gap` block into the barrier obstacle and the gap patch region (in place)."""
    g = cfg.get("mid_ramp_gap")
    if not g or cfg.get("_expanded"):
        return cfg
    cfg["_expanded"] = True
    width = cfg["course"]["width"]
    gap, variant = float(g["gap"]), g["variant"]
    if not 0.0 < gap < width:
        raise CourseSpecError(f"mid_ramp_gap.gap {gap} must be between 0 and the course width {width}")
    if variant not in GAP_VARIANTS:
        raise CourseSpecError(f"mid_ramp_gap.variant {variant!r} must be one of {GAP_VARIANTS}")
    x, t = g["x"], g["thickness"]
    y_end = width - gap                                    # the barrier spans y = 0 .. y_end; the gap is y_end .. width
    cfg.setdefault("obstacles", []).append(
        {"id": "barrier", "center": [x, y_end / 2], "size": [t, y_end], "height": g["height"]})
    p = g["patch"]
    xa, xb, ya = x - t / 2 - p["before"], x + t / 2 + p["after"], max(0.0, y_end - p["margin"])
    cfg.setdefault("regions", []).append(
        {"id": "gap_patch", "class": variant, "polygon": [[xa, ya], [xb, ya], [xb, width], [xa, width]]})
    return cfg


def scene_name(cfg: dict, seed=None, overridden=False) -> str:
    """Scene name for gym_quadruped. Gap and variant are added only when overridden, so a course built with its file
    defaults keeps the plain name that run_waypoints looks up."""
    name = cfg["name"]
    g = cfg.get("mid_ramp_gap")
    if g and overridden:
        name += f"_g{float(g['gap']):.2f}_{g['variant']}"
    return name + ("" if seed is None else f"_s{seed}")


def load_course_cfg(source, gap=None, variant=None) -> dict:
    """Course dict ready for any module: schema checked, overrides applied, mid-ramp gap expanded.

    source: a path to a course YAML or an already-loaded dict (never modified). gap, variant: override the file's
    `mid_ramp_gap` values (only for courses that have one). Adds `_overridden` so scene_name can tell.
    """
    if isinstance(source, dict):
        cfg = copy.deepcopy(source)
    else:
        with open(source) as f:
            cfg = yaml.safe_load(f)
    check_schema(cfg)
    if gap is not None or variant is not None:
        if "mid_ramp_gap" not in cfg:
            raise CourseSpecError(f"course '{cfg['name']}' has no mid_ramp_gap to override")
        if gap is not None:
            cfg["mid_ramp_gap"]["gap"] = gap
        if variant is not None:
            cfg["mid_ramp_gap"]["variant"] = variant
        cfg["_overridden"] = True
    return expand_mid_ramp_gap(cfg)
