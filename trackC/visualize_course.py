"""Draw a course: plan view (regions, the triangles they were split into, walls, obstacles, start, goal) and a side
profile. Use it to check a layout before simulating, and to see exactly how the generator cut the regions up.

Usage (from the repo root): python -m trackC.visualize_course trackC/courses/debug_v03.yaml [--seed N] [--out course.png]
"""
import argparse
import math

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.patches import Rectangle

from trackC.terrain_gen import build, triangulate

ap = argparse.ArgumentParser()
ap.add_argument("config")
ap.add_argument("--seed", type=int, default=None)
ap.add_argument("--out", default="course.png")
ap.add_argument("--title", default="Example course design", help="figure title")
a = ap.parse_args()
cfg = yaml.safe_load(open(a.config))
_, info = build(cfg, a.seed)
course, mu = info["course"], info["mu"]
cmap, norm = plt.get_cmap("Blues"), Normalize(0.0, 0.9)

fig, (ax, ax2) = plt.subplots(2, 1, figsize=(13, 7.8), gridspec_kw={"height_ratios": [3, 1.3]})
fig.suptitle(f"{a.title}: {info['name']}", fontsize=15, fontweight="bold", x=0.01, y=0.99, ha="left", va="top")
fig.text(0.01, 0.945, va="top", s= "One layout. Region shapes, friction values and obstacle positions are set per course file (friction values also per seed); obstacles may be placed on any section, including the ramp.", fontsize=9, color="#444444")

# plan view: every triangle, colored by its surface's friction; thin white edges show the triangulation
for sid, sec, shape in info["pieces"]:
    for tri in triangulate(shape):
        ax.add_patch(MplPolygon(tri, closed=True, fc=cmap(norm(mu[sid])), ec="white", lw=0.4))
for r in cfg["regions"]:                                   # region outlines and labels
    poly = np.array(r["polygon"])
    ax.add_patch(MplPolygon(poly, closed=True, fill=False, ec="#222222", lw=1.2))
    cx, cy = poly.mean(axis=0)
    ax.text(cx, cy, f"{r['id']}\n{mu[r['id']]:.2f}", ha="center", va="center", fontsize=7.5,
            color="white" if mu[r["id"]] > 0.55 else "#111111")
for xc, label in ((course.x1, "bottom crease"), (course.x2, "top crease")):
    ax.axvline(xc, color="#990000", ls="--", lw=1)
    ax.text(xc, course.width / 2 + 0.28, label, color="#990000", ha="center", fontsize=8)
t = cfg["course"]["walls"]["thickness"]
w2 = course.width / 2
for y0 in (w2, -w2 - t):
    ax.add_patch(Rectangle((course.x0 - t, y0), course.x3 - course.x0 + 2 * t, t, color="#888888"))
for x0 in (course.x0 - t, course.x3):
    ax.add_patch(Rectangle((x0, -w2 - t), t, course.width + 2 * t, color="#888888"))
for o in cfg.get("obstacles", []):
    (x, y), (L, W) = o["center"], o["size"]
    ax.add_patch(Rectangle((x - L / 2, y - W / 2), L, W, color="#333333"))
(sx, sy), (gx, gy) = cfg["course"]["start"], cfg["course"]["goal"]
ax.plot(sx, sy, "o", color="#00A000", ms=10)
ax.plot(gx, gy, "*", color="#D4A000", ms=16)
ax.text(sx, sy - 0.32, "start", ha="center", fontsize=8)
ax.text(gx, gy - 0.35, "goal", ha="center", fontsize=8)
ax.set_xlim(course.x0 - 0.4, course.x3 + 0.4)
ax.set_ylim(-w2 - 0.45, w2 + 0.5)
ax.set_aspect("equal")
ax.set_title(f"plan view: {info['n_tiles']} tiles (white edges = triangulation)", loc="left")
ax.set_ylabel("y [m]")
fig.colorbar(ScalarMappable(norm=norm, cmap=cmap), ax=ax, label="friction coefficient", fraction=0.025, pad=0.01)

# side profile
xs = np.linspace(course.x0, course.x3, 400)
zs = [course.z(x) for x in xs]
ax2.fill_between(xs, -0.1, zs, color="#C9D6E8")
ax2.plot(xs, zs, color="#011F5B", lw=2)
h = cfg["course"]["walls"]["height"]
ax2.plot(xs, np.array(zs) + h, color="#888888", ls="--", lw=1, label=f"wall top ({h} m above surface)")
ax2.text((course.x1 + course.x2) / 2, course.H / 2 - 0.25,
         f"ramp {math.degrees(course.theta):.0f} deg, rises {course.H:.2f} m", ha="center", fontsize=8)
ax2.set_xlim(ax.get_xlim())
ax2.set_xlabel("x along the course [m]")
ax2.set_ylabel("z [m]")
ax2.legend(fontsize=8, loc="upper left", frameon=False)
ax2.set_title("side profile", loc="left")
plt.tight_layout(rect=(0, 0, 1, 0.93))
plt.savefig(a.out, dpi=150)
print(f"saved {a.out}")
