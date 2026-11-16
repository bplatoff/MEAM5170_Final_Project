"""Analysis of a step 1.5 exit-test folder (written by exit_test_1_5.py). Can be re-run on any old folder.

    python -m trackA.sim.analyze_1_5 data/exit_test_1_5/<timestamp>

Per run (and summed per path type):
  outcome            goal / timeout / terminated (body touched the world or left the terrain) / crashed
  tracking           cross-track error |e| from the path follower: mean, RMS, 95th percentile, max, and the mean
                     after SETTLE_S (the first seconds include recovering from the spawn jitter); speed error
                     |v_xy - v_ref_xy|
  wall clearance     smallest distance from the CoM to any inner wall face (plan view)
  wall hits          robot contacts with wall_ / obstacle_ geoms, merged into events (contacts less than
                     HIT_GAP_S apart with the same wall are one event): time, wall, robot bodies, max penetration
  windows            for every hit event: all logged states and commands over the HIT_WINDOW_S before it,
                     saved to windows/<run>_hit<k>.npz and plotted
  MPC                solves, solve time (mean, 99th pct, max) against the solve interval, failed solves
                     (acados status other than 0 = converged and 2 = iteration limit, which is normal here)
  stance / slips     from the Track C contact logger. Its detector also flags the short slide right after
                     touchdown, so slips starting within TOUCHDOWN_S of touchdown are counted separately

Writes summary.md, summary.csv, trajectories.png, tracking.png and windows/ into the folder.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml

from trackC.course import Course

SETTLE_S = 3.0        # [s] tracking stats "settled" exclude the first seconds (spawn jitter recovery)
HIT_GAP_S = 0.1       # [s] contacts with the same wall closer than this belong to one hit event
HIT_WINDOW_S = 5.0    # [s] states and commands saved before each hit
TOUCHDOWN_S = 0.05    # [s] a slip starting this soon after touchdown is counted as a touchdown slide
PASS_FRACTION = 19 / 20
LEGS = ("FL", "FR", "RL", "RR")

# Per-step arrays copied into each hit window (everything with one row per simulation step)
STEP_KEYS = ("t", "base_pos", "com_pos", "rpy", "base_lin_vel", "base_ang_vel", "ref_lin_vel", "ref_ang_vel",
             "cross_track_error", "seg_idx", "feet_pos", "feet_vel", "feet_contact", "feet_grf", "mpc_grf")


# ---------------------------------------------------------------- per run
def wall_clearance(xy: np.ndarray, course: Course) -> np.ndarray:
    """Distance from each (x, y) to the nearest inner wall face, plan view. Negative means past the face."""
    hw = course.width / 2
    return np.min(np.stack([hw - xy[:, 1], hw + xy[:, 1], xy[:, 0] - course.x0, course.x3 - xy[:, 0]]), axis=0)


def hit_events(d) -> list[dict]:
    t, wall = d["wall_hit_t"], d["wall_hit_barrier"]
    body, pen = d["wall_hit_body"], d["wall_hit_penetration"]
    events = []
    for k in np.argsort(t, kind="stable"):
        ev = events[-1] if events else None
        if ev and ev["wall"] == wall[k] and t[k] - ev["t_end"] <= HIT_GAP_S:
            ev["t_end"] = float(t[k])
            ev["bodies"].add(str(body[k]))
            ev["max_penetration"] = max(ev["max_penetration"], float(pen[k]))
            ev["n_contacts"] += 1
        else:
            events.append(dict(t_start=float(t[k]), t_end=float(t[k]), wall=str(wall[k]), bodies={str(body[k])},
                               max_penetration=float(pen[k]), n_contacts=1))
    return events


def analyze_run(folder: Path, r: dict, course: Course) -> dict:
    row = dict(tag=r["tag"], path=r["path"], seed=r["seed"])
    npz = folder / f"{r['tag']}.npz"
    if r.get("returncode", 0) != 0 or not npz.exists():
        row.update(outcome="crashed", reached_goal=False)
        return row
    d = np.load(npz, allow_pickle=True)
    t = d["t"]
    e = np.abs(d["cross_track_error"])
    settled = t >= t[0] + SETTLE_S
    v_err = np.linalg.norm(d["base_lin_vel"][:, :2] - d["ref_lin_vel"][:, :2], axis=1)
    clearance = wall_clearance(d["com_pos"][:, :2], course)

    solve_t, status, solve_at = d["mpc_solve_time"], d["mpc_status"], d["mpc_t"]
    interval = float(np.median(np.diff(solve_at))) if len(solve_at) > 1 else np.nan

    events = list(np.load(folder / f"{r['tag']}_contacts.npz", allow_pickle=True)["events"])
    slips = [ev for ev in events if ev["kind"] == "slip"]
    late_slips = [ev for ev in slips if ev["slip_t"] - ev["t0"] > TOUCHDOWN_S]

    hits = hit_events(d)
    outcome = str(d["summary_outcome"])
    row.update(
        outcome=outcome, reached_goal=outcome == "goal",
        time_s=float(d["summary_time"]), final_dist_m=float(d["summary_final_dist_to_goal"]),
        spawn_dx=float(d["spawn_jitter"][0]), spawn_dy=float(d["spawn_jitter"][1]),
        spawn_dyaw_deg=float(np.rad2deg(d["spawn_jitter"][2])),
        cte_mean=float(e.mean()), cte_rms=float(np.sqrt(np.mean(e ** 2))), cte_p95=float(np.percentile(e, 95)),
        cte_max=float(e.max()), cte_mean_settled=float(e[settled].mean()) if settled.any() else np.nan,
        speed_err_mean=float(v_err.mean()),
        min_wall_clearance=float(clearance.min()),
        min_side_clearance=float((course.width / 2 - np.abs(d["com_pos"][:, 1])).min()),
        wall_hit_events=len(hits), wall_hit_contacts=int(sum(h["n_contacts"] for h in hits)),
        wall_hit_max_pen=max((h["max_penetration"] for h in hits), default=0.0),
        mpc_solves=len(solve_t), mpc_interval_ms=1e3 * interval,
        mpc_mean_ms=1e3 * float(np.mean(solve_t)), mpc_p99_ms=1e3 * float(np.percentile(solve_t, 99)),
        mpc_max_ms=1e3 * float(np.max(solve_t)), mpc_over_interval=int(np.sum(solve_t > interval)),
        mpc_failed=int(np.sum(~np.isin(status, (0, 2)))),
        stances=len(events), slips_logger=len(slips), slips_after_touchdown=len(late_slips),
        terminated_by="; ".join(map(str, d["terminated_info"])),
    )
    row["_hits"] = hits
    row["_data"] = d
    return row


def save_windows(folder: Path, row: dict) -> list[str]:
    """For each hit event, save and plot every logged step in the HIT_WINDOW_S before it."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d, out = row["_data"], folder / "windows"
    names = []
    for k, h in enumerate(row["_hits"]):
        out.mkdir(exist_ok=True)
        m = (d["t"] >= h["t_start"] - HIT_WINDOW_S) & (d["t"] <= h["t_start"])
        name = f"{row['tag']}_hit{k}"
        np.savez_compressed(out / f"{name}.npz", hit_t=h["t_start"], wall=h["wall"],
                            bodies=np.array(sorted(h["bodies"])), max_penetration=h["max_penetration"],
                            **{key: d[key][m] for key in STEP_KEYS})
        tt = d["t"][m] - h["t_start"]
        fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
        ax[0].plot(tt, d["com_pos"][m, 0], label="CoM x")
        ax[0].plot(tt, d["com_pos"][m, 1], label="CoM y")
        ax[0].set_ylabel("position [m]")
        for i, c in enumerate(("x", "y")):
            ax[1].plot(tt, d["base_lin_vel"][m, i], label=f"v_{c}", color=f"C{i}")
            ax[1].plot(tt, d["ref_lin_vel"][m, i], "--", label=f"command v_{c}", color=f"C{i}")
        ax[1].plot(tt, d["base_ang_vel"][m, 2], label="yaw rate", color="C2")
        ax[1].plot(tt, d["ref_ang_vel"][m, 2], "--", label="command yaw rate", color="C2")
        ax[1].set_ylabel("velocity [m/s, rad/s]")
        ax[2].plot(tt, d["cross_track_error"][m], color="C3", label="cross-track error (+ = left of path)")
        ax[2].set_ylabel("cross-track error [m]")
        for i, leg in enumerate(LEGS):
            ax[3].plot(tt, d["mpc_grf"][m, i, 2], label=f"{leg} f_z (MPC)")
        ax[3].set_ylabel("GRF [N]")
        ax[3].set_xlabel(f"time relative to hit [s] (wall {h['wall']}, bodies {', '.join(sorted(h['bodies']))})")
        for a in ax:
            a.axvline(0, color="r", lw=1)
            a.legend(fontsize=7, loc="upper left")
            a.grid(alpha=0.3)
        fig.suptitle(f"{name}: {HIT_WINDOW_S:.0f} s before the hit")
        fig.tight_layout()
        fig.savefig(out / f"{name}.png", dpi=110)
        plt.close(fig)
        names.append(name)
    return names


# ---------------------------------------------------------------- whole folder
def plot_overview(folder: Path, rows: list[dict], course: Course):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    paths = sorted({r["path"] for r in rows})
    fig, axes = plt.subplots(len(paths), 1, figsize=(11, 3.6 * len(paths)), squeeze=False)
    hw = course.width / 2
    for ax, p in zip(axes[:, 0], paths):
        for r in (r for r in rows if r["path"] == p and "_data" in r):
            d = r["_data"]
            color = "C0" if r["reached_goal"] else "C3"
            ax.plot(d["com_pos"][:, 0], d["com_pos"][:, 1], color=color, lw=0.8, alpha=0.7)
            for h in r["_hits"]:
                i = np.searchsorted(d["t"], h["t_start"])
                ax.plot(*d["com_pos"][min(i, len(d["t"]) - 1), :2], "x", color="r", ms=8)
        wp = next(r["_data"]["waypoints"] for r in rows if r["path"] == p and "_data" in r)
        ax.plot(wp[:, 0], wp[:, 1], "o--", color="0.5", lw=1, label="waypoint path")
        ax.plot([course.x0, course.x3, course.x3, course.x0, course.x0], [-hw, -hw, hw, hw, -hw], color="k", lw=2)
        n_ok = sum(r["reached_goal"] for r in rows if r["path"] == p)
        n = sum(r["path"] == p for r in rows)
        ax.set_title(f"{p}: {n_ok}/{n} reached the goal (blue: goal, red: not; x: wall hit)")
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
    fig.tight_layout()
    fig.savefig(folder / "trajectories.png", dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 4))
    data = [np.abs(r["_data"]["cross_track_error"]) for r in rows if "_data" in r]
    labels = [r["tag"].replace("run_", "") for r in rows if "_data" in r]
    ax.boxplot(data, showfliers=False)
    ax.set_xticks(range(1, len(labels) + 1), labels, rotation=60, fontsize=7)
    ax.set_ylabel("|cross-track error| [m]")
    ax.set_title("tracking per run (box: quartiles, whiskers: 1.5 IQR)")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(folder / "tracking.png", dpi=120)
    plt.close(fig)


CSV_COLS = ("tag", "path", "seed", "outcome", "time_s", "final_dist_m", "spawn_dx", "spawn_dy", "spawn_dyaw_deg",
            "cte_mean", "cte_mean_settled", "cte_rms", "cte_p95", "cte_max", "speed_err_mean", "min_wall_clearance", "min_side_clearance",
            "wall_hit_events", "wall_hit_contacts", "wall_hit_max_pen", "mpc_solves", "mpc_interval_ms",
            "mpc_mean_ms", "mpc_p99_ms", "mpc_max_ms", "mpc_over_interval", "mpc_failed", "stances", "slips_logger",
            "slips_after_touchdown", "terminated_by")


def fmt(v, nd=3):
    if isinstance(v, float):
        return "nan" if np.isnan(v) else f"{v:.{nd}f}"
    return str(v)


def group_stats(rows: list[dict]) -> dict:
    ok = [r for r in rows if "_data" in r]
    goal = [r for r in ok if r["reached_goal"]]
    g = lambda k, f: f([r[k] for r in ok]) if ok else np.nan  # noqa: E731
    return dict(
        runs=len(rows), goal=len(goal),
        time_mean=float(np.mean([r["time_s"] for r in goal])) if goal else np.nan,
        cte_mean=g("cte_mean", np.mean), cte_mean_settled=g("cte_mean_settled", np.mean),
        cte_p95=g("cte_p95", np.mean), cte_max=g("cte_max", np.max), speed_err=g("speed_err_mean", np.mean),
        clearance=g("min_wall_clearance", np.min), side_clearance=g("min_side_clearance", np.min), hit_events=g("wall_hit_events", np.sum),
        runs_with_hits=sum(r["wall_hit_events"] > 0 for r in ok), mpc_failed=g("mpc_failed", np.sum),
        mpc_mean=g("mpc_mean_ms", np.mean), mpc_max=g("mpc_max_ms", np.max),
        slips_logger=g("slips_logger", np.sum), slips_late=g("slips_after_touchdown", np.sum),
        stances=g("stances", np.sum),
    )


def analyze(folder) -> dict:
    folder = Path(folder)
    manifest = json.loads((folder / "manifest.json").read_text())
    course = Course(yaml.safe_load(open(manifest["course_file"]))["course"])
    rows = [analyze_run(folder, r, course) for r in manifest["runs"]]
    windows = {r["tag"]: save_windows(folder, r) for r in rows if r.get("_hits")}
    plot_overview(folder, rows, course)

    with open(folder / "summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_COLS)
        for r in rows:
            w.writerow([fmt(r.get(c, "")) for c in CSV_COLS])

    groups = {p: group_stats([r for r in rows if r["path"] == p]) for p in sorted({r["path"] for r in rows})}
    groups["all"] = group_stats(rows)
    n, n_goal = groups["all"]["runs"], groups["all"]["goal"]
    passed = n > 0 and n_goal >= np.ceil(PASS_FRACTION * n - 1e-9)

    L = [f"# Step 1.5 exit test: {folder.name}", "",
         f"**{'PASS' if passed else 'FAIL'}**: {n_goal} of {n} runs reached the goal "
         f"(need {int(np.ceil(PASS_FRACTION * n - 1e-9))}).", "",
         f"Course `{manifest['course_file']}`, v_ref {manifest['v_ref']} m/s, timeout {manifest['timeout']} s, "
         f"spawn jitter +-{manifest['spawn_jitter']['xy_m']} m / +-{manifest['spawn_jitter']['yaw_deg']:.0f} deg, "
         f"MPC {manifest['mpc']}, commit {manifest['git_commit']} (Quadruped-PyMPC {manifest['quadruped_pympc_commit']}).",
         "", "## Per path type", "",
         "| | goal | mean time to goal [s] | mean \\|e\\| [m] | mean \\|e\\| settled [m] | mean p95 \\|e\\| [m] "
         "| max \\|e\\| [m] | mean speed err [m/s] | min clearance: any wall / side walls [m] | wall-hit events (runs) "
         "| failed MPC solves | MPC mean / max [ms] | slips: logger / after touchdown (stances) |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for p, s in groups.items():
        L.append(f"| {p} | {s['goal']}/{s['runs']} | {fmt(s['time_mean'], 1)} | {fmt(s['cte_mean'])} "
                 f"| {fmt(s['cte_mean_settled'])} | {fmt(s['cte_p95'])} | {fmt(s['cte_max'])} | {fmt(s['speed_err'])} "
                 f"| {fmt(s['clearance'])} / {fmt(s['side_clearance'])} | {fmt(s['hit_events'], 0)} ({s['runs_with_hits']}) | {fmt(s['mpc_failed'], 0)} "
                 f"| {fmt(s['mpc_mean'], 2)} / {fmt(s['mpc_max'], 2)} "
                 f"| {fmt(s['slips_logger'], 0)} / {fmt(s['slips_late'], 0)} ({fmt(s['stances'], 0)}) |")
    L += ["", "## Per run", "",
          "| run | outcome | time [s] | final dist [m] | spawn dx, dy [m], dyaw [deg] | mean \\|e\\| | settled | p95 "
          "| max | min clearance any / side [m] | wall hits | MPC max [ms] | failed solves | slips (after touchdown) |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        if "_data" not in r:
            L.append(f"| {r['tag']} | {r['outcome']} |" + " |" * 12)
            continue
        L.append(f"| {r['tag']} | {r['outcome']} | {fmt(r['time_s'], 1)} | {fmt(r['final_dist_m'])} "
                 f"| {r['spawn_dx']:+.3f}, {r['spawn_dy']:+.3f}, {r['spawn_dyaw_deg']:+.1f} | {fmt(r['cte_mean'])} "
                 f"| {fmt(r['cte_mean_settled'])} | {fmt(r['cte_p95'])} | {fmt(r['cte_max'])} "
                 f"| {fmt(r['min_wall_clearance'])} / {fmt(r['min_side_clearance'])} | {r['wall_hit_events']} | {fmt(r['mpc_max_ms'], 2)} "
                 f"| {r['mpc_failed']} | {r['slips_logger']} ({r['slips_after_touchdown']}) |")
    L += ["", "## Wall hits", ""]
    any_hit = False
    for r in rows:
        for k, h in enumerate(r.get("_hits", [])):
            any_hit = True
            L.append(f"- `{r['tag']}` hit {k}: t = {h['t_start']:.2f}-{h['t_end']:.2f} s, `{h['wall']}`, "
                     f"bodies {', '.join(sorted(h['bodies']))}, max penetration {1e3 * h['max_penetration']:.1f} mm, "
                     f"window `windows/{r['tag']}_hit{k}.npz/.png`")
    if not any_hit:
        L.append("None.")
    terminated = [r for r in rows if r.get("outcome") == "terminated"]
    if terminated:
        L += ["", "## Terminated runs", ""] + [f"- `{r['tag']}`: {r['terminated_by']}" for r in terminated]
    L += ["", "## Notes", "",
          f"- Cross-track error e comes from the path follower (distance to the active segment). 'Settled' drops the "
          f"first {SETTLE_S:.0f} s, which include recovering from the spawn jitter.",
          "- Wall clearance is from the CoM to the nearest inner wall face in plan view; the body reaches about "
          "0.3 m further. 'Any' includes the back wall, 0.8 m behind the spawn on flat_v1; 'side' is the two "
          "long walls only.",
          f"- A wall-hit event merges contacts with the same wall less than {HIT_GAP_S} s apart. Hits do not fail a "
          "run; a body hit ends the run as 'terminated'.",
          "- Failed MPC solves: acados status other than 0 (converged) or 2 (iteration limit, normal with one SQP "
          "iteration per solve). Solve time is wall-clock time of compute_control, compared to the solve interval.",
          f"- Slips: the Track C detector also fires on the short slide right after touchdown; 'after touchdown' "
          f"counts only slips starting more than {1e3 * TOUCHDOWN_S:.0f} ms after touchdown.",
          "", "Files: `summary.csv`, `trajectories.png`, `tracking.png`, per-run `*.png`, `windows/`."]
    (folder / "summary.md").write_text("\n".join(L) + "\n")
    print("\n".join(L[:4 + 2 + 6 + len(groups)]))
    print(f"\nwrote {folder / 'summary.md'}, summary.csv, trajectories.png, tracking.png"
          + (f", windows/ ({sum(len(v) for v in windows.values())} hit windows)" if windows else ""))
    return dict(passed=bool(passed), groups=groups)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder")
    analyze(ap.parse_args().folder)
