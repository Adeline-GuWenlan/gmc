"""Gate G-H2 on the real K2 scene: orientation events + gate intervals at real
doorways, across a robot-length morphology sweep.

v3 (2026-08-24). Changes vs v2: gate probes are radial SEGMENTS on both sides
of the wall (offset 1.0..2.4 u from door center); gate open iff one free
component intersects both segments. v2 point probes were swallowed by fattened
walls at L32, misreporting a visibly open door corridor as closed (see worklog).
Only door x {L24, L32} rerun; other runs merged from k2_windows_v2.json.

v2 (2026-08-24). Changes vs v1 (results kept in k2_windows_v1.json):
  - robot length sweep: support 1.6 / 2.4 / 3.2 u (~0.8/1.2/1.6 m under the
    0.5 m/u hypothesis) x width 0.6 u. v1 showed support 1.6 < door gap 2.0
    never closes -> no gate to measure.
  - churn pair-count sampled ONCE per run (v1 sampled 45x and it dominated
    runtime; single sample suffices to document S1 saturation ~1e7 pairs).
  - tables window: gate metric disabled (no wall -> v1 probes landed inside
    forbidden mass); S3 event counting is the signal there.

Per (window, robot): sweep theta in [0, pi) (1-part robot is pi-symmetric),
track S3 free-space topology events and, for door windows, gate open/closed via
two probes on opposite sides of the wall. Pilot semantics as v1 (uniform RHO
threshold, weight > W_OBST).

Outputs: results/gmc_h2/k2_windows.json + figs/k2_<win>_<robot>.png
"""
import json
import time
from pathlib import Path

import numpy as np
import shapely
from shapely.geometry import box as shapely_box, LineString
from shapely.strtree import STRtree

ATLAS = Path(__file__).resolve().parents[2]
SLAB = ATLAS / "data" / "gs_scenes" / "k2" / "slab_2d.npz"
OUT = ATLAS / "results" / "gmc_h2"
FIGS = OUT / "figs"

RHO = 2.0
NPOLY = 24
W_OBST = 0.3
N_THETA = 720
WALL_YAW = np.deg2rad(60.75)
PROBE_NEAR, PROBE_FAR = 1.0, 2.4

ROBOTS = {  # sqrt-eigenvalues (length, width); support axes = 2*RHO*these
    "L16": np.array([0.4, 0.15]),
    "L24": np.array([0.6, 0.15]),
    "L32": np.array([0.8, 0.15]),
}
WINDOWS = {
    "door_A": {"center": (22.3, -15.2), "half": 3.0, "gate": True},
    "door_B": {"center": (15.2, -11.0), "half": 3.0, "gate": True},
    "tables": {"center": (36.0, -19.0), "half": 3.0, "gate": False},
}
RUNS = [("door_A", "L24"), ("door_A", "L32"),
        ("door_B", "L24"), ("door_B", "L32")]
MERGE_FROM = "k2_windows_v2.json"   # carry over runs not re-run here


def load_window(center, half):
    d = np.load(SLAB)
    mu, cov3, w = d["mean2"].astype(np.float64), d["cov3"].astype(np.float64), d["weight"]
    m = ((np.abs(mu[:, 0] - center[0]) < half + 0.5)
         & (np.abs(mu[:, 1] - center[1]) < half + 0.5) & (w > W_OBST))
    S = np.empty((m.sum(), 2, 2))
    S[:, 0, 0], S[:, 0, 1], S[:, 1, 0], S[:, 1, 1] = (
        cov3[m, 0], cov3[m, 1], cov3[m, 1], cov3[m, 2])
    return mu[m], S


def wall_direction(mu, center):
    rel = mu - np.asarray(center)
    best, bestscore = None, -1
    for a in (WALL_YAW, WALL_YAW - np.pi / 2):
        d = np.array([np.cos(a), np.sin(a)])
        n = np.array([-d[1], d[0]])
        band = np.abs(rel @ n) < 0.35
        score = np.var(rel[band] @ d) * band.sum()
        if score > bestscore:
            best, bestscore = (d, n), score
    return best


_CIRCLE = np.stack([np.cos(np.linspace(0, 2 * np.pi, NPOLY, endpoint=False)),
                    np.sin(np.linspace(0, 2 * np.pi, NPOLY, endpoint=False))])


def forbidden_polys(mu, S, theta, robot_sqrt):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    lam = R @ np.diag(robot_sqrt ** 2) @ R.T
    A = S + lam
    a, b, d = A[:, 0, 0], A[:, 0, 1], A[:, 1, 1]
    sa = np.sqrt(a)
    l21 = b / sa
    l22 = np.sqrt(np.maximum(d - l21 ** 2, 1e-12))
    px = mu[:, 0, None] + RHO * sa[:, None] * _CIRCLE[0][None, :]
    py = mu[:, 1, None] + RHO * (l21[:, None] * _CIRCLE[0][None, :]
                                 + l22[:, None] * _CIRCLE[1][None, :])
    return shapely.polygons(np.stack([px, py], axis=-1))


def free_topology(polys, ws, probes):
    union = shapely.union_all(polys)
    free = ws.difference(union)
    geoms = [g for g in getattr(free, "geoms", [free])
             if g.geom_type == "Polygon" and g.area > 1e-6]
    holes = sum(len(g.interiors) for g in geoms)
    gate_open = None
    if probes:
        gate_open = any(g.intersects(probes[0]) and g.intersects(probes[1])
                        for g in geoms)
    return (len(geoms), holes), gate_open, free


def sweep(win_name, robot_name):
    win, rob = WINDOWS[win_name], ROBOTS[robot_name]
    mu, S = load_window(win["center"], win["half"])
    ws = shapely_box(win["center"][0] - win["half"], win["center"][1] - win["half"],
                     win["center"][0] + win["half"], win["center"][1] + win["half"])
    probes = []
    wall_deg = None
    if win["gate"]:
        d, n = wall_direction(mu, win["center"])
        wall_deg = float(np.rad2deg(np.arctan2(d[1], d[0])))
        c = np.asarray(win["center"])
        probes = [LineString([c + PROBE_NEAR * n, c + PROBE_FAR * n]),
                  LineString([c - PROBE_NEAR * n, c - PROBE_FAR * n])]
    thetas = np.linspace(0, np.pi, N_THETA, endpoint=False)
    s3_prev = gate_prev = None
    s3_events, gate_switches, open_flags = [], [], []
    churn = None
    t0 = time.time()
    for k, th in enumerate(thetas):
        polys = forbidden_polys(mu, S, th, rob)
        s3, gate_open, _ = free_topology(polys, ws, probes)
        if win["gate"]:
            open_flags.append(bool(gate_open))
            if gate_prev is not None and gate_open != gate_prev:
                gate_switches.append(float(th))
            gate_prev = gate_open
        if s3_prev is not None and s3 != s3_prev:
            s3_events.append(float(th))
        s3_prev = s3
        if k == 0:
            tree = STRtree(polys)
            a, b = tree.query(polys, predicate="intersects")
            churn = int((a < b).sum())
    return {
        "window": win_name, "robot": robot_name,
        "robot_support": [float(2 * RHO * v) for v in rob],
        "center": win["center"], "half": win["half"], "n_splats": int(len(mu)),
        "wall_dir_deg": wall_deg,
        "probes": [list(p.coords) for p in probes],
        "n_theta": N_THETA, "theta_range": "[0, pi)",
        "s3_events": s3_events, "n_s3_events": len(s3_events),
        "gate_switch_angles_deg": [float(np.rad2deg(a)) for a in gate_switches],
        "gate_open_fraction": (float(np.mean(open_flags)) if open_flags else None),
        "open_flags": open_flags,
        "churn_pairs_at_theta0": churn,
        "wall_s": time.time() - t0,
    }


def fig_run(res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    win, rob = WINDOWS[res["window"]], ROBOTS[res["robot"]]
    probe_lines = [LineString(p) for p in res["probes"]]
    mu, S = load_window(win["center"], win["half"])
    ws = shapely_box(win["center"][0] - win["half"], win["center"][1] - win["half"],
                     win["center"][0] + win["half"], win["center"][1] + win["half"])
    thetas = np.linspace(0, np.pi, res["n_theta"], endpoint=False)
    if res["open_flags"]:
        flags = np.array(res["open_flags"])
        show = []
        for want in (True, False):
            idx = np.where(flags == want)[0]
            if len(idx):
                show += [thetas[idx[len(idx) // 3]], thetas[idx[2 * len(idx) // 3]]]
    else:
        show = [thetas[i * res["n_theta"] // 4] for i in range(4)]
    fig, axes = plt.subplots(1, len(show) + 1, figsize=(5.2 * (len(show) + 1), 5.6))
    for ax, th in zip(axes, show):
        polys = forbidden_polys(mu, S, th, rob)
        s3, gate_open, free = free_topology(polys, ws, probe_lines)
        union = shapely.union_all(polys).intersection(ws)
        for g in getattr(union, "geoms", [union]):
            if g.geom_type == "Polygon":
                ax.fill(*g.exterior.xy, color="0.35")
        cmap = plt.get_cmap("tab10")
        geoms = [g for g in getattr(free, "geoms", [free])
                 if g.geom_type == "Polygon" and g.area > 1e-6]
        for i, g in enumerate(sorted(geoms, key=lambda p: -p.area)):
            ax.fill(*g.exterior.xy, color=cmap(i % 10), alpha=0.4)
        for p in res["probes"]:
            ax.plot([p[0][0], p[1][0]], [p[0][1], p[1][1]], "k-", lw=2.5)
        lbl = {True: "OPEN", False: "CLOSED", None: ""}[gate_open]
        ax.set_title(f"theta={np.rad2deg(th):.0f}  {lbl}  comps={s3[0]}")
        ax.set_aspect("equal")
        ax.set_xlim(ws.bounds[0], ws.bounds[2]); ax.set_ylim(ws.bounds[1], ws.bounds[3])
    ax = axes[-1]
    if res["open_flags"]:
        ax.fill_between(np.rad2deg(thetas), 0, 1, where=np.array(res["open_flags"]),
                        color="tab:green", alpha=0.5, label="gate open")
        ax.legend(loc="upper right", fontsize=8)
    for a in res["s3_events"]:
        ax.axvline(np.rad2deg(a), color="tab:red", lw=0.6, alpha=0.6)
    ax.set_xlim(0, 180); ax.set_yticks([])
    ax.set_xlabel("theta (deg)")
    of = res["gate_open_fraction"]
    ax.set_title(f"open frac={'n/a' if of is None else f'{of:.2f}'}, "
                 f"S3 events={res['n_s3_events']} (red)")
    sup = res["robot_support"]
    fig.suptitle(f"K2 {res['window']} / robot {res['robot']} "
                 f"(support {sup[0]:.1f} x {sup[1]:.1f} u): pose-space slices + gate profile")
    fig.tight_layout()
    fig.savefig(FIGS / f"k2_{res['window']}_{res['robot']}.png", dpi=110)
    plt.close(fig)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    FIGS.mkdir(parents=True, exist_ok=True)
    prev = OUT / "k2_windows.json"
    if prev.exists() and not (OUT / "k2_windows_v2.json").exists():
        prev.rename(OUT / "k2_windows_v2.json")
    carried = []
    mf = OUT / MERGE_FROM
    if mf.exists():
        old = json.loads(mf.read_text())["runs"]
        carried = [r for r in old if (r["window"], r["robot"]) not in RUNS]
    # resume support: keep completed runs from a partial k2_windows.json
    done = {}
    partial = OUT / "k2_windows.json"
    if partial.exists():
        for r in json.loads(partial.read_text()).get("runs", []):
            done[(r["window"], r["robot"])] = r
    results = []
    for win_name, robot_name in RUNS:
        if (win_name, robot_name) in done:
            results.append(done[(win_name, robot_name)])
            print(f"{win_name}/{robot_name}: cached, skip", flush=True)
            continue
        res = sweep(win_name, robot_name)
        results.append(res)
        of = res["gate_open_fraction"]
        print(f"{win_name}/{robot_name}: splats={res['n_splats']} "
              f"S3={res['n_s3_events']} open_frac={'n/a' if of is None else f'{of:.2f}'} "
              f"switches@deg={[round(a, 1) for a in res['gate_switch_angles_deg']]} "
              f"churn0={res['churn_pairs_at_theta0']} {res['wall_s']:.0f}s", flush=True)
        fig_run(res)
        (OUT / "k2_windows.json").write_text(json.dumps(
            {"partial": True, "runs": carried + results}, indent=2))
    meta = {
        "config": {"rho": RHO, "npoly": NPOLY, "w_obst": W_OBST, "n_theta": N_THETA,
                   "robots": {k: v.tolist() for k, v in ROBOTS.items()},
                   "probe_near_far": [PROBE_NEAR, PROBE_FAR]},
        "caveats": [
            "theta-grid event counting (lower bound); [0,pi) via 1-part symmetric robot",
            "pilot collision semantics: uniform RHO threshold on weight>W_OBST splats",
            "wall-gap audit not yet done: openings may include glass/scan holes",
        ],
        "runs": carried + results,
    }
    (OUT / "k2_windows.json").write_text(json.dumps(meta, indent=2))
    print("done ->", OUT / "k2_windows.json")


if __name__ == "__main__":
    main()
