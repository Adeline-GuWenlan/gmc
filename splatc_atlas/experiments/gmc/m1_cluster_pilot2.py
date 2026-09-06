"""M1 pilot v2: SOUND conservative clustering via Minkowski-sum primitives.

v1 (m1_cluster_pilot.py) merged at scene level and reused the covariance-add
forbidden ellipse — unsound: scene-support containment does not survive robot
convolution (sqrt concavity shrinks the support gap), measured as 0.82 u^2
leakage and 30 false-open steps on door_A/L24.

v2 fix: merged forbidden primitive := (merged scene support ellipse) MINKOWSKI+
(robot support ellipse at theta), computed exactly for convex polygons via the
edge-merge construction, with CIRCUMSCRIBED polygonizations so the polygon
covers the true ellipse. Soundness chain, for every member i and every theta:

  F_i = E(Sigma_i + Lam_th)             (raw forbidden, covariance-add)
      subset E(Sigma_i) + E(Lam_th)     (support fn: sqrt(a+b) <= sqrt a + sqrt b)
      subset E_m + E(Lam_th)            (merged scene support covers member)
      subset P_m (+) R_th               (circumscribed polygons of both)

so false-open = 0 by construction (up to the sampled member-containment
certificate from clustering, margin 1.02). Cost: fatter primitives -> more
extra-closed; measured against the same v3 raw reference.

Outputs: results/gmc_h2/m1_cluster_pilot2.json + figs/m1_cluster_pilot2.png
"""
import json
import time

import numpy as np
import shapely
from shapely.geometry import box as shapely_box, LineString

import h2_k2_windows as base
import m1_cluster_pilot as v1

OUT = base.OUT
NPOLY = 24
CIRCUM = 1.0 / np.cos(np.pi / NPOLY)     # circumscribed-polygon factor
CHECK_THETAS = 12
WINDOW, ROBOT = "door_A", "L24"

_ANG = np.linspace(0, 2 * np.pi, NPOLY, endpoint=False)
_CIRC = np.stack([np.cos(_ANG), np.sin(_ANG)])          # (2, NPOLY)


def ellipse_poly_vertices(mu, S, factor=CIRCUM):
    """(n, NPOLY, 2) CCW circumscribed polygon vertices of rho-support ellipses."""
    a, b, d = S[:, 0, 0], S[:, 0, 1], S[:, 1, 1]
    sa = np.sqrt(a)
    l21, l22 = b / sa, np.sqrt(np.maximum(d - (b / sa) ** 2, 1e-12))
    r = base.RHO * factor
    px = mu[:, 0, None] + r * sa[:, None] * _CIRC[0][None]
    py = mu[:, 1, None] + r * (l21[:, None] * _CIRC[0][None] + l22[:, None] * _CIRC[1][None])
    return np.stack([px, py], axis=-1)


def robot_poly_vertices(theta, rob):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    lam = R @ np.diag(rob ** 2) @ R.T
    return ellipse_poly_vertices(np.zeros((1, 2)), lam[None])[0]     # (NPOLY, 2)


def minkowski_all(P, Rv):
    """Exact Minkowski sum of each convex CCW polygon in P (n,p,2) with convex
    CCW polygon Rv (r,2), via sorted edge merge. Returns (n, p+r, 2)."""
    n, p, _ = P.shape
    r = len(Rv)
    eP = np.roll(P, -1, axis=1) - P
    eR = np.roll(Rv, -1, axis=0) - Rv
    edges = np.concatenate([eP, np.broadcast_to(eR[None], (n, r, 2))], axis=1)
    # angles normalized to [0, 2pi): from the bottom-most start vertex of a CCW
    # convex polygon, edge angles increase in [0, 2pi) — plain atan2 range
    # (-pi, pi] would insert the downward-pointing edges first and produce
    # self-intersecting chains
    ang = np.mod(np.arctan2(edges[..., 1], edges[..., 0]), 2 * np.pi)
    order = np.argsort(ang, axis=1)
    se = np.take_along_axis(edges, order[..., None].repeat(2, -1), axis=1)
    keyP = P[..., 1] * 1e6 + P[..., 0]
    startP = P[np.arange(n), np.argmin(keyP, axis=1)]
    startR = Rv[np.argmin(Rv[:, 1] * 1e6 + Rv[:, 0])]
    start = startP + startR
    csum = np.cumsum(se, axis=1)
    verts = np.concatenate([np.zeros((n, 1, 2)), csum[:, :-1]], axis=1) + start[:, None, :]
    return verts


def merged_forbidden_polys(scene_polys, theta, rob):
    Rv = robot_poly_vertices(theta, rob)
    return shapely.polygons(minkowski_all(scene_polys, Rv))


def main():
    win = base.WINDOWS[WINDOW]
    rob = base.ROBOTS[ROBOT]
    mu, S = base.load_window(win["center"], win["half"])
    ws = shapely_box(win["center"][0] - win["half"], win["center"][1] - win["half"],
                     win["center"][0] + win["half"], win["center"][1] + win["half"])
    cmu, cS = v1.cluster(mu, S)
    scene_polys = ellipse_poly_vertices(cmu, cS)
    print(f"primitives: {len(mu)} raw -> {len(cmu)} merged", flush=True)

    leak, overcov, t_raw, t_merged = [], [], [], []
    churn_raw = churn_merged = None
    for k, th in enumerate(np.linspace(0, np.pi, CHECK_THETAS, endpoint=False)):
        t0 = time.time()
        rpolys = base.forbidden_polys(mu, S, th, rob)
        runion = shapely.union_all(rpolys).intersection(ws)
        t_raw.append(time.time() - t0)
        t0 = time.time()
        mpolys = merged_forbidden_polys(scene_polys, th, rob)
        munion = shapely.union_all(mpolys).intersection(ws)
        t_merged.append(time.time() - t0)
        leak.append(runion.difference(munion).area)
        overcov.append(munion.difference(runion).area)
        if k == 0:
            from shapely.strtree import STRtree
            for polys, slot in ((rpolys, "raw"), (list(mpolys), "merged")):
                tr = STRtree(polys)
                a, b = tr.query(polys, predicate="intersects")
                if slot == "raw":
                    churn_raw = int((a < b).sum())
                else:
                    churn_merged = int((a < b).sum())

    ref = next(r for r in json.loads((OUT / "k2_windows.json").read_text())["runs"]
               if r["window"] == WINDOW and r["robot"] == ROBOT)
    probes = [LineString(p) for p in ref["probes"]]
    thetas = np.linspace(0, np.pi, ref["n_theta"], endpoint=False)
    flags_m = []
    t0 = time.time()
    for th in thetas:
        polys = merged_forbidden_polys(scene_polys, th, rob)
        _, gate_open, _ = base.free_topology(polys, ws, probes)
        flags_m.append(bool(gate_open))
    t_gate = time.time() - t0
    flags_r = np.array(ref["open_flags"], bool)
    flags_m = np.array(flags_m)
    agree = float((flags_r == flags_m).mean())
    false_open = int((~flags_r & flags_m).sum())
    extra_closed = int((flags_r & ~flags_m).sum())

    report = {
        "window": WINDOW, "robot": ROBOT,
        "construction": "merged scene ellipse (+) robot ellipse, exact convex "
                        "Minkowski sum, circumscribed polygons",
        "n_raw": int(len(mu)), "n_merged": int(len(cmu)),
        "conservative": {"max_leak_area": float(max(leak)),
                         "mean_overcoverage_area": float(np.mean(overcov))},
        "cost": {"slice_s_raw": float(np.mean(t_raw)),
                 "slice_s_merged": float(np.mean(t_merged)),
                 "speedup": float(np.mean(t_raw) / np.mean(t_merged)),
                 "churn_raw": churn_raw, "churn_merged": churn_merged,
                 "gate_sweep_720_s": t_gate},
        "gate_fidelity": {"agreement": agree, "false_open": false_open,
                          "extra_closed": extra_closed,
                          "open_frac_ref": float(flags_r.mean()),
                          "open_frac_merged": float(flags_m.mean())},
    }
    (OUT / "m1_cluster_pilot2.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=1), flush=True)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.6))
    th_show = np.deg2rad(61)
    for ax, (polys, ttl) in zip(axes[:1], [(None, None)]):
        pass
    ax = axes[0]
    rpolys = base.forbidden_polys(mu, S, th_show, rob)
    runion = shapely.union_all(rpolys).intersection(ws)
    munion = shapely.union_all(merged_forbidden_polys(scene_polys, th_show, rob)).intersection(ws)
    for g in getattr(munion, "geoms", [munion]):
        if g.geom_type == "Polygon":
            ax.fill(*g.exterior.xy, color="tab:orange", alpha=0.5,
                    label="merged (Minkowski)" if "merged" not in [t.get_label() for t in ax.patches] else None)
    for g in getattr(runion, "geoms", [runion]):
        if g.geom_type == "Polygon":
            ax.fill(*g.exterior.xy, color="0.3", alpha=0.8)
    ax.set_aspect("equal"); ax.set_xlim(ws.bounds[0], ws.bounds[2]); ax.set_ylim(ws.bounds[1], ws.bounds[3])
    ax.set_title(f"theta=61deg: raw union (dark) inside merged cover (orange)\n"
                 f"leak={max(leak):.2e} u^2, overcoverage={np.mean(overcov):.2f} u^2", fontsize=10)
    ax = axes[1]
    deg = np.rad2deg(thetas)
    ax.fill_between(deg, 1.05, 1.95, where=flags_r, color="tab:green", alpha=0.6)
    ax.fill_between(deg, 0.05, 0.95, where=flags_m, color="tab:blue", alpha=0.6)
    for d0 in deg[flags_r != flags_m]:
        ax.axvline(d0, color="tab:red", lw=0.5, alpha=0.5)
    ax.set_yticks([0.5, 1.5], ["merged v2", "raw ref"])
    ax.set_xlabel("theta (deg)"); ax.set_xlim(0, 180)
    ax.set_title(f"gate: agreement {agree:.3f}, false-open {false_open} (must be 0), "
                 f"extra-closed {extra_closed}", fontsize=10)
    fig.suptitle(f"M1 v2 (sound Minkowski primitives) — {WINDOW}/{ROBOT}: "
                 f"{len(mu)}->{len(cmu)}, churn {churn_raw:.0e}->{churn_merged:.0e}")
    fig.tight_layout()
    fig.savefig(base.FIGS / "m1_cluster_pilot2.png", dpi=110)
    print("saved figure", flush=True)


if __name__ == "__main__":
    main()
