"""H2 pilot: orientation-event density vs splat count.

Hypothesis H2 (05_GMC_RESEARCH_DESIGN.md, table "claim ladder"): topology-changing
orientation events are sparse relative to combinatorial churn, so event-driven
tracking can beat uniform pose discretization.

This pilot sweeps theta on a uniform fine grid (reference method, NOT the final
algorithm) and counts, per full sweep, how often three signatures change:

  S1  nerve 1-skeleton (which forbidden primitives pairwise intersect)  -> churn
  S2  obstacle-union topology inside workspace (#components, #holes)
  S3  free-space topology (#components, #holes)                         -> what
      the mobility complex actually has to re-glue on

The H2 bet is S3 << S1 and S3 growing slowly with N. A theta-grid counter is a
lower bound on true event counts (events between grid points collapse); the JSON
records per-signature step-change fractions so saturation is visible, not hidden.

Semantics note: forbidden primitive uses the closed-form overlap superlevel set
with a fixed Mahalanobis support radius RHO (conservative iso-density support
flavor), i.e. F_ij(theta) = { t : (t-c)^T S^-1 (t-c) <= RHO^2 } with
S = Sigma_i + R Lambda_j R^T, c = mu_i - R nu_j. Exact ellipse; polygonized at
NPOLY vertices for shapely booleans (pilot-grade, not certified).

Run:  python h2_event_density.py [--fast]
Outputs under results/gmc_h2/: results.json, figs/*.png
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
from shapely.geometry import Polygon, box as shapely_box
from shapely.ops import unary_union
from shapely.strtree import STRtree

OUT = Path(__file__).resolve().parents[2] / "results" / "gmc_h2"
FIGS = OUT / "figs"

WORKSPACE = (0.0, 0.0, 10.0, 10.0)
RHO = 2.0          # Mahalanobis support radius
NPOLY = 32         # ellipse polygonization
OBST_SCALE = (0.08, 0.45)   # log-uniform sqrt-eigenvalue range of scene splats

# Deliberately asymmetric two-part robot (no hidden pi-symmetry): elongated body,
# aspect ~ 3:1 at RHO support.
ROBOT_PARTS = [
    {"nu": np.array([-0.25, 0.0]), "lam_sqrt": np.array([0.18, 0.09]), "ang": 0.0},
    {"nu": np.array([0.28, 0.02]), "lam_sqrt": np.array([0.12, 0.07]), "ang": 0.0},
]


def rot(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def make_scene(n, seed):
    rng = np.random.default_rng(seed)
    mus = rng.uniform(1.0, 9.0, size=(n, 2))
    lo, hi = np.log(OBST_SCALE[0]), np.log(OBST_SCALE[1])
    scales = np.exp(rng.uniform(lo, hi, size=(n, 2)))
    angs = rng.uniform(0, np.pi, size=n)
    sigmas = []
    for k in range(n):
        R = rot(angs[k])
        sigmas.append(R @ np.diag(scales[k] ** 2) @ R.T)
    return mus, np.array(sigmas)


_CIRCLE = np.stack([np.cos(np.linspace(0, 2 * np.pi, NPOLY, endpoint=False)),
                    np.sin(np.linspace(0, 2 * np.pi, NPOLY, endpoint=False))])


def forbidden_polygons(mus, sigmas, theta):
    """One ellipse per (scene splat, robot part) at fixed theta."""
    R = rot(theta)
    polys = []
    for part in ROBOT_PARTS:
        Rp = R @ rot(part["ang"])
        lam = Rp @ np.diag(part["lam_sqrt"] ** 2) @ Rp.T
        offs = R @ part["nu"]
        for k in range(len(mus)):
            S = sigmas[k] + lam
            c = mus[k] - offs
            L = np.linalg.cholesky(S)
            pts = (c[:, None] + RHO * (L @ _CIRCLE)).T
            polys.append(Polygon(pts))
    return polys


def topo_counts(geom):
    """(#components, #holes) of a polygonal geometry."""
    if geom.is_empty:
        return (0, 0)
    geoms = list(getattr(geom, "geoms", [geom]))
    polys = [g for g in geoms if g.geom_type == "Polygon" and g.area > 1e-9]
    return (len(polys), sum(len(p.interiors) for p in polys))


def slice_signature(polys, ws):
    tree = STRtree(polys)
    edges = set()
    for a, b in zip(*tree.query(polys, predicate="intersects")):
        if a < b:
            edges.add((int(a), int(b)))
    union = unary_union(polys)
    s2 = topo_counts(union.intersection(ws))
    free = ws.difference(union)
    s3 = topo_counts(free)
    return frozenset(edges), s2, s3, free


def sweep(n, seed, n_theta):
    mus, sigmas = make_scene(n, seed)
    ws = shapely_box(*WORKSPACE)
    thetas = np.linspace(0, 2 * np.pi, n_theta, endpoint=False)
    prev = None
    ev = {"S1": 0, "S2": 0, "S3": 0}
    churn_edges = 0
    s3_timeline = []
    free_area = []
    t0 = time.time()
    for th in thetas:
        polys = forbidden_polygons(mus, sigmas, th)
        e, s2, s3, free = slice_signature(polys, ws)
        free_area.append(free.area)
        s3_timeline.append(s3)
        if prev is not None:
            if e != prev[0]:
                ev["S1"] += 1
                churn_edges += len(e.symmetric_difference(prev[0]))
            if s2 != prev[1]:
                ev["S2"] += 1
            if s3 != prev[2]:
                ev["S3"] += 1
        prev = (e, s2, s3)
    dt = time.time() - t0
    return {
        "n_splats": n, "seed": seed, "n_theta": n_theta,
        "n_primitives": 2 * n,
        "events": ev,
        "step_change_fraction": {k: v / (n_theta - 1) for k, v in ev.items()},
        "nerve_edge_churn_total": churn_edges,
        "free_area_mean": float(np.mean(free_area)),
        "free_area_frac": float(np.mean(free_area) / ws.area),
        "s3_range": [int(min(a for a, _ in s3_timeline)),
                     int(max(a for a, _ in s3_timeline))],
        "wall_s": dt,
    }


def fig_slices(n, seed, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    mus, sigmas = make_scene(n, seed)
    ws = shapely_box(*WORKSPACE)
    fig, axes = plt.subplots(1, 4, figsize=(20, 5.2))
    for ax, deg in zip(axes, [0, 45, 90, 135]):
        th = np.deg2rad(deg)
        polys = forbidden_polygons(mus, sigmas, th)
        _, s2, s3, free = slice_signature(polys, ws)
        union = unary_union(polys).intersection(ws)
        for g in getattr(union, "geoms", [union]):
            if g.geom_type == "Polygon":
                ax.fill(*g.exterior.xy, color="0.35", zorder=2)
                for hole in g.interiors:
                    ax.fill(*hole.xy, color="white", zorder=3)
        free_polys = [g for g in getattr(free, "geoms", [free])
                      if g.geom_type == "Polygon" and g.area > 1e-9]
        cmap = plt.get_cmap("tab10")
        for idx, g in enumerate(sorted(free_polys, key=lambda p: -p.area)):
            ax.fill(*g.exterior.xy, color=cmap(idx % 10), alpha=0.35, zorder=1)
        ax.set_title(f"theta={deg}  free comps={s3[0]} holes={s3[1]}")
        ax.set_xlim(WORKSPACE[0], WORKSPACE[2])
        ax.set_ylim(WORKSPACE[1], WORKSPACE[3])
        ax.set_aspect("equal")
    fig.suptitle(f"Fixed-theta pose-space slices, N={n} splats (seed {seed}): "
                 "forbidden union (gray) + free cells (colored)")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def fig_barcode(n, seed, n_theta, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    mus, sigmas = make_scene(n, seed)
    ws = shapely_box(*WORKSPACE)
    thetas = np.linspace(0, 2 * np.pi, n_theta, endpoint=False)
    prev, marks = None, {"S1 nerve edges": [], "S2 obstacle topo": [],
                         "S3 free topo": []}
    for th in thetas:
        polys = forbidden_polygons(mus, sigmas, th)
        e, s2, s3, _ = slice_signature(polys, ws)
        if prev is not None:
            if e != prev[0]:
                marks["S1 nerve edges"].append(th)
            if s2 != prev[1]:
                marks["S2 obstacle topo"].append(th)
            if s3 != prev[2]:
                marks["S3 free topo"].append(th)
        prev = (e, s2, s3)
    fig, ax = plt.subplots(figsize=(14, 3.2))
    ax.eventplot([marks[k] for k in marks], colors=["0.6", "tab:orange", "tab:red"],
                 lineoffsets=[2, 1, 0], linelengths=0.8)
    ax.set_yticks([2, 1, 0], list(marks.keys()))
    ax.set_xlabel("theta (rad)")
    ax.set_xlim(0, 2 * np.pi)
    ax.set_title(f"Where signatures change along theta, N={n} (seed {seed}) — "
                 "H2 bet: bottom row sparse, top row dense")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def fig_scaling(rows, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ns = sorted({r["n_splats"] for r in rows})
    fig, ax = plt.subplots(figsize=(7.5, 5.5))
    for key, color, label in [("S1", "0.5", "S1 nerve-edge set changes (churn)"),
                              ("S2", "tab:orange", "S2 obstacle topology changes"),
                              ("S3", "tab:red", "S3 free-space topology changes")]:
        med, lo, hi = [], [], []
        for n in ns:
            vals = [r["events"][key] for r in rows if r["n_splats"] == n]
            med.append(np.median(vals)); lo.append(min(vals)); hi.append(max(vals))
        ax.plot(ns, med, "o-", color=color, label=label)
        ax.fill_between(ns, lo, hi, color=color, alpha=0.2)
    ref = np.array(ns, dtype=float)
    ax.plot(ns, ref * (med[0] / ref[0] if med[0] else 1), "--", color="0.8",
            label="linear reference")
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlabel("scene splats N"); ax.set_ylabel("events per 2π sweep")
    ax.set_title("H2 pilot: orientation-event density vs scene size")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fast", action="store_true", help="reduced grid smoke test")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    FIGS.mkdir(parents=True, exist_ok=True)
    n_theta = 180 if args.fast else 1440
    ns = [10, 20] if args.fast else [10, 20, 40, 80, 160]
    seeds = [0] if args.fast else [0, 1, 2]
    rows = []
    for n in ns:
        for seed in seeds:
            r = sweep(n, seed, n_theta)
            rows.append(r)
            print(f"N={n:4d} seed={seed} events={r['events']} "
                  f"churn={r['nerve_edge_churn_total']} "
                  f"free_frac={r['free_area_frac']:.2f} {r['wall_s']:.1f}s",
                  flush=True)
    meta = {
        "config": {"workspace": WORKSPACE, "rho": RHO, "npoly": NPOLY,
                   "obst_scale": OBST_SCALE, "n_theta": n_theta,
                   "robot_parts": [{k: (v.tolist() if isinstance(v, np.ndarray)
                                        else v) for k, v in p.items()}
                                   for p in ROBOT_PARTS]},
        "caveats": [
            "theta-grid counting: lower bound on true event count; check "
            "step_change_fraction for saturation (S1 near 1.0 => undercounted)",
            "polygonized ellipses (NPOLY), not certified geometry",
            "single scene family: uniform random splats, no structured walls",
        ],
        "rows": rows,
    }
    (OUT / "results.json").write_text(json.dumps(meta, indent=2))
    fig_slices(40, 0, FIGS / "slices_N40.png")
    fig_barcode(40, 0, n_theta, FIGS / "barcode_N40.png")
    fig_scaling(rows, FIGS / "events_vs_N.png")
    print("done ->", OUT)


if __name__ == "__main__":
    main()
