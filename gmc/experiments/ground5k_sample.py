"""Ground 5000-pair benchmark, stage G1 Task 1: the seeded pair sampler.

Pairs are drawn uniformly over the whole hall (the scene extent), then filtered:

* (a) known floor: for both robots the body's xy box lies on observed floor cells
  (``floor evidence`` raster from the source capture) and inside the gs3d contact strip;
* (b) both endpoint poses certified free for both robots by the shared gs3d oracle
  (``GaussianBodyOracle.pose``, margin 0.001 m);
* separation: Euclidean xy distance >= 3 m;
* (c) certified connected for both robots: each endpoint attaches by a certified edge to a
  0.25 m lattice whose every node and edge the same oracle certified free, and both
  endpoints attach to one lattice component.  The lattice path is the pair's witness.

Every rejection is counted per filter.  The accepted pairs are independent draws, then
shuffled once, so any prefix is an unbiased sample.  Steps (sbatch, see ``hpc/ground5k``):
``evidence`` -> ``lattice`` -> ``sample`` -> ``plot``.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.contracts import SceneSpec
from gmc.gs3d.robots import crop_by_support_aabb

import ground5k_common as gc

OUT = Path("outputs/ground5k")
RES = Path("results/ground5k")
NEIGHBOURS = ((1, 0), (0, 1), (1, 1), (1, -1))


# ------------------------------------------------------------------- floor evidence --

def floor_evidence(means, opacity, floor, extent, ev, covs=None) -> tuple[np.ndarray, dict]:
    """Observed-floor raster from opaque splats whose centre lies near the fitted floor plane.

    With ``covs`` and ``ev["footprint_level"]`` a splat marks every cell its xy support box
    (``mean +- level * sigma``) touches, provided that box's half-extent is at most
    ``ev["max_half_extent_m"]`` (larger floor-layer splats are ignored, not stretched over
    unobserved floor).  Without ``covs`` a splat marks only its centre cell.
    """
    strip = gc.ContactStrip.from_floor(floor, 1.0)
    fitted = floor["z_floor"] + strip.dev(means[:, 0], means[:, 1])
    rel = means[:, 2] - fitted
    sel = (opacity > ev["tau"]) & (rel >= ev["z_lo_rel_fitted_m"]) & (rel <= ev["z_hi_rel_fitted_m"])
    cell = float(ev["cell_m"])
    nx = int(math.ceil((extent[2] - extent[0]) / cell))
    ny = int(math.ceil((extent[3] - extent[1]) / cell))
    m = means[sel]
    if covs is not None and ev.get("footprint_level"):
        c = covs[sel]
        half = float(ev["footprint_level"]) * np.sqrt(np.stack([c[:, 0, 0], c[:, 1, 1]], 1))
        big = half.max(1) > float(ev["max_half_extent_m"])
        m, half = m[~big], half[~big]
        too_large = int(big.sum())
    else:
        half = np.zeros((len(m), 2))
        too_large = 0
    i0 = np.floor((m[:, 0] - half[:, 0] - extent[0]) / cell).astype(int)
    i1 = np.floor((m[:, 0] + half[:, 0] - extent[0]) / cell).astype(int)
    j0 = np.floor((m[:, 1] - half[:, 1] - extent[1]) / cell).astype(int)
    j1 = np.floor((m[:, 1] + half[:, 1] - extent[1]) / cell).astype(int)
    counts = np.zeros((nx, ny), np.int64)
    span = int(max((i1 - i0).max(initial=0), (j1 - j0).max(initial=0))) + 1
    for di in range(span):
        for dj in range(span):
            i, j = i0 + di, j0 + dj
            ok = (i <= i1) & (j <= j1) & (i >= 0) & (j >= 0) & (i < nx) & (j < ny)
            np.add.at(counts, (i[ok], j[ok]), 1)
    mask = counts >= int(ev["min_count"])
    stats = {"n_input": int(len(means)), "n_floor_layer": int(sel.sum()),
             "too_large_ignored": too_large, "mode": "footprint" if covs is not None and ev.get("footprint_level") else "centre",
             "cells": [nx, ny], "cell_m": cell, "observed_cells": int(mask.sum()),
             "observed_area_m2": float(mask.sum() * cell * cell),
             "extent_area_m2": float(nx * ny * cell * cell)}
    return mask, stats


def hall_region(mask, extent, cfg, floor) -> gc.RasterRegion:
    strip = gc.ContactStrip.from_floor(floor, cfg["contact"]["max_height_error_m"])
    return gc.RasterRegion(mask, extent[:2], cfg["sampler"]["evidence"]["cell_m"],
                           gc.z_range(cfg, floor), strip)


def hall_scene(scene3d, region: gc.RasterRegion, floor, cfg, scene_id) -> SceneSpec:
    """The whole hall as one gs3d SceneSpec: known space and support = observed floor."""
    lo, hi = region.lower, region.upper
    cropped, crop = crop_by_support_aabb(scene3d, lo, hi, level=cfg["level"], tau=cfg["tau"])
    box = gc.Region.from_box((lo[0], lo[1], hi[0], hi[1]), region.z, region.strip)
    err = gc.region_height_error(box, floor, cfg)
    support = gc.make_support(region, floor, cfg, err)
    return SceneSpec(scene_id, cropped, lo, hi, cfg["tau"], cfg["level"], region, support,
                     {"coverage_policy": "observed_floor_raster_and_contact_strip", "crop": crop,
                      "tau": cfg["tau"], "level": cfg["level"], "seed": 0})


# ------------------------------------------------------------------ witness lattice --

def _free(report) -> bool:
    return report.occupancy == "free" and report.safety == "continuous_bound"


def lattice_axes(extent, spacing):
    xs = np.arange(extent[0] + spacing / 2, extent[2], spacing)
    ys = np.arange(extent[1] + spacing / 2, extent[3], spacing)
    return xs, ys


def build_witness(oracle, body, floor, region, extent, spacing, margin, log=None) -> dict:
    """Certified lattice: node = certified-free pose, edge = certified-free swept motion."""
    xs, ys = lattice_axes(extent, spacing)
    nx, ny = len(xs), len(ys)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    cand = region.contains_points(np.column_stack([X.ravel(), Y.ravel()])).reshape(nx, ny)
    free = np.zeros((nx, ny), bool)
    reasons: dict[str, int] = {}
    t0 = time.time()
    for k, (i, j) in enumerate(zip(*np.nonzero(cand))):
        rep = oracle.pose(gc.ground_pose(xs[i], ys[j], body, floor), body, margin_m=margin)
        free[i, j] = _free(rep)
        if not free[i, j]:
            reasons[rep.reason] = reasons.get(rep.reason, 0) + 1
        if log and k % 2000 == 0:
            log(f"{body.name} nodes {k}/{int(cand.sum())} {time.time() - t0:.0f}s")
    edges = []
    tested = 0
    fi, fj = np.nonzero(free)
    for k, (i, j) in enumerate(zip(fi, fj)):
        a = gc.ground_pose(xs[i], ys[j], body, floor)
        for di, dj in NEIGHBOURS:
            p, q = i + di, j + dj
            if 0 <= p < nx and 0 <= q < ny and free[p, q]:
                tested += 1
                b = gc.ground_pose(xs[p], ys[q], body, floor)
                if _free(oracle.edge(a, b, body, margin_m=margin)):
                    edges.append((i * ny + j, p * ny + q))
        if log and k % 2000 == 0:
            log(f"{body.name} edges from node {k}/{len(fi)} {time.time() - t0:.0f}s")
    return {"xs": xs, "ys": ys, "free": free, "edges": np.asarray(edges, np.int64).reshape(-1, 2),
            "stats": {"candidate_nodes": int(cand.sum()), "free_nodes": int(free.sum()),
                      "edges_tested": tested, "edges_free": len(edges),
                      "node_rejections": reasons, "wall_s": time.time() - t0}}


def witness_graph(w: dict):
    """CSR graph + component labels (isolated invalid nodes get their own labels)."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    nx, ny = len(w["xs"]), len(w["ys"])
    n = nx * ny
    e = w["edges"]
    ii, jj = e[:, 0] // ny, e[:, 0] % ny
    pp, qq = e[:, 1] // ny, e[:, 1] % ny
    length = np.hypot(w["xs"][pp] - w["xs"][ii], w["ys"][qq] - w["ys"][jj])
    g = coo_matrix((length, (e[:, 0], e[:, 1])), shape=(n, n)).tocsr()
    g = g.maximum(g.T)
    _, labels = connected_components(g, directed=False)
    return g, labels


def attach(oracle, body, floor, xy, w, margin, radius_cells=1) -> list[int]:
    """Lattice nodes an endpoint reaches by one certified edge, nearest first."""
    xs, ys = w["xs"], w["ys"]
    s = xs[1] - xs[0] if len(xs) > 1 else 1.0
    ci = int(np.floor((xy[0] - xs[0]) / s))
    cj = int(np.floor((xy[1] - ys[0]) / s))
    cand = []
    for i in range(ci - radius_cells + 1, ci + radius_cells + 1):
        for j in range(cj - radius_cells + 1, cj + radius_cells + 1):
            if 0 <= i < len(xs) and 0 <= j < len(ys) and w["free"][i, j]:
                cand.append((math.hypot(xs[i] - xy[0], ys[j] - xy[1]), i, j))
    out = []
    a = gc.ground_pose(xy[0], xy[1], body, floor)
    for _, i, j in sorted(cand):
        b = gc.ground_pose(xs[i], ys[j], body, floor)
        if _free(oracle.edge(a, b, body, margin_m=margin)):
            out.append(i * len(ys) + j)
    return out


def witness_path(graph, w, s_nodes, g_nodes, labels, s_xy, g_xy):
    """Shortest lattice path between one start and one goal attachment in a shared component."""
    from scipy.sparse.csgraph import dijkstra
    common = sorted(set(labels[s_nodes]) & set(labels[g_nodes]))
    if not common:
        return None
    comp = common[0]
    s0 = next(n for n in s_nodes if labels[n] == comp)
    g0 = next(n for n in g_nodes if labels[n] == comp)
    _, pred = dijkstra(graph, indices=s0, return_predecessors=True)
    nodes = [g0]
    while nodes[-1] != s0:
        nodes.append(int(pred[nodes[-1]]))
    nodes.reverse()
    ny = len(w["ys"])
    pts = [tuple(s_xy)] + [(float(w["xs"][n // ny]), float(w["ys"][n % ny])) for n in nodes] + [tuple(g_xy)]
    pts = np.asarray(pts, float)
    return {"component": int(comp), "n_nodes": len(nodes),
            "length_m": float(np.sum(np.hypot(*np.diff(pts, axis=0).T))), "polyline": pts}


# ------------------------------------------------------------------------ sampling --

def draw_endpoint(rng, extent, region, oracles, floor, margin, counts):
    """Uniform over the extent until (a) and (b) hold for both robots."""
    while True:
        counts["endpoint_draws"] += 1
        xy = rng.uniform(extent[:2], extent[2:])
        known = True
        for name, body in gc.BODIES.items():
            q = gc.ground_pose(xy[0], xy[1], body, floor)
            half = np.array([body.radius_m, body.radius_m, body.half_height_m])
            if not region.contains_aabb(tuple(np.asarray(q.xyz) - half), tuple(np.asarray(q.xyz) + half)):
                known = False
                break
        if not known:
            counts["a_unknown_floor"] += 1
            continue
        clear = {}
        for name, body in gc.BODIES.items():
            rep = oracles[name].pose(gc.ground_pose(xy[0], xy[1], body, floor), body, margin_m=margin)
            if not _free(rep):
                counts[f"b_not_free_{name}"] += 1
                clear = None
                break
            clear[name] = float(rep.clearance_lower_m)
        if clear is None:
            continue
        return xy, clear


def sample_pairs(n, rng, extent, region, oracles, witnesses, floor, cfg, log=None) -> tuple[list, dict]:
    sp = cfg["sampler"]
    margin = cfg["margin_m"]
    counts = {"endpoint_draws": 0, "a_unknown_floor": 0, "b_not_free_sweeper": 0,
              "b_not_free_cylinder": 0, "pair_draws": 0, "sep_below_min": 0,
              "c_unattached_sweeper": 0, "c_disconnected_sweeper": 0,
              "c_unattached_cylinder": 0, "c_disconnected_cylinder": 0, "accepted": 0}
    graphs = {k: witness_graph(w) for k, w in witnesses.items()}
    pairs = []
    t0 = time.time()
    while len(pairs) < n:
        s, cs = draw_endpoint(rng, extent, region, oracles, floor, margin, counts)
        g, cg = draw_endpoint(rng, extent, region, oracles, floor, margin, counts)
        counts["pair_draws"] += 1
        sep = float(np.hypot(*(g - s)))
        if sep < sp["min_sep_m"]:
            counts["sep_below_min"] += 1
            continue
        wit = {}
        for name, body in gc.BODIES.items():
            w = witnesses[name]
            graph, labels = graphs[name]
            sn = attach(oracles[name], body, floor, s, w, margin)
            gn = attach(oracles[name], body, floor, g, w, margin) if sn else []
            if not sn or not gn:
                counts[f"c_unattached_{name}"] += 1
                wit = None
                break
            path = witness_path(graph, w, sn, gn, labels, s, g)
            if path is None:
                counts[f"c_disconnected_{name}"] += 1
                wit = None
                break
            wit[name] = path
        if wit is None:
            continue
        counts["accepted"] += 1
        pairs.append({"start": s, "goal": g, "sep_m": sep,
                      "clearance": {k: {"start": cs[k], "goal": cg[k]} for k in gc.BODIES},
                      "witness": wit})
        if log and len(pairs) % 250 == 0:
            log(f"pairs {len(pairs)}/{n} draws {counts['pair_draws']} {time.time() - t0:.0f}s")
    return pairs, counts


def finalize_pairs(pairs, rng, floor, cfg, prefix) -> tuple[list, dict]:
    """Shuffle once, assign ids, attach floor z, known check, witness summary and crop box."""
    order = rng.permutation(len(pairs))
    strip = gc.ContactStrip.from_floor(floor, cfg["contact"]["max_height_error_m"])
    extent = floor["extent"]
    rows, polylines = [], {}
    for k, idx in enumerate(order):
        p = pairs[idx]
        pid = f"{prefix}-{k:05d}"
        pts = [p["start"], p["goal"]] + [w["polyline"] for w in p["witness"].values()]
        box = gc.query_box(np.vstack([np.atleast_2d(x) for x in pts]), cfg["crop"]["pad_m"], extent)
        reg = gc.query_region(box, cfg, floor)
        verts = reg.vertices()
        rows.append({
            "pair_id": pid, "index": k,
            "start": [round(float(v), 6) for v in p["start"]],
            "goal": [round(float(v), 6) for v in p["goal"]],
            "sep_m": round(p["sep_m"], 6),
            "floor_z": {e: {"support_z": floor["z_floor"],
                            "fitted_z": floor["z_floor"] + float(strip.dev(*p[e])),
                            "fitted_minus_support_m": float(strip.dev(*p[e]))}
                        for e in ("start", "goal")},
            "known_region_check": {"start": True, "goal": True,
                                   "rule": "both robots' body xy boxes on observed-floor cells and in the contact strip"},
            "clearance_lb_m": p["clearance"],
            "witness": {k2: {"component": w["component"], "n_nodes": w["n_nodes"],
                             "length_m": round(w["length_m"], 6),
                             "witness_id": f"{k2}:lattice0.25:comp{w['component']}"}
                        for k2, w in p["witness"].items()},
            "crop": {"box_xy": [round(v, 6) for v in box],
                     "region_vertices_xy": np.round(verts, 6).tolist(),
                     "clipped_by_contact_strip": bool(len(verts) != 4 or not np.allclose(
                         sorted(map(tuple, np.round(verts, 9))),
                         sorted([(box[0], box[1]), (box[0], box[3]), (box[2], box[1]), (box[2], box[3])]))),
                     "area_m2": float(reg.polygon().area)},
        })
        polylines[pid] = {k2: w["polyline"] for k2, w in p["witness"].items()}
    return rows, polylines


# ------------------------------------------------------------------------- steps --

def _log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def step_evidence(args, cfg):
    spec = cfg["scenes"][args.scene]
    ev = cfg["sampler"]["evidence"]
    src = spec["floor_evidence_source"]
    if gc.sha256(src["path"]) != src["sha256"]:
        raise ValueError("floor-evidence source hash differs from config")
    _, floor = gc.load_scene(cfg, args.scene)
    with np.load(src["path"], allow_pickle=False) as d:
        means, opacity = d["means"], d["opacity"]
        covs = d["covs"] if ev.get("footprint_level") else None
    mask, stats = floor_evidence(means, opacity, floor, floor["extent"], ev, covs=covs)
    del covs
    strip = gc.ContactStrip.from_floor(floor, cfg["contact"]["max_height_error_m"])
    # histogram of opaque splat heights relative to the fitted plane, for the record
    rel = means[:, 2] - (floor["z_floor"] + gc.ContactStrip.from_floor(floor, 1.).dev(means[:, 0], means[:, 1]))
    h, e = np.histogram(rel[(opacity > ev["tau"]) & (np.abs(rel) < 0.5)], bins=100, range=(-0.5, 0.5))
    out = OUT / args.scene
    out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out / "floor_evidence.npz", mask=mask, extent=np.asarray(floor["extent"]),
                        cell=ev["cell_m"])
    cx = floor["extent"][0] + (np.arange(mask.shape[0]) + .5) * ev["cell_m"]
    cy = floor["extent"][1] + (np.arange(mask.shape[1]) + .5) * ev["cell_m"]
    CX, CY = np.meshgrid(cx, cy, indexing="ij")
    in_strip = np.abs(strip.dev(CX, CY)) <= strip.max_err
    stats.update(contact_strip_cells=int(in_strip.sum()), observed_and_strip_cells=int((mask & in_strip).sum()),
                 observed_and_strip_area_m2=float((mask & in_strip).sum() * ev["cell_m"] ** 2),
                 rel_height_hist={"edges": e.tolist(), "counts": h.tolist()}, params=ev,
                 floor=floor, source=src)
    gc.write_json_atomic(out / "floor_evidence.json", stats)
    _plot_evidence(mask, in_strip, floor, ev["cell_m"], h, e, RES / "figs" / f"floor_evidence_{args.scene}.png")
    _log(json.dumps({k: v for k, v in stats.items() if k not in ("rel_height_hist", "floor", "params", "source")}))


def _plot_evidence(mask, in_strip, floor, cell, h, e, path):
    import matplotlib.pyplot as plt
    ext = floor["extent"]
    fig, ax = plt.subplots(1, 2, figsize=(14, 9), gridspec_kw={"width_ratios": [1.3, 1]})
    img = np.zeros(mask.shape + (3,))
    img[mask & in_strip] = (.55, .75, .55)
    img[mask & ~in_strip] = (.95, .70, .35)
    img[~mask] = (1, 1, 1)
    ax[0].imshow(np.transpose(img, (1, 0, 2)), origin="lower", extent=[ext[0], ext[2], ext[1], ext[3]])
    ax[0].set_title("observed floor (green: inside contact strip, orange: outside)\n"
                    f"cell {cell} m; white = no floor-layer splat")
    ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("y (m)")
    ax[1].bar(.5 * (e[1:] + e[:-1]), h, width=e[1] - e[0])
    ax[1].set_yscale("log"); ax[1].set_xlabel("opaque splat centre z - fitted floor (m)")
    ax[1].set_title("source capture, |dz| < 0.5 m")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(); fig.savefig(path, dpi=90); plt.close(fig)


def _load_hall(args, cfg):
    scene3d, floor = gc.load_scene(cfg, args.scene)
    d = np.load(OUT / args.scene / "floor_evidence.npz")
    region = hall_region(d["mask"], floor["extent"], cfg, floor)
    spec = hall_scene(scene3d, region, floor, cfg, f"ground5k-{args.scene}-hall")
    del scene3d
    return spec, region, floor


def step_lattice(args, cfg):
    spec, region, floor = _load_hall(args, cfg)
    _log(f"hall crop {spec.provenance['crop']}")
    t0 = time.time()
    prepared = PreparedScene(spec)
    _log(f"prepared {prepared.stats} in {time.time() - t0:.0f}s")
    out = OUT / args.scene
    sp = cfg["sampler"]
    pids = {}
    for name in args.robots.split(","):
        pid = os.fork()
        if pid == 0:
            try:
                oracle = GaussianBodyOracle(prepared)
                w = build_witness(oracle, gc.BODIES[name], floor, region, floor["extent"],
                                  sp["lattice_m"], cfg["margin_m"], log=_log)
                np.savez_compressed(out / f"witness_{name}.npz", xs=w["xs"], ys=w["ys"], free=w["free"],
                                    edges=w["edges"])
                gc.write_json_atomic(out / f"witness_{name}.json", {**w["stats"], "oracle": oracle.stats})
                _log(f"{name} done {w['stats']}")
                os._exit(0)
            except BaseException as exc:  # noqa: BLE001
                _log(f"{name} FAILED {exc!r}")
                os._exit(1)
        pids[pid] = name
    bad = []
    for _ in pids:
        pid, status = os.wait()
        if status != 0:
            bad.append(pids[pid])
    if bad:
        raise SystemExit(f"lattice failed for {bad}")


def step_sample(args, cfg):
    spec, region, floor = _load_hall(args, cfg)
    prepared = PreparedScene(spec)
    oracles = {k: GaussianBodyOracle(prepared) for k in gc.BODIES}
    witnesses = {}
    for k in gc.BODIES:
        d = np.load(OUT / args.scene / f"witness_{k}.npz")
        witnesses[k] = {key: d[key] for key in ("xs", "ys", "free", "edges")}
    seed = int(cfg["sampler"]["seed"]) + cfg["scenes"][args.scene]["seed_offset"]
    rng = np.random.default_rng(seed)
    t0 = time.time()
    pairs, counts = sample_pairs(args.n or cfg["sampler"]["n_pairs"], rng, floor["extent"], region,
                                 oracles, witnesses, floor, cfg, log=_log)
    rows, polylines = finalize_pairs(pairs, rng, floor, cfg, cfg["scenes"][args.scene]["pair_prefix"])
    wall = time.time() - t0
    doc = {"schema_version": "ground5k.pairs.v1", "scene": args.scene,
           "scene_sha256": cfg["scenes"][args.scene]["sha256"],
           "config_sha256": gc.sha256(gc.CONFIG), "seed": seed, "n_pairs": len(rows),
           "shuffled": True, "filters": counts,
           "filter_rates": _rates(counts),
           "sampler_wall_s": wall, "oracle_stats": {k: o.stats for k, o in oracles.items()},
           "bias": ("admitted pairs are those a 0.25 m certified lattice connects for BOTH robots, on "
                    "observed floor inside the contact strip; pairs reachable only through gaps the "
                    "coarse lattice misses, or through unobserved floor, are excluded"),
           "pairs": rows}
    out = OUT / args.scene
    gc.write_json_atomic(out / f"pairs_{args.scene}.json", doc)
    np.savez_compressed(out / f"witness_paths_{args.scene}.npz",
                        **{f"{pid}__{k}": v for pid, d in polylines.items() for k, v in d.items()})
    _log(f"wrote {len(rows)} pairs; filters {counts}; {wall:.0f}s")


def _rates(c):
    e = max(c["endpoint_draws"], 1)
    a_pass = c["endpoint_draws"] - c["a_unknown_floor"]
    p = max(c["pair_draws"], 1)
    sep_pass = c["pair_draws"] - c["sep_below_min"]
    return {"a_unknown_floor_of_endpoint_draws": c["a_unknown_floor"] / e,
            "b_not_free_of_known_endpoints": (c["b_not_free_sweeper"] + c["b_not_free_cylinder"]) / max(a_pass, 1),
            "sep_below_min_of_pair_draws": c["sep_below_min"] / p,
            "c_rejected_of_separated_pairs": (c["c_unattached_sweeper"] + c["c_disconnected_sweeper"]
                                              + c["c_unattached_cylinder"] + c["c_disconnected_cylinder"])
            / max(sep_pass, 1)}


def step_plot(args, cfg):
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    out = OUT / args.scene
    doc = json.loads((out / f"pairs_{args.scene}.json").read_text())
    ev = np.load(out / "floor_evidence.npz")
    mask, ext, cell = ev["mask"], ev["extent"], float(ev["cell"])
    wit = {k: np.load(out / f"witness_{k}.npz") for k in gc.BODIES}
    pairs = doc["pairs"]
    S = np.array([p["start"] for p in pairs]); G = np.array([p["goal"] for p in pairs])
    fig, axes = plt.subplots(1, 3, figsize=(22, 11), gridspec_kw={"width_ratios": [1, 1, .8]})
    for ax, title in zip(axes[:2], ("all start (blue) / goal (red) endpoints",
                                    "first 300 pairs: start-goal segments; cylinder lattice (grey)")):
        img = np.where(mask.T, .93, 1.)
        ax.imshow(img, origin="lower", cmap="gray", vmin=0, vmax=1, extent=[ext[0], ext[2], ext[1], ext[3]])
        ax.set_xlim(ext[0], ext[2]); ax.set_ylim(ext[1], ext[3]); ax.set_aspect("equal")
        ax.set_title(title); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    w = wit["cylinder"]
    fr = w["free"]
    X, Y = np.meshgrid(w["xs"], w["ys"], indexing="ij")
    axes[1].scatter(X[fr], Y[fr], s=.3, c="0.6", lw=0)
    axes[0].scatter(S[:, 0], S[:, 1], s=1.5, c="tab:blue", lw=0, alpha=.6)
    axes[0].scatter(G[:, 0], G[:, 1], s=1.5, c="tab:red", lw=0, alpha=.6)
    k = min(300, len(pairs))
    axes[1].add_collection(LineCollection(np.stack([S[:k], G[:k]], 1), colors="tab:purple", lw=.4, alpha=.5))
    ax = axes[2]
    ax.hist([p["sep_m"] for p in pairs], bins=40, color="tab:purple")
    ax.set_xlabel("start-goal xy separation (m)"); ax.set_ylabel("pairs")
    f = doc["filters"]
    ax.set_title(f"{len(pairs)} pairs, seed {doc['seed']}\nendpoint draws {f['endpoint_draws']}, "
                 f"pair draws {f['pair_draws']}", fontsize=10)
    txt = "\n".join(f"{k}: {v}" for k, v in f.items())
    ax.text(.02, .98, txt, transform=ax.transAxes, va="top", fontsize=8, family="monospace",
            bbox=dict(fc="w", ec="0.7"))
    fig.suptitle(f"ground5k pairs on {args.scene}: uniform over the hall floor, >= "
                 f"{cfg['sampler']['min_sep_m']} m, filters (a) observed floor + contact strip, "
                 "(b) endpoints certified free, (c) certified-lattice connected, both robots")
    path = RES / "figs" / f"pairs_{args.scene}.png"
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(); fig.savefig(path, dpi=80); plt.close(fig)
    # spread over the hall: share of endpoints per 5 m x 5 m block
    H, xe, ye = np.histogram2d(np.r_[S[:, 0], G[:, 0]], np.r_[S[:, 1], G[:, 1]],
                               bins=[np.arange(ext[0], ext[2] + 5, 5), np.arange(ext[1], ext[3] + 5, 5)])
    gc.write_json_atomic(RES / f"pairs_{args.scene}_spread.json",
                         {"block_m": 5, "x_edges": xe.tolist(), "y_edges": ye.tolist(),
                          "endpoint_counts": H.astype(int).tolist(),
                          "max_block_share": float(H.max() / H.sum())})
    _log(f"plotted {path}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("step", choices=["evidence", "lattice", "sample", "plot"])
    ap.add_argument("--scene", default="scene_v2")
    ap.add_argument("--robots", default="sweeper,cylinder")
    ap.add_argument("--n", type=int, default=0)
    args = ap.parse_args(argv)
    cfg = gc.load_config()
    {"evidence": step_evidence, "lattice": step_lattice, "sample": step_sample, "plot": step_plot}[args.step](args, cfg)


if __name__ == "__main__":
    main()
