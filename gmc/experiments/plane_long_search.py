# gmc/experiments/plane_long_search.py
"""Amendment 3 P3 (option (a), user decision D4 of 2026-09-18): push A and B apart on the plane floor.

* Sweeper and uav: one shared start/goal with ||goal - start|| >= 8 m in a window <= 12 x 12 m (D2), chosen
  so the two robots take visibly different routes.
* Cylinder: the largest window it can actually certify. It is NOT run on the 12 x 12 case: at the
  cylinder's measured density that would be several times the 156 k-support map that already ran past the
  query cap in P2. Instead this script builds a *ladder* of cylinder cases that share the long case's
  start, with separations stepping up from P2's ~2.4 m, each sized by its exact support count, and the
  ladder is run smallest first until a rung stops certifying.

Sizing is by ``project_scene``'s own ``kept`` count (gmc.height.longrange, pinned by a unit test), never by
``showcase_run.py``'s probe, which under-estimated the cylinder by >= 13x in P2.

CLAIMS BOUNDARY (every artefact): the plane floor is a USER-APPROVED MANUAL SCENE EDIT (Amendment 3 P1),
not a reconstruction method; results are sound w.r.t. the edited scene only. Criterion 3 is relaxed by
user decision D1 to r + 0.05 m on each robot's own certified map. Criterion 1 is recorded, not required,
for the long case.

Run from gmc/: PYTHONPATH=src:experiments MPLBACKEND=Agg python experiments/plane_long_search.py --step search
"""
import argparse
import json
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LogNorm
from matplotlib.patches import Rectangle
from scipy.sparse.csgraph import dijkstra

from gmc.height.casesearch import (cell_of, d1_precheck, edt_clearance, free_components, free_graph,
                                   geodesic_path, line_clearance, overhang_on_line,
                                   path_from_predecessors, path_polyline, path_separation)
from gmc.height.longrange import (count_supports, density_grid, ladder_goals, min_pool_dist,
                                  route_window, shadow_table)
from gmc.height.planefloor import load_plane_scene
from gmc.height.prism import robot_table
from gmc.height.timing import StageTimer
from plane_case_search import (CLAIMS, SCENE_NPZ, Z_C, Sections, certified, clean, figure,
                               glass_clear)

OUT = Path("results/height/plane/long")
FIGS = OUT / "figs"
C = 0.025                      # certified pre-check raster
K = 4                          # coarse screening raster = 0.1 m (block-min of the certified clearance)
ROBOTS = ("sweeper", "cylinder", "uav")
PAIR = ("sweeper", "uav")      # the two robots of the long shared case
D_MIN = 8.0                    # D2 / D4: ||goal - start|| >= 8 m
MAX_SIDE = 12.0                # D2: window <= 12 x 12 m
MARGIN = 0.6                   # window = bbox(start, goal, both routes) + this
LATTICE = 0.5
EDGE = 0.75
PER_START = 12                 # goals kept per start for route extraction (8 by detour ratio + 4 spread)
FINALISTS = 6
MIN_SEPARATION = 0.5           # "the pair still shows different routes"
# Screening caps on the long window, from P2's measured runs: uav 19,867 supports -> 108 s compile /
# 172 s query; cylinder 156,426 -> 676 s / > 7,968 s (UNKNOWN). 60 k keeps the uav well inside the first.
CAP = {"sweeper": 20_000, "uav": 60_000}
LADDER_TARGETS = (2.5, 4.0, 5.5, 7.0, 8.5, 10.0, 11.5)
LADDER_GROW = 3.0              # the ladder's cylinder map = the long window grown by this, within DOMAIN
# Search domain: the hall below y = 32 (P1: the round-tables cluster north of it holds 18 of the
# 28.5 m2 residual near-floor over-approximation).
HALL = [-4.43357959985733, -0.9637084349989891, 21.088036155700674, 37.84499740600586]
DOMAIN = [-4.40, -0.95, 21.05, 32.0]
P2_WINDOW = [-2.5, 7.45, 1.8, 13.05]
P2_KEPT = {"sweeper": 1779, "cylinder": 156426, "uav": 19867}          # results/height/plane/shared/*.json
A2_WINDOWS = {"sweeper": [-0.85, 9.75, 1.15, 12.8], "cylinder": [0.5, 10.0, 3.5, 13.0],
              "uav": [6.0, 5.45, 11.9, 9.1]}                            # results/height/percase/*/case.json
T0 = time.time()


def log(*a):
    print(f"[{time.time() - T0:7.1f}s]", *a, flush=True)


def area(w):
    return (w[2] - w[0]) * (w[3] - w[1])


def pad_min_side(w, side, bounds):
    """Grow a too-thin window symmetrically to ``side`` metres (a 2.5 m straight rung is 1.2 m wide)."""
    w = list(w)
    for lo, hi in ((0, 2), (1, 3)):
        if w[hi] - w[lo] < side:
            c = 0.5 * (w[lo] + w[hi])
            w[lo], w[hi] = max(c - side / 2, bounds[lo]), min(c + side / 2, bounds[hi])
    return [round(v, 3) for v in w]


class SAT:
    """Summed-area table of shadow centres on a 0.1 m grid: an O(1) approximate count for screening only.

    Every number that is reported or decides a job comes from count_supports / project_scene instead."""

    def __init__(self, tab, ext, cell=0.1):
        H, _ = density_grid(tab, ext, cell)
        self.S = np.zeros((H.shape[0] + 1, H.shape[1] + 1))
        self.S[1:, 1:] = (H * cell * cell).cumsum(0).cumsum(1)
        self.ext, self.cell, self.pad = ext, cell, tab["r"] + 0.1

    def __call__(self, w):
        n0, n1 = self.S.shape[0] - 1, self.S.shape[1] - 1
        i0 = int(np.clip((w[0] - self.pad - self.ext[0]) / self.cell, 0, n0))
        i1 = int(np.clip(np.ceil((w[2] + self.pad - self.ext[0]) / self.cell), 0, n0))
        j0 = int(np.clip((w[1] - self.pad - self.ext[1]) / self.cell, 0, n1))
        j1 = int(np.clip(np.ceil((w[3] + self.pad - self.ext[1]) / self.cell), 0, n1))
        S = self.S
        return float(S[i1, j1] - S[i0, j1] - S[i1, j0] + S[i0, j0])


# ------------------------------------------------------------------------------------ exact evaluation
def evaluate_long(maps, win, robots, st, gl, sec, ceil_h, label):
    """P3's verdict on one window: >= 8 m, <= 12 x 12, D1 + connectivity for sweeper and uav, routes that
    differ. The cylinder is evaluated and drawn on the same window but is not part of the verdict (D4)."""
    out = {"label": label, "window": list(win), "start": list(st), "goal": list(gl), "z_c": Z_C,
           "window_side_m": [round(win[2] - win[0], 3), round(win[3] - win[1], 3)],
           "window_m2": round(area(win), 3), "dist_m": float(np.hypot(gl[0] - st[0], gl[1] - st[1])),
           "not_through_glass_adjacent_opening": glass_clear(win), "robots": {}, "n_supports": {},
           "supports_per_m2": {}}
    paths = {}
    for k in ROBOTS:
        r = robots[k].max_radius()
        dist = maps[k]["dist"]
        pre = d1_precheck(dist, win, C, st, gl, r)
        pre["line"] = line_clearance(dist, win, C, st, gl, r=r)
        pre["disc_free_frac"] = float((dist > r).mean())
        pre["occupied_frac"] = float(maps[k]["occ"].mean())
        pre["free_components"] = free_components(dist, r)[1]
        g = geodesic_path(dist, r, cell_of(st, win, dist.shape, C), cell_of(gl, win, dist.shape, C), C) \
            if pre["connected"] else None
        if g is not None:
            paths[k] = path_polyline(g[0], win, C)
            pre["path_len_m"] = float(g[1])
            pre["path_xy"] = paths[k].tolist()
        out["robots"][k] = pre
        out["n_supports"][k] = maps[k]["n_supports"]
        out["supports_per_m2"][k] = round(maps[k]["n_supports"] / area(win), 1)
    s, zb, zt, L = sec(st, gl)
    out["criterion1_recorded_not_required"] = overhang_on_line(s, zb, zt, L, z_c=Z_C, ceiling_height_m=ceil_h)
    if all(k in paths for k in PAIR):
        out["separation_m"] = path_separation(paths["sweeper"], paths["uav"])
        ls, lu = out["robots"]["sweeper"]["path_len_m"], out["robots"]["uav"]["path_len_m"]
        out["detour_ratio"] = max(ls, lu) / max(min(ls, lu), 1e-9)
        out["contrast"] = out["separation_m"] * out["detour_ratio"]
    else:
        out["separation_m"] = out["detour_ratio"] = out["contrast"] = None
    crit = {"dist_at_least_8m": out["dist_m"] >= D_MIN,
            "window_at_most_12x12": bool(out["window_side_m"][0] <= MAX_SIDE + 1e-9
                                         and out["window_side_m"][1] <= MAX_SIDE + 1e-9),
            "not_through_glass_adjacent_opening": out["not_through_glass_adjacent_opening"],
            "routes_differ_by_0.5m": bool((out["separation_m"] or 0.0) >= MIN_SEPARATION)}
    for k in PAIR:
        crit[f"d1_start_clear_{k}"] = out["robots"][k]["start_clear_ok"]
        crit[f"d1_goal_clear_{k}"] = out["robots"][k]["goal_clear_ok"]
        crit[f"connected_{k}"] = out["robots"][k]["connected"]
    out["criteria"] = crit
    out["failed_criteria"] = sorted(k for k, v in crit.items() if not v)
    out["pass"] = not out["failed_criteria"]
    cy = out["robots"]["cylinder"]
    out["cylinder_info_only"] = {"d1_ok": cy["ok"], "connected": cy["connected"],
                                 "start_clear_m": cy["start_clear_m"], "goal_clear_m": cy["goal_clear_m"],
                                 "n_supports": out["n_supports"]["cylinder"]}
    return out


def strip(ev):
    return {k: v for k, v in ev.items() if k != "robots"} | {
        "robots": {k: {kk: vv for kk, vv in ev["robots"][k].items() if kk != "path_xy"} for k in ROBOTS}}


def case_json(ev, z_f, meta, extra):
    c1 = ev["criterion1_recorded_not_required"]
    ov = ({"top": c1["top"], "underside": c1["underside"], "s_interval": c1["s_interval"],
           "longest_run_m": c1["longest_run_m"]} if c1.get("ok") else {})
    return {"window": ev["window"], "start": [ev["start"][0], ev["start"][1], 0.0],
            "goal": [ev["goal"][0], ev["goal"][1], 0.0], "z_floor": z_f, "z_c": Z_C, "overhang": ov,
            "dist_m": ev["dist_m"], "n_supports": ev["n_supports"], "supports_per_m2": ev["supports_per_m2"],
            "gravity_rotation": meta["gravity_rotation"], "scene": "planefloor", "claims_boundary": CLAIMS,
            "criterion1": "recorded, not required, for P3" } | extra


# ------------------------------------------------------------------------------------ the search
def step_search(args, scene=None, meta=None, validate=True):
    timer = StageTimer()
    with timer.stage("scene_load", scene="planefloor" if scene is None else "synthetic") as rec:
        if scene is None:
            scene, meta = load_plane_scene(SCENE_NPZ)
        rec["sizes"]["n_splats"] = len(scene)
    z_f, ceil_h = meta["z_floor"], meta["ceiling_height_m"]
    robots = robot_table(Z_C)
    OUT.mkdir(parents=True, exist_ok=True)
    FIGS.mkdir(parents=True, exist_ok=True)
    res = {"claims_boundary": CLAIMS, "task": "P3 (option (a)): long range on the plane floor",
           "scene_npz": str(SCENE_NPZ), "z_c": Z_C, "domain": DOMAIN, "d_min_m": D_MIN,
           "max_side_m": MAX_SIDE, "caps_screening": CAP}

    def save():
        (OUT / "p3_search.json").write_text(json.dumps(clean(res), indent=1))

    # ---- 1. sizing tables, validated against project_scene's own kept count
    tabs = {}
    for k in ROBOTS:
        t = time.time()
        tabs[k] = shadow_table(scene, robots[k], z_floor=z_f)
        log("shadow table", k, tabs[k]["n"], f"({time.time() - t:.0f}s)")
    val = {}
    if validate:
        val["P2_shared_window"] = {k: {"count": count_supports(tabs[k], P2_WINDOW), "kept": P2_KEPT[k]}
                                   for k in ROBOTS}
        p2a = json.loads(Path("results/height/plane/p2a_search.json").read_text())["regions"]
        for name in ("A", "SW"):
            box = p2a[name]["box"]
            val[f"P2_region_{name}"] = {k: {"count": count_supports(tabs[k], box),
                                            "kept": p2a[name]["n_supports"][k]} for k in ROBOTS}
    for v in val.values():
        for k in ROBOTS:
            v[k]["agree"] = v[k]["count"] == v[k]["kept"]
    res["count_validation"] = val
    res["count_validation_all_agree"] = all(v[k]["agree"] for v in val.values() for k in ROBOTS)
    log("count validation", json.dumps(val))
    grids = {k: density_grid(tabs[k], HALL, 0.5)[0] for k in ROBOTS}
    res["hall_supports_total"] = {k: tabs[k]["n"] for k in ROBOTS}
    res["density_0p5m_percentiles_per_m2"] = {
        k: {p: float(np.percentile(grids[k], p)) for p in (50, 75, 90, 99)} for k in ROBOTS}
    np.savez_compressed(OUT / "p3_density_0p5m.npz", extent=np.array(HALL), cell=0.5,
                        **{k: grids[k].astype(np.float32) for k in ROBOTS})
    save()
    sat = {k: SAT(tabs[k], HALL) for k in ROBOTS}

    # ---- 2. certified clearance over the domain for the pair, coarse screening graph
    fine, coarse, graphs, labs = {}, {}, {}, {}
    for k in PAIR:
        t = time.time()
        m = certified(scene, robots[k], DOMAIN, z_f, timer, f"domain:{k}")
        fine[k] = m["dist"]
        coarse[k] = min_pool_dist(m["dist"], K)
        graphs[k] = free_graph(coarse[k], robots[k].max_radius(), C * K)
        labs[k] = free_components(coarse[k], robots[k].max_radius())[0]
        log("domain map", k, "supports", m["n_supports"], f"({time.time() - t:.0f}s)")
        res.setdefault("domain_supports", {})[k] = m["n_supports"]
        del m
    shp_f, shp_c = fine["sweeper"].shape, coarse["sweeper"].shape
    xs = np.arange(DOMAIN[0] + EDGE, DOMAIN[2] - EDGE + 1e-9, LATTICE)
    ys = np.arange(DOMAIN[1] + EDGE, DOMAIN[3] - EDGE + 1e-9, LATTICE)
    pts = []
    for x in xs:
        for y in ys:
            if not glass_clear([x - 0.5, y - 0.5, x + 0.5, y + 0.5]):
                continue
            ok = True
            for k in PAIR:
                r = robots[k].max_radius()
                if fine[k][cell_of((x, y), DOMAIN, shp_f, C)] < r + 0.05:
                    ok = False
                    break
                if graphs[k][1][cell_of((x, y), DOMAIN, shp_c, C * K)] < 0:
                    ok = False
                    break
            if ok:
                pts.append((float(x), float(y)))
    P = np.array(pts)
    node = {k: np.array([graphs[k][1][cell_of(p, DOMAIN, shp_c, C * K)] for p in pts]) for k in PAIR}
    lab = {k: np.array([labs[k][cell_of(p, DOMAIN, shp_c, C * K)] for p in pts]) for k in PAIR}
    res["lattice_points_passing_D1_for_sweeper_and_uav"] = len(pts)
    log("lattice points passing D1 for sweeper and uav:", len(pts))
    save()

    # ---- 3. pairs >= 8 m, connected for both, routes that fit a <= 12 x 12 window
    rng = np.random.default_rng(0)
    rows, n_geom, n_conn = [], 0, 0
    order = np.arange(len(pts))
    for a in range(0, len(order), 40):
        grp = order[a:a + 40]
        dd, pp = {}, {}
        for k in PAIR:
            dd[k], pp[k] = dijkstra(graphs[k][0], directed=False, indices=node[k][grp],
                                    return_predecessors=True)
        for row, i in enumerate(grp):
            rel = P - P[i]
            D = np.hypot(rel[:, 0], rel[:, 1])
            geo = ((np.arange(len(P)) > i) & (D >= D_MIN)
                   & (np.abs(rel[:, 0]) <= MAX_SIDE - 2 * MARGIN) & (np.abs(rel[:, 1]) <= MAX_SIDE - 2 * MARGIN))
            n_geom += int(geo.sum())
            conn = geo & (lab["sweeper"] == lab["sweeper"][i]) & (lab["uav"] == lab["uav"][i])
            Ls = dd["sweeper"][row][node["sweeper"]]
            Lu = dd["uav"][row][node["uav"]]
            conn &= np.isfinite(Ls) & np.isfinite(Lu)
            n_conn += int(conn.sum())
            js = np.flatnonzero(conn)
            if not len(js):
                continue
            ratio = np.maximum(Ls[js], Lu[js]) / np.maximum(np.minimum(Ls[js], Lu[js]), 1e-9)
            pick = list(js[np.argsort(-ratio)[:PER_START - 4]])
            rest = np.setdiff1d(js, pick)
            if len(rest):
                pick += list(rng.choice(rest, size=min(4, len(rest)), replace=False))
            for j in pick:
                xy = {}
                for k in PAIR:
                    ij = path_from_predecessors(pp[k][row], graphs[k][2], shp_c, node[k][i], node[k][j])
                    xy[k] = None if ij is None else path_polyline(ij, DOMAIN, C * K)
                if xy["sweeper"] is None or xy["uav"] is None:
                    continue
                w = route_window([P[[i, j]], xy["sweeper"], xy["uav"]], MARGIN, MAX_SIDE, bounds=DOMAIN)
                if w is None or not glass_clear(w):
                    continue
                sep = path_separation(xy["sweeper"], xy["uav"], step=0.05)
                ls, lu = float(Ls[j]), float(Lu[j])
                rt = max(ls, lu) / max(min(ls, lu), 1e-9)
                rows.append({"start": P[i].tolist(), "goal": P[j].tolist(), "dist_m": float(D[j]),
                             "window": w, "len_sweeper_m": ls, "len_uav_m": lu, "separation_m": sep,
                             "detour_ratio": rt, "contrast": sep * rt,
                             "approx_supports": {k: sat[k](w) for k in ROBOTS}})
        if a % 400 == 0:
            log(f"starts {a + len(grp)}/{len(order)}: geometric pairs {n_geom}, connected {n_conn}, "
                f"with a <= 12x12 route window {len(rows)}")
    res["pairs_at_least_8m_in_12x12_reach"] = n_geom
    res["pairs_connected_for_both"] = n_conn
    res["pairs_with_route_window"] = len(rows)
    under = [r for r in rows if all(r["approx_supports"][k] <= CAP[k] for k in PAIR)]
    res["pairs_under_support_caps"] = len(under)
    diff = [r for r in under if r["separation_m"] >= MIN_SEPARATION]
    res["pairs_with_routes_differing_by_0.5m"] = len(diff)
    log("pairs: route window", len(rows), "under caps", len(under), "routes differ", len(diff))
    pool = sorted(diff or under, key=lambda r: -r["contrast"])
    res["top_screened"] = pool[:40]
    save()

    # ---- 4. exact pre-check of the finalists, each with its own projection
    sec = Sections(scene, z_f)
    picked = []
    for r in pool:
        if any(np.hypot(*(np.subtract(r["start"], q["start"]))) < 1.5
               and np.hypot(*(np.subtract(r["goal"], q["goal"]))) < 1.5 for q in picked):
            continue
        picked.append(r)
        if len(picked) >= FINALISTS:
            break
    res["finalists"] = []
    chosen = None
    for i, r in enumerate(picked):
        w = r["window"]
        t = time.time()
        maps = {k: certified(scene, robots[k], w, z_f, timer, f"final{i}:{k}") for k in ROBOTS}
        ev = evaluate_long(maps, w, robots, r["start"], r["goal"], sec, ceil_h, f"P3-long-{i}")
        ev["screened"] = {kk: r[kk] for kk in ("separation_m", "detour_ratio", "contrast", "approx_supports")}
        ev["exact_supports_count_supports"] = {k: count_supports(tabs[k], w) for k in ROBOTS}
        ev["seconds"] = round(time.time() - t, 1)
        log("finalist", i, "pass", ev["pass"], "dist", round(ev["dist_m"], 2), "window", w,
            "supports", json.dumps(ev["n_supports"]), "sep", ev["separation_m"], "failed", ev["failed_criteria"])
        figure(maps, w, robots, r["start"], r["goal"], ev, FIGS / f"p3_long_finalist{i}.png",
               f"P3 long-range finalist {i}: {'PASSES' if ev['pass'] else 'FAILS ' + ','.join(ev['failed_criteria'])}"
               f" | |goal-start| {ev['dist_m']:.2f} m | window {ev['window_side_m'][0]:.2f} x "
               f"{ev['window_side_m'][1]:.2f} m | sweeper-uav routes separate by "
               f"{(ev['separation_m'] or 0):.2f} m | cylinder drawn for information only (D4)")
        res["finalists"].append(strip(ev))
        if ev["pass"] and chosen is None:
            chosen = (ev, maps)
            res["finalists"][-1]["chosen"] = True
        save()
    res["long_case_exists"] = chosen is not None
    if chosen is None:
        res["timing"] = timer.to_dict()
        save()
        log("NO long case passed the exact pre-check")
        return
    ev, maps = chosen
    case = case_json(ev, z_f, meta, {
        "label": ev["label"], "pass": True, "criteria": ev["criteria"], "separation_m": ev["separation_m"],
        "detour_ratio": ev["detour_ratio"], "precheck": strip(ev)["robots"],
        "selection": {"search_file": str(OUT / "p3_search.json"), "label": ev["label"],
                      "method": "project_scene -> _support_raster(0.025) -> EDT; D1 per robot; "
                                ">= 8 m; window <= 12 x 12 from bbox(start, goal, both routes) + 0.6 m"}})
    (OUT / "case.json").write_text(json.dumps(clean(case), indent=2))
    log("WROTE", OUT / "case.json")

    # ---- 5. the cylinder ladder: same start, separations stepping up from P2's ~2.4 m
    win, st, gl = ev["window"], ev["start"], ev["goal"]
    # The ladder's map is the long window grown by LADDER_GROW, so that where the cylinder cannot take the
    # sweeper/uav corridor (a table it cannot pass under) the ladder is stopped by compute, not by the crop.
    big = [round(max(win[0] - LADDER_GROW, DOMAIN[0]), 3), round(max(win[1] - LADDER_GROW, DOMAIN[1]), 3),
           round(min(win[2] + LADDER_GROW, DOMAIN[2]), 3), round(min(win[3] + LADDER_GROW, DOMAIN[3]), 3)]
    cy = certified(scene, robots["cylinder"], big, z_f, timer, "ladder_map:cylinder")
    rc = robots["cylinder"].max_radius()
    need = rc + 0.05
    lab_c, _ = free_components(cy["dist"], rc)
    anchor, toward = None, None
    for a_, b_ in ((st, gl), (gl, st)):
        ca = cell_of(a_, big, cy["dist"].shape, C)
        if cy["dist"][ca] >= need and lab_c[ca]:
            anchor, toward = a_, b_
            break
    lad = {"long_case_window": win, "ladder_map_window": big, "ladder_map_supports": cy["n_supports"],
           "long_case_start": st, "long_case_goal": gl,
           "targets_m": list(LADDER_TARGETS), "rungs": []}
    if anchor is None:
        # nearest admissible cylinder point to the start
        cand = [(x, y) for x in np.arange(big[0] + 0.5, big[2] - 0.5, 0.25)
                for y in np.arange(big[1] + 0.5, big[3] - 0.5, 0.25)
                if cy["dist"][cell_of((x, y), big, cy["dist"].shape, C)] >= need]
        if cand:
            anchor = min(cand, key=lambda p: np.hypot(p[0] - st[0], p[1] - st[1]))
            toward = gl
    lad["anchor"], lad["toward"] = anchor, toward
    if anchor is not None:
        ca = cell_of(anchor, big, cy["dist"].shape, C)
        cand = np.array([(x, y) for x in np.arange(big[0] + 0.5, big[2] - 0.5 + 1e-9, 0.25)
                         for y in np.arange(big[1] + 0.5, big[3] - 0.5 + 1e-9, 0.25)
                         if cy["dist"][cell_of((x, y), big, cy["dist"].shape, C)] >= need
                         and lab_c[cell_of((x, y), big, cy["dist"].shape, C)] == lab_c[ca]])
        full = float(np.hypot(toward[0] - anchor[0], toward[1] - anchor[1]))
        targets = [t for t in LADDER_TARGETS if t < full - 0.75] + [full]
        goals = ladder_goals(anchor, cand, targets, toward=toward, tol=0.3) if len(cand) else []
        if goals and goals[-1]["goal"] is None and lab_c[cell_of(toward, big, cy["dist"].shape, C)] == lab_c[ca]:
            goals[-1].update(goal=list(toward), dist_m=full, off_ray_m=0.0)
        for n, g in enumerate(goals):
            rung = {"rung": n, **g}
            if g["goal"] is None:
                rung["why"] = "no admissible cylinder goal at this separation on the ladder map (long window + %.1f m)" % LADDER_GROW + ""
                lad["rungs"].append(rung)
                continue
            gp = geodesic_path(cy["dist"], rc, ca, cell_of(g["goal"], big, cy["dist"].shape, C), C)
            route = path_polyline(gp[0], big, C) if gp else np.array([anchor, g["goal"]])
            w = route_window([np.array([anchor, g["goal"]]), route], MARGIN, MAX_SIDE, bounds=big)
            if w is None:
                rung["why"] = "route window exceeds 12 x 12"
                lad["rungs"].append(rung)
                continue
            w = pad_min_side(w, 2.0, big)
            m = certified(scene, robots["cylinder"], w, z_f, timer, f"rung{n}:cylinder")
            pre = d1_precheck(m["dist"], w, C, anchor, g["goal"], rc)
            gg = geodesic_path(m["dist"], rc, cell_of(anchor, w, m["dist"].shape, C),
                               cell_of(g["goal"], w, m["dist"].shape, C), C) if pre["connected"] else None
            rung.update(window=w, window_m2=round(area(w), 3),
                        window_side_m=[round(w[2] - w[0], 3), round(w[3] - w[1], 3)],
                        n_supports=m["n_supports"], supports_per_m2=round(m["n_supports"] / area(w), 1),
                        d1=pre, route_len_m=(float(gg[1]) if gg else None),
                        route_xy=(path_polyline(gg[0], w, C).tolist() if gg else None),
                        precheck_pass=bool(pre["ok"] and gg is not None))
            log("rung", n, "target", g["target_m"], "dist", round(g["dist_m"], 2), "window", w,
                "supports", m["n_supports"], "precheck", rung["precheck_pass"])
            if rung["precheck_pass"]:
                d = OUT / "cyl_ladder" / f"rung{n}"
                d.mkdir(parents=True, exist_ok=True)
                (d / "case.json").write_text(json.dumps(clean({
                    "window": w, "start": [anchor[0], anchor[1], 0.0], "goal": [g["goal"][0], g["goal"][1], 0.0],
                    "z_floor": z_f, "z_c": Z_C, "overhang": {}, "dist_m": g["dist_m"],
                    "n_supports": {"cylinder": m["n_supports"]}, "gravity_rotation": meta["gravity_rotation"],
                    "scene": "planefloor", "claims_boundary": CLAIMS, "label": f"P3-cyl-rung{n}",
                    "ladder": {"rung": n, "target_m": g["target_m"], "anchor_is_long_case":
                               "start" if anchor == st else ("goal" if anchor == gl else "nearest admissible"),
                               "long_case": str(OUT / "case.json")},
                    "precheck": {"cylinder": pre}}), indent=2))
                rung["case_file"] = str(d / "case.json")
            lad["rungs"].append(rung)
            (OUT / "cyl_ladder.json").write_text(json.dumps(clean(lad), indent=1))
            del m
    (OUT / "cyl_ladder.json").write_text(json.dumps(clean(lad), indent=1))
    res["ladder_file"] = str(OUT / "cyl_ladder.json")

    # ---- 6. figures: hall density with every window, and the ladder over the cylinder map
    fig, axs = plt.subplots(1, 3, figsize=(24, 12))
    for ax, k in zip(axs, ROBOTS):
        g = np.where(grids[k] > 0, grids[k], np.nan)
        im = ax.imshow(g.T, origin="lower", extent=[HALL[0], HALL[0] + g.shape[0] * 0.5,
                                                     HALL[1], HALL[1] + g.shape[1] * 0.5],
                       norm=LogNorm(vmin=10, vmax=max(1e3, np.nanmax(g))), cmap="magma_r")
        plt.colorbar(im, ax=ax, fraction=0.04, label="supports per m² (0.5 m bins, by shadow centre)")
        for w, col, lbl in ((P2_WINDOW, "red", f"P2 shared {P2_KEPT[k]:,}"),
                            (A2_WINDOWS[k], "orange", "Amendment 2 (unedited scene)"),
                            (win, "blue", f"P3 long {ev['n_supports'][k]:,}")):
            ax.add_patch(Rectangle((w[0], w[1]), w[2] - w[0], w[3] - w[1], fill=False, ec=col, lw=2, label=lbl))
        if k == "cylinder":
            for rg in lad["rungs"]:
                if rg.get("window"):
                    w = rg["window"]
                    ax.add_patch(Rectangle((w[0], w[1]), w[2] - w[0], w[3] - w[1], fill=False, ec="green",
                                           lw=1, ls="--"))
        ax.plot([st[0], gl[0]], [st[1], gl[1]], "b-", lw=1)
        ax.set_title(f"{k}: {tabs[k]['n']:,} supports hall-wide; window counts are project_scene's kept",
                     fontsize=9)
        ax.legend(fontsize=7, loc="upper left")
    fig.suptitle("P3 sizing: where the supports are, per robot (green dashed = cylinder ladder rungs)\n"
                 + CLAIMS, fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(FIGS / "p3_density.png", dpi=70)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(14, 12))
    img = np.ones(cy["occ"].shape[::-1] + (3,))
    img[(cy["dist"] > rc).T] = (0.75, 0.95, 0.75)
    img[cy["occ"].T] = (0.8, 0.1, 0.1)
    ax.imshow(img, origin="lower", extent=[big[0], big[2], big[1], big[3]], interpolation="nearest")
    ax.add_patch(Rectangle((win[0], win[1]), win[2] - win[0], win[3] - win[1], fill=False, ec="blue", lw=2,
                           ls=":", label="the long sweeper/uav window"))
    ax.plot([st[0], gl[0]], [st[1], gl[1]], "b:", lw=1)
    for rg in lad["rungs"]:
        if not rg.get("window"):
            continue
        w = rg["window"]
        ax.add_patch(Rectangle((w[0], w[1]), w[2] - w[0], w[3] - w[1], fill=False, ec="k", lw=1.2))
        ax.plot(*rg["goal"], "bs", ms=7)
        ax.annotate(f"rung {rg['rung']}: {rg['dist_m']:.1f} m, {rg['n_supports']:,} sup"
                    + ("" if rg["precheck_pass"] else " (pre-check FAILS)"),
                    rg["goal"], fontsize=8, xytext=(4, 4), textcoords="offset points")
        if rg.get("route_xy"):
            p = np.array(rg["route_xy"])
            ax.plot(p[:, 0], p[:, 1], "-", lw=1.2)
    if anchor is not None:
        ax.plot(*anchor, "go", ms=10)
    ax.set_title("P3 cylinder ladder on the long case's cylinder map (red = certified-occupied, green = disc "
                 "fits); each rung is one pf_ job, run smallest first\n" + CLAIMS, fontsize=8)
    fig.tight_layout()
    fig.savefig(FIGS / "p3_cyl_ladder.png", dpi=80)
    plt.close(fig)
    res["timing"] = timer.to_dict()
    res["seconds"] = round(time.time() - T0, 1)
    save()
    log("done")


# ------------------------------------------------------------------------------------ report (JSON only)
COLORS = {"sweeper": "#2a78d6", "cylinder": "#eb6834", "uav": "#1baf7a"}     # fixed per robot, validated
MARKERS = {"sweeper": "o", "cylinder": "s", "uav": "^"}                      # second encoding (aqua < 3:1)
QUERY_CAP_S = 7200.0                                                         # configs/height_showcase.yaml


def run_row(path, label):
    d = json.loads(Path(path).read_text())
    c, r = d["case"], d.get("result") or {}
    ver, rep, pr = r.get("verify") or {}, d.get("replay3d") or {}, d.get("probe") or {}
    w = c["window"]
    k = int(d.get("budget_scale", 1))
    row = {"label": label, "robot": d["robot"], "file": str(path), "scene": d.get("scene", "processed"),
           "budget_scale": k, "window": w, "window_side_m": [round(w[2] - w[0], 3), round(w[3] - w[1], 3)],
           "window_m2": round(area(w), 3),
           "dist_m": float(np.hypot(c["goal"][0] - c["start"][0], c["goal"][1] - c["start"][1])),
           "supports": d["projection"]["kept"], "supports_per_m2": round(d["projection"]["kept"] / area(w), 1),
           "compile_s": r.get("compile_seconds"), "query_s": r.get("query_seconds"),
           "query_cap_s": QUERY_CAP_S * k, "status": r.get("status"), "reason": r.get("reason"),
           "verify_certified": bool(ver.get("certified")), "verify_min_clearance_m": ver.get("min_clearance"),
           "replay3d_passed": bool(rep.get("passed")), "clearance_lb_m": rep.get("min_clearance_lb"),
           "success": bool(r.get("status") == "REACHABLE" and ver.get("certified") and rep.get("passed")),
           "timing_by_stage_s": (d.get("timing") or {}).get("by_stage"),
           "probe_projected_h": pr.get("projected_hours")}
    if row["compile_s"] is not None and row["query_s"] is not None and row["probe_projected_h"]:
        actual = (row["compile_s"] + row["query_s"]) / 3600.0
        row["probe_underestimate_x"] = actual / row["probe_projected_h"]
        row["probe_underestimate_is_lower_bound"] = row["status"] == "UNKNOWN"
    return row


def power_fit(x, y):
    """log-log least squares: y = a * x**b. Only called with >= 4 points."""
    b, la = np.polyfit(np.log(x), np.log(y), 1)
    return {"a": float(np.exp(la)), "b": float(b), "n": int(len(x)),
            "x_range": [float(min(x)), float(max(x))]}


def step_report(args):
    runs = []
    for k in ROBOTS:
        runs.append(run_row(f"results/height/percase/{k}/{k}.json", f"A2 {k}"))
    for k in ROBOTS:
        for suf in ("", "_b2"):
            f = Path(f"results/height/plane/shared/{k}{suf}.json")
            if f.exists():
                runs.append(run_row(f, f"P2 shared {k}{suf}"))
    for k in PAIR:
        for suf in ("", "_b2"):
            f = OUT / f"{k}{suf}.json"
            if f.exists():
                runs.append(run_row(f, f"P3 long {k}{suf}"))
    lad = json.loads((OUT / "cyl_ladder.json").read_text()) if (OUT / "cyl_ladder.json").exists() else {"rungs": []}
    for rg in lad["rungs"]:
        for suf in ("", "_b2"):
            f = OUT / "cyl_ladder" / f"rung{rg['rung']}" / f"cylinder{suf}.json"
            if f.exists():
                runs.append(run_row(f, f"P3 rung {rg['rung']}{suf}"))

    # supports vs window area: every GMC run plus every exact projection we have
    proj = []
    srch = json.loads((OUT / "p3_search.json").read_text())
    for f in srch.get("finalists", []):
        for k in ROBOTS:
            proj.append({"src": f"P3 {f['label']}", "robot": k, "window_m2": f["window_m2"],
                         "supports": f["n_supports"][k]})
    for rg in lad["rungs"]:
        if rg.get("n_supports") is not None:
            proj.append({"src": f"P3 rung {rg['rung']}", "robot": "cylinder", "window_m2": rg["window_m2"],
                         "supports": rg["n_supports"]})
    if lad.get("ladder_map_window"):
        proj.append({"src": "P3 ladder map", "robot": "cylinder", "window_m2": round(area(lad["ladder_map_window"]), 3),
                     "supports": lad["ladder_map_supports"]})
    ev2 = Path("results/height/plane/p2d_evidence.json")
    if ev2.exists():
        for w in json.loads(ev2.read_text()).get("windows", []):
            if w.get("window"):
                for k in ROBOTS:
                    proj.append({"src": f"P2 shrink margin {w['margin']}", "robot": k,
                                 "window_m2": round(area(w["window"]), 3), "supports": w["n_supports"][k]})
    p2a = json.loads(Path("results/height/plane/p2a_search.json").read_text())["regions"]
    for name, rg in p2a.items():
        for k in ROBOTS:
            proj.append({"src": f"P2 region {name}", "robot": k, "window_m2": rg["area_m2"],
                         "supports": rg["n_supports"][k]})

    # the cylinder at 12 x 12, from the hall density grid (0.5 m bins by shadow centre)
    dz = np.load(OUT / "p3_density_0p5m.npz")
    ext, cell = [float(v) for v in dz["extent"]], float(dz["cell"])
    twelve = {}
    for k in ROBOTS:
        g = dz[k].astype(float) * cell * cell                  # supports per bin
        j_max = int(round((DOMAIN[3] - ext[1]) / cell))        # windows inside the search domain only
        g = g[:, :j_max]
        n = int(round(12.0 / cell))
        S = np.zeros((g.shape[0] + 1, g.shape[1] + 1))
        S[1:, 1:] = g.cumsum(0).cumsum(1)
        tot = S[n:, n:] - S[:-n, n:] - S[n:, :-n] + S[:-n, :-n]
        twelve[k] = {"windows": int(tot.size), "min": float(tot.min()), "p10": float(np.percentile(tot, 10)),
                     "median": float(np.median(tot)), "p90": float(np.percentile(tot, 90)),
                     "max": float(tot.max()),
                     "note": "every 12 x 12 m window on a 0.5 m step inside y <= 32, counted by shadow "
                             "centre (omits the r-dilated rim, so slightly under project_scene's kept)"}
    cyl = [r for r in runs if r["robot"] == "cylinder" and r["compile_s"] is not None]
    fits = {"compile": None, "query": None}
    xs = [r["supports"] for r in cyl]
    if len(cyl) >= 4:
        fits["compile"] = power_fit(xs, [r["compile_s"] for r in cyl])
    fin = [r for r in cyl if r["status"] in ("REACHABLE", "UNREACHABLE")]
    if len(fin) >= 4:
        fits["query"] = power_fit([r["supports"] for r in fin], [r["query_s"] for r in fin])
    extrap = {"cylinder_supports_12x12": twelve["cylinder"], "fits": fits,
              "n_cylinder_runs_with_compile": len(cyl), "n_cylinder_runs_with_finished_query": len(fin),
              "rule": "no fit on fewer than 4 points"}
    for key in ("compile", "query"):
        f = fits[key]
        if f:
            for q in ("min", "median", "max"):
                N = twelve["cylinder"][q]
                extrap[f"{key}_s_at_12x12_{q}"] = f["a"] * N ** f["b"]
                extrap[f"{key}_extrapolation_factor_beyond_range_{q}"] = N / f["x_range"][1]
    out = {"claims_boundary": CLAIMS, "runs": runs, "projections": proj, "supports_12x12": twelve,
           "cylinder_extrapolation": extrap,
           "probe": [{k: r.get(k) for k in ("label", "robot", "supports", "probe_projected_h",
                                            "probe_underestimate_x", "probe_underestimate_is_lower_bound",
                                            "status")} for r in runs if r.get("probe_projected_h")],
           "option_b": "all three robots at >= 8 m via the Amendment 1 grid coreset: NOT RUN (D4: the user "
                       "asked for option (a) first); it is the next step."}
    (OUT / "p3_report.json").write_text(json.dumps(clean(out), indent=1))

    fig, axs = plt.subplots(1, 3, figsize=(21, 6.5))
    for k in ROBOTS:
        pp = [p for p in proj if p["robot"] == k and p["supports"]]
        axs[0].scatter([p["window_m2"] for p in pp], [p["supports"] for p in pp], s=40, marker=MARKERS[k],
                       facecolors="none", edgecolors=COLORS[k], lw=1.2, label=f"{k}: projection only")
        rr = [r for r in runs if r["robot"] == k]
        axs[0].scatter([r["window_m2"] for r in rr], [r["supports"] for r in rr], s=70, marker=MARKERS[k],
                       color=COLORS[k], edgecolors="white", lw=1.5, label=f"{k}: GMC run")
        for ax, key in ((axs[1], "compile_s"), (axs[2], "query_s")):
            ok = [r for r in rr if r[key] is not None and r["status"] != "UNKNOWN"]
            bad = [r for r in rr if r[key] is not None and r["status"] == "UNKNOWN"]
            ax.scatter([r["supports"] for r in ok], [r[key] for r in ok], s=70, marker=MARKERS[k],
                       color=COLORS[k], edgecolors="white", lw=1.5, label=f"{k}")
            if bad:
                ax.scatter([r["supports"] for r in bad], [r[key] for r in bad], s=90, marker=MARKERS[k],
                           facecolors="none", edgecolors=COLORS[k], lw=2,
                           label=f"{k}: UNKNOWN (query hit the cap)" if key == "query_s" else f"{k}: UNKNOWN run")
    for ax in axs:
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.grid(alpha=0.25, lw=0.5)
        ax.tick_params(labelsize=8)
    axs[0].axvline(144, color="0.4", ls="--", lw=1)
    axs[0].text(144, axs[0].get_ylim()[0] * 1.5, " 12 x 12 m", fontsize=8, color="0.3")
    axs[0].set_xlabel("window area (m²)")
    axs[0].set_ylabel("supports (project_scene kept)")
    axs[0].set_title("supports vs window area, per robot", fontsize=10)
    axs[1].set_xlabel("supports")
    axs[1].set_ylabel("compile seconds")
    axs[1].set_title("compile time vs supports (every GMC run)", fontsize=10)
    axs[2].set_xlabel("supports")
    axs[2].set_ylabel("query seconds")
    for cap, lbl in ((QUERY_CAP_S, "query cap 7,200 s"), (2 * QUERY_CAP_S, "BUDGET=2 cap 14,400 s")):
        axs[2].axhline(cap, color="0.4", ls="--", lw=1)
        axs[2].text(axs[2].get_xlim()[0] * 1.1, cap * 1.08, lbl, fontsize=8, color="0.3")
    axs[2].set_title("query time vs supports (hollow = UNKNOWN, a lower bound)", fontsize=10)
    for ax in axs:
        ax.legend(fontsize=7, loc="upper left")
    fig.suptitle("P3: why only this is doable — Amendment 2, P2 and P3 runs on one set of axes\n" + CLAIMS,
                 fontsize=8)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    FIGS.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGS / "p3_scaling.png", dpi=90)
    plt.close(fig)
    log("wrote", OUT / "p3_report.json", FIGS / "p3_scaling.png")

def step_selftest(args):
    """Every code path of --step search on synth3d's table scene, before a Slurm job is spent on the hall.

    The scene is 6 x 4 m, so the distances shrink with it (>= 3.5 m, <= 6.5 m windows) and the "routes
    differ" clause is off: sweeper and uav both go straight through the table gap there. The cylinder
    has to go round through the north gap, so the ladder path is exercised on a real detour.
    """
    global OUT, FIGS, DOMAIN, HALL, D_MIN, MAX_SIDE, LATTICE, EDGE, MIN_SEPARATION, LADDER_TARGETS, MARGIN, \
        LADDER_GROW
    from gmc.height.synth3d import WORKSPACE, Z_FLOOR, table_scene
    scene, _ = table_scene("open")
    meta = {"z_floor": Z_FLOOR, "ceiling_height_m": 2.5, "gravity_rotation": np.eye(3).tolist()}
    OUT = Path("results/height/plane/long/_selftest")
    FIGS = OUT / "figs"
    DOMAIN = HALL = list(WORKSPACE)
    D_MIN, MAX_SIDE, LATTICE, EDGE, MIN_SEPARATION, MARGIN = 3.5, 6.5, 0.25, 0.6, 0.0, 0.6
    LADDER_GROW = 3.0
    LADDER_TARGETS = (1.5, 2.5, 3.5)
    step_search(args, scene=scene, meta=meta, validate=False)
    res = json.loads((OUT / "p3_search.json").read_text())
    lad = json.loads((OUT / "cyl_ladder.json").read_text())
    fail = []
    if not res.get("long_case_exists"):
        fail.append("no case on a scene built to have one")
    if not any(r.get("precheck_pass") for r in lad["rungs"]):
        fail.append("no ladder rung passes its own pre-check")
    for r in lad["rungs"]:
        if r.get("window") and r["n_supports"] != count_supports(
                shadow_table(scene, robot_table(Z_C)["cylinder"], z_floor=Z_FLOOR), r["window"]):
            fail.append(f"rung {r['rung']} support count disagrees with count_supports")
    log("selftest ladder", json.dumps(clean([{k: v for k, v in r.items() if k not in ("route_xy", "d1")}
                                             for r in lad["rungs"]])))
    if fail:
        raise SystemExit("SELFTEST FAILED: " + "; ".join(fail))
    log("SELFTEST PASSED")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", required=True, choices=["search", "selftest", "report"])
    args = ap.parse_args()
    {"search": step_search, "selftest": step_selftest, "report": step_report}[args.step](args)


if __name__ == "__main__":
    main()
