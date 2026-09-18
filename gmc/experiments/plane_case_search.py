# gmc/experiments/plane_case_search.py
"""Amendment 3 P2a/P2b: the shared three-robot case search, on the plane-floor scene.

One start, one goal, one window, one scene, three robots. A candidate is ranked by *expected
morphological contrast* — the straight segment must be usable by the sweeper (band [0.02, 0.10]) and
blocked for the cylinder (band [0.02, 1.75]), start and goal connected for both, and the two routes must
actually run apart — not merely by "all three are solvable".

CLAIMS BOUNDARY, and it belongs on every artefact this script writes: the plane floor is a
USER-APPROVED MANUAL SCENE-DEFINITION CHANGE (Amendment 3 P1), not a reconstruction method and not a
guaranteed outer approximation. Results are sound with respect to the edited scene only; the gap between
that and the room is the open research question (docs/worklog/height_map_diagnosis.md section 4).

Criterion 3 is relaxed by user decision D1 to `r + 0.05` m on each robot's own certified map. That is a
user-approved criterion change, not a discovery: criteria 1 and 3 intersect in 0.9 m2 of the 990 m2 hall
as built and 1.2 m2 with every phantom cell deleted (height_map_diagnosis.md section 3), so the 0.5 m
form is not something P1 could have fixed. Criterion 1 is unchanged.

Every number that decides a case comes from the CERTIFIED map:
    project_scene(robot, window) -> showcase_scene._support_raster(s2, window, 0.025) -> EDT.
showcase_scene._occupancy (the xy-AABB band map) is a selection aid only and is far more pessimistic; it
is not used here.

Steps (each writes its JSON before returning):
  search   region-scale certified rasters, endpoint lattice, pair enumeration, contrast ranking,
           candidate windows, then the EXACT per-window pre-check on the finalists, figures, and
           results/height/plane/case_shared.json for the winner.
  evidence P2d: for the best few candidate windows, per robot, the certified map, the free space
           dist > r, its components with start and goal marked, and the exact criterion that fails.

Run from gmc/: PYTHONPATH=src:experiments MPLBACKEND=Agg python experiments/plane_case_search.py --step search
"""
import argparse
import json
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.sparse.csgraph import dijkstra

from gmc.height.casesearch import (cell_of, contrast, d1_precheck, edt_clearance, free_components,
                                   free_graph, geodesic_path, line_clearance, overhang_on_line,
                                   path_from_predecessors, path_polyline, path_separation,
                                   same_component, with_border)
from gmc.height.pathio import curve_from_dict, sample_curve
from gmc.height.planefloor import load_plane_scene
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.height.timing import StageTimer
from showcase_scene import RASTER, RHO, TAU, _section, _support_raster

SCENE_NPZ = Path("/scratch/wg2381/splathjb/gmc/outputs/height/plane/processed_planefloor.npz")
OUT = Path("results/height/plane")
FIGS = OUT / "figs"
CLAIMS = ("plane floor = USER-APPROVED MANUAL SCENE EDIT (Amendment 3 P1), not a reconstruction method "
          "and not a guaranteed outer approximation; sound w.r.t. the edited scene only. Criterion 3 "
          "relaxed by user decision D1 to r + 0.05 m (crit1 & crit3 = 0.9 m2 as built / 1.2 m2 with "
          "every phantom cell deleted, height_map_diagnosis.md section 3). Criterion 1 unchanged.")

C = RASTER                      # 0.025 m, the certified pre-check raster
Z_C = 1.20                      # uav band centre, as in Amendment 1/2
ROBOTS = ("sweeper", "cylinder", "uav")
LATTICE = 0.25                  # endpoint lattice, metres
EDGE = 0.75                     # endpoints at least this far inside a region (its border is forced occupied)
L_MIN, L_MAX = 2.0, 6.0         # P2 is the short-range case; P3 does >= 8 m
MARGINS = (0.6, 0.9, 1.3, 1.8)  # window margin around the start-goal box
MAX_SIDE, MIN_SIDE = 8.0, 2.0   # spec's window cap (D2 lifts it to 12 x 12 for P3 only)
TOP_PAIRS = 400                 # pairs kept for the geodesic stage
SECTION_PAIRS = 120             # pairs kept for the criterion-1 section stage
WINDOW_CANDIDATES = 40          # windows screened on the cropped region raster
FINALISTS = 6                   # windows re-checked exactly, with their own projection

# Regions from the A2/A4 object catalogue (worklog height_bands.md), all far from the glass doors.
# P1 measured that the north round-tables cluster holds 18 of the 28.5 m2 of residual near-floor
# over-approximation, so nothing is sited in y > 32.
REGIONS = [
    ("A", [4.5, 2.5, 13.5, 11.5], "table A (8.96,7.29) top 0.80 with a bench along its NE side"),
    ("SW", [-2.5, 5.5, 6.5, 16.0], "SW bench (2.1,10.5) seat 0.43; table B (-0.45,10.81) top ~0.8"),
    ("E", [6.5, 10.5, 16.5, 19.5], "hall bench (10,16) seat 0.43; table C (13.28,14.09) top 0.93"),
    ("G", [-0.5, 18.5, 7.5, 26.5], "table G (3.35,22.53) top 0.95; object D (5.8,18)"),
    ("N", [3.5, 24.5, 12.5, 32.0], "north bench (7.8,28.5); object E (11,28); clipped at y=32 (P1)"),
]
GLASS_BOX = (-4.4, 0.4, -3.4, 3.1)   # glass doors, dilated by 0.5 m (A2)
T0 = time.time()


def log(*a):
    print(f"[{time.time() - T0:7.1f}s]", *a, flush=True)


def clean(o):
    if isinstance(o, dict):
        return {k: clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, (np.bool_, bool)):
        return bool(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating, float)):
        return None if not np.isfinite(o) else round(float(o), 6)
    if isinstance(o, np.ndarray):
        return clean(o.tolist())
    return o


def glass_clear(win):
    return not (win[0] < GLASS_BOX[2] and win[2] > GLASS_BOX[0]
                and win[1] < GLASS_BOX[3] and win[3] > GLASS_BOX[1])


# ------------------------------------------------------------------------------------ certified maps
def certified(scene, robot, win, z_f, timer=None, label=None):
    """project_scene -> _support_raster(0.025) -> EDT. The only map any decision here is made on."""
    if timer is not None:
        with timer.stage("project", label=label, n_splats=len(scene), window=list(win)) as rec:
            s2, stats = project_scene(scene, robot, win, z_floor=z_f, tau=TAU)
            rec["sizes"]["n_supports"] = stats["kept"]
    else:
        s2, stats = project_scene(scene, robot, win, z_floor=z_f, tau=TAU)
    occ = _support_raster(s2, win, C)
    dist = edt_clearance(occ, C)
    return {"occ": occ, "dist": dist, "stats": stats, "n_supports": int(stats["kept"])}


class Sections:
    """rho-AABB z-extents of the opaque splats, precomputed once so criterion 1 costs no scene copy."""

    def __init__(self, scene, z_f):
        opq = scene.opacity > TAU
        self.xy = scene.means[opq][:, :2]
        mz = scene.means[opq][:, 2]
        h = RHO * np.sqrt(scene.covs[opq][:, 2, 2])
        self.zb = mz - h - z_f
        self.zt = mz + h - z_f

    def __call__(self, p0, p1, half=0.2):
        p0, p1 = np.asarray(p0, float)[:2], np.asarray(p1, float)[:2]
        d = p1 - p0
        L = float(np.hypot(*d))
        u = d / L
        rel = self.xy - p0
        s = rel @ u
        off = np.abs(rel @ np.array([-u[1], u[0]]))
        sel = (s >= 0) & (s <= L) & (off <= half)
        return s[sel], self.zb[sel], self.zt[sel], L

    def selfcheck(self, scene, z_f, p0, p1):
        """Pin against showcase_scene._section, which does the same thing the slow way."""
        a = self(p0, p1)
        b = _section(scene, z_f, p0, p1, half=0.2)
        return {"n_mine": int(len(a[0])), "n_section": int(len(b[0])),
                "agree": bool(all(np.allclose(np.sort(x), np.sort(y)) for x, y in zip(a[:3], b[:3])))}


# ------------------------------------------------------------------------------------ candidate search
def endpoints(maps, win, robots):
    """Lattice points that pass D1's clearance clause for all three robots, at once."""
    xs = np.arange(win[0] + EDGE, win[2] - EDGE + 1e-9, LATTICE)
    ys = np.arange(win[1] + EDGE, win[3] - EDGE + 1e-9, LATTICE)
    pts, cells = [], []
    shape = maps["sweeper"]["dist"].shape
    for x in xs:
        for y in ys:
            c = cell_of((x, y), win, shape, C)
            if all(maps[k]["dist"][c] >= robots[k].max_radius() + 0.05 for k in ROBOTS):
                pts.append((float(x), float(y)))
                cells.append(c)
    return pts, cells


def pair_rows(maps, win, robots, pts, cells):
    """Pairs that are connected for all three robots, free on the line for the sweeper and blocked for
    the cylinder. Ranked, at this stage, by how much of the straight line the cylinder cannot use."""
    labs = {k: free_components(maps[k]["dist"], robots[k].max_radius())[0] for k in ROBOTS}
    rows = []
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            st, gl, cs, cg = pts[i], pts[j], cells[i], cells[j]
            D = float(np.hypot(gl[0] - st[0], gl[1] - st[1]))
            if not (L_MIN <= D <= L_MAX):
                continue
            if not all(same_component(labs[k], cs, cg) for k in ROBOTS):
                continue
            sw = line_clearance(maps["sweeper"]["dist"], win, C, st, gl, r=robots["sweeper"].max_radius())
            if not sw["free"]:
                continue
            cy = line_clearance(maps["cylinder"]["dist"], win, C, st, gl, r=robots["cylinder"].max_radius())
            if cy["free"]:
                continue                       # no contrast: the cylinder could drive the same line
            rows.append({"start": list(st), "goal": list(gl), "dist_m": D,
                         "line_sweeper": sw, "line_cylinder": cy,
                         "cyl_blocked_m": cy["blocked_len_m"]})
    rows.sort(key=lambda r: -r["cyl_blocked_m"])
    return rows


def geodesics(maps, win, robots, rows, keys=("sweeper", "cylinder"), chunk=40):
    """One graph per robot, one Dijkstra per distinct start: the route each robot would actually take.

    The graph does not depend on the source, so the whole region is paid for once per robot and the
    sources go through in chunks (a 400-source predecessor matrix over a 9 x 10 m region is 160 MB).
    Only the two ground robots are needed to rank contrast; the uav route is drawn later, per window.
    """
    for r in rows:
        for k in keys:
            r[f"path_{k}"] = None
    if not rows:
        return
    shape = maps["sweeper"]["dist"].shape
    starts = sorted({tuple(r["start"]) for r in rows})
    by_start = {}
    for r in rows:
        by_start.setdefault(tuple(r["start"]), []).append(r)
    for k in keys:
        G, idx, flat = free_graph(maps[k]["dist"], robots[k].max_radius(), C)
        live = [p for p in starts if idx[cell_of(p, win, shape, C)] >= 0]
        for a in range(0, len(live), chunk):
            grp = live[a:a + chunk]
            nodes = [int(idx[cell_of(p, win, shape, C)]) for p in grp]
            d, pred = dijkstra(G, directed=False, indices=nodes, return_predecessors=True)
            for row, (p, src) in enumerate(zip(grp, nodes)):
                for r in by_start[p]:
                    g = int(idx[cell_of(r["goal"], win, shape, C)])
                    if g < 0 or not np.isfinite(d[row][g]):
                        continue
                    ij = path_from_predecessors(pred[row], flat, shape, src, g)
                    if ij is None:
                        continue
                    r[f"path_{k}"] = {"len_m": float(d[row][g]),
                                      "xy": path_polyline(ij, win, C).tolist()}


def score(r):
    """Contrast on the routes the robots would take, plus the straight-line story."""
    ps, pc = r.get("path_sweeper"), r.get("path_cylinder")
    if not ps or not pc:
        r["contrast"] = 0.0
        r["separation_m"] = None
        return r
    sep = path_separation(np.array(ps["xy"]), np.array(pc["xy"]))
    r["separation_m"] = sep
    r["detour_ratio"] = pc["len_m"] / max(ps["len_m"], 1e-9)
    r["contrast"] = contrast(ps["len_m"], pc["len_m"], sep)
    return r


def window_for(st, gl, margin, region):
    w = [min(st[0], gl[0]) - margin, min(st[1], gl[1]) - margin,
         max(st[0], gl[0]) + margin, max(st[1], gl[1]) + margin]
    w = [max(w[0], region[0]), max(w[1], region[1]), min(w[2], region[2]), min(w[3], region[3])]
    w = [round(v, 3) for v in w]
    if w[2] - w[0] < MIN_SIDE or w[3] - w[1] < MIN_SIDE:
        return None
    if w[2] - w[0] > MAX_SIDE or w[3] - w[1] > MAX_SIDE:
        return None
    return w


def crop(arr, window, region, shape):
    i0, j0 = cell_of((window[0], window[1]), region, shape, C)
    i1, j1 = cell_of((window[2], window[3]), region, shape, C)
    return arr[i0:i1 + 1, j0:j1 + 1]


def evaluate(maps, win, robots, st, gl, z_c, sec, ceil_h, exact_label=None):
    """The full per-window verdict: D1 per robot, criterion 1, the routes and the contrast."""
    out = {"window": list(win), "start": list(st), "goal": list(gl), "z_c": z_c,
           "window_side_m": [round(win[2] - win[0], 3), round(win[3] - win[1], 3)],
           "window_at_most_8x8": bool(win[2] - win[0] <= MAX_SIDE and win[3] - win[1] <= MAX_SIDE),
           "not_through_glass_adjacent_opening": glass_clear(win),
           "robots": {}, "n_supports": {}, "supports_per_m2": {}}
    area = (win[2] - win[0]) * (win[3] - win[1])
    paths = {}
    for k in ROBOTS:
        r = robots[k].max_radius()
        dist = maps[k]["dist"]
        pre = d1_precheck(dist, win, C, st, gl, r)
        pre["line"] = line_clearance(dist, win, C, st, gl, r=r)
        pre["disc_free_frac"] = float((dist > r).mean())
        pre["occupied_frac"] = float((maps[k]["occ"]).mean())
        lab, ncomp = free_components(dist, r)
        pre["free_components"] = ncomp
        cs = cell_of(st, win, dist.shape, C)
        cg = cell_of(gl, win, dist.shape, C)
        g = geodesic_path(dist, r, cs, cg, C) if pre["connected"] else None
        if g is not None:
            paths[k] = path_polyline(g[0], win, C)
            pre["path_len_m"] = float(g[1])
            pre["path_xy"] = paths[k].tolist()
        out["robots"][k] = pre
        if "n_supports" in maps[k]:
            out["n_supports"][k] = maps[k]["n_supports"]
            out["supports_per_m2"][k] = round(maps[k]["n_supports"] / area, 1)
    s, zb, zt, L = sec(st, gl)
    out["criterion1"] = overhang_on_line(s, zb, zt, L, z_c=z_c, ceiling_height_m=ceil_h)
    if "sweeper" in paths and "cylinder" in paths:
        sep = path_separation(paths["sweeper"], paths["cylinder"])
        out["separation_m"] = sep
        out["detour_ratio"] = out["robots"]["cylinder"]["path_len_m"] / max(
            out["robots"]["sweeper"]["path_len_m"], 1e-9)
        out["contrast"] = contrast(out["robots"]["sweeper"]["path_len_m"],
                                   out["robots"]["cylinder"]["path_len_m"], sep)
    else:
        out["separation_m"] = out["contrast"] = None
    crit = {"window_at_most_8x8": out["window_at_most_8x8"],
            "not_through_glass_adjacent_opening": out["not_through_glass_adjacent_opening"],
            "straight_line_under_overhang": out["criterion1"]["straight_line_under_overhang"],
            "zc_above_overhang_top_by_0.25": out["criterion1"]["zc_above_overhang_top_by_0.25"],
            "zc_band_below_ceiling_side_by_0.30": out["criterion1"]["zc_band_below_ceiling_side_by_0.30"]}
    for k in ROBOTS:
        crit[f"d1_start_clear_{k}"] = out["robots"][k]["start_clear_ok"]
        crit[f"d1_goal_clear_{k}"] = out["robots"][k]["goal_clear_ok"]
        crit[f"connected_{k}"] = out["robots"][k]["connected"]
    crit["sweeper_line_free"] = out["robots"]["sweeper"]["line"]["free"]
    crit["cylinder_line_blocked"] = not out["robots"]["cylinder"]["line"]["free"]
    out["criteria"] = crit
    out["failed_criteria"] = sorted(k for k, v in crit.items() if not v)
    out["pass"] = not out["failed_criteria"]
    if exact_label:
        out["label"] = exact_label
    return out


# ------------------------------------------------------------------------------------ figures
def panel(ax, maps, win, robots, key, st, gl, ev):
    r = robots[key].max_radius()
    occ, dist = maps[key]["occ"], maps[key]["dist"]
    lab, _ = free_components(dist, r)
    img = np.ones(occ.shape[::-1] + (3,))
    img[(dist > r).T] = (0.75, 0.95, 0.75)
    img[occ.T] = (0.80, 0.10, 0.10)
    ext = [win[0], win[2], win[1], win[3]]
    ax.imshow(img, origin="lower", extent=ext, interpolation="nearest")
    cs = cell_of(st, win, dist.shape, C)
    comp = (lab == lab[cs]) if lab[cs] else np.zeros_like(occ)
    ax.imshow(np.ma.masked_where(~comp.T, comp.T), origin="lower", extent=ext, cmap="winter",
              alpha=0.35, interpolation="nearest")
    ax.plot([st[0], gl[0]], [st[1], gl[1]], "k--", lw=1.2, label="straight start->goal")
    p = ev["robots"][key].get("path_xy")
    if p:
        p = np.array(p)
        ax.plot(p[:, 0], p[:, 1], "b-", lw=2.2, label=f"free-space route {ev['robots'][key]['path_len_m']:.2f} m")
    ax.plot(*st[:2], "go", ms=9, label="start")
    ax.plot(*gl[:2], "bs", ms=9, label="goal")
    pre = ev["robots"][key]
    bad = [c for c in ev["failed_criteria"] if c.endswith(key)]
    ax.set_title(f"{key}  r={r:.3f}  band {robots[key].z_lo:.2f}-{robots[key].z_hi:.2f} m  "
                 f"supports={ev['n_supports'].get(key, '-')}\n"
                 f"clear start/goal {pre['start_clear_m']:.3f}/{pre['goal_clear_m']:.3f} m "
                 f"(D1 needs {pre['clearance_required_m']:.3f}); connected={pre['connected']}; "
                 f"components={pre['free_components']}\n"
                 f"straight line: min clearance {pre['line']['min_clearance_m']:.3f} m, "
                 f"{pre['line']['blocked_len_m']:.2f} m unusable"
                 + (f"   FAILS: {', '.join(bad)}" if bad else ""), fontsize=8)
    ax.set_xticks(np.arange(np.ceil(win[0] * 2) / 2, win[2], 0.5))
    ax.set_yticks(np.arange(np.ceil(win[1] * 2) / 2, win[3], 0.5))
    ax.tick_params(labelsize=6)
    ax.grid(alpha=0.3, lw=0.4)
    ax.legend(fontsize=6, loc="upper right")


def figure(maps, win, robots, st, gl, ev, path, title):
    fig, axs = plt.subplots(1, 3, figsize=(24, 9))
    for ax, key in zip(axs, ROBOTS):
        panel(ax, maps, win, robots, key, st, gl, ev)
    fig.suptitle(title + "\n" + CLAIMS, fontsize=9)
    fig.tight_layout(rect=(0, 0.01, 1, 0.93))
    FIGS.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=80)
    plt.close(fig)
    log("figure", path)


# ------------------------------------------------------------------------------------ steps
def step_search(args):
    timer = StageTimer()
    with timer.stage("scene_load", scene="planefloor") as rec:
        scene, meta = load_plane_scene(SCENE_NPZ)
        rec["sizes"]["n_splats"] = len(scene)
    z_f = meta["z_floor"]
    ceil_h = meta["ceiling_height_m"]
    robots = robot_table(Z_C)
    log("scene", len(scene), "z_floor", z_f, "ceiling", ceil_h)
    sec = Sections(scene, z_f)
    check = sec.selfcheck(scene, z_f, (2.32, 7.66), (1.72, 13.15))
    log("section selfcheck vs showcase_scene._section:", json.dumps(check))
    assert check["agree"], "precomputed section disagrees with showcase_scene._section"

    res = {"claims_boundary": CLAIMS, "task": "P2a/P2b: shared three-robot case on the plane floor",
           "scene_npz": str(SCENE_NPZ), "n_splats": len(scene), "z_c": Z_C, "raster_cell": C,
           "criterion1": "unchanged (showcase_scene.step_case, extracted into gmc.height.casesearch)",
           "criterion3": "D1, user-approved: r + 0.05 m on the robot's own CERTIFIED map, plus start "
                         "and goal in one component of dist > r. Rationale: crit1 & crit3 intersect in "
                         "0.9 m2 as built and 1.2 m2 with every phantom cell deleted.",
           "section_selfcheck": check, "regions": {}, "candidates": [], "finalists": []}
    OUT.mkdir(parents=True, exist_ok=True)

    def save():
        (OUT / "p2a_search.json").write_text(json.dumps(clean(res), indent=1))

    cands = []
    for name, box, note in REGIONS:
        if args.region and name not in args.region.split(","):
            continue
        t = time.time()
        area = (box[2] - box[0]) * (box[3] - box[1])
        maps = {k: certified(scene, robots[k], box, z_f, timer, f"{name}:{k}") for k in ROBOTS}
        rsum = {"box": box, "note": note, "area_m2": round(area, 2),
                "n_supports": {k: maps[k]["n_supports"] for k in ROBOTS},
                "supports_per_m2": {k: round(maps[k]["n_supports"] / area, 1) for k in ROBOTS},
                "disc_free_frac": {k: float((maps[k]["dist"] > robots[k].max_radius()).mean())
                                   for k in ROBOTS},
                "free_components": {k: free_components(maps[k]["dist"], robots[k].max_radius())[1]
                                    for k in ROBOTS}}
        log(name, json.dumps(clean(rsum)))
        pts, cells = endpoints(maps, box, robots)
        rsum["lattice_points_passing_D1_for_all_three"] = len(pts)
        rows = pair_rows(maps, box, robots, pts, cells)
        rsum["pairs_connected_and_contrasting"] = len(rows)
        rows = rows[:TOP_PAIRS]
        geodesics(maps, box, robots, rows)
        rows = [score(r) for r in rows]
        rows = [r for r in rows if r["contrast"] > 0]
        rows.sort(key=lambda r: -r["contrast"])
        rsum["pairs_with_two_routes"] = len(rows)
        log(name, "endpoints", len(pts), "pairs", rsum["pairs_connected_and_contrasting"],
            "with routes", len(rows), f"({time.time() - t:.0f}s)")
        if rows:
            log(name, "best pair", json.dumps(clean({k: v for k, v in rows[0].items()
                                                     if not k.startswith("path_")})))
        for r in rows[:SECTION_PAIRS]:
            s, zb, zt, L = sec(r["start"], r["goal"])
            r["criterion1"] = overhang_on_line(s, zb, zt, L, z_c=Z_C, ceiling_height_m=ceil_h)
        with_over = [r for r in rows[:SECTION_PAIRS] if r.get("criterion1", {}).get("ok")]
        rsum["pairs_passing_criterion1"] = len(with_over)
        log(name, "passing criterion 1:", len(with_over))
        res["regions"][name] = rsum
        save()

        # Windows around the best pairs, screened on the cropped region raster.
        seen = set()
        for r in (with_over or rows[:SECTION_PAIRS]):
            key = (tuple(r["start"]), tuple(r["goal"]))
            if key in seen:
                continue
            seen.add(key)
            for m in MARGINS:
                w = window_for(r["start"], r["goal"], m, box)
                if w is None:
                    continue
                cm = {k: {"occ": with_border(crop(maps[k]["occ"], w, box, maps[k]["occ"].shape))}
                      for k in ROBOTS}
                for k in ROBOTS:
                    cm[k]["dist"] = edt_clearance(cm[k]["occ"], C)
                ev = evaluate(cm, w, robots, r["start"], r["goal"], Z_C, sec, ceil_h)
                ev.update(region=name, margin=m, screened="cropped region raster (not a projection)",
                          overhang_note=note)
                cands.append(ev)
            if len(seen) >= 20:
                break
        save()

    cands.sort(key=lambda e: (-int(e["pass"]), -(e["contrast"] or 0.0)))
    res["candidates"] = [{k: v for k, v in c.items() if k != "robots"} | {
        "robots": {k: {kk: vv for kk, vv in c["robots"][k].items() if kk != "path_xy"}
                   for k in ROBOTS}} for c in cands[:WINDOW_CANDIDATES]]
    res["n_candidate_windows"] = len(cands)
    log("candidate windows", len(cands), "passing (screened)", sum(1 for c in cands if c["pass"]))
    save()

    # ---- exact pre-check: each finalist window gets its own projection, not a crop.
    picked, used = [], set()
    for c in cands:
        key = (tuple(c["start"]), tuple(c["goal"]))
        if key in used:
            continue
        used.add(key)
        picked.append(c)
        if len(picked) >= FINALISTS:
            break
    for i, c in enumerate(picked):
        w = c["window"]
        t = time.time()
        maps = {k: certified(scene, robots[k], w, z_f, timer, f"final{i}:{k}") for k in ROBOTS}
        ev = evaluate(maps, w, robots, c["start"], c["goal"], Z_C, sec, ceil_h,
                      exact_label=f"P2-{c['region']}-{i}")
        ev.update(region=c["region"], margin=c["margin"], screened="EXACT: own projection per robot",
                  screened_contrast=c["contrast"], overhang_note=c["overhang_note"],
                  seconds=round(time.time() - t, 1))
        log("finalist", i, ev["label"], "pass", ev["pass"], "contrast", ev["contrast"],
            "supports", json.dumps(ev["n_supports"]), "failed", ev["failed_criteria"])
        res["finalists"].append({k: v for k, v in ev.items() if k != "robots"} | {
            "robots": {k: {kk: vv for kk, vv in ev["robots"][k].items() if kk != "path_xy"}
                       for k in ROBOTS}})
        save()
        figure(maps, w, robots, c["start"], c["goal"], ev, FIGS / f"p2a_finalist{i}_{c['region']}.png",
               f"P2 finalist {i} ({c['region']}): {'PASSES' if ev['pass'] else 'FAILS ' + ','.join(ev['failed_criteria'])}"
               f" | contrast {ev['contrast'] if ev['contrast'] else 0:.2f}"
               f" | separation {ev['separation_m'] if ev['separation_m'] else 0:.2f} m"
               f" | one window, one start, one goal, three robots")
        if ev["pass"] and not any(f.get("chosen") for f in res["finalists"]):
            res["finalists"][-1]["chosen"] = True
            case = {"window": w, "start": [c["start"][0], c["start"][1], 0.0],
                    "goal": [c["goal"][0], c["goal"][1], 0.0], "z_floor": z_f, "z_c": Z_C,
                    "overhang": {"top": ev["criterion1"]["top"],
                                 "underside": ev["criterion1"]["underside"],
                                 "s_interval": ev["criterion1"]["s_interval"],
                                 "object": c["overhang_note"],
                                 "longest_run_m": ev["criterion1"]["longest_run_m"]},
                    "criteria": ev["criteria"], "pass": True,
                    "n_supports": ev["n_supports"], "supports_per_m2": ev["supports_per_m2"],
                    "precheck": {k: {kk: vv for kk, vv in ev["robots"][k].items() if kk != "path_xy"}
                                 for k in ROBOTS},
                    "contrast": ev["contrast"], "separation_m": ev["separation_m"],
                    "detour_ratio": ev.get("detour_ratio"),
                    "gravity_rotation": meta["gravity_rotation"],
                    "scene": "planefloor", "claims_boundary": CLAIMS,
                    "selection": {"search_file": str(OUT / "p2a_search.json"),
                                  "label": ev["label"], "region": c["region"],
                                  "method": "project_scene -> _support_raster(0.025) -> EDT; D1 "
                                            "pre-check per robot; criterion 1 unchanged"}}
            (OUT / "case_shared.json").write_text(json.dumps(clean(case), indent=2))
            log("WROTE", OUT / "case_shared.json")
        save()
    res["timing"] = timer.to_dict()
    res["seconds"] = round(time.time() - T0, 1)
    res["shared_case_exists"] = any(f["pass"] for f in res["finalists"])
    save()
    log("done. shared_case_exists =", res["shared_case_exists"])


def step_evidence(args):
    """P2d: per robot, the map, the free space dist > r, its components, and the criterion that fails.

    A shared case passed, so P2d is no longer the fallback deliverable. The version that earns its
    compute is the **window-shrink series** on the chosen start/goal pair: hold the case fixed, shrink
    the window, and watch where each robot's free space dies. That answers the question the passing
    figure cannot -- why the window has to be 4.3 x 5.6 m when the cylinder pays 156 k supports for it --
    and it hands P3 the smallest window this case survives in.

    The sweeper's straight route needs almost nothing; the cylinder's detour needs room to exist, and the
    window border is certified-occupied, so shrinking the window really does remove the route rather than
    merely cropping the picture.
    """
    timer = StageTimer()
    with timer.stage("scene_load", scene="planefloor") as rec:
        scene, meta = load_plane_scene(SCENE_NPZ)
        rec["sizes"]["n_splats"] = len(scene)
    z_f, ceil_h = meta["z_floor"], meta["ceiling_height_m"]
    robots = robot_table(Z_C)
    sec = Sections(scene, z_f)
    case = json.loads((OUT / "case_shared.json").read_text())
    st, gl = case["start"][:2], case["goal"][:2]
    region = [r[1] for r in REGIONS if r[0] == case["selection"]["region"]][0]
    out = {"claims_boundary": CLAIMS,
           "task": "P2d: per-robot evidence, as a window-shrink series on the shared case",
           "case": case["selection"], "start": st, "goal": gl,
           "chosen_window": case["window"], "windows": []}
    margins = [float(m) for m in args.margins.split(",")]
    for i, m in enumerate(margins):
        w = window_for(st, gl, m, region)
        if w is None:
            log("margin", m, "gives no admissible window (side outside [%.1f, %.1f] m)" % (MIN_SIDE, MAX_SIDE))
            out["windows"].append({"margin": m, "window": None,
                                   "why": f"window side outside [{MIN_SIDE}, {MAX_SIDE}] m"})
            continue
        maps = {k: certified(scene, robots[k], w, z_f, timer, f"m{m}:{k}") for k in ROBOTS}
        ev = evaluate(maps, w, robots, st, gl, Z_C, sec, ceil_h, exact_label=f"margin{m}")
        ev["margin"] = m
        ev["is_the_chosen_window"] = bool(np.allclose(w, case["window"]))
        log("margin", m, "window", w, "area", round((w[2] - w[0]) * (w[3] - w[1]), 2),
            "pass", ev["pass"], "supports", json.dumps(ev["n_supports"]),
            "cylinder route", ev["robots"]["cylinder"].get("path_len_m"),
            "failed", ev["failed_criteria"])
        figure(maps, w, robots, st, gl, ev,
               FIGS / f"p2d_shrink_{str(m).replace('.', 'p')}.png",
               f"P2d window-shrink series, margin {m} m -> {w[2] - w[0]:.2f} x {w[3] - w[1]:.2f} m "
               f"({'THE CHOSEN WINDOW' if ev['is_the_chosen_window'] else 'variant'}): "
               f"{'passes' if ev['pass'] else 'FAILS ' + ','.join(ev['failed_criteria'])} · "
               f"cylinder supports {ev['n_supports']['cylinder']:,} · same start, same goal throughout")
        out["windows"].append({k: v for k, v in ev.items() if k != "robots"} | {
            "robots": {k: {kk: vv for kk, vv in ev["robots"][k].items() if kk != "path_xy"}
                       for k in ROBOTS}})
        (OUT / "p2d_evidence.json").write_text(json.dumps(clean(out), indent=1))
    ok = [w for w in out["windows"] if w.get("pass")]
    out["smallest_window_the_case_survives"] = min(
        (w for w in ok), key=lambda w: (w["window"][2] - w["window"][0]) * (w["window"][3] - w["window"][1]),
        default=None)
    out["timing"] = timer.to_dict()
    (OUT / "p2d_evidence.json").write_text(json.dumps(clean(out), indent=1))
    log("smallest surviving window:",
        json.dumps(clean((out["smallest_window_the_case_survives"] or {}).get("window"))))


def step_selftest(args):
    """The whole pipeline on gmc.height.synth3d's table scene, which has the morphology P2 hunts for.

    The open variant is a wall at x = 0 with two gaps: the south gap is filled by a table (top 0.74 m,
    four thin legs) and the north gap is empty. So from one start to one goal the sweeper goes under the
    table, the uav flies over it, and the cylinder has to go round through the north gap — exactly the
    picture the user described, on a scene small enough to run inside the agent's own allocation. If
    this fails, the bug is in the search, not in the hall.
    """
    from gmc.height.synth3d import GOAL, START, WORKSPACE, Z_FLOOR, table_scene
    scene, info = table_scene("open")
    win = list(WORKSPACE)
    st, gl = tuple(START[:2]), tuple(GOAL[:2])
    robots = robot_table(Z_C)
    sec = Sections(scene, Z_FLOOR)
    check = sec.selfcheck(scene, Z_FLOOR, st, gl)
    assert check["agree"], check
    maps = {k: certified(scene, robots[k], win, Z_FLOOR, None) for k in ROBOTS}
    ev = evaluate(maps, win, robots, st, gl, Z_C, sec, 2.5, exact_label="selftest")
    ev["region"] = "synth3d table_open"
    log("selftest", json.dumps(clean({k: v for k, v in ev.items() if k != "robots"})))
    for k in ROBOTS:
        log("selftest", k, json.dumps(clean({kk: vv for kk, vv in ev["robots"][k].items()
                                             if kk != "path_xy"})))
    figure(maps, win, robots, st, gl, ev, FIGS / "p2a_selftest_synth.png",
           "P2 search self-test on synth3d table_open (NOT the showcase scene): "
           "sweeper under the table, uav over it, cylinder round the north gap")
    fail = []
    if not ev["criteria"]["sweeper_line_free"]:
        fail.append("sweeper cannot drive the straight line under the table")
    if ev["criteria"]["cylinder_line_blocked"] is not True:
        fail.append("cylinder is not blocked on the straight line")
    for k in ROBOTS:
        if not ev["robots"][k]["connected"]:
            fail.append(f"{k} start and goal are not connected")
        if not (ev["robots"][k]["start_clear_ok"] and ev["robots"][k]["goal_clear_ok"]):
            fail.append(f"{k} fails D1 clearance")
    if not ev["criterion1"]["ok"]:
        fail.append(f"criterion 1 fails: {ev['criterion1']}")
    if not (ev["separation_m"] and ev["separation_m"] > 0.5):
        fail.append(f"routes do not separate: {ev['separation_m']}")
    if not ev["pass"]:
        fail.append(f"case does not pass: {ev['failed_criteria']}")

    # and the search stages themselves, on the same scene
    pts, cells = endpoints(maps, win, robots)
    rows = pair_rows(maps, win, robots, pts, cells)
    geodesics(maps, win, robots, rows)
    rows = [r for r in (score(r) for r in rows) if r["contrast"] > 0]
    rows.sort(key=lambda r: -r["contrast"])
    log("selftest endpoints", len(pts), "contrasting pairs", len(rows))
    if not rows:
        fail.append("the pair search found no contrasting pair on a scene built to have one")
    else:
        log("selftest best pair", json.dumps(clean({k: v for k, v in rows[0].items()
                                                    if not k.startswith("path_")})))
    out = {"claims_boundary": "synthetic scene, not the showcase hall",
           "scene": "gmc.height.synth3d.table_scene('open')", "case": clean(
               {k: v for k, v in ev.items() if k != "robots"}),
           "robots": clean({k: {kk: vv for kk, vv in ev["robots"][k].items() if kk != "path_xy"}
                            for k in ROBOTS}),
           "n_endpoints": len(pts), "n_contrasting_pairs": len(rows), "failures": fail}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "p2a_selftest.json").write_text(json.dumps(clean(out), indent=1))
    if fail:
        raise SystemExit("SELFTEST FAILED: " + "; ".join(fail))
    log("SELFTEST PASSED")


def step_compare(args):
    """P2c: the three certified routes over the three maps, on the one window they all ran on.

    Reads the three showcase_run.py results and refuses to draw anything until it has checked that they
    really did share a window, a start and a goal -- that check is the whole claim, so it is an assert,
    not a caption.
    """
    sdir = Path(args.run_dir)
    runs = {k: json.loads((sdir / f"{k}.json").read_text()) for k in ROBOTS}
    case = json.loads((sdir / "case.json").read_text())
    shared, mismatch = {}, {}
    for field in ("window", "start", "goal", "z_floor", "z_c"):
        vals = {k: runs[k]["case"][field] for k in ROBOTS}
        vals["case.json"] = case[field]
        if not all(np.allclose(v, vals["case.json"]) for v in vals.values()):
            mismatch[field] = vals
        shared[field] = case[field]
    if mismatch:
        raise SystemExit("P2c: the three runs did NOT share the case: " + json.dumps(clean(mismatch)))
    log("shared case confirmed identical across all three runs:", json.dumps(clean(shared)))

    win = [float(v) for v in case["window"]]
    z_f = float(case["z_floor"])
    robots = robot_table(case["z_c"])
    timer = StageTimer()
    with timer.stage("scene_load", scene="planefloor") as rec:
        scene, meta = load_plane_scene(SCENE_NPZ)
        rec["sizes"]["n_splats"] = len(scene)
    sec = Sections(scene, z_f)
    maps = {k: certified(scene, robots[k], win, z_f, timer, k) for k in ROBOTS}
    ev = evaluate(maps, win, robots, case["start"][:2], case["goal"][:2], case["z_c"], sec,
                  meta["ceiling_height_m"], exact_label=case["selection"]["label"])

    curves, summary = {}, {}
    for k in ROBOTS:
        res = runs[k].get("result") or {}
        rep = runs[k].get("replay3d") or {}
        ver = res.get("verify") or {}
        summary[k] = {"status": res.get("status"), "certified": bool(ver.get("certified")),
                      "min_clearance": ver.get("min_clearance"), "replay3d_passed": bool(rep.get("passed")),
                      "replay3d_min_clearance_lb": rep.get("min_clearance_lb"),
                      "n_supports": (runs[k].get("projection") or {}).get("kept"),
                      "compile_seconds": res.get("compile_seconds"), "query_seconds": res.get("query_seconds"),
                      "probe": runs[k].get("probe"), "timing": (runs[k].get("timing") or {}).get("by_stage")}
        if res.get("curve") is not None:
            xy = sample_curve(curve_from_dict(res["curve"]), 0.02, robots[k].max_radius())[:, :2]
            curves[k] = np.asarray(xy)
            summary[k]["certified_route_len_m"] = float(
                np.hypot(*np.diff(curves[k], axis=0).T).sum())
        log(k, json.dumps(clean(summary[k])))
    if "sweeper" in curves and "cylinder" in curves:
        summary["certified_route_separation_m"] = path_separation(curves["sweeper"], curves["cylinder"])
        summary["certified_detour_ratio"] = (summary["cylinder"]["certified_route_len_m"]
                                             / summary["sweeper"]["certified_route_len_m"])

    fig, axs = plt.subplots(1, 4, figsize=(32, 9))
    for ax, key in zip(axs, ROBOTS):
        panel(ax, maps, win, robots, key, case["start"][:2], case["goal"][:2], ev)
        if key in curves:
            ax.plot(curves[key][:, 0], curves[key][:, 1], "-", color="darkorange", lw=3.0,
                    label=f"GMC certified route {summary[key]['certified_route_len_m']:.2f} m")
            ax.legend(fontsize=6, loc="upper right")
        s = summary[key]
        ax.set_xlabel(f"{s['status']} · verify.certified={s['certified']} · replay3d={s['replay3d_passed']}"
                      f" · clearance lb {s['replay3d_min_clearance_lb']}", fontsize=7)
    ax = axs[3]
    ax.imshow(np.ones(maps["sweeper"]["occ"].shape[::-1] + (3,)), origin="lower",
              extent=[win[0], win[2], win[1], win[3]], interpolation="nearest")
    ax.imshow(np.ma.masked_where(~maps["cylinder"]["occ"].T, maps["cylinder"]["occ"].T), origin="lower",
              extent=[win[0], win[2], win[1], win[3]], cmap="Greys", alpha=0.35, interpolation="nearest")
    ax.imshow(np.ma.masked_where(~maps["sweeper"]["occ"].T, maps["sweeper"]["occ"].T), origin="lower",
              extent=[win[0], win[2], win[1], win[3]], cmap="autumn_r", alpha=0.9, interpolation="nearest")
    for key, colr in (("sweeper", "tab:green"), ("cylinder", "tab:red"), ("uav", "tab:blue")):
        if key in curves:
            ax.plot(curves[key][:, 0], curves[key][:, 1], "-", color=colr, lw=3.0,
                    label=f"{key} {summary[key]['certified_route_len_m']:.2f} m")
    ax.plot([case["start"][0], case["goal"][0]], [case["start"][1], case["goal"][1]], "k--", lw=1.2)
    ax.plot(case["start"][0], case["start"][1], "go", ms=10)
    ax.plot(case["goal"][0], case["goal"][1], "bs", ms=10)
    ax.set_title("the three certified routes, one window, one start, one goal\n"
                 "grey = what the cylinder sees, red = what the sweeper sees", fontsize=9)
    ax.set_xticks(np.arange(np.ceil(win[0] * 2) / 2, win[2], 0.5))
    ax.set_yticks(np.arange(np.ceil(win[1] * 2) / 2, win[3], 0.5))
    ax.tick_params(labelsize=6)
    ax.grid(alpha=0.3, lw=0.4)
    ax.legend(fontsize=8, loc="upper right")
    sep = summary.get("certified_route_separation_m")
    fig.suptitle(f"Amendment 3 P2c: the shared three-robot case {case['selection']['label']} · "
                 f"start {tuple(round(v, 2) for v in case['start'][:2])} goal "
                 f"{tuple(round(v, 2) for v in case['goal'][:2])} · window {win} · "
                 f"certified routes separate by {sep:.2f} m\n" + CLAIMS, fontsize=9)
    fig.tight_layout(rect=(0, 0.01, 1, 0.93))
    FIGS.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGS / "p2c_three_routes.png", dpi=80)
    plt.close(fig)
    log("figure", FIGS / "p2c_three_routes.png")

    out = {"claims_boundary": CLAIMS, "task": "P2c: three robots, one window, one start, one goal",
           "case": case["selection"], "window": win, "start": case["start"], "goal": case["goal"],
           "shared_case_confirmed_identical_in_all_three_runs": True,
           "runs": summary, "precheck_recomputed": {k: v for k, v in ev.items() if k != "robots"},
           "figure": str(FIGS / "p2c_three_routes.png"), "timing": timer.to_dict()}
    (OUT / "p2c_shared_case.json").write_text(json.dumps(clean(out), indent=1))
    log("wrote", OUT / "p2c_shared_case.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", required=True, choices=["search", "evidence", "selftest", "compare"])
    ap.add_argument("--run-dir", default="results/height/plane/shared")
    ap.add_argument("--region", default="", help="comma-separated region names, default all")
    ap.add_argument("--n", type=int, default=3, help="unused; kept for older invocations")
    ap.add_argument("--margins", default="0.3,0.45,0.6,0.9,1.3,1.8",
                    help="window margins (m) for the --step evidence shrink series")
    args = ap.parse_args()
    {"search": step_search, "evidence": step_evidence, "selftest": step_selftest,
     "compare": step_compare}[args.step](args)


if __name__ == "__main__":
    main()
