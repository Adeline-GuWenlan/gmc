"""Test A on the real K2 3DGS scene: can the robot get through ONE door?

This is a plumbing and cross-check experiment, not a claim.  It asks whether
the certified compiler runs at all on real 3DGS-derived geometry, and whether
its verdicts agree with an independent implementation on the same data.

Why a cross-check is possible at all
------------------------------------
Real scenes have no analytic ground truth, so "the answer is right" is not
checkable here the way ``synth.gate_half_angle`` makes it checkable on the
toy families.  What *is* available:

1.  ``splatc_atlas/results/gmc_h2/k2_windows.json`` (the atlas's v3 run)
    already measured, with a completely separate implementation (shapely
    polygon unions at NPOLY=24, no support-function oracle, no sandwich),
    whether each door is open at each of 720 orientations, for three robot
    lengths.  Those per-theta flags are a falsifiable prediction for this
    compiler, in both directions: some orientations are predicted open and
    some closed.

    Use v3, not ``k2_windows_v2.json``.  v2 probed with a single POINT at a
    fixed offset, which the configuration-space wall swallows as the robot
    grows, reporting an open corridor as closed.  The two disagree at up to
    246 of 720 orientations, and v2 calls door_A/L32 never open where v3
    finds it open 26.5% of the time.  This experiment uses v3's segment
    probes so that agreement cannot come from reproducing v2's artifact; it
    records the v2 verdict too, purely to show which of the two it lands on.
2.  The dose-response is real: at door_A the atlas v3 run finds the gate open
    at 100% of orientations for a 1.6 u robot, 64% for 2.4 u and 27% for
    3.2 u.  A "door" that never closes for any robot would be evidence of a
    scan hole; these close, so the opening behaves like an opening.
3.  A returned path is re-verified by the independent continuous checker
    against every original support, which is a different code path from the
    one that produced it.
4.  A point buried in obstacle mass must not locate in free space.  If it
    does, the pipeline is broken and every other number here is void.

What this experiment still cannot tell you: whether the opening is a real
doorway or glass/a scan hole in the reconstruction.  ``meta.json`` lists that
audit as not done.  See ``k2_scene`` for the full carried caveat.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import shapely
from shapely.geometry import LineString
from shapely.ops import unary_union

from gmc.budget import WorkLedger
from gmc.config import load_config
from gmc.io.gs_io import validate_models
from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import compile_mobility
from gmc.mobility.query import query
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import query_candidate_pairs
from gmc.spatial.slice_compiler import build_slice
from gmc.types import Pose2
from gmc.verification.path import verify_curve

from k2_scene import DOORS, UNITS_PER_M, load_window, wall_frame

_ATLAS_RES = (Path(__file__).resolve().parents[2] / "splatc_atlas" / "results"
              / "gmc_h2")
# k2_windows.json is the atlas's v3 result and the one to cross-check against.
# v3 replaced v2's POINT probes with radial SEGMENTS because a point probe gets
# swallowed by the fattened wall at long robot lengths, misreporting an open
# corridor as closed; v2 and v3 disagree at up to 246 of 720 orientations and
# v2 reports L32 as never open where v3 finds it open a third of the time.
# v2 is loaded only to report that disagreement, never as the reference.
ATLAS_H2 = _ATLAS_RES / "k2_windows.json"
ATLAS_H2_SUPERSEDED = _ATLAS_RES / "k2_windows_v2.json"
PROBE_NEAR, PROBE_FAR = 1.0, 2.4      # atlas v3 probe_near_far

# Atlas robot sweep: sqrt-eigenvalues (length, width); semi-axes are RHO x these
# with RHO = 2.0, so L24 is a 2.4 u x 0.6 u support -- 1.2 m x 0.3 m at the
# unverified 0.5 m/u scale.  The sweep varies LENGTH at fixed width, which is
# the axis the atlas dose-response is measured on.
ATLAS_ROBOTS = {"L16": (0.8, 0.3), "L24": (1.2, 0.3), "L32": (1.6, 0.3)}


def atlas_run(window: str, robot: str, path: Path = ATLAS_H2):
    if not path.exists():
        return None
    data = json.loads(path.read_text())
    for r in data["runs"]:
        if r["window"] == window and r["robot"] == robot:
            return r
    return None


def atlas_open_at(run, theta: float):
    """The atlas verdict at the sampled orientation nearest ``theta``."""
    flags = run.get("open_flags")
    if not flags:
        return None, None
    n = len(flags)
    thetas = np.linspace(0.0, math.pi, n, endpoint=False)
    k = int(np.argmin(np.abs(thetas - (theta % math.pi))))
    return bool(flags[k]), float(thetas[k])


def probe_segments(center, normal, near: float = PROBE_NEAR,
                   far: float = PROBE_FAR):
    """Radial segments on either side of the wall, as in the atlas v3 gate.

    A single point at a fixed offset is the wrong instrument: as the robot
    grows the configuration-space wall grows with it and swallows the point,
    so the gate reads closed while the corridor is visibly open.  A segment
    asks the question that was meant -- is there free space on this side at
    all -- rather than the question a point asks, which is whether one
    arbitrary spot survived.
    """
    c = np.asarray(center, dtype=float)
    n = np.asarray(normal, dtype=float)
    return (LineString([c - near * n, c - far * n]),
            LineString([c + near * n, c + far * n]))


def gate_open(scene, robot, cfg, theta, seg_a, seg_b):
    """Open iff ONE certified-safe component reaches both probe segments.

    Also returns representative points inside that component on each segment,
    so a subsequent query starts and ends somewhere provably free rather than
    at a hardcoded offset.
    """
    t0 = time.time()
    sl = build_slice(scene, robot, float(theta), cfg)
    hit, pts = None, None
    for comp in sl.D_safe:
        ia = comp.geometry.intersection(seg_a)
        ib = comp.geometry.intersection(seg_b)
        if not ia.is_empty and not ib.is_empty:
            hit = comp
            pts = (np.asarray(ia.interpolate(0.5, normalized=True).coords[0]),
                   np.asarray(ib.interpolate(0.5, normalized=True).coords[0]))
            break
    reach_a = any(not c.geometry.intersection(seg_a).is_empty
                  for c in sl.D_safe)
    reach_b = any(not c.geometry.intersection(seg_b).is_empty
                  for c in sl.D_safe)
    return {
        "theta": float(theta),
        "side_a_has_free_space": bool(reach_a),
        "side_b_has_free_space": bool(reach_b),
        "open": hit is not None,
        "n_safe_components": len(sl.D_safe),
        "slice_status": sl.status.name,
        "support_calls": int(sl.support_calls),
        "seconds": time.time() - t0,
        "start": None if pts is None else list(map(float, pts[0])),
        "goal": None if pts is None else list(map(float, pts[1])),
    }, sl, pts


def deep_obstacle_point(sl, workspace, n_grid=48):
    """A workspace point buried in forbidden mass, for the negative control.

    Picked as the grid point covered by ``C_plus`` that is farthest from any
    certified-safe component, so a pipeline that mislabels it as free is not
    merely borderline.
    """
    xmin, ymin, xmax, ymax = workspace.bounds
    xs = np.linspace(xmin, xmax, n_grid)
    ys = np.linspace(ymin, ymax, n_grid)
    gx, gy = np.meshgrid(xs, ys)
    gx, gy = gx.ravel(), gy.ravel()
    inside = shapely.contains_xy(sl.C_plus, gx, gy)
    if not inside.any():
        return None, None
    free = unary_union([c.geometry for c in sl.D_safe]) if sl.D_safe else None
    cand = np.column_stack([gx[inside], gy[inside]])
    if free is None or free.is_empty:
        return cand[0], float("inf")
    pts = shapely.points(cand)
    d = shapely.distance(pts, free)
    k = int(np.argmax(d))
    return cand[k], float(d[k])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/toy.yaml")
    ap.add_argument("--door", default="door_A", choices=sorted(DOORS))
    ap.add_argument("--half", type=float, default=3.0,
                    help="workspace half-width in scene units; the atlas H2 "
                         "runs used 3.0, so the cross-check is only "
                         "apples-to-apples at 3.0")
    ap.add_argument("--probe-near", type=float, default=PROBE_NEAR)
    ap.add_argument("--probe-far", type=float, default=PROBE_FAR)
    ap.add_argument("--robots", nargs="*", default=["L16", "L24", "L32"])
    ap.add_argument("--with-compile", action="store_true",
                    help="run the full mobility compile + query + independent "
                         "verification (the expensive stage)")
    ap.add_argument("--compile-robot", default="L24")
    ap.add_argument("--figure", default=None)
    ap.add_argument("-o", "--output", required=True)
    args = ap.parse_args()

    cfg = load_config(args.config)
    center = DOORS[args.door]
    out = {"door": args.door, "center": list(center), "half": args.half,
           "config": args.config, "units_per_m": UNITS_PER_M,
           "probe_near_far": [args.probe_near, args.probe_far],
           "atlas_reference": str(ATLAS_H2),
           "caveat": ("plumbing + cross-check only; the wall-gap audit in "
                      "meta.json is not done, so an opening here may be "
                      "glass or a scan hole and no planning claim follows")}

    # --- Stage 0: build the real scene ------------------------------------
    t0 = time.time()
    scene, stats = load_window(center, args.half)
    stats["load_seconds"] = time.time() - t0
    out["scene"] = stats
    print(f"=== STAGE 0: scene ===\n"
          f"  window {args.door} {center} half={args.half} u\n"
          f"  in box (any weight)      {stats['n_in_box_any_weight']}\n"
          f"  after weight > {stats['w_min']}       {stats['n_after_weight_filter']}\n"
          f"  dropped (not SPD)        {stats['n_dropped_not_spd']}\n"
          f"  supports used            {stats['n_supports']}\n"
          f"  semi-axes p50            {stats['semi_minor_p50']:.4f} x "
          f"{stats['semi_major_p50']:.4f} u  (p99 major "
          f"{stats['semi_major_p99']:.4f})\n"
          f"  {stats['load_seconds']:.1f}s", flush=True)

    along, normal = wall_frame(scene, center)
    wall_deg = math.degrees(math.atan2(along[1], along[0]))
    seg_a, seg_b = probe_segments(center, normal, args.probe_near,
                                  args.probe_far)
    # A probe segment reaches PROBE_FAR from the door centre, which for a
    # small window sticks out past the workspace box.  The part outside is
    # simply absent from every free component, so the gate is then asking a
    # shorter question than the atlas asked, and the cross-check stops being
    # apples-to-apples.  Measure it rather than assume it.
    frac_a = float(seg_a.intersection(scene.workspace).length / seg_a.length)
    frac_b = float(seg_b.intersection(scene.workspace).length / seg_b.length)
    out["wall"] = {"along_deg": wall_deg,
                   "normal_deg": math.degrees(math.atan2(normal[1], normal[0])),
                   "segment_a": list(map(list, seg_a.coords)),
                   "segment_b": list(map(list, seg_b.coords)),
                   "segment_a_inside_workspace": frac_a,
                   "segment_b_inside_workspace": frac_b}
    if min(frac_a, frac_b) < 0.999:
        print(f"  WARNING: probe segments are clipped by the workspace "
              f"(A {100*frac_a:.0f}% inside, B {100*frac_b:.0f}% inside). "
              f"half must be at least "
              f"{args.probe_far * max(abs(normal[0]), abs(normal[1])):.2f} u "
              f"for the atlas comparison to be exact.", flush=True)
    print(f"  wall bearing {wall_deg:+.2f} deg, probe segments "
          f"{np.round(np.array(seg_a.coords), 3).tolist()} | "
          f"{np.round(np.array(seg_b.coords), 3).tolist()}", flush=True)

    # --- Stage 1+2: dose-response, cross-checked against the atlas ---------
    # For each robot the atlas gives a per-theta open/closed flag list.  We
    # evaluate this compiler at one orientation the atlas calls OPEN and one
    # it calls CLOSED (when both exist), so agreement is falsifiable in both
    # directions rather than only where the door happens to be open.
    print("\n=== STAGE 1-2: gate, dose-response vs atlas H2 (v3) ===",
          flush=True)
    rows = []
    best_for_compile = None
    for rname in args.robots:
        a, b = ATLAS_ROBOTS[rname]
        robot = ellipse_robot(a, b)
        validate_models(scene, robot, cfg)
        run = atlas_run(args.door, rname)
        old_run = atlas_run(args.door, rname, ATLAS_H2_SUPERSEDED)
        frac = run.get("gate_open_fraction") if run else None
        probes_theta = []
        if run and run.get("open_flags"):
            flags = np.asarray(run["open_flags"], dtype=bool)
            th = np.linspace(0.0, math.pi, flags.size, endpoint=False)
            if flags.any():
                probes_theta.append(("atlas_open",
                                     float(th[flags][flags.sum() // 2])))
            if (~flags).any():
                probes_theta.append(("atlas_closed",
                                     float(th[~flags][(~flags).sum() // 2])))
        if not probes_theta:
            probes_theta = [("normal_aligned",
                             float(math.atan2(normal[1], normal[0]) % math.pi))]
        for label, theta in probes_theta:
            res, sl, pts = gate_open(scene, robot, cfg, theta, seg_a, seg_b)
            a_open, a_theta = atlas_open_at(run, theta) if run else (None, None)
            o_open, _ = atlas_open_at(old_run, theta) if old_run else (None, None)
            res.update({"robot": rname, "robot_semi_axes": [a, b],
                        "support_extent_u": [2 * a, 2 * b],
                        "probe_kind": label,
                        "atlas_v3_open_fraction": frac,
                        "atlas_v3_open_at_theta": a_open,
                        "atlas_v2_open_at_theta": o_open,
                        "atlas_theta": a_theta,
                        "agrees_with_atlas_v3": (None if a_open is None
                                                 else bool(a_open == res["open"])),
                        "agrees_with_atlas_v2": (None if o_open is None
                                                 else bool(o_open == res["open"]))})
            rows.append(res)
            if (res["open"] and pts is not None
                    and rname == args.compile_robot
                    and best_for_compile is None):
                best_for_compile = (theta, pts, robot)
            print(f"  {rname} ({2*a:.1f}x{2*b:.1f} u) theta={theta:.4f} "
                  f"[{label}] gmc_open={res['open']} "
                  f"atlas_v3={a_open} atlas_v2={o_open} "
                  f"agree_v3={res['agrees_with_atlas_v3']} "
                  f"agree_v2={res['agrees_with_atlas_v2']} "
                  f"comps={res['n_safe_components']} "
                  f"free_a={res['side_a_has_free_space']} "
                  f"free_b={res['side_b_has_free_space']} "
                  f"calls={res['support_calls']} {res['seconds']:.1f}s",
                  flush=True)
    out["gate_rows"] = rows
    for tag in ("v3", "v2"):
        ok = [r[f"agrees_with_atlas_{tag}"] for r in rows
              if r[f"agrees_with_atlas_{tag}"] is not None]
        out[f"atlas_{tag}_agreement"] = {"checked": len(ok),
                                         "agree": int(sum(ok))}
        print(f"  agreement with atlas {tag}: {sum(ok)}/{len(ok)}", flush=True)

    # --- Stage 3: negative control ----------------------------------------
    print("\n=== STAGE 3: negative control ===", flush=True)
    a, b = ATLAS_ROBOTS[args.compile_robot]
    robot = ellipse_robot(a, b)
    if best_for_compile is not None:
        theta_nc = best_for_compile[0]
    else:
        theta_nc = float(math.atan2(normal[1], normal[0]) % math.pi)
    _, sl_nc, _ = gate_open(scene, robot, cfg, theta_nc, seg_a, seg_b)
    pt, depth = deep_obstacle_point(sl_nc, scene.workspace)
    if pt is None:
        nc = {"ran": False, "reason": "no forbidden mass in workspace"}
        print("  NO forbidden mass found in the window -- control cannot run",
              flush=True)
    else:
        located = sl_nc.locate(tuple(pt), "safe")
        nc = {"ran": True, "point": list(map(float, pt)),
              "distance_to_free_u": depth,
              "located_in_free_space": located is not None,
              "passed": located is None}
        print(f"  point {np.round(pt, 3).tolist()} is {depth:.3f} u inside "
              f"forbidden mass -> located_in_free={located is not None} "
              f"PASS={located is None}", flush=True)
    out["negative_control"] = nc

    # --- Stage 4: the full pipeline ---------------------------------------
    if args.with_compile:
        print(f"\n=== STAGE 4: full compile + query + independent verify "
              f"({args.compile_robot}) ===", flush=True)
        if best_for_compile is None:
            print("  no orientation with an open gate for "
                  f"{args.compile_robot}; skipping the query arm and "
                  "compiling only", flush=True)
            start_xy = np.asarray(seg_a.interpolate(0.5, normalized=True)
                                  .coords[0], dtype=float)
            goal_xy = np.asarray(seg_b.interpolate(0.5, normalized=True)
                                 .coords[0], dtype=float)
        else:
            theta_nc, (start_xy, goal_xy), robot = best_for_compile
        start = Pose2(np.asarray(start_xy, dtype=float), theta_nc)
        goal = Pose2(np.asarray(goal_xy, dtype=float), theta_nc)
        print(f"  start={np.round(start_xy, 4).tolist()} "
              f"goal={np.round(goal_xy, 4).tolist()} theta={theta_nc:.4f} "
              f"(both taken from the certified-safe component that reaches "
              f"both probe segments)", flush=True)
        ledger = WorkLedger()
        t0 = time.time()
        pq = query_candidate_pairs(scene, robot, scene.workspace)
        oracles = list(pq.oracles)
        for o in oracles:
            o.ledger = ledger
        dec = build_slabs(scene, robot, cfg, oracles, ledger=ledger)
        mc = compile_mobility(scene, robot, cfg, oracles, dec, ledger=ledger)
        compile_s = time.time() - t0
        t1 = time.time()
        result = query(start, goal, mc)
        query_s = time.time() - t1
        comp = {"robot": args.compile_robot, "theta": float(theta_nc),
                "start": list(map(float, start_xy)),
                "goal": list(map(float, goal_xy)),
                "n_supports": len(scene.supports),
                "n_pairs": len(oracles), "compile_seconds": compile_s,
                "query_seconds": query_s,
                "safe_nodes": mc.M_safe.number_of_nodes(),
                "safe_edges": mc.M_safe.number_of_edges(),
                "possible_nodes": mc.M_possible.number_of_nodes(),
                "possible_edges": mc.M_possible.number_of_edges(),
                "status": result.status.name,
                "clearance_lb": result.clearance_lower_bound}
        print(f"  n={comp['n_supports']} pairs={comp['n_pairs']} "
              f"compile={compile_s:.1f}s query={query_s:.1f}s "
              f"safe={comp['safe_nodes']}n/{comp['safe_edges']}e "
              f"status={comp['status']} clearance_lb={comp['clearance_lb']}",
              flush=True)
        verdict = {"ran": False}
        if result.curve is not None:
            rep = verify_curve(tuple(oracles), scene.workspace, result.curve,
                               cfg.query.eps_clear, cfg.orientation.theta_min,
                               expected_start=start, expected_goal=goal)
            verdict = {"ran": True, "certified": bool(rep.certified),
                       "min_clearance": float(rep.min_clearance),
                       "failed_segment": rep.failed_segment,
                       "reason": rep.reason,
                       "n_original_pairs": len(oracles)}
            print(f"  independent verify against all {len(oracles)} original "
                  f"pairs: certified={rep.certified} "
                  f"min_clearance={rep.min_clearance:.6f} "
                  f"reason={rep.reason}", flush=True)
        else:
            print("  query returned no curve; nothing to verify", flush=True)
        comp["independent_verification"] = verdict
        out["full_pipeline"] = comp

    # --- Stage 5: figure ---------------------------------------------------
    if args.figure:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 7))
        clipped = sl_nc.C_plus.intersection(scene.workspace)
        for geom in getattr(clipped, "geoms", [clipped]):
            if geom.is_empty:
                continue
            if geom.geom_type != "Polygon":
                continue
            ax.fill(*geom.exterior.xy, color="0.35", lw=0)
            for ring in geom.interiors:
                ax.fill(*ring.xy, color="white", lw=0)
        ax.plot(*scene.workspace.exterior.xy, lw=1.0, color="k", ls=":")
        for c in sl_nc.D_safe:
            ax.plot(*c.geometry.exterior.xy, lw=1.0, color="tab:blue")
        ax.plot(*np.array(seg_a.coords).T, "-", color="tab:green", lw=3,
                label="probe segment A")
        ax.plot(*np.array(seg_b.coords).T, "-", color="tab:red", lw=3,
                label="probe segment B")
        ax.plot(*center, "x", color="k", ms=9, label="door centre")
        ax.set_aspect("equal")
        ax.set_title(f"K2 {args.door}, robot {args.compile_robot}, "
                     f"theta={theta_nc:.3f}\n"
                     f"{len(scene.supports)} real 3DGS supports "
                     f"(1 u ~ 0.5 m, UNVERIFIED)")
        ax.legend(loc="upper right", fontsize=8)
        p = Path(args.figure)
        p.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(p, dpi=130, bbox_inches="tight")
        print(f"\nfigure -> {p}", flush=True)
        out["figure"] = str(p)

    op = Path(args.output)
    op.parent.mkdir(parents=True, exist_ok=True)
    op.write_text(json.dumps(out, indent=2))
    print("\nwrote", op, flush=True)


if __name__ == "__main__":
    main()
