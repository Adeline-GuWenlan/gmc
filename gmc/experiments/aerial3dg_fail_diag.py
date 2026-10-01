"""F1 Task 2: per-pair diagnostics of the G2 non-REACHABLE rows, traced through the planner's own objects.

Run from ``gmc/`` with ``PYTHONPATH=src:experiments`` via sbatch (loads a persisted G2 compile, read-only).
Nothing here changes the planner; every check replays the code path that produced the G2 reason.

* ``endpoints``  every ``{start,goal}_not_certified_free`` row: replays ``api.query``'s locate_cells /
                 ``cells.grow_cell`` test on that endpoint (domain box status, fixed-direction gaps, refined
                 gaps vs ``buffer + slack``) and names the pairs that stop the certificate (scene id, role,
                 mean, 2-sigma top z, refined gap).
* ``bridges``    ``safe_graph_disconnected_possible_connected`` rows: components of the free-cell graph
                 (cells + portals), UNKNOWN-leaf regions touching SAFE leaves of >= 2 cell components
                 (the "bridges" that keep the possible graph connected), the pairs left unresolved in them,
                 gs3d-oracle samples inside the bridge leaves, union-of-inner-polytope coverage of those
                 samples, and a 1 cm oracle free-grid connectivity test across each bridge.
* ``replay``     ``shared_replay_failed`` rows: re-runs the query on the same compile and replays the exported
                 trajectory edge by edge with the gs3d oracle, recording the first rejected edge and why.
"""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path
import time

import numpy as np

from gmc.aerial3d import api as a3api
from gmc.aerial3d.api import load_compiled, query
from gmc.aerial3d.graph import components, touch_pairs
from gmc.aerial3d.octree import BLOCKED, SAFE, UNKNOWN, STATUS_NAMES
from gmc.aerial3d.query import PortalGraph, cells_containing
from gmc.gs3d.contracts import Pose3
from gmc.gs3d.oracle import GaussianBodyOracle
from gmc.gs3d.planner import _json_finite
from gmc.gs3d.validation import verify_path
from gmc.gs3d.trajectory import trajectory_poses

from aerial3dg_run import MANIFEST, QCONFIG, role_of

G2 = Path("results/aerial3dg/g2")
A3C = Path("/scratch/wg2381/splathjb-aerial3dg/gmc/outputs/aerial3dg/demo")


def g2_rows(robot, runs=G2 / "runs"):
    rows = {}
    for f in sorted(glob.glob(str(runs / robot / "task_*.jsonl"))):
        for line in open(f):
            r = json.loads(line)
            rows[r["index"]] = r
    return rows


def plan_point(compiled, uv):
    return np.array([uv[0], uv[1], compiled.domain.ground_z])


def gaussian_info(compiled, k, man):
    """Pair index k -> scene id, role, route-frame mean, 2-sigma half extents, 2-sigma top z."""
    pairs = compiled.pairs
    lvl = pairs.level
    sd = lvl * np.sqrt(np.maximum(np.diagonal(pairs.covs[k]), 0.))
    mu = pairs.means[k]                       # plan frame == route frame (aerial3dg_batch._RouteFrame)
    body = compiled.body
    bottom = body.ground_clearance_m          # route z of the chassis bottom (floor at z 0)
    pid = int(pairs.ids[k])
    return {"pair_index": int(k), "scene_id": pid, "role": role_of(pid, man), "mean_route": mu.round(4).tolist(),
            "two_sigma_half_m": sd.round(4).tolist(), "top_z_2sigma": float(mu[2] + sd[2]),
            "bottom_z_2sigma": float(mu[2] - sd[2]),
            "centre_below_chassis_bottom": bool(mu[2] < bottom),
            "vertical_gap_to_chassis_bottom_m": float(bottom - (mu[2] + sd[2]))}


# ----------------------------------------------------------------------------- endpoints
def endpoint_trace(compiled, p, man, max_refine=64):
    """Replay of ``grow_cell(table, domain, p, 0, buffer)``'s None branches, with names."""
    table, dom, tree = compiled.table, compiled.domain, compiled.tree
    pairs = table.pairs
    buffer_m = tree.config.buffer_m
    out = {"domain_box_status": dom.box_status(p, np.zeros(3)), "domain_row_slack_m": dom.row_slack(p)}
    slack_rows = dom.b - dom.A @ p
    out["tightest_domain_row"] = dom.labels[int(np.argmin(slack_rows))]
    leaves = tree.locate(p)
    out["leaf_status"] = sorted({STATUS_NAMES[int(tree.status[l])] for l in leaves})
    out["in_existing_cell"] = bool(cells_containing(compiled.cells.cells, p))
    n = len(pairs)
    gaps = np.empty(n)
    for s in range(0, n, 32768):
        g, _, _ = table.best_gap(np.arange(s, min(s + 32768, n)), p, np.zeros(3))
        gaps[s:s + len(g)] = g
    thr = buffer_m + pairs.slack
    thr = np.broadcast_to(thr, gaps.shape)
    bad = np.flatnonzero(gaps <= thr)
    out["pairs_failing_fixed_directions"] = int(len(bad))
    blockers = []
    for i in bad[np.argsort(gaps[bad])][:max_refine]:
        rg, _ = table.refine_gap(int(i), p, np.zeros(3))
        info = gaussian_info(compiled, int(i), man)
        info.update(fixed_gap_m=float(gaps[i]), refined_gap_m=float(rg), threshold_m=float(thr[i]),
                    blocks_certificate=bool(rg <= thr[i]))
        blockers.append(info)
    stop = [b for b in blockers if b["blocks_certificate"]]
    out["blockers"] = stop[:8]
    out["n_blockers"] = len(stop)
    out["refined_ok"] = len(blockers) - len(stop)
    out["min_refined_gap_m"] = min((b["refined_gap_m"] for b in blockers), default=None)
    if out["domain_box_status"] != "inside":
        cause = "domain_boundary"
    elif not stop:
        cause = "none_reproduced"
    else:
        b0 = min(stop, key=lambda b: b["refined_gap_m"])
        cause = "under_chassis_gaussian" if b0["centre_below_chassis_bottom"] else "lateral_gaussian"
    out["cause"] = cause
    # exact gs3d oracle at the endpoint (what the sampler accepted)
    oracle = GaussianBodyOracle(compiled.prepared)
    w = compiled.frame.to_world(p)
    rep = oracle.pose(Pose3(tuple(map(float, w)), 0.), compiled.body, margin_m=compiled.config.margin_m)
    out["oracle"] = {"occupancy": rep.occupancy, "clearance_lower_m": rep.clearance_lower_m, "reason": rep.reason}
    return out


def cmd_endpoints(a):
    compiled = load_compiled(a.a3c)
    man = json.loads(MANIFEST.read_text())
    rows = g2_rows(a.robot)
    todo = [r for r in rows.values() if r["reason"].endswith("_not_certified_free")]
    out = {"robot": a.robot, "a3c": str(a.a3c), "compile_id": compiled.compile_id, "n": len(todo),
           "buffer_m": compiled.tree.config.buffer_m, "margin_m": compiled.config.margin_m,
           "chassis_bottom_route_z": compiled.body.ground_clearance_m, "rows": []}
    t0 = time.perf_counter()
    for r in sorted(todo, key=lambda r: r["index"]):
        which = r["reason"].split("_")[0]
        p = plan_point(compiled, r[f"{which}_uv"])
        tr = endpoint_trace(compiled, p, man)
        tr.update(index=r["index"], pair_id=r["pair_id"], reason=r["reason"], endpoint=which,
                  endpoint_uv=r[f"{which}_uv"])
        out["rows"].append(tr)
    out["wall_s"] = time.perf_counter() - t0
    causes = {}
    for t in out["rows"]:
        causes[t["cause"]] = causes.get(t["cause"], 0) + 1
    out["cause_counts"] = causes
    _dump(a.out, out)
    print(a.robot, "endpoints", len(todo), causes, round(out["wall_s"], 1), flush=True)


# ----------------------------------------------------------------------------- bridges
def cell_components(cx):
    pc = np.asarray(cx.portal_cells, dtype=np.int64).reshape(-1, 2)
    return components(len(cx.cells), pc)


def cmd_bridges(a):
    compiled = load_compiled(a.a3c)
    man = json.loads(MANIFEST.read_text())
    tree, cx, table = compiled.tree, compiled.cells, compiled.table
    rows = g2_rows(a.robot)
    todo = [r for r in rows.values() if r["reason"] == "safe_graph_disconnected_possible_connected"]
    t0 = time.perf_counter()
    ccomp = cell_components(cx)
    safe = np.flatnonzero(tree.status == SAFE)
    unk = np.flatnonzero(tree.status == UNKNOWN)
    leafcomp = np.full(len(tree), -1, dtype=np.int64)
    for l in safe:
        cs = cx.leaf_cells.get(int(l), [])
        if cs:
            leafcomp[l] = ccomp[cs[0]]
    # UNKNOWN regions and the cell components they touch
    tp_u = touch_pairs(tree, unk)
    ulab = components(len(unk), np.searchsorted(unk, tp_u)) if len(unk) else np.empty(0, np.int64)
    both = np.union1d(unk, safe)
    tp = touch_pairs(tree, both)
    is_u = np.isin(tp, unk)
    mixed = tp[is_u[:, 0] ^ is_u[:, 1]]
    u_side = np.where(np.isin(mixed[:, 0], unk), mixed[:, 0], mixed[:, 1])
    s_side = np.where(np.isin(mixed[:, 0], unk), mixed[:, 1], mixed[:, 0])
    ureg = ulab[np.searchsorted(unk, u_side)]
    touched = {}
    for rg, sc in zip(ureg, leafcomp[s_side]):
        if sc >= 0:
            touched.setdefault(int(rg), set()).add(int(sc))
    bridges = {rg: cs for rg, cs in touched.items() if len(cs) >= 2}
    # per-pair: endpoint cell components
    graph = PortalGraph(cx)

    def comp_of(p):
        cs = cells_containing(cx.cells, p)
        if cs:
            return sorted({int(ccomp[c]) for c in cs}), "existing_cell"
        near = tree.locate(p)
        # grown query cell: connect like api.query does (LP portals) and read their components
        cell = a3api.grow_cell(table, compiled.domain, p, np.zeros(3), buffer_m=tree.config.buffer_m, kind="query")
        if cell is None:
            return [], "not_certified"
        g = PortalGraph(cx)
        cid = g.add_cell(cell, max_lp=QCONFIG.query_cell_max_lp)
        nb = {int(x) for pid in g.cell_portals[cid] for x in g.p_cells[pid] if x != cid}
        return sorted({int(ccomp[c]) for c in nb}), "query_cell"

    per_pair = []
    comp_cache = {}
    for r in sorted(todo, key=lambda r: r["index"]):
        sc, sk = comp_of(plan_point(compiled, r["start_uv"]))
        gc, gk = comp_of(plan_point(compiled, r["goal_uv"]))
        per_pair.append({"index": r["index"], "pair_id": r["pair_id"], "start_comps": sc, "goal_comps": gc,
                         "start_kind": sk, "goal_kind": gk,
                         "joined_by_bridges": sorted(int(b) for b, cs in bridges.items()
                                                     if set(sc) & cs and set(gc) & cs)})
    # component geometry
    comp_info = {}
    for c in np.unique(ccomp):
        idx = np.flatnonzero(ccomp == c)
        lo = np.min([cx.cells[i].bbox_lower for i in idx], axis=0)
        hi = np.max([cx.cells[i].bbox_upper for i in idx], axis=0)
        comp_info[int(c)] = {"cells": int(len(idx)), "bbox_uv": [round(lo[0], 3), round(lo[1], 3),
                                                                  round(hi[0], 3), round(hi[1], 3)]}
    oracle = GaussianBodyOracle(compiled.prepared)
    body, margin = compiled.body, compiled.config.margin_m

    def free_at(P):
        out = []
        for p in P:
            w = compiled.frame.to_world(p)
            rep = oracle.pose(Pose3(tuple(map(float, w)), 0.), body, margin_m=margin)
            out.append(rep.occupancy)
        return out

    # UNKNOWN-leaf graph (indices into unk) and, per UNKNOWN leaf, the cell components of touching SAFE leaves
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra
    e = np.searchsorted(unk, tp_u)
    G = coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(len(unk), len(unk))).tocsr()
    u_pos = np.searchsorted(unk, u_side)
    combos = {}
    for pp in per_pair:
        if pp["start_comps"] and pp["goal_comps"]:
            combos.setdefault((pp["start_comps"][0], pp["goal_comps"][0]), []).append(pp["index"])
    binfo = {}
    for (ca, cb), owners in sorted(combos.items(), key=lambda kv: -len(kv[1])):
        key = f"{ca}-{cb}"
        A_adj = np.unique(u_pos[leafcomp[s_side] == ca])
        B_adj = np.unique(u_pos[leafcomp[s_side] == cb])
        if not len(A_adj) or not len(B_adj):
            binfo[key] = {"pairs_owned": len(owners), "note": "a component touches no UNKNOWN leaf"}
            continue
        dA = dijkstra(G, directed=False, indices=A_adj, min_only=True, unweighted=True)
        dB = dijkstra(G, directed=False, indices=B_adj, min_only=True, unweighted=True)
        tot = dA + dB
        m = float(np.min(tot))
        if not np.isfinite(m):
            binfo[key] = {"pairs_owned": len(owners), "note": "no UNKNOWN-leaf path between the components"}
            continue
        corridor_pos = np.flatnonzero(tot <= m + 2)
        leaves = unk[corridor_pos]
        c, d = tree.boxes(leaves)
        pids, roles, gauss = set(), {}, {}
        for l in leaves:
            for k in tree.unknown_pairs.get(int(l), ()):
                pids.add(int(k))
        for k in pids:
            gi = gaussian_info(compiled, k, man)
            roles[gi["role"]] = roles.get(gi["role"], 0) + 1
            gauss[k] = gi
        offs = np.array([[i, j, 0.] for i in (-.66, 0., .66) for j in (-.66, 0., .66)])
        samples = (c[:, None, :] + offs[None] * d[:, None, :]).reshape(-1, 3)
        samples[:, 2] = compiled.domain.ground_z
        occ = np.asarray(free_at(samples)).reshape(len(leaves), 9)
        inner = np.zeros((len(leaves), 9), dtype=bool)
        for li, l in enumerate(leaves):
            P = samples[li * 9:(li + 1) * 9]
            for k in tree.unknown_pairs.get(int(l), ()):
                inner[li] |= table.points_in_inner(int(k), P)
        leaf_class = np.where((occ == "free").all(1), "all_free",
                              np.where((occ == "free").any(1), "mixed", "no_free_sample"))
        lo, hi = (c - d).min(0), (c + d).max(0)
        grid = None
        if a.grid and (hi[0] - lo[0] + .2) * (hi[1] - lo[1] + .2) <= a.grid_max_m2:
            step = a.grid_step
            us = np.arange(lo[0] - .1, hi[0] + .1 + 1e-9, step)
            vs = np.arange(lo[1] - .1, hi[1] + .1 + 1e-9, step)
            P = np.array([[u, v, compiled.domain.ground_z] for u in us for v in vs])
            F = (np.asarray(free_at(P)) == "free").reshape(len(us), len(vs))
            lab_grid = _grid_components(F)
            touch = {}
            for (i, j), lab in np.ndenumerate(lab_grid):
                if lab < 0:
                    continue
                cs = cells_containing(cx.cells, P[i * len(vs) + j])
                for cc in {int(ccomp[x]) for x in cs}:
                    touch.setdefault(int(lab), set()).add(cc)
            joining = {k: sorted(v) for k, v in touch.items() if {ca, cb} <= v}
            grid = {"step_m": step, "n_points": int(F.size), "free_points": int(F.sum()),
                    "free_components": int(lab_grid.max() + 1) if F.any() else 0,
                    "free_component_touching_both_cell_components": bool(joining),
                    "joining": {str(k): v for k, v in joining.items()},
                    "u_range": [float(us[0]), float(us[-1])], "v_range": [float(vs[0]), float(vs[-1])]}
            np.save(Path(a.out).with_name(f"corridor_{key}_grid.npy"), F)
        binfo[key] = {
            "components": [int(ca), int(cb)], "pairs_owned": len(owners),
            "corridor_min_hops": m, "corridor_leaves": int(len(leaves)),
            "region_leaves": int(np.count_nonzero(np.isfinite(dA) & np.isfinite(dB))),
            "bbox_uv": [round(lo[0], 3), round(lo[1], 3), round(hi[0], 3), round(hi[1], 3)],
            "unresolved_pairs": len(pids), "unresolved_by_role": roles,
            "unresolved_gaussians": sorted(gauss.values(), key=lambda g: g["mean_route"][1])[:200],
            "leaf_oracle_class": {k: int((leaf_class == k).sum()) for k in ("all_free", "mixed", "no_free_sample")},
            "samples_oracle": {k: int((occ == k).sum()) for k in ("free", "occupied", "unknown")},
            "samples_in_union_of_inner": int(inner.sum()), "samples": int(inner.size),
            "no_free_leaves_all_samples_in_union_of_inner": int(sum(
                inner[i].all() for i in np.flatnonzero(leaf_class == "no_free_sample"))),
            "no_free_leaves": int((leaf_class == "no_free_sample").sum()),
            "leaf_centres_uv": c[:, :2].round(3).tolist(), "leaf_half_uv": d[:, :2].round(4).tolist(),
            "leaf_class": leaf_class.tolist(), "grid": grid}
        print("corridor", key, {k: binfo[key][k] for k in ("pairs_owned", "corridor_min_hops", "corridor_leaves",
                                                          "bbox_uv", "unresolved_by_role", "leaf_oracle_class",
                                                          "no_free_leaves_all_samples_in_union_of_inner")},
              json.dumps(grid)[:300] if grid else None, flush=True)
    out = {"robot": a.robot, "a3c": str(a.a3c), "compile_id": compiled.compile_id, "n_rows": len(todo),
           "cell_components": int(ccomp.max() + 1), "component_info": comp_info,
           "unknown_regions": int(ulab.max() + 1) if len(ulab) else 0,
           "bridging_regions": {str(k): sorted(v) for k, v in bridges.items()},
           "corridors": binfo, "per_pair": per_pair,
           "wall_s": time.perf_counter() - t0}
    _dump(a.out, out)
    print(a.robot, "bridges done", len(todo), round(out["wall_s"], 1), flush=True)


def _grid_components(F):
    from scipy.ndimage import label
    lab, n = label(F, structure=np.ones((3, 3)))
    return lab - 1


# ----------------------------------------------------------------------------- replay
def cmd_replay(a):
    compiled = load_compiled(a.a3c)
    man = json.loads(MANIFEST.read_text())
    rows = g2_rows(a.robot)
    todo = [r for r in rows.values() if r["reason"] == "shared_replay_failed"]
    oracle = GaussianBodyOracle(compiled.prepared)
    body, margin = compiled.body, compiled.config.margin_m
    z_c = compiled.domain.ground_z
    out_rows = []
    t0 = time.perf_counter()
    for r in sorted(todo, key=lambda r: r["index"]):
        s = compiled.frame.to_world([*r["start_uv"], z_c])
        g = compiled.frame.to_world([*r["goal_uv"], z_c])
        q = query(compiled, s, g, config=QCONFIG, call_id=r["pair_id"])
        rec = {"index": r["index"], "pair_id": r["pair_id"], "status_now": q["status"], "reason_now": q["reason"],
               "shared": (q["verification"] or {}).get("shared"), "own_status": ((q["verification"] or {}).get("own") or {}).get("status")}
        if q["polyline_world"] is not None:
            poly = np.asarray(q["polyline_world"], float)
            gs = a3api._gs3d_result(compiled, a3api._densify(poly, QCONFIG.export_max_segment_m), g,
                                     q["verification"]["own"]["clearance_lower_m"])
            poses = trajectory_poses(gs["trajectory"])
            ver = verify_path(oracle, poses, body, margin_m=margin)
            rep = ver["reports"][-1] if ver["reports"] else None
            k = len(ver["reports"]) - 1
            fail = None
            if not ver["passed"] and rep is not None:
                a_, b_ = poses[k], poses[k + 1] if k + 1 < len(poses) else poses[k]
                ra, rb = compiled.frame.to_plan(np.asarray(a_.xyz)), compiled.frame.to_plan(np.asarray(b_.xyz))
                ids = rep.get("primitive_ids") or ()
                gi = []
                for pid in ids[:4]:
                    hit = np.flatnonzero(compiled.pairs.ids == pid)
                    if len(hit):
                        gi.append(gaussian_info(compiled, int(hit[0]), man))
                    else:
                        gi.append({"scene_id": int(pid), "role": role_of(int(pid), man), "not_a_candidate_pair": True})
                fail = {"edge_index": k, "edges_total": len(poses) - 1, "edge_uv": [ra[:2].round(4).tolist(), rb[:2].round(4).tolist()],
                        "edge_len_m": float(np.linalg.norm(rb - ra)), "turn_in_place": bool(np.allclose(ra, rb)),
                        "yaw": [float(a_.yaw), float(b_.yaw)],
                        "occupancy": rep["occupancy"], "safety": rep["safety"], "reason": rep["reason"],
                        "clearance_lower_m": rep["clearance_lower_m"], "primitive_ids": list(ids)[:8],
                        "gaussians": gi}
                # own verifier's clearance along the polyline segment containing the failing edge
            rec.update(replay_passed=ver["passed"], replay_reason=ver["reason"], first_failure=fail,
                       own_clearance_m=q["verification"]["own"]["clearance_lower_m"],
                       polyline_route=compiled.frame.to_plan(poly)[:, :2].round(4).tolist())
        out_rows.append(rec)
        print(r["pair_id"], q["status"], q["reason"], (rec.get("first_failure") or {}).get("reason"),
              (rec.get("first_failure") or {}).get("occupancy"), flush=True)
    reasons = {}
    for x in out_rows:
        k = (x.get("first_failure") or {}).get("reason", x["reason_now"])
        reasons[k] = reasons.get(k, 0) + 1
    _dump(a.out, {"robot": a.robot, "compile_id": compiled.compile_id, "n": len(todo), "first_failure_reasons": reasons,
                  "rows": out_rows, "wall_s": time.perf_counter() - t0})
    print("replay reasons", reasons, flush=True)


def _dump(path, doc):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_json_finite(doc), indent=1, default=float) + "\n")


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("cmd", choices=["endpoints", "bridges", "replay"])
    p.add_argument("--robot", required=True)
    p.add_argument("--a3c", type=Path, default=None)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--grid", action="store_true")
    p.add_argument("--grid-step", type=float, default=.01)
    p.add_argument("--grid-max-m2", type=float, default=4.)
    a = p.parse_args(argv)
    if a.a3c is None:
        a.a3c = A3C / f"{a.robot}.a3c"
    {"endpoints": cmd_endpoints, "bridges": cmd_bridges, "replay": cmd_replay}[a.cmd](a)


if __name__ == "__main__":
    main()
