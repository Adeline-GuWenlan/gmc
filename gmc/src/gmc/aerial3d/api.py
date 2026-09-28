"""Public API: ``compile_complex`` (scene + body + box, reusable) and ``query`` (start + goal).

Result status is three-valued (the ``gmc.types.PlanStatus`` names):

* REACHABLE    graph path found, lifted polyline exists, the own direct verifier
               certifies every segment, AND the baseline's independent gs3d replay
               (``gs3d.trajectory.replay_plan`` with a fresh ``GaussianBodyOracle``) passes;
* UNREACHABLE  start and goal lie in different components of the union of
               non-BLOCKED leaves (possible-space cut; every BLOCKED leaf on the cut is
               inside the inner polytope of one named pair), or an endpoint lies in a
               BLOCKED leaf (certified collision);
* UNKNOWN      everything else, with a reason.
"""
from __future__ import annotations

from contextlib import contextmanager, nullcontext
from dataclasses import asdict, dataclass, field
import hashlib
import json
import resource
from time import perf_counter

import numpy as np

from gmc.gs3d.contracts import BodySpec, GoalRegion, Pose3, SceneSpec
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import _json_finite, _limits, _linear_trajectory
from gmc.gs3d.trajectory import replay_plan

from .cells import CellComplex, CellConfig, grow_cell
from .envelopes import EnvelopeTable, sandwich_audit
from .graph import components, touch_pairs
from .metrics import polyline_metrics
from .octree import BLOCKED, OUTSIDE, SAFE, UNKNOWN, Octree, OctreeConfig
from .pairs import domain_from_scene, pairs_from_scene
from .query import (PortalGraph, cells_containing, merge_corners, shortcut, shorten, shorten_in_cells,
                    simplify, tighten)
from .verify import verify_polyline, verify_segment

# Same names as gmc.types.PlanStatus; not imported, because gmc.types pulls in the 2-D
# (shapely) backend and this 3-D backend must not load it.
REACHABLE, UNREACHABLE, UNKNOWN_S = "REACHABLE", "UNREACHABLE", "UNKNOWN"
METHOD = "gmc.aerial3d: pair-certified support-plane cell complex (uav.md U0)"


@dataclass(frozen=True)
class CompileConfig:
    margin_m: float = .05
    pad_m: float = 1e-3
    octree: OctreeConfig = field(default_factory=OctreeConfig)
    cells: CellConfig = field(default_factory=CellConfig)
    audit_pairs: int = 64
    audit_seed: int = 0
    ground_slab_m: float = 1e-3   # ground bodies: C-space z-window half-thickness (aerial3dg_design.md)


@dataclass(frozen=True)
class QueryConfig:
    shortcut: bool = True
    shortcut_max_attempts: int = 400
    tighten: bool = True
    tighten_rounds: int = 3
    tighten_max_checks: int = 3000
    merge_corners: bool = True
    merge_max_increase_frac: float = .03   # chosen on the synthetic suite (worklog), global
    verify_pad_m: float = .05
    verify_max_depth: int = 12
    query_cell_max_lp: int = 400
    shared_verification: bool = True


def _peak_rss_mb() -> float:
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.


class _Recorder:
    """Own stage records (with peak RSS after each stage) plus an optional gs3d TimingSink."""

    def __init__(self, timer=None, call_id: str = ""):
        self.timer, self.call_id, self.records = timer, call_id, []

    @contextmanager
    def stage(self, name: str, **sizes):
        outer = self.timer.stage(name, call_id=self.call_id, **sizes) if self.timer is not None else nullcontext()
        t0 = perf_counter()
        rec = {"stage": name, "call_id": self.call_id, "seconds": 0., "failed": False, "sizes": dict(sizes)}
        self.records.append(rec)
        try:
            with outer:
                yield rec["sizes"]
        except BaseException:
            rec["failed"] = True
            raise
        finally:
            rec["seconds"] = perf_counter() - t0
            rec["sizes"]["peak_rss_mb"] = _peak_rss_mb()


@dataclass
class CompiledComplex:
    scene: SceneSpec
    body: BodySpec
    config: CompileConfig
    frame: object
    domain: object
    pairs: object
    table: EnvelopeTable
    tree: Octree
    cells: CellComplex
    possible_label: np.ndarray
    prepared: PreparedScene
    audit: dict
    pair_stats: dict
    timings: dict
    compile_id: str

    def summary(self) -> dict:
        return {"compile_id": self.compile_id, "scene_id": self.scene.scene_id, "method": METHOD,
                "body": asdict(self.body), "config": _json_finite(asdict(self.config)),
                "frame": self.frame.json(), "domain": self.domain.json(),
                "pairs": {k: v for k, v in self.pair_stats.items() if k != "covariance_preparation"},
                "octree": _json_finite(self.tree.stats), "cells": _json_finite(self.cells.stats),
                "traceability": self.cells.traceability(),
                "free_boundary_traceability": self.boundary_traceability,
                "possible_components": int(self.possible_label.max() + 1) if len(self.possible_label) else 0,
                "envelopes": dict(self.table.stats), "audit": self.audit,
                "timings": self.timings}


def compile_complex(scene: SceneSpec, body: BodySpec, *, config: CompileConfig = CompileConfig(),
                    timer=None, prepared: PreparedScene | None = None, check=None) -> CompiledComplex:
    entered = perf_counter()
    rec = _Recorder(timer, "compile")
    with rec.stage("scene_prepare") as s:
        prep = prepared if prepared is not None else PreparedScene(scene)
        s["active_supports"] = int(len(prep.ids))
        s["reused_prepared"] = prepared is not None
    with rec.stage("pair_candidates") as s:
        frame, domain = domain_from_scene(scene, body, margin_m=config.margin_m,
                                          ground_slab_m=config.ground_slab_m)
        pairs, pstats = pairs_from_scene(scene, body, frame, domain, margin_m=config.margin_m,
                                         pad_m=config.pad_m, prepared=prep)
        s.update(candidate_pairs=len(pairs), pruned_pairs=pstats["pruned_pairs"])
    with rec.stage("envelopes") as s:
        table = EnvelopeTable(pairs)
        s.update(directions=int(len(table.U)), rho_table_mb=float(table.rho.nbytes / 2 ** 20))
    with rec.stage("octree") as s:
        tree = Octree(table, domain, config.octree, check=check)
        s.update(nodes=tree.stats["nodes"], leaves=len(tree),
                 **{k.lower(): v for k, v in tree.summary()["leaves_by_status"].items()})
    with rec.stage("cells") as s:
        cx = CellComplex.build(tree, table, domain, config.cells, check=check)
        s.update(cells=len(cx.cells), portals=int(len(cx.portal_cells)),
                 support_plane_cells=cx.stats["support_plane_cells"], box_cells=cx.stats["box_cells"])
    with rec.stage("possible_graph") as s:
        live = np.flatnonzero((tree.status == SAFE) | (tree.status == UNKNOWN))
        tp = touch_pairs(tree, live)
        lab = components(len(live), np.searchsorted(live, tp)) if len(live) else np.empty(0, np.int64)
        label = np.full(len(tree), -1, dtype=np.int64)
        label[live] = lab
        s.update(live_leaves=int(len(live)), touch_pairs=int(len(tp)),
                 components=int(lab.max() + 1) if len(lab) else 0)
    with rec.stage("audit") as s:
        rng = np.random.default_rng(config.audit_seed)
        n = len(pairs)
        pick = set(rng.choice(n, size=min(config.audit_pairs, n), replace=False).tolist()) if n else set()
        if n:
            ev = np.linalg.eigvalsh(pairs.covs)
            aniso = ev[:, -1] / np.maximum(ev[:, 0], 1e-300)
            pick |= set(np.argsort(-aniso)[:16].tolist())   # adversarial: most anisotropic
        audit = sandwich_audit(table, sorted(pick), n_dense=3000, seed=config.audit_seed) if pick else \
            {"passed": True, "pairs_audited": 0}
        s.update(pairs_audited=audit["pairs_audited"], passed=bool(audit["passed"]))
    ident = hashlib.sha256(json.dumps({"scene": scene.scene_id, "n_gaussians": int(len(scene.gaussians.ids)),
                                       "body": asdict(body), "config": _json_finite(asdict(config)),
                                       "pairs": int(len(pairs))}, sort_keys=True).encode()).hexdigest()[:16]
    compiled = CompiledComplex(scene, body, config, frame, domain, pairs, table, tree, cx, label, prep,
                               audit, pstats, {}, ident)
    compiled.boundary_traceability = _boundary_traceability(tree)
    compiled.timings = {"compile_wall_s": perf_counter() - entered, "records": rec.records,
                        "peak_rss_mb": _peak_rss_mb(),
                        "definition": "compile = scene preparation (gs3d PreparedScene) + pairs + envelopes "
                                      "+ octree + cells/portals + possible graph + audit; excludes archive "
                                      "load/crop"}
    return compiled


def save_compiled(compiled: CompiledComplex, path) -> dict:
    """Persist a compile (pickle) plus a ``.json`` sidecar holding its SHA-256; returns the sidecar."""
    import pickle
    from pathlib import Path
    path = Path(path)
    blob = pickle.dumps(compiled, protocol=pickle.HIGHEST_PROTOCOL)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(blob)
    tmp.replace(path)
    meta = {"compile_id": compiled.compile_id, "scene_id": compiled.scene.scene_id,
            "body": asdict(compiled.body), "bytes": len(blob), "sha256": hashlib.sha256(blob).hexdigest(),
            "format": "pickle protocol %d of gmc.aerial3d.api.CompiledComplex" % pickle.HIGHEST_PROTOCOL}
    path.with_name(path.name + ".json").write_text(json.dumps(meta, indent=1) + "\n")
    return meta


def load_compiled(path) -> CompiledComplex:
    """Load a persisted compile; fail closed if the bytes differ from the sidecar hash."""
    import pickle
    from pathlib import Path
    path = Path(path)
    meta = json.loads(path.with_name(path.name + ".json").read_text())
    blob = path.read_bytes()
    if hashlib.sha256(blob).hexdigest() != meta["sha256"]:
        raise ValueError("persisted compile hash differs from its sidecar")
    compiled = pickle.loads(blob)
    if not isinstance(compiled, CompiledComplex) or compiled.compile_id != meta["compile_id"]:
        raise ValueError("persisted compile identity mismatch")
    return compiled


def _boundary_traceability(tree: Octree) -> dict:
    """Faces between SAFE and non-SAFE leaves: does the non-SAFE side name its pairs?"""
    from .graph import face_pairs
    safe = tree.status == SAFE
    idx = np.flatnonzero(tree.status != OUTSIDE)
    fp, _ = face_pairs(tree, touch_pairs(tree, idx))
    if not len(fp):
        return {"free_boundary_faces": 0, "traceable_fraction": 1.0}
    a, b = fp[:, 0], fp[:, 1]
    cross = safe[a] ^ safe[b]
    other = np.where(safe[a[cross]], b[cross], a[cross])
    ok = np.array([(tree.status[o] == BLOCKED and tree.blocked_pair[o] >= 0)
                   or (tree.status[o] == UNKNOWN and (len(tree.unknown_pairs.get(int(o), ())) > 0
                                                      or tree.domain_partial[o])) for o in other])
    return {"free_boundary_faces": int(len(other)),
            "traceable_fraction": float(ok.mean()) if len(ok) else 1.0,
            "blocked_side": int(np.count_nonzero(tree.status[other] == BLOCKED)),
            "unknown_side": int(np.count_nonzero(tree.status[other] == UNKNOWN))}


def _cut_certificate(compiled: CompiledComplex, start_comps: set) -> dict:
    tree = compiled.tree
    comp_leaves = np.flatnonzero(np.isin(compiled.possible_label, sorted(start_comps)))
    blocked = np.flatnonzero(tree.status == BLOCKED)
    both = np.union1d(comp_leaves, blocked)
    tp = touch_pairs(tree, both)
    in_comp = np.isin(tp, comp_leaves)
    edge = in_comp[:, 0] ^ in_comp[:, 1]
    cut = np.unique(np.where(in_comp[edge, 0], tp[edge, 1], tp[edge, 0]))
    ids = compiled.pairs.ids[tree.blocked_pair[cut]] if len(cut) else np.empty(0, np.int64)
    traceable = float(np.mean(tree.blocked_pair[cut] >= 0)) if len(cut) else 0.
    c, d = tree.boxes(comp_leaves)
    return {"kind": "possible_space_cut",
            "claim": "start and goal are in different connected components of the union of all non-BLOCKED "
                     "leaves; every point of F_true lies in such a leaf, so no collision-free path exists "
                     "inside the domain (floating-point conservative, not exact arithmetic)",
            "start_components": sorted(int(x) for x in start_comps),
            "start_component_leaves": int(len(comp_leaves)),
            "start_component_bbox_plan": [(c - d).min(0).tolist(), (c + d).max(0).tolist()] if len(c) else None,
            "blocked_leaves_on_cut": int(len(cut)),
            "cut_pair_ids": sorted({int(i) for i in ids}),
            "cut_leaf_centres_plan": tree.boxes(cut[::max(1, len(cut) // 4000)])[0].round(4).tolist()
            if len(cut) else [],
            "cut_distinct_pairs": int(len(set(ids.tolist()))),
            "cut_traceable_fraction": traceable,
            "domain_boundary": "the remaining boundary of the start component is the C-space domain boundary"}


def _endpoint(compiled, p, name, rec):
    tree, dom = compiled.tree, compiled.domain
    if dom.ground_z is not None and abs(p[2] - dom.ground_z) > 1e-7:
        return {"ok": False, "status": UNKNOWN_S, "reason": f"{name}_off_ground_manifold"}
    if dom.row_slack(p) <= dom.tol:
        return {"ok": False, "status": UNKNOWN_S, "reason": f"{name}_outside_domain"}
    leaves = tree.locate(p)
    for l in leaves:
        if tree.status[l] == BLOCKED:
            pid = int(compiled.pairs.ids[tree.blocked_pair[l]])
            return {"ok": False, "status": UNREACHABLE, "reason": f"{name}_in_collision",
                    "certificate": {"kind": "endpoint_inside_inner_polytope", "pair_id": pid, "leaf": int(l)}}
    comps = {int(compiled.possible_label[l]) for l in leaves if compiled.possible_label[l] >= 0}
    return {"ok": True, "leaves": [int(l) for l in leaves], "components": comps}


def query(compiled: CompiledComplex, start, goal, *, config: QueryConfig = QueryConfig(),
          timer=None, call_id: str = "query") -> dict:
    entered = perf_counter()
    rec = _Recorder(timer, call_id)
    frame, body, m = compiled.frame, compiled.body, compiled.config.margin_m
    s_w, g_w = np.asarray(start, float), np.asarray(goal, float)
    s, g = frame.to_plan(s_w), frame.to_plan(g_w)
    diag = {"cells_in_complex": len(compiled.cells.cells), "portals_in_complex": int(len(compiled.cells.portal_cells))}
    out = {"schema_version": "aerial3d.v1", "method": METHOD, "compile_id": compiled.compile_id,
           "scene_id": compiled.scene.scene_id, "status": UNKNOWN_S, "reason": "initializing",
           "start_world": s_w.tolist(), "goal_world": g_w.tolist(),
           "start_plan": s.tolist(), "goal_plan": g.tolist(), "graph_path": None,
           "polyline_world": None, "polyline_plan": None, "lifted_polyline_plan": None,
           "cell_polyline_plan": None,
           "verification": {"own": None, "shared": None}, "clearance_lower_m": None,
           "certificate": None, "metrics": None, "gs3d_result": None, "diagnostics": diag}

    def finish(status, reason, **extra):
        out.update(status=status, reason=reason, **extra)
        out["timings"] = {"algorithm_wall_s": perf_counter() - entered, "records": rec.records,
                          "compile_wall_s": compiled.timings["compile_wall_s"], "mode": "warm_compiled",
                          "definition": "algorithm_wall_s = query entry to assembled result, including the "
                                        "shared gs3d replay; compile time is reported separately"}
        return _json_finite(out)

    with rec.stage("locate"):
        es = _endpoint(compiled, s, "start", rec)
        eg = _endpoint(compiled, g, "goal", rec) if es["ok"] else None
    for e in (es, eg):
        if e is not None and not e["ok"]:
            return finish(e["status"], e["reason"], certificate=e.get("certificate"))
    diag["start_leaves"], diag["goal_leaves"] = es["leaves"], eg["leaves"]
    if es["components"] and eg["components"] and not (es["components"] & eg["components"]):
        with rec.stage("cut_certificate"):
            cert = _cut_certificate(compiled, es["components"])
        return finish(UNREACHABLE, "possible_space_cut", certificate=cert)

    graph = PortalGraph(compiled.cells)
    with rec.stage("locate_cells") as st:
        ends = []
        for p, name in ((s, "start"), (g, "goal")):
            cells = cells_containing(graph.cells, p)
            if not cells:
                cell = grow_cell(compiled.table, compiled.domain, p, np.zeros(3),
                                 buffer_m=compiled.tree.config.buffer_m, kind="query")
                if cell is None:
                    return finish(UNKNOWN_S, f"{name}_not_certified_free")
                cells = [graph.add_cell(cell, max_lp=config.query_cell_max_lp)]
                st[f"{name}_query_cell"] = True
            ends.append(cells)
        st["lp_portals"] = graph.lp_portals
    with rec.stage("graph_search") as st:
        found = graph.search(s, g, ends[0], ends[1])
    if found is None:
        return finish(UNKNOWN_S, "safe_graph_disconnected_possible_connected",
                      certificate={"kind": "none", "possible_components_shared": sorted(es["components"] & eg["components"])})
    portals, cell_seq = found
    with rec.stage("lifting") as st:
        lows = [graph.p_lower[p] for p in portals]
        ups = [graph.p_upper[p] for p in portals]
        lifted = np.vstack([s, (np.asarray(lows).reshape(-1, 3) + np.asarray(ups).reshape(-1, 3)) / 2, g]) \
            if portals else np.vstack([s, g])
        pts = shorten(s, g, lows, ups)
        better = shorten_in_cells(s, g, [graph.cells[c] for c in cell_seq], pts)
        st["cell_sequence_program"] = better is not None
        if better is not None:
            pts = better
        # certificate of the lifted polyline: each segment inside one convex cell
        inside = [bool(graph.cells[c].contains_points(pts[i:i + 2], tol=1e-12).all())
                  for i, c in enumerate(cell_seq)]
        st.update(portals=len(portals), segments_inside_cells=int(sum(inside)))
        if compiled.domain.ground_z is not None:
            # ground body: every vertex back on the support manifold (moves <= slab);
            # the own verifier and the shared replay below re-check the pinned polyline
            st["pinned_max_dz_m"] = float(np.max(np.abs(pts[:, 2] - compiled.domain.ground_z)))
            pts = pts.copy()
            pts[:, 2] = compiled.domain.ground_z
        pts = simplify(pts)
    diag["lifted_segments_inside_their_cell"] = bool(all(inside))
    cell_certified = pts.copy()
    kw = {"pad_m": config.verify_pad_m, "max_depth": config.verify_max_depth}
    keep = m + compiled.tree.config.buffer_m   # post-processing keeps the cells' buffer

    def accept(a, b):
        v = verify_segment(compiled.table, compiled.domain, a, b, **kw)
        return v["status"] == "CERTIFIED" and v["clearance_lower_m"] >= keep

    if config.shortcut and len(pts) > 2:
        with rec.stage("shortcut") as st:
            pts, attempts = shortcut(pts, accept, max_attempts=config.shortcut_max_attempts)
            st["attempts"] = attempts
    if config.tighten and len(pts) > 2:
        with rec.stage("tighten") as st:
            before = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
            pts, checks = tighten(pts, accept, rounds=config.tighten_rounds,
                                  max_checks=config.tighten_max_checks)
            if len(pts) > 2:
                pts, _ = shortcut(pts, accept, max_attempts=config.shortcut_max_attempts)
            st.update(checks=checks, length_before_m=before,
                      length_after_m=float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))))
    if config.merge_corners and len(pts) > 3:
        with rec.stage("merge_corners") as st:
            L = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
            n0 = len(pts)
            pts, checks = merge_corners(pts, accept, max_increase_m=config.merge_max_increase_frac * L)
            st.update(checks=checks, vertices_before=n0, vertices_after=len(pts), length_before_m=L,
                      length_after_m=float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))))
    with rec.stage("own_verification"):
        own = verify_polyline(compiled.table, compiled.domain, pts, **kw)
    own_summary = {k: own[k] for k in ("status", "reason", "clearance_lower_m")}
    own_summary["segments"] = [{k: r.get(k) for k in ("status", "reason", "pairs_checked", "subdivisions",
                                                      "refinements", "clearance_lower_m", "closest_pair_id")}
                               for r in own["segments"]]
    out["verification"]["own"] = own_summary
    poly_w = frame.to_world(pts)
    poly_w[0], poly_w[-1] = s_w, g_w     # exact endpoints (rigid-map roundoff only)
    gp = {"cell_ids": [int(c) for c in cell_seq],
          "cells": [graph.cells[c].summary() for c in cell_seq],
          "portals": [{"cells": list(graph.p_cells[p]), "kind": graph.p_kind[p],
                       "lower_plan": graph.p_lower[p].tolist(), "upper_plan": graph.p_upper[p].tolist()}
                      for p in portals]}
    out.update(graph_path=gp, polyline_plan=pts.tolist(), polyline_world=poly_w.tolist(),
               lifted_polyline_plan=lifted.tolist(), cell_polyline_plan=cell_certified.tolist(),
               metrics=polyline_metrics(poly_w))
    if own["status"] != "CERTIFIED":
        return finish(UNKNOWN_S, f"own_verification_{own['status'].lower()}")
    gs3d = _gs3d_result(compiled, poly_w, g_w, own["clearance_lower_m"])
    if config.shared_verification:
        with rec.stage("shared_verification"):
            replay = replay_plan(gs3d, GaussianBodyOracle(compiled.prepared))
        shared = {k: v for k, v in replay.items() if k not in ("samples", "geometry")}
        shared["geometry"] = {k: replay["geometry"].get(k) for k in ("passed", "safety", "reason", "clearance_lower_m")}
        shared["oracle"] = "gs3d GaussianBodyOracle (baseline collision authority), fresh instance"
        out["verification"]["shared"] = shared
        if not replay["passed"]:
            return finish(UNKNOWN_S, "shared_replay_failed")
        clearance = min(own["clearance_lower_m"], replay["geometry"]["clearance_lower_m"])
    else:
        clearance = own["clearance_lower_m"]
    gs3d["clearance_lower_m"] = clearance
    return finish(REACHABLE, "graph_path_lifted_and_verified", clearance_lower_m=clearance, gs3d_result=gs3d)


def _gs3d_result(compiled, poly_w, goal_w, clearance) -> dict:
    body = compiled.body
    path = [Pose3(tuple(map(float, p))) for p in poly_w]
    poses, traj = _linear_trajectory(path, body, 0., 0.)
    row = lambda q: [*map(float, q.xyz), float(q.yaw)]
    return _json_finite({
        "schema_version": "gs3d.v1", "scene_id": compiled.scene.scene_id,
        "robot": {**asdict(body), "limits": _limits(body)}, "status": "success",
        "reason": "verified_goal_reached", "safety": "continuous_bound",
        "original_goal": [*map(float, goal_w), 0.], "attained_goal": row(poses[-1]),
        "position_tolerance_m": 0., "yaw_tolerance_rad": .05, "trajectory": traj,
        "clearance_lower_m": clearance, "endpoint_reports": {},
        "diagnostics": {"margin_m": compiled.config.margin_m, "planner": METHOD,
                        "compile_id": compiled.compile_id},
        "timings": {"algorithm_wall_s": 0., "preparation_wall_s": 0., "mode": "warm", "records": [],
                    "replans_s": []},
        "provenance": dict(compiled.scene.provenance)})
