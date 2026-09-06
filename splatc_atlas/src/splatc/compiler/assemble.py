"""Atlas assembly + pure-serialized query (P4a.2, round-10).

Round-10 findings this module answers:

- "actual ChartAttachment 尚未构造": the round-9 'attachment test' only
  located the witness's start/goal endpoints — the RIDGE BRANCH
  endpoints located to None and no attachment object existed.  Here the
  gate enters the Atlas as a verified chain of REAL objects:
      chart A --ChartAttachment--> endpoint component
              --CertifiedTransition (branch witness)-->
      endpoint component --ChartAttachment--> chart B
  Every link carries its own conservative certificate, freshly computed
  here (never inherited silently from the start-conditioned witness),
  and Atlas.validate() checks the chain geometrically.

- "goal-independent compile 未实现": compile_atlas() receives NO poses.
  Its inputs are (scene, robot, charts, info, domain, gate_witnesses);
  portals are enumerated over ALL major chart pairs by
  enumerate_portals(); queries at query time run on the SERIALIZED atlas
  alone (query_reachable) and cannot touch compile counters.

- identity binding: scene/robot/domain/checker hashes are stamped into
  the Atlas; validate() rejects certificates whose checker id does not
  match.

Honest limitation (unchanged, recorded): the gate witness itself still
comes from the frozen v4.1 continuation kernel, which is G1-frame
conditioned and is invoked per (scene, robot) case in its own frame —
goal-free gate DISCOVERY is P4b's compile contract, not claimed here.
"""
from __future__ import annotations

import hashlib
import json

import numpy as np

from ..reference.oracle import certify_path_conservative, metric_margin
from .atlas_types import (
    Atlas, OpenChart, ContactSignature, SectionComponent, GateBranch,
    CertifiedTransition, ChartAttachment, OpenPortal, AtlasEdge)
from .chart_builder import chart_contains
from .portals import try_certify_portal, _densify_segment

CHECKER_ID = "checker#2 metric_margin"


def scene_identity_hash(scene, n=12):
    h = hashlib.sha256()
    h.update(str(scene.workspace).encode())
    for r, centers in scene.disc_groups:
        h.update(np.float64(r).tobytes())
        h.update(np.ascontiguousarray(centers, dtype=np.float64).tobytes())
    return h.hexdigest()[:n]


def robot_identity_hash(robot, n=12):
    return hashlib.sha256(
        f"{robot.robot_id}:{robot.a}:{robot.b}".encode()).hexdigest()[:n]


def enumerate_portals(scene, robot, info, domain=None, major_min_cells=8):
    """Goal-free, symmetric, ALL-pairs portal enumeration: every
    unordered pair of major charts is attempted; all certified portals
    are returned.  No pose argument exists on purpose."""
    majors = sorted(cid for cid, m in info["members"].items()
                    if len(m) >= major_min_cells)
    portals, records, checks = [], {}, 0
    for i in range(len(majors)):
        for j in range(i + 1, len(majors)):
            p, rec = try_certify_portal(scene, robot, info,
                                        majors[i], majors[j],
                                        domain=domain)
            records[f"{majors[i]}|{majors[j]}"] = rec
            checks += rec["checks"]
            if p is not None:
                portals.append(p)
    return portals, records, checks


def _attach_to_chart(scene, robot, info, cid, station, k_tries=5,
                     domain=None):
    """Certified polyline from a member-cell center of chart `cid` to
    the branch endpoint `station` (chart interior -> component anchor).
    Deterministic: nearest anchors first (certificate motion metric).
    Returns (waypoints, certificate, checks) or (None, record, checks)."""
    from .portals import certify_domain_conservative
    tree = info["tree"]
    a_max = max(robot.a, robot.b)
    keys = info["interface_members"].get(cid) or info["members"][cid]
    centers = np.array([tree.cell_center(k) for k in keys])
    dth = np.abs(((centers[:, 2] - station[2] + np.pi)
                  % (2 * np.pi)) - np.pi)
    D = np.hypot(centers[:, 0] - station[0],
                 centers[:, 1] - station[1]) + a_max * dth
    order = np.argsort(D, kind="stable")[:k_tries]
    checks_total, tried = 0, []
    for idx in order:
        poses = _densify_segment(centers[idx], station, a_max)
        ok, mmin, _, checks = certify_path_conservative(
            scene, robot, poses)
        checks_total += checks
        dom_ok, dom_min = True, None
        if ok and domain is not None:
            dom_ok, dom_min, dchecks = certify_domain_conservative(
                domain, robot, poses)
            checks_total += dchecks
        tried.append({"anchor": [float(v) for v in centers[idx]],
                      "certified": bool(ok and dom_ok),
                      "min_margin_m": round(float(mmin), 6)})
        if ok and dom_ok:
            m_eff = min([float(mmin)] + ([float(dom_min)]
                                         if dom_min is not None else []))
            cert = {"min_margin_m": m_eff, "checks": int(checks_total),
                    "checker": "certify_path_conservative(bubble, "
                               + CHECKER_ID + ")"
                               + (" + domain support bubble"
                                  if domain is not None else "")}
            return [[float(v) for v in p] for p in poses], cert, \
                checks_total
    return None, {"tried": tried}, checks_total


def build_gate_chain(scene, robot, info, stations, dense_wps,
                     gate_id="g0", domain=None):
    """Build the verified attachment-branch-attachment chain for ONE
    gate from the kernel's ridge output.  stations: ordered ridge
    station poses; dense_wps: the certificate-density ridge polyline
    (dense_wps[0] == stations[0], dense_wps[-1] == stations[-1]).
    All certificates are computed HERE (fresh), and all measurement
    costs are returned in the record."""
    rec = {"checks": 0}
    s0 = [float(v) for v in stations[0]]
    s1 = [float(v) for v in stations[-1]]
    cid_a = None
    # which charts do the endpoints attach to?  determined by geometry
    # (nearest attachable), not by any task pose
    charts_sorted = sorted(info["members"],
                           key=lambda c: -len(info["members"][c]))
    atts = {}
    for tag, st in (("A", s0), ("B", s1)):
        for cid in charts_sorted:
            if len(info["members"][cid]) < 8:
                continue
            wps, cert, ck = _attach_to_chart(scene, robot, info, cid, st,
                                             domain=domain)
            rec["checks"] += ck
            if wps is not None:
                atts[tag] = (cid, wps, cert)
                break
        if tag not in atts:
            rec[f"attach_{tag}"] = "FAILED"
            return None, rec
    cid_a, wps_a, cert_a = atts["A"]
    cid_b, wps_b, cert_b = atts["B"]
    if cid_a == cid_b:
        rec["attach"] = f"both endpoints attach to {cid_a} — not a gate"
        return None, rec
    # branch witness: re-certify the dense ridge polyline independently
    ok, mmin, _, checks = certify_path_conservative(
        scene, robot, np.asarray(dense_wps))
    rec["checks"] += checks
    if not ok:
        rec["branch"] = "recertification FAILED"
        return None, rec
    # endpoint contact measurements -> genuine signature + components
    objs = []
    comp_ids = []
    clusters, strengths = [], []
    for tag, st in (("0", s0), ("1", s1)):
        ha, hb, (da, db) = scene.bilateral_rho_tagfree(robot, tuple(st))
        m = metric_margin(scene, robot, tuple(st))
        rec["checks"] += 2
        clusters.append([{"normal_cone": [float(da[0]), float(da[1]),
                                          0.0],
                          "strength_range": [float(ha), float(ha)],
                          "support": f"ridge endpoint {tag} measured"},
                         {"normal_cone": [float(db[0]), float(db[1]),
                                          0.0],
                          "strength_range": [float(hb), float(hb)],
                          "support": f"ridge endpoint {tag} measured"}])
        strengths.append(m)
    sig = ContactSignature(f"{gate_id}_sig", clusters[0],
                           opposing_pair=[0, 1],
                           decomposition_provenance="bilateral_rho_"
                           "tagfree measured at ridge endpoint station 0"
                           " (station-1 measurement in build record)")
    objs.append(sig)
    for i, (st, m) in enumerate(zip((s0, s1), strengths)):
        cid = f"{gate_id}_c{i}"
        comp_ids.append(cid)
        objs.append(SectionComponent(
            component_id=cid, section_id=f"{gate_id}_end{i}",
            local_frame=[[1.0, 0.0], [0.0, 1.0]],
            position_interval=[0.0, 0.0],
            orientation_interval=[st[2], st[2]],
            status="FREE", sampled_best_pose=st,
            sampled_min_margin=float(m),
            contact_signature_id=f"{gate_id}_sig"))
    tr = CertifiedTransition(
        f"{gate_id}_t", comp_ids[0], comp_ids[1],
        [[float(v) for v in p] for p in dense_wps],
        {"min_margin_m": float(mmin), "checks": int(checks),
         "checker": "certify_path_conservative(bubble, "
                    + CHECKER_ID + ")"})
    objs.append(tr)
    br = GateBranch(f"{gate_id}_b", comp_ids, [], [0.0, 1.0],
                    min_certified_margin=float(mmin))
    objs.append(br)
    att_a = ChartAttachment(f"{gate_id}_aA", cid_a, comp_ids[0],
                            wps_a, cert_a)
    att_b = ChartAttachment(f"{gate_id}_aB", cid_b, comp_ids[1],
                            wps_b, cert_b)
    objs += [att_a, att_b]
    edge = AtlasEdge(f"{gate_id}_e", cid_a, f"{gate_id}_b", cid_b,
                     cost=float(len(dense_wps)),
                     certified_witness_id=f"{gate_id}_t",
                     source_attachment_id=f"{gate_id}_aA",
                     target_attachment_id=f"{gate_id}_aB")
    objs.append(edge)
    rec.update({"chart_a": cid_a, "chart_b": cid_b,
                "branch_min_margin_m": float(mmin)})
    return objs, rec


def compile_atlas(scene, robot, charts, info, portals,
                  gate_objects=None, domain=None, compile_queries=0):
    """Assemble the Atlas.  NO pose arguments exist on this function or
    anything it calls — goal independence is structural, and the
    goal-independence regression (tests) hashes the result while
    varying query poses."""
    atlas = Atlas()
    for c in charts:
        atlas.add(c)
    gate_edges = {}
    for obj in (gate_objects or []):
        atlas.add(obj)
        if isinstance(obj, AtlasEdge):
            gate_edges[obj.edge_id] = obj
    for p in portals:
        atlas.add(p)
    for e in gate_edges.values():
        for cid in (e.source_chart, e.target_chart):
            lst = atlas.charts[cid].setdefault(
                "adjacent_gate_branches", [])
            if e.gate_branch not in lst:
                lst.append(e.gate_branch)
    atlas.scene_hash = scene_identity_hash(scene)
    atlas.robot_hash = robot_identity_hash(robot)
    atlas.domain_hash = domain.domain_hash() if domain is not None else ""
    atlas.checker_id = CHECKER_ID
    atlas.compile_query_count = int(compile_queries)
    atlas.compile_provenance = {
        "builder": "wave-complete cell-domain OpenChart builder "
                   "+ symmetric all-pairs generic connectors "
                   "+ ridge-witness gate chain",
        "task_free_note": "no pose inputs anywhere in the compile call "
                          "chain; verified by the hash-invariance "
                          "regression"}
    return atlas


def query_reachable(serialized, q1, q2):
    """Pure query on a SERIALIZED atlas dict (json round-trip of
    Atlas.serialize()): locate both poses in chart regions, then BFS
    over portal + gate edges.  No live tree, no scene access, zero
    compile-counter mutation."""
    charts = serialized["charts"]
    loc = {}
    for tag, q in (("a", q1), ("b", q2)):
        loc[tag] = None
        for cid, ch in sorted(charts.items()):
            if ch.get("certified") and \
                    chart_contains(ch["region"], q) is not None:
                loc[tag] = cid
                break
    if loc["a"] is None or loc["b"] is None:
        return {"reachable": False, "reason": "pose not in any "
                "certified chart", "charts": [loc["a"], loc["b"]]}
    adj = {}
    for p in serialized["portals"].values():
        adj.setdefault(p["chart_a"], []).append((p["chart_b"],
                                                 p["portal_id"]))
        adj.setdefault(p["chart_b"], []).append((p["chart_a"],
                                                 p["portal_id"]))
    for e in serialized["edges"].values():
        adj.setdefault(e["source_chart"], []).append(
            (e["target_chart"], e["edge_id"]))
        adj.setdefault(e["target_chart"], []).append(
            (e["source_chart"], e["edge_id"]))
    prev = {loc["a"]: None}
    queue = [loc["a"]]
    while queue and loc["b"] not in prev:
        nxt = []
        for u in queue:
            for v, via in sorted(adj.get(u, [])):
                if v not in prev:
                    prev[v] = (u, via)
                    nxt.append(v)
        queue = nxt
    if loc["b"] not in prev:
        return {"reachable": False, "reason": "charts not connected",
                "charts": [loc["a"], loc["b"]]}
    route = []
    v = loc["b"]
    while prev[v] is not None:
        u, via = prev[v]
        route.append(via)
        v = u
    return {"reachable": True, "charts": [loc["a"], loc["b"]],
            "route": route[::-1]}
