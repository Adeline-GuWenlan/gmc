"""P4a OpenChart auto-construction, v2 (round-8 P4a.1).

GOAL-FREE BY CONSTRUCTION: no start/goal enters the refinement.

Round-8 review findings this version fixes:

1. WAVE-COMPLETE REFINEMENT.  v1 stopped mid-wave at `queries >= budget`,
   so chart connectivity depended on grid phase and refinement order (the
   wide-door rigid kill FAIL: the same free space came out as 1 or 2
   charts depending on the rotation).  v2 refines in whole level waves:
   a wave's exact cost (the number of in-domain children) is known before
   it starts, and a wave that does not fit the remaining budget is NOT
   started.  Charts are only ever emitted at a wave boundary, so the
   partition depends on (scene, domain, budget band) — not on the order
   cells happened to leave a priority heap.

2. CELL-DOMAIN SOUNDNESS.  v1's mask tested cell centers only; member
   cells could overhang the domain boundary.  v2 classifies the full xy
   footprint (splatc.compiler.domains): only fully-inside (IN) FREE cells
   become members; CROSS cells are refined like AMBIG ones and are never
   members; OUT cells are neither evaluated nor billed.

3. SERIALIZED CERTIFIED REGION.  v1 shipped only n_cells + a center
   bbox — a boolean claim, not a proof-carrying region (the bbox can
   cross walls).  v2's OpenChart.region carries the grid spec, the
   certified member cell keys, their exact face-overlap adjacency, the
   per-cell certificate slack (m_free - r_cell > 0), and the domain
   descriptor; `chart_contains` / `chart_connect` run on the serialized
   region alone, query-free.

Chart certification semantics (unchanged): every member cell is
individually certified FREE over its FULL extent, so the union of
face-adjacent member cells is a certified free region — straight-line
motions crossing shared faces stay inside the union.  `certified=True`
means exactly that, nothing more (the frontier is listed separately).
"""
from __future__ import annotations

import numpy as np

from ..baselines.probe_methods import ProbeTree, _UF
from .atlas_types import OpenChart


def cell_bounds(root, origin, span, key):
    """Exact bounds of a cell key in the (root, origin, span) grid."""
    l, ix, iy, ik = key
    n = np.asarray(root) * (2 ** l)
    lo = np.asarray(origin) + np.array([ix, iy, ik]) / n * np.asarray(span)
    hi = np.asarray(origin) + (np.array([ix, iy, ik]) + 1.0) / n \
        * np.asarray(span)
    return lo, hi


def build_open_charts(scene, robot, budget=15000, max_level=3,
                      roi=None, theta_span=None, prefix="C", domain=None):
    """Returns (charts, build_info).  charts: list[OpenChart], largest
    first, each with a fully serialized certified region.  build_info:
    queries used, completed wave level, stop reason, member keys per
    chart, the live tree (for in-process use; everything needed offline
    is inside the regions)."""
    tree = ProbeTree(scene, robot, "uniform", max_level=max_level,
                     roi=roi, theta_span=theta_span)

    cls_cache = {}

    def cls(key):
        if domain is None:
            return "IN"
        r = cls_cache.get(key)
        if r is None:
            lo, hi = cell_bounds(tree.ROOT, tree.origin, tree.span, key)
            r = cls_cache[key] = domain.classify_cell(lo, hi)
        return r

    # round-10 blocker fix: a member cell must ALSO certify that the
    # ROBOT BODY stays inside the domain over the cell's full extent —
    # domain.support_margin at the center must clear the same Lipschitz
    # cell radius the obstacle certificate uses.  Query-free (pure
    # geometry), cached per key.
    dom_cache = {}

    def dom_slack(key):
        if domain is None:
            return None
        v = dom_cache.get(key)
        if v is None:
            c = tree.cell_center(key)
            v = dom_cache[key] = (domain.support_margin(
                c[0], c[1], c[2], robot) - tree.cell_radius(key[0]))
        return v

    def dom_ok(key):
        return domain is None or dom_slack(key) > 0.0

    # ---- wave 0: the root grid (OUT cells never evaluated or billed)
    n0 = tree.ROOT
    root_keys = [(0, i, j, k) for i in range(n0[0])
                 for j in range(n0[1]) for k in range(n0[2])]
    tree._evaluate([k for k in root_keys if cls(k) != "OUT"])

    # ---- whole-wave refinement: a wave = every refinable leaf at the
    # current level; its exact cost is counted BEFORE splitting and a
    # wave that does not fit the remaining budget is not started
    level, stop = 0, "max_level"
    while level < max_level:
        frontier = [k for k, r in tree.leaves.items() if k[0] == level
                    and ((r["status"] == "AMBIG" and cls(k) != "OUT")
                         or (r["status"] == "FREE"
                             and (cls(k) == "CROSS" or not dom_ok(k))))]
        if not frontier:
            stop = "no_frontier"
            break
        children = []
        for (l, ix, iy, ik) in frontier:
            for dx in (0, 1):
                for dy in (0, 1):
                    for dk in (0, 1):
                        ch = (l + 1, 2 * ix + dx, 2 * iy + dy, 2 * ik + dk)
                        if cls(ch) != "OUT":
                            children.append(ch)
        if tree.queries + len(children) > budget:
            stop = "budget_before_wave"
            break
        for k in frontier:
            tree.split_cell(k)
        tree._evaluate(children)
        level += 1

    # ---- members: certified FREE, cell footprint fully inside the
    # domain, AND robot support certified inside the domain (round-10)
    free = [k for k, r in tree.leaves.items()
            if r["status"] == "FREE" and cls(k) == "IN" and dom_ok(k)]
    freeset = set(free)
    uf = _UF()
    ambig_adj = {}
    for k in free:
        for nb in tree.face_adjacent_leaves(k):
            if nb in freeset:
                uf.union(k, nb)
            elif tree.leaves.get(nb, {}).get("status") == "AMBIG":
                ambig_adj.setdefault(k, set()).add(nb)
    comps = {}
    for k in free:
        comps.setdefault(uf.find(k), []).append(k)
    ordered = sorted(comps.values(), key=lambda cells: (-len(cells),
                                                        min(cells)))
    grid = {"root": [int(v) for v in tree.ROOT],
            "origin": [float(v) for v in tree.origin],
            "span": [float(v) for v in tree.span]}
    charts, members, interface = [], {}, {}
    for i, cells in enumerate(ordered):
        cid = f"{prefix}{i:03d}"
        cells_sorted = sorted(cells)
        index = {k: j for j, k in enumerate(cells_sorted)}
        edges = sorted(
            (index[k], index[nb])
            for k in cells_sorted for nb in tree.face_adjacent_leaves(k)
            if nb in index and index[k] < index[nb])
        slack = [round(float(tree.leaves[k]["m_free"])
                       - tree.cell_radius(k[0]), 9) for k in cells_sorted]
        dslack = (None if domain is None else
                  [round(dom_slack(k), 9) for k in cells_sorted])
        frontier_ids = sorted({str(a) for k in cells
                               for a in ambig_adj.get(k, ())})
        reps = [list(map(float, tree.cell_center(k)))
                for k in sorted(cells, key=lambda k: k[0])[:5]]
        charts.append(OpenChart(
            chart_id=cid,
            region={"grid": grid, "level_cap": int(max_level),
                    "cells": [list(map(int, k)) for k in cells_sorted],
                    "adjacency": [list(e) for e in edges],
                    "cell_cert_slack_m": slack,
                    "cell_domain_slack_m": dslack,
                    "domain": domain.to_dict() if domain else None},
            certified=True,
            frontier_ids=frontier_ids,
            representative_poses=reps))
        members[cid] = cells_sorted
        interface[cid] = sorted(k for k in cells if k in ambig_adj)
    info = {"queries": int(tree.queries),
            "completed_level": int(level), "stop": stop,
            "n_free": len(free),
            "n_ambig": sum(1 for r in tree.leaves.values()
                           if r["status"] == "AMBIG"),
            "n_free_cross": sum(1 for k, r in tree.leaves.items()
                                if r["status"] == "FREE"
                                and cls(k) == "CROSS"),
            "n_charts": len(charts), "members": members,
            "interface_members": interface, "domain": domain,
            "tree": tree}
    return charts, info


def locate_chart(info, pose):
    """Which chart contains this pose's certified member cell (None if
    the pose falls in an AMBIG/COLL/CROSS cell or outside the domain).
    No queries."""
    dom = info.get("domain")
    if dom is not None and not dom.contains_point(pose[0], pose[1]):
        return None
    tree = info["tree"]
    key = tree.locate(*pose)
    if key is None:
        return None
    for cid, cells in info["members"].items():
        if key in cells:
            return cid
    return None


# ---- offline region API (serialized region only, query-free) -------------

def chart_contains(region, pose):
    """Member cell key containing the pose, or None.  Runs on the
    serialized region alone."""
    grid = region["grid"]
    root = np.asarray(grid["root"])
    origin = np.asarray(grid["origin"])
    span = np.asarray(grid["span"])
    dom = region.get("domain")
    if dom is not None:
        from .domains import RigidRectDomain
        d = RigidRectDomain(dom["rect"], dom["phi_rad"], dom["t"])
        if not d.contains_point(pose[0], pose[1]):
            return None
    th = (pose[2] - origin[2]) % span[2] + origin[2]
    p = np.array([pose[0], pose[1], th])
    cellset = {tuple(c) for c in region["cells"]}
    f = (p - origin) / span
    if not (0 <= f[0] < 1 and 0 <= f[1] < 1):
        return None
    for l in range(int(region["level_cap"]), -1, -1):
        n = root * (2 ** l)
        idx = np.minimum((f * n).astype(int), n - 1)
        key = (l, int(idx[0]), int(idx[1]), int(idx[2]))
        if key in cellset:
            return key
    return None


def _shared_face_midpoint(grid, ka, kb):
    """Midpoint of the exact shared-face overlap of two face-adjacent
    member cells (theta wraps periodically).  The returned pose lies on
    the boundary of BOTH cells, so center(a) -> midpoint -> center(b) is
    a polyline inside the union of the two certified boxes."""
    root, origin, span = grid["root"], grid["origin"], grid["span"]
    loA, hiA = cell_bounds(root, origin, span, ka)
    loB, hiB = cell_bounds(root, origin, span, kb)
    tol = 1e-12
    mid = np.zeros(3)
    shared_dim = None
    for d in range(3):
        if abs(hiA[d] - loB[d]) < tol or abs(hiB[d] - loA[d]) < tol:
            shared_dim = d
            mid[d] = hiA[d] if abs(hiA[d] - loB[d]) < tol else loA[d]
            break
    if shared_dim is None:
        # theta wrap: top edge of one cell == bottom edge of the other
        shared_dim = 2
        top = origin[2] + span[2]
        if abs(hiA[2] - top) < 1e-9 and abs(loB[2] - origin[2]) < 1e-9:
            mid[2] = top
        elif abs(hiB[2] - top) < 1e-9 and abs(loA[2] - origin[2]) < 1e-9:
            mid[2] = origin[2]
        else:
            raise ValueError(f"cells {ka} and {kb} share no face")
    for d in range(3):
        if d == shared_dim:
            continue
        lo = max(loA[d], loB[d])
        hi = min(hiA[d], hiB[d])
        if hi <= lo:
            raise ValueError(f"cells {ka} and {kb}: empty face overlap")
        mid[d] = 0.5 * (lo + hi)
    return mid


def chart_connect(region, q1, q2):
    """Certified local polyline between two poses of the SAME chart, or
    None if either pose is outside the chart.  Query-free: BFS over the
    serialized member adjacency; the polyline routes center -> shared-
    face-overlap midpoint -> center, so every segment lies inside the
    union of certified cells (each segment's endpoints are in one common
    axis-aligned box; boxes are convex).  Returns (waypoints,
    certificate)."""
    k1 = chart_contains(region, q1)
    k2 = chart_contains(region, q2)
    if k1 is None or k2 is None:
        return None
    grid = region["grid"]
    keys = [tuple(c) for c in region["cells"]]
    index = {k: i for i, k in enumerate(keys)}
    adj = {i: [] for i in range(len(keys))}
    for a, b in region["adjacency"]:
        adj[a].append(b)
        adj[b].append(a)
    src, dst = index[k1], index[k2]
    prev = {src: None}
    queue = [src]
    while queue and dst not in prev:
        nxt = []
        for u in queue:
            for v in adj[u]:
                if v not in prev:
                    prev[v] = u
                    nxt.append(v)
        queue = nxt
    if dst not in prev:
        return None      # disconnected members would violate the chart
    chain = []
    v = dst
    while v is not None:
        chain.append(v)
        v = prev[v]
    chain = chain[::-1]

    def center(i):
        lo, hi = cell_bounds(grid["root"], grid["origin"], grid["span"],
                             keys[i])
        return 0.5 * (lo + hi)

    wps = [np.asarray(q1, dtype=float), center(chain[0])]
    for a, b in zip(chain[:-1], chain[1:]):
        wps.append(_shared_face_midpoint(grid, keys[a], keys[b]))
        wps.append(center(b))
    wps.append(np.asarray(q2, dtype=float))
    slack = region["cell_cert_slack_m"]
    cert = {"kind": "certified-cell-union-polyline",
            "n_cells": len(chain),
            "min_cell_cert_slack_m": float(min(slack[i] for i in chain)),
            "checker": "per-cell FREE certificate (m_free > r_cell) + "
                       "convexity of axis-aligned cells"}
    return [list(map(float, w)) for w in wps], cert
