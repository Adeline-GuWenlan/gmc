"""Own direct continuous verifier for translation segments (uav.md §11.2).

Independent of cells and octree: for a closed segment [a, b] of body centres, every
pair whose exact C-obstacle AABB comes within ``pad`` of the segment must be
separated from the whole segment by a support plane of O_i:

    gap_i(u) = |u.(mu_i - m)| - rho_i(u) - |u.h|  > slack,   m = (a+b)/2, h = (b-a)/2

(fixed +-U directions, then pattern-search refinement, then bisection of the
segment).  At maximum depth a sub-segment whose midpoint is inside some inner
polytope is a certified COLLISION; otherwise UNRESOLVED (never free).  The
clearance lower bound is ``margin + min gap``, capped at ``margin + pad`` because
unchecked pairs are farther than ``pad`` along an axis.  The shared final check is
still the baseline's ``gs3d`` replay; this verifier is additional.
"""
from __future__ import annotations

import numpy as np

from .envelopes import EnvelopeTable
from .pairs import Domain


def _segment_gaps(table: EnvelopeTable, index, m, h):
    P = (table.pairs.means[index] - m) @ table.U.T
    g = np.abs(P) - table.rho[index] - np.abs(table.U @ h)[None]
    return g.max(axis=1)


def _refine_segment(table: EnvelopeTable, i: int, m, h) -> float:
    mu = table.pairs.means[i]

    def evaluate(W):
        W = W / np.linalg.norm(W, axis=1, keepdims=True)
        return np.abs(W @ (mu - m)) - table.pairs.rho(W, [i])[0] - np.abs(W @ h), W

    seeds = [table.U[np.argmax(np.abs((mu - m) @ table.U.T) - table.rho[i] - np.abs(table.U @ h))]]
    t = mu - m
    along = h / (np.linalg.norm(h) or 1.)
    perp = t - (t @ along) * along   # direction from the segment's line to the mean
    for v in (t, perp):
        if np.linalg.norm(v) > 0:
            seeds.append(v)
    g, W = evaluate(np.asarray(seeds, float))
    j = int(np.argmax(g))
    gap, u = float(g[j]), W[j]
    phis = np.linspace(0, 2 * np.pi, 12, endpoint=False)
    for theta in np.radians([8., 4., 2., 1., .5, .25, .12, .06, .03]):
        for _ in range(6):
            a = np.array([0., u[2], -u[1]]) if abs(u[0]) < .9 else np.array([-u[2], 0., u[0]])
            a /= np.linalg.norm(a)
            b = np.array([u[1] * a[2] - u[2] * a[1], u[2] * a[0] - u[0] * a[2], u[0] * a[1] - u[1] * a[0]])
            cand = np.cos(theta) * u[None] + np.sin(theta) * (np.cos(phis)[:, None] * a
                                                              + np.sin(phis)[:, None] * b)
            gg, W = evaluate(cand)
            k = int(np.argmax(gg))
            if gg[k] > gap:
                gap, u = float(gg[k]), W[k]
            else:
                break
    return gap


def verify_segment(table: EnvelopeTable, domain: Domain, a, b, *, pad_m: float = .05,
                   max_depth: int = 12) -> dict:
    pairs = table.pairs
    a, b = np.asarray(a, float), np.asarray(b, float)
    out = {"a": a.tolist(), "b": b.tolist(), "pairs_checked": 0, "subdivisions": 0,
           "refinements": 0, "pair_ids": []}
    if min(domain.row_slack(a), domain.row_slack(b)) <= domain.tol:
        out.update(status="UNRESOLVED", reason="outside_domain", clearance_lower_m=None)
        return out
    lo, hi = np.minimum(a, b) - pad_m, np.maximum(a, b) + pad_m
    idx = np.flatnonzero(np.all(pairs.aabb_upper >= lo, axis=1) & np.all(pairs.aabb_lower <= hi, axis=1))
    out["pairs_checked"] = int(len(idx))
    threshold = pairs.slack
    min_gap = pad_m
    min_pair = -1
    # work items: (pair indices still to separate, a, b, depth)
    stack = [(idx, a, b, 0)]
    while stack:
        index, sa, sb, depth = stack.pop()
        if not len(index):
            continue
        m, h = (sa + sb) / 2, (sb - sa) / 2
        g = _segment_gaps(table, index, m, h)
        fail = np.flatnonzero(g <= threshold)
        for j in fail:
            i = int(index[j])
            if table.points_in_inner(i, m[None])[0]:
                out.update(status="COLLISION", reason="point_inside_inner_polytope",
                           pair_ids=[int(pairs.ids[i])], witness_point=m.tolist(), clearance_lower_m=0.)
                return out
            # |g(u)-g(v)| <= |u-v| (|mu-m| + R_i + |h|): refinement is hopeless below this bound
            if g[j] + table.cover_angle * (np.linalg.norm(pairs.means[i] - m) + table.circumradius[i]
                                           + np.linalg.norm(h)) <= threshold:
                continue
            out["refinements"] += 1
            g[j] = _refine_segment(table, i, m, h)
        ok = g > threshold
        if np.any(ok):
            j = int(np.argmin(np.where(ok, g, np.inf)))
            if g[j] < min_gap:
                min_gap, min_pair = float(g[j]), int(index[j])
        rest = index[~ok]
        if not len(rest):
            continue
        if depth >= max_depth:
            for i in rest:
                if table.points_in_inner(int(i), m[None])[0]:
                    out.update(status="COLLISION", reason="point_inside_inner_polytope",
                               pair_ids=[int(pairs.ids[i])], witness_point=m.tolist(),
                               clearance_lower_m=0.)
                    return out
            out.update(status="UNRESOLVED", reason="separation_unresolved_at_max_depth",
                       pair_ids=[int(pairs.ids[i]) for i in rest[:32]], clearance_lower_m=None)
            return out
        out["subdivisions"] += 1
        stack.append((rest, m, sb, depth + 1))
        stack.append((rest, sa, m, depth + 1))
    out.update(status="CERTIFIED", reason="support_plane_separation",
               clearance_lower_m=float(pairs.margin + min(min_gap, pad_m)),
               closest_pair_id=int(pairs.ids[min_pair]) if min_pair >= 0 else None)
    return out


def verify_polyline(table: EnvelopeTable, domain: Domain, points, **kw) -> dict:
    P = np.atleast_2d(np.asarray(points, float))
    segs = list(zip(P[:-1], P[1:])) if len(P) > 1 else [(P[0], P[0])]
    trace = []
    for a, b in segs:
        r = verify_segment(table, domain, a, b, **kw)
        trace.append(r)
        if r["status"] != "CERTIFIED":
            return {"status": r["status"], "reason": r["reason"], "segments": trace,
                    "clearance_lower_m": None}
    return {"status": "CERTIFIED", "reason": "all_segments_separated", "segments": trace,
            "clearance_lower_m": min(r["clearance_lower_m"] for r in trace)}
