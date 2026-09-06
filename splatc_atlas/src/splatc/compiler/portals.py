"""Generic certified connectors between OpenCharts (P4a.2, round-10).

Round-10 corrections embodied here:

1. NOT a topology classifier.  Round-10's width sweep showed a straight
   certified line exists at w=0.58 (still thin-family) but not at
   w=0.70/0.80 with the same anchors — the old "OpenPortal = broad
   interface / refusal = contact gate" reading was an anchor-heuristic
   artifact.  A success here therefore claims exactly one thing: a
   GENERIC CERTIFIED CONNECTOR (straight densified polyline with a
   conservative bubble certificate) exists between the two charts, with
   a certified tube of positive radius around it.  It does NOT claim
   "broad clearance", does NOT classify the interface, and its absence
   does NOT prove a contact gate — contact-structured connectors are a
   separate proof source (the continuation kernel).

2. SYMMETRIC, deterministic candidate enumeration.  Round-10 measured
   try_certify_portal(A,B) succeeding while (B,A) failed (w=0.62): the
   old theta-column policy anchored on chart A only.  Candidates are
   now derived from the UNORDERED pair: for each root-theta column the
   best (min metric-D) cross pair anchored on EITHER side enters the
   candidate set; the pair is canonicalized before any work, so
   (A,B) and (B,A) execute the identical computation by construction.

3. DOMAIN-AWARE certificates.  On rigidly transformed problems the
   obstacle certificate alone leaves the robot free to exit the true
   (rotated) workspace mid-connector; when a domain is given, a second
   conservative bubble certificate over the domain support margin must
   also close (same 1-Lipschitz motion metric), and the recorded tube
   radius uses the WORSE of the two margins.

Billing: every certificate check is counted and reported; a refusal is
evidence (all attempts on record), not silence.
"""
from __future__ import annotations

import numpy as np

from ..common.se2 import wrap_diff
from ..reference.oracle import certify_path_conservative
from .atlas_types import OpenPortal


def _densify_segment(qa, qb, a_max, step=0.02):
    """Straight polyline qa -> qb, wrap-aware in theta, spaced <= `step`
    in the certificate's motion metric D = |dxy| + a_max*|dtheta|."""
    qa = np.asarray(qa, dtype=float)
    qb = np.asarray(qb, dtype=float)
    dth = wrap_diff(qb[2] - qa[2])
    D = float(np.hypot(qb[0] - qa[0], qb[1] - qa[1]) + a_max * abs(dth))
    n = max(2, int(np.ceil(D / step)) + 1)
    t = np.linspace(0.0, 1.0, n)
    return np.stack([qa[0] + t * (qb[0] - qa[0]),
                     qa[1] + t * (qb[1] - qa[1]),
                     qa[2] + t * dth], axis=1)


def certify_domain_conservative(domain, robot, poses, max_depth=14):
    """Conservative bubble certificate that the ROBOT BODY stays inside
    the domain along the polyline: domain.support_margin is 1-Lipschitz
    in D = |dxy| + a_max*|dtheta| (support half-widths are
    a_max-Lipschitz in theta), so a segment with endpoint margins
    m1 + m2 > D is domain-contained throughout; segments that do not
    close are bisected.  Query-free (pure geometry).
    Returns (certified, min_margin_m, checks)."""
    a_max = max(robot.a, robot.b)
    stats = {"min_m": np.inf, "checks": 0}

    def margin(q):
        stats["checks"] += 1
        m = domain.support_margin(q[0], q[1], q[2], robot)
        stats["min_m"] = min(stats["min_m"], m)
        return m

    def seg(q1, q2, m1, m2, depth):
        if m1 <= 0.0 or m2 <= 0.0:
            return False
        dth = wrap_diff(q2[2] - q1[2])
        D = np.hypot(q2[0] - q1[0], q2[1] - q1[1]) + a_max * abs(dth)
        if m1 + m2 > D:
            return True
        if depth >= max_depth:
            return False
        qm = (0.5 * (q1[0] + q2[0]), 0.5 * (q1[1] + q2[1]),
              q1[2] + 0.5 * dth)
        mm = margin(qm)
        return (seg(q1, qm, m1, mm, depth + 1)
                and seg(qm, q2, mm, m2, depth + 1))

    ok = True
    m_prev = margin(tuple(poses[0]))
    for p, q in zip(poses[:-1], poses[1:]):
        m_next = margin(tuple(q))
        if not seg(tuple(p), tuple(q), m_prev, m_next, 0):
            ok = False
            break
        m_prev = m_next
    return ok, float(stats["min_m"]), int(stats["checks"])


def try_certify_portal(scene, robot, info, cid_a, cid_b, step=0.02,
                       domain=None):
    """Attempt to certify a generic connector between two charts of the
    same build.  SYMMETRIC: the pair is canonicalized (sorted chart ids)
    before any computation, so argument order cannot change the result.
    Returns (portal_or_None, record); the record documents every attempt
    (anchors, metric distance, certificate outcomes, checks)."""
    cid_a, cid_b = sorted((cid_a, cid_b))
    tree = info["tree"]
    a_max = max(robot.a, robot.b)

    def anchors(cid):
        keys = info["interface_members"].get(cid) or info["members"][cid]
        return list(keys), np.array([tree.cell_center(k) for k in keys])

    keys_a, ca = anchors(cid_a)
    keys_b, cb = anchors(cid_b)
    if not len(ca) or not len(cb):
        return None, {"attempts": [], "checks": 0,
                      "note": "no anchor cells"}
    dxy = np.hypot(ca[:, None, 0] - cb[None, :, 0],
                   ca[:, None, 1] - cb[None, :, 1])
    dth = np.abs(((ca[:, None, 2] - cb[None, :, 2] + np.pi)
                  % (2 * np.pi)) - np.pi)
    D = dxy + a_max * dth
    col_a = np.array([int(k[3] // (2 ** k[0])) for k in keys_a])
    col_b = np.array([int(k[3] // (2 ** k[0])) for k in keys_b])
    # best pair per theta column, anchored on EITHER side (symmetric)
    best = {}
    for i in range(len(keys_a)):
        j = int(np.argmin(D[i]))
        for key in (("a", int(col_a[i])),):
            if key not in best or D[i, j] < best[key][0]:
                best[key] = (float(D[i, j]), i, j)
    for j in range(len(keys_b)):
        i = int(np.argmin(D[:, j]))
        key = ("b", int(col_b[j]))
        if key not in best or D[i, j] < best[key][0]:
            best[key] = (float(D[i, j]), i, j)
    # dedupe identical (i, j) pairs; deterministic order by (D, i, j)
    cand = sorted({(d, i, j) for (d, i, j) in best.values()})
    attempts, checks_total = [], 0
    for d, i, j in cand:
        poses = _densify_segment(ca[i], cb[j], a_max, step)
        ok, mmin, depth, checks = certify_path_conservative(
            scene, robot, poses)
        checks_total += checks
        rec = {"anchor_a": [float(v) for v in ca[i]],
               "anchor_b": [float(v) for v in cb[j]],
               "metric_D": round(d, 6),
               "obstacle_certified": bool(ok),
               "obstacle_min_margin_m": round(float(mmin), 6),
               "checks": int(checks)}
        dom_ok, dom_min = True, None
        if ok and domain is not None:
            dom_ok, dom_min, dchecks = certify_domain_conservative(
                domain, robot, poses)
            checks_total += dchecks
            rec["domain_certified"] = bool(dom_ok)
            rec["domain_min_margin_m"] = round(float(dom_min), 6)
        rec["certified"] = bool(ok and dom_ok)
        attempts.append(rec)
        if ok and dom_ok:
            margins = [float(mmin)] + ([float(dom_min)]
                                       if dom_min is not None else [])
            m_eff = min(margins)
            portal = OpenPortal(
                portal_id=f"P_{cid_a}_{cid_b}",
                chart_a=cid_a, chart_b=cid_b,
                waypoints=[[float(v) for v in p] for p in poses],
                certificate={"min_margin_m": float(m_eff),
                             "obstacle_min_margin_m": float(mmin),
                             "domain_min_margin_m": dom_min,
                             "checks": int(checks_total),
                             "checker": "certify_path_conservative"
                                        "(bubble, checker#2 "
                                        "metric_margin)"
                                        " + domain support bubble",
                             # every pose within metric distance r of
                             # the polyline keeps margin >= m_eff - r
                             # (1-Lipschitz), so the tube of radius
                             # m_eff/2 is certified with margin
                             # >= m_eff/2 > 0
                             "certified_tube_radius_m":
                                 float(0.5 * m_eff)},
                kind="generic_certified_connector")
            return portal, {"attempts": attempts, "checks": checks_total}
    return None, {"attempts": attempts, "checks": checks_total}
