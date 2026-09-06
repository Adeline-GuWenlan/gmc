"""ABLATION ARM (review round-4 §2.3, the key isolation): SAME continuation
architecture, SCALAR-CLEARANCE-ONLY information.

Everything structural is kept from v4.1 with identical frozen constants:
250mm seed grid, m-descending candidate trials (K=8), polish-validate,
bidirectional predictor-corrector, 15-point theta scan +-0.35 with
lobe-local run selection (+-2 steps), metric step law (4x, 5mm floor,
0.12 cap, x0.4 shrink, 3 retries), 800-iteration cap, section components,
certified connectors from healthiest anchors, 2cm densify polish, the same
conservative path certificate, |x|>0.7 room-exit gates.

The ONLY change: the probe returns the SCALAR capped clearance rho(q)
(min over all pairs) — no opposing-side decomposition, no contact
directions, no temporal signature.  Consequences implemented faithfully:
  balance (B-bisection)  ->  lateral ternary MAXIMIZATION of m along u
  grad_B                 ->  grad m (finite difference; points toward the
                             local clearance ridge)
  bilateral seed filter  ->  in-shell filter (0 > m or m < 0.2 cannot ask
                             "both sides"), validation m>0 and m<=0.22
  active-set exit        ->  m >= 0.249 (nothing in any shell)

If this arm matches the pair-contact method, the advantage lies in the
low-dimensional continuation architecture alone; if it fails in the narrow
regime, active pair identity carries real residual (Claim E evidence
either way).

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/ridge_continuation_scalar.py
"""
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, Q_START, GOAL, GOAL_RADIUS, R_EDGE)
from splatc.reference.oracle import certify_path_conservative

TABDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "tables")


def run_case_scalar(w, dy0=0.013, tilt_deg=7.0, robot_id="R_long_ellipse",
                    k_seed=8, return_geometry=False, door_plug=False):
    # return_geometry (P3.8): attach the station chain / dense polyline to
    # the result for chart-factored attachment.  ZERO billing change; the
    # canonical ablation table never sets it.  door_plug: pure scene
    # parameterization (P3.8 negative control).
    scene = make_g1_scene(w, door_offset=dy0, door_tilt=np.radians(tilt_deg),
                          door_plug=door_plug)
    robot = robot_library()[robot_id]
    M_SCALE = robot.b + R_EDGE
    nq = [0]
    ev = {"events": [], "sections": 0}

    def probe(q):
        nq[0] += 1
        _, rho = scene.eval_points(robot, q[2],
                                   np.array([q[0]]), np.array([q[1]]))
        return float(rho[0])

    # ---- seed grid (identical pitch/filters, scalar semantics) -----------
    cands = []
    for x in np.linspace(-3.0, 3.0, 25):
        for y in np.linspace(-1.8, 1.8, 17):
            for th in np.linspace(0, np.pi, 16, endpoint=False):
                m = probe((x, y, th))
                if m < 0.2 and m > -0.49:      # in-shell, not deep plateau
                    cands.append((m, x, y, th))
    cands.sort(key=lambda c: -c[0])
    c_seed_mark = nq[0]
    if not cands:
        return {"w": w, "status": "NO_SEED", "c_seed": nq[0]}

    def lateral_max(q, u_hat):
        # scalar analog of balance: ternary MAXIMIZE m along u (+-0.15m,
        # 12 iters ~ the 11-deep bisection's resolution)
        lo, hi = -0.15, 0.15
        for _ in range(12):
            t1, t2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
            m1 = probe((q[0] + t1 * u_hat[0], q[1] + t1 * u_hat[1], q[2]))
            m2 = probe((q[0] + t2 * u_hat[0], q[1] + t2 * u_hat[1], q[2]))
            if m1 < m2:
                lo = t1
            else:
                hi = t2
        t = 0.5 * (lo + hi)
        return np.array([q[0] + t * u_hat[0], q[1] + t * u_hat[1], q[2]])

    def theta_opt(q):
        lo0, hi0 = q[2] - 0.35, q[2] + 0.35
        ts = np.linspace(lo0, hi0, 15)
        ms = [probe((q[0], q[1], t)) for t in ts]
        runs, i0 = [], None
        for i, mv in enumerate(ms):
            if mv > 0 and i0 is None:
                i0 = i
            elif mv <= 0 and i0 is not None:
                runs.append((i0, i - 1))
                i0 = None
        if i0 is not None:
            runs.append((i0, len(ms) - 1))
        local = [r for r in runs if r[0] - 2 <= 7 <= r[1] + 2]
        if not local:
            return None, len(runs)
        k, best = None, -np.inf
        for a, b in local:
            for i in range(a, b + 1):
                if ms[i] > best:
                    best, k = ms[i], i
        lo = ts[max(0, k - 1)]
        hi = ts[min(len(ts) - 1, k + 1)]
        for _ in range(8):
            t1, t2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
            if probe((q[0], q[1], t1)) < probe((q[0], q[1], t2)):
                lo = t1
            else:
                hi = t2
        return np.array([q[0], q[1], 0.5 * (lo + hi)]), len(runs)

    def grad_m(q, eps=0.02):
        g = np.array([probe((q[0] + eps, q[1], q[2]))
                      - probe((q[0] - eps, q[1], q[2])),
                      probe((q[0], q[1] + eps, q[2]))
                      - probe((q[0], q[1] - eps, q[2]))]) / (2 * eps)
        n = np.linalg.norm(g)
        return g / n if n > 1e-9 else np.array([0.0, 1.0])

    def section_eval(q_anchor, u_hat):
        ev["sections"] += 1
        comps, cur = [], None
        for t in np.linspace(q_anchor[2] - 0.35, q_anchor[2] + 0.35, 15):
            qb = lateral_max(np.array([q_anchor[0], q_anchor[1], t]), u_hat)
            mv = probe(qb)
            if mv > 0:
                if cur is None:
                    cur = {"th_lo": t, "th_hi": t, "m_best": mv,
                           "q_best": qb.copy()}
                else:
                    cur["th_hi"] = t
                    if mv > cur["m_best"]:
                        cur["m_best"], cur["q_best"] = mv, qb.copy()
            elif cur is not None:
                comps.append(cur)
                cur = None
        if cur is not None:
            comps.append(cur)
        return comps

    conn_checks = [0]

    def polish_segment(a, b):
        a, b = np.asarray(a, float), np.asarray(b, float)
        seg = np.linalg.norm(b[:2] - a[:2])
        dd = (b[:2] - a[:2]) / max(seg, 1e-9)
        u_seg = np.array([-dd[1], dd[0]])
        out = []
        n_sub = max(1, int(seg / 0.02))
        for i in range(1, n_sub + 1):
            t = i / n_sub
            qm = a + t * (b - a)
            if 1 <= i < n_sub:
                qo, _ = theta_opt(lateral_max(qm, u_seg))
                if qo is not None:
                    qm = qo
            out.append(qm if i < n_sub else b.copy())
        return out

    def certified_connector(wps_anchor):
        chain = [np.asarray(wps_anchor[0], float)]
        for a, b in zip(wps_anchor[:-1], wps_anchor[1:]):
            chain += polish_segment(a, b)
        ok, mmin, _, checks = certify_path_conservative(
            scene, robot, np.array([tuple(float(v) for v in p)
                                    for p in chain]))
        conn_checks[0] += checks
        return ok, chain, checks, mmin

    def try_branch_switch(stations, st_m, target, u_hat):
        order = sorted(range(1, min(len(stations), 8) + 1),
                       key=lambda b: -(st_m[-b] if not np.isnan(st_m[-b])
                                       else -1e9))
        for back in order:
            q_m = stations[-back]
            comps_m = section_eval(q_m, u_hat)
            tgt = np.array(target["q_best"])
            cand = [[q_m, tgt]]
            sib = [c for c in comps_m
                   if not (c["th_hi"] < target["th_lo"]
                           or c["th_lo"] > target["th_hi"])]
            if sib:
                sc = max(sib, key=lambda c: c["m_best"])
                cand.insert(0, [q_m, np.array(sc["q_best"]), tgt])
            for wps in cand:
                ok, chain, checks, mmin = certified_connector(wps)
                ev["events"].append(
                    {"kind": "branch_switch" if ok else "conn_fail",
                     "back": back, "min_mm": round(mmin * 1000, 2)})
                if ok:
                    del stations[len(stations) - back + 1:]
                    del st_m[len(st_m) - back + 1:]
                    for p in chain[1:]:
                        stations.append(np.asarray(p, float))
                        st_m.append(np.nan)
                    st_m[-1] = target["m_best"]
                    return tgt.copy()
        return None

    def track(q0, t_dir0):
        t_dir = t_dir0.copy()
        q = q0.copy()
        stations = [q.copy()]
        st_m = [np.nan]
        lost, shrink, skip_secant = 0, 1.0, False
        for _ in range(800):
            m = probe(q)
            st_m[-1] = m
            if m >= 0.249 and abs(q[0]) > 0.7:
                return stations, "contact_lost"
            m_metric = max(m, 0.0) * M_SCALE
            step = float(np.clip(4.0 * max(m_metric, 0.001), 0.005, 0.12)) \
                * shrink
            if len(stations) >= 2 and not skip_secant:
                d = stations[-1][:2] - stations[-2][:2]
                n = np.linalg.norm(d)
                if n > 1e-9:
                    t_dir = d / n
            skip_secant = False
            q_pred = q.copy()
            q_pred[:2] = q[:2] + step * t_dir
            u_hat = np.array([-t_dir[1], t_dir[0]])
            q_new, n_runs = theta_opt(lateral_max(q_pred, u_hat))
            mv = probe(q_new) if q_new is not None else -1.0
            if q_new is None or mv <= 0:
                comps = section_eval(q_pred, u_hat)
                if comps:
                    target = max(comps, key=lambda c: c["m_best"])
                    q_sw = try_branch_switch(stations, st_m, target, u_hat)
                    if q_sw is not None:
                        lost, shrink = 0, 1.0
                        q = q_sw
                        skip_secant = True
                        continue
                lost += 1
                if lost >= 3:
                    return stations, "ridge_dead"
                shrink *= 0.4
                continue
            lost, shrink = 0, 1.0
            q = q_new
            stations.append(q.copy())
            st_m.append(mv)
        return stations, "loop_limit"

    # ---- seed selection (same trial/validate loop, scalar validation) ----
    # k_seed=8 is the frozen architecture constant; k_seed=64 is a
    # deliberately scalar-FAVORABLE extension (the scalar in-shell filter
    # admits every near-wall pose, so its m-descending head is flooded
    # with wall-face candidates — pair identity's seed selectivity is
    # exactly what is being ablated here)
    seed, t_dir = None, None
    for k, c in enumerate(cands[:k_seed]):
        cand = np.array([c[1], c[2], c[3]])
        g = grad_m(cand)
        qp, _ = theta_opt(lateral_max(cand, g))
        if qp is None:
            continue
        m = probe(qp)
        if m > 0 and m <= 0.22:                # free AND still in contact
            seed, t_dir = qp, np.array([-g[1], g[0]])
            break
    c_seed = nq[0]
    if seed is None:
        return {"w": w, "status": "NO_SEED", "c_seed": c_seed}
    stA, termA = track(seed, t_dir)
    stB, termB = track(seed, -t_dir)
    c_track = nq[0] - c_seed
    sS, sG = (stA, stB) if stA[-1][0] <= stB[-1][0] else (stB, stA)
    stations = list(reversed(sS)) + sG[1:]
    xs = [float(s[0]) for s in stations]
    base = {"w": w, "terminated": f"{termA}/{termB}",
            "c_seed": c_seed, "c_track": c_track,
            "n_sections": ev["sections"], "n_events": len(ev["events"]),
            "pair_ops": int(scene.pair_ops_total()),
            "x_range": [round(min(xs), 3), round(max(xs), 3)]}
    if not (min(xs) < -0.65 and max(xs) > 0.65):
        base["status"] = "UNKNOWN_TRACK_LOST"
        base["stations"] = len(stations)
        return base
    c0 = nq[0]
    dense = [stations[0]]
    for a, b in zip(stations[:-1], stations[1:]):
        dense += polish_segment(a, b)
    c_densify = nq[0] - c0
    wps = [tuple(Q_START), (Q_START[0], Q_START[1], float(dense[0][2]))]
    wps += [tuple(float(v) for v in s) for s in dense]
    wps += [(GOAL[0], GOAL[1], float(dense[-1][2]))]
    ok, mmin, depth, checks = certify_path_conservative(
        scene, robot, np.array(wps))
    base.update({
        "status": "CERTIFIED_REACHABLE" if ok else "CERT_FAIL",
        "c_densify": c_densify, "c_cert": checks,
        "c_connector": conn_checks[0],
        "total": c_seed + c_track + c_densify + checks + conn_checks[0],
        "stations": f"{len(sS)}+{len(sG) - 1}",
        "min_margin_mm": round(mmin * 1000, 2)})
    base["pair_ops"] = int(scene.pair_ops_total())
    if return_geometry:
        base["_geometry"] = {
            "stations": [[float(v) for v in s] for s in stations],
            "dense": [[float(v) for v in s] for s in dense]}
    return base


def main():
    rows = []
    for k_seed in (8, 64):
        for w in (0.49, 0.5025, 0.505, 0.51, 0.52, 0.54, 0.58):
            t0 = time.time()
            r = run_case_scalar(w, k_seed=k_seed)
            r["k_seed"] = k_seed
            r["seconds"] = round(time.time() - t0, 1)
            rows.append(r)
            print(r, flush=True)
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from prov import make_provenance
    out = {"provenance": make_provenance(
        __file__, "ABLATION: same-architecture scalar-clearance "
        "continuation (no pair identity / sides / temporal signature)"),
        "rows": rows}
    os.makedirs(TABDIR, exist_ok=True)
    with open(os.path.join(TABDIR, "ridge_scalar_ablation.json"), "w") as f:
        json.dump(out, f, indent=1,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print("-> results/tables/ridge_scalar_ablation.json")


if __name__ == "__main__":
    main()
