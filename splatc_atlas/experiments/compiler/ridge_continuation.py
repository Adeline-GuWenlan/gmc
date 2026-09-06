"""Contact-Ridge-Guided Continuation with Certified Path Output — v4.1
(P1a: side-tag-free bilateral grouping; P1b: branch-aware gate chart).

v4.1 changes (review round-3 P1b; v4.0 archived at
archive/ridge_continuation_v40_20260813.py):
  - Sections carry ALL sampled-free (n,theta) components (section_eval),
    recorded into diag["chart"] with triggers seed / multimodal (free
    coarse-scan hint at accepted stations) / reject / backtrack.
  - The m_esc/2 dead-end trim is REPLACED by the review's principled
    branch switch: on lobe death, walk back to a section whose MERGED
    component spans both lobes, build the explicit connector, certify it
    immediately (billed c_connector), splice on success, UNKNOWN on
    failure.  Events (lobe_death / branch_switch / connector_cert_fail /
    march_death) land in diag["events"].
  - Components are SAMPLED (K=15 frozen); certification lives on
    connectors + the final path (review answer #1).

v4 changes (review round-3 P1a; v3.1 archived at
archive/ridge_continuation_v31_20260813.py):
  - The probe no longer uses oracle wall-side tags: h+/h- come from
    bilateral_rho_tagfree (opposing contact-direction clustering, largest
    circular gap cold / reference-direction continuity along the march).
    Validation: bitwise match with side_rho on ALL free poses (309/309
    sweep stations + 2925/2925 wall-neighborhood grid poses,
    tagfree_validation.py); divergence confined to colliding poses where
    side_rho fills a -0.5 quick-collide marker and tag-free reports the
    exact PW value (strictly more informative).
  - TAGFREE=False restores the oracle-tag probe (declared ablation only).

v3 changes (HANDOFF step 2, unit separation):
  - march termination is ACTIVE-SET based (a side at RHO_CAP has no wall
    in its shell) instead of the 0.22 PW proxy;
  - step law runs in METERS via the sound in-shell factor b+R_edge;
  - balance() reports no-bracket as failure (None) instead of silently
    returning its input; bisection depth 11 for 1.25mm-slack cases;
  - certificates were already metric (unchanged).

v2 changes (post station-dump + experiments A/B/C, worklog 08-13; v1
archived at archive/ridge_continuation_v1_20260813.py):
  - C_seed: 250mm grid (exp A: coarsest pitch with FREE bilateral
    candidates at every open width; 6800 poses), argmax restricted to FREE
    candidates (m > 0).  v1's 500mm grid had ZERO free bilateral poses for
    w<=0.54 and its argmax admitted colliding candidates — the station-1
    deaths.
  - C_track: predictor-corrector marches BOTH directions along
    ±perp(grad B) from the seed; each march exits when bilateral contact is
    lost in open space (max(h+,h-) > 0.22 and |x| > 0.7).  v1 marched
    goal-ward only, leaving the start half of the corridor untracked and
    bridging it with a straight line through the jamb (the CERT_FAILs).
  - Assembly start -> [start-side march reversed] -> seed -> [goal-side
    march] -> goal, 2cm densify, bubble certificate: unchanged.
No door metadata is read anywhere; side grouping via generator tags remains
the DECLARED oracle-side-grouping ablation.  Termination/step thresholds
are still PW-dimensionless — unit separation is the next HANDOFF step,
deliberately not mixed into this change.

Station dump (zero extra billed probes) -> results/tables/
ridge_stations.json; DIAGNOSTIC-ONLY metric-margin profile (checker #2,
counted as c_diag, never billed into C_seed/C_track/C_cert) localizes any
certificate failure.  Plots via ridge_diag_plots.py.

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/ridge_continuation.py
"""
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, Q_START, GOAL, GOAL_RADIUS, R_EDGE)
from splatc.reference.oracle import certify_path_conservative, metric_margin
from splatc.common.se2 import wrap_diff

TABDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "tables")
TAGFREE = True   # False = oracle side-tag probe (declared ablation only)


def margin_profile(scene, robot, wps, ds=0.005):
    """DIAGNOSTIC ONLY (checker #2, not billed): metric margin sampled every
    ~ds along the linearly interpolated waypoint polyline (motion metric
    |dp| + a*|dtheta|, same as the certificate).  Returns the sample list,
    per-segment minima, and the global argmin."""
    samples = []          # (s, seg, x, y, th, margin_m)
    seg_min = []          # (seg, margin_m)
    s_acc = 0.0
    n_checks = 0
    for i, (p, qn) in enumerate(zip(wps[:-1], wps[1:])):
        dth = wrap_diff(qn[2] - p[2])
        L = np.hypot(qn[0] - p[0], qn[1] - p[1]) + robot.a * abs(dth)
        n = max(2, int(np.ceil(L / ds)) + 1)
        m_seg = np.inf
        for t in np.linspace(0.0, 1.0, n):
            pose = (p[0] + t * (qn[0] - p[0]),
                    p[1] + t * (qn[1] - p[1]),
                    p[2] + t * dth)
            m = metric_margin(scene, robot, pose)
            n_checks += 1
            m_seg = min(m_seg, m)
            samples.append((s_acc + t * L, i) + pose + (m,))
        seg_min.append((i, m_seg))
        s_acc += L
    k = int(np.argmin([s[-1] for s in samples]))
    argmin = {"s": samples[k][0], "seg": samples[k][1],
              "pose": list(samples[k][2:5]), "margin_m": samples[k][5]}
    return samples, seg_min, argmin, n_checks


def run_case(w, dy0=0.013, tilt_deg=7.0, robot_id="R_long_ellipse",
             door_plug=False):
    # robot_id is pure parameterization for P2 generalization; door_plug
    # is pure scene parameterization for the P3.8 negative control; every
    # frozen constant is unchanged (M_SCALE derives from the robot's own b)
    scene = make_g1_scene(w, door_offset=dy0, door_tilt=np.radians(tilt_deg),
                          door_plug=door_plug)
    robot = robot_library()[robot_id]
    M_SCALE = robot.b + R_EDGE   # sound PW->metric factor in-shell: h*(b+R)
    nq = [0]
    diag = {"w": w, "seed_top": [], "visits": [], "rejects": [],
            "balance_no_bracket": [], "stations": [], "terminated": None}

    # probe: tag-free bilateral clearance (P1a).  ref_state["chain"] is off
    # during the seed grid scan (each pose classified cold, like exp A) and
    # on during tracking/densify (temporal signature continuity).
    ref_state = {"cur": None, "chain": False}

    def probe(q):
        nq[0] += 1
        if not TAGFREE:                        # oracle-tag ablation path
            s = scene.side_rho(robot, q[2],
                               np.array([q[0]]), np.array([q[1]]))
            return float(s[1][0]), float(s[-1][0])
        r = ref_state["cur"] if ref_state["chain"] else None
        ha, hb, dirs = scene.bilateral_rho_tagfree(robot, q, r)
        if ref_state["chain"]:
            ref_state["cur"] = dirs
        return ha, hb

    # ---- C_seed: 250mm grid (exp A pitch); bilateral candidates ----------
    # Shallow collisions are admissible (exp B: corrector basin covers
    # them); the quick-collide plateau (m <= -0.49) carries no signal and
    # is excluded (v1's station-1 deaths).
    cands = []
    for x in np.linspace(-3.0, 3.0, 25):
        for y in np.linspace(-1.8, 1.8, 17):
            for th in np.linspace(0, np.pi, 16, endpoint=False):
                hp, hm = probe((x, y, th))
                m = min(hp, hm)
                if max(hp, hm) < 0.2 and m > -0.49:
                    cands.append((m, hp, hm, x, y, th))
    cands.sort(key=lambda c: -c[0])
    diag["seed_top"] = [list(c) for c in cands[:10]]
    if not cands:
        c_seed = nq[0]
        diag["terminated"] = "NO_SEED"
        return {"w": w, "status": "NO_SEED", "c_seed": c_seed}, diag

    def balance(q, u_hat):
        # bisection on lateral offset to zero B = h+ - h-.  v3: no-bracket
        # is a FAILURE signal (None), not a silent identity (the v1 station-1
        # deaths hid behind that); 11 iterations fix the balanced-point
        # lateral resolution at 0.15mm METRIC-equivalent scale, needed once
        # delta reaches 1.25mm.
        def B(t):
            hp, hm = probe((q[0] + t * u_hat[0], q[1] + t * u_hat[1], q[2]))
            return hp - hm
        lo, hi = -0.15, 0.15
        blo, bhi = B(lo), B(hi)
        if blo * bhi > 0:
            diag["balance_no_bracket"].append(
                {"visit_i": len(diag["visits"]),
                 "q": [float(v) for v in q], "B_lo": blo, "B_hi": bhi})
            return None                        # caller must handle
        for _ in range(11):
            mid = 0.5 * (lo + hi)
            bm = B(mid)
            if blo * bm <= 0:
                hi, bhi = mid, bm
            else:
                lo, blo = mid, bm
        t = 0.5 * (lo + hi)
        return np.array([q[0] + t * u_hat[0], q[1] + t * u_hat[1], q[2]])

    def theta_opt(q):
        # maximize min(h+,h-) over theta.  Coarse scan first: near the
        # mouth the free theta window is a narrow spike (±3.7° at w=0.505)
        # in a collision plateau — plain ternary assumes unimodality and
        # collapsed onto the wrong lobe (dump 08-13, v2.1 reject #1).
        # P1b: also returns the number of feasible runs seen by the coarse
        # scan (free multi-lobe hint; >1 triggers a full section record).
        lo0, hi0 = q[2] - 0.35, q[2] + 0.35
        ts = np.linspace(lo0, hi0, 15)          # 2.9° spacing < window
        ms = [min(*probe((q[0], q[1], t))) for t in ts]
        runs, i0 = [], None
        for i, mv in enumerate(ms):
            if mv > 0 and i0 is None:
                i0 = i
            elif mv <= 0 and i0 is not None:
                runs.append((i0, i - 1))
                i0 = None
        if i0 is not None:
            runs.append((i0, len(ms) - 1))
        # P1b: LOBE-LOCAL corrector — only runs within 2 scan steps of the
        # incoming theta (scan center, index 7) are eligible; a global
        # argmax was silently jumping lobes (v4.1 first run: w=0.505 back
        # to CERT_FAIL with 0 events).  Crossing lobes is exclusively the
        # certified branch-switch protocol's job.
        local = [r for r in runs if r[0] - 2 <= 7 <= r[1] + 2]
        if not local:
            return None, len(runs)              # corrector fails -> reject
        k, best_m = None, -np.inf
        for a, b in local:
            for i in range(a, b + 1):
                if ms[i] > best_m:
                    best_m, k = ms[i], i
        lo = ts[max(0, k - 1)]
        hi = ts[min(len(ts) - 1, k + 1)]
        for _ in range(8):                      # ternary refine in bracket
            t1, t2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
            m1 = min(*probe((q[0], q[1], t1)))
            m2 = min(*probe((q[0], q[1], t2)))
            if m1 < m2:
                lo = t1
            else:
                hi = t2
        return np.array([q[0], q[1], 0.5 * (lo + hi)]), len(runs)

    def grad_B_xy(q, eps=0.02):
        hp1, hm1 = probe((q[0] + eps, q[1], q[2]))
        hp2, hm2 = probe((q[0] - eps, q[1], q[2]))
        hp3, hm3 = probe((q[0], q[1] + eps, q[2]))
        hp4, hm4 = probe((q[0], q[1] - eps, q[2]))
        g = np.array([(hp1 - hm1) - (hp2 - hm2),
                      (hp3 - hm3) - (hp4 - hm4)]) / (2 * eps)
        n = np.linalg.norm(g)
        return g / n if n > 1e-9 else np.array([0.0, 1.0])

    # ---- C_track: predictor-corrector BOTH ways along the ridge ----------
    # P1b (review round-3 §3/§9.2): sections carry ALL sampled-free
    # components, not one argmax; the mouth double lobe becomes chart
    # structure (v2.2 finding: sequential balance->theta has a spurious
    # fixed point on the blocked-lobe medial axis).  Components are
    # SAMPLED (K=15 balanced thetas, frozen constants) — certification
    # lives on connectors and the final path, per review answer #1.
    TH_TOL = 0.7 / 14.0                        # scan spacing (derived)

    def section_eval(q_anchor, u_hat):
        pts = []
        for t in np.linspace(q_anchor[2] - 0.35, q_anchor[2] + 0.35, 15):
            qb = balance(np.array([q_anchor[0], q_anchor[1], t]), u_hat)
            if qb is None:
                pts.append((t, None, -1.0))
                continue
            hp2, hm2 = probe(qb)
            pts.append((t, qb, min(hp2, hm2)))
        comps, cur = [], None
        for t, qb, mv in pts:
            if qb is not None and mv > 0:
                if cur is None:
                    cur = {"th_lo": t, "th_hi": t, "m_best": mv,
                           "m_min": mv, "n_pts": 1,
                           "q_best": qb.copy()}
                else:
                    cur["th_hi"] = t
                    cur["n_pts"] += 1
                    cur["m_min"] = min(cur["m_min"], mv)
                    if mv > cur["m_best"]:
                        cur["m_best"], cur["q_best"] = mv, qb.copy()
            elif cur is not None:
                comps.append(cur)
                cur = None
        if cur is not None:
            comps.append(cur)
        return comps

    def record_section(march, station, q_anchor, comps, trigger):
        diag.setdefault("chart", []).append(
            {"march": march, "station": station, "trigger": trigger,
             "x": float(q_anchor[0]), "y": float(q_anchor[1]),
             "anchor_th": float(q_anchor[2]),
             "components": [
                 {"th_lo": float(c["th_lo"]), "th_hi": float(c["th_hi"]),
                  "m_best": float(c["m_best"]), "m_min": float(c["m_min"]),
                  "n_pts": int(c["n_pts"]),
                  "q_best": [float(v) for v in c["q_best"]]}
                 for c in comps]})

    def overlaps(c1, c2, tol=TH_TOL):
        return not (c1["th_hi"] + tol < c2["th_lo"]
                    or c1["th_lo"] - tol > c2["th_hi"])

    conn_checks = [0]                          # connector certificate cost

    def polish_segment(a, b):
        """2cm-spaced interpolation with corrector polish — the standard
        witness densification, shared by the final witness assembly and
        connector candidates (a raw chord across a thin section neck is
        NOT free; the polished polyline follows the neck — this is what
        made v3.1's spliced jumps certifiable)."""
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
                qb2 = balance(qm, u_seg)
                if qb2 is not None:
                    qo, _ = theta_opt(qb2)
                    if qo is not None:
                        qm = qo
        # keep endpoint exact
            out.append(qm if i < n_sub else b.copy())
        return out

    def certified_connector(wps_anchor):
        """Polish a candidate connector (anchor pose list) at witness
        density, certify it immediately.  Returns (ok, polished, checks,
        min_mm)."""
        chain = [np.asarray(wps_anchor[0], float)]
        for a, b in zip(wps_anchor[:-1], wps_anchor[1:]):
            chain += polish_segment(a, b)
        ok, mmin, _, checks = certify_path_conservative(
            scene, robot, np.array([tuple(float(v) for v in p)
                                    for p in chain]))
        conn_checks[0] += checks
        return ok, chain, checks, mmin

    def try_branch_switch(stations, st_m, march, target, u_hat):
        """Review round-3 answer #2, replacing the m_esc/2 tail trim:
        propose explicit connectors to the target lobe from recent
        stations — HEALTHIEST ANCHOR FIRST ("return to the last healthy
        certified component", operationalized as descending recorded
        station margin; jumping from the dying crawl tail certifies but
        hugs the section neck at ~0mm — v4.1c dump).  Candidates go via
        the sibling component of the anchor's section when present, else
        direct; each is polished at witness density and certified
        immediately.  First certified connector wins: splice (drop the
        tail beyond the anchor).  All fail -> UNKNOWN."""
        order = sorted(range(1, min(len(stations), 8) + 1),
                       key=lambda b: -(st_m[-b] if not np.isnan(st_m[-b])
                                       else -1e9))
        for back in order:
            q_m = stations[-back]
            comps_m = section_eval(q_m, u_hat)
            record_section(march, len(stations) - back, q_m, comps_m,
                           "backtrack")
            tgt = np.array(target["q_best"])
            cand_wps = []
            sib = [c for c in comps_m if overlaps(c, target)]
            if sib:
                sc = max(sib, key=lambda c: c["m_best"])
                cand_wps.append([q_m, np.array(sc["q_best"]), tgt])
            cand_wps.append([q_m, tgt])
            for wps in cand_wps:
                ok, chain, checks, mmin = certified_connector(wps)
                diag.setdefault("events", []).append(
                    {"kind": ("branch_switch" if ok
                              else "connector_cert_fail"),
                     "march": march, "back": back, "checks": checks,
                     "n_wps": len(wps),
                     "conn_min_mm": round(mmin * 1000, 2),
                     "from_th": float(q_m[2]),
                     "to_interval": [float(target["th_lo"]),
                                     float(target["th_hi"])]})
                if ok:
                    del stations[len(stations) - back + 1:]
                    del st_m[len(st_m) - back + 1:]
                    for p in chain[1:]:
                        stations.append(np.asarray(p, float))
                        st_m.append(np.nan)
                    st_m[-1] = target["m_best"]
                    return tgt.copy()
        return None

    def track(q0, t_dir0, march):
        t_dir = t_dir0.copy()
        q = q0.copy()
        stations = [q.copy()]
        st_m = [np.nan]                        # per-station margin record
        lost = 0
        shrink = 1.0
        skip_secant = False
        # iteration cap must cover march length / step floor (~2.5m / 5mm
        # = 500 at delta<=1.25mm; the 200 cap silently truncated the
        # gamma-delta 1.25mm march mid-corridor as "loop_limit")
        for _ in range(800):
            hp, hm = probe(q)
            m = min(hp, hm)
            st_m[-1] = m
            visit = {"march": march, "station": len(stations) - 1,
                     "retry": lost, "x": float(q[0]), "y": float(q[1]),
                     "th": float(q[2]), "hp": hp, "hm": hm,
                     "B": hp - hm, "m": m, "step": None}
            diag["visits"].append(visit)
            # v3 termination: ACTIVE-SET semantics — a side at RHO_CAP has
            # no wall within its broad-phase shell, i.e. bilateral contact
            # is truly gone (the 0.22 PW proxy is retired; 0.3 > cap was
            # the unreachable-threshold bug)
            if (hp >= 0.249 or hm >= 0.249) and abs(q[0]) > 0.7:
                return stations, "bilateral_lost"
            # v3 step law in METERS: sound PW->metric factor b+R_edge
            m_metric = max(m, 0.0) * M_SCALE
            step = float(np.clip(4.0 * max(m_metric, 0.001), 0.005, 0.12)) \
                * shrink
            visit["step"] = step
            if len(stations) >= 2 and not skip_secant:   # secant predictor
                d = stations[-1][:2] - stations[-2][:2]
                n = np.linalg.norm(d)
                if n > 1e-9:
                    t_dir = d / n
            skip_secant = False
            visit["t_dir"] = [float(t_dir[0]), float(t_dir[1])]
            q_pred = q.copy()
            q_pred[:2] = q[:2] + step * t_dir
            u_hat = np.array([-t_dir[1], t_dir[0]])
            qb = balance(q_pred, u_hat)
            q_new, n_runs = (theta_opt(qb) if qb is not None
                             else (None, 0))
            hp, hm = probe(q_new) if q_new is not None else (-1.0, -1.0)
            if q_new is None or min(hp, hm) <= 0:
                # sequential (lobe-local) corrector failed: evaluate the
                # FULL section (all sampled-free components, not argmax)
                comps = section_eval(q_pred, u_hat)
                record_section(march, len(stations) - 1, q_pred, comps,
                               "reject")
                if comps:
                    # ANY section-derived jump carries a certificate at
                    # accept time: a sampled-connected component can hide
                    # a thin neck whose straight chord is NOT free
                    # (w=0.505 dump: single comp 163..189°, neck
                    # m_min=0.0017; the uncertified chord cost -1.08mm)
                    cont = [c for c in comps
                            if c["th_lo"] - TH_TOL <= q[2]
                            <= c["th_hi"] + TH_TOL]
                    if not cont:
                        diag.setdefault("events", []).append(
                            {"kind": "lobe_death", "march": march,
                             "station": len(stations) - 1,
                             "th": float(q[2])})
                    target = max(comps, key=lambda c: c["m_best"])
                    q_sw = try_branch_switch(stations, st_m, march,
                                             target, u_hat)
                    if q_sw is not None:
                        lost, shrink = 0, 1.0
                        q = q_sw
                        skip_secant = True
                        continue
                else:
                    diag.setdefault("events", []).append(
                        {"kind": "section_blocked", "march": march,
                         "station": len(stations) - 1, "th": float(q[2])})
                # no certified way across: shrink-retry, then honest dead
                lost += 1
                diag["rejects"].append(
                    {"visit_i": len(diag["visits"]), "march": march,
                     "retry": lost,
                     "q_new": (None if q_new is None else
                               [float(v) for v in q_new]),
                     "hp": hp, "hm": hm})
                if lost >= 3:
                    diag.setdefault("events", []).append(
                        {"kind": "march_death", "march": march,
                         "station": len(stations) - 1})
                    return stations, "ridge_dead"
                shrink *= 0.4
                continue
            lost = 0
            shrink = 1.0
            q = q_new
            stations.append(q.copy())
            st_m.append(min(hp, hm))
            if n_runs > 1:
                # free multi-lobe hint from the corrector's own coarse
                # scan: chart the section at this accepted station
                comps = section_eval(q, u_hat)
                record_section(march, len(stations) - 1, q, comps,
                               "multimodal")
        return stations, "loop_limit"

    # ---- seed selection: polish candidates in m-descending order until
    # one VALIDATES as genuine bilateral contact.  A polished seed whose
    # sides read the cap (max > 0.22) is a wall-face pose, not a gap pose —
    # the march termination would be instantly true (dump 08-13 v2 run).
    seed, t_dir = None, None
    for k, c in enumerate(cands[:8]):
        cand = np.array([c[3], c[4], c[5]])
        ref_state["chain"], ref_state["cur"] = True, None  # fresh chain
        g = grad_B_xy(cand)
        qb = balance(cand, g)
        if qb is None:                         # no B bracket: not a gap pose
            diag.setdefault("seed_rejected", []).append(
                {"cand": [float(v) for v in cand], "m_cand": c[0],
                 "reason": "no_bracket"})
            continue
        qp, _ = theta_opt(qb)
        if qp is None:                         # no feasible local run
            diag.setdefault("seed_rejected", []).append(
                {"cand": [float(v) for v in cand], "m_cand": c[0],
                 "reason": "no_local_run"})
            continue
        hp, hm = probe(qp)
        if min(hp, hm) > 0 and max(hp, hm) <= 0.22:
            seed, t_dir = qp, np.array([-g[1], g[0]])  # perp(grad B)
            diag["seed_grid"] = [float(v) for v in cand] + [c[0]]
            diag["seed_tries"] = k + 1
            break
        diag.setdefault("seed_rejected", []).append(
            {"cand": [float(v) for v in cand], "m_cand": c[0],
             "polished": [float(v) for v in qp], "hp": hp, "hm": hm})
    c_seed = nq[0]
    if seed is None:
        diag["terminated"] = "NO_SEED"
        return {"w": w, "status": "NO_SEED", "c_seed": c_seed}, diag
    diag["seed_polished"] = [float(v) for v in seed]
    record_section(-1, 0, seed, section_eval(seed, g), "seed")
    stA, termA = track(seed, t_dir, 0)
    stB, termB = track(seed, -t_dir, 1)
    diag["terminated"] = f"{termA}/{termB}"
    c_track = nq[0] - c_seed
    # orient: the march ending at smaller x is the start-side chain
    sS, sG = (stA, stB) if stA[-1][0] <= stB[-1][0] else (stB, stA)
    stations = list(reversed(sS)) + sG[1:]     # sS[0] == sG[0] == seed
    diag["stations"] = [[float(v) for v in s] for s in stations]
    xs = [float(s[0]) for s in stations]

    if not (min(xs) < -0.65 and max(xs) > 0.65):   # never spanned the wall
        wps = [tuple(Q_START), (Q_START[0], Q_START[1], stations[0][2])]
        wps += [tuple(float(v) for v in s) for s in stations]
        # billing boundary (round-6): snapshot BEFORE the diagnostic
        # margin_profile so pair_ops_core excludes diag work; the plain
        # pair_ops field keeps its historical meaning (incl. diag)
        po_core = int(scene.pair_ops_total())
        bp_core = int(scene.bp_hits_total())
        samples, seg_min, argmin, c_diag = margin_profile(scene, robot, wps)
        diag["profile"] = {"kind": "prefix_only(track_lost)",
                           "wps": [list(p) for p in wps],
                           "samples": samples, "seg_min": seg_min,
                           "argmin": argmin, "c_diag": c_diag}
        return {"w": w, "status": "UNKNOWN_TRACK_LOST",
                "c_seed": c_seed, "c_track": c_track,
                "c_connector": conn_checks[0],
                "stations": len(stations),
                "pair_ops_core": po_core, "bp_hits_core": bp_core,
                "pair_ops": int(scene.pair_ops_total()),
                "bp_hits": int(scene.bp_hits_total()),
                "n_sections": len(diag.get("chart", [])),
                "n_events": len(diag.get("events", [])),
                "x_range": [round(min(xs), 3), round(max(xs), 3)]}, diag

    # ---- densify the witness at certificate density (C_cert side):
    # sub-2cm spacing with corrector polish keeps the POLYLINE inside the
    # low-margin corridor (tracker steps stay delta-free; review §11)
    c0 = nq[0]
    dense = [stations[0]]
    for a, b in zip(stations[:-1], stations[1:]):
        dense += polish_segment(a, b)
    c_densify = nq[0] - c0

    # ---- C_cert: bubble certificate over the assembled witness -----------
    wps = [tuple(Q_START), (Q_START[0], Q_START[1], float(dense[0][2]))]
    wps += [tuple(float(v) for v in s) for s in dense]
    wps += [(GOAL[0], GOAL[1], float(dense[-1][2]))]
    # billing boundary (round-6) — three chronological capture points:
    #   core = seed+track+densify (incl. connector certification, which
    #          happens inline during tracking)
    #   cert = core + final conservative path certificate
    #   pair_ops (historical field) = cert + diagnostic margin_profile
    po_core = int(scene.pair_ops_total())
    bp_core = int(scene.bp_hits_total())
    ok, mmin, depth, checks = certify_path_conservative(
        scene, robot, np.array(wps))
    po_cert = int(scene.pair_ops_total())
    bp_cert = int(scene.bp_hits_total())
    samples, seg_min, argmin, c_diag = margin_profile(scene, robot, wps)
    diag["profile"] = {"kind": "certificate_polyline",
                       "wps": [list(p) for p in wps],
                       "samples": samples, "seg_min": seg_min,
                       "argmin": argmin, "c_diag": c_diag}
    return {"w": w, "status": "CERTIFIED_REACHABLE" if ok else "CERT_FAIL",
            "c_seed": c_seed, "c_track": c_track,
            "c_densify": c_densify, "c_cert": checks,
            "c_connector": conn_checks[0],
            "total": c_seed + c_track + c_densify + checks + conn_checks[0],
            "stations": f"{len(sS)}+{len(sG) - 1}",
            "pair_ops_core": po_core, "bp_hits_core": bp_core,
            "pair_ops_cert": po_cert, "bp_hits_cert": bp_cert,
            "pair_ops": int(scene.pair_ops_total()),
            "bp_hits": int(scene.bp_hits_total()),
            "n_sections": len(diag.get("chart", [])),
            "n_events": len(diag.get("events", [])),
            "x_range": [round(min(xs), 3), round(max(xs), 3)],
            "min_margin_mm": round(mmin * 1000, 2)}, diag


def main():
    rows, diags = [], {}
    for w in (0.49, 0.505, 0.51, 0.52, 0.54, 0.58):
        t0 = time.time()
        r, d = run_case(w)
        r["seconds"] = round(time.time() - t0, 1)
        rows.append(r)
        diags[f"{w:.3f}"] = d
        print(r, flush=True)
    os.makedirs(TABDIR, exist_ok=True)
    from prov import make_provenance
    # round-7: the old arm label claimed a geometry-independence property
    # the report had already withdrawn (G1-frame exit/span constants are
    # geometric priors); provenance must say what the code does
    prov = make_provenance(
        __file__, "continuation-v4.1 ("
        + ("tag-free G1 contact grouping" if TAGFREE
           else "ORACLE side grouping — ablation")
        + "; G1-frame-conditioned exit/span logic)")
    with open(os.path.join(TABDIR, "ridge_continuation.json"), "w") as f:
        json.dump({"provenance": prov, "rows": rows}, f, indent=2)
    diags["_provenance"] = prov
    with open(os.path.join(TABDIR, "ridge_stations.json"), "w") as f:
        json.dump(diags, f)
    print("station dump -> results/tables/ridge_stations.json")


if __name__ == "__main__":
    main()
