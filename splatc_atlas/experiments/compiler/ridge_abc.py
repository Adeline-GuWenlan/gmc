"""Experiments A/B/C for ridge-continuation failure isolation (review round-2
§14 + HANDOFF step 2).  ALL of this file is DIAGNOSTIC: experiments B and C
read door_tilt/door_offset from scene.meta to build oracle axis poses, and
every such use is labeled "oracle" in the output.  Nothing here is the
method; results gate the redesign of ridge_continuation.py.

  A  seed audit: replicate the coarse seed grid, then refine it 2x/4x, and
     count FREE bilateral candidates per refinement (grid-seeding cost
     ladder; per-x-slab table for the coarse grid).
  B  corrector recovery: perturb oracle on-axis poses by known lateral /
     angular offsets, run corrector variants (single pass, iterated,
     2x2 KKT Newton), measure recovery basin.
  C  oracle-seed tracker: seed at the funnel mouth (oracle), track BOTH
     directions with the unmodified tracker loop, assemble
     start->...->goal, densify, certify.  Isolates "everything downstream
     of seeding".

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/ridge_abc.py
"""
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, Q_START, GOAL, GOAL_RADIUS)
from splatc.reference.oracle import certify_path_conservative
from ridge_continuation import margin_profile

TABDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "tables")
DY0, TILT_DEG = 0.013, 7.0


def build(w):
    scene = make_g1_scene(w, door_offset=DY0, door_tilt=np.radians(TILT_DEG))
    robot = robot_library()["R_long_ellipse"]
    return scene, robot


def make_probe(scene, robot, nq):
    def probe(q):
        s = scene.side_rho(robot, q[2], np.array([q[0]]), np.array([q[1]]))
        nq[0] += 1
        return float(s[1][0]), float(s[-1][0])
    return probe


def axis_frame(scene):
    """ORACLE (declared): door axis direction/normal from scene metadata."""
    tilt = scene.meta["door_tilt"]
    off = scene.meta["door_offset"]
    d = np.array([np.cos(tilt), np.sin(tilt)])
    n = np.array([-np.sin(tilt), np.cos(tilt)])
    return d, n, off, tilt


def axis_pose(scene, t):
    d, n, off, tilt = axis_frame(scene)
    return np.array([t * d[0], off + t * d[1], tilt])


# ---------------------------------------------------------------- A ----
def exp_A(w, factors=(1, 2, 4)):
    scene, robot = build(w)
    out = {"w": w, "ladder": [], "slab_table": []}
    for f in factors:
        xs = np.linspace(-3.0, 3.0, 12 * f + 1)
        ys = np.linspace(-1.8, 1.8, 8 * f + 1)
        ths = np.linspace(0, np.pi, 8 * f, endpoint=False)
        XX, YY = np.meshgrid(xs, ys, indexing="ij")
        X, Y = XX.ravel(), YY.ravel()
        n_probes = 0
        n_cand = n_free = 0
        best = None, -np.inf
        slabs = {}
        for th in ths:
            s = scene.side_rho(robot, th, X, Y)
            hp, hm = s[1], s[-1]
            n_probes += X.size
            m = np.minimum(hp, hm)
            bilat = np.maximum(hp, hm) < 0.2
            n_cand += int(bilat.sum())
            n_free += int((bilat & (m > 0)).sum())
            for i in np.flatnonzero(bilat):
                key = round(float(X[i]), 3)
                rec = (float(m[i]), float(X[i]), float(Y[i]), float(th),
                       float(hp[i]), float(hm[i]))
                if key not in slabs or rec[0] > slabs[key][0]:
                    slabs[key] = rec
                if rec[0] > best[1]:
                    best = rec, rec[0]
        out["ladder"].append(
            {"factor": f, "pitch_mm": round(500.0 / f, 1),
             "n_probes": n_probes, "n_bilateral": n_cand,
             "n_free_bilateral": n_free,
             "best": list(best[0]) if best[0] else None})
        if f == 1:
            out["slab_table"] = [
                {"x": k, "best_m": round(v[0], 4), "y": v[2],
                 "th_deg": round(np.degrees(v[3]), 1),
                 "hp": round(v[4], 4), "hm": round(v[5], 4)}
                for k, v in sorted(slabs.items())]
    return out


# ---------------------------------------------------------------- B ----
def correctors(scene, robot):
    nq = [0]
    probe = make_probe(scene, robot, nq)

    def balance(q, u_hat):
        def B(t):
            hp, hm = probe((q[0] + t * u_hat[0], q[1] + t * u_hat[1], q[2]))
            return hp - hm
        lo, hi = -0.15, 0.15
        blo, bhi = B(lo), B(hi)
        if blo * bhi > 0:
            return q, False
        for _ in range(9):
            mid = 0.5 * (lo + hi)
            bm = B(mid)
            if blo * bm <= 0:
                hi, bhi = mid, bm
            else:
                lo, blo = mid, bm
        t = 0.5 * (lo + hi)
        return np.array([q[0] + t * u_hat[0], q[1] + t * u_hat[1], q[2]]), True

    def theta_opt(q):
        lo, hi = q[2] - 0.35, q[2] + 0.35
        for _ in range(12):
            t1, t2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
            m1 = min(*probe((q[0], q[1], t1)))
            m2 = min(*probe((q[0], q[1], t2)))
            if m1 < m2:
                lo = t1
            else:
                hi = t2
        return np.array([q[0], q[1], 0.5 * (lo + hi)])

    def single(q, u_hat):
        q1, _ = balance(q, u_hat)
        return theta_opt(q1)

    def iterated(q, u_hat, rounds=5):
        for _ in range(rounds):
            q1, _ = balance(q, u_hat)
            q1 = theta_opt(q1)
            if np.linalg.norm(q1 - q) < 1e-4:
                return q1
            q = q1
        return q

    def newton_kkt(q, u_hat, iters=6, e=0.01):
        # 2x2 damped Newton on residual r(n,th) = (B, dm/dth)
        def resid(qq):
            hp, hm = probe(qq)
            mp = min(*probe((qq[0], qq[1], qq[2] + e)))
            mm = min(*probe((qq[0], qq[1], qq[2] - e)))
            return np.array([hp - hm, (mp - mm) / (2 * e)]), min(hp, hm)
        for _ in range(iters):
            r0, m0 = resid(q)
            if m0 <= -0.49:                    # quick-collide plateau: no info
                return q
            if abs(r0[0]) < 1e-3 and abs(r0[1]) < 1e-2:
                return q
            h = 0.008
            qn = np.array([q[0] + h * u_hat[0], q[1] + h * u_hat[1], q[2]])
            qt = np.array([q[0], q[1], q[2] + h])
            rn, _ = resid(qn)
            rt, _ = resid(qt)
            J = np.column_stack([(rn - r0) / h, (rt - r0) / h])
            try:
                du = np.linalg.solve(J, -r0)
            except np.linalg.LinAlgError:
                return q
            du = np.clip(du, -0.05, 0.05)
            q = np.array([q[0] + du[0] * u_hat[0], q[1] + du[0] * u_hat[1],
                          q[2] + du[1]])
        return q
    return {"single": single, "iterated": iterated,
            "newton_kkt": newton_kkt}, probe, nq


def exp_B(w):
    scene, robot = build(w)
    d, n_hat, off, tilt = axis_frame(scene)  # oracle frame (declared)
    corr, probe, nq = correctors(scene, robot)
    lat_mm = [5, 10, 20, 50, 100]
    ang_deg = [1, 3, 5, 10, 15]
    joint = [(10, 3), (20, 5), (50, 10)]
    trials = ([(s * l / 1000.0, 0.0) for l in lat_mm for s in (1, -1)]
              + [(0.0, s * np.radians(a)) for a in ang_deg for s in (1, -1)]
              + [(s1 * l / 1000.0, s2 * np.radians(a))
                 for (l, a) in joint for s1 in (1, -1) for s2 in (1, -1)])
    rows = []
    for t_ax in (-0.4, 0.0, 0.4):
        q0 = axis_pose(scene, t_ax)            # oracle on-axis pose
        for (dn, dth) in trials:
            qp = np.array([q0[0] + dn * n_hat[0], q0[1] + dn * n_hat[1],
                           q0[2] + dth])
            hp0, hm0 = probe(qp)
            for name, fn in corr.items():
                c0 = nq[0]
                qf = fn(qp.copy(), n_hat.copy())
                hpf, hmf = probe(qf)
                mf = min(hpf, hmf)
                rows.append({
                    "t_axis": t_ax, "dn_mm": round(dn * 1000, 1),
                    "dth_deg": round(np.degrees(dth), 1),
                    "m_start": round(min(hp0, hm0), 4), "corr": name,
                    "m_final": round(mf, 4), "recovered": bool(mf > 0),
                    "lat_err_mm": round(float(
                        np.dot(qf[:2] - q0[:2], n_hat)) * 1000, 2),
                    "th_err_deg": round(np.degrees(qf[2] - q0[2]), 2),
                    "queries": nq[0] - c0})
    return {"w": w, "oracle_axis": True, "rows": rows}


# ---------------------------------------------------------------- C ----
def exp_C(w, t_seed=-0.65):
    scene, robot = build(w)
    d, n_hat, off, tilt = axis_frame(scene)  # oracle frame (declared)
    corr, probe, nq = correctors(scene, robot)
    balance = corr["single"]                   # single pass = tracker corrector

    def track(q_seed, direction, x_stop):
        """Unmodified tracker loop (step law, secant, retry, 0.22 exit),
        parameterized only by march direction and exit side."""
        t_dir = direction.copy()
        q = q_seed.copy()
        stations = [q.copy()]
        lost = 0
        for _ in range(200):
            hp, hm = probe(q)
            m = min(hp, hm)
            if max(hp, hm) > 0.22 and (q[0] - x_stop) * direction[0] > 0:
                break
            step = float(np.clip(4.0 * max(m, 0.002), 0.02, 0.12))
            if len(stations) >= 2:
                dd = stations[-1][:2] - stations[-2][:2]
                nn = np.linalg.norm(dd)
                if nn > 1e-9:
                    t_dir = dd / nn
            q_pred = q.copy()
            q_pred[:2] = q[:2] + step * t_dir
            u_hat = np.array([-t_dir[1], t_dir[0]])
            q_new = balance(q_pred, u_hat)
            hp, hm = probe(q_new)
            if min(hp, hm) <= 0:
                lost += 1
                if lost >= 3:
                    break
                q[:2] = q[:2] + 0.4 * step * t_dir
                continue
            lost = 0
            q = q_new
            stations.append(q.copy())
        return stations

    c0 = nq[0]
    seed = axis_pose(scene, t_seed)            # ORACLE seed at funnel mouth
    seed = balance(seed, n_hat.copy())
    c_seed = nq[0] - c0
    fwd = track(seed, d.copy(), 0.7)           # goal-ward
    bwd = track(seed, -d.copy(), -0.7)         # start-ward
    c_track = nq[0] - c0 - c_seed
    stations = list(reversed(bwd[1:])) + fwd   # bwd[0] == fwd[0] == seed
    xr = [float(s[0]) for s in stations]
    crossed = bool(min(xr) < -0.65 and max(xr) > 0.65)

    # densify (same as ridge_continuation) + certify
    c1 = nq[0]
    dense = [stations[0]]
    for a, b in zip(stations[:-1], stations[1:]):
        seg = np.linalg.norm(b[:2] - a[:2])
        dd = (b[:2] - a[:2]) / max(seg, 1e-9)
        u_seg = np.array([-dd[1], dd[0]])
        n_sub = max(1, int(seg / 0.02))
        for i in range(1, n_sub + 1):
            t = i / n_sub
            qm = a + t * (b - a)
            if 1 <= i < n_sub:
                qm = balance(qm, u_seg)
            dense.append(qm)
    c_densify = nq[0] - c1
    wps = [tuple(Q_START), (Q_START[0], Q_START[1], float(dense[0][2]))]
    wps += [tuple(float(v) for v in s) for s in dense]
    wps += [(GOAL[0], GOAL[1], float(dense[-1][2]))]
    ok, mmin, depth, checks = certify_path_conservative(
        scene, robot, np.array(wps))
    samples, seg_min, argmin, c_diag = margin_profile(scene, robot, wps)
    return {"w": w, "oracle_seed": True, "t_seed": t_seed,
            "status": "CERTIFIED_REACHABLE" if ok else
            ("CERT_FAIL" if crossed else "TRACK_LOST"),
            "crossed_door": crossed,
            "stations_bwd": len(bwd), "stations_fwd": len(fwd),
            "x_range": [round(min(xr), 3), round(max(xr), 3)],
            "c_seed": c_seed, "c_track": c_track, "c_densify": c_densify,
            "c_cert": checks, "total": c_seed + c_track + c_densify + checks,
            "min_margin_mm": round(mmin * 1000, 2),
            "profile_min_mm": round(argmin["margin_m"] * 1000, 2),
            "profile_argmin_seg": argmin["seg"],
            "stations": [[float(v) for v in s] for s in stations],
            "wps_n": len(wps)}


def main():
    res = {"A": [], "B": [], "C": []}
    for w in (0.49, 0.505, 0.51, 0.52, 0.54, 0.58):
        t0 = time.time()
        a = exp_A(w)
        a["seconds"] = round(time.time() - t0, 1)
        res["A"].append(a)
        print(f"A w={w}: " + "; ".join(
            f"f={l['factor']}({l['pitch_mm']}mm): {l['n_free_bilateral']} free"
            f"/{l['n_bilateral']} bilat, {l['n_probes']}q"
            for l in a["ladder"]), flush=True)
    for w in (0.505, 0.52, 0.54, 0.58):
        t0 = time.time()
        b = exp_B(w)
        b["seconds"] = round(time.time() - t0, 1)
        res["B"].append(b)
        for cname in ("single", "iterated", "newton_kkt"):
            rr = [r for r in b["rows"] if r["corr"] == cname]
            nrec = sum(r["recovered"] for r in rr)
            print(f"B w={w} {cname}: {nrec}/{len(rr)} recovered", flush=True)
    for w in (0.49, 0.505, 0.51, 0.52, 0.54, 0.58):
        t0 = time.time()
        c = exp_C(w)
        c["seconds"] = round(time.time() - t0, 1)
        res["C"].append(c)
        print(f"C w={w}: {c['status']} stations {c['stations_bwd']}+"
              f"{c['stations_fwd']} x[{c['x_range'][0]},{c['x_range'][1]}] "
              f"cert_min {c['min_margin_mm']}mm profile_min "
              f"{c['profile_min_mm']}mm total {c['total']}q", flush=True)
    os.makedirs(TABDIR, exist_ok=True)
    from prov import make_provenance
    res["provenance"] = make_provenance(
        __file__, "diagnostics A(seed audit)/B(corrector, oracle axis)/"
        "C(oracle-seed tracker)")
    with open(os.path.join(TABDIR, "ridge_abc.json"), "w") as f:
        json.dump(res, f, indent=1,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print("-> results/tables/ridge_abc.json")


if __name__ == "__main__":
    main()
