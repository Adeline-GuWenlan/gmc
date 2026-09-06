"""Gamma(delta) complexity experiment (HANDOFF step 3, review round-2 §4).

Core metric:  Gamma(delta) = C_discover+cert / C_cert-only.
delta = (w - 2b)/2 in {40, 20, 10, 5, 2.5, 1.25} mm  ->  w = 0.5 + 2*delta,
plus the closed door w = 0.49 as control.

Arms:
  continuation : the v4.1 pipeline (ridge_continuation.run_case; tag-free
                 grouping, G1-frame-conditioned exit/span logic),
                 costs C_seed / C_track / C_densify / C_cert.
  cold_start   : DIAGNOSTIC arm (station x-grid uses the wall extent, and the
                 lateral bisection direction is the y axis — declared).  At
                 every 2cm station across the wall, find a free balanced pose
                 FROM SCRATCH: K-doubling theta scan (K=8..256) x y-balance
                 bisection; no information flows between stations.  Isolates
                 the value of theta/branch continuity ("延拓消除首次命中").
  cert_only    : ORACLE witness denominator (declared): axis polyline at
                 theta = tilt, 2cm spacing, start/goal connectors, priced by
                 certify_path_conservative checks.

Fits C = A * delta^-alpha per series (log-log least squares, successes only).
Volume-method reference (hybrid floor 4k/16k/128k at 40/20/10mm; >=128k fail
at <=5mm) is cited from results/tables/floor_probe_hybrid.json, not re-run.

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/gamma_delta.py
"""
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, Q_START, GOAL, R_EDGE)
from splatc.reference.oracle import certify_path_conservative
from ridge_continuation import run_case

TABDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "tables")
DY0, TILT_DEG = 0.013, 7.0
DELTAS_MM = (40.0, 20.0, 10.0, 5.0, 2.5, 1.25)


def build(w):
    scene = make_g1_scene(w, door_offset=DY0, door_tilt=np.radians(TILT_DEG))
    robot = robot_library()["R_long_ellipse"]
    return scene, robot


def axis_pose(scene, t):
    tilt = scene.meta["door_tilt"]
    off = scene.meta["door_offset"]
    return (t * np.cos(tilt), off + t * np.sin(tilt), tilt)


def cert_only(w):
    """ORACLE witness (declared): axis polyline, 2cm spacing, certified."""
    scene, robot = build(w)
    ts = np.arange(-1.0, 1.0 + 1e-9, 0.02)
    wps = [tuple(Q_START), (Q_START[0], Q_START[1], axis_pose(scene, ts[0])[2])]
    wps += [axis_pose(scene, t) for t in ts]
    wps += [(GOAL[0], GOAL[1], axis_pose(scene, ts[-1])[2])]
    ok, mmin, depth, checks = certify_path_conservative(
        scene, robot, np.array(wps))
    return {"w": w, "certified": bool(ok), "checks": int(checks),
            "min_margin_mm": round(mmin * 1000, 2), "depth": int(depth)}


def cold_start(w, spacing=0.02, k_max=256):
    """DIAGNOSTIC arm (declared): per-station from-scratch discovery."""
    scene, robot = build(w)
    nq = [0]

    def probe(q):
        s = scene.side_rho(robot, q[2], np.array([q[0]]), np.array([q[1]]))
        nq[0] += 1
        return float(s[1][0]), float(s[-1][0])

    def balance_y(x, th):
        def B(y):
            hp, hm = probe((x, y, th))
            return hp - hm
        lo, hi = -0.15, 0.15
        blo, bhi = B(lo), B(hi)
        if blo * bhi > 0:
            return None
        for _ in range(11):
            mid = 0.5 * (lo + hi)
            bm = B(mid)
            if blo * bm <= 0:
                hi, bhi = mid, bm
            else:
                lo, blo = mid, bm
        return 0.5 * (lo + hi)

    stations = np.arange(-0.64, 0.64 + 1e-9, spacing)
    rows = []
    for x in stations:
        c0 = nq[0]
        found, k_used = None, None
        K = 8
        while K <= k_max and found is None:
            for th in np.linspace(0, np.pi, K, endpoint=False):
                y = balance_y(x, th)
                if y is None:
                    continue
                hp, hm = probe((x, y, th))
                if min(hp, hm) > 0:
                    found, k_used = (x, y, th), K
                    break
            K *= 2
        rows.append({"x": round(float(x), 3), "ok": found is not None,
                     "K": k_used, "queries": nq[0] - c0})
    n_ok = sum(r["ok"] for r in rows)
    return {"w": w, "stations": len(rows), "solved": n_ok,
            "all_solved": n_ok == len(rows), "total_queries": nq[0],
            "per_station": rows}


def fit_alpha(deltas, costs):
    """least-squares slope of log C vs log delta (C = A * delta^-alpha)"""
    d = np.log(np.array(deltas, dtype=float))
    c = np.log(np.array(costs, dtype=float))
    if len(d) < 2:
        return None
    A = np.vstack([d, np.ones_like(d)]).T
    slope, _ = np.linalg.lstsq(A, c, rcond=None)[0]
    return round(float(-slope), 3)


def main():
    out = {"deltas_mm": list(DELTAS_MM), "continuation": [], "cold_start": [],
           "cert_only": [], "closed_door": {}, "fits": {}}
    for dmm in DELTAS_MM:
        w = round(0.5 + 2 * dmm / 1000.0, 4)
        t0 = time.time()
        row, diag = run_case(w)
        row["delta_mm"] = dmm
        row["seconds"] = round(time.time() - t0, 1)
        out["continuation"].append(row)
        print("cont ", row, flush=True)

        co = cert_only(w)
        co["delta_mm"] = dmm
        out["cert_only"].append(co)
        print("cert ", co, flush=True)

        t0 = time.time()
        cs = cold_start(w)
        cs["delta_mm"] = dmm
        cs["seconds"] = round(time.time() - t0, 1)
        out["cold_start"].append(
            {k: v for k, v in cs.items() if k != "per_station"})
        print("cold ", {k: v for k, v in cs.items() if k != "per_station"},
              flush=True)

    # closed-door control
    row, _ = run_case(0.49)
    out["closed_door"]["continuation"] = row
    cs = cold_start(0.49)
    out["closed_door"]["cold_start"] = {
        k: v for k, v in cs.items() if k != "per_station"}
    print("closed", row["status"],
          "cold solved", cs["solved"], "/", cs["stations"], flush=True)

    # exponent fits over certified successes
    cont_ok = [r for r in out["continuation"]
               if r["status"] == "CERTIFIED_REACHABLE"]
    cert_ok = [r for r in out["cert_only"] if r["certified"]]
    cold_ok = [r for r in out["cold_start"] if r["all_solved"]]
    out["fits"] = {
        "continuation_total": fit_alpha(
            [r["delta_mm"] for r in cont_ok], [r["total"] for r in cont_ok]),
        "continuation_minus_seed": fit_alpha(
            [r["delta_mm"] for r in cont_ok],
            [r["total"] - r["c_seed"] for r in cont_ok]),
        "continuation_track": fit_alpha(
            [r["delta_mm"] for r in cont_ok], [r["c_track"] for r in cont_ok]),
        "cert_only": fit_alpha(
            [r["delta_mm"] for r in cert_ok], [r["checks"] for r in cert_ok]),
        "cold_start": fit_alpha(
            [r["delta_mm"] for r in cold_ok],
            [r["total_queries"] for r in cold_ok]),
    }
    # Gamma per delta (successes).  Denominator naming per review round-3:
    #   gamma_total     = whole pipeline / oracle-witness cert
    #   gamma_amortized = (track+densify+cert) / oracle-witness cert
    #                     (one-time seed excluded — only meaningful once
    #                      reuse across goals/queries is demonstrated)
    #   gamma_self      = whole pipeline / cert cost of ITS OWN output path
    out["gamma"] = [
        {"delta_mm": r["delta_mm"],
         "gamma_total": round(r["total"] / co["checks"], 2),
         "gamma_amortized": round(
             (r["total"] - r["c_seed"]) / co["checks"], 2),
         "gamma_self": round(r["total"] / r["c_cert"], 2)}
        for r, co in zip(out["continuation"], out["cert_only"])
        if r["status"] == "CERTIFIED_REACHABLE" and co["certified"]]
    # local (narrowest-3) exponents — the review-audited robust statistic
    out["fits_narrow3"] = {
        "continuation_track": fit_alpha(
            [r["delta_mm"] for r in cont_ok][-3:],
            [r["c_track"] for r in cont_ok][-3:]),
        "cert_only": fit_alpha(
            [r["delta_mm"] for r in cert_ok][-3:],
            [r["checks"] for r in cert_ok][-3:]),
        "continuation_total": fit_alpha(
            [r["delta_mm"] for r in cont_ok][-3:],
            [r["total"] for r in cont_ok][-3:]),
    }
    print("fits ", out["fits"], flush=True)
    print("fits3", out["fits_narrow3"], flush=True)
    print("gamma", out["gamma"], flush=True)
    from prov import make_provenance
    out["provenance"] = make_provenance(
        __file__, "gamma: continuation-v4.1(tag-free, gate chart) / "
        "cold-start stationwise oracle-positioned lower-bound ablation / "
        "oracle-witness cert-only")
    os.makedirs(TABDIR, exist_ok=True)
    with open(os.path.join(TABDIR, "gamma_delta.json"), "w") as f:
        json.dump(out, f, indent=1,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print("-> results/tables/gamma_delta.json")


if __name__ == "__main__":
    main()
