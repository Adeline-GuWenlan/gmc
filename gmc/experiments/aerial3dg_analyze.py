"""G3 analysis of the aerial3d-ground 5000-pair run (G2) + G1's single-pair breakdown.

Reads only committed artefacts (``results/aerial3dg/{g1,g2}``); nothing is re-planned here.

* ``select``   deterministic gallery pairs (rules below, fixed before rendering) -> configs/aerial3dg/g3_gallery_pairs.json
* ``analyze``  distributions, robot comparison, reuse, distance tiers, scale/resolution, oracle-map cross-check
               -> results/aerial3dg/g3/analysis.json
* ``csv``      the filled measurement template copy (G1's four demo columns + two 5000-pair columns)
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

G1 = Path("results/aerial3dg/g1")
G2 = Path("results/aerial3dg/g2")
ROBOTS = ("sweeper", "cylinder")
GAP_U, LAMP_U = -5.6, -0.85          # the captured structure at u~-5.6 and the lamp centre line (route u, m)
MAJOR_ROUTE_CHANGE_M = 0.5           # Hausdorff distance between two routes that counts as a different route


def load_rows(runs: Path = G2 / "runs") -> dict:
    out = {}
    for robot in ROBOTS:
        rows = {}
        for f in sorted((runs / robot).glob("task_*.jsonl")):
            for line in f.read_text().splitlines():
                if line.strip():
                    d = json.loads(line)
                    rows[d["index"]] = d
        out[robot] = rows
    return out


def densify(poly, step=.02) -> np.ndarray:
    p = np.asarray(poly, float)[:, :2]
    out = [p[:1]]
    for a, b in zip(p[:-1], p[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out.append(a + (b - a) * np.linspace(0, 1, n + 1)[1:, None])
    return np.vstack(out)


def hausdorff(p, q) -> float:
    a, b = densify(p), densify(q)
    return float(max(cKDTree(b).query(a)[0].max(), cKDTree(a).query(b)[0].max()))


def crossings(poly, u0) -> list[float]:
    """v values where a route polyline crosses the line u = u0."""
    p = np.asarray(poly, float)[:, :2]
    vs = []
    for a, b in zip(p[:-1], p[1:]):
        if (a[0] - u0) * (b[0] - u0) < 0:
            vs.append(float(a[1] + (u0 - a[0]) / (b[0] - a[0]) * (b[1] - a[1])))
    return vs


def straddles(r, u0) -> bool:
    return (r["start_uv"][0] - u0) * (r["goal_uv"][0] - u0) < 0


def both_reachable(R) -> list[dict]:
    out = []
    for i, s in R["sweeper"].items():
        c = R["cylinder"][i]
        if s["status"] == c["status"] == "REACHABLE":
            out.append({"index": i, "pair_id": s["pair_id"], "dist_m": s["dist_m"], "start_uv": s["start_uv"],
                        "goal_uv": s["goal_uv"], "L_sweeper": s["path_length_m"], "L_cylinder": c["path_length_m"],
                        "dL": c["path_length_m"] - s["path_length_m"],
                        "hausdorff_m": hausdorff(s["route_polyline"], c["route_polyline"])})
    return sorted(out, key=lambda r: r["index"])


# ----------------------------------------------------------------------------- gallery selection
GALLERY_RULES = {
    "T1_thread_vs_detour": "both REACHABLE and G1's pre-registered contrast rule (sweeper L <= 1.02 d, cylinder "
                           "L >= 1.05 d); the largest L_cylinder - L_sweeper",
    "T2_thread_vs_detour_reverse": "same rule, travelling the opposite u direction to T1; the largest L_cylinder - "
                                   "L_sweeper",
    "L1_gap_and_lamp": "sweeper REACHABLE with a route crossing both u=-5.6 and the lamp line u=-0.85, cylinder "
                       "certified UNREACHABLE; the straightest sweeper route (min L/d)",
    "N1_gap_only": "sweeper REACHABLE crossing u=-5.6 but not the lamp, cylinder UNKNOWN "
                   "(safe_graph_disconnected_possible_connected); the straightest sweeper route (min L/d)",
    "S1_sweeper_max_detour": "sweeper REACHABLE with the largest L/d over all 5000 pairs",
    "B1_both_same_route": "both REACHABLE with route Hausdorff < 0.2 m; the largest d (control: no morphology effect)",
}


def select_gallery(R) -> list[dict]:
    sw, cy = R["sweeper"], R["cylinder"]
    both = both_reachable(R)
    qual = [b for b in both if b["L_sweeper"] <= 1.02 * b["dist_m"] and b["L_cylinder"] >= 1.05 * b["dist_m"]]
    t1 = max(qual, key=lambda b: b["dL"])
    sgn = np.sign(t1["goal_uv"][0] - t1["start_uv"][0])
    t2 = max((b for b in qual if np.sign(b["goal_uv"][0] - b["start_uv"][0]) != sgn), key=lambda b: b["dL"])
    reach = {i: r for i, r in sw.items() if r["status"] == "REACHABLE"}
    ratio = {i: r["path_length_m"] / r["dist_m"] for i, r in reach.items()}
    l1 = min((i for i, r in reach.items() if crossings(r["route_polyline"], GAP_U) and
              crossings(r["route_polyline"], LAMP_U) and cy[i]["status"] == "UNREACHABLE"), key=ratio.get)
    n1 = min((i for i, r in reach.items() if crossings(r["route_polyline"], GAP_U) and
              not crossings(r["route_polyline"], LAMP_U) and
              cy[i]["reason"] == "safe_graph_disconnected_possible_connected"), key=ratio.get)
    s1 = max(reach, key=ratio.get)
    b1 = max((b for b in both if b["hausdorff_m"] < .2), key=lambda b: b["dist_m"])
    picks = [("T1_thread_vs_detour", t1["index"]), ("T2_thread_vs_detour_reverse", t2["index"]),
             ("L1_gap_and_lamp", l1), ("N1_gap_only", n1), ("S1_sweeper_max_detour", s1),
             ("B1_both_same_route", b1["index"])]
    out = []
    for name, i in picks:
        s, c = sw[i], cy[i]
        out.append({"name": name, "role": GALLERY_RULES[name], "g2_index": i, "pair_id": s["pair_id"],
                    "start_uv": s["start_uv"], "goal_uv": s["goal_uv"], "dist_m": s["dist_m"],
                    "g2_status": {"sweeper": s["status"], "cylinder": c["status"]},
                    "g2_reason": {"sweeper": s["reason"], "cylinder": c["reason"]},
                    "g2_length_m": {"sweeper": s["path_length_m"], "cylinder": c["path_length_m"]}})
    return out


def cmd_select(a):
    R = load_rows()
    pairs = select_gallery(R)
    doc = {"box_route_uv": [-9.0, -0.35, 3.7, 2.75],
           "selection": "G3 gallery from the G2 5000-pair results (gmc/results/aerial3dg/g2/runs); rules fixed before "
                        "rendering, applied by experiments/aerial3dg_analyze.py select", "rules": GALLERY_RULES,
           "pairs": pairs}
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(doc, indent=1) + "\n")
    for p in pairs:
        print(p["name"], p["pair_id"], round(p["dist_m"], 3), p["g2_status"], p["g2_length_m"])


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    se = sub.add_parser("select")
    se.add_argument("--out", type=Path, default=Path("configs/aerial3dg/g3_gallery_pairs.json"))
    a = p.parse_args(argv)
    {"select": cmd_select}[a.cmd](a)


if __name__ == "__main__":
    main()
