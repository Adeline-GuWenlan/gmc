"""C2 post-hoc diagnostic (NOT part of the pre-registered method, never counted as success).

For every aerial3d result whose own verifier certified the path but whose in-query shared
gs3d replay failed (status UNKNOWN, reason ``shared_replay_failed``):

1. per original segment, the shared oracle's ``edge`` report (occupancy / reason) with a fresh
   oracle on a freshly prepared index -- which segment failed, and was it a collision
   (``occupied``) or an unresolved known-space / margin test;
2. the SAME polyline with every segment split into collinear pieces of at most ``--piece-m``
   (geometry unchanged: same point set), replayed with ``replay_plan`` on a fresh oracle.

gs3d's known-space test is on the world-axis-aligned AABB of a whole swept segment against the
rotated route prism, so a long diagonal segment near a prism face can be ``map_unknown`` although
every configuration on it lies inside the domain; step 2 shows whether that is all that failed.

Usage (from ``gmc/``)::

    python experiments/uavconn_posthoc.py --template SPEC --uavlamp-root R --out DIR RESULT.json [...]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from gmc.aerial3d.api import _gs3d_result
from gmc.gs3d import scene_uavlamp as su
from gmc.gs3d.contracts import Pose3
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.trajectory import replay_plan

from uavlamp_query import UAV, build_scene
from uavlamp_run import check_spec
from uavconn_run import resolve, scene_key


def split(poly, piece_m: float) -> np.ndarray:
    P = np.asarray(poly, float)
    out = [P[0]]
    for a, b in zip(P[:-1], P[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / piece_m)))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.asarray(out)


def world_aabb_route_extent(a, b, frame, half):
    """Route-frame extent of the world AABB of the swept body (what gs3d's known-space test sees)."""
    lo, hi = np.minimum(a, b) - half, np.maximum(a, b) + half
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    r = frame.to_route(corners)
    return r.min(0).round(4).tolist(), r.max(0).round(4).tolist()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("results", nargs="+", type=Path)
    ap.add_argument("--template", type=Path, required=True)
    ap.add_argument("--uavlamp-root", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--piece-m", type=float, default=.20)
    a = ap.parse_args(argv)
    template = resolve(check_spec(json.loads(a.template.read_text())), a.uavlamp_root)
    full, mdoc = su.load_uavlamp_derivative(template["archive"], template["manifest"])
    frame, scene, *_ = build_scene(template, full, mdoc)
    del full
    margin = float(template.get("margin_m", .05))
    half = np.array([UAV.radius_m, UAV.radius_m, UAV.half_height_m])
    box = template["box_route"]
    stub = SimpleNamespace(body=UAV, scene=scene, config=SimpleNamespace(margin_m=margin), compile_id="posthoc")
    a.out.mkdir(parents=True, exist_ok=True)
    rows = []
    for path in a.results:
        doc = json.loads(path.read_text())
        if scene_key(resolve(check_spec(doc["spec"]), a.uavlamp_root)) != scene_key(template):
            raise ValueError(f"{path}: not the template's scene")
        r = doc["result"]
        row = {"name": doc["name"], "result": str(path), "status": r["status"], "reason": r["reason"],
               "piece_m": a.piece_m, "counted": False,
               "note": "post-hoc diagnostic; the pre-registered outcome stays as reported by the run"}
        if r["status"] == "UNKNOWN" and r["reason"] == "shared_replay_failed" and r["polyline_world"]:
            P = np.asarray(r["polyline_world"], float)
            oracle = GaussianBodyOracle(PreparedScene(scene))
            segs = []
            for s0, s1 in zip(P[:-1], P[1:]):
                rep = oracle.edge(Pose3(tuple(s0)), Pose3(tuple(s1)), UAV, margin_m=margin)
                lo, hi = world_aabb_route_extent(s0, s1, frame, half)
                segs.append({"length_m": float(np.linalg.norm(s1 - s0)), "occupancy": rep.occupancy,
                             "reason": rep.reason, "clearance_lower_m": rep.clearance_lower_m,
                             "world_aabb_route_lower": lo, "world_aabb_route_upper": hi,
                             "box_route": box})
            row["original_segments"] = segs
            Q = split(P, a.piece_m)
            res = _gs3d_result(stub, Q, P[-1], r["verification"]["own"]["clearance_lower_m"])
            rep = replay_plan(res, GaussianBodyOracle(PreparedScene(scene)))
            row["subdivided"] = {"pieces": int(len(Q) - 1), "passed": rep["passed"],
                                 "geometry": {k: rep["geometry"].get(k) for k in ("passed", "safety", "reason",
                                                                                  "clearance_lower_m")},
                                 "same_point_set": "collinear subdivision of the own-verified polyline"}
        rows.append(row)
        print(json.dumps({k: row.get(k) for k in ("name", "status", "reason")} |
                         {"segments": [(round(s["length_m"], 3), s["occupancy"], s["reason"])
                                       for s in row.get("original_segments", [])],
                          "subdivided": row.get("subdivided")}, default=float), flush=True)
    (a.out / "posthoc.json").write_text(json.dumps({"schema": "uavconn.c2_posthoc.v1", "rows": rows},
                                                   indent=1, default=float) + "\n")


if __name__ == "__main__":
    main()
