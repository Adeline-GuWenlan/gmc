"""F2 Task 1d: why the shared replay rejected F2-03140's route on geometry (``map_unknown``) although aerial3d's
own verifier certified it.  Light (manifest frame + box only, no archive).

The shared oracle's edge test (``gs3d/oracle.py`` ``edge``) asks the known space to contain the WORLD AABB of the
swept segment, ``[min(pa, pb) - half, max(pa, pb) + half]``, then maps that AABB's 8 corners into the route frame
(``integration.RouteBoxKnownSpace.contains_aabb``).  aerial3d's domain is per pose: the body's world AABB inside the
prism (F1 §1a).  The set of such centres is convex, so a segment whose two ends satisfy it satisfies it at every
point; but the AABB of the union of two displaced world AABBs, re-expressed in the 64-degree rotated route frame,
can stick out of the prism.  This script measures, for every exported (densified, <= 0.20 m) segment, by how much
(a) each end pose's own body AABB and (b) the swept-segment AABB exceed the prism faces.
"""
from __future__ import annotations

import argparse
from itertools import product
import json
from pathlib import Path

import numpy as np

from gmc.aerial3d.api import _densify
from aerial3dg_run import MANIFEST, QCONFIG, ROBOTS, _dump


def excess(lo_w, hi_w, origin, R, box_lo, box_hi):
    """Largest distance (m, route frame) by which the corners of world AABB [lo_w, hi_w] leave the prism."""
    corners = np.asarray(list(product(*zip(lo_w, hi_w))), float)
    r = (corners - origin) @ R.T
    return float(max(np.max(box_lo - r), np.max(r - box_hi)))


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--rows", type=Path, default=Path("results/aerial3dg/f2/gmc/cylinder/task_00.jsonl"))
    p.add_argument("--pair", default="F2-03140")
    p.add_argument("--box", type=float, nargs=4, default=[-11.5, 2.85, -6.4, 7.5])
    p.add_argument("--out", type=Path, default=Path("results/aerial3dg/f2/diag/replay_domain_F2-03140.json"))
    a = p.parse_args(argv)
    man = json.loads(MANIFEST.read_text())
    origin = np.asarray(man["frame"]["origin_world_m"], float)
    R = np.asarray(man["frame"]["world_to_route"], float)
    box_lo, box_hi = np.array([*a.box[:2], 0.]), np.array([*a.box[2:], 2.43])
    row = next(json.loads(l) for l in open(a.rows) if json.loads(l)["pair_id"] == a.pair)
    body = ROBOTS["cylinder"]
    half = np.array([body.radius_m, body.radius_m, body.half_height_m])
    poly_r = np.asarray(row["route_polyline"], float)
    poly_w = poly_r @ R + origin
    dens = _densify(poly_w, QCONFIG.export_max_segment_m)
    segs = []
    for k, (pa, pb) in enumerate(zip(dens[:-1], dens[1:])):
        segs.append({"segment": k, "a_route": ((pa - origin) @ R.T).round(4).tolist(),
                     "b_route": ((pb - origin) @ R.T).round(4).tolist(),
                     "pose_a_excess_m": excess(pa - half, pa + half, origin, R, box_lo, box_hi),
                     "pose_b_excess_m": excess(pb - half, pb + half, origin, R, box_lo, box_hi),
                     "swept_excess_m": excess(np.minimum(pa, pb) - half, np.maximum(pa, pb) + half, origin, R,
                                              box_lo, box_hi)})
    bad = [s for s in segs if s["swept_excess_m"] > 1e-12]
    out = {"pair_id": a.pair, "status": row["status"], "reason": row["reason"],
           "own_verification": row["own_verification"], "route_polyline": row["route_polyline"],
           "export_max_segment_m": QCONFIG.export_max_segment_m, "segments": len(segs),
           "max_pose_excess_m": max(max(s["pose_a_excess_m"], s["pose_b_excess_m"]) for s in segs),
           "segments_with_swept_excess": len(bad), "max_swept_excess_m": max(s["swept_excess_m"] for s in segs),
           "first_bad_segment": bad[0] if bad else None, "rows": segs, "rule": __doc__.strip()}
    _dump(a.out, out)
    print(json.dumps({k: out[k] for k in ("pair_id", "segments", "max_pose_excess_m", "segments_with_swept_excess",
                                          "max_swept_excess_m", "first_bad_segment")}, default=float))


if __name__ == "__main__":
    main()
