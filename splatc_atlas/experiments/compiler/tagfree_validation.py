"""P1a validation: tag-free bilateral grouping vs oracle side tags.

Compares bilateral_rho_tagfree (opposing-direction clustering, no wall-side
tags) against the oracle side_rho on:
  (a) every station of the v3.1 sweep dump (poses ON the marched ridge);
  (b) a deterministic Halton-like grid of poses in the wall neighborhood
      (|x|<=1.2, |y|<=0.6, theta in [0,pi)) per width — free and colliding.

Match criteria (free poses; colliding poses excluded because side_rho's
quick-collide path fills a -0.5 marker where tag-free reports exact PW):
  pair match  : {h_a,h_b} == {h+,h-} within 1e-9 up to label swap
  bilat match : bilateral-contact detection (both sides < CAP) agrees

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/tagfree_validation.py
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import make_g1_scene, robot_library
from splatc.gaussian_geometry.primitives import RHO_CAP

TABDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "tables")
DY0, TILT_DEG = 0.013, 7.0


def compare(scene, robot, poses, use_ref_chain):
    ref = None
    n = pair_ok = bilat_ok = free_n = 0
    worst = 0.0
    for q in poses:
        s = scene.side_rho(robot, q[2], np.array([q[0]]), np.array([q[1]]))
        hp, hm = float(s[1][0]), float(s[-1][0])
        ha, hb, dirs = scene.bilateral_rho_tagfree(robot, q, ref)
        if use_ref_chain:
            ref = dirs
        n += 1
        tag_bilat = max(hp, hm) < RHO_CAP - 1e-12
        tf_bilat = max(ha, hb) < RHO_CAP - 1e-12
        bilat_ok += (tag_bilat == tf_bilat)
        if min(hp, hm) > 0:                      # free poses: exact compare
            free_n += 1
            d1 = max(abs(ha - hp), abs(hb - hm))
            d2 = max(abs(ha - hm), abs(hb - hp))
            err = min(d1, d2)
            worst = max(worst, err)
            pair_ok += (err < 1e-9)
    return {"poses": n, "free": free_n, "pair_match": pair_ok,
            "bilat_match": bilat_ok, "worst_free_err": worst}


def main():
    with open(os.path.join(TABDIR, "ridge_stations.json")) as f:
        dump = json.load(f)
    dump.pop("_provenance", None)
    robot = robot_library()["R_long_ellipse"]
    grand = {"stations": [0, 0, 0], "grid": [0, 0, 0]}
    for wkey, d in sorted(dump.items()):
        w = float(wkey)
        scene = make_g1_scene(w, door_offset=DY0,
                              door_tilt=np.radians(TILT_DEG))
        # (a) marched stations, ref-chain on (temporal continuity, as used)
        st = [tuple(s) for s in d.get("stations", [])]
        ra = compare(scene, robot, st, use_ref_chain=True)
        # (b) wall-neighborhood grid, cold per pose (as the seed scan uses)
        g = 17
        xs = np.linspace(-1.2, 1.2, g)
        ys = np.linspace(-0.6, 0.6, g)
        ths = np.linspace(0, np.pi, 8, endpoint=False)
        poses = [(x, y, t) for x in xs for y in ys for t in ths]
        rb = compare(scene, robot, poses, use_ref_chain=False)
        print(f"w={wkey}: stations {ra['pair_match']}/{ra['free']} pair-match "
              f"(worst {ra['worst_free_err']:.2e}), "
              f"{ra['bilat_match']}/{ra['poses']} bilat-agree | "
              f"grid {rb['pair_match']}/{rb['free']} pair-match "
              f"(worst {rb['worst_free_err']:.2e}), "
              f"{rb['bilat_match']}/{rb['poses']} bilat-agree", flush=True)
        for k, r in (("stations", ra), ("grid", rb)):
            grand[k][0] += r["pair_match"]
            grand[k][1] += r["free"]
            grand[k][2] += r["poses"]
    print("TOTAL stations: %d/%d free-pose pair-match; grid: %d/%d" %
          (grand["stations"][0], grand["stations"][1],
           grand["grid"][0], grand["grid"][1]))


if __name__ == "__main__":
    main()
