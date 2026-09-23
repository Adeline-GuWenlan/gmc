"""Build the uav-lamp derivative archive and measure every edit's real-scene contacts.

Usage (from ``gmc/``, via sbatch):
    python experiments/uavlamp_build.py --config configs/uavlamp_scene.json \
        --output outputs/uavlamp/scene
Writes ``<output>/uavlamp_scene.npz`` + ``manifest.json``; the manifest gains a
``measured_contacts`` block computed from the *output* archive's source rows.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from gmc.gs3d import scene_uavlamp as su

TAU, LEVEL = su.TAU, su.LEVEL


def _q(x, q):
    return float(np.quantile(x, q)) if len(x) else None


def measure(scene, doc):
    frame = su.Frame.from_dict(doc["frame"])
    first = min(e["id_range"][0] for e in doc["edits"])
    src = (scene.ids < first) & (scene.opacity > TAU)
    q = doc["query"]
    lo = np.array([-5.5, -1.2, -.3]); hi = np.array([4.5, 3.4, 3.0])
    world_lo, world_hi = frame.to_world(lo), frame.to_world(hi)
    wlo, whi = np.minimum(world_lo, world_hi), np.maximum(world_lo, world_hi)
    corners = frame.to_world(np.array([[a, b, c] for a in (lo[0], hi[0]) for b in (lo[1], hi[1])
                                        for c in (lo[2], hi[2])]))
    wlo, whi = corners.min(0), corners.max(0)
    near = src & np.all(scene.means >= wlo - 1, axis=1) & np.all(scene.means <= whi + 1, axis=1)
    m = scene.means[near]; c = scene.covs[near]; ids = scene.ids[near]
    loc = frame.to_route(m)
    ex = LEVEL * np.sqrt(np.einsum("ij,njk,ik->ni", frame.R, c, frame.R))
    table_u, table_v = (-.2, 3.15), (0., 1.0)
    in_table = ((loc[:, 0] > table_u[0]) & (loc[:, 0] < table_u[1]) & (loc[:, 1] > table_v[0])
                & (loc[:, 1] < table_v[1]) & (loc[:, 2] < 1.3))
    rows = []
    for u0 in np.arange(-4.5, 3.6, .25):
        b = (loc[:, 0] >= u0) & (loc[:, 0] < u0 + .25) & (loc[:, 2] > .1) & (loc[:, 2] < 2.6)
        a_sel = b & (loc[:, 1] > -1.0) & (loc[:, 1] < .6) & ~in_table
        b_sel = b & (loc[:, 1] > 1.8) & (loc[:, 1] < 3.2)
        fa = loc[a_sel, 1] + ex[a_sel, 1]
        fb = loc[b_sel, 1] - ex[b_sel, 1]
        rows.append({"u": [round(float(u0), 3), round(float(u0 + .25), 3)],
                     "wallA_n": int(a_sel.sum()), "wallA_face_v_q50": _q(fa, .5), "wallA_face_v_q999": _q(fa, .999),
                     "wallB_n": int(b_sel.sum()), "wallB_face_v_q001": _q(fb, .001), "wallB_face_v_q50": _q(fb, .5),
                     "wallB_back_v_max": _q(loc[b_sel, 1] + ex[b_sel, 1], .999)})
    top = loc[:, 2] + ex[:, 2]
    tab = in_table & (loc[:, 2] > .3)
    tab_hi = tab & (top > .8)
    table = {"n_supports": int(tab.sum()), "top_q50": _q(top[tab], .5), "top_q99": _q(top[tab], .99),
             "top_max": _q(top[tab], 1.), "footprint_top_gt_0p8_u": [_q(loc[tab_hi, 0] - ex[tab_hi, 0], .001),
                                                                    _q(loc[tab_hi, 0] + ex[tab_hi, 0], .999)],
             "footprint_top_gt_0p8_v": [_q(loc[tab_hi, 1] - ex[tab_hi, 1], .001),
                                        _q(loc[tab_hi, 1] + ex[tab_hi, 1], .999)]}
    appr = (loc[:, 0] > -3.2) & (loc[:, 0] < -.5) & (loc[:, 1] > .1) & (loc[:, 1] < 2.25)
    floor = appr & (loc[:, 2] < .3)
    float_sel = appr & (loc[:, 2] >= .3) & (loc[:, 2] < 2.4) & (loc[:, 1] > .35) & (loc[:, 1] < 2.05)
    order = np.argsort(-ex[float_sel].max(1))[:10]
    floaters = [{"id": int(ids[float_sel][k]), "route": loc[float_sel][k].round(3).tolist(),
                 "half_extent": ex[float_sel][k].round(3).tolist()} for k in order]
    booth = (loc[:, 0] > -.85) & (loc[:, 0] < 3.3) & (loc[:, 1] > .1) & (loc[:, 1] < 2.25) \
        & (loc[:, 2] > .3) & (loc[:, 2] < 2.4) & ~in_table
    goal = np.asarray(q["goal_route"])
    d_goal = np.linalg.norm(loc - goal, axis=1)
    edits = {e["edit_id"]: e for e in doc["edits"]}
    fa_all = [r["wallA_face_v_q999"] for r in rows if r["wallA_face_v_q999"] is not None and -4.2 <= r["u"][0] < 3.3]
    fb_all = [r["wallB_face_v_q50"] for r in rows if r["wallB_face_v_q50"] is not None and -4.2 <= r["u"][0] < 3.3]
    contacts = []
    for e in doc["edits"]:
        lo_e, hi_e = e["solid_aabb_route_m"]["lower"], e["solid_aabb_route_m"]["upper"]
        contacts.append({"edit_id": e["edit_id"], "solid_v": [lo_e[1], hi_e[1]], "solid_z": [lo_e[2], hi_e[2]],
                         "gap_to_wallA_m (solid v_min - max wallA face q999; <=0 means embedded)":
                             (lo_e[1] - max(fa_all)) if fa_all else None,
                         "gap_to_wallB_m (min wallB face q50 - solid v_max; <=0 means embedded)":
                             (min(fb_all) - hi_e[1]) if fb_all else None,
                         "declared_contact": e["contact"]})
    return {"wall_face_rows": rows, "table": table,
            "floor_top_in_approach": {"q50": _q(top[floor], .5), "q99": _q(top[floor], .99), "max": _q(top[floor], 1.)},
            "approach_floaters_largest": floaters, "n_approach_floaters": int(float_sel.sum()),
            "n_source_supports_inside_booth_air_excluding_table": int(booth.sum()),
            "nearest_source_support_to_goal_m": float(d_goal.min()),
            "edit_contacts": contacts}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--config", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args(argv)
    config = json.loads(a.config.read_text())
    out = a.output
    archive, manifest = out / "uavlamp_scene.npz", out / "manifest.json"
    doc = su.build_uavlamp_derivative(config, archive, manifest)
    scene, doc2 = su.load_uavlamp_derivative(archive, manifest)
    doc2["config_path"] = str(a.config)
    doc2["measured_contacts"] = measure(scene, doc2)
    manifest.write_text(json.dumps(doc2, indent=1) + "\n")
    summary = {k: doc2[k] for k in ("scene_id", "derivative", "config_sha256")}
    summary["n_edits"] = len(doc2["edits"])
    summary["measured"] = {k: v for k, v in doc2["measured_contacts"].items() if k != "wall_face_rows"}
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
