"""M1 refine pilot: two-level certified hierarchy closes the fidelity gap.

Coarse layer (v3 tangent-poly merged primitives) is sound: coarse-open =>
certified open. Coarse-closed steps get LOCAL refinement: merged primitives
whose scene polygon touches the door-corridor region (probe hull buffered by
robot half-length) are replaced by their raw members; everything else stays
merged. Mixed set still covers the true forbidden set, so refined-open is also
certified; refined-closed can only err via far-field conservatism (a true path
detouring outside the patch) — measured against the raw 720-theta reference.

Metrics per (window, robot): agreement after refinement, residual
disagreements, refined-step count, wall time coarse/refine/total vs full-raw.

Outputs: results/gmc_h2/m1_refine_pilot.json
"""
import json
import time

import numpy as np
import shapely
from shapely.geometry import box as shapely_box, LineString, MultiPoint

import h2_k2_windows as base
import m1_cluster_pilot3 as v3
import m1_cluster_pilot2 as v2

OUT = base.OUT
CONFIGS = [("door_A", "L24"), ("door_A", "L32"), ("door_B", "L32")]
PATCH_MARGIN = 0.3


def run(win_name, robot_name, ref):
    win, rob = base.WINDOWS[win_name], base.ROBOTS[robot_name]
    mu, S = base.load_window(win["center"], win["half"])
    ws = shapely_box(win["center"][0] - win["half"], win["center"][1] - win["half"],
                     win["center"][0] + win["half"], win["center"][1] + win["half"])
    probes = [LineString(p) for p in ref["probes"]]
    half_len = base.RHO * rob[0]

    buckets = v3.bucket_indices(mu, S)
    scene_polys = v3.tangent_polys(mu, S)
    corridor = MultiPoint([c for p in ref["probes"] for c in p]).convex_hull.buffer(
        half_len + PATCH_MARGIN)
    scene_geoms = shapely.polygons(scene_polys)
    hot = np.array([g.intersects(corridor) for g in scene_geoms])
    raw_idx = np.concatenate([buckets[i] for i in np.flatnonzero(hot)])
    cold_polys = scene_polys[~hot]
    n_raw_patch = len(raw_idx)

    thetas = np.linspace(0, np.pi, ref["n_theta"], endpoint=False)
    flags_ref = np.array(ref["open_flags"], bool)
    flags_two, refined = [], 0
    t_coarse = t_refine = 0.0
    for th in thetas:
        t0 = time.time()
        polys = v2.merged_forbidden_polys(scene_polys, th, rob)
        _, gate_open, _ = base.free_topology(polys, ws, probes)
        t_coarse += time.time() - t0
        if gate_open:
            flags_two.append(True)          # certified open at coarse level
            continue
        t0 = time.time()
        mixed = np.concatenate([
            base.forbidden_polys(mu[raw_idx], S[raw_idx], th, rob),
            v2.merged_forbidden_polys(cold_polys, th, rob)])
        _, gate_open2, _ = base.free_topology(list(mixed), ws, probes)
        t_refine += time.time() - t0
        refined += 1
        flags_two.append(bool(gate_open2))
    flags_two = np.array(flags_two)
    agree = float((flags_two == flags_ref).mean())
    false_open = int((~flags_ref & flags_two).sum())
    false_closed = int((flags_ref & ~flags_two).sum())
    t_raw_est = ref["n_theta"] * 0.56       # measured full-raw slice cost
    res = {
        "window": win_name, "robot": robot_name,
        "n_raw_patch_members": int(n_raw_patch),
        "n_hot_buckets": int(hot.sum()), "n_cold_merged": int((~hot).sum()),
        "refined_steps": refined, "n_theta": ref["n_theta"],
        "agreement": agree, "false_open": false_open, "false_closed": false_closed,
        "open_frac_ref": float(flags_ref.mean()),
        "open_frac_twolevel": float(flags_two.mean()),
        "t_coarse_s": t_coarse, "t_refine_s": t_refine,
        "t_total_s": t_coarse + t_refine,
        "t_fullraw_est_s": t_raw_est,
        "speedup_vs_fullraw": t_raw_est / (t_coarse + t_refine),
    }
    print(json.dumps(res, indent=1), flush=True)
    return res


def main():
    all_ref = json.loads((OUT / "k2_windows.json").read_text())["runs"]
    results = []
    for w, r in CONFIGS:
        ref = next(x for x in all_ref if x["window"] == w and x["robot"] == r)
        results.append(run(w, r, ref))
    (OUT / "m1_refine_pilot.json").write_text(json.dumps(
        {"patch_margin": PATCH_MARGIN, "results": results}, indent=2))
    print("done", flush=True)


if __name__ == "__main__":
    main()
