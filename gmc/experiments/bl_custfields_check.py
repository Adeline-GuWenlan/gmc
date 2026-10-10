"""bl B2: what the cust_fields world construction does to the map (gmc-venv; reporting, no planning).

Per region x robot x construction variant (``bl_custfields`` world from the shared raster):
  * pieces, chains, stars, merges, covers crossing the workspace boundary, construction wall time;
  * free space: map-free vs NF-world-free (map-free and outside every cover) inside the workspace;
  * per pair (tuning set and F4's 5000): start / goal swallowed (no NF-free cell within snap_max_m), and whether the
    snapped endpoints are connected (8-conn) in the map and in the NF world -> "passage closed by the construction"
    = connected in the map, not in the NF world.  Per F4 lateral-clearance band.
Output ``results/baselines/cust_fields/world_check_<variant>.json``.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
import bl_custfields as C  # noqa: E402
import bl_raster as R  # noqa: E402


class _Scene(dict):
    pass


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--variant", required=True)
    ap.add_argument("--set", default="{}")
    ap.add_argument("--regions", nargs="+", default=["WWEST", "GAPW1", "S"])
    ap.add_argument("--robots", nargs="+", default=["cylinder", "sweeper"])
    a = ap.parse_args(argv)
    from scipy import ndimage
    from bl_harness import load_pairs, _lat_band
    cfg = dict(C.DEFAULTS, **json.loads(a.set))
    out = {"variant": a.variant, "config": cfg, "by": {}}
    for region in a.regions:
        for robot in a.robots:
            sp = Path("outputs/baselines/scene") / f"{region}_{robot}.npz"
            scene = R.load_scene(sp)
            t0 = time.perf_counter()
            st = C.Adapter().build_setup(scene, scene["meta"]["body"], cfg)
            wall = time.perf_counter() - t0
            arrays, info = R.load(C.GMC / st["info"]["raster"])
            nf_free = st["nf_free"].astype(bool)
            lab_nf, _ = ndimage.label(nf_free, structure=np.ones((3, 3), int))
            lab_map = arrays["free_label"]
            g = st["grid"]
            rec = {"setup": st["info"], "construction_wall_s": wall}
            for src in ("tuning", "f4"):
                rows = []
                for p in load_pairs(src, region):
                    e = []
                    for uv in (p["start_uv"], p["goal_uv"]):
                        hm = R.nearest_free(arrays, info, uv)
                        c, dd = C.snap_point(st["occ"], st["nf_free"], g, uv, float(cfg["snap_max_m"]))
                        hn = None if c is None else (*(int(v[0]) for v in R.to_index(g, c)), c, dd)
                        e.append((hm, hn))
                    map_conn = all(h[0] is not None for h in e) and \
                        lab_map[e[0][0][0], e[0][0][1]] == lab_map[e[1][0][0], e[1][0][1]]
                    swallowed = any(h[1] is None for h in e)
                    nf_conn = (not swallowed) and lab_nf[e[0][1][0], e[0][1][1]] == lab_nf[e[1][1][0], e[1][1][1]]
                    rows.append({"pair_id": p["pair_id"], "lat": p.get("lateral_clearance_m"),
                                 "map_connected": bool(map_conn), "endpoint_swallowed": bool(swallowed),
                                 "nf_connected": bool(nf_conn),
                                 "snap_m": None if swallowed else max(e[0][1][3], e[1][1][3])})
                n = len(rows)
                summ = {"n": n, "map_connected": sum(r["map_connected"] for r in rows),
                        "endpoint_swallowed": sum(r["endpoint_swallowed"] for r in rows),
                        "nf_connected": sum(r["nf_connected"] for r in rows),
                        "passage_closed_by_construction": sum(r["map_connected"] and not r["endpoint_swallowed"]
                                                              and not r["nf_connected"] for r in rows)}
                if src == "f4":
                    by = {}
                    for r in rows:
                        b = by.setdefault(_lat_band(r["lat"]), {"n": 0, "nf_connected": 0, "swallowed": 0})
                        b["n"] += 1
                        b["nf_connected"] += r["nf_connected"]
                        b["swallowed"] += r["endpoint_swallowed"]
                    summ["by_lateral_band"] = by
                rec[src] = summ
            out["by"][f"{region}_{robot}"] = rec
            s = rec["setup"]
            print(region, robot, {k: s.get(k) for k in ("pieces", "chains", "stars", "covers", "merges", "piece_merges",
                                                       "covers_crossing_workspace_boundary", "map_free_frac_in_ws",
                                                       "nf_free_frac_in_ws")},
                  "tuning", {k: v for k, v in rec["tuning"].items()}, "f4",
                  {k: v for k, v in rec["f4"].items() if k != "by_lateral_band"}, f"{wall:.1f}s", flush=True)
    o = Path("results/baselines/cust_fields") / f"world_check_{a.variant}.json"
    o.parent.mkdir(parents=True, exist_ok=True)
    o.write_text(json.dumps(out, indent=1, default=float) + "\n")


if __name__ == "__main__":
    main()
