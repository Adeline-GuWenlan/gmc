"""M1 bucket-parameter sweep: coarse-layer fidelity vs cost.

Grid over (cell size, angle bins) for the v3 tangent-poly construction,
evaluated on door_A/L24 against the raw 720-theta reference. false_open must
be 0 everywhere (soundness is construction-level, not parameter-dependent);
the trade is n_merged/churn/slice-time vs extra_closed.

Output: results/gmc_h2/m1_param_sweep.json
"""
import json
import time

import numpy as np
import shapely
from shapely.geometry import box as shapely_box, LineString

import h2_k2_windows as base
import m1_cluster_pilot as v1
import m1_cluster_pilot2 as v2
import m1_cluster_pilot3 as v3

OUT = base.OUT
GRID = [(0.5, 8), (0.5, 12), (0.3, 8), (0.3, 12), (0.8, 8)]
WINDOW, ROBOT = "door_A", "L24"


def main():
    win, rob = base.WINDOWS[WINDOW], base.ROBOTS[ROBOT]
    mu, S = base.load_window(win["center"], win["half"])
    ws = shapely_box(win["center"][0] - win["half"], win["center"][1] - win["half"],
                     win["center"][0] + win["half"], win["center"][1] + win["half"])
    ref = next(r for r in json.loads((OUT / "k2_windows.json").read_text())["runs"]
               if r["window"] == WINDOW and r["robot"] == ROBOT)
    probes = [LineString(p) for p in ref["probes"]]
    thetas = np.linspace(0, np.pi, ref["n_theta"], endpoint=False)
    flags_ref = np.array(ref["open_flags"], bool)
    rows = []
    for cell, bins in GRID:
        v1.CELL, v1.ANGLE_BINS = cell, bins      # parameterize pilot bucketing
        scene_polys = v3.tangent_polys(mu, S)
        overcov = []
        for th in np.linspace(0, np.pi, 4, endpoint=False):
            runion = shapely.union_all(base.forbidden_polys(mu, S, th, rob)).intersection(ws)
            munion = shapely.union_all(v2.merged_forbidden_polys(scene_polys, th, rob)).intersection(ws)
            assert runion.difference(munion).area < 1e-9
            overcov.append(munion.difference(runion).area)
        flags, t0 = [], time.time()
        for th in thetas:
            polys = v2.merged_forbidden_polys(scene_polys, th, rob)
            _, gate_open, _ = base.free_topology(polys, ws, probes)
            flags.append(bool(gate_open))
        dt = time.time() - t0
        flags = np.array(flags)
        from shapely.strtree import STRtree
        polys0 = list(v2.merged_forbidden_polys(scene_polys, 0.0, rob))
        tr = STRtree(polys0)
        a, b = tr.query(polys0, predicate="intersects")
        rows.append({
            "cell": cell, "angle_bins": bins, "n_merged": int(len(scene_polys)),
            "agreement": float((flags == flags_ref).mean()),
            "false_open": int((~flags_ref & flags).sum()),
            "extra_closed": int((flags_ref & ~flags).sum()),
            "mean_overcoverage": float(np.mean(overcov)),
            "churn0": int((a < b).sum()),
            "slice_ms": float(dt / len(thetas) * 1000),
        })
        print(rows[-1], flush=True)
    (OUT / "m1_param_sweep.json").write_text(json.dumps(
        {"window": WINDOW, "robot": ROBOT, "rows": rows}, indent=2))
    print("done", flush=True)


if __name__ == "__main__":
    main()
