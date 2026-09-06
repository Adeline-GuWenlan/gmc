"""Regression: src/splatc/gmc two-level gate_query with adaptive patch growth
vs the raw 720-theta K2 reference. Config door_B/L32 — the worst fixed-patch
case (8 false-closed in m1_refine_pilot). Expect agreement 1.0.

Note the cost asymmetry by design: adaptive exactness escalates every truly
CLOSED step to a fully-raw slice (hot.all() termination), so it is slower than
the fixed-patch policy (agreement 0.989) — policy choice exposed via API.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, box as shapely_box

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from splatc.gmc import build_merged, gate_query  # noqa: E402

import h2_k2_windows as base  # noqa: E402

WINDOW, ROBOT = "door_B", "L32"

win = base.WINDOWS[WINDOW]
lam0 = np.diag(base.ROBOTS[ROBOT] ** 2)
mu, S = base.load_window(win["center"], win["half"])
ws = shapely_box(win["center"][0] - win["half"], win["center"][1] - win["half"],
                 win["center"][0] + win["half"], win["center"][1] + win["half"])
ref = next(r for r in json.loads((base.OUT / "k2_windows.json").read_text())["runs"]
           if r["window"] == WINDOW and r["robot"] == ROBOT)
probes = [LineString(p) for p in ref["probes"]]
flags_ref = np.array(ref["open_flags"], bool)
thetas = np.linspace(0, np.pi, ref["n_theta"], endpoint=False)

ms = build_merged(mu, S, rho=base.RHO)
print(f"{len(mu)} raw -> {len(ms.polys)} merged", flush=True)

flags, levels = [], {"coarse": 0, "refined": 0, "exact": 0}
t0 = time.time()
for k, th in enumerate(thetas):
    res = gate_query(ms, th, lam0, ws, probes)
    flags.append(res["verdict"] == "open")
    levels[res["level"].split("-")[0]] += 1
    if k % 120 == 0:
        print(f"  {k}/720 {time.time()-t0:.0f}s", flush=True)
flags = np.array(flags)
out = {
    "window": WINDOW, "robot": ROBOT,
    "agreement": float((flags == flags_ref).mean()),
    "false_open": int((~flags_ref & flags).sum()),
    "false_closed": int((flags_ref & ~flags).sum()),
    "certificate_levels": levels,
    "wall_s": time.time() - t0,
}
(base.OUT / "m1_src_regression.json").write_text(json.dumps(out, indent=2))
print(json.dumps(out, indent=1), flush=True)
