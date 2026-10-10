"""bl B2 probe (cust_fields env, CPU): repo-side facts the cust_fields adapter relies on.

1. ``bl_custfields.squircle_level`` (our cover test) agrees with the repo's ``Rectangular.check_point_inside`` once the
   repo's hidden +0.1 m width/height pad is undone (and ``Workspace`` likewise).
2. A workspace ``StarTree`` (the repo's form for obstacles attached to the workspace boundary) crashes in
   ``ForestToStar.compute_virtual_ws`` (``Rectangular`` called without ``theta`` / ``s``).
3. Cost of the plain NF vs the number of obstacles M: wall time of one ``compute_potential_at_point`` and of one
   ``neg_gradient`` (4 potentials) in a 8 m x 5 m workspace with M squircles on a lattice.
Writes ``results/baselines/cust_fields/probe.json``.
"""
import json
import math
import os
import sys
import tempfile
import time
import traceback
from pathlib import Path

import numpy as np
import yaml

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import bl_custfields as C  # noqa: E402

sys.path.insert(0, C.REPO)
from NF.geometry import Rectangular, Workspace, World  # noqa: E402
from NF.navigation import NavigationFunction  # noqa: E402
from NF.transformation import ForestToStar  # noqa: E402


def world_file(obstacles, ws, extra_ws_stars=()):
    d = {"obstacles": obstacles, "workspace": [{"shape": "StarTree", "stars": [ws, *extra_ws_stars]}]}
    f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    yaml.safe_dump(d, f)
    f.close()
    return f.name


def main():
    out = {}
    rng = np.random.default_rng(0)
    # 1. level agreement
    bad, n = 0, 0
    for _ in range(200):
        c = rng.uniform(-2, 2, 2)
        a, b = rng.uniform(.06, 1.5, 2)
        th, s = rng.uniform(0, np.pi), rng.choice([.85, .99])
        r = Rectangular("Rectangular", c.tolist(), 2 * a - C.REPO_PAD, 2 * b - C.REPO_PAD, th, s)
        Q = c + rng.normal(size=(200, 2)) * max(a, b)
        ours = C.squircle_level(Q, c, a, b, th, s) <= 0
        repo = np.array([r.check_point_inside(q) for q in Q])
        lv = C.squircle_level(Q, c, a, b, th, s)
        mism = (ours != repo) & (np.abs(lv) > 1e-9)
        bad += int(mism.sum())
        n += len(Q)
    out["level_agreement"] = {"points": n, "mismatches": bad}
    ws = Workspace("Workspace", [0., 0.], 8. - C.REPO_PAD, 5. - C.REPO_PAD, 0., .9999)
    Q = rng.uniform([-4.2, -2.7], [4.2, 2.7], (4000, 2))
    inside = np.array([ws.check_point_inside(q) for q in Q])
    rect = (np.abs(Q[:, 0]) <= 4) & (np.abs(Q[:, 1]) <= 2.5)
    out["workspace_inside_implies_rectangle"] = bool(not (inside & ~rect).any())
    out["workspace_frac_of_rectangle_points_inside"] = float(inside[rect].mean())
    # 2. workspace StarTree crash
    wsd = {"type": "Workspace", "center": [3.5, 2.], "width": 7., "height": 4., "theta": 0., "s": .9999}
    son = {"type": "Rectangular", "center": [6.8, 2.], "width": 1., "height": .5, "theta": 0., "s": .85}
    try:
        w = World(world_file([], wsd, [son]))
        ForestToStar(w, np.array([1., 1.]), [1e10, 1e8]).compute_f(np.array([2., 2.]))
        out["workspace_tree"] = "ran"
    except Exception as exc:
        out["workspace_tree"] = f"{type(exc).__name__}: {exc}"
        out["workspace_tree_where"] = traceback.format_exc().strip().splitlines()[-3:]
    # 3. NF cost vs M
    wsd = {"type": "Workspace", "center": [0., 0.], "width": 8. - .1, "height": 5. - .1, "theta": 0., "s": .9999}
    out["nf_cost"] = []
    for M in (1, 2, 4, 8, 16, 32, 64):
        k = int(math.ceil(math.sqrt(M * 1.6)))
        xs = np.linspace(-3.4, 3.4, k)
        ys = np.linspace(-2., 2., max(1, int(math.ceil(M / k))))
        cs = [(x, y) for y in ys for x in xs][:M]
        obs = [{"shape": "Star", "star": [{"type": "Rectangular", "center": [float(x), float(y)], "width": .2,
                                           "height": .15, "theta": .3, "s": .85}]} for x, y in cs]
        w = World(world_file(obs, wsd))
        nf = NavigationFunction(w, np.array([3.9, 0.05, 0.]), 1e3, [1e10, 1e8, 1e6, 1e4, 1e2, 1e1])
        q = np.array([-3.85, -2.35])
        t0 = time.perf_counter()
        reps = 0
        while time.perf_counter() - t0 < 2. and reps < 50:
            nf.compute_potential_at_point(q)
            reps += 1
        pot = (time.perf_counter() - t0) / reps
        tnf = C._test_nf_module()
        t1 = time.perf_counter()
        tnf.neg_gradient(nf, q)
        grad = time.perf_counter() - t1
        out["nf_cost"].append({"M": M, "potential_s": pot, "neg_gradient_s": grad})
        print(M, pot, grad, flush=True)
        if grad > 30:
            break
    o = C.GMC / "results/baselines/cust_fields/probe.json"
    o.parent.mkdir(parents=True, exist_ok=True)
    o.write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
