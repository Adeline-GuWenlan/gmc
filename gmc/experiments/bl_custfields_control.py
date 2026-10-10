"""bl B2 control (cust_fields env, CPU): the adapter's own pipeline on worlds that satisfy the NF's assumptions.

Separates "the method on these maps" from "our pipeline is broken".  A synthetic C-space raster (same arrays and grid
as ``bl_raster``) with a few blobs, run through ``bl_custfields`` exactly as the harness does (build_setup ->
instantiate -> plan), with
  A. all blobs strictly inside the workspace (the forest-world assumption holds), and
  B. the same plus one wall piece attached to the workspace boundary (what every real region has).
Each world: a set of start/goal pairs on a lattice of free points; report goal_reached / nf_stuck / nf_max_steps.
Writes ``results/baselines/cust_fields/control.json``.
"""
import json
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import bl_custfields as C  # noqa: E402


def raster(blobs, res=.01, W=6., H=4., rho=.2):
    nx, ny = int(W / res), int(H / res)
    xs, ys = (np.arange(nx) + .5) * res, (np.arange(ny) + .5) * res
    X, Y = np.meshgrid(xs, ys)
    g = np.zeros((ny, nx), bool)
    for kind, p in blobs:
        if kind == "disk":
            g |= (X - p[0]) ** 2 + (Y - p[1]) ** 2 <= (p[2] + rho) ** 2
        else:                                         # axis-aligned bar (x0, x1, y0, y1)
            dx = np.maximum(np.maximum(p[0] - X, X - p[1]), 0)
            dy = np.maximum(np.maximum(p[2] - Y, Y - p[3]), 0)
            g |= dx * dx + dy * dy <= rho * rho
    m = .001
    ws = (X - res / 2 >= rho + m) & (X + res / 2 <= W - rho - m) & (Y - res / 2 >= rho + m) & (Y + res / 2 <= H - rho - m)
    occ = (g | ~ws).astype(np.uint8)
    info = {"res_m": res, "u0": 0., "v0": 0., "nx": nx, "ny": ny}
    return g, ws, occ, info


class _A(C.Adapter):
    """``build_setup`` minus the raster file: same world construction on the synthetic arrays."""

    def build(self, g, ws, occ, info, cfg, W, H, rho):
        sq = float(cfg["s"])
        chains, groups, st = C.build_obstacles_chain(g, ws, info, sq, 0., float(cfg["waste"]), int(cfg["max_chain"]),
                                                     tuple(cfg["tighten"]))
        covers = [c for ch in chains for c in ch]
        cmask = C.cover_mask(covers, sq, info, occ.shape)
        pad = C.REPO_PAD
        obs = []
        for ch in chains:
            stars = [{"type": "Rectangular", "center": [float(c[0][0]), float(c[0][1])], "width": float(2 * c[1] - pad),
                      "height": float(2 * c[2] - pad), "theta": float(c[3]), "s": sq} for c in ch]
            obs.append({"shape": "StarTree", "stars": stars} if len(stars) > 1 else {"shape": "Star", "star": stars})
        lo, hi = rho + .001, np.array([W, H]) - rho - .001
        world = {"obstacles": obs, "workspace": [{"shape": "StarTree", "stars": [{
            "type": "Workspace", "center": [W / 2, H / 2], "width": float(hi[0] - lo - pad),
            "height": float(hi[1] - lo - pad), "theta": 0., "s": .9999}]}]}
        nf_free = ws & ~cmask & (occ == 0)
        return {"world": world, "occ": occ, "nf_free": nf_free.astype(np.uint8),
                "grid": {"u0": 0., "v0": 0., "res_m": info["res_m"]}, "cfg": cfg,
                "info": dict(st, raster="(synthetic)", raster_sha256=None)}


def main():
    W, H, rho = 6., 4., .2
    worlds = {
        "A_interior_only": [("disk", (2., 2., .3)), ("disk", (4., 1.2, .25)), ("bar", (3.5, 4.3, 2.6, 2.9))],
        "B_plus_boundary_wall": [("disk", (2., 2., .3)), ("disk", (4., 1.2, .25)), ("bar", (3.5, 4.3, 2.6, 2.9)),
                                 ("bar", (3., 3.2, 0., 1.6))],
    }
    cfg = dict(C.DEFAULTS)
    out = {}
    os.chdir(tempfile.mkdtemp())
    for name, blobs in worlds.items():
        g, ws, occ, info = raster(blobs, W=W, H=H, rho=rho)
        ad = _A()
        st = ad.build(g, ws, occ, info, cfg, W, H, rho)
        ad_inst = C.Adapter()
        st["info"]["raster"] = None
        # instantiate without the raster SHA check (synthetic)
        import yaml
        sys.path.insert(0, C.REPO)
        from NF.geometry import World
        from NF.navigation import NavigationFunction
        yml = Path("world_%s.yaml" % name)
        yml.write_text(yaml.safe_dump(st["world"]))
        live = {"world": World(str(yml)), "NF": NavigationFunction, "tnf": C._test_nf_module(), "state": st,
                "cfg": cfg}
        pts = [(x, y) for x in (.5, 1.5, 3., 5.5) for y in (.5, 2., 3.5)]
        res = []
        for i, s in enumerate(pts):
            for gpt in pts[i + 1:][:3]:
                r = ad_inst.plan(live, list(s), list(gpt), {})
                res.append({"start": s, "goal": gpt, "claimed": r["claimed"], "reason": r["claimed_reason"],
                            "steps": (r.get("info") or {}).get("steps"), "s": r["stages"].get("nf_descend_s")})
        cnt = {}
        for r in res:
            cnt[r["reason"]] = cnt.get(r["reason"], 0) + 1
        out[name] = {"chains": len(st["world"]["obstacles"]), "counts": cnt, "rows": res}
        print(name, cnt, flush=True)
    o = C.GMC / "results/baselines/cust_fields/control.json"
    o.write_text(json.dumps(out, indent=1, default=float) + "\n")


if __name__ == "__main__":
    main()
