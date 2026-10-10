"""bl B2: PNO (github.com/ExistentialRobotics/PNO @ 6384751) adapter for ``bl_worker`` -- zero-shot only.

Runs in the ``pno`` env (torch 2.5.1) with the value-function inference on a GPU; the search is CPU Python.

Model: the published pretrained 2D planning operator, ``DEEPNORM2dMultiGoal(4, 8, 8, 16)`` with the HF weights
``PNO`` (or ``PNOwPINN``; HF ``lukebhan/generalizableMotionPlanningViaOperatorLearning`` rev 36a76173, SHA-256 checked
against the B0 sidecars), and its SDF approximator ``FNO2d(4, 1, 8, 8, 16)`` ``FNOSDF`` -- exactly the 2D example
notebook's models and conventions (B0: mask (N, S, S, 1) 1 = free, goal (col, row), output x mask;
chi = smooth_chi(mask, FNOSDF(mask), 5)).  No training or fine-tuning of any kind (the user's decision).  The street-map
``PNO2D`` checkpoint the repo's 2D heuristic notebooks load is not published, so the published model is used inside
the paper's own 2D extraction pipeline:

Path extraction = the paper's ``2D_Neural_Heuristics/heuristics.py::planningoperator`` + ``AStar.plan`` (repo code,
imported): erode the obstacles by ``erosion`` cells (``1 - binary_erosion(map, iterations=erosion)``, "to under
approximate the value function"), infer V for the goal, ``V / (mask + 1e-9)``, ``max(V, euclidean)``, and run the
repo's 8-connected grid A* (``astar/astar.py`` + ``environment_simple.Environment2D``) on the (un-eroded) map with
that heuristic.  The model's V is in its training units (1/4 cell at 256^2, i.e. map width = 64); it is converted to
cells by ``S / 64`` (``v_unit``) so that ``max(V, euclidean)`` compares like with like (checked on an empty map by
the probe).  Grid A* is complete on the grid: PNO's quality changes the expansions (time), not the answer.

Map: the shared rasteriser's C-space map (``bl_raster``, judge-conservative at its resolution) resampled to the model's
S x S input grid over the region box's longer side (the shorter side padded as occupied): a coarse cell is free only
if every fine cell its closed square overlaps is free (max-pool with partial overlaps), so a free coarse cell is
judge-free everywhere and 8-connected moves between free cells are judge-free (the diagonal crosses only the shared
corner).  Resolution loss = coarse cell / fine cell (recorded).

Endpoints: start/goal cells that the conservative map marks occupied (the F4 endpoints are typically 1-5 mm from an
obstacle) are moved to the nearest free cell centre (recorded, as SplatNav's own A* does); the harness adds the exact
start / goal segments, which the judge checks like the rest.  If the snapped endpoints are in different 8-connected
free components the repo's A* would exhaust the start component and return no path; we return that answer directly
(``astar_no_path_component``) instead of spending the search -- same outcome, labelled.
"""
from __future__ import annotations

import hashlib
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

import bl_raster as R

REPO = "/scratch/wg2381/ext_repos/PNO"
GMC = Path(__file__).resolve().parents[1]
MODELS = GMC / "outputs/baselines/pno/models"
DEFAULTS = {"raster_res_m": 0.005, "S": 2048, "model": "PNO", "erosion": 4, "smooth_coef": 5.,
            "v_unit": "S/64", "snap_max_m": 1.0, "collinear_merge": True}


def _sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 22), b""):
            h.update(b)
    return h.hexdigest()


def resample_maxpool(occ, res_f, S, side_m):
    """Coarse S x S occupancy over [0, side_m]^2 (relative to the fine grid origin): a coarse cell is occupied iff
    some fine cell its closed square overlaps is occupied, or it reaches outside the fine grid."""
    ny, nx = occ.shape
    res_c = side_m / S
    edges = np.arange(S + 1) * res_c
    lo = np.floor(edges[:-1] / res_f - 1e-9).astype(np.int64)
    hi = np.ceil(edges[1:] / res_f + 1e-9).astype(np.int64) - 1                 # inclusive fine index range
    sat = np.zeros((ny + 1, nx + 1), np.int64)
    sat[1:, 1:] = np.cumsum(np.cumsum(occ.astype(np.int64), 0), 1)

    def count(r0, r1, c0, c1):                     # inclusive ranges, clipped; outside the fine grid -> occupied
        a0, a1 = np.clip(r0, 0, ny), np.clip(r1 + 1, 0, ny)
        b0, b1 = np.clip(c0, 0, nx), np.clip(c1 + 1, 0, nx)
        return sat[a1][:, b1] - sat[a0][:, b1] - sat[a1][:, b0] + sat[a0][:, b0]
    cnt = count(lo, hi, lo, hi)
    outside = ((lo < 0) | (hi > ny - 1))[:, None] | ((lo < 0) | (hi > nx - 1))[None, :]
    return ((cnt > 0) | outside).astype(np.uint8), res_c


def collinear_merge(cells):
    """Drop interior grid-path vertices where the step direction does not change (same geometry)."""
    c = np.asarray(cells, np.int64)
    if len(c) <= 2:
        return c
    d = np.diff(c, axis=0)
    keep = np.r_[True, np.any(d[1:] != d[:-1], axis=1), True]
    return c[keep]


class Adapter:
    def __init__(self):
        self.setup_info, self.instance_info = {}, {}

    def build_setup(self, scene, body, config):
        from scipy import ndimage
        cfg = dict(DEFAULTS, **config)
        meta = scene["meta"]
        path = GMC / R.raster_path(meta["region"], meta["robot"], float(cfg["raster_res_m"]))
        arrays, rinfo = R.load(path)
        if rinfo["scene_export_sha256"] != R._sha(GMC / "outputs/baselines/scene" /
                                                   f"{meta['region']}_{meta['robot']}.npz"):
            raise ValueError("raster was not built from this scene export")
        t0 = time.perf_counter()
        lo, hi = meta["known_route_lower_m"], meta["known_route_upper_m"]
        side = max(hi[0] - lo[0], hi[1] - lo[1])
        S = int(cfg["S"])
        occ, res_c = resample_maxpool(arrays["occ"], rinfo["res_m"], S, side)
        lab, ncomp = ndimage.label(1 - occ, structure=np.ones((3, 3), int))
        erosion = int(cfg["erosion"])
        mask = 1 - occ
        eroded = (1 - ndimage.binary_erosion(1 - mask, iterations=erosion).astype(mask.dtype)) if erosion > 0 \
            else mask.copy()                                  # heuristics.py planningoperator, verbatim operation
        info = {"raster": str(path.relative_to(GMC)), "raster_sha256": rinfo["sha256"],
                "raster_build_wall_s": rinfo["build_wall_s"], "raster_res_m": rinfo["res_m"], "S": S,
                "grid_res_m": res_c, "resolution_loss_factor": res_c / rinfo["res_m"], "side_m": side,
                "box_m": [hi[0] - lo[0], hi[1] - lo[1]], "free_frac_box": float(mask[:int(math.ceil(
                    (hi[1] - lo[1]) / res_c)), :int(math.ceil((hi[0] - lo[0]) / res_c))].mean()),
                "free_frac_fine": float(1 - arrays["occ"].mean()), "free_components_8conn": int(ncomp),
                "erosion": erosion, "resample_wall_s": time.perf_counter() - t0}
        self.setup_info = info
        return {"occ": occ, "label": lab.astype(np.int32), "eroded_mask": eroded.astype(np.uint8),
                "u0": float(lo[0]), "v0": float(lo[1]), "res": res_c, "S": S, "cfg": cfg, "info": info}

    def instantiate(self, state, config):
        import torch
        cfg = dict(state["cfg"], **{k: v for k, v in config.items() if k.startswith("q_")})
        p = GMC / state["info"]["raster"]
        if R._sha(p) != state["info"]["raster_sha256"]:
            raise ValueError(f"raster {p} changed since this setup was built (stale setup artifact)")
        dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        torch.set_default_device(dev)                  # the notebook does this; model internals rely on it
        if dev.type == "cpu":                          # B0: oneMKL rejects the stride-0 rfft2 input on CPU
            _rfft2 = torch.fft.rfft2
            torch.fft.rfft2 = lambda x, *a, **k: _rfft2(x.contiguous(), *a, **k)
        sys.path.insert(0, os.path.join(REPO, "examples"))
        sys.path.insert(0, os.path.join(REPO, "2D_Neural_Heuristics"))
        from models.deepnormMultiGoal import DEEPNORM2dMultiGoal
        from models.fno import FNO2d
        from astar.astar import AStar
        from astar.environment_simple import Environment2D
        weights = {}

        def load(model, name):
            w = MODELS / name / "best_model.pt"
            sha = _sha(w)
            if sha != (MODELS / name / "best_model.pt.sha256").read_text().split()[0]:
                raise ValueError(f"{w}: SHA-256 differs from the B0 sidecar")
            weights[name] = sha
            model.load_state_dict(torch.load(w, map_location=dev, weights_only=True))
            return model.to(dev).eval()
        t0 = time.perf_counter()
        sdf_model = load(FNO2d(4, 1, 8, 8, 16), "FNOSDF")
        model = load(DEEPNORM2dMultiGoal(4, 8, 8, 16), str(cfg["model"]))
        S = state["S"]
        with torch.no_grad():
            m = torch.tensor(state["eroded_mask"], dtype=torch.float, device=dev).reshape(1, S, S, 1)
            sdf = sdf_model(m)
            chi = torch.mul(torch.tanh(sdf * float(cfg["smooth_coef"])), (m - 0.5)) + 0.5     # notebook smooth_chi
            if dev.type == "cuda":
                torch.cuda.synchronize()
        self.instance_info = dict(state["info"], weights_sha256=weights, device=str(dev),
                                  gpu=torch.cuda.get_device_name(0) if dev.type == "cuda" else None,
                                  torch=torch.__version__, chi_wall_s=time.perf_counter() - t0,
                                  sdf_cells_p50_p99=[float(x) for x in np.percentile(
                                      sdf.detach().cpu().numpy()[m.cpu().numpy() > 0], [50, 99])]
                                  if bool((m > 0).any()) else None)
        v_unit = S / 64. if cfg["v_unit"] == "S/64" else float(cfg["v_unit"])
        I, Jx = np.meshgrid(np.arange(S), np.arange(S), indexing="ij")
        return {"torch": torch, "dev": dev, "model": model, "chi": chi, "state": state, "cfg": cfg,
                "AStar": AStar, "Env": Environment2D, "v_unit": v_unit, "IJ": np.stack([I, Jx], -1)}

    def _cell(self, st, uv, cfg):
        """(row, col) of the free cell for an endpoint, + snap distance (0 if its own cell is free)."""
        info = {"u0": st["u0"], "v0": st["v0"], "res_m": st["res"]}
        iy, ix = (int(v[0]) for v in R.to_index(info, uv))
        S = st["S"]
        if 0 <= iy < S and 0 <= ix < S and st["occ"][iy, ix] == 0:
            c = R.centre(info, iy, ix)[0]
            return (iy, ix), float(np.linalg.norm(c - np.asarray(uv, float))), False
        hit = R.nearest_free({"occ": st["occ"]}, info, uv, max_m=float(cfg["snap_max_m"]))
        if hit is None:
            return None, None, True
        return (hit[0], hit[1]), hit[3], True

    def plan(self, live, start_uv, goal_uv, query):
        torch, st, cfg = live["torch"], live["state"], live["cfg"]
        t0 = time.perf_counter()
        s, s_d, s_moved = self._cell(st, start_uv, cfg)
        g, g_d, g_moved = self._cell(st, goal_uv, cfg)
        info = {"start_snap_m": s_d, "goal_snap_m": g_d, "start_moved": s_moved, "goal_moved": g_moved,
                "start_cell": s, "goal_cell": g}
        stages = {"snap_s": time.perf_counter() - t0}
        if s is None or g is None:
            return {"claimed": False, "claimed_reason": "endpoint_no_free_cell", "path_uv": None, "stages": stages,
                    "info": info}
        if st["label"][s] != st["label"][g]:
            return {"claimed": False, "claimed_reason": "astar_no_path_component", "path_uv": None,
                    "stages": stages, "info": info}
        S = st["S"]
        t1 = time.perf_counter()
        with torch.no_grad():
            goal = torch.tensor([[g[1], g[0]]], dtype=torch.long, device=live["dev"])     # model goal = (col, row)
            V = live["model"](live["chi"], goal)
            if live["dev"].type == "cuda":
                torch.cuda.synchronize()
        V = V.detach().cpu().numpy().reshape(S, S)
        stages["value_infer_s"] = time.perf_counter() - t1
        t2 = time.perf_counter()
        mask = 1 - st["occ"]
        V = V * live["v_unit"] / (mask + 10e-10)                                  # planningoperator: V / (mask + 1e-9)
        eucl = np.linalg.norm(live["IJ"] - np.array(g)[None, None, :], axis=-1)   # euclideannorm, (row, col) cells
        h = np.maximum(eucl, V)
        env = live["Env"](np.array(g), st["occ"], h)
        stages["heuristic_s"] = time.perf_counter() - t2
        t3 = time.perf_counter()
        cost, path, _, expands, _ = live["AStar"].plan(np.array(s), env)
        stages["astar_s"] = time.perf_counter() - t3
        info.update(astar_expansions=int(expands), astar_cost_cells=float(cost),
                    v_at_start_cells=float(V[s]), eucl_at_start_cells=float(eucl[s]))
        if not math.isfinite(cost) or not len(path):
            return {"claimed": False, "claimed_reason": "astar_no_path", "path_uv": None, "stages": stages,
                    "info": info}
        cells = np.asarray([tuple(c) for c in path], np.int64)
        info["grid_path_cells"] = int(len(cells))
        if cfg["collinear_merge"]:
            cells = collinear_merge(cells)
        uv = np.c_[st["u0"] + (cells[:, 1] + .5) * st["res"], st["v0"] + (cells[:, 0] + .5) * st["res"]]
        return {"claimed": True, "claimed_reason": "astar_path", "path_uv": uv.tolist(), "stages": stages,
                "info": info}
