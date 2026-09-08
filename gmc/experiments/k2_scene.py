"""Load a window of the real K2 3DGS slab as a gmc ``SceneModel2D``.

The slab (``splatc_atlas/data/gs_scenes/k2/slab_2d.npz``) holds 656,487 2D
Gaussians obtained by conditioning the cleaned 3D splats onto a robot-height
slab; see ``splatc_atlas/docs/worklog/gmc_M0_k2.md`` for the pipeline.  Fields:

    mean2   (N, 2) float32   2D means in the raw PLY frame
    cov3    (N, 3) float32   packed [xx, xy, yy]
    weight  (N,)   float32   opacity x height density

Scale is ``1 unit ~= 0.5 m`` and is marked UNVERIFIED in ``meta.json``; every
length printed by the experiments is therefore reported in units first.

The solidity semantics (``weight > W_OBST``, iso-level ``RHO``) are inherited
from the atlas H2 experiment deliberately, so that anything measured here can
be put beside ``splatc_atlas/results/gmc_h2/k2_windows_v2.json`` without a
semantics mismatch.

KNOWN LIMITATION carried from ``meta.json``: wall openings may include glass or
scan holes, i.e. free space that is not really free (false-free), and the
wall-gap audit has not been run.  Nothing measured on this scene may be turned
into a planning or compression claim until that audit exists.  These
experiments test whether the compiler *runs* on real data, not whether its
answer about this building is true.
"""
from pathlib import Path

import numpy as np
from shapely.geometry import box as shapely_box

from gmc.types import GaussianSupport2D, SceneModel2D

SLAB = (Path(__file__).resolve().parents[2]
        / "splatc_atlas" / "data" / "gs_scenes" / "k2" / "slab_2d.npz")

RHO = 2.0                       # iso-level, matches configs' support_level_scene
W_OBST = 0.3                    # solidity threshold, matches atlas h2_k2_windows
WALL_YAW = np.deg2rad(60.75)    # dominant wall bearing from meta.json calibration
UNITS_PER_M = 2.0               # 1 unit ~= 0.5 m (UNVERIFIED)

DOORS = {
    "door_A": (22.3, -15.2),
    "door_B": (15.2, -11.0),
}


def load_window(center, half: float, *, w_min: float = W_OBST,
                level: float = RHO, min_eig: float = 1e-10,
                margin: float = 0.75, name: str | None = None):
    """Window of the K2 slab as a scene, plus the counts that were dropped.

    Supports are taken from a box of half-width ``half + margin`` but the
    workspace is the box of half-width ``half``: a splat whose mean sits just
    outside the workspace can still intrude into it, so dropping it would
    silently open free space at the boundary.

    Returns ``(scene, stats)``.  ``stats`` records every filter's effect,
    because each one removes obstacle mass and so can only move the answer
    towards REACHABLE -- they are scene definitions, not speed knobs.
    """
    cx, cy = float(center[0]), float(center[1])
    d = np.load(SLAB)
    mu = d["mean2"].astype(np.float64)
    cov3 = d["cov3"].astype(np.float64)
    w = d["weight"].astype(np.float64)

    in_box = ((np.abs(mu[:, 0] - cx) <= half + margin)
              & (np.abs(mu[:, 1] - cy) <= half + margin))
    keep = in_box & (w > float(w_min))
    idx = np.flatnonzero(keep)

    cov = np.empty((idx.size, 2, 2), dtype=np.float64)
    cov[:, 0, 0] = cov3[idx, 0]
    cov[:, 0, 1] = cov3[idx, 1]
    cov[:, 1, 0] = cov3[idx, 1]
    cov[:, 1, 1] = cov3[idx, 2]
    cov = 0.5 * (cov + cov.transpose(0, 2, 1))
    eig = np.linalg.eigvalsh(cov)
    spd = np.isfinite(eig).all(axis=1) & (eig[:, 0] > float(min_eig))

    supports = tuple(
        GaussianSupport2D(mean=mu[i], covariance=cov[k], level=float(level),
                          primitive_id=int(pid))
        for pid, (k, i) in enumerate(
            (k, i) for k, i in enumerate(idx) if spd[k]))

    ws = shapely_box(cx - half, cy - half, cx + half, cy + half)
    semi = level * np.sqrt(eig[spd])
    stats = {
        "center": [cx, cy], "half": float(half), "margin": float(margin),
        "w_min": float(w_min), "level": float(level),
        "n_in_box_any_weight": int(in_box.sum()),
        "n_after_weight_filter": int(idx.size),
        "n_dropped_not_spd": int((~spd).sum()),
        "n_supports": len(supports),
        "semi_minor_p50": float(np.median(semi[:, 0])) if semi.size else None,
        "semi_major_p50": float(np.median(semi[:, 1])) if semi.size else None,
        "semi_major_p99": (float(np.percentile(semi[:, 1], 99))
                           if semi.size else None),
        "workspace_area_u2": float(ws.area),
    }
    scene = SceneModel2D(supports=supports, workspace=ws,
                         name=name or f"k2_{cx:g}_{cy:g}_h{half:g}")
    return scene, stats


def wall_frame(scene, center, *, band: float = 0.35):
    """Unit vectors ``(along_wall, wall_normal)`` at a door centre.

    Both candidate bearings from the meta.json calibration are scored by how
    much obstacle mass lies in a thin band through the centre; the wall is the
    one the mass lines up with.  Same estimator as the atlas H2 experiment.
    """
    mu = np.array([s.mean for s in scene.supports], dtype=np.float64)
    rel = mu - np.asarray(center, dtype=np.float64)
    best, best_score = None, -np.inf
    for a in (WALL_YAW, WALL_YAW - np.pi / 2.0):
        along = np.array([np.cos(a), np.sin(a)])
        normal = np.array([-along[1], along[0]])
        in_band = np.abs(rel @ normal) < band
        if in_band.sum() < 8:
            continue
        score = float(np.var(rel[in_band] @ along) * in_band.sum())
        if score > best_score:
            best, best_score = (along, normal), score
    if best is None:
        along = np.array([np.cos(WALL_YAW), np.sin(WALL_YAW)])
        best = (along, np.array([-along[1], along[0]]))
    return best
