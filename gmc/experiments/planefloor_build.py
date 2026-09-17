# gmc/experiments/planefloor_build.py
"""Amendment 3 §P1: build the plane-floor variant of the showcase scene, and prove it.

**Claims boundary, and it is not a footnote:** the plane floor is a *user-approved manual
scene-definition change* ("地面的高斯用大平面替代，手动解决噪音 for now"). It is surgery on the
scene, not a reconstruction method and not an outer approximation with a guarantee. Everything
downstream is sound with respect to the *edited* scene only; the gap between that and the room is
the open research question (`docs/worklog/height_map_diagnosis.md` §4).

Steps, one ``--step`` per job:

``diagnose``   P1a — re-rasterise the hall at 0.05 m and reproduce the diagnosis numbers
               (94,780 phantom cells / 236.95 m2 / 367 components) as a regression check.
``build``      P1b — apply the replacement rule, insert the analytic plane, write the scene, and
               *measure* on the built scene that the inserted splats never reach ``z_floor + 0.02``.
``survivors``  P1c — table A/B/C/G legs and tops, the SW-hall bench and its end supports, the
               plinths and the reception counter: support counts before/after, one figure each.
``evidence``   P1d — band-occupancy table before/after, monotonicity, window-A projected support
               counts for sweeper and cylinder, and the criterion-3-style free-area map.
"""
import argparse
import json
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from gmc.height.planefloor import phantom_cell_mask, speckle_stats

from showcase_scene import (BANDS, CELL, DATA, RHO, TAU, _occupancy, load_processed)

RES = Path("results/height/plane")
FIGS = RES / "figs"
RAW = Path("/scratch/wg2381/splathjb/gmc/outputs/height/plane")
SCENE_NPZ = DATA / "processed_planefloor.npz"

# P1b parameters. `spacing`/`sigma_n` are the plane's; `max_top` and `cell` are the rule's.
P1B = {"cell": CELL, "max_top": 0.10, "level": RHO, "spacing": 1.0, "sigma_n": 0.004,
       "opacity": 0.95, "z_lo": 0.02, "margin": 0.005}

# The numbers `height_map_diagnosis.md` §2 reports, and the stop condition of P1a.
DIAG_EXPECT = {"phantom_cells": 94780, "phantom_m2": 236.95, "phantom_components": 367}

SURFACE, INK = "#fcfcfb", "#0b0b0b"
plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["DejaVu Sans"],
                     "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
                     "savefig.facecolor": SURFACE, "text.color": INK})

CAPTION = ("plane floor = USER-APPROVED MANUAL SCENE EDIT (Amendment 3 P1), "
           "not a reconstruction method and not a guaranteed outer approximation")


def _extent_and_bands():
    """The extent and cached band maps the diagnosis actually used."""
    d = np.load(DATA / "maps.npz")
    return [float(v) for v in d["extent"]], d


def _band_maps(scene, z_f, ext):
    return {b: _occupancy(scene, z_f, b, ext) for b in BANDS}


def _band_table(occ):
    return {f"occ_{a:.2f}_{b:.2f}": float(occ[(a, b)].mean()) for (a, b) in BANDS}


def _dump(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=2, default=float) + "\n")
    print(f"wrote {path}", flush=True)


# ----------------------------------------------------------------------------- P1a --


def step_diagnose():
    t0 = time.time()
    scene, g0 = load_processed()
    z_f = g0["floor"]["z_floor"]
    ext, cached = _extent_and_bands()
    print(f"loaded {len(scene):,} splats in {time.time() - t0:.1f}s; extent {ext}", flush=True)

    occ = _band_maps(scene, z_f, ext)
    low = occ[BANDS[0]]
    phantom = phantom_cell_mask(low, [occ[b] for b in BANDS[1:]])
    above = np.zeros_like(low)
    for b in BANDS[1:]:
        above |= occ[b]

    ph = speckle_stats(phantom, CELL)
    fur = speckle_stats(occ[(0.55, 1.00)], CELL)
    got = {"phantom_cells": ph["cells"], "phantom_m2": round(ph["area_m2"], 2),
           "phantom_components": ph["components"]}
    reproduced = (got["phantom_cells"] == DIAG_EXPECT["phantom_cells"]
                  and abs(got["phantom_m2"] - DIAG_EXPECT["phantom_m2"]) < 0.01
                  and got["phantom_components"] == DIAG_EXPECT["phantom_components"])

    # Cell-for-cell agreement with the cached maps the diagnosis read, per band.
    cache_diff = {f"occ_{a:.2f}_{b:.2f}": int((occ[(a, b)] != cached[f"occ_{a:.2f}_{b:.2f}"]).sum())
                  for (a, b) in BANDS}

    # Why the low band is so full: the fitted floor plane is tilted 0.285 deg, which over a 25 m
    # hall is +-6.4 cm -- most of the [0.02, 0.10] band. A horizontal band over a tilted floor
    # eats the floor surface itself wherever the plane rises above z_floor + 0.02.
    n = np.asarray(g0["floor"]["normal"], float)
    n /= np.linalg.norm(n)
    c0 = np.asarray(g0["floor"]["centroid"], float)
    nx, ny = low.shape
    gx = ext[0] + (np.arange(nx) + 0.5) * CELL
    gy = ext[1] + (np.arange(ny) + 0.5) * CELL
    XX, YY = np.meshgrid(gx, gy, indexing="ij")
    plane_h = (c0[2] - (n[0] * (XX - c0[0]) + n[1] * (YY - c0[1])) / n[2]) - z_f
    open_floor = ~above
    edges = [-0.08, -0.04, -0.02, 0.0, 0.02, 0.04, 0.08]
    tilt_rows = []
    for a, b in zip(edges[:-1], edges[1:]):
        sel = open_floor & (plane_h >= a) & (plane_h < b)
        tilt_rows.append({"fitted_plane_above_z_floor_m": [a, b], "open_floor_cells": int(sel.sum()),
                          "phantom_frac": float(phantom[sel].mean()) if sel.any() else None})

    out = {
        "claims_boundary": CAPTION,
        "task": "P1a regression check: reproduce the diagnosis phantom numbers by re-rasterising",
        "scene": {"loader": "showcase_scene.load_processed() (Amendment 1 floor rule ON)",
                  "n_splats": len(scene), "z_floor": z_f, "cell_m": CELL, "rho": RHO, "tau": TAU,
                  "extent": ext, "bands": [list(b) for b in BANDS]},
        "expected": DIAG_EXPECT, "got": got, "reproduced": bool(reproduced),
        "phantom": ph, "furniture_band_0.55_1.00": fur,
        "band_occupancy_frac": _band_table(occ),
        "open_floor_cells": int(open_floor.sum()),
        "phantom_frac_of_open_floor": float(phantom.sum() / open_floor.sum()),
        "phantom_frac_of_all_low_band": float(phantom.sum() / low.sum()),
        "recomputed_vs_cached_maps_npz_differing_cells": cache_diff,
        "floor_tilt": {
            "tilt_deg": g0["floor"]["tilt_deg"], "normal": n.tolist(),
            "fitted_plane_height_above_z_floor_m": [float(plane_h.min()), float(plane_h.max())],
            "note": ("the bands are horizontal slabs above the scalar z_floor, but the fitted floor "
                     "is a tilted plane; where the plane rises above z_floor + 0.02 the band "
                     "contains the floor surface itself"),
            "phantom_rate_by_plane_height": tilt_rows},
        "seconds": time.time() - t0,
    }
    _dump(RES / "p1a_diagnosis_repro.json", out)
    np.savez_compressed(RAW / "p1a_bands.npz", extent=np.array(ext), phantom=phantom,
                        **{f"occ_{a:.2f}_{b:.2f}": occ[(a, b)] for (a, b) in BANDS})
    print(f"RAW bands -> {RAW / 'p1a_bands.npz'}", flush=True)
    print(json.dumps({"expected": DIAG_EXPECT, "got": got, "reproduced": reproduced,
                      "cache_diff": cache_diff}, indent=2), flush=True)
    if not reproduced:
        raise SystemExit("P1a STOP CONDITION: the diagnosis numbers did not reproduce")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", required=True,
                    choices=["diagnose", "build", "survivors", "evidence"])
    a = ap.parse_args()
    RES.mkdir(parents=True, exist_ok=True)
    FIGS.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    {"diagnose": step_diagnose}[a.step]()


if __name__ == "__main__":
    main()
