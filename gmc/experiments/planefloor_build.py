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

from gmc.height.planefloor import (build_plane_floor, load_plane_scene,
                                   monotone_toward_floor, open_floor_cell_mask,
                                   phantom_cell_mask, save_plane_scene, speckle_stats)

from showcase_scene import (BANDS, CELL, DATA, RHO, TAU, _occupancy, load_processed)

RES = Path("results/height/plane")
FIGS = RES / "figs"
RAW = Path("/scratch/wg2381/splathjb/gmc/outputs/height/plane")
# `DATA` (splatc_atlas) is read-only for this chain, so the edited scene lives under RAW.
SCENE_NPZ = RAW / "processed_planefloor.npz"

# P1b parameters. `spacing`/`sigma_n` are the plane's; `max_top` is the rule's.
P1B = {"max_top": 0.10, "level": RHO, "spacing": 1.0, "sigma_n": 0.004,
       "opacity": 0.95, "z_lo": 0.02, "margin": 0.005}

# The two admissible footprint masks, least aggressive first. `phantom` is the literal §P1b
# wording; `open_floor` drops its "occupied in the low band" half and keeps the overhead-free
# guarantee that protects real geometry. See planefloor.replace_mask for why the literal wording
# cannot work on this scene.
VARIANTS = ("phantom", "open_floor")

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


def _p1a_bands():
    """The bands P1a re-rasterised and proved identical, cell for cell, to `maps.npz`."""
    d = np.load(RAW / "p1a_bands.npz")
    ext = [float(v) for v in d["extent"]]
    occ = {b: d[f"occ_{b[0]:.2f}_{b[1]:.2f}"] for b in BANDS}
    return ext, occ, d["phantom"]


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


# ----------------------------------------------------------------------------- P1b --


def _measure_inserted(scene, first_id, z_f):
    """The trap in P1b, measured on the built scene rather than inferred from sigma_n."""
    ins = scene.subset(scene.ids >= first_id)
    tops = ins.aabb(RHO)[1][:, 2]
    return {"n_inserted": len(ins),
            "inserted_rho_top_max_above_floor": float(tops.max() - z_f),
            "inserted_rho_top_p99_above_floor": float(np.percentile(tops - z_f, 99)),
            "z_lo": P1B["z_lo"],
            "invisible_to_every_robot_band": bool(tops.max() < z_f + P1B["z_lo"])}


def step_build(variant=None, max_top=None):
    t0 = time.time()
    scene, g0 = load_processed()
    floor = g0["floor"]
    z_f = floor["z_floor"]
    ext, occ0, phantom = _p1a_bands()
    openf = open_floor_cell_mask([occ0[b] for b in BANDS[1:]])
    before = _band_table(occ0)
    kw = dict(P1B)
    if max_top is not None:
        kw["max_top"] = float(max_top)
    print(f"loaded {len(scene):,} splats in {time.time() - t0:.1f}s; "
          f"phantom {int(phantom.sum()):,} cells, open floor {int(openf.sum()):,} cells", flush=True)

    masks = {"phantom": phantom, "open_floor": openf}
    rows, scenes = {}, {}
    for name in VARIANTS:
        t1 = time.time()
        built, st = build_plane_floor(scene, masks[name], ext, CELL, floor=floor, **kw)
        occ1 = _band_maps(built, z_f, ext)
        bt = _band_table(occ1)
        low, above = occ1[BANDS[0]], bt[f"occ_{BANDS[1][0]:.2f}_{BANDS[1][1]:.2f}"]
        st.update(
            footprint_mask=name,
            footprint_mask_cells=int(masks[name].sum()),
            band_occupancy_frac_before=before,
            band_occupancy_frac_after=bt,
            # monotone_toward_floor reads top band first, floor band last
            monotone_toward_floor=monotone_toward_floor(list(bt.values())[::-1]),
            low_band_after=bt[f"occ_{BANDS[0][0]:.2f}_{BANDS[0][1]:.2f}"],
            band_above_after=above,
            phantom_cells_still_occupied=int((phantom & low).sum()),
            phantom_cells_cleared_frac=float(1.0 - (phantom & low).sum() / phantom.sum()),
            residual=speckle_stats(phantom & low, CELL),
            measured_on_built_scene=_measure_inserted(built, st["first_inserted_id"], z_f),
            seconds=time.time() - t1)
        rows[name] = st
        scenes[name] = built
        print(f"[{name}] replaced {st['replaced']:,} splats -> low band "
              f"{st['low_band_after']:.4f} (was {before['occ_0.02_0.10']:.4f}, band above "
              f"{above:.4f}), monotone={st['monotone_toward_floor']}, phantom cleared "
              f"{st['phantom_cells_cleared_frac']:.1%}, inserted top "
              f"{st['measured_on_built_scene']['inserted_rho_top_max_above_floor']:.4f} m "
              f"above floor in {st['seconds']:.1f}s", flush=True)

    if variant is None:
        ok = [n for n in VARIANTS if rows[n]["monotone_toward_floor"]]
        chosen = ok[0] if ok else VARIANTS[-1]
    else:
        chosen = variant
    st = rows[chosen]
    built = scenes[chosen]
    meta = {"claims_boundary": CAPTION, "variant": chosen, "params": st["params"],
            "floor": floor, "z_floor": z_f, "ceiling_height_m": g0["ceiling_height_m"],
            "extent": ext, "gravity_rotation": g0["gravity_rotation"],
            "base_scene": "splatc_atlas/.../showcase/processed.npz via load_processed() "
                          "(Amendment 1 floor rule ON), then Amendment 3 P1b",
            "n_splats": len(built), "replaced": st["replaced"], "inserted": st["inserted"],
            "first_inserted_id": st["first_inserted_id"],
            "band_occupancy_frac_after": st["band_occupancy_frac_after"]}
    save_plane_scene(SCENE_NPZ, built, meta)
    print(f"wrote {SCENE_NPZ} ({SCENE_NPZ.stat().st_size / 1e9:.2f} GB)", flush=True)

    # Re-open what was written and re-measure the trap on the reloaded scene.
    again, meta2 = load_plane_scene(SCENE_NPZ)
    reread = _measure_inserted(again, meta2["first_inserted_id"], z_f)
    assert reread["n_inserted"] == st["inserted"], (reread, st["inserted"])
    del again

    out = {"claims_boundary": CAPTION,
           "task": "P1b: replace the phantom near-floor splats with one analytic floor plane",
           "chosen_variant": chosen,
           "chosen_because": ("least aggressive footprint mask whose low band stops being the "
                              "most occupied one" if variant is None else "forced on the CLI"),
           "monotone_target": {"band_above_0.10_0.55_before": before["occ_0.10_0.55"],
                               "low_band_0.02_0.10_before": before["occ_0.02_0.10"]},
           "scene_npz": str(SCENE_NPZ), "loader": "gmc.height.planefloor.load_plane_scene",
           "meta": meta, "reread_from_disk": reread,
           "variants": rows, "seconds": time.time() - t0}
    _dump(RES / "p1b_build.json", out)

    _fig_build(ext, occ0, phantom, scenes, rows, chosen, z_f)
    print(json.dumps({"chosen": chosen, "low_band": st["low_band_after"],
                      "band_above": st["band_above_after"],
                      "monotone": st["monotone_toward_floor"],
                      "inserted_top_above_floor": st["measured_on_built_scene"][
                          "inserted_rho_top_max_above_floor"],
                      "replaced": st["replaced"], "inserted": st["inserted"]}, indent=2), flush=True)


def _fig_build(ext, occ0, phantom, scenes, rows, chosen, z_f):
    """Before / after / what is left, on the same axes as diagnosis fig1."""
    e = [ext[0], ext[2], ext[1], ext[3]]
    low0 = occ0[BANDS[0]]
    panels = [("sweeper band 0.02-0.10 m, as built\n"
               f"{low0.mean():.1%} occupied; red = phantom ({phantom.sum() * CELL ** 2:.0f} m2)",
               low0 & ~phantom, phantom)]
    for name in VARIANTS:
        low1 = _occupancy(scenes[name], z_f, BANDS[0], ext)
        res = phantom & low1
        panels.append((f"after the '{name}' rule{' (CHOSEN)' if name == chosen else ''}\n"
                       f"{low1.mean():.1%} occupied vs {rows[name]['band_above_after']:.1%} in the "
                       f"band above; {res.sum() * CELL ** 2:.0f} m2 phantom left",
                       low1 & ~res, res))
    fig, axs = plt.subplots(1, len(panels), figsize=(7.2 * len(panels), 11))
    for ax, (title, real, red) in zip(np.atleast_1d(axs), panels):
        img = np.ones(real.shape[::-1] + (3,))
        img[real.T] = (0.23, 0.23, 0.22)
        img[red.T] = (0.82, 0.23, 0.23)
        ax.imshow(img, origin="lower", extent=e, interpolation="nearest")
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.grid(alpha=0.25, lw=0.4)
    fig.suptitle("Amendment 3 P1b - " + CAPTION, fontsize=10)
    plt.tight_layout()
    plt.savefig(FIGS / "p1b_low_band_before_after.png", dpi=95)
    plt.close()
    print(f"wrote {FIGS / 'p1b_low_band_before_after.png'}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", required=True,
                    choices=["diagnose", "build", "survivors", "evidence"])
    ap.add_argument("--variant", choices=VARIANTS, help="force the P1b footprint mask")
    ap.add_argument("--max-top", type=float, help="override the P1b rho-top ceiling (m)")
    a = ap.parse_args()
    RES.mkdir(parents=True, exist_ok=True)
    FIGS.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    if a.step == "diagnose":
        step_diagnose()
    elif a.step == "build":
        step_build(variant=a.variant, max_top=a.max_top)


if __name__ == "__main__":
    main()
