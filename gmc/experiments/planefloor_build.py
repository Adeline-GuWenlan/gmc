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
from scipy import ndimage

from gmc.height.band_shadow import band_overlap_mask
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.height.planefloor import (build_plane_floor, footprint_spans, load_plane_scene,
                                   monotone_toward_floor, open_floor_cell_mask,
                                   phantom_cell_mask, replace_mask, save_plane_scene,
                                   speckle_stats)

from showcase_scene import (BANDS, CELL, DATA, RASTER, RHO, TAU, WINDOW_A, _occupancy,
                            _support_raster, load_processed)

RES = Path("results/height/plane")
FIGS = RES / "figs"
RAW = Path("/scratch/wg2381/splathjb/gmc/outputs/height/plane")
# `DATA` (splatc_atlas) is read-only for this chain, so the edited scene lives under RAW.
SCENE_NPZ = RAW / "processed_planefloor.npz"

# P1b parameters. `spacing`/`sigma_n` are the plane's; `max_top` is the rule's.
P1B = {"max_top": 0.10, "level": RHO, "spacing": 1.0, "sigma_n": 0.004,
       "opacity": 0.95, "z_lo": 0.02, "margin": 0.005}

# (overhead-free mask, footprint the splat is judged by), least aggressive first. `phantom` is
# the literal §P1b wording and `open_floor` its superset -- on this scene they are provably
# identical, and the second row is kept as the evidence for that. What is actually binding is the
# footprint: see planefloor.replace_mask.
SWEEP = (("phantom", "aabb"), ("open_floor", "aabb"), ("phantom", "shadow"),
         ("phantom", "centre"))
VARIANTS = tuple(f"{m}+{f}" for m, f in SWEEP)
NORTH_Y = 20.0       # the hall's two halves behave differently; report them apart

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
    diag = _residual_diagnostic(scene, masks["phantom"], ext, z_f, kw, phantom)
    print("residual diagnostic " + json.dumps(diag), flush=True)

    rows, scenes = {}, {}
    for mask_name, fp in SWEEP:
        name = f"{mask_name}+{fp}"
        t1 = time.time()
        built, st = build_plane_floor(scene, masks[mask_name], ext, CELL, floor=floor,
                                      footprint=fp, **kw)
        occ1 = _band_maps(built, z_f, ext)
        bt = _band_table(occ1)
        low, above = occ1[BANDS[0]], bt["occ_0.10_0.55"]
        res = phantom & low
        st.update(
            variant=name, overhead_mask=mask_name, footprint=fp,
            overhead_mask_cells=int(masks[mask_name].sum()),
            band_occupancy_frac_before=before, band_occupancy_frac_after=bt,
            # monotone_toward_floor reads top band first, floor band last
            monotone_toward_floor=monotone_toward_floor(list(bt.values())[::-1]),
            low_band_after=bt["occ_0.02_0.10"], band_above_after=above,
            phantom_cells_still_occupied=int(res.sum()),
            phantom_cells_cleared_frac=float(1.0 - res.sum() / phantom.sum()),
            residual=speckle_stats(res, CELL),
            residual_m2_by_region=_by_region(res, ext),
            residual_distance_to_real_geometry_m=_dist_to_real(res, openf),
            removed_component_m2=_removed_morphology(phantom & ~low, CELL),
            measured_on_built_scene=_measure_inserted(built, st["first_inserted_id"], z_f),
            seconds=time.time() - t1)
        rows[name] = st
        scenes[name] = built
        print(f"[{name}] replaced {st['replaced']:,} -> low band {st['low_band_after']:.4f} "
              f"(was {before['occ_0.02_0.10']:.4f}; band above {above:.4f}), "
              f"monotone={st['monotone_toward_floor']}, phantom cleared "
              f"{st['phantom_cells_cleared_frac']:.1%}, residual S/N "
              f"{st['residual_m2_by_region']['south_m2']:.0f}/"
              f"{st['residual_m2_by_region']['north_m2']:.0f} m2, inserted top "
              f"{st['measured_on_built_scene']['inserted_rho_top_max_above_floor']:.4f} m "
              f"in {st['seconds']:.1f}s", flush=True)

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
    st["equal_thickness_after"] = _equal_thickness_check(built, z_f, ext)
    st["equal_thickness_before"] = _equal_thickness_check(scene, z_f, ext)
    print("equal-thickness bands before " + json.dumps(st["equal_thickness_before"])
          + " after " + json.dumps(st["equal_thickness_after"]), flush=True)
    save_plane_scene(SCENE_NPZ, built, meta)
    print(f"wrote {SCENE_NPZ} ({SCENE_NPZ.stat().st_size / 1e9:.2f} GB)", flush=True)

    again, meta2 = load_plane_scene(SCENE_NPZ)
    reread = _measure_inserted(again, meta2["first_inserted_id"], z_f)
    assert reread["n_inserted"] == st["inserted"], (reread, st["inserted"])
    del again

    out = {"claims_boundary": CAPTION,
           "task": "P1b: replace the phantom near-floor splats with one analytic floor plane",
           "chosen_variant": chosen,
           "chosen_because": ("least aggressive (overhead mask, footprint) pair whose low band "
                              "stops being the most occupied" if variant is None
                              else "forced on the CLI"),
           "monotone_target": {"band_above_0.10_0.55_before": before["occ_0.10_0.55"],
                               "low_band_0.02_0.10_before": before["occ_0.02_0.10"]},
           "residual_diagnostic": diag,
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


def _dist_to_real(residual, open_floor):
    """How far is the residual from geometry that has mass above 0.10 m?

    This decides P1's verdict, so it is measured rather than asserted. If what survives the rule
    hugs real geometry, the residual is the AABB rasteriser over-approximating real near-floor
    objects (wall bases, furniture feet), not leftover phantom dust -- and no parameter setting
    can remove it without deleting the real object it belongs to.
    """
    if not residual.any():
        return None
    d = ndimage.distance_transform_edt(np.asarray(open_floor, dtype=bool)) * CELL
    v = d[residual]
    return {"percentiles_m": {str(q): float(np.percentile(v, q)) for q in (50, 75, 90, 99)},
            "frac_within_0.30m": float((v <= 0.30).mean()),
            "definition": "distance from each surviving phantom cell to the nearest cell "
                          "occupied in some band above 0.10 m"}


def _equal_thickness_check(scene, z_f, ext):
    """A monotonicity check without the band-thickness confound, as a supporting diagnostic.

    The required column compares [0.02, 0.10] (0.08 m thick) against [0.10, 0.55] (0.45 m, and so
    it catches every tabletop in the hall). This compares the sweeper band against the 0.08 m
    band directly above it, which is the like-for-like question "does occupancy fall toward the
    floor". It supplements the required column, it does not replace it.
    """
    pairs = [(0.02, 0.10), (0.10, 0.18), (0.18, 0.26)]
    fr = {f"occ_{a:.2f}_{b:.2f}": float(_occupancy(scene, z_f, (a, b), ext).mean())
          for a, b in pairs}
    vals = list(fr.values())
    return {"equal_thickness_bands_0.08m": fr,
            "decreases_toward_the_floor": bool(vals[0] <= vals[1])}


def _by_region(mask, ext):
    """The hall's north end reconstructs differently from its south; never average over both."""
    ny_split = int((NORTH_Y - ext[1]) / CELL)
    a = CELL ** 2
    return {"south_m2": float(mask[:, :ny_split].sum() * a),
            "north_m2": float(mask[:, ny_split:].sum() * a),
            "split_at_y": NORTH_Y}


def _removed_morphology(cleared, cell):
    """Is what we deleted dust or coherent objects? The reader has to be able to judge."""
    st = speckle_stats(cleared, cell)
    return {k: st[k] for k in ("cells", "area_m2", "components", "median_blob_m2",
                               "frac_blobs_le_100cm2")}


def _residual_diagnostic(scene, phantom, ext, z_f, kw, phantom_ref):
    """Why the literal rule leaves 31 % of the phantom layer: measure the footprints it judges by.

    Restricted to the splats that can actually mark the sweeper band -- opaque, in band, rho-top
    below the rule's ceiling -- this compares the width of the rho-AABB the literal rule uses
    against the width of the band-clipped shadow the certified projector uses.
    """
    band = (z_f + P1B["z_lo"], z_f + kw["max_top"])
    opq = scene.subset(scene.opacity > TAU)
    m = band_overlap_mask(opq.means[:, 2], opq.covs[:, 2, 2], band[0], band[1], RHO)
    sub = opq.subset(m)
    lo, hi = sub.aabb(RHO)
    keep = hi[:, 2] < z_f + kw["max_top"]
    sub = sub.subset(keep)
    out = {"opaque_splats_in_sweeper_band": int(m.sum()),
           "of_those_with_rho_top_below_the_rule_ceiling": int(len(sub))}
    shape = phantom.shape
    for mode in ("aabb", "shadow"):
        i0, i1, j0, j1 = footprint_spans(sub, ext, CELL, shape, level=RHO, mode=mode,
                                         band_abs=band)
        w = np.maximum(i1 - i0 + 1, j1 - j0 + 1) * CELL
        out[f"{mode}_footprint_width_m"] = {str(q): float(np.percentile(w, q))
                                            for q in (50, 90, 99, 99.9)}
        out[f"{mode}_footprint_cells"] = {"mean": float(((i1 - i0 + 1) * (j1 - j0 + 1)).mean())}
    gone = {mode: replace_mask(sub, phantom, ext, CELL, z_floor=z_f, max_top=kw["max_top"],
                               level=RHO, footprint=mode).sum()
            for mode in ("aabb", "shadow", "centre")}
    out["removable_of_those"] = {k: int(v) for k, v in gone.items()}
    return out

def _fig_build(ext, occ0, phantom, scenes, rows, chosen, z_f):
    """Before / after / what is left, on the same axes as diagnosis fig1."""
    e = [ext[0], ext[2], ext[1], ext[3]]
    low0 = occ0[BANDS[0]]
    panels = [("sweeper band 0.02-0.10 m, as built\n"
               f"{low0.mean():.1%} occupied; red = phantom ({phantom.sum() * CELL ** 2:.0f} m2)",
               low0 & ~phantom, phantom)]
    for name in (VARIANTS[0], chosen):
        low1 = _occupancy(scenes[name], z_f, BANDS[0], ext)
        res = phantom & low1
        panels.append((f"after '{name}'{' (CHOSEN)' if name == chosen else ''}\n"
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
    fig.suptitle("Amendment 3 P1b - " + CAPTION, fontsize=10, y=0.995)
    plt.tight_layout(rect=(0, 0, 1, 0.94))
    plt.savefig(FIGS / "p1b_low_band_before_after.png", dpi=95)
    plt.close()
    print(f"wrote {FIGS / 'p1b_low_band_before_after.png'}", flush=True)


# ----------------------------------------------------------------------------- P1c --

# The objects P1c must prove survive, with the local frame each is checked in. Table frames are
# the A4 `table_frames.py` 2-98 % extents quoted in percase_search_cylinder.TABLES; the rest are
# that file's OBJECTS catalogue. `bins` are the three height bands the A4 column test uses to find
# vertical members (legs, bench end supports): a 4 cm cell is a member iff it holds an opaque
# splat centre in all three.
P1C_OBJECTS = [
    ("table_A", (8.96, 7.29), -22.9, 2.55, 0.72, ((0.10, 0.30), (0.30, 0.50), (0.50, 0.68))),
    ("table_B", (-0.45, 10.81), 60.9, 2.35, 0.93, ((0.10, 0.30), (0.30, 0.50), (0.50, 0.68))),
    ("table_C", (13.28, 14.09), -21.4, 1.74, 0.68, ((0.10, 0.30), (0.30, 0.50), (0.50, 0.78))),
    ("table_G", (3.35, 22.53), 31.5, 1.64, 0.62, ((0.10, 0.30), (0.30, 0.50), (0.50, 0.68))),
    ("SW_hall_bench", (2.1, 10.5), 0.0, 2.6, 2.6, ((0.10, 0.20), (0.20, 0.30), (0.30, 0.40))),
    ("plinth_W", (-1.2, 9.0), 0.0, 1.6, 1.6, ((0.10, 0.30), (0.30, 0.50), (0.50, 0.70))),
    ("plinth_SW", (3.8, 6.5), 0.0, 1.6, 1.6, ((0.10, 0.30), (0.30, 0.50), (0.50, 0.70))),
    ("plinth_S", (8.8, 4.2), 0.0, 1.6, 1.6, ((0.10, 0.30), (0.30, 0.50), (0.50, 0.70))),
    ("plinth_E", (12.3, 11.5), 0.0, 1.6, 1.6, ((0.10, 0.30), (0.30, 0.50), (0.50, 0.70))),
    ("plinth_NW", (2.2, 16.5), 0.0, 1.6, 1.6, ((0.10, 0.30), (0.30, 0.50), (0.50, 0.70))),
    ("reception_counter", (-0.2, 1.25), 0.0, 4.0, 4.0, ((0.10, 0.30), (0.30, 0.50), (0.50, 0.70))),
]
# Height bands each object's mass is counted in. The sweeper band is the one the edit can touch;
# everything from 0.10 m up must come through untouched, and that is the actual gate.
MASS_BANDS = ((0.02, 0.10), (0.10, 0.15), (0.15, 0.50), (0.50, 1.20))
MEMBER_G = 0.04          # A4 column-test cell
FINE = 0.01              # figure raster


def _frame(angle_deg):
    a = np.deg2rad(float(angle_deg))
    v = np.array([np.cos(a), np.sin(a)])       # long axis
    return v, np.array([-v[1], v[0]])


def _crop_xy(scene, win, pad):
    """Splats whose rho-AABB can reach `win` grown by `pad`; project_scene would drop the rest."""
    lo, hi = scene.aabb(RHO)
    m = ((hi[:, 0] >= win[0] - pad) & (lo[:, 0] <= win[2] + pad)
         & (hi[:, 1] >= win[1] - pad) & (lo[:, 1] <= win[3] + pad))
    return scene.subset(m)


def _members(opq, h, C, v, u, half_v, half_u, bins):
    """A4 legs_check.py: 4 cm cells holding an opaque splat centre in each of three height bins."""
    d = opq.means[:, :2] - C
    sv, su = d @ v, d @ u
    box = (np.abs(sv) <= half_v) & (np.abs(su) <= half_u)
    gi = np.floor(sv / MEMBER_G).astype(int)
    gj = np.floor(su / MEMBER_G).astype(int)
    i0 = int(np.floor(-half_v / MEMBER_G))
    j0 = int(np.floor(-half_u / MEMBER_G))
    shape = (int(2 * half_v / MEMBER_G) + 2, int(2 * half_u / MEMBER_G) + 2)
    cols = None
    for lo, hi in bins:
        g = np.zeros(shape, bool)
        m = box & (h >= lo) & (h < hi)
        g[np.clip(gi[m] - i0, 0, shape[0] - 1), np.clip(gj[m] - j0, 0, shape[1] - 1)] = True
        cols = g if cols is None else (cols & g)
    lab, n = ndimage.label(ndimage.binary_dilation(cols, np.ones((2, 2))), np.ones((3, 3)))
    out = []
    for k in range(1, n + 1):
        ij = np.argwhere((lab == k) & cols)
        if not len(ij):
            continue
        s_v = (ij[:, 0] + i0 + 0.5) * MEMBER_G
        s_u = (ij[:, 1] + j0 + 0.5) * MEMBER_G
        rad = float(np.max(np.hypot(s_v - s_v.mean(), s_u - s_u.mean()))) + MEMBER_G
        out.append({"xy": (C + s_v.mean() * v + s_u.mean() * u).round(3).tolist(),
                    "cells": int(len(ij)), "radius": round(rad, 3),
                    "_cells_ij": set(map(tuple, ij.tolist()))})
    out.sort(key=lambda m: -m["cells"])
    return out


def _member_cell_index(opq, h, C, v, u, half_v, half_u, band):
    """Which 4 cm member cell each opaque splat in `band` falls in, or (-1, -1) for none."""
    d = opq.means[:, :2] - C
    sv, su = d @ v, d @ u
    sel = ((np.abs(sv) <= half_v) & (np.abs(su) <= half_u)
           & (h >= band[0]) & (h < band[1]))
    i0 = int(np.floor(-half_v / MEMBER_G))
    j0 = int(np.floor(-half_u / MEMBER_G))
    gi = np.floor(sv / MEMBER_G).astype(int) - i0
    gj = np.floor(su / MEMBER_G).astype(int) - j0
    return sel, gi, gj


def step_survivors():
    t0 = time.time()
    before, g0 = load_processed()
    z_f = g0["floor"]["z_floor"]
    after, meta = load_plane_scene(SCENE_NPZ)
    assert abs(meta["z_floor"] - z_f) < 1e-12, (meta["z_floor"], z_f)
    print(f"before {len(before):,} splats, after {len(after):,} "
          f"(variant {meta['variant']}) in {time.time() - t0:.1f}s", flush=True)
    robots = robot_table(1.20)
    kept_ids = np.isin(before.ids, after.ids)
    rows, gate = {}, {}
    for name, C, ang, length, width, bins in P1C_OBJECTS:
        t1 = time.time()
        C = np.asarray(C, float)
        v, u = _frame(ang)
        half_v, half_u = 0.5 * length, 0.5 * width
        reach = float(np.hypot(half_v, half_u))
        win = [float(C[0] - reach - 0.5), float(C[1] - reach - 0.5),
               float(C[0] + reach + 0.5), float(C[1] + reach + 0.5)]
        row = {"centre": C.tolist(), "long_axis_deg": ang, "length_m": length,
               "width_m": width, "window": win, "member_bins": [list(b) for b in bins]}

        opq_m = before.opacity > TAU
        opq = before.subset(opq_m)
        h = opq.means[:, 2] - z_f
        okept = kept_ids[opq_m]
        aopq = after.subset(after.opacity > TAU)
        ah = aopq.means[:, 2] - z_f
        d = opq.means[:, :2] - C
        inbox = (np.abs(d @ v) <= half_v) & (np.abs(d @ u) <= half_u)
        row["opaque_splats_in_footprint"] = {
            f"h_{a:.2f}_{b:.2f}": {"before": int((inbox & (h >= a) & (h < b)).sum()),
                                   "after": int((inbox & (h >= a) & (h < b) & okept).sum())}
            for a, b in MASS_BANDS}

        members = _members(opq, h, C, v, u, half_v, half_u, bins)
        row["vertical_members_found"] = len(members)

        maps = {}
        for key in ("sweeper", "cylinder"):
            rob = robots[key]
            pad = rob.max_radius() + 1.0
            for tag, sc in (("before", before), ("after", after)):
                s2, pst = project_scene(_crop_xy(sc, win, pad), rob, win, z_floor=z_f)
                occ = _support_raster(s2, win, cell=RASTER)
                maps[key, tag] = (s2, occ)
                ctr = np.array([s.mean for s in s2.supports]).reshape(-1, 2)
                dd = ctr - C if len(ctr) else np.zeros((0, 2))
                inb = ((np.abs(dd @ v) <= half_v) & (np.abs(dd @ u) <= half_u)
                       if len(ctr) else np.zeros(0, bool))
                row[f"{key}_{tag}"] = {"supports_in_window": len(s2.supports),
                                       "supports_in_footprint": int(inb.sum()),
                                       "window_occupied_frac": float(occ.mean()),
                                       "projection_kept": pst["kept"]}
        for m in members:
            for key in ("sweeper", "cylinder"):
                for tag in ("before", "after"):
                    s2, occ = maps[key, tag]
                    gx = win[0] + (np.arange(occ.shape[0]) + 0.5) * RASTER
                    gy = win[1] + (np.arange(occ.shape[1]) + 0.5) * RASTER
                    XX, YY = np.meshgrid(gx, gy, indexing="ij")
                    disc = np.hypot(XX - m["xy"][0], YY - m["xy"][1]) <= m["radius"]
                    m[f"{key}_{tag}_occupied_frac_in_disc"] = round(float(occ[disc].mean()), 4)
                    ctr = np.array([s.mean for s in s2.supports]).reshape(-1, 2)
                    near = (np.hypot(*(ctr - m["xy"]).T) <= m["radius"] if len(ctr)
                            else np.zeros(0, bool))
                    m[f"{key}_{tag}_supports_in_disc"] = int(np.sum(near))
        row["members"] = [{k: val for k, val in m.items() if k != "_cells_ij"}
                          for m in members]

        # The gate measures the *member*, not the member's disc. A member disc is a dilated blob
        # of 4 cm cells plus a margin, so for a plinth or the counter it is up to 1.74 m across
        # and is mostly floor; the floor dust legitimately removed inside it says nothing about
        # whether the object survived. The tell that this is the right reading: discs tight enough
        # to be the member (radius <= 0.24 m) come out bit-identical before and after.
        after_members = _members(aopq, ah, C, v, u, half_v, half_u, bins)
        same_members = ([sorted(m["_cells_ij"]) for m in members]
                        == [sorted(m["_cells_ij"]) for m in after_members])
        sel, gi, gj = _member_cell_index(opq, h, C, v, u, half_v, half_u, MASS_BANDS[0])
        mcells = set().union(*[m["_cells_ij"] for m in members]) if members else set()
        in_member = np.array([(int(a), int(b)) in mcells
                              for a, b in zip(gi[sel], gj[sel])], dtype=bool) \
            if sel.any() else np.zeros(0, bool)
        nf_before = int(in_member.sum())
        nf_after = int((in_member & okept[sel]).sum())
        row["member_cell_sweeper_band_splats"] = {"before": nf_before, "after": nf_after}
        mass_ok = all(row["opaque_splats_in_footprint"][f"h_{a:.2f}_{b:.2f}"]["after"]
                      >= row["opaque_splats_in_footprint"][f"h_{a:.2f}_{b:.2f}"]["before"]
                      for a, b in MASS_BANDS[1:])
        nf_ok = nf_after >= nf_before
        # "still blocks", not "blocks": a member that had no sweeper support before the edit
        # cannot have lost one. The reception counter has a 1-cell member at (-1.94, 0.27) that
        # was 0 -> 0, and an absolute >= 1 threshold failed the whole object on it.
        blocks_ok = all(m["sweeper_after_supports_in_disc"] >= 1 for m in members
                        if m["sweeper_before_supports_in_disc"] >= 1)
        row["gate"] = {"no_mass_lost_above_0.10m": bool(mass_ok),
                       "member_cells_identical": bool(same_members),
                       "no_member_sweeper_band_splat_lost": bool(nf_ok),
                       "every_member_still_blocks_the_sweeper": bool(blocks_ok),
                       "pass": bool(mass_ok and same_members and nf_ok and blocks_ok)}
        row["disc_occupied_fraction_is_information_not_a_gate"] = (
            "a member disc includes the floor around the member; dust removed there is the point "
            "of the edit, not damage. Compare discs with radius <= 0.24 m, which are the member.")
        gate[name] = row["gate"]["pass"]
        row["seconds"] = time.time() - t1
        rows[name] = row
        print(f"[{name}] members {len(members)} gate {row['gate']} "
              f"sweeper supports {row['sweeper_before']['supports_in_footprint']}->"
              f"{row['sweeper_after']['supports_in_footprint']} in footprint, window "
              f"{row['sweeper_before']['supports_in_window']}->"
              f"{row['sweeper_after']['supports_in_window']} in {row['seconds']:.1f}s", flush=True)
        _fig_object(name, row, members, maps, win, C, v, u, half_v, half_u, meta)

    out = {"claims_boundary": CAPTION,
           "task": "P1c: prove nothing real died (Amendment 3 P1c)",
           "collateral": _collateral(before, after, z_f),
           "method": "A4 outputs/height/a4/legs_check.py, generalised over the P1c object list",
           "scene": {"before": "showcase_scene.load_processed()", "after": str(SCENE_NPZ),
                     "variant": meta["variant"], "params": meta["params"]},
           "gate_definition": {
               "no_mass_lost_above_0.10m": "opaque splat count in the object footprint, per height "
                                           "band above 0.10 m, must not decrease",
               "member_cells_identical": "the three-bin column test must find the same member "
                                         "cells on the edited scene as on the original",
               "no_member_sweeper_band_splat_lost": "opaque splats at 0.02-0.10 m whose centre "
                                                    "lies in a member cell must not decrease -- "
                                                    "the member's own near-floor mass",
               "every_member_still_blocks_the_sweeper": "each member that had >= 1 sweeper "
                                                        "support in its disc before still has "
                                                        "one; a member with none before cannot "
                                                        "have lost one",
               "not_a_gate": "disc occupied fraction: a disc is the member plus a margin of floor "
                             "(up to 1.74 m across for the counter), so dust removed inside it is "
                             "the edit working, not damage"},
           "all_objects_pass": bool(all(gate.values())), "pass_by_object": gate,
           "objects": rows, "seconds": time.time() - t0}
    _dump(RES / "p1c_survivors.json", out)
    print(json.dumps({"all_objects_pass": out["all_objects_pass"], "by_object": gate}, indent=2),
          flush=True)
    if not out["all_objects_pass"]:
        raise SystemExit("P1c STOP CONDITION: an object lost mass; tighten the rule")


def _collateral(before, after, z_f):
    """What the edit deleted, by height, hall-wide -- the cost the reader has to be able to see.

    Every removed splat is by construction confined below `z_floor + 0.10` with nothing above the
    cell it sat in, so the collateral this cannot rule out is a *real* object entirely below about
    10 cm with nothing over it -- a kerb, a threshold strip, a cable cover. In this gallery that is
    accepted as part of the user-approved manual edit; it is quantified here rather than implied.
    """
    gone = before.subset(~np.isin(before.ids, after.ids))
    h = gone.means[:, 2] - z_f
    opq = gone.opacity > TAU
    return {"removed_total": len(gone), "removed_opaque": int(opq.sum()),
            "removed_opaque_centre_height_above_floor_m": {
                str(q): float(np.percentile(h[opq], q)) for q in (1, 50, 90, 99)},
            "removed_opaque_rho_top_above_floor_m": {
                str(q): float(np.percentile(gone.subset(opq).aabb(RHO)[1][:, 2] - z_f, q))
                for q in (50, 90, 99, 100)},
            "note": "all removed splats have a rho-top below z_floor + max_top and nothing above "
                    "the cell they sat in; a real object entirely below that height with nothing "
                    "over it would be deleted too, and cannot be distinguished from dust here"}


def _fig_object(name, row, members, maps, win, C, v, u, half_v, half_u, meta):
    fig, axs = plt.subplots(1, 2, figsize=(19, 9.2))
    for ax, tag in zip(axs, ("before", "after")):
        s2, occ = maps["sweeper", tag]
        r = occ.copy()
        r[0, :] = r[-1, :] = r[:, 0] = r[:, -1] = False      # drop _support_raster's border
        img = np.ones(r.shape[::-1] + (3,))
        img[r.T] = (0.80, 0.12, 0.12)
        ax.imshow(img, origin="lower", extent=[win[0], win[2], win[1], win[3]],
                  interpolation="nearest")
        for m in members:
            ax.add_patch(plt.Circle(m["xy"], m["radius"], fill=False, color="#0b8ec9", lw=1.6))
        corners = [C + a * half_v * v + b * half_u * u
                   for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1))]
        ax.plot(*np.array(corners).T, "--", color="#1a1a8c", lw=1.2)
        ax.set_title(f"{name} sweeper map {tag}: {row[f'sweeper_{tag}']['supports_in_window']} "
                     f"supports in window, {row[f'sweeper_{tag}']['supports_in_footprint']} in the "
                     f"dashed footprint\ncyan = vertical members from the 3-bin column test "
                     f"({len(members)} found); red = projected support raster at {RASTER} m",
                     fontsize=10)
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.grid(alpha=0.3, lw=0.4)
    fig.suptitle(f"Amendment 3 P1c - {name} - variant {meta['variant']} - {CAPTION}",
                 fontsize=9, y=0.995)
    plt.tight_layout(rect=(0, 0, 1, 0.94))
    plt.savefig(FIGS / f"p1c_{name}.png", dpi=85)
    plt.close()


# ----------------------------------------------------------------------------- P1d --

# Amendment 1 window A, the window every before/after projection comparison in this series uses.
CRIT3_R = 0.5            # spec 5.3 criterion 3 radius, for the free-area map
EQUAL_BANDS = ((0.02, 0.10), (0.10, 0.18), (0.18, 0.26))


def _certified_band_raster(scene, band, win, z_f, robot_r=0.0):
    """Occupancy of one height band on the *certified* raster, not the AABB aid.

    The band table P1 is judged by comes from `showcase_scene._occupancy`, which marks the whole
    xy bounding box of every splat and calls itself "a selection aid only". The map GMC actually
    consumes is built by `project_scene` from band-clipped outer shadow *ellipses* and rasterised
    by `_support_raster`. Both are reported, because the difference between them is most of what
    is left in the low band after P1b.
    """
    rob = _band_prism(band, robot_r)
    s2, pst = project_scene(_crop_xy(scene, win, robot_r + 1.0), rob, win, z_floor=z_f)
    occ = _support_raster(s2, win, cell=RASTER)
    inner = occ[1:-1, 1:-1]          # _support_raster forces the window border occupied
    return {"n_supports": len(s2.supports), "occupied_frac": float(inner.mean()),
            "projection_kept": pst["kept"]}, s2, occ


def _band_prism(band, radius):
    """A zero-radius prism for the band, so the raster measures the band and not a robot.

    This is a measuring instrument for the evidence table, not a planning robot: the frozen robot
    table (spec 3.3) is untouched and no run uses this.
    """
    from gmc.height.prism import ellipse_prism
    r = max(float(radius), 1e-3)
    return ellipse_prism(r, r, float(band[0]), float(band[1]),
                         f"band_{band[0]:.2f}_{band[1]:.2f}")


def step_evidence():
    t0 = time.time()
    before, g0 = load_processed()
    z_f = g0["floor"]["z_floor"]
    after, meta = load_plane_scene(SCENE_NPZ)
    ext, occ0, phantom = _p1a_bands()
    robots = robot_table(1.20)
    print(f"before {len(before):,}, after {len(after):,} (variant {meta['variant']}) "
          f"in {time.time() - t0:.1f}s", flush=True)

    # (1) the required band-occupancy table, on the same AABB aid the 40.7/17.4 column came from
    occ1 = _band_maps(after, z_f, ext)
    bt0, bt1 = _band_table(occ0), _band_table(occ1)
    table = {"aid": "showcase_scene._occupancy, xy-AABB of every opaque in-band splat, 0.05 m "
                    "cells; the same selection aid the 40.7/17.4/13.3/11.7/11.1 column came from",
             "before_frac": bt0, "after_frac": bt1,
             "monotone_before": monotone_toward_floor(list(bt0.values())[::-1]),
             "monotone_after": monotone_toward_floor(list(bt1.values())[::-1]),
             "low_band_vs_band_above": {
                 "before": bt0["occ_0.02_0.10"] / bt0["occ_0.10_0.55"],
                 "after": bt1["occ_0.02_0.10"] / bt1["occ_0.10_0.55"]}}
    eq = {tag: {f"occ_{a:.2f}_{b:.2f}": float(_occupancy(sc, z_f, (a, b), ext).mean())
                for a, b in EQUAL_BANDS}
          for tag, sc in (("before", before), ("after", after))}
    for tag in eq:
        v = list(eq[tag].values())
        eq[tag]["decreases_toward_the_floor"] = bool(v[0] <= v[1])
        eq[tag]["low_over_next"] = v[0] / v[1]
    table["equal_thickness_0.08m"] = eq

    # (2) projected support counts on window A, before/after, sweeper and cylinder
    winA = {"window": WINDOW_A, "raster_cell": RASTER}
    panels = {}
    for key in ("sweeper", "cylinder"):
        rob = robots[key]
        for tag, sc in (("before", before), ("after", after)):
            s2, pst = project_scene(_crop_xy(sc, WINDOW_A, rob.max_radius() + 1.0), rob,
                                    WINDOW_A, z_floor=z_f)
            occ = _support_raster(s2, WINDOW_A, cell=RASTER)
            dist = ndimage.distance_transform_edt(~occ) * RASTER
            r = rob.max_radius()
            lab, _ = ndimage.label(dist > r)
            sizes = np.bincount(lab.ravel())[1:] * RASTER ** 2 if lab.max() else np.zeros(0)
            row = {"supports": len(s2.supports), "occupied_frac": float(occ.mean()),
                   "disc_fits_frac": float((dist > r).mean()),
                   "free_components": int(lab.max()),
                   "largest_free_component_m2": float(sizes.max()) if len(sizes) else 0.0,
                   "crit3_clear_frac": float((dist >= CRIT3_R).mean()),
                   "projection": pst}
            winA[f"{key}_{tag}"] = row
            panels[key, tag] = (occ, dist > r, dist >= CRIT3_R, row)
            print(f"[windowA {key} {tag}] " + json.dumps(
                {k: v for k, v in row.items() if k != "projection"}), flush=True)

    # (3) the same low band on the certified raster rather than the AABB aid, window A
    cert = {}
    for a, b in EQUAL_BANDS:
        for tag, sc in (("before", before), ("after", after)):
            row, _, _ = _certified_band_raster(sc, (a, b), WINDOW_A, z_f)
            cert[f"band_{a:.2f}_{b:.2f}_{tag}"] = row
            print(f"[certified {a:.2f}-{b:.2f} {tag}] " + json.dumps(row), flush=True)
    for tag in ("before", "after"):
        v = [cert[f"band_{a:.2f}_{b:.2f}_{tag}"]["occupied_frac"] for a, b in EQUAL_BANDS]
        cert[f"decreases_toward_the_floor_{tag}"] = bool(v[0] <= v[1])
        cert[f"low_over_next_{tag}"] = v[0] / v[1] if v[1] else None

    out = {"claims_boundary": CAPTION, "task": "P1d: before/after evidence for the plane floor",
           "scene": {"before": "showcase_scene.load_processed()", "after": str(SCENE_NPZ),
                     "variant": meta["variant"], "params": meta["params"]},
           "band_occupancy": table, "window_A": winA,
           "certified_raster_window_A": cert,
           "verdict_inputs": {
               "low_band_before": bt0["occ_0.02_0.10"], "low_band_after": bt1["occ_0.02_0.10"],
               "band_above": bt1["occ_0.10_0.55"],
               "strictly_below_the_band_above": bool(
                   bt1["occ_0.02_0.10"] <= bt1["occ_0.10_0.55"])},
           "seconds": time.time() - t0}
    _dump(RES / "p1d_evidence.json", out)
    _fig_windowA(panels, robots)
    _fig_bands(table)
    print(json.dumps(out["verdict_inputs"], indent=2), flush=True)


def _fig_windowA(panels, robots):
    w = WINDOW_A
    fig, axs = plt.subplots(2, 3, figsize=(24, 17))
    for i, key in enumerate(("sweeper", "cylinder")):
        r = robots[key].max_radius()
        for j, tag in enumerate(("before", "after")):
            occ, fit, crit3, row = panels[key, tag]
            img = np.ones(occ.shape[::-1] + (3,))
            img[fit.T] = (0.75, 0.95, 0.75)
            img[crit3.T] = (0.20, 0.70, 0.20)
            img[occ.T] = (0.80, 0.10, 0.10)
            ax = axs[i, j]
            ax.imshow(img, origin="lower", extent=[w[0], w[2], w[1], w[3]],
                      interpolation="nearest")
            ax.set_title(f"{key} {tag}: {row['supports']} supports (red)\n"
                         f"light green = disc r={r:g} fits {row['disc_fits_frac']:.1%}, "
                         f"dark = criterion-3 clear (>={CRIT3_R} m) {row['crit3_clear_frac']:.1%}",
                         fontsize=10)
            ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.grid(alpha=0.3, lw=0.4)
        b, a = panels[key, "before"][0], panels[key, "after"][0]
        img = np.ones(b.shape[::-1] + (3,))
        img[(b & a).T] = (0.35, 0.35, 0.35)
        img[(b & ~a).T] = (0.90, 0.55, 0.10)
        img[(~b & a).T] = (0.10, 0.25, 0.90)
        ax = axs[i, 2]
        ax.imshow(img, origin="lower", extent=[w[0], w[2], w[1], w[3]], interpolation="nearest")
        nb, na = int(b.sum()), int(a.sum())
        ax.set_title(f"{key}: grey = occupied in both ({int((b & a).sum()):,} cells)\n"
                     f"orange = cleared by the edit ({int((b & ~a).sum()):,}), "
                     f"blue = newly occupied ({int((~b & a).sum()):,} — must be 0)", fontsize=10)
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.grid(alpha=0.3, lw=0.4)
    fig.suptitle("Amendment 3 P1d - window A projected certified maps - " + CAPTION,
                 fontsize=10, y=0.995)
    plt.tight_layout(rect=(0, 0, 1, 0.95)); plt.savefig(FIGS / "p1d_windowA_before_after.png", dpi=80); plt.close()
    print(f"wrote {FIGS / 'p1d_windowA_before_after.png'}", flush=True)


def _fig_bands(table):
    labels = [k.replace("occ_", "").replace("_", "-") for k in table["before_frac"]]
    x = np.arange(len(labels))
    fig, axs = plt.subplots(1, 2, figsize=(16, 6))
    axs[0].bar(x - 0.2, [100 * v for v in table["before_frac"].values()], 0.4,
               label="as built", color="#b0aca0")
    axs[0].bar(x + 0.2, [100 * v for v in table["after_frac"].values()], 0.4,
               label="plane floor", color="#2f6f9f")
    axs[0].set_xticks(x); axs[0].set_xticklabels(labels, fontsize=9)
    axs[0].set_ylabel("cells occupied (%)")
    axs[0].set_title("required column: bands of spec 5.3 (0.08 m against 0.45 m)\n"
                     f"low/above {table['low_band_vs_band_above']['before']:.2f}x -> "
                     f"{table['low_band_vs_band_above']['after']:.2f}x")
    axs[0].legend(); axs[0].grid(alpha=0.3, axis="y")
    eq = table["equal_thickness_0.08m"]
    lab2 = [k.replace("occ_", "").replace("_", "-") for k in eq["before"] if k.startswith("occ")]
    x2 = np.arange(len(lab2))
    for off, tag, colr in ((-0.2, "before", "#b0aca0"), (0.2, "after", "#2f6f9f")):
        axs[1].bar(x2 + off, [100 * eq[tag][k] for k in eq[tag] if k.startswith("occ")], 0.4,
                   label=tag, color=colr)
    axs[1].set_xticks(x2); axs[1].set_xticklabels(lab2, fontsize=9)
    axs[1].set_title("supporting check, equal 0.08 m thickness\n"
                     f"low/next {eq['before']['low_over_next']:.2f}x -> "
                     f"{eq['after']['low_over_next']:.2f}x")
    axs[1].legend(); axs[1].grid(alpha=0.3, axis="y")
    fig.suptitle("Amendment 3 P1d - band occupancy - " + CAPTION, fontsize=9, y=0.995)
    plt.tight_layout(rect=(0, 0, 1, 0.93)); plt.savefig(FIGS / "p1d_band_occupancy.png", dpi=110); plt.close()
    print(f"wrote {FIGS / 'p1d_band_occupancy.png'}", flush=True)


# ------------------------------------------------------- P1f: the hand-off P2 actually runs --


def step_handoff():
    """Execute the P2/P3 entry path on the real scene, so P2 does not discover it is broken.

    Everything P1 hands over is exercised here: `load_plane_scene` on the 0.8 GB archive, the meta
    keys `showcase_run.py --scene planefloor` reads out of it, `project_scene` on the loaded scene
    for all three robots, and the P1e timing block round-tripping through JSON.
    """
    from gmc.height.timing import STAGES, StageTimer
    t0 = time.time()
    timer = StageTimer()
    with timer.stage("scene_load", scene="planefloor") as rec:
        scene, meta = load_plane_scene(SCENE_NPZ)
        rec["sizes"]["n_splats"] = len(scene)
    # exactly the keys experiments/showcase_run.py reads when --scene planefloor is given
    needed = ("floor", "ceiling_height_m", "gravity_rotation", "variant", "params", "z_floor",
              "extent", "first_inserted_id")
    missing = [k for k in needed if k not in meta]
    z_f = meta["z_floor"]
    robots = robot_table(1.20)
    proj = {}
    for key in ("sweeper", "cylinder", "uav"):
        rob = robots[key]
        with timer.stage("project", label=key, n_splats=len(scene)) as rec:
            s2, pst = project_scene(scene, rob, WINDOW_A, z_floor=z_f)
            rec["sizes"]["n_supports"] = pst["kept"]
        ins = np.array([s.primitive_id for s in s2.supports], dtype=np.int64)
        proj[key] = {"supports": len(s2.supports),
                     "inserted_plane_splats_in_map": int((ins >= meta["first_inserted_id"]).sum()),
                     "band_abs_above_floor": [rob.z_lo, rob.z_hi]}
        print(f"[handoff {key}] " + json.dumps(proj[key]), flush=True)
    block = json.loads(json.dumps(timer.to_dict(), default=float))
    out = {"claims_boundary": CAPTION,
           "task": "P1f: run the P2/P3 entry path end to end on the built scene",
           "scene_npz": str(SCENE_NPZ), "n_splats": len(scene),
           "loader": "gmc.height.planefloor.load_plane_scene",
           "cli": "experiments/showcase_run.py --scene planefloor",
           "meta_keys_present": sorted(meta.keys()), "meta_keys_missing": missing,
           "variant": meta["variant"], "params": meta["params"],
           "projection_window_A": proj,
           "no_inserted_plane_splat_reaches_any_robot_band": bool(
               all(v["inserted_plane_splats_in_map"] == 0 for v in proj.values())),
           "timing_block": block,
           "timing_stages_recorded": sorted(block["by_stage"]),
           "timing_stages_missing_as_expected": block["missing_stages"],
           "seconds": time.time() - t0}
    _dump(RES / "p1f_handoff.json", out)
    print(json.dumps({k: out[k] for k in ("meta_keys_missing", "timing_stages_recorded",
                                          "no_inserted_plane_splat_reaches_any_robot_band")},
                     indent=2), flush=True)
    assert not missing, f"meta is missing keys showcase_run.py needs: {missing}"
    assert out["no_inserted_plane_splat_reaches_any_robot_band"], proj
    assert set(block["by_stage"]) == {"scene_load", "project"}, block["by_stage"]
    assert set(STAGES) - set(block["by_stage"]) == set(block["missing_stages"])
    print("P1f HANDOFF OK", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", required=True,
                    choices=["diagnose", "build", "survivors", "evidence", "handoff"])
    ap.add_argument("--variant", choices=VARIANTS, help="force the P1b (mask, footprint) pair")
    ap.add_argument("--max-top", type=float, help="override the P1b rho-top ceiling (m)")
    a = ap.parse_args()
    RES.mkdir(parents=True, exist_ok=True)
    FIGS.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    if a.step == "diagnose":
        step_diagnose()
    elif a.step == "build":
        step_build(variant=a.variant, max_top=a.max_top)
    elif a.step == "survivors":
        step_survivors()
    elif a.step == "evidence":
        step_evidence()
    elif a.step == "handoff":
        step_handoff()


if __name__ == "__main__":
    main()
