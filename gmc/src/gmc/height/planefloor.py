"""Plane floor (Amendment 3 §P1): swap the phantom near-floor splats for one analytic plane.

**This is a user-approved MANUAL scene-definition change, not a reconstruction method and not an
outer approximation with a guarantee.** The user's framing was "地面的高斯用大平面替代，手动解决噪音
for now" — a stopgap. Anything computed on the edited scene is sound *with respect to the edited
scene*; the gap between that and the room is the open research question
(`docs/worklog/height_map_diagnosis.md` §4). Every report, figure caption and video title that uses
this module must say so.

The edit has two halves, and the module keeps them separate so each can be checked on its own:

* **which splats go** — :func:`phantom_cell_mask` is the diagnosis' test (occupied in the sweeper
  band, free in every band above it), :func:`open_floor_cell_mask` is the superset that drops the
  "occupied in the sweeper band" half, and :func:`replace_mask` is the conservative per-splat rule
  built on either: a splat goes only if its whole rho-footprint lies inside the mask, so anything
  that touches one cell of real geometry survives by construction;
* **what arrives** — :func:`plane_tiles` lays large flat opaque Gaussians in the fitted floor plane
  with a normal sigma small enough that their rho-extent never reaches ``z_floor + z_lo``. The
  constructor *checks* that, it does not assume it, and :func:`build_plane_floor` reports the
  measured maximum top so a caller can check it again on the built scene.

:func:`speckle_stats` is the morphology summary that separates speckle from furniture outlines, and
:func:`monotone_toward_floor` is the acceptance test the whole stage is judged by.
"""
import json
from pathlib import Path

import numpy as np
from scipy import ndimage

from .band_shadow import band_overlap_mask, outer_shadow_ellipses
from .ply3d import GaussianScene3D

Z_LO_DEFAULT = 0.02        # robot ground clearance (spec §3.3, frozen)
TAU_DEFAULT = 0.3


# ------------------------------------------------------------------ P1a: the phantom test --


def phantom_cell_mask(low_band, bands_above):
    """Cells occupied in the sweeper band and free in **every** band above it.

    This is the discriminating test of `height_map_diagnosis.md` §2: a 2-10 cm obstacle with
    nothing at all above it, repeated over hundreds of m2 of an art gallery, is not furniture.
    """
    low = np.asarray(low_band, dtype=bool)
    above = np.zeros_like(low)
    for band in bands_above:
        b = np.asarray(band, dtype=bool)
        if b.shape != low.shape:
            raise ValueError(f"every band must have the same shape as the low band: "
                             f"{b.shape} != {low.shape}")
        above |= b
    return low & ~above


def speckle_stats(mask, cell):
    """Component count and blob-size summary — speckle (phantom) vs outlines (furniture)."""
    mask = np.asarray(mask, dtype=bool)
    area = float(cell) ** 2
    lab, n = ndimage.label(mask)
    sizes = np.bincount(lab.ravel())[1:] * area if n else np.zeros(0)
    return {"cells": int(mask.sum()), "area_m2": float(mask.sum() * area),
            "components": int(n),
            "median_blob_m2": float(np.median(sizes)) if n else 0.0,
            "frac_blobs_le_100cm2": float((sizes <= 4 * area).mean()) if n else 0.0}


def monotone_toward_floor(fracs):
    """True iff the lowest band is no more occupied than the band above it.

    ``fracs`` is ordered top band first, floor band last — the order the evidence table is read in.
    Occupancy should *increase* with height if anything, because everything standing on the floor
    is seen by the low band too; the as-built showcase map does the opposite (40.7 % against
    17.4 %), which is the defect P1 exists to fix.
    """
    f = [float(v) for v in fracs]
    if len(f) < 2:
        raise ValueError("need at least two bands")
    return bool(f[-1] <= f[-2])


def open_floor_cell_mask(bands_above):
    """Cells free in **every** band above the sweeper band: demonstrably nothing overhead.

    ``phantom_cell_mask(low, above) == low & open_floor_cell_mask(above)``, so this is a superset
    of the phantom set and carries the same overhead-free guarantee. It is the footprint test that
    actually works on this scene -- see :func:`replace_mask`.
    """
    above = None
    for band in bands_above:
        b = np.asarray(band, dtype=bool)
        if above is None:
            above = np.zeros_like(b)
        elif b.shape != above.shape:
            raise ValueError(f"every band must have the same shape: {b.shape} != {above.shape}")
        above |= b
    if above is None:
        raise ValueError("need at least one band above")
    return ~above


# ------------------------------------------------------------------ P1b: the replacement rule --


def footprint_cell_span(lo_xy, hi_xy, extent, cell, shape):
    """Inclusive cell index ranges of axis-aligned xy boxes, clipped to the grid."""
    lo = np.atleast_2d(np.asarray(lo_xy, dtype=float))[:, :2]
    hi = np.atleast_2d(np.asarray(hi_xy, dtype=float))[:, :2]
    x0, y0 = float(extent[0]), float(extent[1])
    nx, ny = int(shape[0]), int(shape[1])
    i0 = np.clip(np.floor((lo[:, 0] - x0) / cell).astype(np.int64), 0, nx - 1)
    i1 = np.clip(np.floor((hi[:, 0] - x0) / cell).astype(np.int64), 0, nx - 1)
    j0 = np.clip(np.floor((lo[:, 1] - y0) / cell).astype(np.int64), 0, ny - 1)
    j1 = np.clip(np.floor((hi[:, 1] - y0) / cell).astype(np.int64), 0, ny - 1)
    return i0, i1, j0, j1


FOOTPRINTS = ("aabb", "shadow", "centre")


def footprint_spans(scene3d, extent, cell, shape, *, level, mode, band_abs):
    """Cell spans of the xy region each splat is judged by. See :func:`replace_mask`.

    ``aabb``    the whole ellipsoid's xy box, which is what ``showcase_scene._occupancy`` marks;
    ``shadow``  the band-clipped outer shadow ellipse's box -- the obstacle ``project_scene``
                actually builds, and therefore what the robot actually sees;
    ``centre``  the single cell holding the splat's centre.
    """
    if mode not in FOOTPRINTS:
        raise ValueError(f"footprint must be one of {FOOTPRINTS}, not {mode!r}")
    if mode == "aabb":
        lo, hi = scene3d.aabb(float(level))
        return footprint_cell_span(lo, hi, extent, cell, shape)
    c = scene3d.means[:, :2].astype(float, copy=True)
    half = np.zeros_like(c)
    if mode == "shadow":
        z0, z1 = (float(v) for v in band_abs)
        inband = band_overlap_mask(scene3d.means[:, 2], scene3d.covs[:, 2, 2], z0, z1,
                                   float(level))
        if inband.any():
            oe = outer_shadow_ellipses(scene3d.means[inband], scene3d.covs[inband], z0, z1,
                                       float(level))
            c[inband] = oe["centre"]
            half[inband] = np.sqrt(np.stack([oe["Q"][:, 0, 0], oe["Q"][:, 1, 1]], axis=1))
    return footprint_cell_span(c - half, c + half, extent, cell, shape)


def replace_mask(scene3d, footprint_mask, extent, cell, *, z_floor, max_top=0.10, level=2.0,
                 tau=TAU_DEFAULT, require_opaque=False, footprint="aabb", band_lo=Z_LO_DEFAULT):
    """Amendment 3 §P1b: replace a splat iff its rho-footprint lies (in xy) only inside
    ``footprint_mask`` **and** its rho-top is below ``z_floor + max_top``.

    Both halves are conservative in the direction that protects real geometry: the footprint is
    the rho-AABB (a superset of the splat's shadow), and one cell outside the mask anywhere under
    it is enough to keep the splat. Transparent splats (opacity <= tau) are invisible to the
    projector anyway; ``require_opaque`` keeps them in the scene instead of folding them into the
    plane, which only matters for rendering.

    Two footprint masks are admissible, and P1 reports both:

    * :func:`phantom_cell_mask` — the literal §P1b wording. On this scene it is nearly useless,
      and the reason is visible in `diagnosis/fig1_phantom_floor.png` panel 1: the phantom layer
      is salt-and-pepper *dust* over the open floor (367 components, median blob 0.04 m2), so
      almost every splat's 2-3 cell footprint straddles a cell that happens to be free in the
      low band and is therefore not phantom. The test keeps the dust it was written to remove.
    * :func:`open_floor_cell_mask` — cells free in every band **above** 0.10 m, which is the
      other half of the same conjunction (``phantom = low & open_floor``, so this is a superset).
      It is solid exactly where the dust is, and it carries the identical guarantee, which is the
      one that protects real geometry: a splat is removed only if **nothing at all** stands
      anywhere over its footprint between 0.10 m and 2.50 m. Anything with mass above 0.10 m --
      every table leg, bench support, plinth and counter -- is therefore kept by construction,
      not by luck.

    On this scene the two are **exactly equivalent**, which is worth stating because it is not
    obvious: any splat that reaches into the sweeper band marks its whole footprint occupied in
    the low band, so for a candidate ``footprint subset open_floor`` already implies
    ``footprint subset (low & open_floor) = phantom``. The same argument shows ``max_top`` is
    never the binding clause: every splat covering a phantom cell has a rho-top below
    ``z_floor + 0.10``, or the cell would not have been free in the band above.

    What *is* binding is ``footprint``, i.e. which xy region a splat is judged by:

    * ``aabb`` -- the whole ellipsoid's xy box, the region ``showcase_scene._occupancy`` marks.
      It is a gross over-approximation of what a splat occludes (that function calls itself a
      "selection aid only"), and it is why 31 % of the phantom layer survives the rule: a wide
      floor splat whose box happens to touch one cell under a table leg is protected by a leg
      half a metre away.
    * ``shadow`` -- the band-clipped outer shadow ellipse, which is the obstacle
      ``project_scene`` builds and the robot actually sees. Judging a splat by what it occludes
      rather than by its bounding box is the accurate test, not a looser one, and it is the one
      the *certified* pipeline is built on.
    * ``centre`` -- the single cell holding the splat's centre; the most permissive bound, kept
      so the sweep has a floor to report against.
    """
    phantom = np.asarray(footprint_mask, dtype=bool)
    lo, hi = scene3d.aabb(float(level))
    i0, i1, j0, j1 = footprint_spans(
        scene3d, extent, float(cell), phantom.shape, level=level, mode=footprint,
        band_abs=(float(z_floor) + float(band_lo), float(z_floor) + float(max_top)))

    # "every cell under the footprint is phantom" <=> the box holds no non-phantom cell, which a
    # summed-area table of ~phantom answers for all splats at once.
    sat = np.zeros((phantom.shape[0] + 1, phantom.shape[1] + 1), dtype=np.int64)
    sat[1:, 1:] = np.cumsum(np.cumsum(~phantom, axis=0), axis=1)
    non_phantom = (sat[i1 + 1, j1 + 1] - sat[i0, j1 + 1]
                   - sat[i1 + 1, j0] + sat[i0, j0])

    inside = (lo[:, 0] >= extent[0]) & (hi[:, 0] <= extent[2]) \
        & (lo[:, 1] >= extent[1]) & (hi[:, 1] <= extent[3])
    low_enough = hi[:, 2] < float(z_floor) + float(max_top)
    out = (non_phantom == 0) & inside & low_enough
    if require_opaque:
        out &= scene3d.opacity > float(tau)
    return out


def _plane_basis(normal):
    n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    seed = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = seed - (seed @ n) * n
    u /= np.linalg.norm(u)
    return n, u, np.cross(n, u)


def plane_tiles(extent, *, z_floor, normal, centroid, spacing=1.0, sigma_n=0.004,
                opacity=0.95, first_id=0, z_top_max, level=2.0, sigma_xy=None,
                local_z=None, max_sink=0.05):
    """A tiling of large flat opaque Gaussians lying in the fitted floor plane.

    ``sigma_xy`` defaults to ``0.4 * spacing``, which makes the rho=``level`` disc of each tile
    reach ``2 * 0.4 * spacing = 0.8 * spacing`` — more than the ``spacing / sqrt(2) ~ 0.707 *
    spacing`` half-diagonal of its cell, so the tiling has no holes.

    The tile centre follows the fitted plane, or ``local_z(x, y)`` when given (the measured local
    floor height is a better ground surface than a single global plane fit when the fit is tilted),
    but is **clamped** so that ``centre_z + level * sqrt(Sigma_zz) <= z_top_max``. That clamp is
    what makes the inserted plane invisible to every robot band, and it is checked, not assumed:
    a ``sigma_n`` too large for ``z_top_max`` on its own is a ``ValueError``.
    """
    n, u, v = _plane_basis(normal)
    c0 = np.asarray(centroid, dtype=float)
    s_xy = float(0.4 * spacing if sigma_xy is None else sigma_xy)
    s_n = float(sigma_n)

    cov = (s_xy ** 2) * (np.outer(u, u) + np.outer(v, v)) + (s_n ** 2) * np.outer(n, n)
    cov = 0.5 * (cov + cov.T)
    half = float(level) * np.sqrt(cov[2, 2])
    # The clamp below always honours z_top_max; the failure mode a large sigma_n really has is
    # sinking the whole ground surface to buy the headroom, so that is what is refused.
    sink = float(z_floor) - (float(z_top_max) - half)
    if sink > float(max_sink):
        raise ValueError(
            f"sigma_n={s_n} gives a rho={level} half-height of {half:.4f} m, so fitting under "
            f"z_top_max={z_top_max} would sink the plane {sink:.4f} m below the floor "
            f"(max_sink={max_sink})")

    x0, y0, x1, y1 = (float(t) for t in extent)
    gx = np.arange(x0 + 0.5 * spacing, x1 + 0.5 * spacing, spacing)
    gy = np.arange(y0 + 0.5 * spacing, y1 + 0.5 * spacing, spacing)
    xx, yy = np.meshgrid(gx, gy, indexing="ij")
    xy = np.column_stack([xx.ravel(), yy.ravel()])

    # z of the fitted plane at each tile centre: (p - c0) . n = 0
    plane_z = c0[2] - (n[0] * (xy[:, 0] - c0[0]) + n[1] * (xy[:, 1] - c0[1])) / n[2]
    target = plane_z if local_z is None else np.asarray(local_z, dtype=float)
    if target.shape != plane_z.shape:
        raise ValueError("local_z must have one entry per tile")
    z = np.minimum(target, float(z_top_max) - half)

    means = np.column_stack([xy, z])
    covs = np.repeat(cov[None, :, :], len(means), axis=0)
    ids = first_id + np.arange(len(means), dtype=np.int64)
    tops = z + half
    if tops.max() > float(z_top_max) + 1e-12:
        raise ValueError(f"tile tops reach {tops.max()} above z_top_max={z_top_max}")
    return {"means": means, "covs": covs, "ids": ids,
            "opacity": np.full(len(means), float(opacity)),
            "sigma_xy": s_xy, "sigma_n": s_n, "spacing": float(spacing),
            "rho_half_height": half, "plane_z": plane_z,
            "clamped": int((target > z + 1e-12).sum()),
            "max_drop_below_target_m": float(np.max(target - z)) if len(z) else 0.0}


def build_plane_floor(scene3d, footprint_mask, extent, cell, *, floor, spacing=1.0, sigma_n=0.004,
                      max_top=0.10, z_lo=Z_LO_DEFAULT, margin=0.005, level=2.0,
                      opacity=0.95, tau=TAU_DEFAULT, require_opaque=False,
                      measure_local_floor=True, footprint="aabb"):
    """Drop the phantom near-floor splats and insert the analytic plane in their place.

    Returns ``(scene, stats)``. ``stats["inserted_rho_top_max_above_floor"]`` is measured on the
    built scene with ``aabb(level)`` — the number Amendment 3 asks to be reported rather than
    inferred from ``sigma_n`` — and must stay below ``z_lo``.
    """
    gone = replace_mask(scene3d, footprint_mask, extent, cell, z_floor=floor["z_floor"],
                        max_top=max_top, level=level, tau=tau, require_opaque=require_opaque,
                        footprint=footprint, band_lo=z_lo)
    kept = scene3d.subset(~gone)
    z_top_max = float(floor["z_floor"]) + float(z_lo) - float(margin)

    local = None
    if measure_local_floor and gone.any():
        local = _local_floor_z(scene3d.subset(gone), extent, spacing, floor)

    tiles = plane_tiles(extent, z_floor=floor["z_floor"], normal=floor["normal"],
                        centroid=floor["centroid"], spacing=spacing, sigma_n=sigma_n,
                        opacity=opacity, first_id=int(scene3d.ids.max()) + 1,
                        z_top_max=z_top_max, level=level, local_z=local)

    out = GaussianScene3D(np.vstack([kept.means, tiles["means"]]),
                          np.vstack([kept.covs, tiles["covs"]]),
                          np.concatenate([kept.opacity, tiles["opacity"]]),
                          np.concatenate([kept.ids, tiles["ids"]]),
                          scene3d.name)
    ins_top = float((tiles["means"][:, 2] + level * np.sqrt(tiles["covs"][:, 2, 2])).max())
    stats = {
        "claims_boundary": "user-approved manual scene edit; sound wrt the edited scene only",
        "n_input": len(scene3d), "replaced": int(gone.sum()),
        "replaced_opaque_tau": int((gone & (scene3d.opacity > tau)).sum()),
        "kept": len(kept), "inserted": len(tiles["means"]), "n_output": len(out),
        "first_inserted_id": int(tiles["ids"][0]),
        "params": {"cell": float(cell), "max_top": float(max_top), "level": float(level),
                   "spacing": float(spacing), "sigma_xy": tiles["sigma_xy"],
                   "sigma_n": tiles["sigma_n"], "opacity": float(opacity),
                   "z_lo": float(z_lo), "margin": float(margin),
                   "require_opaque": bool(require_opaque),
                   "footprint": str(footprint),
                   "measure_local_floor": bool(measure_local_floor)},
        "z_top_max_budget": z_top_max,
        "inserted_rho_top_max": ins_top,
        "inserted_rho_top_max_above_floor": ins_top - float(floor["z_floor"]),
        "inserted_rho_half_height": tiles["rho_half_height"],
        "inserted_headroom_below_z_lo": (float(floor["z_floor"]) + float(z_lo)) - ins_top,
        "tiles_clamped_off_the_fitted_plane": tiles["clamped"],
        "tile_max_drop_below_target_m": tiles["max_drop_below_target_m"],
        "fitted_plane_z_above_floor_range": [
            float(tiles["plane_z"].min() - floor["z_floor"]),
            float(tiles["plane_z"].max() - floor["z_floor"])],
    }
    return out, stats


def _local_floor_z(removed, extent, spacing, floor):
    """Median rho-centre z of the replaced splats per tile; the fitted plane where there are none.

    The showcase floor fit is tilted 0.285 deg, which over a 25 m hall is +-6.4 cm -- most of the
    sweeper band. A single global plane therefore mis-places the rendered ground by up to that much;
    the splats we are deleting say where the floor actually was, tile by tile.
    """
    n, _, _ = _plane_basis(floor["normal"])
    c0 = np.asarray(floor["centroid"], dtype=float)
    x0, y0, x1, y1 = (float(t) for t in extent)
    gx = np.arange(x0 + 0.5 * spacing, x1 + 0.5 * spacing, spacing)
    gy = np.arange(y0 + 0.5 * spacing, y1 + 0.5 * spacing, spacing)
    nx, ny = len(gx), len(gy)
    plane_z = c0[2] - (n[0] * (np.repeat(gx, ny) - c0[0])
                       + n[1] * (np.tile(gy, nx) - c0[1])) / n[2]
    if len(removed) == 0:
        return plane_z
    i = np.clip(((removed.means[:, 0] - x0) / spacing).astype(np.int64), 0, nx - 1)
    j = np.clip(((removed.means[:, 1] - y0) / spacing).astype(np.int64), 0, ny - 1)
    flat = i * ny + j
    out = plane_z.copy()
    order = np.argsort(flat, kind="stable")
    fs, zs = flat[order], removed.means[order, 2]
    edges = np.flatnonzero(np.r_[True, fs[1:] != fs[:-1], True])
    for a, b in zip(edges[:-1], edges[1:]):
        out[fs[a]] = np.median(zs[a:b])
    return out


# ------------------------------------------------------------------ the P2/P3 hand-off --


def save_plane_scene(path, scene, meta):
    """Write the edited scene plus enough metadata that a downstream stage needs nothing else.

    The base scene's own ``processed.npz`` lives in a directory this chain may only read, so the
    plane-floor variant is written under ``gmc/outputs/height/plane/`` instead. ``meta`` travels
    inside the archive as JSON so that :func:`load_plane_scene` is a one-liner for P2 and P3 and
    cannot be pointed at a scene whose floor parameters came from somewhere else.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, means=scene.means, covs=scene.covs, opacity=scene.opacity, ids=scene.ids,
             meta=np.array(json.dumps(meta, default=float)))
    return path


def load_plane_scene(path, name="showcase_planefloor"):
    """``(GaussianScene3D, meta)`` for the plane-floor scene. The P1 hand-off to P2 and P3.

    ``meta`` carries ``floor`` (the same dict ``showcase_scene.load_processed`` returns in
    ``g0["floor"]``), ``z_floor``, ``ceiling_height_m``, ``extent`` and the full P1b parameter set,
    so a downstream run never has to reload the unedited scene to find its floor.
    """
    d = np.load(Path(path), allow_pickle=False)
    scene = GaussianScene3D(d["means"], d["covs"], d["opacity"], d["ids"], name)
    return scene, json.loads(str(d["meta"]))
