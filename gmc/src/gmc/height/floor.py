# gmc/src/gmc/height/floor.py
"""Floor-surface rule (Amendment 1, spec §3.1a): drop flat splats lying on the fitted floor."""
import numpy as np


def floor_surface_mask(scene3d, normal, centroid, *, max_offset=0.05, max_tilt_deg=15.0,
                       max_flatness=0.5, chunk=1_000_000):
    n = np.asarray(normal, float)
    n = n / np.linalg.norm(n)
    p0 = np.asarray(centroid, float)
    out = np.zeros(len(scene3d), dtype=bool)
    cos_max = np.cos(np.deg2rad(max_tilt_deg))
    for s in range(0, len(scene3d), chunk):
        m = scene3d.means[s:s + chunk]
        w, v = np.linalg.eigh(scene3d.covs[s:s + chunk])        # ascending eigenvalues
        near = np.abs((m - p0) @ n) <= max_offset
        lying = np.abs(v[:, :, 0] @ n) >= cos_max               # thinnest axis ~ normal
        flat = np.sqrt(np.maximum(w[:, 0], 0)) <= max_flatness * np.sqrt(np.maximum(w[:, 1], 0))
        out[s:s + chunk] = near & lying & flat
    return out


def apply_floor_rule(scene3d, floor, **kw):
    mask = floor_surface_mask(scene3d, floor["normal"], floor["centroid"], **kw)
    params = {"max_offset": kw.get("max_offset", 0.05), "max_tilt_deg": kw.get("max_tilt_deg", 15.0),
              "max_flatness": kw.get("max_flatness", 0.5)}
    stats = {"n_input": len(scene3d), "removed": int(mask.sum()),
             "removed_opaque_tau0.3": int((mask & (scene3d.opacity > 0.3)).sum()), "params": params}
    return scene3d.subset(~mask), stats
