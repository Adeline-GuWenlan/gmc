"""3D splat scene -> the 2D obstacle map one prism robot sees (spec §3.1-§3.2).

Every step either removes obstacle mass and is counted (opacity, band, window)
or only grows obstacles (3D/2D eigenvalue floors, outer ellipses).
"""
import numpy as np
from shapely.geometry import box as shapely_box

from ..types import GaussianSupport2D, SceneModel2D
from .band_shadow import band_overlap_mask, outer_shadow_ellipses

EIG3_FLOOR = 1e-12


def project_scene(scene3d, robot, window, *, z_floor, tau=0.3, level=2.0,
                  name=None):
    xmin, ymin, xmax, ymax = map(float, window)
    z_lo, z_hi = robot.band_abs(z_floor)
    r = robot.max_radius()
    stats = {"n_input": len(scene3d), "tau": float(tau), "level": float(level),
             "window": [xmin, ymin, xmax, ymax], "band_abs": [z_lo, z_hi],
             "dilate_radius": r}

    opaque = scene3d.opacity > float(tau)
    stats["dropped_opacity"] = int((~opaque).sum())
    sub = scene3d.subset(opaque)

    covs = sub.covs.copy()
    w, v = np.linalg.eigh(covs)
    low = w.min(axis=1) < EIG3_FLOOR
    stats["floored_3d"] = int(low.sum())
    if low.any():
        wl = np.maximum(w[low], EIG3_FLOOR)
        covs[low] = (v[low] * wl[:, None, :]) @ v[low].transpose(0, 2, 1)

    inband = band_overlap_mask(sub.means[:, 2], covs[:, 2, 2], z_lo, z_hi, level)
    stats["excluded_band"] = int((~inband).sum())
    means, covs, pid = sub.means[inband], covs[inband], sub.ids[inband]

    # Cheap superset test: the whole ellipsoid's xy box.
    half = level * np.sqrt(np.stack([covs[:, 0, 0], covs[:, 1, 1]], axis=1))
    near = ((means[:, 0] - half[:, 0] <= xmax + r)
            & (means[:, 0] + half[:, 0] >= xmin - r)
            & (means[:, 1] - half[:, 1] <= ymax + r)
            & (means[:, 1] + half[:, 1] >= ymin - r))
    out_far = int((~near).sum())
    means, covs, pid = means[near], covs[near], pid[near]

    supports = []
    floored_2d = cand2 = out_box = dedup = 0
    if len(means):
        oe = outer_shadow_ellipses(means, covs, z_lo, z_hi, level)
        c, Q = oe["centre"], oe["Q"]
        eh = np.sqrt(np.stack([Q[:, 0, 0], Q[:, 1, 1]], axis=1))
        inside = ((c[:, 0] - eh[:, 0] <= xmax + r) & (c[:, 0] + eh[:, 0] >= xmin - r)
                  & (c[:, 1] - eh[:, 1] <= ymax + r) & (c[:, 1] + eh[:, 1] >= ymin - r))
        out_box = int((~inside).sum())
        rows = np.ascontiguousarray(np.concatenate([c, Q.reshape(-1, 4)], axis=1))
        first = np.zeros(len(rows), dtype=bool)
        cand_idx = np.flatnonzero(inside)
        _, keep_pos = np.unique(rows[cand_idx].view(np.dtype((np.void, 48))),
                                return_index=True)
        first[cand_idx[np.sort(keep_pos)]] = True
        dedup = int(inside.sum() - first.sum())
        inside = first
        floored_2d = int(oe["floored"][inside].sum())
        cand2 = int((oe["candidate"][inside] == 2).sum())
        for ci, Qi, idi in zip(c[inside], Q[inside], pid[inside]):
            supports.append(GaussianSupport2D(
                mean=ci, covariance=Qi / level ** 2, level=level,
                primitive_id=int(idi)))
    stats.update(outside_window=out_far + out_box, deduplicated=dedup,
                 kept=len(supports),
                 floored_2d=floored_2d, candidate2=cand2)
    scene2d = SceneModel2D(supports=tuple(supports),
                           workspace=shapely_box(xmin, ymin, xmax, ymax),
                           name=name or f"{scene3d.name}__{robot.name}")
    return scene2d, stats
