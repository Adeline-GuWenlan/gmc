"""Amendment 3 P3: sizing long-range windows before a job is spent on them, and the cylinder's ladder.

P2 learned the expensive way that ``showcase_run.py``'s probe under-estimates the cylinder by >= 13x, so
P3 sizes every window by the number GMC is actually handed: ``project_scene``'s ``kept`` count. Calling
``project_scene`` once per candidate window would redo the eigen-decomposition of all 7.1 M splats each
time, so :func:`shadow_table` does the window-independent part once per robot (opacity, 3D eigenvalue
floor, band test, certified outer shadow ellipse, de-duplication) and :func:`count_supports` then applies
``project_scene``'s two window tests to it. A unit test pins the count to ``kept`` window by window.

Nothing here changes what a robot sees: tau, rho, the band and the shadow are ``project_scene``'s own.
"""
import numpy as np

from .band_shadow import band_overlap_mask, outer_shadow_ellipses
from .project import EIG3_FLOOR

CHUNK = 1_000_000


def shadow_table(scene3d, robot, *, z_floor, tau=0.3, level=2.0):
    """The window-independent half of ``project_scene``: one row per support it could ever keep.

    Returns the xy centre and xy half-extent of each splat's rho-ellipsoid (``project_scene``'s cheap
    "near" test), the centre and half-extent of its certified outer shadow ellipse (the "inside" test),
    and the robot's dilation radius. Rows with an identical shadow are dropped as ``project_scene`` does.
    """
    z_lo, z_hi = robot.band_abs(z_floor)
    opaque = np.flatnonzero(scene3d.opacity > float(tau))
    m_all, h_all, c_all, e_all, rows_all = [], [], [], [], []
    for a in range(0, len(opaque), CHUNK):
        sel = opaque[a:a + CHUNK]
        means = scene3d.means[sel]              # dtypes exactly as project_scene sees them
        covs = scene3d.covs[sel].copy()
        w, v = np.linalg.eigh(covs)
        low = w.min(axis=1) < EIG3_FLOOR
        if low.any():
            wl = np.maximum(w[low], EIG3_FLOOR)
            covs[low] = (v[low] * wl[:, None, :]) @ v[low].transpose(0, 2, 1)
        inband = band_overlap_mask(means[:, 2], covs[:, 2, 2], z_lo, z_hi, level)
        means, covs = means[inband], covs[inband]
        if not len(means):
            continue
        half = level * np.sqrt(np.stack([covs[:, 0, 0], covs[:, 1, 1]], axis=1))
        oe = outer_shadow_ellipses(means, covs, z_lo, z_hi, level)
        c, Q = oe["centre"], oe["Q"]
        m_all.append(means[:, :2])
        h_all.append(half)
        c_all.append(c)
        e_all.append(np.sqrt(np.stack([Q[:, 0, 0], Q[:, 1, 1]], axis=1)))
        rows_all.append(np.ascontiguousarray(np.concatenate([c, Q.reshape(-1, 4)], axis=1)))
    if not m_all:
        z = np.zeros((0, 2))
        return {"m": z, "h3": z, "c": z, "eh": z, "r": robot.max_radius(), "n": 0}
    rows = np.ascontiguousarray(np.concatenate(rows_all))
    _, keep = np.unique(rows.view(np.dtype((np.void, 48))), return_index=True)
    keep = np.sort(keep)
    return {"m": np.concatenate(m_all)[keep], "h3": np.concatenate(h_all)[keep],
            "c": np.concatenate(c_all)[keep], "eh": np.concatenate(e_all)[keep],
            "r": robot.max_radius(), "n": int(len(keep))}


def count_supports(tab, window):
    """How many supports ``project_scene(scene, robot, window)`` keeps: both of its window tests."""
    x0, y0, x1, y1 = map(float, window)
    r = tab["r"]
    m, h, c, e = tab["m"], tab["h3"], tab["c"], tab["eh"]
    near = ((m[:, 0] - h[:, 0] <= x1 + r) & (m[:, 0] + h[:, 0] >= x0 - r)
            & (m[:, 1] - h[:, 1] <= y1 + r) & (m[:, 1] + h[:, 1] >= y0 - r))
    inside = ((c[:, 0] - e[:, 0] <= x1 + r) & (c[:, 0] + e[:, 0] >= x0 - r)
              & (c[:, 1] - e[:, 1] <= y1 + r) & (c[:, 1] + e[:, 1] >= y0 - r))
    return int((near & inside).sum())


def density_grid(tab, extent, cell):
    """Supports per m², binned by shadow-ellipse centre on a ``cell`` grid over ``extent``."""
    x0, y0, x1, y1 = map(float, extent)
    nx, ny = int(round((x1 - x0) / cell)), int(round((y1 - y0) / cell))
    H, _, _ = np.histogram2d(tab["c"][:, 0], tab["c"][:, 1], bins=[nx, ny],
                             range=[[x0, x0 + nx * cell], [y0, y0 + ny * cell]])
    return H / (cell * cell), [x0, y0, x1, y1]


def route_window(point_sets, margin, max_side, bounds=None, min_side=0.0):
    """The bounding box of every point (endpoints, routes) plus ``margin``, clipped to ``bounds``.

    ``None`` when a side exceeds ``max_side`` (D2's 12 m) or falls below ``min_side``.
    """
    pts = np.concatenate([np.asarray(p, float).reshape(-1, 2) for p in point_sets])
    w = [pts[:, 0].min() - margin, pts[:, 1].min() - margin,
         pts[:, 0].max() + margin, pts[:, 1].max() + margin]
    if bounds is not None:
        w = [max(w[0], bounds[0]), max(w[1], bounds[1]), min(w[2], bounds[2]), min(w[3], bounds[3])]
    w = [round(float(v), 3) for v in w]
    sx, sy = w[2] - w[0], w[3] - w[1]
    if sx > max_side + 1e-9 or sy > max_side + 1e-9 or sx < min_side or sy < min_side:
        return None
    return w


def min_pool_dist(dist, k):
    """Coarsen a clearance map by ``k`` taking the block minimum: never more optimistic than the fine map.

    A partial block at the far edge is reported as 0 (occupied), so the coarse map is a screening aid that
    can only lose routes, never invent them.
    """
    dist = np.asarray(dist, float)
    nx, ny = -(-dist.shape[0] // k), -(-dist.shape[1] // k)
    pad = np.zeros((nx * k, ny * k))
    pad[:dist.shape[0], :dist.shape[1]] = dist
    return pad.reshape(nx, k, ny, k).min(axis=(1, 3))


def ladder_goals(anchor, candidates, targets, toward, tol=0.2, perp_weight=0.5):
    """One goal per target separation, stepping out from ``anchor`` along the ray towards ``toward``.

    ``candidates`` are points already known to be admissible goals (D1 clearance, same component as the
    anchor). For each target the chosen goal is within ``tol`` of the target distance, ahead of the
    anchor along the ray, and as close to the ray as possible. A target with no such point is reported
    with ``goal = None`` rather than silently skipped.
    """
    a = np.asarray(anchor, float)[:2]
    cand = np.asarray(candidates, float).reshape(-1, 2)
    u = np.asarray(toward, float)[:2] - a
    u = u / np.linalg.norm(u)
    rel = cand - a
    d = np.hypot(rel[:, 0], rel[:, 1])
    along = rel @ u
    perp = np.abs(rel @ np.array([-u[1], u[0]]))
    out, used = [], set()
    for t in targets:
        ok = (np.abs(d - t) <= tol) & (along > 0)
        ok &= np.array([tuple(p) not in used for p in map(tuple, cand)]) if len(cand) else ok
        if not ok.any():
            out.append({"target_m": float(t), "goal": None, "dist_m": None, "off_ray_m": None})
            continue
        score = np.abs(d - t) + perp_weight * perp
        i = int(np.flatnonzero(ok)[np.argmin(score[ok])])
        used.add(tuple(cand[i]))
        out.append({"target_m": float(t), "goal": [float(cand[i, 0]), float(cand[i, 1])],
                    "dist_m": float(d[i]), "off_ray_m": float(perp[i])})
    return out
