"""Independent 3D replay of a returned path (spec §3.4).

Disjointness of a prism and a splat is certified only by exhibiting a unit
direction with a positive support gap; see the plan's Task 5 header.
"""
import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree

from .pathio import sample_curve


def sphere_dirs(n):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    th = np.pi * (1 + 5 ** 0.5) * i
    return np.column_stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi),
                            np.cos(phi)])


def _prism_support(V, centre, F, z_lo, z_hi):
    vxy = V[:, :2]
    return (vxy @ centre
            + np.sqrt(np.maximum(np.einsum("ki,ij,kj->k", vxy, F, vxy), 0.0))
            + np.maximum(V[:, 2] * z_lo, V[:, 2] * z_hi))


def pair_gaps(V, mu, Sig, level, centre, F, z_lo, z_hi):
    hp = _prism_support(V, np.asarray(centre, float), np.asarray(F, float),
                        float(z_lo), float(z_hi))
    quad = np.einsum("ki,mij,kj->mk", V, Sig, V)
    gaps = mu @ V.T - level * np.sqrt(np.maximum(quad, 0.0)) - hp[None, :]
    return gaps.max(axis=1)


def refine_gap(mu, Sig, level, centre, F, z_lo, z_hi, v0):
    def neg_gap(ang):
        v = np.array([np.cos(ang[0]) * np.sin(ang[1]),
                      np.sin(ang[0]) * np.sin(ang[1]), np.cos(ang[1])])
        return -pair_gaps(v[None], mu[None], Sig[None], level, centre, F,
                          z_lo, z_hi)[0]
    v0 = np.asarray(v0, float) / np.linalg.norm(v0)
    a0 = [np.arctan2(v0[1], v0[0]), np.arccos(np.clip(v0[2], -1, 1))]
    res = minimize(neg_gap, a0, method="Nelder-Mead",
                   options={"xatol": 1e-9, "fatol": 1e-12, "maxiter": 600})
    return float(max(-res.fun, -neg_gap(a0)))


def _refined_gap(g, mu, Sig, level, centre, F, z_lo, z_hi):
    v0 = np.append(mu[:2] - centre, 0.0)
    if not np.any(v0):
        v0 = np.array([1.0, 0.0, 0.0])
    return max(g, refine_gap(mu, Sig, level, centre, F, z_lo, z_hi, v0))


def _footprints(robot, pose, delta):
    """World-frame (centre, F) per footprint support, dilated to cover delta."""
    x, y, th = map(float, pose)
    c, s = np.cos(th), np.sin(th)
    R = np.array([[c, -s], [s, c]])
    out = []
    for sup in robot.footprint.supports:
        shape = sup.level ** 2 * np.asarray(sup.covariance, float)
        b = float(np.sqrt(np.linalg.eigvalsh(shape)[0]))
        lam = 1.0 + float(delta) / b
        out.append((np.array([x, y]) + R @ np.asarray(sup.mean, float),
                    lam ** 2 * (R @ shape @ R.T)))
    return out


def _dilated_radius(robot, delta):
    r = 0.0
    for sup in robot.footprint.supports:
        w = np.linalg.eigvalsh(sup.level ** 2 * np.asarray(sup.covariance, float))
        lam = 1.0 + float(delta) / float(np.sqrt(w[0]))
        r = max(r, float(np.linalg.norm(sup.mean)) + lam * float(np.sqrt(w[-1])))
    return r


def replay_curve(scene3d, robot, curve, *, z_floor, tau=0.3, level=2.0,
                 delta=0.01, n_coarse=256, n_fine=2048, margin=0.05,
                 big_extent=0.5):
    if robot.is_2d:
        raise ValueError("replay_curve needs a prism robot with a height band")
    z_lo, z_hi = robot.band_abs(z_floor)
    sub = scene3d.subset(scene3d.opacity > float(tau))
    lo, hi = sub.aabb(level)
    ext = 0.5 * (hi - lo)[:, :2].max(axis=1)
    zmask = (lo[:, 2] <= z_hi + margin) & (hi[:, 2] >= z_lo - margin)
    big = np.flatnonzero(zmask & (ext > big_extent))
    small = np.flatnonzero(zmask & (ext <= big_extent))
    tree = cKDTree(sub.means[small, :2]) if len(small) else None
    Vc, Vf = sphere_dirs(n_coarse), sphere_dirs(n_fine)
    poses = sample_curve(curve, delta, robot.max_radius())
    reach = _dilated_radius(robot, delta) + margin + big_extent

    n_pairs = n_refined = 0
    min_lb, worst, collisions = margin, None, []
    for k, pose in enumerate(poses):
        near = big
        if tree is not None:
            idx = tree.query_ball_point(pose[:2], r=reach)
            near = np.concatenate([big, small[np.asarray(idx, dtype=int)]])
        for centre, F in _footprints(robot, pose, delta):
            ph = np.sqrt(np.diag(F))
            sel = near[(lo[near, 0] <= centre[0] + ph[0] + margin)
                       & (hi[near, 0] >= centre[0] - ph[0] - margin)
                       & (lo[near, 1] <= centre[1] + ph[1] + margin)
                       & (hi[near, 1] >= centre[1] - ph[1] - margin)
                       & (lo[near, 2] <= z_hi + margin)
                       & (hi[near, 2] >= z_lo - margin)]
            if not len(sel):
                continue
            n_pairs += len(sel)
            mu, Sig = sub.means[sel], sub.covs[sel]
            g = pair_gaps(Vc, mu, Sig, level, centre, F, z_lo, z_hi)
            low = g <= margin
            if low.any():
                g[low] = np.maximum(g[low], pair_gaps(Vf, mu[low], Sig[low], level,
                                                      centre, F, z_lo, z_hi))
            refined = g <= 0
            for j in np.flatnonzero(refined):
                n_refined += 1
                g[j] = _refined_gap(g[j], mu[j], Sig[j], level, centre, F, z_lo, z_hi)
            # The reported clearance comes from one pair: tighten its
            # direction-grid bound before accepting it (refinement only raises g).
            j = int(np.argmin(g))
            for _ in range(16):
                if refined[j] or g[j] >= min_lb:
                    break
                n_refined += 1
                g[j] = _refined_gap(g[j], mu[j], Sig[j], level, centre, F, z_lo, z_hi)
                refined[j] = True
                j = int(np.argmin(g))
            if g[j] < min_lb:
                min_lb = float(g[j])
                worst = {"sample_index": k, "pose": pose.tolist(),
                         "splat_id": int(sub.ids[sel[j]]), "gap": float(g[j])}
            for jj in np.flatnonzero(g <= 0)[:max(0, 20 - len(collisions))]:
                collisions.append({"sample_index": k, "pose": pose.tolist(),
                                   "splat_id": int(sub.ids[sel[jj]]),
                                   "gap": float(g[jj])})
    return {"passed": not collisions, "n_samples": int(len(poses)),
            "n_pairs_checked": int(n_pairs), "n_refined": int(n_refined),
            "min_clearance_lb": float(min_lb), "worst": worst,
            "collisions": collisions,
            "params": {"tau": tau, "level": level, "delta": delta,
                       "n_coarse": n_coarse, "n_fine": n_fine,
                       "margin": margin, "band_abs": [z_lo, z_hi]}}
