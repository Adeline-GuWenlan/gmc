"""Band-clipped xy-shadow of a 3D Gaussian iso-ellipsoid (spec §3.2).

Soundness is analytic, not numeric: candidate 1 is the projection of the whole
ellipsoid (a superset of any clipped part); candidate 2 contains the shadow
because every cross-section centre lies on the segment [m(t_a), m(t_b)] and
every cross-section radius is at most r_max, and
(sqrt(a) + sqrt(b))^2 <= (1 + 1/k) a + (1 + k) b for every k > 0.
Inflation and the eigenvalue floor only grow the result.
"""
import numpy as np

REL_INFLATE = 1e-9
COV_EIG_FLOOR = 1e-9


def band_overlap_mask(means_z, s_zz, z_lo_abs, z_hi_abs, level):
    half = float(level) * np.sqrt(np.asarray(s_zz, dtype=np.float64))
    mz = np.asarray(means_z, dtype=np.float64)
    return (mz - half <= float(z_hi_abs)) & (mz + half >= float(z_lo_abs))


def _t_range(mz, szz, z_lo_abs, z_hi_abs, level):
    sq = np.sqrt(szz)
    ta = np.clip((z_lo_abs - mz) / sq, -level, level)
    tb = np.clip((z_hi_abs - mz) / sq, -level, level)
    return ta, tb, sq


def shadow_support(mean3, cov3, z_lo_abs, z_hi_abs, level, U):
    m = np.asarray(mean3, dtype=np.float64)
    C = np.asarray(cov3, dtype=np.float64)
    U = np.asarray(U, dtype=np.float64)
    ta, tb, sq = _t_range(m[2], C[2, 2], float(z_lo_abs), float(z_hi_abs),
                          float(level))
    if ta > tb:
        raise ValueError("band does not overlap the splat")
    c = C[:2, 2]
    S = C[:2, :2] - np.outer(c, c) / C[2, 2]
    beta = (U @ c) / sq
    gamma = np.sqrt(np.maximum(np.einsum("ki,ij,kj->k", U, S, U), 0.0))
    denom = np.sqrt(beta ** 2 + gamma ** 2)
    tstar = np.where(denom > 0, level * beta / np.where(denom > 0, denom, 1), 0.0)
    t = np.clip(tstar, ta, tb)
    return U @ m[:2] + beta * t + gamma * np.sqrt(np.maximum(level ** 2 - t ** 2, 0.0))


def outer_shadow_ellipses(means, covs, z_lo_abs, z_hi_abs, level):
    means = np.asarray(means, dtype=np.float64)
    covs = np.asarray(covs, dtype=np.float64)
    level = float(level)
    mz, szz = means[:, 2], covs[:, 2, 2]
    ta, tb, sq = _t_range(mz, szz, float(z_lo_abs), float(z_hi_abs), level)
    if np.any(ta > tb):
        raise ValueError("every row must overlap the band")
    A = covs[:, :2, :2]
    c = covs[:, :2, 2]
    S = A - np.einsum("ni,nj->nij", c, c) / szz[:, None, None]
    S = 0.5 * (S + S.transpose(0, 2, 1))

    Q1 = level ** 2 * A
    ctr1 = means[:, :2]

    tmin = np.where((ta <= 0) & (tb >= 0), 0.0,
                    np.minimum(np.abs(ta), np.abs(tb)))
    R = (level ** 2 - tmin ** 2)[:, None, None] * S
    d = c * ((tb - ta) / (2 * sq))[:, None]
    D = np.einsum("ni,nj->nij", d, d)
    trD = np.einsum("ni,ni->n", d, d)
    trR = np.einsum("nii->n", R)
    has_d = (trD > 0) & (trR > 0)
    k = np.sqrt(np.where(has_d, trD / np.where(trR > 0, trR, 1.0), 1.0))
    Q2 = np.where(has_d[:, None, None],
                  (1 + 1 / k)[:, None, None] * D + (1 + k)[:, None, None] * R,
                  R + D)
    ctr2 = means[:, :2] + c * ((0.5 * (ta + tb)) / sq)[:, None]

    use2 = np.linalg.det(Q2) < np.linalg.det(Q1)
    Q = np.where(use2[:, None, None], Q2, Q1) * (1.0 + REL_INFLATE)
    centre = np.where(use2[:, None], ctr2, ctr1)

    cov = 0.5 * (Q + Q.transpose(0, 2, 1)) / level ** 2
    w, v = np.linalg.eigh(cov)
    floored = w.min(axis=1) < COV_EIG_FLOOR
    w = np.maximum(w, COV_EIG_FLOOR)
    cov = (v * w[:, None, :]) @ v.transpose(0, 2, 1)
    Q = level ** 2 * 0.5 * (cov + cov.transpose(0, 2, 1))
    return {"centre": centre, "Q": Q,
            "candidate": np.where(use2, 2, 1), "floored": floored}
