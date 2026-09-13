import numpy as np

from gmc.height.band_shadow import (COV_EIG_FLOOR, band_overlap_mask,
                                    outer_shadow_ellipses, shadow_support)

RHO = 2.0


def unit_dirs(k):
    a = 2 * np.pi * np.arange(k) / k
    return np.column_stack([np.cos(a), np.sin(a)])


def random_splats(n, seed):
    rng = np.random.default_rng(seed)
    q, _ = np.linalg.qr(rng.normal(size=(n, 3, 3)))
    sig = np.exp(rng.uniform(-9.0, 0.0, size=(n, 3)))    # axis ratio up to ~8e3
    covs = (q * sig[:, None, :] ** 2) @ q.transpose(0, 2, 1)
    covs = 0.5 * (covs + covs.transpose(0, 2, 1))
    means = rng.uniform(-3, 3, size=(n, 3))
    half = RHO * np.sqrt(covs[:, 2, 2])
    zc = means[:, 2] + rng.uniform(-1, 1, n) * half
    hw = rng.uniform(0.0, 1.5, n) * half
    return means, covs, zc - hw, zc + hw


def test_overlap_is_exact_and_touching_counts():
    mz, szz = np.array([1.0]), np.array([0.01])          # extent [0.8, 1.2]
    assert band_overlap_mask(mz, szz, 1.2, 1.5, RHO)[0]
    assert not band_overlap_mask(mz, szz, np.nextafter(1.2, 2), 1.5, RHO)[0]
    assert band_overlap_mask(mz, szz, 0.0, 0.8, RHO)[0]
    assert not band_overlap_mask(mz, szz, 0.0, 0.79, RHO)[0]


def test_unclipped_band_is_the_full_projection():
    means, covs, _, _ = random_splats(50, 1)
    U = unit_dirs(360)
    for m, C in zip(means, covs):
        h = shadow_support(m, C, -1e9, 1e9, RHO, U)
        full = U @ m[:2] + RHO * np.sqrt(np.einsum("ki,ij,kj->k", U, C[:2, :2], U))
        np.testing.assert_allclose(h, full, rtol=1e-9, atol=1e-11)


def test_outer_ellipse_dominates_shadow_support():
    means, covs, lo, hi = random_splats(300, 2)
    U = unit_dirs(720)
    for i in range(300):
        out = outer_shadow_ellipses(means[i:i + 1], covs[i:i + 1], lo[i], hi[i], RHO)
        c, Q = out["centre"][0], out["Q"][0]
        h_e = U @ c + np.sqrt(np.einsum("ki,ij,kj->k", U, Q, U))
        h_s = shadow_support(means[i], covs[i], lo[i], hi[i], RHO, U)
        assert np.all(h_e - h_s >= -1e-10 * (1 + np.abs(h_s))), i


def test_points_inside_ellipsoid_and_band_project_inside():
    rng = np.random.default_rng(3)
    means, covs, lo, hi = random_splats(60, 4)
    for i in range(60):
        out = outer_shadow_ellipses(means[i:i + 1], covs[i:i + 1], lo[i], hi[i], RHO)
        c, Q = out["centre"][0], out["Q"][0]
        L = np.linalg.cholesky(covs[i])
        w = rng.normal(size=(20000, 3))
        w *= (RHO * rng.uniform(0, 1, (20000, 1)) ** (1 / 3)
              / np.linalg.norm(w, axis=1, keepdims=True))
        p = means[i] + w @ L.T
        p = p[(p[:, 2] >= lo[i]) & (p[:, 2] <= hi[i])]
        if len(p) == 0:
            continue
        d = p[:, :2] - c
        q = np.einsum("ni,ij,nj->n", d, np.linalg.inv(Q), d)
        assert q.max() <= 1 + 1e-9, (i, q.max())


def test_clipping_a_tilted_needle_shrinks_the_bound():
    t = np.deg2rad(45)
    R = np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])
    C = R @ np.diag([0.01 ** 2, 0.01 ** 2, 0.5 ** 2]) @ R.T
    m = np.array([[0.0, 0.0, 1.0]])
    out = outer_shadow_ellipses(m, C[None], 1.0, 1.05, RHO)
    assert out["candidate"][0] == 2
    full_area = np.sqrt(np.linalg.det(RHO ** 2 * C[:2, :2]))
    assert np.sqrt(np.linalg.det(out["Q"][0])) < 0.5 * full_area


def test_degenerate_needle_is_floored_not_dropped():
    C = np.diag([1e-14, 1e-14, 0.04])
    out = outer_shadow_ellipses(np.zeros((1, 3)), C[None], -0.1, 0.1, RHO)
    assert out["floored"][0]
    assert np.linalg.eigvalsh(out["Q"][0] / RHO ** 2).min() >= COV_EIG_FLOOR * (1 - 1e-12)
