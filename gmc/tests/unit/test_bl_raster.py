"""bl B2 shared rasteriser tests (``experiments/bl_raster.py``). Pure: synthetic Gaussians, no scene.

The judge-facing measurement (map vs ``GaussianBodyOracle`` on sampled cells of the real regions) is
``bl_raster_check.py`` (sbatch), not a unit test.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

GMC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(GMC / "experiments"))

import bl_raster as R  # noqa: E402

LEVEL = 2.


def _rot(rng):
    q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    return q * np.sign(np.linalg.det(q))


def _gaussians(rng, n, flat=False):
    means, covs = [], []
    for _ in range(n):
        s = rng.uniform(.005, .3, 3)
        if flat:
            s[2] = rng.uniform(.002, .01)
        Q = _rot(rng) if not flat else _yaw_tilt(rng)
        covs.append(Q @ np.diag(s ** 2) @ Q.T)
        means.append(rng.uniform([-1, -1, -.2], [1, 1, 1.2]))
    return np.array(means), np.array(covs)


def _yaw_tilt(rng):
    a, t = rng.uniform(0, 2 * np.pi), rng.uniform(-.2, .2)
    Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])
    return Rz @ Rx


def _maximiser(mu, C, zlo, zhi, d):
    """Explicit point of E n slab attaining the closed-form support in planar direction d (or None if empty)."""
    A = np.linalg.cholesky(C)
    az = A[2]
    n = az / np.linalg.norm(az)
    sz = math.sqrt(C[2, 2])
    tlo, thi = (zlo - mu[2]) / (LEVEL * sz), (zhi - mu[2]) / (LEVEL * sz)
    if tlo > 1 or thi < -1:
        return None
    tlo, thi = max(tlo, -1.), min(thi, 1.)
    c = LEVEL * (d[0] * A[0] + d[1] * A[1])
    alpha = c @ n
    cperp = c - alpha * n
    t = float(np.clip(alpha / np.linalg.norm(c), tlo, thi))
    w = t * n + math.sqrt(max(1 - t * t, 0.)) * (cperp / np.linalg.norm(cperp) if np.linalg.norm(cperp) > 0 else 0)
    return mu + LEVEL * A @ w


def test_slab_support_is_attained_and_bounds_samples():
    rng = np.random.default_rng(1)
    means, covs = _gaussians(rng, 60)
    means2, covs2 = _gaussians(rng, 60, flat=True)
    means, covs = np.r_[means, means2], np.r_[covs, covs2]
    zlo, zhi = .019, 1.751
    dirs = R.directions(16)
    H, keep = R.slab_support(means, covs, LEVEL, zlo, zhi, dirs)
    for i in range(len(means)):
        A = np.linalg.cholesky(covs[i])
        w = rng.normal(size=(20000, 3))
        w *= (rng.uniform(size=(20000, 1)) ** (1 / 3)) / np.linalg.norm(w, axis=1, keepdims=True)
        X = means[i] + LEVEL * w @ A.T
        X = X[(X[:, 2] >= zlo) & (X[:, 2] <= zhi)]
        assert keep[i] == (_maximiser(means[i], covs[i], zlo, zhi, dirs[0]) is not None)
        if not keep[i]:
            assert len(X) == 0
            continue
        if len(X):                                  # soundness: every sampled point is inside every half-plane
            assert np.all(X[:, :2] @ dirs.T <= H[i][None, :] + 1e-12)
        for k, d in enumerate(dirs):                # tightness: the closed form is attained by a point of E n slab
            x = _maximiser(means[i], covs[i], zlo, zhi, d)
            assert (x - means[i]) @ np.linalg.solve(covs[i], x - means[i]) <= LEVEL ** 2 * (1 + 1e-9)
            assert zlo - 1e-9 <= x[2] <= zhi + 1e-9
            assert abs(d @ x[:2] - H[i, k]) < 1e-7


def _dist_lower(p, mu, C, zlo, zhi, ndir=2048):
    """Rigorous lower bound on dist(p, S) for planar points p: max over directions of d.p - h_S(d)."""
    dirs = R.directions(ndir)
    H, keep = R.slab_support(mu[None], C[None], LEVEL, zlo, zhi, dirs)
    if not keep[0]:
        return np.full(len(p), np.inf)
    return np.concatenate([np.max(q @ dirs.T - H[0][None, :], axis=1) for q in np.array_split(p, max(1, len(p) // 4000))])


def _meta(nx=1.6, ny=1.2, angle=0.):
    c, s = math.cos(angle), math.sin(angle)
    Rm = np.array([[c, s, 0.], [-s, c, 0.], [0., 0., 1.]])        # world_to_plan
    return {"region": "T", "robot": "t", "level": LEVEL, "margin_m": .001, "tau": .3, "z_c": .885,
            "known_route_lower_m": [-nx / 2, -ny / 2, 0.], "known_route_upper_m": [nx / 2, ny / 2, 2.43],
            "frame": {"world_to_plan": Rm.tolist(), "origin_world_m": [3., -2., -1.]}}


BODY = {"radius_m": .3, "half_height_m": .865, "ground_clearance_m": .02}
SMALL = {"radius_m": .1, "half_height_m": .3, "ground_clearance_m": .02}


@pytest.mark.parametrize("seed", [3, 4])
def test_free_cells_are_free_everywhere_and_marked_cells_are_near(seed):
    rng = np.random.default_rng(seed)
    meta = _meta(3.2, 2.4)
    body = SMALL
    meta["z_c"] = .02 + .3
    means, covs = _gaussians(rng, 6)
    means[:, 2] = rng.uniform(.0, .8, len(means))
    scene = {"means": means, "covs": covs, "meta": meta}
    res = .02
    arrays, info = R.build(scene, body, res_m=res)
    zlo, zhi = info["slab_z"]
    rho = info["rho_m"]
    gauss = arrays["gauss"].astype(bool)
    ny, nx = gauss.shape
    iy, ix = np.nonzero(~gauss)
    c = R.centre(info, iy, ix)
    # every point of a free cell: corners, edge midpoints, centre, random points
    offs = np.array([[a, b] for a in (-.5, 0, .5) for b in (-.5, 0, .5)]) * res
    pts = (c[:, None, :] + offs[None]).reshape(-1, 2)
    pts = np.r_[pts, c + rng.uniform(-.5, .5, size=c.shape) * res]
    pts = pts[rng.permutation(len(pts))[:40000]]
    for i in range(len(means)):
        dl = _dist_lower(pts, means[i], covs[i], zlo, zhi)
        assert np.all(dl > rho - 1e-12), f"free cell within rho of gaussian {i}"
    # tightness: a marked cell's centre is within rho + res/sqrt(2) of some S (+ the refinement's residual)
    iy, ix = np.nonzero(gauss)
    c = R.centre(info, iy, ix)
    best = np.full(len(c), np.inf)
    for i in range(len(means)):
        best = np.minimum(best, _dist_lower(c, means[i], covs[i], zlo, zhi, ndir=4096))
    over = res / math.sqrt(2) + 2e-4
    assert np.all(best <= rho + over)


def test_floor_splat_counts_only_its_cap_above_the_chassis():
    meta = _meta(3., 3.)
    body = BODY
    zlo = meta["z_c"] - body["half_height_m"] - meta["margin_m"]           # 0.019
    assert abs(zlo - .019) < 1e-12
    flat = np.diag([.5 ** 2, .5 ** 2, .005 ** 2])                          # 2-sigma: 1 m x 1 m x 1 cm
    below = {"means": np.array([[0., 0., zlo - .0101]]), "covs": flat[None], "meta": meta}    # top 0.0189
    cap = {"means": np.array([[0., 0., zlo - .0099]]), "covs": flat[None], "meta": meta}      # top 0.0191
    a0, i0 = R.build(below, body, res_m=.01)
    a1, i1 = R.build(cap, body, res_m=.01)
    assert i0["gauss_cells"] == 0
    assert i1["gauss_cells"] > 0
    # the cap is the tiny disk where the splat's top pokes above 0.019: its C-space is ~ disk(rho + cap radius)
    t = .0001 / .01                                                         # slab cuts at 1 - t of the half-height
    cap_r = 1. * math.sqrt(1 - (1 - t) ** 2)
    area = i1["gauss_cells"] * .01 ** 2
    assert area < math.pi * (i1["rho_m"] + cap_r + .02) ** 2
    assert area > math.pi * (i1["rho_m"] + cap_r) ** 2 * .95
    full = {"means": np.array([[0., 0., .5]]), "covs": flat[None], "meta": meta}
    a2, i2 = R.build(full, body, res_m=.01)
    assert i2["gauss_cells"] > 3 * i1["gauss_cells"]


@pytest.mark.parametrize("angle", [0., 1.0, 2.2])
def test_known_and_workspace_layers_match_the_judge_conditions_at_every_point(angle):
    rng = np.random.default_rng(7)
    meta = _meta(2., 1.4, angle)
    scene = {"means": np.zeros((0, 3)), "covs": np.zeros((0, 3, 3)), "meta": meta}
    arrays, info = R.build(scene, BODY, res_m=.01)
    free = arrays["occ"] == 0
    iy, ix = np.nonzero(free)
    c = R.centre(info, iy, ix)
    pts = np.r_[c, c + rng.uniform(-.5, .5, size=c.shape) * .01,
                c + np.array([.005, .005]), c - np.array([.005, .005])]
    assert R.known_ok(meta, BODY, pts).all()
    assert (R.workspace_margin(meta, BODY, pts) > meta["margin_m"]).all()
    # and the layers are not vacuous: some box points fail each condition
    probe = np.c_[rng.uniform(-1, 1, 4000), rng.uniform(-.7, .7, 4000)]
    assert (~R.known_ok(meta, BODY, probe)).any()
    ws = R.workspace_margin(meta, BODY, probe) > meta["margin_m"]
    # A disk inside a rotated box never pushes its world AABB out of the box's world AABB (corner geometry:
    # r - r sqrt(2) cos(psi) <= 0), so the workspace bound only bites in an axis-aligned frame, where it takes the
    # margin (1 mm) off the box edges that the known-space test allows the body to touch.
    assert (R.known_ok(meta, BODY, probe) & ~ws).any() == (angle == 0.)


def test_save_load_round_trip_and_fail_closed(tmp_path):
    meta = _meta()
    scene = {"means": np.array([[0., 0., .5]]), "covs": np.eye(3)[None] * .01, "meta": meta}
    arrays, info = R.build(scene, BODY, res_m=.02)
    p = tmp_path / "x.npz"
    side = R.save(p, arrays, info)
    a2, s2 = R.load(p, expect_sha256=side["sha256"])
    assert np.array_equal(a2["occ"], arrays["occ"]) and s2["sha256"] == side["sha256"]
    with pytest.raises(ValueError):
        R.load(p, expect_sha256="0" * 64)
    doc = json.loads(Path(f"{p}.json").read_text())
    doc["sha256"] = "1" * 64
    Path(f"{p}.json").write_text(json.dumps(doc))
    with pytest.raises(ValueError):
        R.load(p)


def test_occupied_at_and_nearest_free():
    meta = _meta(2., 2.)
    scene = {"means": np.array([[0., 0., .5]]), "covs": (np.eye(3) * .05 ** 2)[None], "meta": meta}
    arrays, info = R.build(scene, SMALL | {"half_height_m": .865}, res_m=.02)
    assert R.occupied_at(arrays, info, [[0., 0.]])[0]
    assert R.occupied_at(arrays, info, [[50., 0.]])[0]
    iy, ix, c, d = R.nearest_free(arrays, info, [0., 0.])
    assert not R.occupied_at(arrays, info, [c])[0] and arrays["occ"][iy, ix] == 0
    assert d > .1 + .001 + .1 - .03
    free = np.argwhere(arrays["occ"] == 0)
    brute = np.linalg.norm(R.centre(info, free[:, 0], free[:, 1]), axis=1).min()
    assert abs(d - brute) < 1e-12
    assert R.nearest_free(arrays, info, [0., 0.], max_m=.05) is None
