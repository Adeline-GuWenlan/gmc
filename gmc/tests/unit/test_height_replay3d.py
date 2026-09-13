from pathlib import Path

import numpy as np

from gmc.height.pathio import curve_from_dict
from gmc.height.ply3d import GaussianScene3D
from gmc.height.prism import robot_table
from gmc.height.replay3d import pair_gaps, refine_gap, replay_curve, sphere_dirs

RHO = 2.0


def disc(r):
    return np.eye(2) * r * r


def test_gap_is_a_lower_bound_on_true_distance():
    mu = np.array([[1.0, 0.0, 0.5]])
    Sig = (np.eye(3) * 0.1 ** 2)[None]                    # radius 0.2 at level 2
    g = pair_gaps(sphere_dirs(2048), mu, Sig, RHO, np.zeros(2), disc(0.3), 0.0, 1.0)
    assert 0.49 <= g[0] <= 0.5 + 1e-12


def test_overlap_never_certifies_even_after_refinement():
    mu = np.array([0.35, 0.0, 0.5])
    Sig = np.eye(3) * 0.1 ** 2
    g = pair_gaps(sphere_dirs(2048), mu[None], Sig[None], RHO, np.zeros(2), disc(0.3), 0.0, 1.0)
    assert g[0] <= 0
    assert refine_gap(mu, Sig, RHO, np.zeros(2), disc(0.3), 0.0, 1.0,
                      np.array([1.0, 0.0, 0.0])) <= 0


def test_splat_above_band_is_separated_vertically():
    mu = np.array([[0.0, 0.0, 1.5]])
    Sig = (np.eye(3) * 0.1 ** 2)[None]
    g = pair_gaps(sphere_dirs(2048), mu, Sig, RHO, np.zeros(2), disc(0.3), 0.0, 1.0)
    assert 0.29 <= g[0] <= 0.3 + 1e-12


def pillar_scene(y_centre):
    zs = np.linspace(0.1, 1.9, 10)
    means = np.column_stack([np.zeros(10), np.full(10, y_centre), zs])
    covs = np.tile(np.eye(3) * 0.1 ** 2, (10, 1, 1))     # radius 0.2 blobs
    return GaussianScene3D(means, covs, np.full(10, 0.9), np.arange(10), "pillar")


LINE = {"schema_version": 2, "segments": [
    {"kind": "TRANSLATION", "q0": [-1.0, 0.0, 0.0], "q1": [1.0, 0.0, 0.0]}]}


def test_straight_path_through_pillar_fails():
    rep = replay_curve(pillar_scene(0.0), robot_table()["cylinder"],
                       curve_from_dict(LINE), z_floor=0.0)
    assert not rep["passed"] and rep["collisions"]


def test_clearance_below_delta_is_flagged_conservatively():
    # true clearance 0.005 < delta: the dilated footprint must still collide
    rep = replay_curve(pillar_scene(0.3 + 0.2 + 0.005), robot_table()["cylinder"],
                       curve_from_dict(LINE), z_floor=0.0, delta=0.01)
    assert not rep["passed"]


def test_clear_path_passes_with_bounded_clearance():
    rep = replay_curve(pillar_scene(0.3 + 0.2 + 0.05), robot_table()["cylinder"],
                       curve_from_dict(LINE), z_floor=0.0, delta=0.01)
    assert rep["passed"]
    assert 0.03 <= rep["min_clearance_lb"] <= 0.05 + 1e-9


def test_replay_imports_nothing_from_the_projection_or_core():
    src = Path(__import__("gmc.height.replay3d", fromlist=["x"]).__file__).read_text()
    for banned in ("from .band_shadow", "from .project", "from ..mobility",
                   "from ..geometry", "from ..orientation", "from ..verification",
                   "from ..spatial", "gmc.mobility", "gmc.geometry",
                   "gmc.orientation", "gmc.verification", "gmc.spatial"):
        assert banned not in src.split('"""', 2)[-1], banned


def test_diagonal_splat_beyond_kdtree_reach_still_collides():
    # C1 fix: a needle along the xy diagonal has AABB half-extent 0.495 (a
    # "small" splat) but reaches 0.70 from its centre; the centre stays 0.95 m
    # from every pose, beyond the old reach 0.31 + 0.05 + 0.5, yet the needle
    # end is 0.25 m from the path, inside the cylinder.
    u = np.array([1.0, 1.0, 0.0]) / np.sqrt(2)
    Sig = 0.35 ** 2 * np.outer(u, u) + 0.01 ** 2 * (np.eye(3) - np.outer(u, u))
    s3 = GaussianScene3D(np.append(0.95 * u[:2], 1.0)[None], Sig[None],
                         np.array([0.9]), np.array([7]), "diagonal_needle")
    line = {"schema_version": 2, "segments": [
        {"kind": "TRANSLATION", "q0": [-0.5, 0.5, 0.0], "q1": [0.5, -0.5, 0.0]}]}
    rep = replay_curve(s3, robot_table()["cylinder"], curve_from_dict(line), z_floor=0.0)
    assert not rep["passed"] and rep["collisions"][0]["splat_id"] == 7


def test_kdtree_prefilter_keeps_every_pair_the_aabb_test_keeps():
    # spheres at the corners of the footprint box: AABBs meet per axis, but the
    # centres are sqrt(2) times the per-axis bound away
    means = np.array([[sx * 0.8, sy * 0.8, 1.0] for sx in (-1, 1) for sy in (-1, 1)])
    s3 = GaussianScene3D(means, np.tile(np.eye(3) * 0.225 ** 2, (4, 1, 1)),
                         np.full(4, 0.9), np.arange(4), "corners")
    still = {"schema_version": 2, "segments": [
        {"kind": "ROTATION", "q0": [0.0, 0.0, 0.0], "q1": [0.0, 0.0, 0.0]}]}
    curve, cyl = curve_from_dict(still), robot_table()["cylinder"]
    rep = replay_curve(s3, cyl, curve, z_floor=0.0)
    ref = replay_curve(s3, cyl, curve, z_floor=0.0, big_extent=1e9)
    assert ref["n_pairs_checked"] == 4 * ref["n_samples"]
    assert rep["n_pairs_checked"] == ref["n_pairs_checked"]
    assert rep["passed"] and rep["min_clearance_lb"] == ref["min_clearance_lb"]
