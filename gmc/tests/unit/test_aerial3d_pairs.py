"""Pair C-obstacle kernel (uav.md §5) for the axisymmetric UAV: support, AABB, frame, domain."""
import numpy as np
import pytest

from gmc.aerial3d.pairs import (PairSet, PlanningFrame, domain_from_scene,
                                pairs_from_scene)
from gmc.gs3d.contracts import BodySpec, Pose3
from gmc.gs3d.geometry import cylinder_ellipsoid_bound
from gmc.gs3d.integration import RouteBoxKnownSpace
from gmc.gs3d.oracle import GaussianBodyOracle
from gs3d_core_fixtures import UAV, make_scene

R, H, M, KAPPA = .25, .10, .05, 2.


def _unit(n, seed=0):
    v = np.random.default_rng(seed).normal(size=(n, 3))
    return v / np.linalg.norm(v, axis=1, keepdims=True)


def _rotation(seed):
    q, _ = np.linalg.qr(np.random.default_rng(seed).normal(size=(3, 3)))
    return q * np.sign(np.linalg.det(q))


def _pairs(means, covs):
    means = np.asarray(means, float).reshape(-1, 3)
    covs = np.asarray(covs, float).reshape(-1, 3, 3)
    return PairSet(means, covs, np.arange(len(means)), level=KAPPA, radius=R,
                   half_height=H, margin=M)


def test_support_value_of_spherical_gaussian_matches_closed_form():
    s = .1
    p = _pairs([(1., 2., 3.)], s * s * np.eye(3))
    for u, expected in [((0, 0, 1), 3. + KAPPA * s + H + M),
                        ((0, 0, -1), -3. + KAPPA * s + H + M),
                        ((1, 0, 0), 1. + KAPPA * s + R + M),
                        ((0, -1, 0), -2. + KAPPA * s + R + M)]:
        assert p.support(np.asarray(u, float))[0] == pytest.approx(expected, abs=1e-12)


def test_support_points_are_on_the_boundary_and_inside_the_c_obstacle():
    cov = _rotation(3) @ np.diag([.3, .02, .1]) ** 2 @ _rotation(3).T
    p = _pairs([(.2, -.1, .5)], cov)
    U = _unit(200, 1)
    X = p.support_points(U)[0]
    # u . x(u) equals h(u): x(u) is a support point.
    assert np.allclose(np.sum(U * X, axis=1), p.support(U)[0], atol=1e-12)
    # Each support point satisfies every support inequality (dense check).
    V = _unit(20000, 2)
    hv = p.support(V)[0]
    assert np.max(X @ V.T - hv[None, :]) <= 1e-12


def test_c_obstacle_membership_agrees_with_the_baseline_oracle():
    """O = E ⊕ Cyl ⊕ B(m): p is outside O iff the baseline clearance exceeds m."""
    rng = np.random.default_rng(7)
    cov = _rotation(11) @ np.diag([.25, .04, .12]) ** 2 @ _rotation(11).T
    mean = np.array([0., 0., 1.])
    p = _pairs([mean], cov)
    V = _unit(40000, 5)
    hv = p.support(V)[0]
    checked = 0
    for q in rng.uniform([-1, -1, 0.2], [1, 1, 1.8], size=(300, 3)):
        base = cylinder_ellipsoid_bound(q, q, R, H, mean, cov, KAPPA, margin_m=M,
                                        max_iterations=200)
        outside_depth = float(np.max(V @ q - hv))  # <= true distance to O
        if base.overlap:
            assert outside_depth <= 1e-9
            checked += 1
        elif base.clearance_lower_m > M + 1e-3:
            # Baseline certifies p free with room; then p is outside O by at least that room.
            assert outside_depth > 0
            assert outside_depth >= base.clearance_lower_m - M - 2e-3
            checked += 1
    assert checked > 200


def test_exact_aabb_is_the_support_on_the_axes():
    cov = _rotation(4) @ np.diag([.2, .05, .01]) ** 2 @ _rotation(4).T
    p = _pairs([(1., -1., .4)], cov)
    for k in range(3):
        e = np.eye(3)[k]
        assert p.aabb_upper[0, k] >= p.support(e)[0]
        assert p.aabb_upper[0, k] - p.support(e)[0] < 1e-7
        assert p.aabb_lower[0, k] <= -p.support(-e)[0]
        assert -p.support(-e)[0] - p.aabb_lower[0, k] < 1e-7


def test_planning_frame_is_rigid_rotation_about_z_and_round_trips():
    t = .7
    R_ = np.array([[np.cos(t), np.sin(t), 0], [-np.sin(t), np.cos(t), 0], [0, 0, 1]])
    f = PlanningFrame(R_, np.array([3., -2., .5]))
    w = np.random.default_rng(0).normal(size=(10, 3))
    assert np.allclose(f.to_world(f.to_plan(w)), w, atol=1e-12)
    with pytest.raises(ValueError):
        PlanningFrame(_rotation(1), np.zeros(3))  # tilts z: the cylinder would not be invariant


def test_pairs_from_scene_keeps_opacity_above_tau_and_full_covariance():
    scene = make_scene([(0, 0, 1), (1, 0, 1)], [(.3, .2, .1), (.3, .2, .1)],
                       rotations=np.stack([_rotation(1), _rotation(2)]))
    g = scene.gaussians
    object.__setattr__(g, "opacity", np.array([.9, .2]))  # second below tau=.3
    frame, domain = domain_from_scene(scene, UAV, margin_m=M)
    pairs, stats = pairs_from_scene(scene, UAV, frame, domain, margin_m=M)
    assert list(pairs.ids) == [0]
    assert np.allclose(pairs.covs[0], g.covs[0])  # identity frame: covariance untouched
    assert abs(pairs.covs[0][0, 1]) > 1e-6  # off-diagonals preserved
    assert stats["opacity_selected"] == 1


def test_route_frame_pairs_rotate_covariance_and_preserve_support():
    t = 1.1
    Rw = np.array([[np.cos(t), np.sin(t), 0], [-np.sin(t), np.cos(t), 0], [0, 0, 1]])
    origin = np.array([2., 1., -.3])
    known = RouteBoxKnownSpace(tuple(origin), tuple(map(tuple, Rw)), (-1, -1, 0), (2, 1.5, 2))
    lo, hi = known.world_bounds()
    cov = _rotation(9) @ np.diag([.2, .05, .1]) ** 2 @ _rotation(9).T
    mean_w = origin + np.array([.5, .2, 1.]) @ Rw
    scene = make_scene([mean_w], [(1, 1, 1)], lower=lo, upper=hi, known=known)
    object.__setattr__(scene.gaussians, "covs", cov[None])
    frame, domain = domain_from_scene(scene, UAV, margin_m=M)
    pairs, _ = pairs_from_scene(scene, UAV, frame, domain, margin_m=M)
    world = _pairs([mean_w], cov)
    for u_w in _unit(50, 3):
        u_p = frame.R @ u_w
        # h_plan(R u) = h_world(u) - u . origin   (x_plan = R (x_world - origin))
        assert pairs.support(u_p)[0] == pytest.approx(world.support(u_w)[0] - u_w @ origin, abs=1e-12)


def test_known_box_domain_matches_the_baseline_oracle_in_an_empty_scene():
    lower, upper = (-2., -1., 0.), (2., 1., 3.)
    scene = make_scene(lower=lower, upper=upper)
    frame, domain = domain_from_scene(scene, UAV, margin_m=M)
    oracle = GaussianBodyOracle(scene)
    rng = np.random.default_rng(1)
    agree = 0
    for q in rng.uniform(np.array(lower) - .1, np.array(upper) + .1, size=(2000, 3)):
        slack = domain.row_slack(q)  # >0 strictly inside every row, <0 outside
        if abs(slack) < 1e-6:
            continue
        free = oracle.pose(Pose3(tuple(q)), UAV, margin_m=M).occupancy == "free"
        assert free == (slack > 0)
        agree += 1
    assert agree > 1500
    assert np.allclose(domain.bbox_lower, np.array(lower) + (R + M, R + M, H + M))
    assert np.allclose(domain.bbox_upper, np.array(upper) - (R + M, R + M, H + M))


def test_route_prism_domain_matches_the_baseline_oracle_in_an_empty_scene():
    t = 1.13  # the gallery frame is ~64 degrees off the world axes
    Rw = np.array([[np.cos(t), np.sin(t), 0], [np.sin(t), -np.cos(t), 0], [0, 0, 1]])
    origin = np.array([9.8, 26.9, -1.2])
    known = RouteBoxKnownSpace(tuple(origin), tuple(map(tuple, Rw)), (-3., -.35, 0.), (3.7, 2.75, 2.43))
    lo, hi = known.world_bounds()
    scene = make_scene(lower=lo, upper=hi, known=known)
    frame, domain = domain_from_scene(scene, UAV, margin_m=M)
    assert np.allclose(frame.R, Rw) and np.allclose(frame.origin, origin)
    oracle = GaussianBodyOracle(scene)
    rng = np.random.default_rng(2)
    agree = inside = 0
    for qp in rng.uniform((-3.2, -.5, -.1), (3.9, 2.9, 2.6), size=(3000, 3)):
        slack = domain.row_slack(qp)
        if abs(slack) < 1e-6:
            continue
        q = frame.to_world(qp)
        free = oracle.pose(Pose3(tuple(q)), UAV, margin_m=M).occupancy == "free"
        assert free == (slack > 0), (qp, slack)
        agree += 1
        inside += free
    assert agree > 2500 and inside > 500
    e = R * (abs(Rw[0, 0]) + abs(Rw[0, 1]))
    assert domain.bbox_lower[0] == pytest.approx(-3. + e, abs=1e-7)
    assert domain.bbox_lower[2] == pytest.approx(H + M, abs=1e-7)  # world z bound with margin binds
