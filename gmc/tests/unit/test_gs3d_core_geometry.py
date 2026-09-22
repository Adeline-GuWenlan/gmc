from dataclasses import replace
import numpy as np
import pytest

from gmc.gs3d.contracts import BodySpec, Pose3, SearchBudget
from gmc.gs3d.geometry import cylinder_ellipsoid_bound
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene, QueryBudget, BudgetExceeded
from gmc.gs3d.spatial import SupportIndex
from gs3d_core_fixtures import UAV, CYLINDER, SWEEPER, KnownBox, FlatSupport, make_scene


def free(report):
    return report.occupancy == "free" and report.safety == "continuous_bound"


@pytest.mark.parametrize("vertical", [False, True])
def test_endpoints_free_but_whole_edge_crosses_obstacle(vertical):
    scene = make_scene([(0, 0, 1.5)], [(.05, .3, .3)])
    body = BodySpec("tiny", .1, .1, "uav_translation")
    a = Pose3((0, 0, .5) if vertical else (-1, 0, 1.5))
    b = Pose3((0, 0, 2.5) if vertical else (1, 0, 1.5))
    oracle = GaussianBodyOracle(scene)
    assert free(oracle.pose(a, body, margin_m=.05))
    assert free(oracle.pose(b, body, margin_m=.05))
    report = oracle.edge(a, b, body, margin_m=.05)
    assert report.occupancy == "occupied"
    assert report.primitive_ids == (0,)


@pytest.mark.parametrize("separation, expected", [(.45, False), (.499, False), (.501, True), (.8, True)])
def test_tangency_and_margin(separation, expected):
    oracle = GaussianBodyOracle(make_scene([(0, 0, 1)], [(.2, .2, .2)]))
    report = oracle.pose(Pose3((separation, 0, 1)), UAV, margin_m=.05)
    assert free(report) == expected
    if expected:
        assert .05 < report.clearance_lower_m <= separation - .45


def test_rotated_anisotropy_changes_actual_collision():
    t = np.pi / 4
    R = np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])
    scene = make_scene([(0, 0, 1.5)], [(.8, .08, .08)], rotations=[R])
    oracle = GaussianBodyOracle(scene)
    body = BodySpec("small", .03, .03, "uav_translation")
    axis = R[:, 0]
    for scale in (.4, .8):
        q = Pose3(tuple(np.array([0, 0, 1.5]) + scale * axis))
        assert not free(oracle.pose(q, body, margin_m=0.))
    assert free(oracle.pose(Pose3(tuple(np.array([0, 0, 1.5]) + .9 * axis)), body, margin_m=.01))
    cross = Pose3((.5, 0., 1.0))
    assert not free(oracle.pose(cross, body, margin_m=.01))
    diagonal_only = replace(scene.gaussians, covs=np.array([np.diag(np.diag(scene.gaussians.covs[0]))]))
    wrong_oracle = GaussianBodyOracle(replace(scene, gaussians=diagonal_only))
    assert free(wrong_oracle.pose(cross, body, margin_m=.01))
    # Opposite diagonal shows why retaining off-diagonal covariance also avoids
    # false blockage by an axis-aligned interpretation.
    assert free(oracle.pose(Pose3((.45, 0., 1.95)), body, margin_m=.01))
    assert not free(wrong_oracle.pose(Pose3((.3, 0., 1.8)), body, margin_m=.01))


def test_conservative_broadphase_matches_brute_force_with_distant_large_support():
    rng = np.random.default_rng(10)
    means = rng.uniform(-10, 10, (250, 3))
    means[0] = [100, 0, 0]
    covs = np.array([np.diag(rng.uniform(.01, 4, 3)) for _ in means])
    covs[0] = np.diag([2600., .01, .01])
    index = SupportIndex(means, covs, 2.)
    for _ in range(12):
        lo = rng.uniform(-3, 0, 3)
        hi = lo + rng.uniform(.2, 3, 3)
        found = {int(i) for chunk in index.query(lo, hi) for i in chunk}
        expected = set(np.flatnonzero(np.all((index.upper >= lo) & (index.lower <= hi), axis=1)))
        assert found == expected
    found = {int(i) for chunk in index.query(np.full(3, -.1), np.full(3, .1)) for i in chunk}
    assert 0 in found


def test_large_support_center_outside_workspace_still_blocks():
    scene = make_scene([(100, 0, 1)], [(101, .1, .1)])
    assert not free(GaussianBodyOracle(scene).pose(Pose3((0, 0, 1)), UAV, margin_m=0.))


def test_body_extent_and_ground_support_are_not_points():
    scene = make_scene([(0, 0, .4)], [(.8, .8, .15)], support=FlatSupport())
    oracle = GaussianBodyOracle(scene)
    assert free(oracle.pose(Pose3((0, 0, .06)), SWEEPER, margin_m=.001))
    assert not free(oracle.pose(Pose3((0, 0, .885)), CYLINDER, margin_m=.001))
    assert free(oracle.pose(Pose3((0, 0, .8)), UAV, margin_m=.05))
    assert oracle.pose(Pose3((0, 0, 2)), CYLINDER, margin_m=.001).safety == "invalid"
    empty = GaussianBodyOracle(make_scene())
    assert empty.pose(Pose3((0, 0, .06)), SWEEPER, margin_m=.001).reason == "ground_support_missing"


def test_coverage_hole_swept_body_and_bounds():
    bounds = ((-2, -1, 0), (2, 1, 3))
    coverage = KnownBox(*bounds, holes=(((-.1, -.2, .5), (.1, .2, 1.5)),))
    oracle = GaussianBodyOracle(make_scene(known=coverage))
    a, b = Pose3((-1, 0, 1)), Pose3((1, 0, 1))
    assert free(oracle.pose(a, UAV, margin_m=.05))
    assert oracle.edge(a, b, UAV, margin_m=.05).reason == "map_unknown"
    for z in (.05, 2.95):
        assert oracle.pose(Pose3((1, 0, z)), UAV, margin_m=0.).reason == "body_exceeds_workspace_bounds"


def test_missing_ground_footprint_support_not_just_centre():
    known = KnownBox((-2, -1, 0), (2, 1, 1), holes=(((.1, -.1, 0), (.2, .1, .1)),))
    scene = make_scene(support=FlatSupport(known=known))
    report = GaussianBodyOracle(scene).pose(Pose3((0, 0, .06)), SWEEPER, margin_m=.001)
    assert report.reason == "ground_swept_support_unproven"


@pytest.mark.parametrize("bad", [np.nan, np.inf, -1])
def test_invalid_pose_body_and_margin_fail_closed(bad):
    oracle = GaussianBodyOracle(make_scene())
    assert oracle.pose(Pose3((0, 0, 1)), replace(UAV, radius_m=bad), margin_m=.05).safety == "invalid"
    assert oracle.pose(Pose3((0, 0, 1)), UAV, margin_m=bad).safety == "invalid"
    if not np.isfinite(bad):
        assert oracle.pose(Pose3((bad, 0, 1)), UAV, margin_m=.05).safety == "invalid"


def test_covariance_validation_and_counted_outward_floor():
    scene = make_scene([(0, 0, 1)], [(.2, .2, .2)])
    for c in (np.diag([-.1, .1, .1]), np.array([[1, .1, 0], [0, 1, 0], [0, 0, 1]])):
        with pytest.raises(ValueError):
            PreparedScene(replace(scene, gaussians=replace(scene.gaussians, covs=np.array([c]))))
    small = replace(scene, gaussians=replace(scene.gaussians, covs=np.array([np.diag([0, 1e-14, .1])])))
    prepared = PreparedScene(small)
    assert prepared.stats["outward_floored_covariances"] == 1
    assert np.linalg.eigvalsh(prepared.covs[0]).min() >= 1e-12
    with pytest.raises(ValueError):
        PreparedScene(replace(scene, bounds_min=(2, -1, 0)))
    with pytest.raises(ValueError):
        replace(scene.gaussians, means=np.zeros((2, 3)), covs=np.repeat(scene.gaussians.covs, 2, axis=0), opacity=np.ones(2), ids=np.zeros(2, dtype=int))


def test_zero_edge_and_bounded_solver_failure():
    oracle = GaussianBodyOracle(make_scene())
    q = Pose3((0, 0, 1))
    assert oracle.pose(q, UAV, margin_m=.05) == oracle.edge(q, q, UAV, margin_m=.05)
    # Contact need not produce an occupied witness but must never be free.
    bound = cylinder_ellipsoid_bound(np.array([.45, 0, 1]), np.array([.45, 0, 1]),
                                    .25, .1, np.array([0, 0, 1]), np.eye(3) * .01, 2.,
                                    margin_m=.05, max_iterations=1)
    assert bound.clearance_lower_m <= .05


def test_budget_caps_inside_geometry():
    limits = SearchBudget(max_oracle_calls=1)
    budget = QueryBudget(limits)
    oracle = GaussianBodyOracle(make_scene(), budget=budget)
    oracle.pose(Pose3((0, 0, 1)), UAV, margin_m=.05)
    with pytest.raises(BudgetExceeded, match="max_oracle_calls"):
        oracle.pose(Pose3((0, 0, 1)), UAV, margin_m=.05)
    assert budget.oracle_calls == 1
