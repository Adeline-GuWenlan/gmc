"""Own direct continuous pair verifier (uav.md §11.2): support-plane separation + bisection."""
import numpy as np
import pytest

from gmc.aerial3d.envelopes import EnvelopeTable
from gmc.aerial3d.pairs import domain_from_scene, pairs_from_scene
from gmc.aerial3d.verify import verify_polyline, verify_segment
from gmc.gs3d.contracts import Pose3
from gmc.gs3d.oracle import GaussianBodyOracle
from aerial3d_fixtures import UAV
from gs3d_core_fixtures import make_scene

M = .05


def _setup(scene):
    frame, domain = domain_from_scene(scene, UAV, margin_m=M)
    pairs, _ = pairs_from_scene(scene, UAV, frame, domain, margin_m=M)
    return frame, domain, EnvelopeTable(pairs)


def test_between_waypoint_obstacle_is_caught_although_endpoints_are_free():
    # gs3d fixture "between-waypoint": thin plate between two free endpoints
    scene = make_scene([(0., 0., 1.)], [(.05, .3, .3)], lower=(-2., -1., 0.), upper=(2., 1., 2.))
    frame, domain, table = _setup(scene)
    a, b = np.array([-1., 0., 1.]), np.array([1., 0., 1.])
    assert verify_segment(table, domain, a, a)["status"] == "CERTIFIED"
    assert verify_segment(table, domain, b, b)["status"] == "CERTIFIED"
    out = verify_segment(table, domain, a, b)
    assert out["status"] == "COLLISION"
    assert out["pair_ids"] == [0]


def test_certified_segments_agree_with_the_baseline_edge_oracle():
    rng = np.random.default_rng(0)
    means = rng.uniform((-1.5, -.8, .3), (1.5, .8, 1.7), size=(12, 3))
    axes = rng.uniform(.05, .4, size=(12, 3))
    rots = []
    for s in range(12):
        q, _ = np.linalg.qr(np.random.default_rng(100 + s).normal(size=(3, 3)))
        rots.append(q * np.sign(np.linalg.det(q)))
    scene = make_scene(means, axes, rotations=np.stack(rots), lower=(-2., -1., 0.), upper=(2., 1., 2.))
    frame, domain, table = _setup(scene)
    oracle = GaussianBodyOracle(scene)
    certified = collided = 0
    for _ in range(500):
        a = rng.uniform(domain.bbox_lower, domain.bbox_upper)
        b = a + rng.normal(size=3) * .4
        if domain.row_slack(b) <= 0:
            continue
        out = verify_segment(table, domain, a, b)
        base = oracle.edge(Pose3(tuple(a)), Pose3(tuple(b)), UAV, margin_m=M)
        if out["status"] == "CERTIFIED":
            certified += 1
            assert base.occupancy == "free"
            # both are lower bounds on the same clearance; neither may exceed the truth,
            # so the own bound cannot exceed the baseline's by more than the baseline's looseness
            assert out["clearance_lower_m"] > M
        elif out["status"] == "COLLISION":
            collided += 1
            assert base.occupancy != "free"
    assert certified > 20 and collided > 20


def test_clearance_bound_matches_the_analytic_value_for_a_sphere():
    s = .1
    scene = make_scene([(0., 0., 1.)], [(2 * s, 2 * s, 2 * s)], lower=(-2., -1., 0.), upper=(2., 1., 2.))
    frame, domain, table = _setup(scene)
    gap = .02
    y = 2 * s + UAV.radius_m + M + gap  # body side 'gap' beyond the margin
    out = verify_segment(table, domain, np.array([-.5, y, 1.]), np.array([.5, y, 1.]), pad_m=.1)
    assert out["status"] == "CERTIFIED"
    assert out["clearance_lower_m"] == pytest.approx(M + gap, abs=1e-6)


def test_polyline_trace_and_domain_exit():
    scene = make_scene(lower=(-2., -1., 0.), upper=(2., 1., 2.))
    frame, domain, table = _setup(scene)
    pts = np.array([[-1., 0, 1.], [0., 0., 1.], [1., 0., 1.]])
    trace = verify_polyline(table, domain, pts)
    assert trace["status"] == "CERTIFIED" and len(trace["segments"]) == 2
    out = verify_polyline(table, domain, np.array([[-1., 0, 1.], [0., 0., 1.95]]))
    assert out["status"] != "CERTIFIED" and out["segments"][0]["reason"] == "outside_domain"
