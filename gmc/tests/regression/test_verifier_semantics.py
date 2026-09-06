"""Safety regressions for whole-curve verification semantics."""
import numpy as np
import pytest
from shapely.geometry import Polygon, box

from gmc.geometry.support import PairOracle
from gmc.mobility.witness import (PoseCurve, PoseSegment, SegmentKind)
from gmc.types import GaussianSupport2D, PairID, Pose2
from gmc.verification.continuous import (clearance_lb, pair_margin,
                                         translation_safe)
from gmc.verification.path import verify_curve


WORKSPACE = box(-4.0, -4.0, 4.0, 4.0)


def _pose(x, y, theta):
    return Pose2(np.array([x, y]), theta)


def _disc_oracle(radius=1.0):
    half = radius / 2.0
    scene = GaussianSupport2D(np.zeros(2), np.eye(2), half, 0)
    body = GaussianSupport2D(np.zeros(2), np.eye(2), half, 0)
    return PairOracle(PairID(0, 0), scene, body)


def _shifted_disc_oracle(x, primitive_id):
    scene = GaussianSupport2D(np.array([x, 0.0]), np.eye(2), 0.5,
                              primitive_id)
    body = GaussianSupport2D(np.zeros(2), np.eye(2), 0.5, 0)
    return PairOracle(PairID(primitive_id, 0), scene, body)


def _verify(curve, oracles=()):
    return verify_curve(oracles, WORKSPACE, curve, eps_clear=0.0,
                        theta_min=1e-4)


def test_rotation_cannot_teleport():
    segment = PoseSegment(SegmentKind.ROTATION,
                          _pose(0.0, 0.0, 0.0),
                          _pose(1.0, 0.0, np.pi / 2))
    report = _verify(PoseCurve((segment,)))
    assert not report.certified
    assert report.reason == "rotation_changes_position"


def test_translation_cannot_change_theta():
    segment = PoseSegment(SegmentKind.TRANSLATION,
                          _pose(0.0, 0.0, 0.0),
                          _pose(1.0, 0.0, np.pi / 2))
    report = _verify(PoseCurve((segment,)))
    assert not report.certified
    assert report.reason == "translation_changes_theta"


def test_control_points_must_obey_segment_primitive():
    moving_rotation = PoseSegment(
        SegmentKind.ROTATION, _pose(0.0, 0.0, 0.0),
        _pose(0.0, 0.0, 1.0),
        control_points=(_pose(0.1, 0.0, 0.5),),
    )
    twisting_translation = PoseSegment(
        SegmentKind.TRANSLATION, _pose(0.0, 0.0, 0.0),
        _pose(1.0, 0.0, 0.0),
        control_points=(_pose(0.5, 0.0, 0.2),),
    )
    assert _verify(PoseCurve((moving_rotation,))).reason == \
        "rotation_changes_position"
    assert _verify(PoseCurve((twisting_translation,))).reason == \
        "translation_changes_theta"


def test_c0_and_fixed_theta_are_periodic():
    first = PoseSegment(SegmentKind.TRANSLATION,
                        _pose(-1.0, 0.0, 0.0),
                        _pose(0.0, 0.0, 2.0 * np.pi))
    second = PoseSegment(SegmentKind.TRANSLATION,
                         _pose(0.0, 0.0, 0.0),
                         _pose(1.0, 0.0, -2.0 * np.pi))
    report = _verify(PoseCurve((first, second)))
    assert report.certified, report


def test_empty_curve_is_not_a_certificate():
    report = _verify(PoseCurve(()))
    assert not report.certified
    assert report.reason == "empty_curve"
    assert report.min_clearance == -np.inf


def test_requested_endpoint_is_bound_without_fixed_tolerance():
    # A fixed 1e-9 endpoint tolerance can hide a collision-status change for
    # a support whose body-frame mean is very large.  The verifier therefore
    # binds the exact represented request (modulo exact angle periodicity).
    actual = _pose(1.0, 0.0, 0.0)
    requested = _pose(1.0, 0.0, 5e-13)
    segment = PoseSegment(SegmentKind.TRANSLATION, actual, actual)
    report = verify_curve(
        (), WORKSPACE, PoseCurve((segment,)), eps_clear=0.0,
        theta_min=1e-4, expected_start=requested, expected_goal=actual,
    )
    assert not report.certified
    assert report.reason == "start_endpoint_mismatch"


def test_requested_periodic_endpoint_representation_is_accepted():
    actual = _pose(1.0, 0.0, 0.0)
    requested = _pose(1.0, 0.0, 2.0 * np.pi)
    segment = PoseSegment(SegmentKind.TRANSLATION, actual, actual)
    report = verify_curve(
        (), WORKSPACE, PoseCurve((segment,)), eps_clear=0.0,
        theta_min=1e-4, expected_start=requested, expected_goal=requested,
    )
    assert report.certified, report


def test_degenerate_segment_still_checks_collision():
    q = _pose(0.0, 0.0, 0.0)
    segment = PoseSegment(SegmentKind.TRANSLATION, q, q)
    report = _verify(PoseCurve((segment,)), [_disc_oracle()])
    assert not report.certified
    assert report.reason == "collision"


def test_workspace_boundary_contributes_to_clearance_and_eps_floor():
    segment = PoseSegment(
        SegmentKind.TRANSLATION,
        _pose(-1.0, 3.9995, 0.0),
        _pose(1.0, 3.9995, 0.0),
    )
    report = verify_curve((), WORKSPACE, PoseCurve((segment,)),
                          eps_clear=1e-3, theta_min=1e-4)
    assert not report.certified
    assert report.reason == "workspace_clearance"
    assert 0.0 < report.min_clearance < 1e-3


def test_no_obstacle_clearance_is_bounded_by_workspace_not_infinity():
    segment = PoseSegment(
        SegmentKind.TRANSLATION,
        _pose(-3.0, 0.0, 0.0),
        _pose(3.0, 0.0, 0.0),
    )
    report = _verify(PoseCurve((segment,)))
    assert report.certified
    assert report.min_clearance == pytest.approx(1.0)


def test_rotation_near_workspace_boundary_obeys_eps_floor():
    segment = PoseSegment(
        SegmentKind.ROTATION,
        _pose(3.9995, 0.0, 0.0),
        _pose(3.9995, 0.0, 0.5),
    )
    report = verify_curve((), WORKSPACE, PoseCurve((segment,)),
                          eps_clear=1e-3, theta_min=1e-4)
    assert not report.certified
    assert report.reason == "workspace_clearance"


def test_slanted_workspace_distance_is_rounded_down_for_eps_certificate():
    workspace = Polygon([(0.0, 0.0), (1.0, 1.0), (0.0, 2.0)])
    q = _pose(0.766467257480158, 0.7664698723708319, 0.0)
    segment = PoseSegment(SegmentKind.TRANSLATION, q, q)
    # The exact distance to y=x is 1.849006927561922108...e-6, while
    # GEOS rounds upward to 1.8490069275619223e-6.  This epsilon is the first
    # binary64 value above the exact distance, so certification must reject.
    eps = 1.8490069275619221e-6

    report = verify_curve(
        (), workspace, PoseCurve((segment,)), eps_clear=eps,
        theta_min=1e-4,
    )

    assert not report.certified
    assert report.reason == "workspace_clearance"


def test_translation_reports_a_swept_clearance_lower_bound():
    # The exact clearance of this horizontal segment from the unit disc is 1.
    # Point-only adaptive samples used to report 1.0285, which is not a lower
    # bound for the whole segment.
    ok, lower = translation_safe(
        [_disc_oracle()], np.array([-3.0, 2.0]), np.array([3.0, 2.0]), 0.0)
    assert ok
    assert 0.0 < lower <= 1.0 + 1e-10


def test_clearance_branch_and_bound_matches_naive_pair_minimum():
    # Deliberately place the far pair first.  The radial lower bound sorts the
    # near pair first; after its exact finite-direction margin is known, the
    # far support kernel cannot lower the minimum and is safely skipped.
    far = _shifted_disc_oracle(100.0, 1)
    near = _shifted_disc_oracle(0.0, 0)
    point = np.array([3.0, 0.2])
    bounded = clearance_lb([far, near], point, 0.0)

    far_naive = _shifted_disc_oracle(100.0, 1)
    near_naive = _shifted_disc_oracle(0.0, 0)
    naive = min(pair_margin(far_naive, point, 0.0),
                pair_margin(near_naive, point, 0.0))
    assert bounded == pytest.approx(naive, abs=1e-12)
    assert near.calls > 0
    assert far.calls == 0


class _NanOracle:
    def center(self, theta):
        return np.zeros(2)

    def support_values(self, theta, directions):
        return np.full(len(directions), np.nan)

    def theta_lipschitz(self):
        return 0.0


def test_verifier_fails_closed_on_nonfinite_oracle_output():
    segment = PoseSegment(SegmentKind.TRANSLATION,
                          _pose(-1.0, 0.0, 0.0),
                          _pose(1.0, 0.0, 0.0))
    report = _verify(PoseCurve((segment,)), [_NanOracle()])
    assert not report.certified
    assert report.reason == "collision"
    assert report.min_clearance == -np.inf


def test_one_shot_oracle_iterable_is_reused_for_every_segment():
    first = PoseSegment(
        SegmentKind.TRANSLATION, _pose(-2.0, 1.0, 0.0),
        _pose(-1.0, 1.0, 0.0),
    )
    second = PoseSegment(
        SegmentKind.TRANSLATION, _pose(-1.0, 1.0, 0.0),
        _pose(1.0, -1.0, 0.0),
    )
    oracle = _disc_oracle(radius=0.2)

    report = _verify(PoseCurve((first, second)), (item for item in [oracle]))

    assert not report.certified
    assert report.reason == "collision"
    assert report.failed_segment == 1
