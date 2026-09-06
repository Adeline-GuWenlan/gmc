"""Adversarial checks for the reversible-DD M9 certificate contract."""
import numpy as np
import shapely

from gmc.dynamics.reversible_dd import (CorridorVerificationContext,
                                        ReversibleDDCertifier,
                                        ReversibleDiffDrive)
from gmc.io.robot_io import ellipse_robot
from gmc.mobility.witness import PoseCurve, PoseSegment, SegmentKind
from gmc.synth import empty_scene, single_obstacle
from gmc.types import CertStatus, Pose2


def _q(x, y, theta):
    return Pose2(np.array([x, y], dtype=float), theta)


def _certify(curve, start=None, goal=None):
    class Budget:
        candidate_curve = curve
        max_wall_seconds = 1.0

    start = curve.segments[0].q0 if start is None and curve.segments else start
    goal = curve.segments[-1].q1 if goal is None and curve.segments else goal
    scene = empty_scene(workspace=(-10, 10, -10, 10))
    context = CorridorVerificationContext(
        shapely.box(-10, -10, 10, 10), scene, ellipse_robot(0.1, 0.08),
        eps_clear=0.0, theta_min=1e-4,
    )
    return ReversibleDDCertifier().certify(
        context, start, goal,
        ReversibleDiffDrive(), Budget(),
    )


def test_translation_cannot_change_heading():
    curve = PoseCurve((PoseSegment(
        SegmentKind.TRANSLATION, _q(0, 0, 0), _q(1, 0, 0.2)),))
    result = _certify(curve)
    assert result.status is CertStatus.UNKNOWN
    assert result.reason_code == "translation_changes_heading"


def test_rotation_cannot_change_position():
    curve = PoseCurve((PoseSegment(
        SegmentKind.ROTATION, _q(0, 0, 0), _q(0.1, 0, 1.0)),))
    result = _certify(curve)
    assert result.status is CertStatus.UNKNOWN
    assert result.reason_code == "rotation_changes_position"


def test_curve_endpoints_are_part_of_certificate():
    curve = PoseCurve((PoseSegment(
        SegmentKind.TRANSLATION, _q(0, 0, 0), _q(1, 0, 0)),))
    result = _certify(curve, start=_q(-1, 0, 0), goal=_q(1, 0, 0))
    assert result.status is CertStatus.UNKNOWN
    assert result.reason_code == "endpoint_mismatch"


def test_periodic_c0_join_is_accepted():
    curve = PoseCurve((
        PoseSegment(SegmentKind.TRANSLATION,
                    _q(0, 0, 2 * np.pi), _q(1, 0, 2 * np.pi)),
        PoseSegment(SegmentKind.TRANSLATION,
                    _q(1, 0, 0), _q(2, 0, 0)),
    ))
    result = _certify(curve, start=_q(0, 0, 0), goal=_q(2, 0, 2 * np.pi))
    assert result.status is CertStatus.CERTIFIED


def test_empty_curve_is_not_an_executable_edge_certificate():
    curve = PoseCurve(())

    class Budget:
        candidate_curve = curve
        max_wall_seconds = 1.0

    result = ReversibleDDCertifier().certify(
        CorridorVerificationContext(
            shapely.box(-1, -1, 1, 1),
            empty_scene(workspace=(-1, 1, -1, 1)), ellipse_robot(0.1, 0.08),
            eps_clear=0.0, theta_min=1e-4,
        ), _q(0, 0, 0), _q(0, 0, 0),
        ReversibleDiffDrive(), Budget(),
    )
    assert result.status is CertStatus.UNKNOWN
    assert result.reason_code == "empty_candidate_curve"


def test_sideways_translation_is_not_differential_drive_executable():
    curve = PoseCurve((PoseSegment(
        SegmentKind.TRANSLATION, _q(0, 0, 0), _q(0, 1, 0)),))
    result = _certify(curve)
    assert result.status is CertStatus.UNKNOWN
    assert result.reason_code == "translation_not_heading_aligned"


def test_backward_translation_is_reversible_drive_executable():
    curve = PoseCurve((PoseSegment(
        SegmentKind.TRANSLATION, _q(0, 0, 0), _q(-1, 0, 0)),))
    result = _certify(curve)
    assert result.status is CertStatus.CERTIFIED


def test_missing_corridor_cannot_be_certified():
    curve = PoseCurve((PoseSegment(
        SegmentKind.TRANSLATION, _q(0, 0, 0), _q(1, 0, 0)),))

    class Budget:
        candidate_curve = curve
        max_wall_seconds = 1.0

    result = ReversibleDDCertifier().certify(
        None, _q(0, 0, 0), _q(1, 0, 0), ReversibleDiffDrive(), Budget(),
    )
    assert result.status is CertStatus.UNKNOWN
    assert result.reason_code == "uncertified_corridor_context"


def test_zero_dynamic_budget_fails_closed():
    curve = PoseCurve((PoseSegment(
        SegmentKind.TRANSLATION, _q(0, 0, 0), _q(1, 0, 0)),))

    class Budget:
        candidate_curve = curve
        max_wall_seconds = 0.0

    result = ReversibleDDCertifier().certify(
        CorridorVerificationContext(
            shapely.box(-10, -10, 10, 10),
            empty_scene(workspace=(-10, 10, -10, 10)),
            ellipse_robot(0.1, 0.08), eps_clear=0.0, theta_min=1e-4,
        ), _q(0, 0, 0), _q(1, 0, 0),
        ReversibleDiffDrive(), Budget(),
    )
    assert result.status is CertStatus.UNKNOWN
    assert result.reason_code == "dynamic_budget_exhausted"


def test_long_translation_cannot_hide_macroscopic_lateral_motion():
    curve = PoseCurve((PoseSegment(
        SegmentKind.TRANSLATION, _q(0, 0, 0), _q(1e12, 500, 0)),))
    result = _certify(curve)
    assert result.status is CertStatus.UNKNOWN
    assert result.reason_code == "translation_not_heading_aligned"


def test_raw_polygon_is_not_a_collision_free_corridor_certificate():
    curve = PoseCurve((PoseSegment(
        SegmentKind.TRANSLATION, _q(-1, 0, 0), _q(1, 0, 0)),))

    class Budget:
        candidate_curve = curve
        max_wall_seconds = 1.0

    result = ReversibleDDCertifier().certify(
        shapely.box(-2, -2, 2, 2), curve.segments[0].q0,
        curve.segments[-1].q1, ReversibleDiffDrive(), Budget(),
    )
    assert result.status is CertStatus.UNKNOWN
    assert result.reason_code == "uncertified_corridor_context"


def test_curve_through_obstacle_cannot_receive_dynamic_certificate():
    curve = PoseCurve((PoseSegment(
        SegmentKind.TRANSLATION, _q(-1, 0, 0), _q(1, 0, 0)),))
    scene = single_obstacle(0.3, workspace=(-2, 2, -2, 2))
    context = CorridorVerificationContext(
        scene.workspace, scene, ellipse_robot(0.1, 0.08),
        eps_clear=0.0, theta_min=1e-4,
    )

    class Budget:
        candidate_curve = curve
        max_wall_seconds = 1.0

    result = ReversibleDDCertifier().certify(
        context, curve.segments[0].q0, curve.segments[-1].q1,
        ReversibleDiffDrive(), Budget(),
    )
    assert result.status is CertStatus.UNKNOWN
    assert result.reason_code == "geometric_verification_failed"
    assert "collision" in result.uncertainty_sources


def test_zero_area_corridor_is_invalid():
    curve = PoseCurve((PoseSegment(
        SegmentKind.TRANSLATION, _q(0, 0, 0), _q(1, 0, 0)),))
    scene = empty_scene(workspace=(-2, 2, -2, 2))
    context = CorridorVerificationContext(
        shapely.LineString([(0, 0), (1, 0)]), scene,
        ellipse_robot(0.1, 0.08), eps_clear=0.0, theta_min=1e-4,
    )

    class Budget:
        candidate_curve = curve
        max_wall_seconds = 1.0

    result = ReversibleDDCertifier().certify(
        context, curve.segments[0].q0, curve.segments[-1].q1,
        ReversibleDiffDrive(), Budget(),
    )
    assert result.status is CertStatus.UNKNOWN
    assert result.reason_code == "invalid_corridor"


def test_name_spoof_does_not_substitute_for_capability_model():
    curve = PoseCurve((PoseSegment(
        SegmentKind.TRANSLATION, _q(0, 0, 0), _q(-1, 0, 0)),))

    class Spoof:
        name = "reversible_differential_drive"
        allow_reverse = False
        allow_in_place_rotation = True

    scene = empty_scene(workspace=(-2, 2, -2, 2))
    context = CorridorVerificationContext(
        scene.workspace, scene, ellipse_robot(0.1, 0.08),
        eps_clear=0.0, theta_min=1e-4,
    )

    class Budget:
        candidate_curve = curve
        max_wall_seconds = 1.0

    result = ReversibleDDCertifier().certify(
        context, curve.segments[0].q0, curve.segments[-1].q1,
        Spoof(), Budget(),
    )
    assert result.status is CertStatus.UNKNOWN
    assert result.reason_code == "wrong_dynamics_model"
