"""Fail-closed regressions for large binary64 world coordinates."""

from dataclasses import replace

import numpy as np
import pytest
from shapely.geometry import Polygon, box

from gmc.geometry.c_obstacle import pose_collides
from gmc.geometry.envelopes import approximate_pair
from gmc.geometry.support import PairOracle, unit_dirs
from gmc.mobility.witness import PoseCurve, PoseSegment, SegmentKind
from gmc.spatial.bvh import candidate_pairs
from gmc.spatial.slice_compiler import build_slice
from gmc.types import (GaussianSupport2D, PairID, Pose2, RobotModel2D,
                       SceneModel2D)
from gmc.verification.continuous import (pair_margin, rotation_interval_safe,
                                         translation_safe)
from gmc.verification.path import verify_curve

from ..conftest import make_cfg


def _disc(mean, radius, primitive_id):
    return GaussianSupport2D(
        np.asarray(mean, dtype=float),
        np.eye(2) * float(radius) ** 2,
        1.0,
        primitive_id,
    )


def _collision_reproducer():
    """A locked case where the former world-frame margin was +0.015625."""
    scene = GaussianSupport2D(
        [1.0e14, -1.0e14],
        [[0.7688206441477675, 0.0], [0.0, 0.11600789096835083]],
        1.1667239341110718,
        0,
    )
    body = GaussianSupport2D(
        [-0.8579329665427731, -0.05620604131050988],
        [[0.887841680922069, 0.0], [0.0, 0.7719083320580539]],
        1.2111706589909614,
        0,
    )
    theta = 2.526700790204038
    point = np.array([100000000000001.39, -99999999999999.7])
    return scene, body, theta, point


def test_pair_local_margin_rejects_locked_large_coordinate_collision():
    scene, body, theta, point = _collision_reproducer()
    oracle = PairOracle(PairID(0, 0), scene, body)

    assert pose_collides(scene, body, point, theta)
    # The old formula ``u@t - h_world`` returned exactly +0.015625 here.
    assert pair_margin(oracle, point, theta) < 0.0


def test_independent_curve_verifier_cannot_certify_that_collision():
    scene_support, body, theta, point = _collision_reproducer()
    oracle = PairOracle(PairID(0, 0), scene_support, body)
    workspace = box(
        point[0] - 8.0, point[1] - 8.0,
        point[0] + 8.0, point[1] + 8.0,
    )
    pose = Pose2(point, theta)
    curve = PoseCurve((PoseSegment(
        SegmentKind.TRANSLATION, pose, pose,
    ),))

    report = verify_curve(
        (oracle,), workspace, curve, eps_clear=0.0, theta_min=1e-4,
    )

    assert not report.certified
    assert report.reason == "collision"
    assert report.min_clearance < 0.0


def test_sub_femtoredian_rotation_is_not_treated_as_zero():
    # A body support one trillion units from the robot origin moves 0.5 mm
    # over 5e-16 rad.  The former fixed 1e-15 shortcut sampled only theta=0
    # and incorrectly certified this interval even though its far endpoint is
    # in collision.
    scene = _disc((0.0, 0.0), 1e-6, 0)
    body = _disc((1e12, 0.0), 1e-6, 0)
    oracle = PairOracle(PairID(0, 0), scene, body)
    point = np.array([-1e12, -5e-4])

    assert pair_margin(oracle, point, 0.0) > 0.0
    assert pair_margin(oracle, point, 5e-16) < 0.0
    safe, lower = rotation_interval_safe(
        (oracle,), point, 0.0, 5e-16, theta_min=1e-18,
    )

    assert not safe
    assert lower < 0.0


def test_large_world_translation_cannot_step_over_sub_ulp_obstacle():
    # The exact affine segment crosses the pair centre at base + 0.4 ULP.
    # Binary64 world probes can only land on the surrounding grid points, so
    # the former forward-step verifier skipped the small obstacle entirely.
    base = 1e14
    ulp = float(np.spacing(base))
    scene = _disc((base, 0.0), 5e-4, 0)
    body = _disc((-0.4 * ulp, 0.0), 5e-4, 0)
    oracle = PairOracle(PairID(0, 0), scene, body)
    p0 = np.array([base - 10.0 * ulp, 0.0])
    p1 = np.array([base + 10.0 * ulp, 0.0])
    safe, lower = translation_safe((oracle,), p0, p1, 0.0)
    assert not safe
    assert lower < 0.0

    curve = PoseCurve((PoseSegment(
        SegmentKind.TRANSLATION, Pose2(p0, 0.0), Pose2(p1, 0.0),
    ),))
    workspace = box(base - 20.0 * ulp, -1.0,
                    base + 20.0 * ulp, 1.0)
    report = verify_curve(
        (oracle,), workspace, curve, eps_clear=0.0, theta_min=1e-4,
        expected_start=Pose2(p0, 0.0), expected_goal=Pose2(p1, 0.0),
    )

    assert not report.certified
    # The public verifier is fail-closed too.  Its scale-aware GEOS boundary
    # bound may reject this deliberately unrepresentable workspace before it
    # reaches the independently checked obstacle segment.
    assert report.reason in {"workspace_clearance", "collision"}


def test_pair_margin_is_equivariant_under_exact_large_translation():
    scene = GaussianSupport2D(
        [0.0, 0.0], [[0.7, 0.08], [0.08, 0.3]], 1.2, 0,
    )
    body = GaussianSupport2D(
        [0.0, 0.0], [[0.4, -0.03], [-0.03, 0.2]], 0.9, 0,
    )
    theta = 0.73
    point = np.array([0.75, -0.5])
    shift = np.array([2.0 ** 46, -(2.0 ** 46)])
    base = PairOracle(PairID(0, 0), scene, body)
    shifted = PairOracle(
        PairID(0, 0),
        GaussianSupport2D(
            scene.mean + shift, scene.covariance, scene.level, 0,
        ),
        body,
    )

    assert pair_margin(shifted, point + shift, theta) == pytest.approx(
        pair_margin(base, point, theta), abs=2e-15,
    )


def test_large_coordinate_envelope_keeps_dense_outer_and_inner_directions():
    scene = GaussianSupport2D(
        [1.0e8, -3.7e7],
        [[0.7688206441477675, 0.05], [0.05, 0.11600789096835083]],
        1.1667239341110718,
        0,
    )
    body = GaussianSupport2D(
        [-0.8579329665427731, -0.05620604131050988],
        [[0.887841680922069, 0.03], [0.03, 0.7719083320580539]],
        1.2111706589909614,
        0,
    )
    oracle = PairOracle(PairID(0, 0), scene, body)
    theta = 2.526700790204038
    cfg = make_cfg(mode="theorem", eps_pair=5e-3)

    sandwich = approximate_pair(oracle, theta, cfg.pair_approx)
    directions = unit_dirs(np.linspace(0.0, 2.0 * np.pi, 4096,
                                       endpoint=False))

    assert sandwich.outer_translation_slack > 0.0
    assert sandwich.inner_translation_slack > 0.0
    inner_vertices = np.asarray(sandwich.inner.exterior.coords[:-1])
    assert all(pose_collides(scene, body, point, theta)
               for point in inner_vertices)

    # Audit both support inequalities after cancelling the large centre in
    # long double; world-frame dot products would recreate the tested bug.
    center = oracle.center_longdouble(theta)
    outer_local = (np.asarray(sandwich.outer.exterior.coords[:-1],
                              dtype=np.longdouble) - center)
    inner_local = (np.asarray(sandwich.inner.exterior.coords[:-1],
                              dtype=np.longdouble) - center)
    u_long = np.asarray(directions, dtype=np.longdouble)
    h_true = np.asarray(
        oracle.local_support_values(theta, directions), dtype=np.longdouble,
    )
    h_outer = np.max(u_long @ outer_local.T, axis=1)
    h_inner = np.max(u_long @ inner_local.T, axis=1)
    assert np.all(h_true <= h_outer)
    assert np.all(h_inner <= h_true)
    assert float(np.max(h_outer - h_true)) <= \
        sandwich.hausdorff_upper
    assert float(np.max(h_true - h_inner)) <= \
        sandwich.hausdorff_upper


def test_compile_rejects_unrepresentable_world_precision():
    center = np.array([1.0e14, -1.0e14])
    scene = SceneModel2D(
        (_disc(center, 0.4, 0),),
        box(center[0] - 10.0, center[1] - 10.0,
            center[0] + 10.0, center[1] + 10.0),
        "large-world",
    )
    robot = RobotModel2D((_disc((0.0, 0.0), 0.2, 0),))
    cfg = make_cfg(mode="theorem", eps_pair=1e-3)

    with pytest.raises(ValueError, match="world-coordinate resolution"):
        build_slice(scene, robot, 0.0, cfg)

    # Raising the three positive spatial tolerances above one ULP makes the
    # representability contract explicit and allows compilation to proceed.
    coarse = replace(
        cfg,
        geometry=replace(cfg.geometry, workspace_precision=0.1),
        pair_approx=replace(cfg.pair_approx, eps_pair=0.1),
        query=replace(cfg.query, eps_clear=0.1),
    )
    built = build_slice(scene, robot, 0.0, coarse)
    assert built.sandwiches


def test_bvh_fail_includes_when_tiny_world_buffer_would_collapse():
    coordinate = np.array([1.0e12, 0.37e12])
    radius = 1.1e-5
    workspace = box(
        coordinate[0] - 10.0, coordinate[1] - 10.0,
        coordinate[0] + 10.0, coordinate[1] + 10.0,
    )
    scene = SceneModel2D((_disc(coordinate, radius, 3),), workspace)
    robot = RobotModel2D((_disc((0.0, 0.0), radius, 7),))

    # On affected GEOS builds workspace.buffer(radius) is EMPTY.  Candidate
    # discovery no longer constructs that buffer and must never interpret its
    # NaN distance as a proof of disjointness.
    pairs = candidate_pairs(scene, robot, workspace)
    assert [pair.pair_id for pair in pairs] == [PairID(3, 7)]


def test_bvh_candidate_set_is_large_translation_equivariant_for_slanted_ws():
    local_workspace = Polygon([
        (-5.0, -3.0), (4.0, -4.0), (6.0, 2.0), (-3.0, 5.0),
    ])
    shift = np.array([1.0e12, -0.37e12])
    shifted_workspace = Polygon(
        np.asarray(local_workspace.exterior.coords) + shift,
    )
    local_supports = (
        _disc((-4.5, -2.5), 0.08, 0),
        _disc((2.0, 1.0), 0.12, 1),
        _disc((20.0, 15.0), 0.09, 2),
    )
    shifted_supports = tuple(
        GaussianSupport2D(
            support.mean + shift, support.covariance,
            support.level, support.primitive_id,
        )
        for support in local_supports
    )
    robot = RobotModel2D((
        _disc((0.0, 0.0), 0.05, 10),
        _disc((0.2, -0.1), 0.04, 11),
    ))
    local_scene = SceneModel2D(local_supports, local_workspace)
    shifted_scene = SceneModel2D(shifted_supports, shifted_workspace)

    local_ids = [pair.pair_id for pair in candidate_pairs(
        local_scene, robot, local_workspace)]
    shifted_ids = [pair.pair_id for pair in candidate_pairs(
        shifted_scene, robot, shifted_workspace)]

    assert shifted_ids == local_ids
    assert PairID(0, 10) in shifted_ids
    assert PairID(1, 11) in shifted_ids
