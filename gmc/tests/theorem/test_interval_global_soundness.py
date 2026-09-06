"""Independent global soundness checks for theorem interval projections.

These tests deliberately do not derive truth from interval support values.
Obstacle membership comes from the independent ellipse contact oracle, while
positive-clearance paths are certified again by the whole-curve verifier.
"""
from dataclasses import replace

import networkx as nx
import numpy as np
import pytest
import shapely
from shapely.geometry import box

from gmc.geometry.c_obstacle import scene_pose_collides
from gmc.geometry.support import unit_dirs
from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import compile_mobility, node_id
from gmc.mobility.lineage import component_slice
from gmc.mobility.witness import (PoseCurve, PoseSegment, SegmentKind)
from gmc.orientation.intervals import TWO_PI
from gmc.orientation.slab_builder import adjacent_slab_pairs, build_slabs
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import single_obstacle
from gmc.types import (CertStatus, GaussianSupport2D, Pose2, RobotModel2D,
                       SceneModel2D)
from gmc.verification.path import verify_curve

from ..conftest import make_cfg


@pytest.fixture(scope="module")
def interval_world():
    """A small rotating, anisotropic case shared by the property checks."""
    scene = single_obstacle(
        r=0.4, workspace=(-2.0, 2.0, -2.0, 2.0),
    )
    robot = ellipse_robot(0.3, 0.13)
    cfg = make_cfg(
        mode="theorem", eps_pair=2e-2, theta_min=2e-2,
        initial_intervals=4,
    )
    oracles = candidate_pairs(scene, robot, scene.workspace)
    decomposition = build_slabs(scene, robot, cfg, oracles)
    mobility = compile_mobility(
        scene, robot, cfg, oracles, decomposition,
    )
    assert decomposition.global_possible_cover_status is CertStatus.CERTIFIED
    return scene, robot, cfg, oracles, decomposition, mobility


def _slab_at(decomposition, theta):
    wrapped = float(theta) % TWO_PI
    ordered = sorted(decomposition.slabs, key=lambda slab: slab.interval.lo)
    for slab in ordered[:-1]:
        if slab.interval.lo <= wrapped < slab.interval.hi:
            return slab
    last = ordered[-1]
    assert last.interval.lo <= wrapped < last.interval.hi
    return last


def _component_containing(slab, xy, side="possible"):
    current = component_slice(slab)
    components = current.D_possible if side == "possible" else current.D_safe
    point = shapely.Point(tuple(np.asarray(xy, dtype=float)))
    return next(
        (index for index, component in enumerate(components)
         if component.geometry.covers(point)),
        None,
    )


def test_true_free_random_poses_are_in_corresponding_possible_projection(
        interval_world):
    """M_true is not lost by any certified interval-wide D_possible."""
    scene, robot, _, _, decomposition, _ = interval_world
    rng = np.random.default_rng(20260902)
    checked = 0

    for _ in range(400):
        theta = float(rng.uniform(0.0, TWO_PI))
        xy = rng.uniform(-1.05, 1.05, size=2)
        if scene_pose_collides(scene, robot, xy, theta):
            continue

        slab = _slab_at(decomposition, theta)
        component = _component_containing(slab, xy, "possible")
        assert component is not None, (
            "certified interval D_possible lost an independently free pose: "
            f"seed=20260902 theta={theta!r} xy={xy.tolist()} "
            f"slab={slab.slab_id} interval="
            f"({slab.interval.lo!r}, {slab.interval.hi!r})"
        )
        checked += 1
        if checked == 80:
            break

    assert checked == 80


def test_interval_safe_points_are_free_at_many_orientations(interval_world):
    """Every sampled D_safe point remains truly free throughout its slab."""
    scene, robot, _, _, decomposition, _ = interval_world
    checked = 0

    for slab in decomposition.slabs:
        assert slab.cover_slice is not None
        for component in slab.cover_slice.D_safe:
            xy = np.asarray(component.representative, dtype=float)
            assert scene.workspace.covers(shapely.Point(tuple(xy)))
            for theta in np.linspace(
                    slab.interval.lo, slab.interval.hi, 9):
                assert not scene_pose_collides(
                    scene, robot, xy, float(theta),
                ), (
                    "certified interval D_safe contains a colliding pose: "
                    f"xy={xy.tolist()} theta={float(theta)!r} "
                    f"slab={slab.slab_id}"
                )
                checked += 1

    assert checked >= 9 * len(decomposition.slabs)


def test_verified_positive_clearance_crossings_remain_connected_in_possible(
        interval_world):
    """A true path crossing each adjacent slab boundary survives M_possible."""
    scene, _, cfg, oracles, decomposition, mobility = interval_world
    xy = np.array([1.4, 0.0])

    for left, right in adjacent_slab_pairs(
            decomposition.slabs, periodic=True):
        theta0 = float(left.interval.midpoint)
        theta1 = float(right.interval.midpoint)
        if theta1 < theta0:
            theta1 += TWO_PI
        start = Pose2(xy, theta0)
        goal = Pose2(xy, theta1)
        curve = PoseCurve((PoseSegment(
            SegmentKind.ROTATION, start, goal,
        ),))
        report = verify_curve(
            oracles, scene.workspace, curve,
            eps_clear=0.05, theta_min=cfg.orientation.theta_min,
            expected_start=start, expected_goal=goal,
        )
        assert report.certified, (
            f"test path itself was not certified: {report.reason} "
            f"for slabs {left.slab_id}->{right.slab_id}"
        )
        assert report.min_clearance > 0.05

        left_component = _component_containing(left, xy, "possible")
        right_component = _component_containing(right, xy, "possible")
        assert left_component is not None and right_component is not None
        left_node = node_id(left.slab_id, left_component, "possible")
        right_node = node_id(right.slab_id, right_component, "possible")
        assert nx.has_path(mobility.M_possible, left_node, right_node)
        assert mobility.M_possible.has_edge(left_node, right_node), (
            "verified adjacent-slab crossing is missing from M_possible: "
            f"{left_node}->{right_node}"
        )


def test_zero_two_pi_endpoint_and_wrap_lineage_are_periodic(interval_world):
    """The exact 0/2pi seam is covered on both sides and glued in the graph."""
    scene, robot, _, _, decomposition, mobility = interval_world
    ordered = sorted(decomposition.slabs, key=lambda slab: slab.interval.lo)
    first, last = ordered[0], ordered[-1]
    xy = np.array([1.4, 0.0])

    assert first.interval.lo == 0.0
    assert last.interval.hi == TWO_PI
    assert not scene_pose_collides(scene, robot, xy, 0.0)
    assert not scene_pose_collides(scene, robot, xy, TWO_PI)

    first_component = _component_containing(first, xy, "possible")
    last_component = _component_containing(last, xy, "possible")
    assert first_component is not None
    assert last_component is not None
    first_node = node_id(first.slab_id, first_component, "possible")
    last_node = node_id(last.slab_id, last_component, "possible")
    assert mobility.M_possible.has_edge(last_node, first_node)


def _disc(mean, radius, primitive_id):
    return GaussianSupport2D(
        np.asarray(mean, dtype=float),
        np.eye(2) * float(radius) ** 2,
        1.0,
        primitive_id,
    )


def test_large_coordinate_cover_is_certified_sound_or_explicitly_unresolved():
    """Large-coordinate status may vary, but every claim must be auditable."""
    coordinate = 1.0e12
    center = np.array([coordinate, -coordinate])
    scene = SceneModel2D(
        (_disc(center, 0.4, 0),),
        box(center[0] - 10.0, center[1] - 10.0,
            center[0] + 10.0, center[1] + 10.0),
        "large-coordinate-interval-cover",
    )
    robot = RobotModel2D((_disc((0.0, 0.0), 0.2, 0),))
    cfg = make_cfg(
        mode="theorem", eps_pair=1e-3, theta_min=0.2,
        initial_intervals=1,
    )
    cfg = replace(
        cfg,
        geometry=replace(cfg.geometry, workspace_precision=1e-3),
        query=replace(cfg.query, eps_clear=1e-3),
    )
    # This clears the M0 representability gate.  A platform with true extended
    # precision may certify the downstream polygon construction, while one
    # whose long double aliases binary64 may conservatively abstain.
    assert abs(np.spacing(coordinate)) < 1e-3

    oracles = candidate_pairs(scene, robot, scene.workspace)
    decomposition = build_slabs(scene, robot, cfg, oracles)

    assert decomposition.global_possible_cover_status in {
        CertStatus.CERTIFIED, CertStatus.UNKNOWN,
    }
    provenance = decomposition.global_possible_cover_provenance
    assert provenance["full_circle_tiling"]

    if decomposition.global_possible_cover_status is CertStatus.UNKNOWN:
        assert provenance["reason"] == \
            "incomplete_orientation_interval_pair_cover"
        assert (provenance["certified_interval_cover_count"]
                < provenance["slab_count"])
        assert provenance["uncertified_slab_ids"]
        unresolved = [
            slab for slab in decomposition.slabs
            if slab.cover_slice is None
        ]
        assert unresolved
        for slab in unresolved:
            detail = slab.cover_provenance
            assert detail["status"] == CertStatus.UNKNOWN.name
            assert isinstance(detail.get("reason"), str)
            assert detail["reason"] not in {"", "interval_cover_not_built"}
            assert isinstance(detail.get("evidence_scope"), str)
            assert detail["evidence_scope"] not in {"", "none"}
        return

    assert provenance["reason"] == \
        "all_orientation_intervals_have_certified_pair_covers"
    assert (provenance["certified_interval_cover_count"]
            == provenance["slab_count"])
    assert not provenance["uncertified_slab_ids"]

    # Replay every pair certificate in directed support space.  This avoids
    # world-frame cancellation and does not admit a tolerance or buffer that
    # could turn a failed containment proof into a pass.
    directions = unit_dirs(np.linspace(
        0.0, TWO_PI, 1024, endpoint=False,
    ))
    u_long = np.asarray(directions, dtype=np.longdouble)
    oracle_by_id = {oracle.pair_id: oracle for oracle in oracles}
    for slab in decomposition.slabs:
        assert slab.cover_slice is not None
        assert slab.cover_slice.status is CertStatus.CERTIFIED
        assert slab.cover_provenance["status"] == CertStatus.CERTIFIED.name
        for certificate in slab.cover_slice.sandwiches:
            assert certificate.status is CertStatus.CERTIFIED
            checks = certificate.provenance.get("checks")
            assert checks and all(value is True for value in checks.values())
            oracle = oracle_by_id[certificate.pair_id]
            outer_world = np.asarray(
                certificate.outer.exterior.coords[:-1],
                dtype=np.longdouble,
            )
            inner_world = None
            if not certificate.inner.is_empty:
                inner_world = np.asarray(
                    certificate.inner.exterior.coords[:-1],
                    dtype=np.longdouble,
                )
            for theta in np.linspace(
                    slab.interval.lo, slab.interval.hi, 9):
                center_long = oracle.center_longdouble(float(theta))
                h_outer = np.max(
                    u_long @ (outer_world - center_long).T,
                    axis=1,
                )
                h_true = np.asarray(
                    oracle.local_support_values(float(theta), directions),
                    dtype=np.longdouble,
                )
                assert np.all(h_true <= h_outer)
                if inner_world is not None:
                    h_inner = np.max(
                        u_long @ (inner_world - center_long).T,
                        axis=1,
                    )
                    assert np.all(h_inner <= h_true)
