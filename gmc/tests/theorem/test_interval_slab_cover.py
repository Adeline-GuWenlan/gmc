"""Theorem-mode orientation intervals form a fail-closed global cover."""
import numpy as np
import pytest

from gmc.io.robot_io import ellipse_robot
from gmc.orientation.intervals import TWO_PI
import gmc.orientation.slab_builder as slab_builder
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import single_door, single_obstacle
from gmc.types import CertStatus, GaussianSupport2D, RobotModel2D

from ..conftest import make_cfg


def _small_decomposition(mode: str):
    cfg = make_cfg(
        mode=mode, eps_pair=2e-2, theta_min=2e-2,
        initial_intervals=4,
    )
    scene = single_obstacle(r=0.35)
    robot = ellipse_robot(0.22, 0.12)
    oracles = candidate_pairs(scene, robot, scene.workspace)
    return slab_builder.build_slabs(scene, robot, cfg, oracles)


def _pre_rotated_hidden_narrow_gate(mode: str):
    """A roughly 16-degree gate between the eight initial orientations."""
    cfg = make_cfg(
        mode=mode, eps_pair=2e-2, theta_min=2e-3,
        initial_intervals=8,
    )
    # A thin wall retains the analytic opening while keeping this theorem test
    # substantially smaller than the presentation/demo wall.
    scene = single_door(
        0.42, wall_t=0.18, workspace=(-2.0, 2.0, -1.2, 1.2),
    )
    base = ellipse_robot(0.5, 0.2).supports[0]
    alpha = -np.pi / 16.0
    rotation = np.array([
        [np.cos(alpha), -np.sin(alpha)],
        [np.sin(alpha), np.cos(alpha)],
    ])
    body = GaussianSupport2D(
        mean=base.mean,
        covariance=rotation @ base.covariance @ rotation.T,
        level=base.level,
        primitive_id=base.primitive_id,
    )
    robot = RobotModel2D((body,), name="pre_rotated_ellipse")
    oracles = candidate_pairs(scene, robot, scene.workspace)
    return slab_builder.build_slabs(scene, robot, cfg, oracles)


def test_theorem_interval_projections_tile_the_full_circle():
    dec = _small_decomposition("theorem")
    ordered = sorted(dec.slabs, key=lambda slab: slab.interval.lo)

    assert ordered[0].interval.lo == 0.0
    assert ordered[-1].interval.hi == TWO_PI
    assert all(left.interval.hi == right.interval.lo
               for left, right in zip(ordered, ordered[1:]))
    assert dec.global_possible_cover_status is CertStatus.CERTIFIED
    assert dec.global_possible_cover_provenance["full_circle_tiling"]
    assert (dec.global_possible_cover_provenance[
                "certified_interval_cover_count"] == len(ordered))
    assert all(slab.cover_slice is not None for slab in ordered)
    assert all(slab.cover_slice.status is CertStatus.CERTIFIED
               for slab in ordered)
    assert all(slab.cover_provenance["evidence_scope"]
               == "entire_orientation_interval" for slab in ordered)
    assert all(certificate.interval == slab.interval
               for slab in ordered
               for certificate in slab.cover_slice.sandwiches)


def test_hidden_narrow_gate_can_have_cover_without_no_event_certificate():
    dec = _pre_rotated_hidden_narrow_gate("theorem")

    # The interval projection is sound whether or not an event lies between
    # the left/mid/right samples.  Event-free certification remains separate.
    assert dec.global_possible_cover_status is CertStatus.CERTIFIED
    assert all(slab.cover_slice is not None for slab in dec.slabs)
    assert not any(slab.predicates.certifies_no_event for slab in dec.slabs)
    assert (dec.global_possible_cover_provenance["event_free_interval_count"]
            == 0)


def test_prototype_mode_keeps_legacy_unknown_cover_status():
    dec = _small_decomposition("prototype")

    assert dec.global_possible_cover_status is CertStatus.UNKNOWN
    assert all(slab.cover_slice is None for slab in dec.slabs)
    assert all(slab.cover_provenance["reason"]
               == "interval_cover_requires_theorem_mode"
               for slab in dec.slabs)


def test_one_failed_interval_pair_certificate_prevents_global_certification(
        monkeypatch):
    real_builder = slab_builder.build_interval_pair_certificate
    failed = False

    def fail_once(*args, **kwargs):
        nonlocal failed
        if not failed:
            failed = True
            raise FloatingPointError("injected interval certificate failure")
        return real_builder(*args, **kwargs)

    monkeypatch.setattr(
        slab_builder, "build_interval_pair_certificate", fail_once)
    dec = _small_decomposition("theorem")

    assert failed
    assert dec.global_possible_cover_status is CertStatus.UNKNOWN
    failed_slabs = [
        slab for slab in dec.slabs
        if slab.cover_provenance["reason"]
        == "interval_pair_certificate_failed"
    ]
    assert len(failed_slabs) == 1
    assert failed_slabs[0].cover_slice is None
    assert (dec.global_possible_cover_provenance[
                "certified_interval_cover_count"] == len(dec.slabs) - 1)


def test_programming_error_is_not_silently_converted_to_unknown(monkeypatch):
    def broken_builder(*args, **kwargs):
        raise TypeError("injected API contract violation")

    monkeypatch.setattr(
        slab_builder, "build_interval_pair_certificate", broken_builder)
    with pytest.raises(TypeError, match="API contract violation"):
        _small_decomposition("theorem")
