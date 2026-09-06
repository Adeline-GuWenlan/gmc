"""Theorem evidence for interval-wide pair obstacle certificates."""

from dataclasses import replace

import numpy as np
import pytest
from shapely.geometry import Polygon

from gmc.config import PairApproxCfg
from gmc.geometry.c_obstacle import pose_collides
from gmc.geometry.envelopes import approximate_pair
from gmc.geometry.support import PairOracle, unit_dirs
from gmc.orientation.interval_certificate import (
    IntervalPairCertificate,
    _convex_polygon_covers_vertices,
    build_interval_pair_certificate,
)
from gmc.orientation.intervals import Interval
from gmc.types import CertStatus, GaussianSupport2D, PairID, rotation2


def _support(rng, primitive_id, mean_scale):
    angle = rng.uniform(0.0, np.pi)
    R = rotation2(angle)
    radii = rng.uniform(0.15, 0.8, 2)
    covariance = R @ np.diag(radii ** 2) @ R.T
    return GaussianSupport2D(
        rng.uniform(-mean_scale, mean_scale, 2),
        covariance,
        rng.uniform(0.7, 1.5),
        primitive_id,
    )


def _certified_cases(count=5):
    rng = np.random.default_rng(781)
    cfg = PairApproxCfg(
        initial_directions=16,
        max_directions=256,
        eps_pair=4e-3,
        certificate_mode="theorem",
    )
    cases = []
    for case_id in range(count):
        oracle = PairOracle(
            PairID(case_id, case_id),
            _support(rng, case_id, 2.0),
            _support(rng, case_id, 0.5),
        )
        theta = float(rng.uniform(-2.0, 2.0))
        half_width = float(rng.uniform(0.02, 0.07))
        interval = Interval(theta - half_width, theta + half_width)
        sandwich = approximate_pair(oracle, theta, cfg)
        assert sandwich.status is CertStatus.CERTIFIED
        certificate = build_interval_pair_certificate(
            sandwich, oracle, interval, 1e-8,
        )
        assert certificate.status is CertStatus.CERTIFIED
        cases.append((oracle, interval, certificate))
    return cases


def test_interval_pair_support_containment_random_theta_and_directions():
    """Dense audit of inner_common <= O(theta) <= outer_cover."""
    rng = np.random.default_rng(9104)
    for oracle, interval, certificate in _certified_cases():
        angles = rng.uniform(0.0, 2.0 * np.pi, 700)
        directions = unit_dirs(angles)
        outer_vertices = np.asarray(
            certificate.outer_cover.exterior.coords[:-1], dtype=float,
        )
        inner_vertices = np.asarray(
            certificate.inner_common.exterior.coords[:-1], dtype=float,
        )
        h_outer = np.max(directions @ outer_vertices.T, axis=1)
        h_inner = np.max(directions @ inner_vertices.T, axis=1)

        for theta in rng.uniform(interval.lo, interval.hi, 11):
            h_true = oracle.support_values(float(theta), directions)
            assert np.all(h_true <= h_outer + 2e-12)
            assert np.all(h_inner <= h_true + 2e-12)


def test_inner_common_sampled_points_collide_throughout_interval():
    """Use the independent ellipse contact oracle, not support code."""
    for oracle, interval, certificate in _certified_cases(3):
        vertices = np.asarray(
            certificate.inner_common.exterior.coords[:-1], dtype=float,
        )
        # Spread a bounded sample over the polygon and include its centroid.
        indices = np.linspace(0, len(vertices) - 1, 8, dtype=int)
        points = np.vstack([
            vertices[indices],
            np.asarray(certificate.inner_common.centroid.coords[0]),
        ])
        for theta in np.linspace(interval.lo, interval.hi, 7):
            assert all(
                pose_collides(oracle.scene, oracle.body, point, float(theta))
                for point in points
            )


def test_single_orientation_interval_reduces_to_midpoint_certificate():
    oracle, _, _ = _certified_cases(1)[0]
    theta = 0.375
    cfg = PairApproxCfg(
        initial_directions=16,
        max_directions=256,
        eps_pair=4e-3,
        certificate_mode="theorem",
    )
    sandwich = approximate_pair(oracle, theta, cfg)
    interval = Interval(theta, theta)
    certificate = build_interval_pair_certificate(
        sandwich, oracle, interval, 1e-8,
    )

    assert isinstance(certificate, IntervalPairCertificate)
    assert certificate.status is CertStatus.CERTIFIED
    assert certificate.motion_bound == 0.0
    assert certificate.pair_id == oracle.pair_id
    assert certificate.theta == theta
    assert certificate.outer is certificate.outer_cover
    assert certificate.inner is certificate.inner_common
    assert certificate.outer.covers(sandwich.outer)
    assert sandwich.inner.covers(certificate.inner)


def test_empty_midpoint_inner_remains_empty_without_poisoning_outer_proof():
    oracle, interval, _ = _certified_cases(1)[0]
    cfg = PairApproxCfg(
        initial_directions=16,
        max_directions=256,
        eps_pair=4e-3,
        certificate_mode="theorem",
    )
    sandwich = approximate_pair(oracle, interval.midpoint, cfg)
    empty_inner = replace(sandwich, inner=type(sandwich.inner)())
    certificate = build_interval_pair_certificate(
        empty_inner, oracle, interval, 1e-8,
    )

    # Empty is a valid (weak) common inner obstacle.  It makes the possible
    # free-space bound less informative but cannot create a false collision.
    assert certificate.status is CertStatus.CERTIFIED
    assert certificate.inner_common.is_empty
    assert not certificate.outer_cover.is_empty
    assert certificate.provenance["inner_common_empty"] is True


def test_unresolvable_large_coordinate_precision_fails_closed():
    scene = GaussianSupport2D(
        [1.0e14, -1.0e14],
        [[0.7, 0.05], [0.05, 0.3]],
        1.2,
        0,
    )
    body = GaussianSupport2D(
        [-0.4, 0.15],
        [[0.25, 0.03], [0.03, 0.12]],
        0.9,
        0,
    )
    oracle = PairOracle(PairID(0, 0), scene, body)
    interval = Interval(0.39, 0.41)
    cfg = PairApproxCfg(
        initial_directions=16,
        max_directions=256,
        eps_pair=0.5,
        certificate_mode="theorem",
    )
    sandwich = approximate_pair(oracle, interval.midpoint, cfg)
    assert sandwich.status is CertStatus.CERTIFIED

    # 1e-8 is far below one binary64 ULP at 1e14.  If the requested outward
    # pad cannot be represented, an empty UNKNOWN object is safer than a
    # geometry that merely looks dilated in GEOS.
    certificate = build_interval_pair_certificate(
        sandwich, oracle, interval, 1e-8,
    )
    assert certificate.status is CertStatus.UNKNOWN
    assert certificate.outer_cover.is_empty
    assert certificate.inner_common.is_empty
    assert certificate.provenance["reason"] == "interval_pair_numeric_failure"


def test_uncertified_midpoint_is_never_promoted():
    oracle, interval, _ = _certified_cases(1)[0]
    cfg = PairApproxCfg(
        initial_directions=16,
        max_directions=16,
        eps_pair=1e-12,
        certificate_mode="theorem",
    )
    sandwich = approximate_pair(oracle, interval.midpoint, cfg)
    assert sandwich.status is CertStatus.APPROX_UNCERTIFIED
    result = build_interval_pair_certificate(sandwich, oracle, interval, 1e-8)
    assert result.status is CertStatus.UNKNOWN
    assert result.outer_cover.is_empty
    assert result.inner_common.is_empty


def test_corrupted_certified_direction_schedule_is_not_hidden_as_unknown():
    oracle, interval, _ = _certified_cases(1)[0]
    cfg = PairApproxCfg(
        initial_directions=16,
        max_directions=256,
        eps_pair=4e-3,
        certificate_mode="theorem",
    )
    sandwich = approximate_pair(oracle, interval.midpoint, cfg)
    corrupted = replace(
        sandwich,
        angles=np.asarray(["not-a-direction"], dtype=object),
    )

    with pytest.raises(ValueError):
        build_interval_pair_certificate(
            corrupted, oracle, interval, 1e-8,
        )


def test_stored_direction_rows_cannot_be_substituted_behind_same_angles():
    oracle, interval, _ = _certified_cases(1)[0]
    cfg = PairApproxCfg(
        initial_directions=16,
        max_directions=256,
        eps_pair=4e-3,
        certificate_mode="theorem",
    )
    sandwich = approximate_pair(oracle, interval.midpoint, cfg)
    corrupted = replace(
        sandwich,
        directions=np.roll(sandwich.directions, 1, axis=0),
    )

    with pytest.raises(ValueError, match="do not match their angles"):
        build_interval_pair_certificate(
            corrupted, oracle, interval, 1e-8,
        )


def test_exact_convex_nesting_rejects_one_ulp_protrusion():
    outer = Polygon([(0.0, 0.0), (1.0, 0.0),
                     (1.0, 1.0), (0.0, 1.0)])
    boundary_inner = Polygon([(0.25, 0.25), (1.0, 0.25),
                              (1.0, 0.75), (0.25, 0.75)])
    protruding_inner = Polygon([
        (0.25, 0.25),
        (np.nextafter(1.0, np.inf), 0.25),
        (np.nextafter(1.0, np.inf), 0.75),
        (0.25, 0.75),
    ])

    assert _convex_polygon_covers_vertices(outer, boundary_inner)
    assert not _convex_polygon_covers_vertices(outer, protruding_inner)
