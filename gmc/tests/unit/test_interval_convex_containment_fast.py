"""Differential tests for the strict-convex interval containment fast path."""

import numpy as np
from shapely.geometry import MultiPoint, Polygon
from shapely.geometry.polygon import orient

from gmc.orientation.interval_certificate import (
    _convex_polygon_covers_vertices,
    _filtered_orientation_signs,
    _finite_polygon,
)


def _quadratic_reference(outer: Polygon, inner: Polygon) -> bool:
    """Historical O(n*m) exact-sign implementation used as an oracle."""
    if inner.is_empty:
        return True
    if not _finite_polygon(outer) or not _finite_polygon(inner):
        return False
    outer_vertices = np.asarray(
        orient(outer, sign=1.0).exterior.coords[:-1], dtype=float,
    )
    inner_vertices = np.asarray(inner.exterior.coords[:-1], dtype=float)
    if len(outer_vertices) < 3 or len(inner_vertices) < 3:
        return False
    previous = np.roll(outer_vertices, 1, axis=0)
    following = np.roll(outer_vertices, -1, axis=0)
    for left, vertex, right in zip(previous, outer_vertices, following):
        if _filtered_orientation_signs(
                left, vertex, right[None, :])[0] < 0:
            return False
    for start, end in zip(outer_vertices, following):
        if np.any(_filtered_orientation_signs(
                start, end, inner_vertices) < 0):
            return False
    return True


def _reverse(poly: Polygon) -> Polygon:
    return Polygon(list(poly.exterior.coords)[::-1])


def test_random_convex_polygons_match_quadratic_reference():
    rng = np.random.default_rng(20260903)
    for _ in range(160):
        angles = np.sort(rng.uniform(0.0, 2.0 * np.pi, size=48))
        radii = rng.uniform(0.7, 2.0, size=len(angles))
        points = np.column_stack((radii * np.cos(angles),
                                  radii * np.sin(angles)))
        outer = MultiPoint(points).convex_hull
        assert isinstance(outer, Polygon)

        center = np.asarray(outer.centroid.coords[0], dtype=float)
        vertices = np.asarray(outer.exterior.coords[:-1], dtype=float)
        scale = rng.uniform(0.05, 0.95)
        inner = MultiPoint(
            center + scale * (vertices - center),
        ).convex_hull
        assert isinstance(inner, Polygon)

        candidates = (inner, _reverse(inner),
                      Polygon(np.asarray(inner.exterior.coords[:-1])
                              + rng.uniform(2.5, 5.0, size=2)))
        for candidate in candidates:
            expected = _quadratic_reference(outer, candidate)
            assert _convex_polygon_covers_vertices(outer, candidate) is expected
            assert _convex_polygon_covers_vertices(
                _reverse(outer), candidate,
            ) is expected


def test_collinear_outer_vertices_retain_quadratic_semantics():
    outer = Polygon([
        (0.0, 0.0), (1.0, 0.0), (2.0, 0.0),
        (2.0, 2.0), (0.0, 2.0),
    ])
    inside = Polygon([(0.25, 0.25), (1.75, 0.25), (1.0, 1.5)])
    outside = Polygon([(0.25, 0.25), (2.25, 0.25), (1.0, 1.5)])
    for candidate in (inside, outside):
        assert _convex_polygon_covers_vertices(
            outer, candidate,
        ) is _quadratic_reference(outer, candidate)


def test_nonconvex_outer_fails_closed():
    outer = Polygon([
        (0.0, 0.0), (2.0, 0.0), (1.0, 1.0),
        (2.0, 2.0), (0.0, 2.0),
    ])
    inner = Polygon([(0.1, 0.9), (0.3, 0.9), (0.3, 1.1), (0.1, 1.1)])
    assert _quadratic_reference(outer, inner) is False
    assert _convex_polygon_covers_vertices(outer, inner) is False


def test_boundary_and_one_ulp_protrusion_match_at_large_coordinates():
    base = 1.0e14
    outer = Polygon([
        (base, base), (base + 4.0, base),
        (base + 4.0, base + 4.0), (base, base + 4.0),
    ])
    boundary = Polygon([
        (base + 1.0, base + 1.0), (base + 4.0, base + 1.0),
        (base + 4.0, base + 3.0), (base + 1.0, base + 3.0),
    ])
    x_out = np.nextafter(base + 4.0, np.inf)
    protruding = Polygon([
        (base + 1.0, base + 1.0), (x_out, base + 1.0),
        (x_out, base + 3.0), (base + 1.0, base + 3.0),
    ])
    assert _quadratic_reference(outer, boundary) is True
    assert _convex_polygon_covers_vertices(outer, boundary) is True
    assert _quadratic_reference(outer, protruding) is False
    assert _convex_polygon_covers_vertices(outer, protruding) is False


def test_empty_inner_keeps_vacuous_containment_contract():
    outer = Polygon([(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)])
    assert _convex_polygon_covers_vertices(outer, Polygon()) is True
