"""Precision reduction must preserve inner/outer certificate direction."""
from dataclasses import replace
from types import SimpleNamespace

import pytest
import shapely
from shapely.geometry import MultiPolygon, Polygon

from gmc.geometry.predicates import set_precision_inner, set_precision_outer
from gmc.geometry.predicates import polygon_components
from gmc.spatial.slice_compiler import assemble_slice
from gmc.types import CertStatus, PairID, SceneModel2D


def test_outer_precision_cannot_shrink_a_collision_boundary():
    exact_outer = shapely.box(-0.2, -0.2, 0.200000004, 0.2)
    collision = shapely.Point(0.200000002, 0.0)
    assert exact_outer.covers(collision)
    reduced = set_precision_outer(exact_outer, 1e-8)
    assert reduced.covers(collision)
    assert shapely.get_precision(reduced) == 1e-8


def test_inner_precision_cannot_expand_a_free_boundary():
    exact_inner = shapely.box(-0.2, -0.2, 0.199999996, 0.2)
    free = shapely.Point(0.199999998, 0.0)
    assert not exact_inner.covers(free)
    reduced = set_precision_inner(exact_inner, 1e-8)
    assert not reduced.covers(free)
    assert shapely.get_precision(reduced) == 1e-8


def test_slice_union_keeps_direction_after_fixed_precision_intersections(cfg):
    """Guard the second directional pass before global fixed-grid overlays.

    Even when every input vertex lies on the grid, a fixed-precision union can
    snap newly created edge intersections inward or outward.  The slice
    compiler must therefore pad each outer leaf and erode each inner leaf
    before assembling the two global unions.
    """
    outer = (
        Polygon(((-3.0, 1.6), (1.0, 1.5), (0.0, 1.0))),
        Polygon(((2.3, 3.0), (1.2, 1.3), (-2.0, -0.8))),
    )
    inner = (
        Polygon(((0.3, 1.3), (2.2, 0.4), (-1.4, -2.0))),
        Polygon(((2.9, 2.5), (2.0, -1.5), (-0.7, -2.5))),
    )
    sandwiches = tuple(
        SimpleNamespace(
            pair_id=PairID(index, 0),
            outer=outer[index],
            inner=inner[index],
            status=CertStatus.CERTIFIED,
        )
        for index in range(2)
    )
    scene = SceneModel2D((), shapely.box(-5.0, -5.0, 5.0, 5.0),
                         "fixed_precision_union_direction")
    coarse = replace(
        cfg,
        geometry=replace(cfg.geometry, workspace_precision=0.1),
    )

    compiled = assemble_slice(scene, 0.0, coarse, sandwiches)
    exact_outer_union = shapely.union_all(outer)
    exact_inner_union = shapely.union_all(inner)

    assert exact_outer_union.difference(compiled.C_plus).is_empty
    assert compiled.C_minus.difference(exact_inner_union).is_empty


def test_possible_side_never_drops_a_positive_area_component(cfg):
    tiny = shapely.box(0.0, 0.0, 1e-8, 1e-8)
    # The safe-side sliver threshold is intentionally much larger than this
    # component; possible/free topology must nevertheless retain it.
    scene = SceneModel2D((), tiny, "tiny_free_component")
    compiled = assemble_slice(scene, 0.0, cfg, ())
    assert len(compiled.D_safe) == 0
    assert len(compiled.D_possible) == 1


def test_equal_area_component_order_is_canonical():
    left = shapely.box(-2.0, 0.0, -1.0, 1.0)
    right = shapely.box(1.0, 0.0, 2.0, 1.0)
    forward = polygon_components(MultiPolygon([right, left]), 0.0)
    reverse = polygon_components(MultiPolygon([left, right]), 0.0)
    assert [p.wkb_hex for p in forward] == [p.wkb_hex for p in reverse]
    assert forward[0].bounds == left.bounds


def test_component_extraction_rejects_unrepaired_invalid_ring():
    bow_tie = Polygon([
        (0.0, 0.0), (1.0, 1.0), (1.0, 0.0), (0.0, 1.0), (0.0, 0.0),
    ])
    assert not bow_tie.is_valid
    with pytest.raises(FloatingPointError, match="valid Boolean result"):
        polygon_components(bow_tie, 0.0)
