"""Topology predicates must not contain a hidden absolute area scale."""
from types import SimpleNamespace

import shapely

from gmc.spatial.arrangement import match_components


def _component(geometry):
    return SimpleNamespace(geometry=geometry)


def test_tiny_component_matches_itself_just_like_scaled_copy():
    tiny = _component(shapely.box(0.0, 0.0, 1e-5, 1e-5))
    large = _component(shapely.box(0.0, 0.0, 10.0, 10.0))
    tiny_match = match_components([tiny], [tiny])
    large_match = match_components([large], [large])
    assert tiny_match[0] == [(0, 0)]
    assert tiny_match[3]
    assert large_match[0] == tiny_match[0]
    assert large_match[3] == tiny_match[3]
