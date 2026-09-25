"""Query-time polyline post-processing: corner merging (certified)."""
import numpy as np
import pytest

from gmc.aerial3d.query import merge_corners


def _arc_path(n=5, radius=.05):
    # straight in along +x, wrap a quarter turn around (0,0) of the given radius, leave along +y
    t = np.linspace(-np.pi / 2, 0, n)
    arc = np.c_[radius * np.cos(t), radius * np.sin(t), np.zeros(n)]
    return np.vstack([[-2., -radius, 0.], arc, [radius, 2., 0.]])


def test_corner_wrap_is_merged_into_the_tangent_intersection():
    P = _arc_path()
    Q, checks = merge_corners(P, lambda a, b: True, max_increase_m=.05)
    assert len(Q) == 3
    assert np.allclose(Q[1], [.05, -.05, 0.], atol=1e-9)      # the two end tangents meet here
    L = lambda X: np.sum(np.linalg.norm(np.diff(X, axis=0), axis=1))
    assert L(Q) - L(P) <= .05 and L(Q) >= L(P) - 1e-12


def test_merge_is_refused_without_certification_or_beyond_the_length_budget():
    P = _arc_path()
    Q, _ = merge_corners(P, lambda a, b: False, max_increase_m=.05)
    assert np.array_equal(Q, P)
    Q, _ = merge_corners(P, lambda a, b: True, max_increase_m=1e-6)
    assert len(Q) > 3
