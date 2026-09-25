"""Leaf adjacency from exact integer box coordinates, and connected components.

Touch adjacency (closed boxes intersect: face, edge or vertex contact) is used for
the possible-side graph: the union of the given closed leaves has exactly the
components of this graph.  Face adjacency (a shared face of positive area) is used
for SAFE-side portals.
"""
from __future__ import annotations

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


def _doubled_centres(tree, leaves):
    return 2 * tree.lo[leaves] + tree.size[leaves][:, None]


def touch_pairs(tree, leaves) -> np.ndarray:
    """All (a, b), a < b, of the given leaves whose closed boxes intersect."""
    leaves = np.asarray(leaves, dtype=np.int64)
    if len(leaves) < 2:
        return np.empty((0, 2), dtype=np.int64)
    c2 = _doubled_centres(tree, leaves).astype(float)  # exact small integers
    sizes = tree.size[leaves]
    groups = {int(s): np.flatnonzero(sizes == s) for s in np.unique(sizes)}
    kd = {s: cKDTree(c2[g]) for s, g in groups.items()}
    out = []
    keys = sorted(groups)
    for ia, sa in enumerate(keys):
        for sb in keys[ia:]:
            reach = sa + sb
            if sa == sb:
                pairs = kd[sa].query_pairs(reach + .5, p=np.inf, output_type="ndarray")
                a, b = groups[sa][pairs[:, 0]], groups[sa][pairs[:, 1]]
            else:
                m = kd[sa].sparse_distance_matrix(kd[sb], reach + .5, p=np.inf, output_type="ndarray")
                a, b = groups[sa][m["i"]], groups[sb][m["j"]]
            if len(a):
                delta = np.abs(c2[a] - c2[b])
                ok = np.all(delta <= reach, axis=1)
                out.append(np.stack([leaves[a[ok]], leaves[b[ok]]], axis=1))
    if not out:
        return np.empty((0, 2), dtype=np.int64)
    pairs = np.concatenate(out)
    return np.sort(pairs, axis=1)


def face_pairs(tree, pairs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Filter touch pairs to positive-area face contacts; returns (pairs, axis)."""
    if not len(pairs):
        return pairs, np.empty(0, dtype=np.int64)
    a, b = pairs[:, 0], pairs[:, 1]
    delta = np.abs(_doubled_centres(tree, a) - _doubled_centres(tree, b))
    reach = (tree.size[a] + tree.size[b])[:, None]
    on = delta == reach
    strict = delta < reach
    face = (on.sum(axis=1) == 1) & (strict.sum(axis=1) == 2)
    return pairs[face], np.argmax(on[face], axis=1)


def shared_face(tree, a: int, b: int, axis: int):
    """Closed shared face rectangle of two face-adjacent leaves, as (lower, upper) in plan frame."""
    lo = np.maximum(tree.lo[a], tree.lo[b])
    hi = np.minimum(tree.lo[a] + tree.size[a], tree.lo[b] + tree.size[b])
    lower = tree.origin + lo * tree.unit
    upper = tree.origin + hi * tree.unit  # unit is the per-axis leaf edge
    return lower, upper


def components(n: int, edges: np.ndarray) -> np.ndarray:
    """Component label per node 0..n-1 for an undirected edge list over those nodes."""
    if n == 0:
        return np.empty(0, dtype=np.int64)
    if len(edges):
        m = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(n, n))
    else:
        m = coo_matrix((n, n))
    _, labels = connected_components(m, directed=False)
    return labels.astype(np.int64)


def possible_components(tree, leaves, pairs) -> dict[int, int]:
    """Leaf -> component id over the given leaves and touch pairs among them."""
    leaves = np.sort(np.asarray(leaves, dtype=np.int64))
    pairs = np.asarray(pairs, dtype=np.int64).reshape(-1, 2)
    e = np.searchsorted(leaves, pairs)
    labels = components(len(leaves), e)
    return {int(l): int(labels[i]) for i, l in enumerate(leaves)}
