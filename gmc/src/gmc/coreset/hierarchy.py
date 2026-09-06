"""M0 deterministic Gaussian hierarchy over scene supports.

A binary k-d style tree built by median split along the widest spread axis of
the member centres.  The construction is deterministic (ties broken by
primitive id), goal independent, and robot independent, so one hierarchy is
reused across every robot and every query -- this is what makes the coreset a
compile-once asset rather than a per-query cost.

Node payload carries the features the learned coarsener will consume in a
later phase (design revision 4.2).  They are cheap aggregates of the member
supports, so computing them now costs nothing and fixes the feature contract.
"""
from dataclasses import dataclass, field

import numpy as np

from ..types import GaussianSupport2D, SceneModel2D, Vec2


@dataclass(frozen=True)
class HierarchyNode:
    """One group of scene supports.

    ``members`` are indices into ``scene.supports`` and are always sorted, so
    two runs over the same scene produce byte-identical trees.
    """
    node_id: int
    members: tuple[int, ...]
    center: Vec2                 # centroid of member means
    radius: float                # conservative bounding-disc radius about center
    depth: int
    children: tuple = ()

    @property
    def is_leaf(self) -> bool:
        return not self.children

    @property
    def size(self) -> int:
        return len(self.members)


def _bounding_disc(supports, members) -> tuple[Vec2, float]:
    """Centroid plus a radius that provably covers every member ellipse."""
    means = np.array([supports[i].mean for i in members], dtype=np.float64)
    center = means.mean(axis=0)
    radii = np.array([supports[i].bounding_radius() for i in members],
                     dtype=np.float64)
    offsets = np.linalg.norm(means - center[None, :], axis=1)
    return center, float(np.max(offsets + radii))


def build_hierarchy(scene: SceneModel2D, leaf_size: int = 1) -> HierarchyNode:
    """Build the deterministic hierarchy for ``scene``.

    ``leaf_size`` is the largest group left unsplit.  The default of 1 keeps
    the finest cut identical to the original scene, which is the fallback the
    design revision requires (4.3 property 4).
    """
    if leaf_size < 1:
        raise ValueError("leaf_size must be >= 1")
    supports = scene.supports
    if not supports:
        raise ValueError("cannot build a hierarchy over an empty scene")
    counter = [0]

    def build(members: tuple[int, ...], depth: int) -> HierarchyNode:
        node_id = counter[0]
        counter[0] += 1
        center, radius = _bounding_disc(supports, members)
        if len(members) <= leaf_size:
            return HierarchyNode(node_id, members, center, radius, depth)
        means = np.array([supports[i].mean for i in members], dtype=np.float64)
        spread = means.max(axis=0) - means.min(axis=0)
        axis = int(np.argmax(spread))
        if spread[axis] <= 0.0:
            # Coincident centres: no geometric split is meaningful, so this is
            # a leaf regardless of size.  Splitting by index would produce
            # groups no macro ellipse can separate.
            return HierarchyNode(node_id, members, center, radius, depth)
        # Sort by (coordinate, other coordinate, primitive id) so the split is
        # independent of the incoming member order.
        order = sorted(members, key=lambda i: (supports[i].mean[axis],
                                               supports[i].mean[1 - axis],
                                               supports[i].primitive_id))
        mid = len(order) // 2
        left = build(tuple(sorted(order[:mid])), depth + 1)
        right = build(tuple(sorted(order[mid:])), depth + 1)
        return HierarchyNode(node_id, members, center, radius, depth,
                             (left, right))

    return build(tuple(range(len(supports))), 0)


def iter_nodes(root: HierarchyNode):
    """Pre-order traversal."""
    stack = [root]
    while stack:
        node = stack.pop()
        yield node
        stack.extend(reversed(node.children))


def node_features(scene: SceneModel2D, node: HierarchyNode,
                  robot_scale: float = 1.0) -> dict:
    """Robot-normalised aggregate features (design revision 4.2, 8.1).

    Consumed by the learned coarsener in a later phase; recorded now so the
    hand-crafted and learned coarseners share one feature contract.
    """
    if robot_scale <= 0.0:
        raise ValueError("robot_scale must be positive")
    members = [scene.supports[i] for i in node.members]
    means = np.array([s.mean for s in members], dtype=np.float64)
    eigs = np.array([np.linalg.eigvalsh(s.covariance) * s.level ** 2
                     for s in members], dtype=np.float64)
    semi = np.sqrt(np.maximum(eigs, 0.0))          # (n, 2) semi-axes, ascending
    extent = means.max(axis=0) - means.min(axis=0)
    aniso = semi[:, 1] / np.maximum(semi[:, 0], np.finfo(float).tiny)
    return {
        "node_id": node.node_id,
        "depth": node.depth,
        "count": node.size,
        "center": node.center.tolist(),
        "radius_norm": node.radius / robot_scale,
        "extent_norm": (extent / robot_scale).tolist(),
        "semi_min_norm": float(semi[:, 0].mean() / robot_scale),
        "semi_max_norm": float(semi[:, 1].mean() / robot_scale),
        "anisotropy_mean": float(aniso.mean()),
        "anisotropy_max": float(aniso.max()),
        "spread_norm": float(np.linalg.norm(means.std(axis=0)) / robot_scale),
        "packing": float(node.size * semi.prod(axis=1).mean()
                         / max(node.radius ** 2, np.finfo(float).tiny)),
    }
