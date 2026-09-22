"""Immutable 3D support-AABB BVH; no nearest-K truncation or xy projection."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterator

import numpy as np

from .geometry import numerical_slack


@dataclass
class _Node:
    lower: np.ndarray
    upper: np.ndarray
    left: int = -1
    right: int = -1
    indices: np.ndarray | None = None


class SupportIndex:
    """Median-split BVH of complete support boxes, O(N) memory.

    Oversized supports remain in enclosing nodes and cannot be missed. Query
    work is O(log N + candidates) in favourable spatial distributions, O(N)
    in the worst case. Closed AABB overlap includes tangent and distant-centred
    large splats. Candidate iteration bounds temporary memory and checks budget.
    """

    def __init__(self, means, covariances, level, *, leaf_size=32, check=None):
        if not isinstance(leaf_size, int) or leaf_size < 1:
            raise ValueError("leaf_size must be a positive integer")
        half = level * np.sqrt(np.diagonal(covariances, axis1=1, axis2=2))
        self.slack_m = numerical_slack(means, half)
        self.lower = means - half - self.slack_m
        self.upper = means + half + self.slack_m
        if not np.isfinite(self.lower).all() or not np.isfinite(self.upper).all():
            raise ValueError("nonfinite Gaussian support bounds")
        self.nodes: list[_Node] = []

        def build(indices):
            if check:
                check()
            lower, upper = self.lower[indices].min(0), self.upper[indices].max(0)
            node_id = len(self.nodes)
            node = _Node(lower, upper)
            self.nodes.append(node)
            if len(indices) <= leaf_size:
                node.indices = indices
            else:
                axis = int(np.argmax(upper - lower))
                centres = means[indices, axis]
                mid = len(indices) // 2
                order = np.argpartition(centres, mid)
                node.left = build(indices[order[:mid]])
                node.right = build(indices[order[mid:]])
            return node_id

        if len(means):
            build(np.arange(len(means)))
        self.lower.flags.writeable = self.upper.flags.writeable = False

    def query(self, lower, upper, *, check: Callable[[], None] | None = None,
              stats: dict | None = None) -> Iterator[np.ndarray]:
        stack = [0] if self.nodes else []
        while stack:
            if check:
                check()
            node = self.nodes[stack.pop()]
            if stats is not None:
                stats["bvh_nodes_visited"] = stats.get("bvh_nodes_visited", 0) + 1
            if np.any(node.upper < lower) or np.any(node.lower > upper):
                continue
            if node.indices is None:
                stack.extend((node.right, node.left))
            else:
                ids = node.indices
                keep = np.all((self.upper[ids] >= lower) & (self.lower[ids] <= upper), axis=1)
                if np.any(keep):
                    yield ids[keep]
