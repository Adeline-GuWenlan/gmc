"""Balanced incremental union tree used by P5 fixed-slice updates."""
from __future__ import annotations

import shapely


def _empty():
    return shapely.Polygon()


def _union(left, right, *, ledger=None):
    if left.is_empty:
        return right, 0
    if right.is_empty:
        return left, 0
    if ledger is not None:
        # Charge before the GEOS call.  A rejected operation is neither
        # executed nor counted, preserving the ledger's hard-cap semantics.
        ledger.charge("union_operations")
    return shapely.union(left, right), 1


class IncrementalUnionTree:
    """Maintain a union while replacing a small subset of keyed leaves."""

    def __init__(self, keyed_geometries, *, ledger=None):
        items = list(keyed_geometries)
        self.ledger = ledger
        self.keys = tuple(key for key, _ in items)
        self._positions = {key: i for i, key in enumerate(self.keys)}
        size = 1
        while size < max(1, len(items)):
            size *= 2
        self._size = size
        self._nodes = [_empty() for _ in range(2 * size)]
        for i, (_, geometry) in enumerate(items):
            self._nodes[size + i] = geometry
        self.build_operations = self._rebuild_all()

    @property
    def geometry(self):
        return self._nodes[1]

    def clone(self):
        """Return an independent tree with shared immutable geometries.

        Shapely geometries are immutable, so copying the node array is enough
        to make subsequent leaf replacements transactional: a failed slice
        build cannot corrupt the last committed union state.
        """
        other = object.__new__(type(self))
        other.keys = self.keys
        other._positions = self._positions.copy()
        other._size = self._size
        other._nodes = self._nodes.copy()
        other.ledger = self.ledger
        other.build_operations = 0
        return other

    def _rebuild_all(self) -> int:
        operations = 0
        for node in range(self._size - 1, 0, -1):
            self._nodes[node], executed = _union(
                self._nodes[2 * node], self._nodes[2 * node + 1],
                ledger=self.ledger,
            )
            operations += executed
        return operations

    def update(self, replacements: dict) -> int:
        """Replace leaves and return the number of GEOS unions executed."""
        dirty = set()
        operations = 0
        # Stage all nodes so a hard budget failure cannot expose a partially
        # updated tree to a caller that reuses this instance directly.
        nodes = self._nodes.copy()
        for key, geometry in replacements.items():
            if key not in self._positions:
                raise KeyError(f"unknown union-tree leaf: {key!r}")
            leaf = self._size + self._positions[key]
            nodes[leaf] = geometry
            leaf //= 2
            while leaf:
                dirty.add(leaf)
                leaf //= 2
        for node in sorted(dirty, reverse=True):
            nodes[node], executed = _union(
                nodes[2 * node], nodes[2 * node + 1], ledger=self.ledger,
            )
            operations += executed
        self._nodes = nodes
        return operations
