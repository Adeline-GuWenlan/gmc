"""M4 contact nerve v0 (Guide §7.4).

The nerve is a combinatorial certificate of the obstacle union and an event
candidate generator — NOT a planner. Maximal simplices (maximal subsets of
obstacles with nonempty common intersection) must be stored; truncating to
the pair graph / 2-skeleton fabricates homology (§14.4 counterexample).

v0 strategy for small active sets: pairwise overlap candidates via STRtree,
maximal cliques of the overlap graph, then recursive incremental common
intersection with memoization to recover the true maximal simplices (a clique
of the pairwise graph can still have empty common intersection — Helly in R^2
needs triples, so geometric verification is mandatory).
"""
from dataclasses import dataclass

import networkx as nx
import shapely
from shapely.strtree import STRtree


@dataclass(frozen=True)
class Nerve:
    simplices: tuple          # tuple[frozenset[int], ...] maximal, by sandwich index
    edges: tuple              # pairwise-overlap edges (i, j)
    witness_points: dict      # frozenset -> (x, y) point in the common intersection


def _common_intersection(polys, members, *, ledger=None):
    geom = polys[min(members)]
    for m in sorted(members)[1:]:
        if ledger is not None:
            ledger.charge("intersection_tests")
        geom = geom.intersection(polys[m])
        if geom.is_empty:
            return None
    return geom if not geom.is_empty else None


def build_contact_nerve(sandwiches, side: str = "outer", *, ledger=None) -> Nerve:
    polys = [getattr(s, side) for s in sandwiches]
    n = len(polys)
    if n == 0:
        return Nerve(simplices=(), edges=(), witness_points={})
    if ledger is not None:
        ledger.charge("spatial_index_builds")
    tree = STRtree(polys)
    if ledger is not None:
        ledger.charge("spatial_index_queries")
    # Query only bounding-box candidates.  Supplying a GEOS predicate here
    # would hide an unknown number of intersection calls inside STRtree and
    # make the work ledger unauditable.  Test each unique candidate explicitly
    # and charge before the predicate call.
    a, b = tree.query(polys)
    candidates = sorted({(int(i), int(j))
                         for i, j in zip(a, b) if i < j})
    edges = []
    for i, j in candidates:
        if ledger is not None:
            ledger.charge("intersection_tests")
        if polys[i].intersects(polys[j]):
            edges.append((i, j))
    g = nx.Graph()
    g.add_nodes_from(range(n))
    g.add_edges_from(edges)

    maximal: set = set()
    memo: dict = {}

    def has_common(members: frozenset) -> bool:
        if members not in memo:
            memo[members] = _common_intersection(
                polys, members, ledger=ledger) is not None
        return memo[members]

    def descend(members: frozenset):
        if has_common(members):
            if not any(members < m for m in maximal):
                for m in [m for m in maximal if m < members]:
                    maximal.discard(m)
                maximal.add(members)
            return
        if len(members) <= 2:
            return
        for x in members:
            descend(members - {x})

    for clique in nx.find_cliques(g):
        descend(frozenset(clique))

    witness = {}
    for m in maximal:
        geom = _common_intersection(polys, m, ledger=ledger)
        rep = geom.representative_point()
        witness[m] = (float(rep.x), float(rep.y))
    return Nerve(simplices=tuple(sorted(maximal, key=lambda s: (-len(s), sorted(s)))),
                 edges=tuple(edges), witness_points=witness)
