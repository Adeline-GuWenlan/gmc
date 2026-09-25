"""Support-plane convex free cells seeded by SAFE leaves, and certified portals."""
import numpy as np
import pytest

from gmc.aerial3d.cells import CellComplex, CellConfig, grow_cell
from gmc.aerial3d.envelopes import EnvelopeTable
from gmc.aerial3d.graph import components, face_pairs, touch_pairs
from gmc.aerial3d.octree import SAFE, Octree, OctreeConfig
from gmc.aerial3d.pairs import domain_from_scene, pairs_from_scene
from gmc.gs3d.contracts import Pose3
from gmc.gs3d.oracle import GaussianBodyOracle
from aerial3d_fixtures import UAV, full_wall, mini_booth, open_ascent

M = .05


@pytest.fixture(scope="module")
def gap_wall():
    scene, _ = full_wall(gap=True)
    frame, domain = domain_from_scene(scene, UAV, margin_m=M)
    pairs, _ = pairs_from_scene(scene, UAV, frame, domain, margin_m=M)
    table = EnvelopeTable(pairs)
    tree = Octree(table, domain, OctreeConfig())
    return scene, frame, domain, table, tree


def _sample_in_cell(cell, n, rng):
    P = rng.uniform(cell.bbox_lower, cell.bbox_upper, size=(20 * n, 3))
    return P[cell.contains_points(P)][:n]


def test_grown_cell_contains_seed_and_every_facet_is_a_pair_support_plane_or_domain(gap_wall):
    scene, frame, domain, table, tree = gap_wall
    safe = np.flatnonzero(tree.status == SAFE)
    leaf = safe[np.argmax(tree.size[safe])]
    c, d = tree.box(leaf)
    cell = grow_cell(table, domain, c, d, buffer_m=tree.config.buffer_m, seed_leaf=int(leaf))
    assert cell.contains_boxes(c[None], d[None])[0]
    assert set(np.unique(cell.row_kind)) <= {"domain", "pair"}
    V = np.random.default_rng(0).normal(size=(4000, 3)); V /= np.linalg.norm(V, axis=1, keepdims=True)
    for r in np.flatnonzero(cell.row_kind == "pair"):
        i = int(cell.row_pair[r])
        n, beta = cell.A[r], cell.b[r]
        X = table.pairs.support_points(V, [i])[0]           # points of O_i
        assert np.min(X @ n) >= beta + tree.config.buffer_m - 1e-9   # O_i beyond the facet by >= buffer
        assert table.pairs.ids[i] == cell.row_pair_id[r]


def test_cell_points_are_free_for_the_baseline_oracle(gap_wall):
    scene, frame, domain, table, tree = gap_wall
    cx = CellComplex.build(tree, table, domain, CellConfig())
    oracle = GaussianBodyOracle(scene)
    rng = np.random.default_rng(1)
    checked = 0
    for cell in cx.cells[:40]:
        for q in _sample_in_cell(cell, 15, rng):
            assert oracle.pose(Pose3(tuple(frame.to_world(q))), UAV, margin_m=M).occupancy == "free"
            checked += 1
    assert checked > 200


def test_every_safe_leaf_is_covered_and_portals_lie_in_both_cells(gap_wall):
    scene, frame, domain, table, tree = gap_wall
    cx = CellComplex.build(tree, table, domain, CellConfig())
    safe = np.flatnonzero(tree.status == SAFE)
    assert set(cx.leaf_cells) == set(int(l) for l in safe)
    for (a, b), lo, hi in zip(cx.portal_cells, cx.portal_lower, cx.portal_upper):
        centre, half = (lo + hi) / 2, (hi - lo) / 2
        assert cx.cells[a].contains_boxes(centre[None], half[None])[0]
        assert cx.cells[b].contains_boxes(centre[None], half[None])[0]


def test_cell_graph_is_at_least_as_connected_as_the_safe_leaf_face_graph(gap_wall):
    scene, frame, domain, table, tree = gap_wall
    cx = CellComplex.build(tree, table, domain, CellConfig())
    labels = components(len(cx.cells), np.asarray(cx.portal_cells).reshape(-1, 2))
    safe = np.flatnonzero(tree.status == SAFE)
    fp, _ = face_pairs(tree, touch_pairs(tree, safe))
    for a, b in fp:
        la = {labels[c] for c in cx.leaf_cells[int(a)]}
        lb = {labels[c] for c in cx.leaf_cells[int(b)]}
        assert la == lb and len(la) == 1


def test_cell_budget_falls_back_to_box_cells_and_still_covers_every_safe_leaf(gap_wall):
    scene, frame, domain, table, tree = gap_wall
    cx = CellComplex.build(tree, table, domain, CellConfig(max_cells=2))
    kinds = [c.kind for c in cx.cells]
    assert kinds.count("support_plane") == 2 and kinds.count("box") > 0
    safe = np.flatnonzero(tree.status == SAFE)
    assert set(cx.leaf_cells) == set(int(l) for l in safe)
    assert cx.stats["box_cells"] == kinds.count("box")


def test_empty_scene_is_one_cell_equal_to_the_domain():
    scene, _ = open_ascent()
    frame, domain = domain_from_scene(scene, UAV, margin_m=M)
    pairs, _ = pairs_from_scene(scene, UAV, frame, domain, margin_m=M)
    table = EnvelopeTable(pairs)
    tree = Octree(table, domain, OctreeConfig())
    cx = CellComplex.build(tree, table, domain, CellConfig())
    assert len(cx.cells) == 1
    assert np.allclose(cx.cells[0].bbox_lower, domain.bbox_lower, atol=1e-7)
    assert np.allclose(cx.cells[0].bbox_upper, domain.bbox_upper, atol=1e-7)


def test_traceability_fractions_are_reported():
    scene, _ = mini_booth()
    frame, domain = domain_from_scene(scene, UAV, margin_m=M)
    pairs, _ = pairs_from_scene(scene, UAV, frame, domain, margin_m=M)
    table = EnvelopeTable(pairs)
    tree = Octree(table, domain, OctreeConfig())
    cx = CellComplex.build(tree, table, domain, CellConfig())
    t = cx.traceability()
    assert t["support_plane_cell_facets"] > 0
    assert t["pair_facet_fraction_of_support_plane_cells"] + t["domain_facet_fraction_of_support_plane_cells"] == pytest.approx(1.)
    assert t["facets_without_provenance"] == 0


def test_portals_keep_several_spread_regions_per_cell_pair(gap_wall):
    scene, frame, domain, table, tree = gap_wall
    cx = CellComplex.build(tree, table, domain, CellConfig(portal_samples=8, portal_bin_m=.5))
    pairs = [tuple(p) for p in cx.portal_cells]
    counts = {}
    for p in pairs:
        counts[p] = counts.get(p, 0) + 1
    assert max(counts.values()) > 1 and max(counts.values()) <= 8
    # samples of one pair lie in distinct bins
    for p, n in counts.items():
        if n > 1:
            idx = [i for i, q in enumerate(pairs) if q == p]
            centres = (cx.portal_lower[idx] + cx.portal_upper[idx]) / 2
            bins = {tuple(np.floor(c / .5).astype(int)) for c in centres}
            assert len(bins) == n
            break
