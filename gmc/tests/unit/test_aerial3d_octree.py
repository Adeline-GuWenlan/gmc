"""Pair-driven adaptive octree (uav.md §13.3) and the possible-side graph."""
import numpy as np
import pytest

from gmc.aerial3d.envelopes import EnvelopeTable
from gmc.aerial3d.graph import possible_components, touch_pairs
from gmc.aerial3d.octree import BLOCKED, OUTSIDE, SAFE, UNKNOWN, Octree, OctreeConfig
from gmc.aerial3d.pairs import domain_from_scene, pairs_from_scene
from gmc.gs3d.contracts import Pose3
from gmc.gs3d.oracle import GaussianBodyOracle
from aerial3d_fixtures import UAV, full_wall, make, open_ascent
from gs3d_core_fixtures import make_scene

M = .05


def _build(scene, body=UAV, **cfg):
    frame, domain = domain_from_scene(scene, body, margin_m=M)
    pairs, _ = pairs_from_scene(scene, body, frame, domain, margin_m=M)
    table = EnvelopeTable(pairs)
    return frame, domain, table, Octree(table, domain, OctreeConfig(**cfg))


def test_empty_scene_is_one_safe_component_tiling_the_roots():
    scene, _ = open_ascent()
    _, domain, _, tree = _build(scene)
    assert set(np.unique(tree.status)) <= {SAFE, OUTSIDE, UNKNOWN}
    assert np.count_nonzero(tree.status == SAFE) > 0
    # leaves tile the root grid exactly (integer volume)
    assert int(np.sum(tree.size.astype(np.int64) ** 3)) == int(np.prod(tree.root_shape)) * tree.root_units ** 3
    live = np.flatnonzero((tree.status == SAFE) | (tree.status == UNKNOWN))
    comp = possible_components(tree, live, touch_pairs(tree, live))
    assert len(set(comp.values())) == 1


def test_leaf_labels_agree_with_the_baseline_oracle():
    scene = make_scene([(0., 0., 1.2), (.6, .3, .7)], [(.5, .3, .2), (.2, .6, .4)],
                       rotations=np.stack([np.eye(3), np.array([[.8, -.6, 0], [.6, .8, 0], [0, 0, 1.]])]),
                       lower=(-1.5, -1., 0.), upper=(1.5, 1., 2.5))
    frame, domain, table, tree = _build(scene)
    oracle = GaussianBodyOracle(scene)
    rng = np.random.default_rng(0)
    counts = {SAFE: 0, BLOCKED: 0}
    for status in (SAFE, BLOCKED):
        leaves = np.flatnonzero(tree.status == status)
        assert len(leaves) > 0
        for leaf in rng.choice(leaves, size=min(60, len(leaves)), replace=False):
            c, d = tree.box(leaf)
            for q in c + rng.uniform(-1, 1, size=(5, 3)) * d:
                occ = oracle.pose(Pose3(tuple(frame.to_world(q))), UAV, margin_m=M).occupancy
                if status == SAFE:
                    assert occ == "free", (leaf, q, occ)
                else:
                    assert occ != "free", (leaf, q, occ)
                counts[status] += 1
    assert counts[SAFE] > 100 and counts[BLOCKED] > 100


def test_every_leaf_carries_pair_provenance():
    scene, _ = full_wall(gap=True)
    _, _, table, tree = _build(scene)
    blocked = np.flatnonzero(tree.status == BLOCKED)
    assert len(blocked) and np.all(tree.blocked_pair[blocked] >= 0)
    assert np.all(tree.blocked_pair[blocked] < len(table.pairs))
    for leaf in np.flatnonzero(tree.status == UNKNOWN):
        pairs = tree.unknown_pairs.get(int(leaf), np.empty(0))
        assert len(pairs) > 0 or tree.domain_partial[leaf]
    safe = np.flatnonzero(tree.status == SAFE)
    assert np.all(tree.n_candidates[safe] >= 0)
    near = safe[tree.n_candidates[safe] > 0]
    assert len(near) and np.all(tree.min_gap_pair[near] >= 0)
    assert np.all(tree.min_gap[near] > tree.config.buffer_m)


def test_closed_full_height_wall_separates_the_possible_graph():
    scene, queries = full_wall(gap=False)
    frame, domain, table, tree = _build(scene)
    start, goal = (np.asarray(p) for p in queries["across"])
    live = np.flatnonzero((tree.status == SAFE) | (tree.status == UNKNOWN))
    comp = possible_components(tree, live, touch_pairs(tree, live))
    s = {comp[int(l)] for l in tree.locate(start) if int(l) in comp}
    g = {comp[int(l)] for l in tree.locate(goal) if int(l) in comp}
    assert s and g and not (s & g)


def test_open_wall_gap_keeps_the_possible_graph_connected():
    scene, queries = full_wall(gap=True)
    _, _, _, tree = _build(scene)
    start, goal = (np.asarray(p) for p in queries["across"])
    live = np.flatnonzero((tree.status == SAFE) | (tree.status == UNKNOWN))
    comp = possible_components(tree, live, touch_pairs(tree, live))
    s = {comp[int(l)] for l in tree.locate(start) if int(l) in comp}
    g = {comp[int(l)] for l in tree.locate(goal) if int(l) in comp}
    assert s & g


def test_locate_returns_the_leaves_whose_closed_box_contains_the_point():
    scene, _ = full_wall(gap=True)
    _, _, _, tree = _build(scene)
    rng = np.random.default_rng(3)
    for q in rng.uniform(tree.origin, tree.origin + np.asarray(tree.root_shape) * tree.root_m, size=(50, 3)):
        leaves = tree.locate(q)
        assert len(leaves) >= 1
        for leaf in leaves:
            c, d = tree.box(leaf)
            assert np.all(np.abs(q - c) <= d + 1e-9)


def test_outside_leaves_contain_no_domain_point():
    scene, _ = full_wall(gap=True)
    _, domain, _, tree = _build(scene)
    rng = np.random.default_rng(4)
    for leaf in np.flatnonzero(tree.status == OUTSIDE)[:100]:
        c, d = tree.box(leaf)
        for q in c + rng.uniform(-1, 1, size=(10, 3)) * d:
            assert domain.row_slack(q) <= 0


def test_grid_tiles_an_axis_aligned_domain_without_boundary_leaves():
    scene, _ = full_wall(gap=True)
    _, domain, _, tree = _build(scene)
    assert not np.any(tree.status == OUTSIDE)
    assert not np.any(tree.domain_partial)
    assert np.all(tree.unit <= tree.config.min_cell_m + 1e-15)
    top = tree.origin + np.asarray(tree.root_shape) * tree.root_m
    assert np.allclose(tree.origin, domain.bbox_lower, atol=1e-7)
    assert np.allclose(top, domain.bbox_upper, atol=1e-7)
