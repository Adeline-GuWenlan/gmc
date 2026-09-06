"""Round-7 fairness fix: exact face-overlap adjacency must equal a
brute-force GEOMETRIC adjacency oracle (interval arithmetic over all leaf
pairs, theta-periodic), with symmetry, on real refined trees of both
kinds.  The old face-center single probe is checked to be a subset —
demonstrating what it could miss on hanging faces."""
import sys
import os

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, Q_START, GOAL, GOAL_RADIUS)
from splatc.baselines.probe_methods import ProbeTree
from splatc.baselines.aniso_tree import AnisoTree

TOL = 1e-9


def bounds_probe(tree, key):
    n = np.array(tree.ROOT) * (2 ** key[0])
    size = tree.span / n
    c = tree.cell_center(key)
    return c - size / 2, c + size / 2


def bounds_aniso(tree, key):
    size = tree.cell_size(key)
    c = tree.cell_center(key)
    return c - size / 2, c + size / 2


def brute_force_adjacency(tree, bounds_fn):
    """Geometric oracle: leaves a, b are face-adjacent iff along exactly
    one dim their intervals TOUCH (incl. theta wrap) and along the other
    two they OVERLAP with positive measure."""
    keys = list(tree.leaves)
    B = {k: bounds_fn(tree, k) for k in keys}
    th_span = tree.span[2]
    adj = {k: set() for k in keys}
    for i, a in enumerate(keys):
        la, ha = B[a]
        for b in keys[i + 1:]:
            lb, hb = B[b]
            for dim in range(3):
                touch = (abs(ha[dim] - lb[dim]) < TOL
                         or abs(hb[dim] - la[dim]) < TOL)
                if dim == 2 and not touch:
                    touch = (abs((ha[2] - lb[2]) % th_span) < TOL
                             or abs((hb[2] - la[2]) % th_span) < TOL)
                if not touch:
                    continue
                ok = True
                for d in range(3):
                    if d == dim:
                        continue
                    if min(ha[d], hb[d]) - max(la[d], lb[d]) <= TOL:
                        ok = False
                        break
                if ok:
                    adj[a].add(b)
                    adj[b].add(a)
                    break
    return adj


def old_probe_neighbors(tree, key, bounds_fn):
    lo, hi = bounds_fn(tree, key)
    c = (lo + hi) / 2
    size = hi - lo
    out = set()
    for dim in range(3):
        for sgn in (-1, 1):
            p = c.copy()
            p[dim] += sgn * (size[dim] / 2 + 1e-6)
            nb = tree.locate(*p)
            if nb is not None and nb != key:
                out.add(nb)
    return out


def check_tree(tree, bounds_fn):
    oracle = brute_force_adjacency(tree, bounds_fn)
    n_extra = 0
    for k in tree.leaves:
        exact = tree.face_adjacent_leaves(k)
        assert exact == oracle[k], \
            f"{k}: exact {sorted(exact - oracle[k])[:3]} vs oracle " \
            f"{sorted(oracle[k] - exact)[:3]}"
        old = old_probe_neighbors(tree, k, bounds_fn)
        assert old <= exact | {k}, f"{k}: probe found non-neighbor"
        n_extra += len(exact - old)
    # symmetry follows from oracle equality; report that hanging faces
    # actually occur in this tree (else the test is vacuous)
    return n_extra


def test_probe_tree_exact_adjacency_matches_geometric_oracle():
    scene = make_g1_scene(0.9, door_offset=0.013, door_tilt=np.radians(7.0))
    robot = robot_library()["R_long_ellipse"]
    tree = ProbeTree(scene, robot, "generic")
    tree.run([1500], Q_START, GOAL, GOAL_RADIUS)
    assert len(tree.leaves) > 500
    n_extra = check_tree(tree, bounds_probe)
    assert n_extra > 0, "no hanging faces exercised — enlarge the tree"


def test_aniso_tree_exact_adjacency_matches_geometric_oracle():
    scene = make_g1_scene(0.9, door_offset=0.013, door_tilt=np.radians(7.0))
    robot = robot_library()["R_long_ellipse"]
    tree = AnisoTree(scene, robot,
                     lambda key, rec, t: abs(rec["rho"]) +
                     0.25 * (key[0] + key[1] + key[2]))
    tree.run([1500], Q_START, GOAL, GOAL_RADIUS)
    assert len(tree.leaves) > 500
    n_extra = check_tree(tree, bounds_aniso)
    assert n_extra > 0, "no hanging faces exercised — enlarge the tree"


def test_quotiented_probe_tree_adjacency():
    scene = make_g1_scene(0.9, door_offset=0.013, door_tilt=np.radians(7.0))
    robot = robot_library()["R_long_ellipse"]
    tree = ProbeTree(scene, robot, "uniform", roi=(-3, 3, -1.8, 1.8),
                     theta_span=float(np.pi))
    tree.run([1200], Q_START, GOAL, GOAL_RADIUS)
    check_tree(tree, bounds_probe)
