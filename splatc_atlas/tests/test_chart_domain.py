"""P4a.1 domain-soundness + wave-completeness tests (round-8).

Round-8 findings under test:
  - the v1 center-only mask admitted member cells whose corners overhang
    the domain (114 cells at w=1.10, phi=30deg) -> every member cell must
    now lie FULLY inside the domain (corner check = the regression);
  - v1 stopped refinement mid-wave, so the partition depended on grid
    phase / heap order -> two budgets inside the same wave gap must now
    produce IDENTICAL charts and identical spend.
"""
import sys
import os

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from splatc.datasets.g1_gate import make_g1_scene, robot_library, WORKSPACE
from splatc.datasets.transforms import rigid_transform_scene
from splatc.compiler.chart_builder import build_open_charts, cell_bounds
from splatc.compiler.domains import RigidRectDomain

PHI = np.radians(30.0)
T = (0.3, -0.2)


def test_classify_cell_three_way():
    d = RigidRectDomain((-1.0, 1.0, -1.0, 1.0), phi=np.radians(30.0))
    assert d.classify_cell(np.array([-0.2, -0.2]),
                           np.array([0.2, 0.2])) == "IN"
    assert d.classify_cell(np.array([5.0, 5.0]),
                           np.array([6.0, 6.0])) == "OUT"
    # straddles the rotated boundary
    assert d.classify_cell(np.array([0.6, 0.6]),
                           np.array([1.4, 1.4])) == "CROSS"
    # near a rotated corner: center-in / corner-out cells are CROSS
    assert d.classify_cell(np.array([0.9, 0.2]),
                           np.array([1.3, 0.6])) == "CROSS"


def _rotated_build(budget=30000):
    scene = make_g1_scene(1.10)
    sc = rigid_transform_scene(scene, PHI, T)
    dom = RigidRectDomain(WORKSPACE, PHI, T)
    robot = robot_library()["R_long_ellipse"]
    return build_open_charts(sc, robot, budget=budget, domain=dom), dom


def test_members_fully_inside_domain():
    (charts, info), dom = _rotated_build()
    tree = info["tree"]
    n_checked = 0
    for cells in info["members"].values():
        for k in cells:
            lo, hi = cell_bounds(tree.ROOT, tree.origin, tree.span, k)
            for x in (lo[0], hi[0]):
                for y in (lo[1], hi[1]):
                    assert dom.contains_point(x, y), \
                        f"member cell {k} corner ({x},{y}) outside domain"
            n_checked += 1
    assert n_checked > 100
    # the check must actually bite: boundary-crossing FREE cells exist
    # and were excluded from membership
    assert info["n_free_cross"] > 0


def test_wave_complete_budget_invariance():
    scene = make_g1_scene(0.505, door_offset=0.013,
                          door_tilt=np.radians(7.0))
    robot = robot_library()["R_long_ellipse"]
    # level-2 wave completes ~17.5k; the next wave does not fit under
    # either budget -> both must stop at the SAME wave boundary with
    # identical spend and identical charts (no mid-wave partitions)
    (_, i1) = build_open_charts(scene, robot, budget=20000)
    (_, i2) = build_open_charts(scene, robot, budget=25000)
    assert i1["stop"] == i2["stop"] == "budget_before_wave"
    assert i1["queries"] == i2["queries"]
    assert i1["completed_level"] == i2["completed_level"]
    assert i1["members"] == i2["members"]
