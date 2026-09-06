"""P4a OpenChart auto-construction tests (round-6): goal-free coarse
build on G1 must produce certified charts with start and goal in
DIFFERENT charts (the wall separates the rooms), and the chart TOPOLOGY
(count + start/goal separation) must survive a 90-degree whole-scene
rotation.  Disc scenes rotate exactly, but the ROOT grid's per-axis cell
counts do not swap with the axes, so the cell decomposition differs —
the invariant is the chart topology, not cell-level correspondence
(finer-grained frame/phase sensitivity is measured, not asserted, in
experiments/compiler/p4a_semantic.py)."""
import sys
import os

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, Q_START, GOAL)
from splatc.datasets.transforms import (
    rigid_transform_scene, rigid_transform_pose)
from splatc.compiler.chart_builder import build_open_charts, locate_chart

BUDGET = 6000


def build(scene):
    robot = robot_library()["R_long_ellipse"]
    return build_open_charts(scene, robot, budget=BUDGET)


def test_g1_two_rooms_separate_charts():
    scene = make_g1_scene(0.505, door_offset=0.013,
                          door_tilt=np.radians(7.0))
    charts, info = build(scene)
    assert info["n_charts"] >= 2
    assert all(c.certified for c in charts)
    start_chart = locate_chart(info, Q_START)
    goal_chart = locate_chart(info, (GOAL[0], GOAL[1], Q_START[2]))
    assert start_chart is not None and goal_chart is not None
    assert start_chart != goal_chart
    # goal-free construction: no queries at locate time
    q = info["queries"]
    locate_chart(info, Q_START)
    assert info["tree"].queries == q


def test_chart_construction_90deg_equivariant():
    scene = make_g1_scene(0.505, door_offset=0.013,
                          door_tilt=np.radians(7.0))
    charts0, info0 = build(scene)
    phi = np.pi / 2
    charts1, info1 = build(rigid_transform_scene(scene, phi))
    assert info1["n_charts"] == info0["n_charts"]
    s1 = locate_chart(info1, rigid_transform_pose(Q_START, phi))
    g1 = locate_chart(info1, rigid_transform_pose(
        (GOAL[0], GOAL[1], Q_START[2]), phi))
    assert s1 is not None and g1 is not None and s1 != g1
