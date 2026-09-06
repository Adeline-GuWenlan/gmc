"""P4a.1 serialized-region tests (round-8): OpenChart.region must be a
proof-carrying object — the offline API (chart_contains/chart_connect)
runs on the serialized region ALONE (json round-trip, no tree), its
polylines stay inside the certified cell union, and the union claim is
independently confirmed by the conservative oracle certificate."""
import sys
import os
import json

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from splatc.datasets.g1_gate import make_g1_scene, robot_library, Q_START
from splatc.compiler.chart_builder import (
    build_open_charts, locate_chart, chart_contains, chart_connect,
    cell_bounds)
from splatc.reference.oracle import certify_path_conservative

SCENE = make_g1_scene(0.505, door_offset=0.013, door_tilt=np.radians(7.0))
ROBOT = robot_library()["R_long_ellipse"]
CHARTS, INFO = build_open_charts(SCENE, ROBOT, budget=30000)
START_CID = locate_chart(INFO, Q_START)
REGION = json.loads(json.dumps(
    next(c for c in CHARTS if c.chart_id == START_CID).region))


def _in_some_member_cell(region, pose):
    grid = region["grid"]
    th = (pose[2] - grid["origin"][2]) % grid["span"][2] + grid["origin"][2]
    for c in region["cells"]:
        lo, hi = cell_bounds(grid["root"], grid["origin"], grid["span"], c)
        if (lo[0] - 1e-9 <= pose[0] <= hi[0] + 1e-9
                and lo[1] - 1e-9 <= pose[1] <= hi[1] + 1e-9
                and lo[2] - 1e-9 <= th <= hi[2] + 1e-9):
            return True
    return False


def test_contains_matches_tree_locate():
    for cells in INFO["members"].values():
        for k in list(cells)[::50] + [cells[0], cells[-1]]:
            center = INFO["tree"].cell_center(k)
            key = chart_contains(REGION, center)
            expected = k if k in INFO["members"][START_CID] else None
            assert key == expected


def test_contains_rejects_outside_poses():
    assert chart_contains(REGION, (99.0, 99.0, 0.0)) is None
    # a pose in an AMBIG/COLL cell (the door center at a blocked angle)
    assert chart_contains(REGION, (0.0, 0.013, 0.0)) is None


def test_connect_stays_inside_certified_union():
    reps = [tuple(INFO["tree"].cell_center(k))
            for k in (INFO["members"][START_CID][0],
                      INFO["members"][START_CID][-1])]
    out = chart_connect(REGION, reps[0], reps[1])
    assert out is not None
    wps, cert = out
    assert cert["min_cell_cert_slack_m"] > 0
    # every densely sampled pose on every segment lies in a member cell
    for p, q in zip(wps[:-1], wps[1:]):
        dth = (q[2] - p[2] + np.pi) % (2 * np.pi) - np.pi
        for t in np.linspace(0.0, 1.0, 9):
            pose = (p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]),
                    p[2] + t * dth)
            assert _in_some_member_cell(REGION, pose)
    # independent confirmation: the conservative oracle certificate
    # closes over the same polyline (queries billed to the TEST, not to
    # the chart — chart_connect itself is query-free)
    q0 = INFO["tree"].queries
    ok, mmin, _, _ = certify_path_conservative(SCENE, ROBOT, np.array(wps))
    assert ok and mmin > 0
    assert INFO["tree"].queries == q0


def test_connect_rejects_foreign_pose():
    goal_side = next(cid for cid in INFO["members"] if cid != START_CID
                     and len(INFO["members"][cid]) > 100)
    foreign = tuple(INFO["tree"].cell_center(
        INFO["members"][goal_side][0]))
    assert chart_connect(REGION, tuple(Q_START), foreign) is None
