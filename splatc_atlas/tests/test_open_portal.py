"""P4a.1 OpenPortal discrimination tests (round-8).

The design ruling under test: an interface between two coarse charts is
NOT automatically a contact gate.  The generic straight-line connector
must certify a broad-clearance interface (wide door), and must FAIL both
a thin contact gate and a sealed wall — with no door metadata, no
orientation window, no G1 frame constant involved.
"""
import sys
import os

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from splatc.datasets.g1_gate import make_g1_scene, robot_library, Q_START, GOAL
from splatc.compiler.chart_builder import build_open_charts, locate_chart
from splatc.compiler.portals import try_certify_portal

ROBOT = robot_library()["R_long_ellipse"]
BUDGET = 30000


def _portal(scene):
    charts, info = build_open_charts(scene, ROBOT, budget=BUDGET)
    cids = sorted(info["members"], key=lambda c: -len(info["members"][c]))
    assert len(cids) >= 2
    portal, rec = try_certify_portal(scene, ROBOT, info, cids[0], cids[1])
    return portal, rec, info


def test_wide_door_portal_certifies():
    scene = make_g1_scene(1.10)
    portal, rec, info = _portal(scene)
    assert portal is not None
    assert portal.certificate["min_margin_m"] > 0
    assert portal.certificate["checks"] > 0
    # portal endpoints are anchored inside the two charts
    a = locate_chart(info, portal.waypoints[0])
    b = locate_chart(info, portal.waypoints[-1])
    assert a is not None and b is not None and a != b
    assert {a, b} == {portal.chart_a, portal.chart_b}


def test_thin_gate_portal_refused():
    scene = make_g1_scene(0.505, door_offset=0.013,
                          door_tilt=np.radians(7.0))
    portal, rec, info = _portal(scene)
    assert portal is None
    # refusal is evidence, not silence: every attempt is on record
    assert len(rec["attempts"]) > 0
    assert all(not a["certified"] for a in rec["attempts"])
    assert rec["checks"] > 0


def test_sealed_wall_portal_refused():
    scene = make_g1_scene(0.505, door_offset=0.013,
                          door_tilt=np.radians(7.0), door_plug=True)
    portal, rec, info = _portal(scene)
    assert portal is None
    assert all(not a["certified"] for a in rec["attempts"])
