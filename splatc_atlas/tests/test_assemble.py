"""P4a.2 contract tests (round-10): goal-independent compile, real
attachment-branch-attachment chain, serialized query, portal symmetry,
theta-wrap chart_connect.

These are the regressions for the three round-10 blockers:
  - compile receives no poses; hashing the atlas while varying query
    poses must show zero drift and zero compile-counter motion;
  - the gate enters the atlas as REAL ChartAttachment objects anchored
    at the RIDGE endpoints (which the round-9 'attachment test' never
    located), validated geometrically;
  - try_certify_portal(A,B) == try_certify_portal(B,A) by construction.
"""
import sys
import os
import json

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from splatc.datasets.g1_gate import make_g1_scene, robot_library, Q_START, GOAL
from splatc.compiler.chart_builder import build_open_charts, chart_connect
from splatc.compiler.portals import try_certify_portal
from splatc.compiler.assemble import (
    enumerate_portals, build_gate_chain, compile_atlas, query_reachable)

ROBOT = robot_library()["R_long_ellipse"]
# the gate witness is REGENERATED here by the frozen kernel (~1 min,
# deterministic) instead of read from results/tables — the clean-room
# acceptance runs pytest BEFORE the chain has produced any table, so a
# test that reads chain artifacts deadlocks the gate ordering (learned
# from acceptance job 17355854)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "experiments", "compiler"))


@pytest.fixture(scope="module")
def thin_atlas():
    from ridge_continuation import run_case
    scene = make_g1_scene(0.505, door_offset=0.013,
                          door_tilt=np.radians(7.0))
    charts, info = build_open_charts(scene, ROBOT, budget=60000)
    portals, _, pchecks = enumerate_portals(scene, ROBOT, info)
    row, diag = run_case(0.505)
    assert row["status"] == "CERTIFIED_REACHABLE"
    objs, rec = build_gate_chain(scene, ROBOT, info, diag["stations"],
                                 diag["profile"]["wps"][2:-1])
    assert objs is not None, rec
    atlas = compile_atlas(scene, ROBOT, charts, info, portals,
                          gate_objects=objs,
                          compile_queries=info["queries"] + pchecks
                          + rec["checks"])
    return atlas


def test_gate_chain_attaches_ridge_endpoints(thin_atlas):
    a = thin_atlas
    assert a.validate() == []
    e = a.edges["g0_e"]
    assert e["source_chart"] != e["target_chart"]
    # the attachments anchor the BRANCH endpoint components (the poses
    # the round-9 test never located), and their chart-side endpoints
    # are inside the serialized regions — enforced by validate(), which
    # would have flagged both otherwise; check anchors are the ridge
    # endpoint stations, not start/goal
    for aid in (e["source_attachment_id"], e["target_attachment_id"]):
        anchor = a.attachments[aid]["waypoints"][-1]
        assert abs(anchor[0]) < 1.6      # ridge endpoints sit near the
        assert anchor != list(Q_START)   # wall, far from start/goal
    assert a.transitions["g0_t"]["certificate"]["min_margin_m"] > 0


def test_compile_is_pose_free_and_hash_stable(thin_atlas):
    a = thin_atlas
    h0 = a.atlas_hash()
    q0 = a.compile_query_count
    ser = json.loads(a.serialize())
    rng = np.random.RandomState(7)
    n_reach = 0
    for _ in range(10):
        q1 = (float(rng.uniform(-2.8, -1.2)),
              float(rng.uniform(-1.2, 1.2)),
              float(rng.uniform(0, 2 * np.pi)))
        q2 = (float(rng.uniform(1.2, 2.8)),
              float(rng.uniform(-1.2, 1.2)),
              float(rng.uniform(0, 2 * np.pi)))
        r = query_reachable(ser, q1, q2)
        n_reach += bool(r["reachable"])
    assert a.atlas_hash() == h0
    assert a.compile_query_count == q0
    assert n_reach >= 5    # most sampled cross-room pairs resolve


def test_serialize_reload_validate_query(thin_atlas):
    from splatc.compiler.atlas_types import Atlas
    ser = json.loads(thin_atlas.serialize())
    # reload into a fresh Atlas (no live tree anywhere) and re-validate
    b = Atlas(charts=ser["charts"], signatures=ser["signatures"],
              components=ser["components"], branches=ser["branches"],
              transitions=ser["transitions"], portals=ser["portals"],
              attachments=ser["attachments"], edges=ser["edges"],
              compile_provenance=ser["compile_provenance"],
              compile_query_count=ser["compile_query_count"],
              scene_hash=ser["scene_hash"], robot_hash=ser["robot_hash"],
              domain_hash=ser["domain_hash"],
              checker_id=ser["checker_id"])
    assert b.validate() == []
    assert b.atlas_hash() == thin_atlas.atlas_hash()
    r = query_reachable(ser, tuple(Q_START),
                        (GOAL[0], GOAL[1], Q_START[2]))
    assert r["reachable"] and r["route"] == ["g0_e"]


def test_portal_symmetry_by_construction():
    # w=0.62 was the round-10 asymmetric case: A->B certified, B->A not
    scene = make_g1_scene(0.62, door_offset=0.013,
                          door_tilt=np.radians(7.0))
    charts, info = build_open_charts(scene, ROBOT, budget=30000)
    cids = sorted(info["members"], key=lambda c: -len(info["members"][c]))
    p1, r1 = try_certify_portal(scene, ROBOT, info, cids[0], cids[1])
    p2, r2 = try_certify_portal(scene, ROBOT, info, cids[1], cids[0])
    assert (p1 is None) == (p2 is None)
    if p1 is not None:
        assert p1.portal_id == p2.portal_id
        assert p1.waypoints == p2.waypoints
        assert p1.certificate == p2.certificate
        assert p1.certificate["certified_tube_radius_m"] > 0
    assert r1["attempts"] == r2["attempts"]


def test_chart_connect_across_theta_wrap():
    two_pi = 2 * np.pi
    region = {"grid": {"root": [1, 1, 4], "origin": [0.0, -1.0, 0.0],
                       "span": [1.0, 2.0, two_pi]},
              "level_cap": 0,
              "cells": [[0, 0, 0, 0], [0, 0, 0, 3]],
              "adjacency": [[0, 1]],
              "cell_cert_slack_m": [0.1, 0.1],
              "cell_domain_slack_m": None,
              "domain": None}
    out = chart_connect(region, (0.5, 0.0, 0.4), (0.5, 0.0, 6.0))
    assert out is not None
    wps, cert = out
    assert cert["min_cell_cert_slack_m"] > 0
    # the route must pass through the wrap plane theta = 0 (== 2*pi)
    assert any(abs(w[2]) < 1e-9 or abs(w[2] - two_pi) < 1e-9
               for w in wps)
