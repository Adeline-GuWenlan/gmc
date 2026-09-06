"""P4a contract tests: atlas types serialization determinism + the frozen
compile/query API's goal-independence guards (round-5 review risk #2)."""
import sys
import os

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from splatc.compiler.atlas_types import (
    Atlas, OpenChart, ContactSignature, SectionComponent, GateBranch,
    CertifiedTransition, ChartAttachment, AtlasEdge)
from splatc.compiler.api import compile_scene_robot, query_atlas


def _region(x0, x1):
    # round-8: certified=True requires a real serialized region (grid
    # spec + member cells), not a bbox claim
    return {"grid": {"root": [1, 1, 1], "origin": [x0, -2.0, 0.0],
                     "span": [x1 - x0, 4.0, 6.283185307179586]},
            "level_cap": 0, "cells": [[0, 0, 0, 0]], "adjacency": [],
            "cell_cert_slack_m": [0.05], "domain": None}


def small_atlas():
    # round-7 fixture fixes: opposing_pair must index TWO existing
    # clusters, and a transition must join two DIFFERENT components.
    # round-8 fixture fixes: serialized certified regions, witness
    # endpoint anchoring, chart-to-branch attachments on the edge, and
    # the adjacent_gate_branches mirror.
    a = Atlas()
    a.add(ContactSignature("sig0", [{"normal_cone": [0.0, 1.0, 0.3],
                                     "strength_range": [0.0, 0.2],
                                     "support": "upper jamb"},
                                    {"normal_cone": [0.0, -1.0, 0.3],
                                     "strength_range": [0.0, 0.2],
                                     "support": "lower jamb"}],
                           opposing_pair=[0, 1]))
    a.add(SectionComponent("c0", "s0", [[1, 0], [0, 1]], [-0.01, 0.01],
                           [2.9, 3.3], "FREE", [0.0, 0.0, 3.1], 0.05,
                           "sig0"))
    a.add(SectionComponent("c1", "s1", [[1, 0], [0, 1]], [-0.01, 0.01],
                           [2.9, 3.3], "FREE", [0.02, 0.0, 3.1], 0.05,
                           "sig0"))
    a.add(GateBranch("b0", ["c0", "c1"], [], [0.0, 1.0]))
    a.add(CertifiedTransition("t0", "c0", "c1",
                              [[0.0, 0.0, 3.1], [0.02, 0.0, 3.1]],
                              {"min_margin_m": 0.01, "checks": 3,
                               "checker": "bubble/checker2"}))
    a.add(OpenChart("chartL", _region(-3.0, -0.7), certified=True,
                    adjacent_gate_branches=["b0"]))
    a.add(OpenChart("chartR", _region(0.7, 3.0), certified=True,
                    adjacent_gate_branches=["b0"]))
    a.add(ChartAttachment("aL", "chartL", "c0",
                          [[-1.8, 0.0, 3.1], [0.0, 0.0, 3.1]],
                          {"min_margin_m": 0.02, "checks": 4,
                           "checker": "bubble/checker2"}))
    # round-10 convention: attachment waypoints run CHART INTERIOR ->
    # COMPONENT ANCHOR (validate checks both ends geometrically)
    a.add(ChartAttachment("aR", "chartR", "c1",
                          [[1.8, 0.0, 3.1], [0.02, 0.0, 3.1]],
                          {"min_margin_m": 0.02, "checks": 4,
                           "checker": "bubble/checker2"}))
    a.add(AtlasEdge("e0", "chartL", "b0", "chartR", 1.0, "t0",
                    source_attachment_id="aL", target_attachment_id="aR"))
    # round-10 identity binding
    a.scene_hash = "test-scene-000000"
    a.robot_hash = "test-robot-000000"
    a.checker_id = "bubble/checker2"
    return a


def test_serialization_deterministic():
    h1 = small_atlas().atlas_hash()
    h2 = small_atlas().atlas_hash()
    assert h1 == h2


def test_hash_sensitive_to_content():
    a = small_atlas()
    h1 = a.atlas_hash()
    a.add(OpenChart("chartX", {"bbox": [0, 1, 0, 1]}))
    assert a.atlas_hash() != h1


def test_sampled_vs_certified_fields_distinct():
    a = small_atlas()
    c = a.components["c0"]
    assert c["sampled_min_margin"] == 0.05
    assert c["certified_min_margin"] is None   # never auto-promoted


def test_compile_rejects_goal_keys():
    for bad in ({"goal": (2, 0)}, {"start_pose": (0, 0, 0)},
                {"goal_ranking": "nearest"}, {"goals": []}):
        with pytest.raises(ValueError):
            compile_scene_robot(None, None, bad)


def test_compile_and_query_unimplemented_but_contracted():
    with pytest.raises(NotImplementedError):
        compile_scene_robot(None, None, {"seed_pitch_mm": 250})
    with pytest.raises(TypeError):
        query_atlas("not an atlas", None, None)
    with pytest.raises(NotImplementedError):
        query_atlas(small_atlas(), (-2, 0, 1.57), (2, 0))
