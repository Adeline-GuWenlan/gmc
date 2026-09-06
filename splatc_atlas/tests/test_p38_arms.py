"""P3.8 contract tests: chart-factored connector baselines.

Red lines mechanically enforced here:
  - no arm reads door metadata (scene.meta guard);
  - a claimed connector must survive independent conservative
    re-certification with endpoints inside the given regions;
  - the plugged control refuses everywhere;
  - fixed seed => bit-identical accounting (deterministic replay).
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from splatc.datasets.g1_gate import make_g1_scene, robot_library
from splatc.compiler.chart_builder import build_open_charts
from splatc.baselines.connector_arms import (
    rrt_connect, lazy_prm_bridge, hrm_se2, region_bbox, region_top_cells,
    in_region)
from splatc.reference.oracle import certify_path_conservative

ROBOT = robot_library()["R_long_ellipse"]
DEALIGN = dict(door_offset=0.013, door_tilt=np.radians(7.0))


class _GuardedMeta(dict):
    """scene.meta stand-in that fails the test on any door-prior read."""
    _BANNED = ("door_width", "door_offset", "door_tilt", "door_style")

    def __getitem__(self, k):
        assert k not in self._BANNED, f"arm read door metadata {k!r}"
        return super().__getitem__(k)

    def get(self, k, default=None):
        assert k not in self._BANNED, f"arm read door metadata {k!r}"
        return super().get(k, default)


def _frontiers(w, plug=False):
    scene = make_g1_scene(w, door_plug=plug, **DEALIGN)
    charts, info = build_open_charts(scene, ROBOT, budget=150000,
                                     max_level=3)
    assert len(charts) >= 2
    return charts[0].region, charts[1].region


def _guarded_scene(w, plug=False):
    scene = make_g1_scene(w, door_plug=plug, **DEALIGN)
    scene.meta = _GuardedMeta(scene.meta)
    return scene


@pytest.fixture(scope="module")
def wide_frontiers():
    return _frontiers(0.62)


def _audit(w, plug, wps, rA, rB):
    assert wps is not None
    assert in_region(rA, tuple(wps[0])) or in_region(rB, tuple(wps[0]))
    assert in_region(rA, tuple(wps[-1])) or in_region(rB, tuple(wps[-1]))
    scene = make_g1_scene(w, door_plug=plug, **DEALIGN)
    ok, mmin, _, _ = certify_path_conservative(
        scene, ROBOT, np.asarray(wps, dtype=float))
    assert ok and mmin > 0


def test_region_helpers_query_free(wide_frontiers):
    rA, rB = wide_frontiers
    x0, x1, y0, y1 = region_bbox(rA)
    assert x0 < x1 and y0 < y1
    (c,) = region_top_cells(rA, 1)
    assert in_region(rA, c)


def test_rrt_connect_wide_success_and_audit(wide_frontiers):
    rA, rB = wide_frontiers
    scene = _guarded_scene(0.62)
    r = rrt_connect(scene, ROBOT, rA, rB, seed=0, budget=60000)
    assert r["success"], r
    assert r["nq"] <= 60000
    _audit(0.62, False, r["path"], rA, rB)


def test_prm_bridge_wide_success_and_audit(wide_frontiers):
    rA, rB = wide_frontiers
    scene = _guarded_scene(0.62)
    r = lazy_prm_bridge(scene, ROBOT, rA, rB, seed=0, budget=60000)
    assert r["success"], r
    _audit(0.62, False, r["path"], rA, rB)


def test_hrm_no_door_metadata_and_budget(wide_frontiers):
    rA, rB = wide_frontiers
    scene = _guarded_scene(0.62)
    r = hrm_se2(scene, ROBOT, rA, rB, budget=25000)
    assert r["nq"] <= 25000
    if r["success"]:
        _audit(0.62, False, r["path"], rA, rB)


def test_seed_determinism_wide(wide_frontiers):
    rA, rB = wide_frontiers
    a = rrt_connect(_guarded_scene(0.62), ROBOT, rA, rB, seed=3,
                    budget=15000)
    b = rrt_connect(_guarded_scene(0.62), ROBOT, rA, rB, seed=3,
                    budget=15000)
    assert (a["success"], a["nq"], a["n_states"]) \
        == (b["success"], b["nq"], b["n_states"])


def test_plugged_refusal():
    rA, rB = _frontiers(0.505, plug=True)
    scene = _guarded_scene(0.505, plug=True)
    r = rrt_connect(scene, ROBOT, rA, rB, seed=0, budget=12000)
    assert not r["success"]
    r = lazy_prm_bridge(_guarded_scene(0.505, plug=True), ROBOT, rA, rB,
                        seed=0, budget=12000)
    assert not r["success"]
