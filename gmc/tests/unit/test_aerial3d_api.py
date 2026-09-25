"""Compile (scene + body + box) / query (start + goal) API and result contract."""
import json

import numpy as np
import pytest

from gmc.aerial3d.api import CompileConfig, QueryConfig, compile_complex, query
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.timing import PlanTiming
from gmc.gs3d.trajectory import replay_plan
from aerial3d_fixtures import UAV, full_wall, low_wall, open_ascent

COMPILE_STAGES = {"pair_candidates", "envelopes", "octree", "cells", "possible_graph", "audit"}
QUERY_STAGES = {"locate", "graph_search", "lifting", "own_verification", "shared_verification"}


@pytest.fixture(scope="module")
def low():
    scene, queries = low_wall()
    timer = PlanTiming()
    compiled = compile_complex(scene, UAV, timer=timer)
    return scene, queries, compiled, timer


def test_compile_records_timed_substeps_and_is_reused_by_queries(low):
    scene, queries, compiled, timer = low
    stages = {r["stage"] for r in compiled.timings["records"]}
    assert COMPILE_STAGES <= stages
    for r in compiled.timings["records"]:
        assert r["seconds"] >= 0 and "peak_rss_mb" in r["sizes"]
    assert compiled.timings["compile_wall_s"] > 0
    start, goal = queries["over"]
    r1 = query(compiled, start, goal)
    r2 = query(compiled, goal, start)
    assert r1["status"] == r2["status"] == "REACHABLE"
    assert r1["compile_id"] == r2["compile_id"] == compiled.compile_id


def test_reachable_result_contract_and_shared_replay(low):
    scene, queries, compiled, _ = low
    start, goal = queries["over"]
    r = query(compiled, start, goal)
    json.dumps(r, allow_nan=False)
    assert r["status"] == "REACHABLE"
    assert QUERY_STAGES <= {t["stage"] for t in r["timings"]["records"]}
    assert r["verification"]["own"]["status"] == "CERTIFIED"
    assert r["verification"]["shared"]["passed"]
    assert r["clearance_lower_m"] > .05
    poly = np.asarray(r["polyline_world"])
    assert np.allclose(poly[0], start) and np.allclose(poly[-1], goal)
    assert poly[:, 2].max() > 1.0 + .03 + UAV.half_height_m + .05  # over the 1.0 m wall
    gp = r["graph_path"]
    assert gp["cells"] and all("facets_by_kind" in c for c in gp["cells"])
    assert len(gp["portals"]) == len(gp["cells"]) - 1
    # the baseline's own independent replay accepts the exported gs3d.v1 result
    replay = replay_plan(r["gs3d_result"], GaussianBodyOracle(PreparedScene(scene)))
    assert replay["passed"] and replay["variable_z"]


def test_open_ascent_is_a_straight_vertical_segment():
    scene, queries = open_ascent()
    compiled = compile_complex(scene, UAV)
    r = query(compiled, *queries["ascent"])
    assert r["status"] == "REACHABLE"
    poly = np.asarray(r["polyline_world"])
    assert len(poly) == 2 and np.allclose(poly[:, :2], 0.)
    assert r["metrics"]["path_length_m"] == pytest.approx(2.3)


def test_closed_wall_is_certified_unreachable_with_a_pair_cut():
    scene, queries = full_wall(gap=False)
    compiled = compile_complex(scene, UAV)
    r = query(compiled, *queries["across"])
    assert r["status"] == "UNREACHABLE"
    cert = r["certificate"]
    assert cert["kind"] == "possible_space_cut"
    assert cert["blocked_leaves_on_cut"] > 0 and cert["cut_pair_ids"]
    assert cert["cut_traceable_fraction"] == 1.0
    assert r["polyline_world"] is None and r["gs3d_result"] is None


def test_gap_wall_goes_around_through_the_gap():
    scene, queries = full_wall(gap=True)
    compiled = compile_complex(scene, UAV)
    r = query(compiled, *queries["across"])
    assert r["status"] == "REACHABLE"
    poly = np.asarray(r["polyline_world"])
    crossing = poly[np.argmin(np.abs(poly[:, 0]))]
    assert np.max(poly[:, 1]) > .1 + .03 + UAV.radius_m  # detours past the wall end


def test_start_in_collision_is_reported():
    scene, queries = low_wall()
    compiled = compile_complex(scene, UAV)
    r = query(compiled, (0., 0., .5), queries["over"][1])
    assert r["status"] in ("UNREACHABLE", "UNKNOWN")
    assert r["reason"].startswith("start")


def test_status_names_match_the_shared_plan_status_enum():
    from gmc.types import PlanStatus
    from gmc.aerial3d import api
    assert (api.REACHABLE, api.UNREACHABLE, api.UNKNOWN_S) == tuple(
        PlanStatus[n].name for n in ("REACHABLE", "UNREACHABLE", "UNKNOWN"))


def test_lifted_path_is_shortest_within_its_cell_sequence():
    """Portal points move over the whole intersection of their two convex cells (not just a
    portal leaf), and every lifted segment stays inside one cell."""
    scene, queries = low_wall()
    compiled = compile_complex(scene, UAV)
    r = query(compiled, *queries["over"], config=QueryConfig(shortcut=False, tighten=False))
    assert r["status"] == "REACHABLE"
    assert r["diagnostics"]["lifted_segments_inside_their_cell"]
    lifting = [t for t in r["timings"]["records"] if t["stage"] == "lifting"][0]["sizes"]
    assert lifting["cell_sequence_program"]
    assert r["metrics"]["path_length_m"] < 3.8   # 4.22 m when confined to one portal leaf


def test_default_path_is_tightened_to_a_taut_string():
    """Certified tightening pulls the route onto the wall's top corners (analytic ~3.43 m;
    the 0.10 m lattice needs 3.66 m)."""
    scene, queries = low_wall()
    compiled = compile_complex(scene, UAV)
    r = query(compiled, *queries["over"])
    assert r["status"] == "REACHABLE"
    assert r["metrics"]["path_length_m"] < 3.55
    assert r["verification"]["own"]["status"] == "CERTIFIED" and r["verification"]["shared"]["passed"]
