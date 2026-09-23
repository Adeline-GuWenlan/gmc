"""L2 runner contract on a synthetic booth slice (no real archive).

The slice is built through L1's real builder and loaded through the manifest-checked
loader: a corridor (walls v=+-.5, floor, drop ceiling 1.8), a lamp box spanning it
(underside .8) sealed to the ceiling by a bulkhead, and a table (top .6) past the lamp.
"""
import inspect
import json

import numpy as np
import pytest

from gmc.gs3d import scene_uavlamp as su
from gmc.gs3d.contracts import GoalRegion, PlannerConfig, Pose3

import uavlamp_run as ur

IDENTITY = {"origin_world_m": [0., 0., 0.], "world_to_route": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}
LAMP = {"footprint_route_uv": [[-.15, -.5], [.15, .5]], "underside_z": .8, "top_z": .95}
TABLE = {"top_route_uv": [[1.0, -.5], [1.6, .5]], "top_z": .6}


def panel(edit_id, role, center, plane, size):
    return {"edit_id": edit_id, "role": role, "kind": "panel", "center_route": list(center),
            "plane": plane, "size_m": list(size), "spacing_m": .1, "half_thickness_m": .02,
            "color_rgb": [.5, .5, .5], "opacity": .95, "justification": "test", "contact": "test"}


@pytest.fixture(scope="module")
def slice_archive(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("slice")
    means = np.full((4, 3), 50.) + np.arange(4)[:, None]          # source rows far away
    np.savez(tmp / "src.npz", means=means, covs=np.repeat(np.eye(3)[None] * 1e-4, 4, axis=0),
             opacity=np.full(4, .9), ids=np.arange(4), meta=np.array(json.dumps({"z_floor": 0.})))
    edits = [panel("wl", "partition", (.3, -.5, .9), "uz", (3.0, 1.8)),
             panel("wr", "partition", (.3, .5, .9), "uz", (3.0, 1.8)),
             panel("ceil", "drop_ceiling", (.3, 0, 1.8), "uv", (3.0, 1.)),
             panel("floor", "floor", (.3, 0, 0.), "uv", (3.0, 1.)),
             panel("bulk", "bulkhead", (0, 0, (.95 + 1.8) / 2), "vz", (1., 1.8 - .95)),
             panel("table", "table", (1.3, 0, .6), "uv", (.6, 1.))]
    edits += su.lamp_box_edits("lamp", center_uv=(0., 0.), size_uv=(.3, 1.), underside_z=.8,
                               height=.15, spacing=.1, half_thickness=.02)
    config = {"schema_version": "uavlamp.scene.v1", "scene_id": "slice", "source": str(tmp / "src.npz"),
              "frame": IDENTITY, "edits": edits,
              "query": {"start_route": [-.8, 0, .45], "goal_route": [1.3, 0, 1.35],
                        "box_route": {"lower": [-1.2, -.5, 0.], "upper": [1.8, .5, 1.8]}}}
    su.build_uavlamp_derivative(config, tmp / "slice.npz", tmp / "slice.json")
    return tmp


def spec(tmp, name, **extra):
    doc = {"name": name, "output": str(tmp / name), "archive": str(tmp / "slice.npz"),
           "manifest": str(tmp / "slice.json"), "frame": IDENTITY,
           "box_route": {"lower": [-1.2, -.5, 0.], "upper": [1.8, .5, 1.8]},
           "start_route": [-.8, 0, .45], "goal_route": [1.3, 0, 1.35],
           "resolution_m": .1, "margin_m": .05, "budget": {"max_wall_s": 90.},
           "lamp": LAMP, "table": TABLE, **extra}
    path = tmp / f"{name}.json"
    path.write_text(json.dumps(doc))
    return path


def test_single_query_contract_end_to_end_with_cold_and_warm_calls(slice_archive):
    doc = ur.run(spec(slice_archive, "main"), slice_archive / "main", cold=1, warm=2)
    assert doc["status"] == "success", doc["reason"]
    assert doc["planner_calls_per_query"] == 1 and len(doc["all_calls"]) == 3
    assert [c["mode"] for c in doc["all_calls"]] == ["cold", "warm", "warm"]
    assert all(c["same_arguments_as_primary"] for c in doc["all_calls"])
    assert doc["identical_routes_across_calls"]
    rec = doc["call_arguments_verbatim"]
    assert set(rec["arguments"]) == {"scene", "body", "start", "goal", "config"}
    assert rec["arguments"]["start"]["xyz"] == pytest.approx([-.8, 0, .45])
    assert rec["arguments"]["goal"]["original"]["xyz"] == pytest.approx([1.3, 0, 1.35])
    assert rec["arguments"]["goal"]["position_tolerance_m"] == 0.
    assert rec["arguments"]["scene"]["known_space"]["lower_route_m"] == [-1.2, -.5, 0.]
    assert doc["replay"]["passed"]
    ev = doc["evidence"]
    assert ev["passes_under_lamp"] and ev["under_precedes_above_table"]
    assert ev["under_lamp_intervals"][0]["z_max"] <= .8 - .10 - .05 + 1e-9
    assert ev["above_table_intervals"][0]["z_min"] >= .6 + .10 + .05 - 1e-9
    assert ev["clearance_lower_m_replay"] > .05
    t = doc["timing"]
    assert len(t["cold_algorithm_wall_s"]) == 1 and len(t["warm_algorithm_wall_s"]) == 2
    assert t["index_preparation_wall_s"] > 0
    assert all(c["preparation_wall_s"] == 0 for c in doc["all_calls"] if c["mode"] == "cold")
    assert (slice_archive / "main" / "result.json").exists()


def test_plug_under_lamp_exhausts_without_route(slice_archive):
    plug = {**panel("plug", "test_only_plug", (0, 0, .4), "vz", (1., .8)), "spacing_m": .1}
    doc = ur.run(spec(slice_archive, "plug", extra_builders=[plug]), slice_archive / "plug", cold=0, warm=1)
    assert doc["status"] in ("no_path_on_lattice", "verification_failed")
    assert doc["reason"] in ("reachable_lattice_exhausted", "reachable_frontier_has_unproven_edges")
    assert doc["result"]["trajectory"] is None and doc["evidence"] is None
    assert doc["scene_variants_test_only"]["extra_rows"][0]["edit_id"] == "plug"


@pytest.mark.parametrize("key", ["waypoints", "candidate_path_world_m", "z_schedule", "altitude_cost"])
def test_spec_with_route_steering_keys_is_rejected(slice_archive, key):
    with pytest.raises(ValueError, match="not allowed"):
        ur.run(spec(slice_archive, f"bad_{key}", **{key: [[0, 0, .5]]}), slice_archive / "bad", 0, 1)


def test_single_plan_call_accepts_no_waypoints():
    params = list(inspect.signature(ur.single_plan_call).parameters)
    assert params == ["planner", "scene", "start", "goal", "config", "timer"]

    class CountingPlanner:
        prepared = None
        calls = []

        def plan(self, *args, **kwargs):
            self.calls.append((args, kwargs))
            return {"status": "stub"}

    with pytest.raises(TypeError):
        ur.single_plan_call(CountingPlanner(), None, Pose3((0, 0, 0)), GoalRegion(Pose3((1, 0, 0))),
                            PlannerConfig(), waypoints=[Pose3((.5, 0, 0))])
    with pytest.raises(TypeError):     # a list of poses is not a goal
        ur.single_plan_call(CountingPlanner(), None, Pose3((0, 0, 0)), [Pose3((.5, 0, 0)), Pose3((1, 0, 0))],
                            PlannerConfig())
    assert CountingPlanner.calls == []


def _poly(*knots, step=.01):
    out = [np.asarray(knots[0], float)]
    for a, b in zip(knots, knots[1:]):
        n = int(np.ceil(np.linalg.norm(np.subtract(b, a)) / step))
        out += [np.asarray(a) + (np.subtract(b, a)) * k / n for k in range(1, n + 1)]
    return np.asarray(out)


def evidence(route):
    return ur.ordered_evidence(route, lamp=LAMP, table=TABLE, radius=.25, half_height=.1, margin=.05)


def test_ordered_evidence_under_then_above_table():
    ev = evidence(_poly((-.8, 0, .45), (.45, 0, .45), (.65, 0, 1.35), (1.3, 0, 1.35)))
    assert ev["passes_under_lamp"] and ev["under_precedes_above_table"]
    (u,), (a,) = ev["under_lamp_intervals"], ev["above_table_intervals"]
    assert u["s_from_m"] == pytest.approx(.8 - .15 - .25, abs=.011)        # body edge reaches footprint
    assert u["s_to_m"] == pytest.approx(.8 + .15 + .25, abs=.011)
    assert u["z_min"] == u["z_max"] == pytest.approx(.45)
    assert a["z_min"] >= .75 and ev["z_range"] == pytest.approx([.45, 1.35])
    assert ev["goal_minus_lowest_under_lamp_z"] == pytest.approx(.9)


def test_ordered_evidence_rejects_over_and_wrong_order():
    over = evidence(_poly((-.8, 0, 1.2), (1.3, 0, 1.2)))
    assert not over["passes_under_lamp"] and over["over_lamp_intervals"] and not over["under_lamp_intervals"]
    backwards = evidence(_poly((1.3, 0, 1.35), (.65, 0, 1.35), (.45, 0, .45), (-.8, 0, .45)))
    assert backwards["under_lamp_intervals"] and backwards["above_table_intervals"]
    assert not backwards["under_precedes_above_table"]
