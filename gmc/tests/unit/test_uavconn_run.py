"""C2 aerial3d runner contract on a synthetic booth slice (no real archive).

Same slice as ``test_uavlamp_run`` (L1's real builder, manifest-checked loader): a
corridor, a lamp box sealed to the drop ceiling by a bulkhead, a table past the lamp.
The runner must compile once per scene, then make start + goal query calls only, replay
every path with the baseline's fresh-oracle ``replay_plan`` and extract the same ordered
evidence as the baseline runner.
"""
import inspect
import json

import numpy as np
import pytest

from gmc.aerial3d import api
from gmc.gs3d import scene_uavlamp as su

import uavconn_run as cr

IDENTITY = {"origin_world_m": [0., 0., 0.], "world_to_route": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}
# Wide slice (runner contract): corridor v = +-.8, lamp underside .8, table top .6 at u 1.2..1.8.
# Tight slice = test_uavlamp_run's (v = +-.5, table at u 1.0): C-space climb shaft ~.25 m.
GEOM = {"wide": {"half_v": .8, "table_u": (1.2, 1.8), "goal": [1.5, 0, 1.35], "u_max": 2.1},
        "tight": {"half_v": .5, "table_u": (1.0, 1.6), "goal": [1.3, 0, 1.35], "u_max": 1.8}}


def panel(edit_id, role, center, plane, size):
    return {"edit_id": edit_id, "role": role, "kind": "panel", "center_route": list(center),
            "plane": plane, "size_m": list(size), "spacing_m": .1, "half_thickness_m": .02,
            "color_rgb": [.5, .5, .5], "opacity": .95, "justification": "test", "contact": "test"}


def geometry(kind):
    g = GEOM[kind]
    hv, (t0, t1), um = g["half_v"], g["table_u"], g["u_max"]
    box = {"lower": [-1.2, -hv, 0.], "upper": [um, hv, 1.8]}
    lamp = {"footprint_route_uv": [[-.15, -hv], [.15, hv]], "underside_z": .8, "top_z": .95}
    table = {"top_route_uv": [[t0, -.5], [t1, .5]], "top_z": .6}
    return g, box, lamp, table


def build_slice(tmp, kind):
    g, box, _, _ = geometry(kind)
    hv, (t0, t1), um = g["half_v"], g["table_u"], g["u_max"]
    means = np.full((4, 3), 50.) + np.arange(4)[:, None]          # source rows far away
    np.savez(tmp / "src.npz", means=means, covs=np.repeat(np.eye(3)[None] * 1e-4, 4, axis=0),
             opacity=np.full(4, .9), ids=np.arange(4), meta=np.array(json.dumps({"z_floor": 0.})))
    cu, lu = (um - 1.2) / 2, um + 1.2
    edits = [panel("wl", "partition", (cu, -hv, .9), "uz", (lu, 1.8)),
             panel("wr", "partition", (cu, hv, .9), "uz", (lu, 1.8)),
             panel("ceil", "drop_ceiling", (cu, 0, 1.8), "uv", (lu, 2 * hv)),
             panel("floor", "floor", (cu, 0, 0.), "uv", (lu, 2 * hv)),
             panel("bulk", "bulkhead", (0, 0, (.95 + 1.8) / 2), "vz", (2 * hv, 1.8 - .95)),
             panel("table", "table", ((t0 + t1) / 2, 0, .6), "uv", (t1 - t0, 1.))]
    edits += su.lamp_box_edits("lamp", center_uv=(0., 0.), size_uv=(.3, 2 * hv), underside_z=.8,
                               height=.15, spacing=.1, half_thickness=.02)
    config = {"schema_version": "uavlamp.scene.v1", "scene_id": f"slice_{kind}", "source": str(tmp / "src.npz"),
              "frame": IDENTITY, "edits": edits,
              "query": {"start_route": [-.8, 0, .45], "goal_route": g["goal"], "box_route": box}}
    su.build_uavlamp_derivative(config, tmp / "slice.npz", tmp / "slice.json")
    return tmp


@pytest.fixture(scope="module")
def slice_archive(tmp_path_factory):
    return build_slice(tmp_path_factory.mktemp("slice_c2_wide"), "wide")


@pytest.fixture(scope="module")
def tight_archive(tmp_path_factory):
    return build_slice(tmp_path_factory.mktemp("slice_c2_tight"), "tight")


def spec(tmp, name, kind="wide", **extra):
    g, box, lamp, table = geometry(kind)
    doc = {"name": name, "output": str(tmp / name), "archive": str(tmp / "slice.npz"),
           "manifest": str(tmp / "slice.json"), "frame": IDENTITY, "box_route": box,
           "start_route": [-.8, 0, .45], "goal_route": g["goal"],
           "resolution_m": .1, "margin_m": .05, "budget": {"max_wall_s": 90.},
           "lamp": lamp, "table": table, **extra}
    doc["archive_sha256"] = json.loads((tmp / "slice.json").read_text())["derivative"]["sha256"]
    path = tmp / f"{name}.json"
    path.write_text(json.dumps(doc))
    return path


def test_query_api_takes_only_start_and_goal():
    sig = inspect.signature(api.query)
    positional = [n for n, p in sig.parameters.items() if p.kind is p.POSITIONAL_OR_KEYWORD]
    assert positional == ["compiled", "start", "goal"]
    assert all(p.kind is p.KEYWORD_ONLY for n, p in sig.parameters.items() if n not in positional)
    assert set(sig.parameters) - set(positional) == {"config", "timer", "call_id"}


def test_one_compile_start_goal_queries_replay_and_evidence(slice_archive):
    high = spec(slice_archive, "high", start_route=[-.8, 0, 1.2])
    docs = cr.run([spec(slice_archive, "main"), high], slice_archive / "out", warm=2,
                  expected_sha256=None)
    assert [d["name"] for d in docs] == ["main", "high"]
    assert len({d["compile"]["compile_id"] for d in docs}) == 1          # compiled once, reused
    for d in docs:
        assert d["status"] == "REACHABLE", d["reason"]
        assert [c["mode"] for c in d["all_calls"]] == ["cold", "warm", "warm"]
        assert d["identical_routes_across_calls"]
        assert all(c["same_arguments_as_primary"] for c in d["all_calls"])
        rec = d["call_arguments_verbatim"]
        assert set(rec["arguments"]) == {"compiled", "start", "goal"}
        assert rec["keyword_arguments"]["config"] == cr.asdict(api.QueryConfig())
        assert d["replay"]["passed"] and "fresh" in d["replay"]["oracle"]
        ev = d["evidence"]
        assert ev["passes_under_lamp"] and ev["under_precedes_above_table"]
        assert ev["under_lamp_intervals"][0]["z_max"] <= .8 - .10 - .05 + 1e-9
        assert ev["above_table_intervals"][0]["z_min"] >= .6 + .10 + .05 - 1e-9
        assert ev["clearance_lower_m_replay"] >= .05
        t = d["timing"]
        assert len(t["cold_algorithm_wall_s"]) == 1 and len(t["warm_algorithm_wall_s"]) == 2
        assert t["compile_wall_s"] > 0
        assert t["time_to_first_path_s"] == pytest.approx(t["compile_wall_s"] + t["cold_algorithm_wall_s"][0])
        assert (slice_archive / "out" / d["name"] / "result.json").exists()
    assert docs[1]["call_arguments_verbatim"]["arguments"]["start"] == pytest.approx([-.8, 0, 1.2])
    assert (slice_archive / "out" / "compile.json").exists()


def test_plug_variant_is_certified_unreachable_and_attributed(slice_archive):
    plug = {**panel("plug", "test_only_plug", (0, 0, .4), "vz", (1.6, .8)), "spacing_m": .1}
    (d,) = cr.run([spec(slice_archive, "plug", extra_builders=[plug])], slice_archive / "out_plug", warm=1,
                  expected_sha256=None)
    assert d["status"] == "UNREACHABLE" and d["reason"] == "possible_space_cut"
    assert d["replay"] is None and d["evidence"] is None
    roles = d["certificate_attribution"]["cut_pairs_by_role"]
    assert roles.get("test_only_plug", 0) > 0
    assert d["scene_variants_test_only"]["extra_rows"][0]["edit_id"] == "plug"


def test_specs_of_different_scenes_are_rejected(slice_archive):
    other = spec(slice_archive, "other_box", box_route={"lower": [-1.2, -.8, 0.], "upper": [2.0, .8, 1.8]})
    with pytest.raises(ValueError, match="same scene"):
        cr.run([spec(slice_archive, "main"), other], slice_archive / "bad", warm=0, expected_sha256=None)


@pytest.mark.parametrize("key", ["waypoints", "candidate_path_world_m", "z_schedule", "altitude_cost"])
def test_route_steering_keys_are_rejected(slice_archive, key):
    with pytest.raises(ValueError, match="not allowed"):
        cr.run([spec(slice_archive, f"bad_{key}", **{key: [[0, 0, .5]]})], slice_archive / "bad", warm=0,
               expected_sha256=None)


def test_frozen_archive_hash_is_enforced(slice_archive):
    with pytest.raises(ValueError, match="hash"):
        cr.run([spec(slice_archive, "main")], slice_archive / "bad", warm=0, expected_sha256="0" * 64)


def test_tight_slice_is_never_certified_unreachable(tight_archive):
    """L2's tight slice: the lattice finds the route; the complex may miss a ~.25 m C-space
    shaft at min_cell .05 (resolution bound) -- then it must say UNKNOWN, not UNREACHABLE."""
    (d,) = cr.run([spec(tight_archive, "tight", kind="tight")], tight_archive / "out", warm=0,
                  expected_sha256=None)
    assert d["status"] in ("REACHABLE", "UNKNOWN")
    if d["status"] == "REACHABLE":
        assert d["replay"]["passed"]


def test_extended_pairs_are_seeded_oracle_free_and_share_the_template_scene(slice_archive, tmp_path):
    import uavconn_pairs as cp
    from gmc.gs3d.contracts import Pose3
    from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
    from uavlamp_query import UAV, build_scene
    tmpl = json.loads(spec(slice_archive, "tmpl").read_text())
    full, doc = su.load_uavlamp_derivative(tmpl["archive"], tmpl["manifest"])
    frame, scene, *_ = build_scene(tmpl, full, doc)
    regions = {"A_start": {"lower": [-1.0, -.3, .2], "upper": [-.5, .3, .6]},
               "A_goal": {"lower": [1.2, -.3, .8], "upper": [1.8, .3, 1.5]}}
    one = cp.generate(scene, frame, n_a=2, n_b=2, seed=7, regions=regions, min_dist_m=.5)
    two = cp.generate(scene, frame, n_a=2, n_b=2, seed=7, regions=regions, min_dist_m=.5)
    assert [r["start_route"] for r in one["pairs"]] == [r["start_route"] for r in two["pairs"]]
    assert [r["class"] for r in one["pairs"]] == ["A", "A", "B", "B"]
    oracle = GaussianBodyOracle(PreparedScene(scene))
    for r in one["pairs"]:
        assert r["distance_m"] >= .5
        for key in ("start_route", "goal_route"):
            rep = oracle.pose(Pose3(tuple(frame.to_world(r[key]))), UAV, margin_m=.05)
            assert rep.occupancy == "free"
    paths = cp.write_specs(tmpl, one["pairs"], tmp_path / "specs", "out")
    specs = [json.loads(open(p).read()) for p in paths]
    assert len({cr.scene_key(s) for s in specs} | {cr.scene_key(tmpl)}) == 1
    assert all(s["budget"]["max_wall_s"] == 18000. for s in specs)


def test_same_smoothing_on_both_methods_is_freshly_reverified(slice_archive):
    import uavconn_smooth as cs
    import uavlamp_run as ur
    sp = spec(slice_archive, "smooth_main")
    cr.run([sp], slice_archive / "sm_a3", warm=0, expected_sha256=None)
    ur.run(sp, slice_archive / "sm_lat", cold=0, warm=1)
    cs.main(["--template", str(sp), "--out", str(slice_archive / "sm_out"),
             "--run", f"aerial3d:main={slice_archive / 'sm_a3' / 'smooth_main' / 'result.json'}",
             "--run", f"lattice:main={slice_archive / 'sm_lat' / 'result.json'}"])
    for method in ("aerial3d", "lattice"):
        row = json.loads((slice_archive / "sm_out" / f"{method}__main.json").read_text())
        assert row["raw_replay_passed"]
        assert row["fresh_reverification"]["passed"], row["fresh_reverification"]
        assert row["smoothed"]["integrated_squared_jerk"] > 0
        assert row["evidence"]["passes_under_lamp"]
