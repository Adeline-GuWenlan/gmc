"""Run evidence must be self-contained, structured, and fail closed."""
from dataclasses import replace
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import networkx as nx
import numpy as np
import pytest
import shapely
import yaml

from gmc.cli import cmd_query, cmd_verify, compile_run
from gmc.config import LoggingCfg, load_config
from gmc.io.robot_io import ellipse_robot
from gmc.mobility.query import PlanResult
from gmc.mobility.witness import (PoseCurve, PoseSegment, SegmentKind)
from gmc.reporting.artifacts import (config_payload, load_run_inputs,
                                     file_sha256,
                                     make_run_dir, save_mobility,
                                     save_plan_result,
                                     validate_fixed_slice_direction_artifact_tree,
                                     validate_mobility_lineage_payload)
from gmc.synth import empty_scene
from gmc.types import (GaussianSupport2D, PlanStatus, Pose2, SceneModel2D)

from ..conftest import make_cfg


def _external_config(tmp_path: Path, *, intermediate=True) -> Path:
    cfg = replace(
        make_cfg(theta_min=0.1, initial_intervals=1),
        orientation=replace(
            make_cfg(theta_min=0.1, initial_intervals=1).orientation,
            max_depth=1,
        ),
        logging=LoggingCfg(save_intermediate_geometry=intermediate,
                           save_failed_cases=True),
    )
    path = tmp_path / "external.yaml"
    path.write_text(yaml.safe_dump(config_payload(cfg), sort_keys=False))
    return path


def _compile_empty_bundle(tmp_path: Path, *, intermediate=True) -> tuple[Path, Path]:
    external = _external_config(tmp_path, intermediate=intermediate)
    cfg = load_config(external)
    run = make_run_dir(tmp_path, "run")
    compile_run(empty_scene(workspace=(-1.0, 1.0, -1.0, 1.0)),
                ellipse_robot(0.1, 0.08), cfg, run)
    return run, external


def test_make_run_dir_rejects_nonempty_existing_run(tmp_path):
    run = make_run_dir(tmp_path, "same")
    (run / "input" / "old.yaml").write_text("stale: true\n")
    with pytest.raises(FileExistsError, match="already contains evidence"):
        make_run_dir(tmp_path, "same")


def test_bundle_is_movable_and_external_config_changes_do_not_affect_replay(
        tmp_path):
    run, external = _compile_empty_bundle(tmp_path)
    manifest = json.loads((run / "manifest.json").read_text())
    assert manifest["stage"] == "compile_complete"
    assert manifest["config"]["source"] == "input/config.yaml"
    assert not Path(manifest["config"]["source"]).is_absolute()
    assert manifest["counts"]["pairs"]["total"] == 0
    assert "tree_nodes" in manifest["bvh"]
    assert "totals" in manifest["work_ledger"]
    assert manifest["determinism"]["random_seed"] is None
    assert manifest["artifact_scope"]["pickle_used"] is False
    assert any("query proof binding" in item
               for item in manifest["artifact_scope"]["missing"])
    missing = manifest["artifact_scope"]["missing"]
    for required in ("nerve.json", "oracle_stats.json",
                     "event_brackets.json"):
        assert any(required in item for item in missing)
    alternatives = manifest["artifact_scope"]["available_alternatives"]
    assert "not a formal event-bracket certificate" in \
        alternatives["sampled_event_evidence"]
    assert "build_nerve=False" in alternatives["nerve_status"]
    assert not list((run / "slices").glob("slice_*/nerve.json"))
    assert (run / "input" / "workspace.geojson").is_file()
    assert (run / "pairs" / "pruning.json").is_file()
    assert (run / "pairs" / "stats.json").is_file()
    pruning = manifest["pair_pruning_artifact"]
    assert pruning["status"] == "complete"
    assert pruning["independent_replay_validated"] is True
    assert pruning["decision_count"] == 0
    direction_binding = manifest["fixed_slice_direction_sets_artifact"]
    assert direction_binding["status"] == "complete"
    direction_path = run / direction_binding["path"]
    assert direction_path.is_file()
    direction_payload = json.loads(direction_path.read_text())
    assert validate_fixed_slice_direction_artifact_tree(run)["valid"]
    assert direction_binding["unique_slice_count"] == \
        direction_payload["unique_slice_count"]
    first_slice = next((run / "slices").glob("slice_*"))
    for name in ("meta.json", "components.geojson", "sandwiches.geojson",
                 "directions.json", "provenance.json",
                 "sandwich_unions.geojson"):
        assert (first_slice / name).is_file()
    provenance = json.loads((first_slice / "provenance.json").read_text())
    for relative in provenance["inputs"].values():
        assert (first_slice / relative).resolve().is_file()

    # Make the original config invalid and move the complete run.  Replay
    # must still use the frozen, run-local config.
    external.write_text("not: a valid GMC config\n")
    moved = tmp_path / "elsewhere" / "moved_run"
    moved.parent.mkdir()
    shutil.move(str(run), moved)
    _, frozen_cfg, _, _ = load_run_inputs(moved)
    assert frozen_cfg.query.eps_clear == pytest.approx(2e-3)

    cmd_query(SimpleNamespace(
        run=str(moved), start="-0.5,0,0", goal="0.5,0,0"))
    query_dir = next((moved / "queries").iterdir())
    result = json.loads((query_dir / "result.json").read_text())
    assert result["request"]["start"] == [-0.5, 0.0, 0.0]
    assert result["request"]["compiled_graph_snapshot_used"] is False
    assert result["artifact_scope"]["compiled_graph_snapshot_used"] is False
    proof_binding = result["query_proof_binding"]
    assert proof_binding["status"] == "complete"
    assert proof_binding["proof_kind"] == "verified_pose_curve"
    proof_path = query_dir / proof_binding["path"]
    assert proof_path.is_file()
    proof_text = proof_path.read_text()
    proof = json.loads(proof_text)
    path_payload = json.loads((query_dir / "path.json").read_text())
    assert proof["whole_curve_verification"]["certified"] is True
    assert len(proof["segments"]) == len(path_payload["segments"])
    assert all(segment["certificate_ids"]
               for segment in path_payload["segments"])
    assert [row["certificate_id"] for row in proof["segments"]] == [
        segment["certificate_ids"][-1]
        for segment in path_payload["segments"]
    ]
    replay = result["report"]["replay_compile"]
    assert replay["wall_seconds"] >= 0.0
    assert "totals" in replay["work_ledger"]
    assert replay["graph_source"] == "rebuilt_from_frozen_run_inputs"
    assert cmd_verify(SimpleNamespace(plan=str(query_dir / "result.json"))) == 0
    verified = json.loads((query_dir / "verify.json").read_text())
    assert verified["certified"] is True
    assert verified["query_id"] == query_dir.name
    assert verified["query_proof_validation"]["valid"] is True

    # A geometrically unchanged path is not sufficient if its bound proof was
    # edited after creation.  The verifier must reject the broken digest.
    tampered_proof = json.loads(proof_path.read_text())
    tampered_proof["whole_curve_verification"]["reason"] = "tampered"
    proof_path.write_text(json.dumps(tampered_proof))
    assert cmd_verify(SimpleNamespace(plan=str(query_dir / "result.json"))) == 1
    verified = json.loads((query_dir / "verify.json").read_text())
    assert verified["certified"] is False
    assert verified["reason"] == "verification_error"
    assert "query_proof_digest" in verified["error"]["message"]
    assert "query_proof_digest" in verified[
        "query_proof_validation"]["errors"]

    # Restore the proof to isolate the endpoint-binding assertion below.
    proof_path.write_text(proof_text)

    # Verification must bind the saved curve to the saved query request.  A
    # tiny non-zero endpoint change cannot be hidden by a fixed tolerance.
    result["request"]["start"][2] = 5e-13
    (query_dir / "result.json").write_text(json.dumps(result))
    assert cmd_verify(SimpleNamespace(plan=str(query_dir / "result.json"))) == 1
    verified = json.loads((query_dir / "verify.json").read_text())
    assert verified["certified"] is False
    assert verified["reason"] == "start_endpoint_mismatch"


def test_structured_result_and_certificate_ids_are_not_stringified(tmp_path):
    run = make_run_dir(tmp_path, "structured")
    q0 = Pose2(np.array([0.0, 0.0]), 0.0)
    q1 = Pose2(np.array([0.5, 0.0]), 0.0)
    segment = PoseSegment(
        SegmentKind.TRANSLATION, q0, q1,
        certificate_ids=("pair-1", "workspace-1"),
    )
    result = PlanResult(
        PlanStatus.UNKNOWN,
        curve=PoseCurve((segment,)),
        ambiguity=[("budget", {"used": 4, "limit": 4})],
        report={"nested": {"ok": False, "count": 2}},
    )
    save_plan_result(run, "q_structured", result, versions=False)
    payload = json.loads(
        (run / "queries" / "q_structured" / "result.json").read_text())
    path = json.loads(
        (run / "queries" / "q_structured" / "path.json").read_text())
    assert payload["ambiguity"][0][1] == {"used": 4, "limit": 4}
    assert payload["report"]["nested"] == {"ok": False, "count": 2}
    assert path["segments"][0]["certificate_ids"] == [
        "pair-1", "workspace-1"]


def test_verify_failure_is_persisted_and_returns_nonzero(tmp_path):
    run, _ = _compile_empty_bundle(tmp_path, intermediate=False)
    query_dir = run / "queries" / "q_bad"
    query_dir.mkdir()
    (query_dir / "path.json").write_text(json.dumps({
        "segments": [{
            "kind": "TRANSLATION",
            "q0": [0.0, 0.0, 0.0],
            "q1": [2.0, 0.0, 0.0],
            "control_points": [],
            "certificate_ids": ["deliberately-invalid"],
        }],
    }))
    assert cmd_verify(SimpleNamespace(plan=str(query_dir))) == 1
    payload = json.loads((query_dir / "verify.json").read_text())
    assert payload["certified"] is False
    assert payload["reason"] == "workspace"
    assert payload["query_id"] == "q_bad"


def test_nonterminal_query_automatically_writes_regression_case(tmp_path):
    run, _ = _compile_empty_bundle(tmp_path, intermediate=False)
    cmd_query(SimpleNamespace(
        run=str(run), start="2,0,0", goal="0,0,0",
    ))
    query_dir = next((run / "queries").iterdir())
    result = json.loads((query_dir / "result.json").read_text())
    assert result["status"] == "INVALID_GEOMETRY"
    binding = result["failed_case_binding"]
    assert binding["status"] == "complete"
    failure_path = query_dir / binding["path"]
    assert binding["sha256"] == file_sha256(failure_path)
    failure = json.loads(failure_path.read_text())
    assert failure["request"]["start"] == [2.0, 0.0, 0.0]
    assert failure["status"] == "INVALID_GEOMETRY"
    assert failure["replay_contract"][
        "serialized_graph_is_authoritative"] is False


def test_tampered_run_input_is_rejected_by_query(tmp_path):
    run, _ = _compile_empty_bundle(tmp_path, intermediate=False)
    scene_path = run / "input" / "scene.yaml"
    raw = yaml.safe_load(scene_path.read_text())
    raw["workspace"]["exterior"][1][0] = 1.5
    scene_path.write_text(yaml.safe_dump(raw, sort_keys=False))
    with pytest.raises(ValueError, match="scene hash mismatch"):
        cmd_query(SimpleNamespace(
            run=str(run), start="0,0,0", goal="0.1,0,0"))


def test_tampered_frozen_config_is_rejected_field_by_field(tmp_path):
    run, _ = _compile_empty_bundle(tmp_path, intermediate=False)
    config_path = run / "input" / "config.yaml"
    raw = yaml.safe_load(config_path.read_text())
    raw["query"]["eps_clear"] = 0.123
    config_path.write_text(yaml.safe_dump(raw, sort_keys=False))
    with pytest.raises(ValueError, match=r"config\.query\.eps_clear"):
        load_run_inputs(run)


def test_graphml_preserves_semantics_and_safe_witness_is_recheckable(tmp_path):
    run = make_run_dir(tmp_path, "graph_artifacts")
    cfg = make_cfg(theta_min=0.1, initial_intervals=1)
    scene = empty_scene(workspace=(-1.0, 1.0, -1.0, 1.0))
    robot = ellipse_robot(0.1, 0.08)
    safe = nx.Graph()
    safe.add_node("safe_a", slab=3, comp=4, provenance={"sample": "left"})
    safe.add_node("safe_b", slab=5, comp=6, provenance={"sample": "right"})
    witness = ([np.array([0.0, 0.0])], 0.0, 0.2)
    safe.add_edge("safe_a", "safe_b", weight=0.2, status="SAFE",
                  witness=witness)
    possible = nx.Graph()
    possible.add_node("possible_a", slab=3, comp=8)
    possible.add_node("possible_b", slab=5, comp=9)
    possible.add_edge("possible_a", "possible_b", weight=0.2,
                      status="POSSIBLE",
                      reasons=("no_common_rotation_anchor",))
    mc = SimpleNamespace(
        M_safe=safe, M_possible=possible, scene=scene, robot=robot,
        cfg=cfg, oracles=[],
    )
    summary = save_mobility(run, mc)
    assert summary["safe_edge_witness_files"] == 1
    assert summary["safe_edge_eps_clear_certified"] == 1
    assert summary["lineage_file"] == "mobility/lineage.json"
    assert (run / summary["lineage_file"]).is_file()
    lineage = json.loads((run / summary["lineage_file"]).read_text())
    assert validate_mobility_lineage_payload(lineage, mc)["valid"]
    lineage["nodes"][0]["component_index"] += 1
    assert not validate_mobility_lineage_payload(lineage, mc)["valid"]

    safe_loaded = nx.read_graphml(run / "mobility" / "safe.graphml")
    possible_loaded = nx.read_graphml(run / "mobility" / "possible.graphml")
    assert int(safe_loaded.nodes["safe_a"]["slab"]) == 3
    assert int(safe_loaded.nodes["safe_a"]["comp"]) == 4
    assert json.loads(
        safe_loaded.nodes["safe_a"]["provenance"])["sample"] == "left"
    possible_edge = possible_loaded.edges["possible_a", "possible_b"]
    assert json.loads(possible_edge["reasons"]) == [
        "no_common_rotation_anchor"]

    safe_edge = safe_loaded.edges["safe_a", "safe_b"]
    witness_path = run / "mobility" / safe_edge["witness_file"]
    certificate = json.loads(witness_path.read_text())
    assert safe_edge["certificate_id"] == certificate["certificate_id"]
    assert certificate["certificate_id"] == "rotation_safe_edge_00000"
    assert certificate["witness"]["theta_min"] == pytest.approx(0.1)
    assert certificate["witness"]["eps_clear"] == pytest.approx(2e-3)
    assert certificate["witness"]["all_collision_free"] is True
    assert certificate["witness"]["checks"][0]["min_margin"] == "Infinity"


def test_large_coordinate_numerics_and_sandwich_slacks_are_replayable(tmp_path):
    external = _external_config(tmp_path, intermediate=True)
    cfg = load_config(external)
    origin = 1.0e6
    support = GaussianSupport2D(
        mean=np.array([origin, -origin]),
        covariance=np.eye(2) * (0.05 / 2.0) ** 2,
        level=2.0,
        primitive_id=7,
    )
    scene = SceneModel2D(
        (support,),
        shapely.box(origin - 2.0, -origin - 2.0,
                    origin + 2.0, -origin + 2.0),
        name="large_coordinate_scene",
    )
    run = make_run_dir(tmp_path, "large_coordinate_run")
    compile_run(scene, ellipse_robot(0.1, 0.08), cfg, run)

    manifest_path = run / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    numerics = manifest["numerical_validation"]
    assert numerics["policy_source"] == "gmc.io.gs_io.validate_models"
    assert numerics["world_coordinate_max_magnitude"] >= origin
    assert numerics["world_coordinate_ulp"] == pytest.approx(
        abs(np.spacing(numerics["world_coordinate_max_magnitude"])))
    assert numerics["accepted"] is True
    assert set(numerics["thresholds"]) == {
        "workspace_precision", "eps_pair", "eps_clear"}
    assert all(row["ulp_not_coarser"] is True
               for row in numerics["thresholds"].values())
    assert manifest["scene_support_order"] == [7]
    assert manifest["i7_complete"] is False
    assert manifest["artifact_scope"]["i7_complete"] is False

    direction_files = sorted((run / "slices").glob("slice_*/directions.json"))
    records = [record for path in direction_files
               for record in json.loads(path.read_text())]
    assert records
    slack_fields = {
        "outer_translation_slack",
        "inner_translation_slack",
        "inner_shrink_factor",
    }
    for record in records:
        assert slack_fields <= set(record)
        assert record["outer_translation_slack"] >= 0.0
        assert record["inner_translation_slack"] >= 0.0
        assert 0.0 <= record["inner_shrink_factor"] <= 1.0
    sandwich_file = next(
        (run / "slices").glob("slice_*/sandwiches.geojson"))
    features = json.loads(sandwich_file.read_text())["features"]
    assert features and slack_fields <= set(features[0]["properties"])

    # Numerical context is also checked against the frozen inputs on replay;
    # it is not decorative manifest text.
    manifest["numerical_validation"]["world_coordinate_ulp"] *= 2.0
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="world_coordinate_ulp"):
        load_run_inputs(run)
