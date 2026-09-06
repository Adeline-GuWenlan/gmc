"""Interval-cover artifacts are exact, honest, and independently checked."""
from dataclasses import replace
import gzip
import json

import pytest

from gmc.cli import compile_run
from gmc.io.robot_io import ellipse_robot
from gmc.orientation.slab_builder import build_slabs
from gmc.reporting.artifacts import (load_run_inputs, make_run_dir,
                                     save_slabs,
                                     validate_fixed_slice_direction_artifact_tree,
                                     write_manifest)
from gmc.reporting.replay import (INTERVAL_INDEX_RELATIVE, file_sha256,
                                  stable_interval_id,
                                  validate_interval_artifact_tree)
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import single_obstacle
from gmc.types import CertStatus
from gmc.verification.invariants import check_manifest_reproducibility

from ..conftest import make_cfg


def _theorem_cfg(*, rounds=3):
    cfg = make_cfg(
        mode="theorem", eps_pair=2e-2, theta_min=0.1,
        initial_intervals=1,
    )
    return replace(cfg, query=replace(
        cfg.query, max_refinement_rounds=rounds))


def _compile_bundle(tmp_path, *, rounds=3):
    run = make_run_dir(tmp_path, "interval_artifacts")
    mc = compile_run(
        single_obstacle(r=0.25), ellipse_robot(0.12, 0.08),
        _theorem_cfg(rounds=rounds), run,
    )
    return run, mc


def test_interval_pair_cover_tree_and_nondefault_rounds_are_persisted(
        tmp_path):
    run, mc = _compile_bundle(tmp_path, rounds=11)
    manifest = json.loads((run / "manifest.json").read_text())
    index = json.loads((run / INTERVAL_INDEX_RELATIVE).read_text())

    assert manifest["schema_version"] == 4
    assert manifest["config"]["max_refinement_rounds"] == 11
    assert manifest["config"]["values"]["query"][
        "max_refinement_rounds"] == 11
    _, frozen_cfg, _, _ = load_run_inputs(run)
    assert frozen_cfg.query.max_refinement_rounds == 11

    direction_binding = manifest["fixed_slice_direction_sets_artifact"]
    assert direction_binding["status"] == "complete"
    direction_path = run / direction_binding["path"]
    assert direction_binding["sha256"] == file_sha256(direction_path)
    direction_payload = json.loads(direction_path.read_text())
    direction_report = validate_fixed_slice_direction_artifact_tree(
        run, decomposition=mc.decomposition, index=direction_payload,
    )
    assert direction_report["valid"], direction_report["errors"]
    first_slice_path = run / direction_payload["slice_files"][0]["path"]
    with gzip.open(first_slice_path, "rt", encoding="utf-8") as stream:
        first_slice = json.load(stream)
    first_schedule = first_slice["schedules"][0]
    for direction in first_schedule["unit_directions"]:
        assert direction["x"]["hex"] == float(
            direction["x"]["value"]).hex()
        assert direction["y"]["hex"] == float(
            direction["y"]["value"]).hex()

    declared = manifest["interval_cover_artifacts"]
    assert declared["index"] == INTERVAL_INDEX_RELATIVE
    assert declared["index_sha256"] == file_sha256(
        run / INTERVAL_INDEX_RELATIVE)
    assert declared["artifact_tree_validated"] is True
    assert declared["global_cover_provenance"] == \
        index["global_cover_provenance"]
    assert declared["interval_event_free_certified"] is False
    assert index["event_proof_contract"][
        "interval_event_free_certified"] is False

    entry = index["slabs"][0]
    slab = mc.decomposition.slabs[0]
    expected_interval_id = stable_interval_id(
        slab.interval.lo, slab.interval.hi)
    assert entry["interval_id"] == expected_interval_id
    assert entry["interval"]["lo"]["hex"] == \
        float(slab.interval.lo).hex()
    assert entry["interval"]["hi"]["hex"] == \
        float(slab.interval.hi).hex()
    assert entry["cover_status"] == "CERTIFIED"
    assert entry["artifact"] is not None

    certificate = json.loads(
        (run / entry["artifact"]["certificate"]).read_text())
    assert certificate["pair_certificates"]
    pair = certificate["pair_certificates"][0]
    assert expected_interval_id in pair["certificate_id"]
    assert float(slab.interval.midpoint).hex() in pair["certificate_id"]
    assert pair["status"] == "CERTIFIED"
    directions = pair["midpoint_sandwich"]["support_directions_rad"]
    assert len(directions) >= 3
    assert all(direction["hex"] == float(direction["value"]).hex()
               for direction in directions)
    assert all(expected_interval_id in component["component_id"]
               for component in certificate["cover_components"])

    report = validate_interval_artifact_tree(run, manifest=manifest)
    assert report["valid"], report["errors"]
    assert report["certified_interval_cover_count"] == 1
    # This is deliberately only the first I7 tranche.
    assert manifest["i7_complete"] is False
    assert manifest["artifact_scope"]["i7_complete"] is False
    assert check_manifest_reproducibility(run / "manifest.json") is False


def test_interval_artifact_tamper_is_detected(tmp_path):
    run, _ = _compile_bundle(tmp_path)
    index = json.loads((run / INTERVAL_INDEX_RELATIVE).read_text())
    geometry = run / index["slabs"][0]["artifact"]["geometry"]
    # A JSON-preserving edit is still a content-integrity violation.
    geometry.write_text(geometry.read_text() + "\n")

    report = validate_interval_artifact_tree(run)
    assert not report["valid"]
    assert "slab_0_geometry_digest" in report["errors"]


def test_interval_artifact_missing_file_is_detected(tmp_path):
    run, _ = _compile_bundle(tmp_path)
    index = json.loads((run / INTERVAL_INDEX_RELATIVE).read_text())
    certificate = run / index["slabs"][0]["artifact"]["certificate"]
    certificate.unlink()

    report = validate_interval_artifact_tree(run)
    assert not report["valid"]
    assert "slab_0_certificate_missing" in report["errors"]


def test_fixed_slice_direction_file_tamper_is_detected(tmp_path):
    run, mc = _compile_bundle(tmp_path)
    index = json.loads(
        (run / "slices" / "direction_sets.json").read_text())
    direction_path = run / index["slice_files"][0]["path"]
    direction_path.write_bytes(direction_path.read_bytes() + b"tamper")

    report = validate_fixed_slice_direction_artifact_tree(
        run, decomposition=mc.decomposition, index=index,
    )
    assert not report["valid"]
    assert "slice_file_0_digest" in report["errors"]


def test_certified_cover_does_not_claim_hidden_interval_event_exclusion(
        tmp_path):
    cfg = _theorem_cfg()
    scene = single_obstacle(r=0.25)
    robot = ellipse_robot(0.12, 0.08)
    oracles = candidate_pairs(scene, robot, scene.workspace)
    decomposition = build_slabs(scene, robot, cfg, oracles)
    original = decomposition.slabs[0]
    assert original.cover_slice is not None
    assert original.cover_slice.status is CertStatus.CERTIFIED
    assert original.predicates.certifies_no_event is False

    # Model the important hidden-event case: sampled regularity is uncertain,
    # while the independent interval obstacle cover remains certified.
    decomposition.slabs[0] = replace(
        original, kind="uncertain", status=CertStatus.UNKNOWN)
    run = make_run_dir(tmp_path, "hidden_uncertain")
    save_slabs(run, decomposition)
    index = json.loads((run / INTERVAL_INDEX_RELATIVE).read_text())
    entry = index["slabs"][0]

    assert entry["slab_kind"] == "uncertain"
    assert entry["cover_status"] == "CERTIFIED"
    assert entry["event_regularity"]["sampled_regular_candidate"] is True
    assert entry["event_regularity"][
        "interval_event_free_certified"] is False
    certificate = json.loads(
        (run / entry["artifact"]["certificate"]).read_text())
    assert certificate["cover_status"] == "CERTIFIED"
    assert certificate["event_regularity"][
        "interval_event_free_certified"] is False
    report = validate_interval_artifact_tree(run)
    assert report["valid"], report["errors"]


def test_manifest_extra_cannot_override_reserved_evidence(tmp_path):
    run = make_run_dir(tmp_path, "reserved")
    scene = single_obstacle(r=0.25)
    robot = ellipse_robot(0.12, 0.08)
    cfg = _theorem_cfg()
    with pytest.raises(ValueError, match="scene_hash"):
        write_manifest(run, scene, robot, cfg, {
            "stage": "compile_complete",
            "scene_hash": "attacker-controlled",
        })


def test_boolean_i7_upgrade_cannot_bypass_full_replay_contract(tmp_path):
    run, _ = _compile_bundle(tmp_path)
    path = run / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest["i7_complete"] = True
    manifest["artifact_scope"]["i7_complete"] = True
    path.write_text(json.dumps(manifest))

    # The actual interval tree remains valid, but it cannot substitute for
    # the six still-missing full-I7 proof families.
    assert validate_interval_artifact_tree(
        run, manifest=manifest)["valid"]
    assert check_manifest_reproducibility(path) is False


def test_full_i7_roles_cannot_alias_one_unrelated_hashed_file(tmp_path):
    run, _ = _compile_bundle(tmp_path)
    path = run / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest["i7_complete"] = True
    manifest["artifact_scope"]["i7_complete"] = True

    unrelated = "input/config.yaml"
    unrelated_sha256 = file_sha256(run / unrelated)
    roles = {
        "pair_pruning_bounds",
        "fixed_slice_direction_sets",
        "formal_event_brackets",
        "component_lineage",
        "mobility_witnesses",
        "query_proof_bindings",
    }
    manifest["artifact_scope"]["full_replay_contract"] = {
        "schema_version": 1,
        "profile": "gmc_i7_full_replay_v1",
        "independent_replay_ready": True,
        "objects": {
            role: {
                "status": "complete",
                "path": unrelated,
                "sha256": unrelated_sha256,
            }
            for role in roles
        },
    }
    path.write_text(json.dumps(manifest))

    # File existence and matching digests are not semantic replay.  In
    # particular, one valid frozen config cannot impersonate six proof roles.
    assert validate_interval_artifact_tree(
        run, manifest=manifest)["valid"]
    assert check_manifest_reproducibility(path) is False
