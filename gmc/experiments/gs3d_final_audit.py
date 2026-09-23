"""Reproduce final GS3D evidence; this never grants human/release acceptance."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import xml.etree.ElementTree as ET

import numpy as np


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n")


def classify(junit, baseline, output):
    reference = read(baseline)
    expected = {row["test"] for row in reference["failures"]}
    root = ET.parse(junit).getroot()
    failures, unexpected, tests, skipped = [], [], 0, 0
    for case in root.iter("testcase"):
        tests += 1
        skipped += case.find("skipped") is not None
        for kind in ("failure", "error"):
            failure = case.find(kind)
            if failure is None:
                continue
            name = case.get("classname") + "::" + case.get("name")
            message = failure.get("message", "")
            missing_asset = (name in expected and
                ("AtlasAssetError: Atlas root does not contain the sealed round4_final_package" in message
                 or ("FileNotFoundError" in message and "round4_final_package" in message
                     and "src/splatc/datasets/g1_gate.py" in message)))
            row = {"test": name, "kind": kind, "message": message,
                   "category": "reproduced_missing_sealed_atlas_asset" if missing_asset else "unexpected"}
            failures.append(row)
            if not missing_asset:
                unexpected.append(row)
    observed = {row["test"] for row in failures}
    write(output, {"tests": tests, "passed": tests - len(failures) - skipped,
                   "failures": failures, "failed_or_errors": len(failures), "skipped": skipped,
                   "unexpected_failures": unexpected,
                   "baseline_failure_names_match": observed == expected,
                   "baseline_failures_not_reproduced": sorted(expected - observed),
                   "all_available_checks_pass": not unexpected and observed == expected,
                   "junit": str(junit), "junit_sha256": sha256(junit),
                   "baseline_sha256": sha256(baseline),
                   "suite_seconds": sum(float(s.get("time", 0)) for s in root.iter("testsuite"))})


def reproduce(a5_gmc, output):
    import gmc.height.project as legacy_projection
    from gs3d_integration import run as integrate
    from gs3d_render import render
    from gs3d_smoothing import run as smooth
    from gs3d_smoothing_render import render as render_smooth

    scene = a5_gmc / "results/gs3d_scene/showcase_airborne_v1.npz"
    scene_manifest = scene.with_suffix(".manifest.json")
    document = read(scene_manifest)
    output.mkdir(parents=True, exist_ok=True)
    source_checks = [{**row, "actual_sha256": sha256(row["path"])}
                     for row in document["source_inputs"]]
    assert all(row["sha256"] == row["actual_sha256"] for row in source_checks)
    assert sha256(scene) == document["derivative"]["sha256"]
    write(output / "sources.json", source_checks)

    def projection_forbidden(*args, **kwargs):
        raise AssertionError("GS3D production reproduction called legacy 2D projection")

    legacy_projection.project_scene = projection_forbidden
    raw_dir, smooth_dir = output / "integration", output / "smoothing"
    trace = Path("results/height/plane/long/cyl_ladder/rung4/cylinder.json")
    manifest = integrate(scene, scene_manifest, raw_dir, trace_path=trace, warm_calls=5)
    assert manifest["accepted"] and all(manifest["checks"].values())
    render(scene, scene_manifest, raw_dir, raw_dir / "render")

    predecessor = read("results/gs3d/integration_acceptance.json")
    previous_hashes = predecessor["verification"]["raw_artifact_hashes"]
    current_hashes, same_trajectories = {}, {}
    for name in ("uav", "sweeper", "cylinder"):
        previous_path = a5_gmc / "results/gs3d/integration" / f"{name}.json"
        assert sha256(previous_path) == previous_hashes[name]
        previous, current = read(previous_path), read(raw_dir / f"{name}.json")
        if name == "uav":
            a, b = previous["trajectory"], current["trajectory"]
        elif name == "sweeper":
            a, b = previous["result"]["trajectory"], current["result"]["trajectory"]
        else:
            a = previous["goal_sensitivity"]["cases"][0]["result"]["trajectory"]
            b = current["goal_sensitivity"]["cases"][0]["result"]["trajectory"]
        same_trajectories[name] = a == b
        assert same_trajectories[name], f"{name} changed from accepted A5 raw geometry/timing"
        current_hashes[name] = sha256(raw_dir / f"{name}.json")
    # Planner wall timings change, so pin the newly verified files explicitly;
    # no predecessor receipt or prior success record is overwritten.
    receipt = {"schema_version": "gs3d.a7-reproduced-inputs.v1", "scene": manifest["scene"],
               "verification": {"raw_artifact_hashes": current_hashes},
               "same_trajectory_as_hash_verified_a5": same_trajectories}
    receipt_path = output / "smoothing_input_receipt.json"
    write(receipt_path, receipt)
    smoothed = smooth(scene, scene_manifest, raw_dir, receipt_path, smooth_dir, trace)
    assert smoothed["accepted"] and all(smoothed["checks"].values())
    assert all(r["candidate_continuous_verification"] for r in smoothed["robots"].values())
    render_smooth(scene, scene_manifest, smooth_dir, smooth_dir / "render")
    hashes = {str(path.relative_to(output)): sha256(path) for path in sorted(output.rglob("*"))
              if path.is_file()}
    sources = list(Path("src/gmc/gs3d").glob("*.py")) + list(Path("experiments").glob("gs3d_*.py"))
    write(output / "reproduction.json", {
        "computational_checks_passed": True, "human_visual_review_pending": True,
        "projection_trap_active": True, "source_inputs_unchanged": True,
        "same_a5_trajectories": same_trajectories,
        "scene_sha256": sha256(scene), "artifact_sha256": hashes,
        "source_sha256": {str(p): sha256(p) for p in sources},
        "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "python": platform.python_version(), "numpy": np.__version__,
        "host": platform.node(), "seed": 0,
        "acceptance": "pending independent artifact review, full-suite classification and Git audit"})


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("reproduce")
    run.add_argument("--a5-gmc", type=Path, required=True)
    run.add_argument("--output", type=Path, required=True)
    suite = sub.add_parser("classify")
    suite.add_argument("--junit", type=Path, required=True)
    suite.add_argument("--baseline", type=Path, required=True)
    suite.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "reproduce":
        reproduce(args.a5_gmc, args.output)
    else:
        classify(args.junit, args.baseline, args.output)


if __name__ == "__main__":
    main()
