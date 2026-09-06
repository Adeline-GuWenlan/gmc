"""Minimal CLI workflow (Guide §13.1):
  gmc synth | compile-slice | compile | query | verify | benchmark
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import yaml

from . import synth
from .config import load_config
from .io.gs_io import load_scene, save_scene, validate_models
from .io.robot_io import ellipse_robot, load_robot, save_robot
from .types import Pose2


def _parse_pose(text: str) -> Pose2:
    x, y, th = (float(v) for v in text.split(","))
    return Pose2(np.array([x, y]), th)


def cmd_synth(args):
    maker = synth.FAMILIES[args.family]
    kwargs = {}
    if args.width is not None:
        key = "width" if args.family == "single-door" else "slot_width"
        kwargs[key] = args.width
    scene = maker(**kwargs)
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    save_scene(scene, out)
    a, b = (float(v.split("=")[1]) for v in args.robot.split())
    save_robot(ellipse_robot(a, b), out.with_name(out.stem + "_robot.yaml"))
    print(f"scene -> {out} ({len(scene.supports)} supports); robot a={a} b={b}")


def cmd_compile_slice(args):
    from .reporting.plots import plot_slice
    from .spatial.slice_compiler import build_slice
    cfg = load_config(args.config)
    scene = load_scene(args.scene)
    robot = load_robot(Path(args.scene).with_name(
        Path(args.scene).stem + "_robot.yaml"))
    validate_models(scene, robot, cfg)
    sl = build_slice(scene, robot, args.theta, cfg)
    print(f"theta={args.theta}: safe={len(sl.D_safe)} "
          f"possible={len(sl.D_possible)} status={sl.status.name} "
          f"support_calls={sl.support_calls}")
    if args.plot:
        plot_slice(sl, args.plot)
        print("plot ->", args.plot)


def compile_run(scene, robot, cfg, run_dir: Path):
    from .budget import WorkLedger
    from .mobility.graph import compile_mobility
    from .orientation.slab_builder import build_slabs
    from .reporting.artifacts import (save_fixed_slice_direction_sets,
                                      save_frozen_config,
                                      save_intermediate_geometry,
                                      save_mobility, save_pair_pruning,
                                      save_slabs,
                                      write_manifest)
    from .reporting.metrics import collect_metrics, write_metrics
    from .spatial.bvh import query_candidate_pairs
    run_dir = Path(run_dir)
    validate_models(scene, robot, cfg)
    existing_files = ([path for path in run_dir.rglob("*") if path.is_file()]
                      if run_dir.exists() else [])
    if existing_files:
        raise FileExistsError(
            "run already contains evidence; compile requires a pristine "
            f"directory: {run_dir} ({existing_files[0].relative_to(run_dir)})")
    for sub in ("input", "pairs", "slices", "slabs",
                "mobility/witnesses", "queries", "figures"):
        (run_dir / sub).mkdir(parents=True, exist_ok=True)
    save_scene(scene, run_dir / "input" / "scene.yaml")
    save_robot(robot, run_dir / "input" / "robot.yaml")
    save_frozen_config(run_dir, cfg)
    t0 = time.time()
    pair_query = query_candidate_pairs(scene, robot, scene.workspace)
    save_pair_pruning(run_dir, pair_query)
    oracles = list(pair_query.oracles)
    ledger = WorkLedger()
    for oracle in oracles:
        oracle.ledger = ledger
    with ledger.phase("slab_compile"):
        dec = build_slabs(scene, robot, cfg, oracles, ledger=ledger)
    with ledger.phase("mobility_witness"):
        mc = compile_mobility(scene, robot, cfg, oracles, dec,
                              ledger=ledger)
    save_slabs(run_dir, dec)
    save_fixed_slice_direction_sets(run_dir, dec)
    with ledger.phase("artifact_witness_recheck"):
        witness_summary = save_mobility(run_dir, mc)
    wall = time.time() - t0
    metrics = collect_metrics(
        mc, dec, wall, pair_stats=pair_query.stats, ledger=ledger
    )
    write_metrics(run_dir, metrics)

    intermediate = {}
    if cfg.logging.save_intermediate_geometry:
        intermediate = save_intermediate_geometry(
            run_dir, scene, pair_query, dec)

    unique_slices = []
    seen = set()
    for sl in dec.slice_cache.values():
        if id(sl) not in seen:
            seen.add(id(sl))
            unique_slices.append(sl)
    counts = {
        "pairs": {
            "total": int(metrics["n_pairs_total"]),
            "retained": int(metrics["n_pairs_retained"]),
            "pruned": int(metrics["n_pairs_pruned"]),
        },
        "slices": {
            "unique": int(dec.n_slices),
            "safe_components": int(sum(len(sl.D_safe)
                                       for sl in unique_slices)),
            "possible_components": int(sum(len(sl.D_possible)
                                           for sl in unique_slices)),
        },
        "slabs": {
            "total": len(dec.slabs),
            "regular": int(sum(s.kind == "regular" for s in dec.slabs)),
            "uncertain": int(sum(s.kind == "uncertain" for s in dec.slabs)),
            "mid_safe_components": int(sum(len(s.mid_slice.D_safe)
                                           for s in dec.slabs)),
            "mid_possible_components": int(sum(len(s.mid_slice.D_possible)
                                               for s in dec.slabs)),
        },
        "mobility": {
            "safe_nodes": mc.M_safe.number_of_nodes(),
            "safe_edges": mc.M_safe.number_of_edges(),
            "possible_nodes": mc.M_possible.number_of_nodes(),
            "possible_edges": mc.M_possible.number_of_edges(),
            **witness_summary,
        },
    }
    produced = [
        "input/scene.yaml", "input/robot.yaml", "input/config.yaml",
        "pairs/pruning.json",
        "slabs/slabs.json", "slices/direction_sets.json",
        "slices/direction_sets/slice_*.json.gz",
        "mobility/safe.graphml",
        "mobility/possible.graphml", "mobility/lineage.json",
        "metrics.json", "manifest.json",
    ]
    not_applicable = []
    if witness_summary["safe_edge_witness_files"]:
        produced.append("mobility/witnesses/safe_edge_*.json")
    else:
        not_applicable.append(
            "SAFE edge witness files (compiled SAFE graph has no edges)")
    missing = [
        "figures (not generated by compile)",
        "raw per-call support-value trace",
        ("slices/slice_*/nerve.json (not generated: compile uses "
         "build_nerve=False, so no nerve was computed)"),
        ("pairs/oracle_stats.json (no standalone per-oracle evaluation "
         "table)"),
        ("orientation/event_brackets.json (no standalone certified event "
         "bracket artifact; uncertain/sample intervals are not equivalent "
         "to formal event brackets)"),
        ("query proof binding (no query has run at compile time; cmd_query "
         "creates a content-addressed proofs.json and independently replayed "
         "verify.json)"),
    ]
    if cfg.logging.save_intermediate_geometry:
        produced += [
            "input/workspace.geojson", "pairs/stats.json",
            "slices/slice_*/meta.json",
            "slices/slice_*/components.geojson",
            "slices/slice_*/sandwiches.geojson",
            "slices/slice_*/directions.json",
            "slices/slice_*/provenance.json",
            "slices/slice_*/sandwich_unions.geojson",
        ]
    else:
        missing.append(
            "intermediate geometry disabled by logging.save_intermediate_geometry")

    # Completion marker is written only after every declared compile artifact.
    write_manifest(run_dir, scene, robot, cfg, {
        "stage": "compile_complete",
        "wall_seconds": wall,
        "counts": counts,
        "bvh": metrics.get("bvh", {}),
        "work_ledger": metrics.get("work_ledger", {}),
        "artifact_scope": {
            "profile": "replayable_compile_v1",
            "i7_complete": False,
            "i7_reason": (
                "formal generic event brackets are not available and no "
                "per-query proof exists at compile time"),
            "produced": produced,
            "missing": missing,
            "not_applicable": not_applicable,
            "available_alternatives": {
                "candidate_pair_summary": (
                    "pairs/pruning.json contains every Cartesian pair, its "
                    "directed-rounding bound, and retained/rejected verdict"),
                "oracle_work_summary": (
                    "metrics.json and manifest work_ledger; per-slice "
                    "sandwich support_calls/directions are in "
                    "slices/slice_*/directions.json when enabled"),
                "sampled_event_evidence": (
                    "slabs/slabs.json predicates and uncertain intervals; "
                    "this is not a formal event-bracket certificate"),
                "lineage_summary": (
                    "mobility/lineage.json contains nodes, edges, all "
                    "candidate-link outcomes, safe-to-possible mappings, "
                    "and SAFE witness references"),
                "nerve_status": (
                    "unavailable because build_nerve=False; no empty nerve "
                    "file is emitted"),
            },
            "intermediate": intermediate,
            "pickle_used": False,
        },
    })
    return mc


def cmd_compile(args):
    from .reporting.artifacts import make_run_dir
    cfg = load_config(args.config)
    scene = load_scene(args.scene)
    robot = load_robot(Path(args.scene).with_name(
        Path(args.scene).stem + "_robot.yaml"))
    run_dir = make_run_dir(Path(args.output).parent, Path(args.output).name)
    mc = compile_run(scene, robot, cfg, run_dir)
    print(f"compiled -> {run_dir}: safe {mc.M_safe.number_of_nodes()}n/"
          f"{mc.M_safe.number_of_edges()}e, possible "
          f"{mc.M_possible.number_of_nodes()}n/"
          f"{mc.M_possible.number_of_edges()}e")


def cmd_query(args):
    from dataclasses import asdict
    from .budget import WorkLedger
    from .mobility.graph import compile_mobility
    from .mobility.query import query
    from .orientation.slab_builder import build_slabs
    from .reporting.artifacts import load_run_inputs, save_plan_result
    from .spatial.bvh import query_candidate_pairs
    run_dir = Path(args.run)
    _, cfg, scene, robot = load_run_inputs(run_dir)
    replay_t0 = time.time()
    pair_query = query_candidate_pairs(scene, robot, scene.workspace)
    oracles = list(pair_query.oracles)
    replay_ledger = WorkLedger()
    for oracle in oracles:
        oracle.ledger = replay_ledger
    with replay_ledger.phase("replay_slab_compile"):
        dec = build_slabs(scene, robot, cfg, oracles, ledger=replay_ledger)
    with replay_ledger.phase("replay_mobility_witness"):
        mc = compile_mobility(scene, robot, cfg, oracles, dec,
                              ledger=replay_ledger)
    replay_compile = {
        "wall_seconds": time.time() - replay_t0,
        "work_ledger": replay_ledger.snapshot(),
        "bvh": asdict(pair_query.stats),
        "graph_source": "rebuilt_from_frozen_run_inputs",
        "compiled_graph_snapshot_used": False,
        "graphml_role": "compile-time evidence snapshot only",
    }
    start, goal = _parse_pose(args.start), _parse_pose(args.goal)
    result = query(start, goal, mc)
    result.report["replay_compile"] = replay_compile
    qid = f"q_{time.time_ns()}"
    save_plan_result(run_dir, qid, result, request={
        "start": [*map(float, start.xy), float(start.theta)],
        "goal": [*map(float, goal.xy), float(goal.theta)],
        "graph_execution_source": "rebuilt_from_frozen_run_inputs",
        "compiled_graph_snapshot_used": False,
    }, artifact_scope={
        "execution_graph": "rebuilt_from_frozen_run_inputs",
        "compiled_graph_snapshot_used": False,
        "compiled_graphml_role": "compile-time evidence snapshot only",
        "missing": [],
    }, proof_context=mc)
    print(f"{result.status.name}  (query {qid})")
    if result.clearance_lower_bound is not None:
        print(f"clearance_lb = {result.clearance_lower_bound:.4f}")
    for a in result.ambiguity:
        print("ambiguity:", a)


def _find_run_dir(path: Path) -> Path:
    """Find the nearest ancestor that is a self-contained GMC run."""
    start = path if path.is_dir() else path.parent
    for candidate in (start, *start.parents):
        if ((candidate / "manifest.json").is_file()
                and (candidate / "input").is_dir()):
            return candidate
    raise FileNotFoundError(f"no GMC run manifest found above {path}")


def _load_plan_document(path: Path) -> tuple[Path, dict]:
    """Accept a query directory, path.json, or result.json."""
    if path.is_dir():
        if (path / "path.json").is_file():
            path = path / "path.json"
        elif (path / "result.json").is_file():
            path = path / "result.json"
        else:
            raise FileNotFoundError(f"no path.json or result.json in {path}")
    document = json.loads(path.read_text())
    if not isinstance(document, dict):
        raise ValueError("plan root must be a JSON object")
    if "segments" in document:
        return path, document
    if "trajectory_file" in document:
        target = (path.parent / document["trajectory_file"]).resolve()
        try:
            target.relative_to(path.parent.resolve())
        except ValueError as exc:
            raise ValueError("trajectory_file escapes the query directory") from exc
        trajectory = json.loads(target.read_text())
        if not isinstance(trajectory, dict) or "segments" not in trajectory:
            raise ValueError("trajectory document has no segments")
        return target, trajectory
    curve = document.get("curve")
    if isinstance(curve, dict) and "segments" in curve:
        return path, curve
    raise ValueError("plan has no trajectory segments")


def _load_query_endpoints(query_dir: Path) -> tuple[Pose2 | None, Pose2 | None]:
    """Load the original request when a path belongs to a query artifact.

    A standalone ``path.json`` can still be checked as a geometric curve.  If
    its sibling ``result.json`` declares a request, however, verification is
    also required to prove that the curve realizes those exact endpoints.
    """
    result_path = Path(query_dir) / "result.json"
    if not result_path.is_file():
        return None, None
    result = json.loads(result_path.read_text())
    if not isinstance(result, dict):
        raise ValueError("query result root must be a JSON object")
    request = result.get("request", {})
    if not isinstance(request, dict):
        raise ValueError("query request must be a JSON object")
    has_start, has_goal = "start" in request, "goal" in request
    if has_start != has_goal:
        raise ValueError("query request must contain both start and goal")
    if not has_start:
        return None, None

    def pose(name: str) -> Pose2:
        value = request[name]
        if (not isinstance(value, list) or len(value) != 3
                or any(isinstance(item, bool) for item in value)):
            raise ValueError(f"query request {name} must be [x, y, theta]")
        try:
            return Pose2(np.asarray(value[:2], dtype=float), float(value[2]))
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"query request {name} must contain finite numbers") from exc

    return pose("start"), pose("goal")


def cmd_verify(args):
    from .mobility.witness import PoseCurve, PoseSegment, SegmentKind
    from .reporting.artifacts import (load_run_inputs, save_verify_result,
                                      scene_hash, robot_hash,
                                      validate_saved_query_proof)
    from .spatial.bvh import candidate_pairs
    from .verification.path import verify_curve
    supplied = Path(args.plan)
    query_dir = supplied if supplied.is_dir() else supplied.parent
    query_id = query_dir.name
    input_hashes = {}
    proof_validation = None
    try:
        resolved_plan, plan = _load_plan_document(supplied)
        query_dir = resolved_plan.parent
        query_id = query_dir.name
        run_dir = _find_run_dir(resolved_plan)
        manifest, cfg, scene, robot = load_run_inputs(run_dir)
        input_hashes = {
            "scene": scene_hash(scene),
            "robot": robot_hash(robot),
            "manifest_scene": manifest["scene_hash"],
            "manifest_robot": manifest["robot_hash"],
        }
        segs = []
        for s in plan["segments"]:
            if not isinstance(s, dict):
                raise ValueError("each trajectory segment must be an object")
            q0, q1 = s["q0"], s["q1"]
            controls = s.get("control_points", [])
            segs.append(PoseSegment(
                kind=SegmentKind[s["kind"]],
                q0=Pose2(np.array(q0[:2]), q0[2]),
                q1=Pose2(np.array(q1[:2]), q1[2]),
                control_points=tuple(Pose2(np.array(p[:2]), p[2])
                                     for p in controls),
                certificate_ids=tuple(s.get("certificate_ids", ())),
            ))
        expected_start, expected_goal = _load_query_endpoints(query_dir)
        oracles = candidate_pairs(scene, robot, scene.workspace)
        rep = verify_curve(oracles, scene.workspace, PoseCurve(tuple(segs)),
                           cfg.query.eps_clear, cfg.orientation.theta_min,
                           expected_start=expected_start,
                           expected_goal=expected_goal)
        # A modern query bundle carries a content-addressed proof object in
        # addition to the independently replayed geometry above.  Keep legacy
        # standalone paths verifiable, but never accept a present proof whose
        # digest, frozen-input binding, request, graph path, or segment IDs no
        # longer agree with the saved query artifacts.
        if rep.certified:
            proof_validation = validate_saved_query_proof(
                query_dir, scene, robot)
            if proof_validation["present"] and not proof_validation["valid"]:
                raise ValueError(
                    "query proof invalid: "
                    + ", ".join(proof_validation["errors"])
                )
        save_verify_result(query_dir, query_id, rep,
                           input_hashes=input_hashes,
                           proof_validation=proof_validation)
        print(f"certified={rep.certified} min_clearance={rep.min_clearance:.4f} "
              f"reason={rep.reason}")
        return 0 if rep.certified else 1
    except Exception as exc:
        save_verify_result(query_dir, query_id, error=exc,
                           input_hashes=input_hashes,
                           proof_validation=proof_validation)
        print(f"certified=False reason=verification_error: {exc}")
        return 1


def _strict_gate_payload_passes(gate) -> bool:
    return bool(
        type(gate) is dict
        and gate.get("declared") is True
        and gate.get("pass") is True
        and gate.get("errors") == []
    )


def _full_gate_report_passes(spec: dict, report: dict) -> bool:
    staged = spec.get("stages") is not None
    expected_kind = (
        "atlas_g1_full_gate_intervals_staged" if staged
        else "atlas_g1_full_gate_intervals"
    )
    if report.get("kind") != expected_kind \
            or not _strict_gate_payload_passes(report.get("gate")):
        return False

    declared_profile = spec.get("formal_profile")
    profile = report.get("profile_contract")
    if type(profile) is not dict:
        return False
    if declared_profile is None:
        if (profile.get("mode") != "diagnostic_only"
                or profile.get("profile") is not None
                or profile.get("validated") is not False):
            return False
    elif (profile.get("mode") != "formal"
          or profile.get("profile") != declared_profile
          or profile.get("validated") is not True
          or report["gate"].get("formal_profile") != declared_profile
          or report["gate"].get("formal_profile_validated") is not True):
        return False

    if staged:
        expected_stages = spec.get("stages")
        reported_stages = report.get("stages")
        if not isinstance(expected_stages, list) \
                or not isinstance(reported_stages, list):
            return False
        expected_names = [row.get("stage") for row in expected_stages]
        reported_names = [
            row.get("stage") for row in reported_stages
            if isinstance(row, dict)
        ]
        if (len(reported_names) != len(reported_stages)
                or reported_names != expected_names
                or report.get("stage_order") != expected_names
                or any(not _strict_gate_payload_passes(row.get("gate"))
                       for row in reported_stages)):
            return False
    return True


def cmd_benchmark(args):
    from .benchmarking.runner import run_benchmark_file
    spec = yaml.safe_load(Path(args.spec).resolve().read_text())
    if not isinstance(spec, dict):
        raise ValueError("benchmark spec must be a mapping")
    report = run_benchmark_file(args.spec, output=args.output,
                                allow_blind=args.allow_blind)
    if args.output:
        print(f"benchmark -> {Path(args.output).resolve()}")
        aggregate = report.get("aggregate")
        if aggregate is None:
            aggregate = report.get("aggregate_by_stage")
        if aggregate is None:
            raise ValueError("benchmark report has no aggregate summary")
        print(json.dumps(aggregate, indent=2))
    else:
        print(json.dumps(report, indent=2, sort_keys=True))
    if spec.get("kind") == "atlas_g1_full_gate_intervals":
        return 0 if _full_gate_report_passes(spec, report) else 1
    gate = report.get("gate")
    if gate is not None:
        if type(gate) is not dict or gate.get("pass") is not True:
            return 1
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="gmc")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("synth")
    p.add_argument("family", choices=sorted(synth.FAMILIES))
    p.add_argument("--width", type=float, default=None)
    p.add_argument("--robot", default="a=0.60 b=0.25")
    p.add_argument("-o", "--output", required=True)
    p.set_defaults(fn=cmd_synth)

    p = sub.add_parser("compile-slice")
    p.add_argument("scene")
    p.add_argument("--theta", type=float, required=True)
    p.add_argument("--config", default="configs/default.yaml")
    p.add_argument("--plot", default=None)
    p.set_defaults(fn=cmd_compile_slice)

    p = sub.add_parser("compile")
    p.add_argument("scene")
    p.add_argument("--config", default="configs/default.yaml")
    p.add_argument("-o", "--output", required=True)
    p.set_defaults(fn=cmd_compile)

    p = sub.add_parser("query")
    p.add_argument("run")
    p.add_argument("--start", required=True)
    p.add_argument("--goal", required=True)
    p.set_defaults(fn=cmd_query)

    p = sub.add_parser("verify")
    p.add_argument("plan")
    p.set_defaults(fn=cmd_verify)

    p = sub.add_parser("benchmark")
    p.add_argument("spec")
    p.add_argument("-o", "--output", default=None)
    p.add_argument("--allow-blind", action="store_true",
                   help="explicitly opt into the sealed Atlas blind split")
    p.set_defaults(fn=cmd_benchmark)

    import sys
    raw = list(sys.argv[1:]) if argv is None else list(argv)
    # argparse rejects negative-leading values after a space; the guide's
    # canonical invocation uses `--start "-2,0,1.57"` (§13.1), so fold such
    # pairs into --flag=value form.
    folded = []
    i = 0
    while i < len(raw):
        if raw[i] in ("--start", "--goal") and i + 1 < len(raw):
            folded.append(f"{raw[i]}={raw[i + 1]}")
            i += 2
        else:
            folded.append(raw[i])
            i += 1
    args = ap.parse_args(folded)
    return args.fn(args)


if __name__ == "__main__":
    raise SystemExit(main())
