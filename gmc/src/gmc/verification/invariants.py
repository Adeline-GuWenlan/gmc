"""Executable permanent invariants I1--I7 (Guide §1.3).

These checks are deliberately whole-compiler checks. A previous diagnostic
sampled one slice and ten pairs while presenting the result as I1--I7; that
was useful smoke coverage, but not an invariant gate.
"""
import json
from pathlib import Path

import numpy as np

from ..geometry.support import unit_dirs
from ..mobility.lineage import (candidate_links, certified_cover_slice,
                                component_slice, slab_has_safe_components)
from ..verification.continuous import rotation_interval_safe


def _numeric_le(left, right) -> bool:
    left = np.asarray(left, dtype=float)
    right = np.asarray(right, dtype=float)
    if (left.shape != right.shape or not np.all(np.isfinite(left))
            or not np.all(np.isfinite(right))):
        return False
    scale = np.maximum(1.0, np.maximum(np.abs(left), np.abs(right)))
    tol = 256.0 * np.finfo(float).eps * scale
    return bool(np.all(left <= right + tol))


def check_pair_nesting(sandwich, oracle, n_dirs: int = 64) -> bool:
    """I1: support(inner) <= support(true) <= support(outer).

    The audit directions include both an independent dense grid and every
    direction used to construct the sandwich. This remains a numerical
    executable invariant; theorem status itself comes from the stored sector
    certificate in :mod:`geometry.envelopes`.
    """
    if int(n_dirs) != n_dirs or n_dirs < 3:
        raise ValueError("n_dirs must be an integer >= 3")
    dense = np.linspace(0.0, 2.0 * np.pi, int(n_dirs), endpoint=False) + 0.0137
    angles = np.unique(np.concatenate([
        dense, np.asarray(sandwich.angles, dtype=float),
    ]))
    if not np.all(np.isfinite(angles)):
        return False
    U = unit_dirs(angles)
    h_true = oracle.support_values(sandwich.theta, U)
    try:
        vin = np.asarray(sandwich.inner.exterior.coords[:-1], dtype=float)
        vout = np.asarray(sandwich.outer.exterior.coords[:-1], dtype=float)
    except AttributeError:
        return False
    if (vin.ndim != 2 or vout.ndim != 2 or vin.shape[1:] != (2,)
            or vout.shape[1:] != (2,) or len(vin) < 3 or len(vout) < 3
            or not np.all(np.isfinite(vin)) or not np.all(np.isfinite(vout))):
        return False
    h_in = np.max(U @ vin.T, axis=1)
    h_out = np.max(U @ vout.T, axis=1)
    return _numeric_le(h_in, h_true) and _numeric_le(h_true, h_out)


def check_free_nesting(sl) -> bool:
    """I2: C_minus subset C_plus, hence F_safe subset F_possible."""
    try:
        return bool(sl.C_minus.difference(sl.C_plus).is_empty)
    except Exception:
        return False


def check_graph_nesting(mc) -> bool:
    """I3: every safe node/edge has its semantic possible counterpart."""
    if not _global_interval_cover_consistent(mc.decomposition):
        return False
    for _, data in mc.M_safe.nodes(data=True):
        slab_id, comp_id = data.get("slab"), data.get("comp")
        if not isinstance(slab_id, int) or not isinstance(comp_id, int):
            return False
        slab = mc.decomposition.slabs[slab_id]
        sl = component_slice(slab)
        if (not slab_has_safe_components(slab)
                or comp_id < 0 or comp_id >= len(sl.D_safe)):
            return False
        possible_node = mc.safe_to_possible.get((slab_id, comp_id))
        if possible_node not in mc.M_possible:
            return False
        possible_data = mc.M_possible.nodes[possible_node]
        possible_id = possible_data.get("comp")
        if (not isinstance(possible_id, int) or possible_id < 0
                or possible_id >= len(sl.D_possible)):
            return False
        possible_comp = sl.D_possible[possible_id]
        safe_comp = sl.D_safe[comp_id]
        if not safe_comp.geometry.difference(possible_comp.geometry).is_empty:
            return False
    for left, right in mc.M_safe.edges:
        left_data, right_data = mc.M_safe.nodes[left], mc.M_safe.nodes[right]
        possible_left = mc.safe_to_possible.get(
            (left_data["slab"], left_data["comp"]))
        possible_right = mc.safe_to_possible.get(
            (right_data["slab"], right_data["comp"]))
        if (possible_left is None or possible_right is None
                or not mc.M_possible.has_edge(possible_left, possible_right)):
            return False
    return True


def _parse_edge_witness(witness):
    if not isinstance(witness, tuple) or len(witness) != 3:
        return None
    anchors, theta0, theta1 = witness
    try:
        theta0, theta1 = float(theta0), float(theta1)
        anchors = tuple(np.asarray(anchor, dtype=float) for anchor in anchors)
    except (TypeError, ValueError):
        return None
    if (not anchors or not np.isfinite(theta0) or not np.isfinite(theta1)
            or any(anchor.shape != (2,) or not np.all(np.isfinite(anchor))
                   for anchor in anchors)):
        return None
    return anchors, theta0, theta1


def check_witness_ownership(mc, *, replay: bool = True) -> bool:
    """I4: each SAFE edge owns a well-formed, independently replayable arc."""
    for _, _, data in mc.M_safe.edges(data=True):
        if data.get("status") != "SAFE":
            return False
        parsed = _parse_edge_witness(data.get("witness"))
        if parsed is None:
            return False
        if replay:
            anchors, theta0, theta1 = parsed
            if not any(rotation_interval_safe(
                    mc.oracles, anchor, theta0, theta1,
                    mc.cfg.orientation.theta_min, floor=0.0,
            )[0] for anchor in anchors):
                return False
    return True


def check_no_silent_fallback(mc) -> bool:
    """I5 structural gate for the failure-as-POSSIBLE/UNKNOWN discipline."""
    if not _global_interval_cover_consistent(mc.decomposition):
        return False
    if any(not slab_has_safe_components(
            mc.decomposition.slabs[data["slab"]])
           for _, data in mc.M_safe.nodes(data=True)):
        return False
    if any(data.get("status") != "SAFE"
           for _, _, data in mc.M_safe.edges(data=True)):
        return False
    if any(data.get("status") not in {"SAFE", "POSSIBLE"}
           for _, _, data in mc.M_possible.edges(data=True)):
        return False
    return True


def _global_interval_cover_consistent(decomposition) -> bool:
    """A certified global cover must be backed by every slab cover.

    Event regularity and interval free-space coverage are independent proof
    obligations.  In particular an event-containing slab may still have a
    valid interval cover, while three regular midpoint samples do not prove a
    global possible-space cover.
    """
    status = getattr(decomposition, "global_possible_cover_status", None)
    if getattr(status, "name", None) != "CERTIFIED":
        return True
    slabs = tuple(getattr(decomposition, "slabs", ()))
    if not slabs:
        return False
    for slab in slabs:
        cover = certified_cover_slice(slab)
        if cover is None or not check_free_nesting(cover):
            return False
    return True


def _same_slice_geometry(left, right) -> bool:
    if not (left.C_plus.equals(right.C_plus)
            and left.C_minus.equals(right.C_minus)):
        return False
    if len(left.D_safe) != len(right.D_safe) or len(left.D_possible) != len(
            right.D_possible):
        return False
    return (all(a.geometry.equals(b.geometry)
                for a, b in zip(left.D_safe, right.D_safe))
            and all(a.geometry.equals(b.geometry)
                    for a, b in zip(left.D_possible, right.D_possible)))


def check_periodic_coverage(decomposition, mc=None) -> bool:
    """I6: interval tiling, endpoint slice identity, lineage, and graph glue."""
    slabs = sorted(decomposition.slabs, key=lambda slab: slab.interval.lo)
    if not slabs:
        return False
    cursor = 0.0
    tol = 64.0 * np.finfo(float).eps * (2.0 * np.pi)
    for slab in slabs:
        lo, hi = float(slab.interval.lo), float(slab.interval.hi)
        if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
            return False
        if abs(lo - cursor) > tol:
            return False
        cursor = hi
    if abs(cursor - 2.0 * np.pi) > tol:
        return False
    first_slice, last_slice = slabs[0].left_slice, slabs[-1].right_slice
    if not _same_slice_geometry(first_slice, last_slice):
        return False
    if mc is not None and len(slabs) > 1:
        from ..mobility.graph import node_id
        for link in candidate_links(slabs[-1], slabs[0], "possible"):
            left = node_id(link.slab_a, link.comp_a, "possible")
            right = node_id(link.slab_b, link.comp_b, "possible")
            if not mc.M_possible.has_edge(left, right):
                return False
    return True


def _validate_pair_pruning_replay(
        run_dir: Path, relative: str, _manifest, _cfg, scene, robot) -> bool:
    """Replay the declared all-pairs pruning table from frozen inputs."""
    if relative != "pairs/pruning.json":
        return False
    try:
        from ..reporting.proof_records import validate_pair_pruning_payload
        payload = json.loads((run_dir / relative).read_text())
        report = validate_pair_pruning_payload(
            payload, scene, robot, scene.workspace,
        )
    except (OSError, TypeError, ValueError, json.JSONDecodeError):
        return False
    return report.get("valid") is True


def _validate_fixed_slice_direction_replay(
        run_dir: Path, relative: str, _manifest, _cfg, _scene, _robot) -> bool:
    """Validate the declared direction index and every bound slice file."""
    if relative != "slices/direction_sets.json":
        return False
    try:
        from ..reporting.artifacts import \
            validate_fixed_slice_direction_artifact_tree
        report = validate_fixed_slice_direction_artifact_tree(run_dir)
    except (OSError, TypeError, ValueError, json.JSONDecodeError):
        return False
    return report.get("valid") is True


def _validate_formal_event_bracket_replay(
        _run_dir: Path, _relative: str, _manifest, _cfg, _scene,
        _robot) -> bool:
    """Fail closed until generic event brackets have a theorem validator.

    Finite left/mid/right predicate samples and the interval obstacle cover do
    not prove event exclusion.  Accepting an ad-hoc schema/profile here would
    turn those weaker records into a false full-I7 claim.
    """
    return False


def _validate_component_lineage_replay(
        _run_dir: Path, _relative: str, _manifest, _cfg, _scene,
        _robot) -> bool:
    """Fail closed until lineage is rebuilt independently from frozen inputs.

    ``validate_mobility_lineage_payload`` currently compares against an
    already-compiled in-memory graph.  The full-run checker has no independent
    compiler replay object yet, so structural JSON checks would not establish
    the claimed semantics.
    """
    return False


def _validate_mobility_witness_replay(
        _run_dir: Path, _relative: str, _manifest, _cfg, _scene,
        _robot) -> bool:
    """Fail closed until every SAFE witness has a full-run replay index."""
    return False


def _validate_query_proof_binding_replay(
        _run_dir: Path, _relative: str, _manifest, _cfg, _scene,
        _robot) -> bool:
    """Fail closed until all query verdicts have a full-run proof index."""
    return False


_FULL_REPLAY_ROLE_VALIDATORS = {
    "pair_pruning_bounds": _validate_pair_pruning_replay,
    "fixed_slice_direction_sets": _validate_fixed_slice_direction_replay,
    "formal_event_brackets": _validate_formal_event_bracket_replay,
    "component_lineage": _validate_component_lineage_replay,
    "mobility_witnesses": _validate_mobility_witness_replay,
    "query_proof_bindings": _validate_query_proof_binding_replay,
}


def check_manifest_reproducibility(manifest_or_path) -> bool:
    """I7: require an explicitly complete replay-evidence contract.

    Mere field presence is not I7.  In particular, a manifest that honestly
    records missing direction sets, pruning bounds, or formal event brackets
    must make this strict gate fail until those artifacts actually exist.
    """
    try:
        manifest_path = None
        if isinstance(manifest_or_path, (str, Path)):
            manifest_path = Path(manifest_or_path)
            if manifest_path.is_dir():
                manifest_path = manifest_path / "manifest.json"
            manifest = json.loads(manifest_path.read_text())
        else:
            manifest = manifest_or_path
        required = {
            "scene_hash", "robot_hash", "length_unit", "workspace",
            "scene_support_levels", "robot_support_levels", "config",
            "software", "artifact_scope", "numerical_validation",
            "i7_complete",
        }
        config_required = {
            "source", "eps_pair", "certificate_mode",
            "initial_directions", "max_directions", "initial_intervals",
            "theta_min", "max_depth", "max_support_calls",
            "max_wall_seconds", "eps_clear", "max_refinement_rounds",
            "workspace_precision",
        }
        if not (isinstance(manifest, dict) and required <= set(manifest)
                and isinstance(manifest["config"], dict)
                and config_required <= set(manifest["config"])
                and isinstance(manifest["artifact_scope"], dict)):
            return False
        scope = manifest["artifact_scope"]
        if (manifest["i7_complete"] is not True
                or scope.get("i7_complete") is not True
                or manifest_path is None):
            return False

        # A complete claim must survive independent input and interval-tree
        # replay.  Loading a dictionary alone cannot establish file evidence.
        run_dir = manifest_path.parent
        from ..reporting.artifacts import load_run_inputs
        from ..reporting.replay import (file_sha256,
                                        validate_interval_artifact_tree)
        frozen_manifest, cfg, scene, robot = load_run_inputs(run_dir)
        interval_report = validate_interval_artifact_tree(
            run_dir, manifest=manifest)
        if not interval_report["valid"]:
            return False

        # Full I7 remains a stronger contract than the interval-cover tranche.
        # Each proof family must name and hash an actual replay object; boolean
        # completion flags or empty placeholder files cannot satisfy the gate.
        contract = scope.get("full_replay_contract")
        roles = set(_FULL_REPLAY_ROLE_VALIDATORS)
        if (not isinstance(contract, dict)
                or contract.get("schema_version") != 1
                or contract.get("profile") != "gmc_i7_full_replay_v1"
                or contract.get("independent_replay_ready") is not True
                or not isinstance(contract.get("objects"), dict)
                or set(contract["objects"]) != roles):
            return False
        role_paths = {}
        for role in sorted(roles):
            record = contract["objects"][role]
            if (not isinstance(record, dict)
                    or record.get("status") != "complete"
                    or not isinstance(record.get("path"), str)
                    or not isinstance(record.get("sha256"), str)):
                return False
            path = (run_dir / record["path"]).resolve()
            try:
                path.relative_to(run_dir.resolve())
            except ValueError:
                return False
            if (not path.is_file() or path.stat().st_size == 0
                    or file_sha256(path) != record["sha256"]):
                return False
            role_paths[role] = (record["path"], path)

        # A proof family must bind its own replay object.  Aliases and symlink
        # spellings of one file cannot stand in for six independent objects.
        paths_are_distinct = len({
            path for _relative, path in role_paths.values()
        }) == len(roles)

        # Do not short-circuit this list: every role is dispatched to its own
        # semantic validator.  Roles whose independent replay contract is not
        # implemented deliberately return False above, keeping full I7
        # unreachable instead of promoting hashes or shallow JSON schemas.
        semantic_results = [
            _FULL_REPLAY_ROLE_VALIDATORS[role](
                run_dir, role_paths[role][0], frozen_manifest, cfg,
                scene, robot,
            )
            for role in sorted(roles)
        ]
        return paths_are_distinct and all(semantic_results)
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        return False


def _unique_slices(decomposition):
    seen = set()
    for slab in decomposition.slabs:
        for sl in (slab.left_slice, slab.mid_slice, slab.right_slice):
            marker = id(sl)
            if marker not in seen:
                seen.add(marker)
                yield sl


def check_all(mc, manifest=None, *, replay_witnesses: bool = True) -> dict:
    """Run all in-memory invariants; include I7 when a manifest is supplied."""
    oracle_by_id = {oracle.pair_id: oracle for oracle in mc.oracles}
    pair_nesting = True
    free_nesting = True
    for sl in _unique_slices(mc.decomposition):
        free_nesting = free_nesting and check_free_nesting(sl)
        for sandwich in sl.sandwiches:
            oracle = oracle_by_id.get(sandwich.pair_id)
            if oracle is None or not check_pair_nesting(sandwich, oracle):
                pair_nesting = False
                break
        if not pair_nesting:
            break
    result = {
        "I1_pair_nesting_all_slices": pair_nesting,
        "I2_free_nesting_all_slices": free_nesting,
        "I3_graph_nesting": check_graph_nesting(mc),
        "I4_witness_ownership": check_witness_ownership(
            mc, replay=replay_witnesses),
        "I5_no_silent_fallback": check_no_silent_fallback(mc),
        "I6_periodicity_and_glue": check_periodic_coverage(
            mc.decomposition, mc),
    }
    if manifest is not None:
        result["I7_artifact_reproducibility"] = \
            check_manifest_reproducibility(manifest)
    return result
