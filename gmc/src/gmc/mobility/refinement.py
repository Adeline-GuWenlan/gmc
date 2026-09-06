"""Atomic query-local orientation refinement.

One successful operation bisects exactly one slab, rebuilds the derived
mobility graphs, validates their structural invariants, and only then commits
the new revision to the caller's :class:`MobilityCompiler` object.
"""
from dataclasses import dataclass

from ..orientation.slab_builder import (
    adjacent_slab_pairs,
    decomposition_structure_failures,
    refine_slab,
)
from ..orientation.provenance import decomposition_binding_failures
from ..verification.invariants import (
    check_graph_nesting,
    check_no_silent_fallback,
    check_periodic_coverage,
    check_witness_ownership,
)
from .graph import compile_mobility, node_id
from .lineage import certified_cover_slice, component_slice


class RefinementInvariantError(RuntimeError):
    """A rebuilt graph contradicted the compiler's structural contract."""

    def __init__(self, failures):
        self.failures = tuple(failures)
        super().__init__(
            "refined mobility graph failed invariants: "
            + ", ".join(self.failures)
        )


@dataclass(frozen=True)
class RefinementOutcome:
    changed: bool
    record: dict


def structural_invariant_failures(mc) -> tuple[str, ...]:
    """Cheap I3--I6/query structure checks that require no support calls."""
    failures = list(decomposition_binding_failures(
        mc.decomposition, mc.scene, mc.robot, mc.cfg, mc.oracles,
    ))
    failures.extend(decomposition_structure_failures(
        mc.decomposition, mc.cfg,
    ))
    slabs = tuple(getattr(mc.decomposition, "slabs", ()))
    if [getattr(slab, "slab_id", None) for slab in slabs] != list(
            range(len(slabs))):
        failures.append("slab_id_index_alignment")
    if int(getattr(mc, "graph_revision", -1)) != int(
            getattr(mc.decomposition, "revision", 0)):
        failures.append("graph_decomposition_revision")

    # A formal upper-graph cut is only as complete as its nodes and every
    # adjacent-slab lineage.  Check these directly instead of assuming that a
    # graph produced earlier is still bound to the decomposition.
    possible_nodes_complete = True
    for slab in slabs:
        for component in range(len(component_slice(slab).D_possible)):
            expected = node_id(slab.slab_id, component, "possible")
            if expected not in mc.M_possible:
                possible_nodes_complete = False
                break
        if not possible_nodes_complete:
            break
    if not possible_nodes_complete:
        failures.append("possible_node_completeness")

    possible_adjacency_complete = True
    for left, right in adjacent_slab_pairs(list(slabs), periodic=True):
        if (certified_cover_slice(left) is None
                or certified_cover_slice(right) is None):
            continue
        left_components = component_slice(left).D_possible
        right_components = component_slice(right).D_possible
        for left_index, left_component in enumerate(left_components):
            for right_index, right_component in enumerate(right_components):
                # This gate deliberately does not reuse candidate_links: an
                # omission in lineage generation must not validate itself.
                if left_component.geometry.intersection(
                        right_component.geometry).is_empty:
                    continue
                a = node_id(left.slab_id, left_index, "possible")
                b = node_id(right.slab_id, right_index, "possible")
                if not mc.M_possible.has_edge(a, b):
                    possible_adjacency_complete = False
                    break
            if not possible_adjacency_complete:
                break
        if not possible_adjacency_complete:
            break
    if not possible_adjacency_complete:
        failures.append("possible_adjacency_completeness")

    checks = (
        ("I3_graph_nesting", check_graph_nesting),
        ("I4_witness_ownership", lambda value: check_witness_ownership(
            value, replay=False,
        )),
        ("I5_no_silent_fallback", check_no_silent_fallback),
        ("I6_periodicity_and_glue", lambda value: check_periodic_coverage(
            value.decomposition, value,
        )),
    )
    for name, check in checks:
        if not check(mc):
            failures.append(name)
    return tuple(dict.fromkeys(failures))


def refine_compiler(mc, slab_id: int) -> RefinementOutcome:
    """Bisect one slab and atomically install the validated graph revision."""
    refined, record = refine_slab(
        mc.scene, mc.robot, mc.cfg, mc.oracles, mc.decomposition, slab_id,
    )
    if not record["changed"]:
        return RefinementOutcome(False, record)

    rebuilt = compile_mobility(
        mc.scene, mc.robot, mc.cfg, mc.oracles, refined,
        ledger=getattr(refined, "ledger", None),
        validate_oracles=False,
    )
    failures = structural_invariant_failures(rebuilt)
    if failures:
        raise RefinementInvariantError(failures)

    # Commit only after every derived object and structural gate succeeds.
    mc.decomposition = rebuilt.decomposition
    mc.M_safe = rebuilt.M_safe
    mc.M_possible = rebuilt.M_possible
    mc.safe_to_possible = rebuilt.safe_to_possible
    mc.lineage_records = rebuilt.lineage_records
    mc.graph_revision = rebuilt.graph_revision
    mc.structural_contract = True
    return RefinementOutcome(True, record)
