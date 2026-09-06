"""M5 adaptive orientation slab decomposition (Guide §8.1).

There are two independent notions of certification here:

* the finite left/mid/right predicates classify a slab as a regular candidate
  or as uncertain; and
* theorem-mode interval pair certificates construct a conservative free-space
  projection for *every* orientation in the slab.

The second object is sufficient for a global possible-space cover even when a
topology event may occur inside the interval.  Keeping the notions separate is
important: interval cover soundness must not be inferred from three samples,
and a hidden event must not discard an otherwise sound outer cover of truth.
"""
from dataclasses import dataclass, field, replace

import numpy as np
import shapely

from ..io.gs_io import validate_models
from ..types import CertStatus
from ..spatial.bvh import candidate_pairs, validate_candidate_oracles
from ..spatial.slice_compiler import (
    CertifiedSlice,
    PairSandwichSlice,
    assemble_slice,
    build_pair_sandwich_slice,
    build_slice,
    check_compilation_deadline,
)
from .event_detector import PredicateReport, collect_predicates
from .interval_certificate import (IntervalPairCertificate,
                                   build_interval_pair_certificate)
from .intervals import TWO_PI, Interval, initial_partition
from .provenance import (capture_decomposition_binding,
                         validate_decomposition_binding)


_INTERVAL_CERTIFICATE_FAILURES = (
    FloatingPointError,
    OverflowError,
    np.linalg.LinAlgError,
    shapely.GEOSException,
)

_ADAPTIVE_CONSTRUCTION = "adaptive_left_mid_right"
_CONNECTIVITY_CONSTRUCTION = "fixed_translation_connectivity"


@dataclass(frozen=True)
class _OrientationBoundarySample:
    """An angle-only endpoint owned by the connectivity-only compiler.

    Endpoint free-space geometry is deliberately absent: it is neither needed
    nor admissible evidence for an interval-wide connectivity verdict.
    """

    theta: float


@dataclass(frozen=True)
class Slab:
    interval: Interval
    status: CertStatus                       # EMPIRICALLY_VALIDATED | UNKNOWN
    kind: str                                # "regular" | "uncertain"
    left_slice: CertifiedSlice | PairSandwichSlice | _OrientationBoundarySample
    mid_slice: CertifiedSlice | PairSandwichSlice
    right_slice: CertifiedSlice | PairSandwichSlice | _OrientationBoundarySample
    predicates: PredicateReport
    slab_id: int = -1
    # A projection built from interval pair certificates, not a fixed-theta
    # sample.  D_safe is free for every theta in ``interval`` and D_possible
    # covers every truly free pose projected into translation space.
    cover_slice: CertifiedSlice | None = None
    cover_provenance: dict = field(default_factory=lambda: {
        "status": CertStatus.UNKNOWN.name,
        "reason": "interval_cover_not_built",
        "evidence_scope": "none",
    })
    # Depth in the binary orientation tree.  It is persisted so query-local
    # refinement obeys the same max-depth contract as the initial build.
    depth: int = 0


@dataclass
class SlabDecomposition:
    slabs: list
    slice_cache: dict = field(repr=False, default_factory=dict)
    n_slices: int = 0
    # A possible-graph cut proves global unreachability only if every
    # orientation interval is covered by a sound outer/free-space theorem.
    # Prototype mode has finite left/mid/right checks only and stays UNKNOWN;
    # theorem mode may replace this after the complete cover is assembled.
    global_possible_cover_status: CertStatus = CertStatus.UNKNOWN
    global_possible_cover_provenance: dict = field(default_factory=lambda: {
        "reason": "no_interval_or_global_possible_cover_certificate",
        "evidence_scope": "finite_left_mid_right_samples_only",
    })
    # A graph/decomposition revision advances exactly once per successful
    # query-local split.  Failed or no-progress attempts leave it unchanged.
    revision: int = 0
    refinement_history: list = field(default_factory=list)
    # Retain the P5 slice builder so later splits can populate only the two new
    # quarter-angle slices while reusing the decomposition's exact-angle cache.
    slice_builder: object = field(default=None, repr=False)
    ledger: object = field(default=None, repr=False)
    input_binding: object = field(default=None, repr=False)
    # Connectivity-only decompositions intentionally omit sampled regularity
    # geometry and must be refined by their matching transactional API.
    construction_mode: str = _ADAPTIVE_CONSTRUCTION


def _full_circle_tiling(slabs: list[Slab]) -> bool:
    """Whether the final intervals tile [0, 2*pi] without a gap/overlap."""
    if not slabs:
        return False
    ordered = sorted(slabs, key=lambda slab: slab.interval.lo)
    cursor = 0.0
    for slab in ordered:
        lo = float(slab.interval.lo)
        hi = float(slab.interval.hi)
        if (not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo
                or lo != cursor):
            return False
        cursor = hi
    return bool(cursor == TWO_PI)


def _build_interval_cover(scene, cfg, oracles, interval: Interval,
                          mid: CertifiedSlice | PairSandwichSlice, *,
                          ledger=None, deadline=None):
    """Build one fail-closed theorem projection over ``interval``.

    This is shared by the initial compiler and query-local splits.  Expected
    numerical/certificate failures remain UNKNOWN evidence; unexpected
    programming errors propagate to the query INTERNAL_ERROR gate.
    """
    check_compilation_deadline(deadline, "interval_cover_entry")
    base = {
        "status": CertStatus.UNKNOWN.name,
        "interval": [float(interval.lo), float(interval.hi)],
        "midpoint": float(interval.midpoint),
        "pair_count": len(mid.sandwiches),
        "certified_pair_count": 0,
    }
    if cfg.pair_approx.certificate_mode != "theorem":
        return None, {
            **base,
            "reason": "interval_cover_requires_theorem_mode",
            "evidence_scope": "finite_left_mid_right_samples_only",
        }
    if not all(s.status is CertStatus.CERTIFIED for s in mid.sandwiches):
        return None, {
            **base,
            "reason": "midpoint_pair_sandwich_not_certified",
            "evidence_scope": "theorem_midpoint_sandwiches_incomplete",
        }

    oracle_by_id = {oracle.pair_id: oracle for oracle in oracles}
    calls0 = sum(oracle.calls for oracle in oracles)
    certificates = []
    try:
        for sandwich in mid.sandwiches:
            check_compilation_deadline(
                deadline, "interval_cover_before_pair_certificate")
            oracle = oracle_by_id.get(sandwich.pair_id)
            if oracle is None:
                return None, {
                    **base,
                    "reason": "interval_pair_oracle_missing",
                    "evidence_scope": "interval_pair_certificate_incomplete",
                    "failed_pair": {
                        "scene_id": sandwich.pair_id.scene_id,
                        "body_id": sandwich.pair_id.body_id,
                    },
                }
            certificate = build_interval_pair_certificate(
                sandwich, oracle, interval,
                cfg.geometry.workspace_precision,
            )
            if (certificate.pair_id != sandwich.pair_id
                    or certificate.status is not CertStatus.CERTIFIED):
                return None, {
                    **base,
                    "reason": "interval_pair_certificate_not_certified",
                    "evidence_scope": "interval_pair_certificate_incomplete",
                    "failed_pair": {
                        "scene_id": sandwich.pair_id.scene_id,
                        "body_id": sandwich.pair_id.body_id,
                    },
                    "pair_status": certificate.status.name,
                }
            certificates.append(certificate)
            check_compilation_deadline(
                deadline, "interval_cover_after_pair_certificate")
    except _INTERVAL_CERTIFICATE_FAILURES as exc:
        return None, {
            **base,
            "reason": "interval_pair_certificate_failed",
            "evidence_scope": "interval_pair_certificate_incomplete",
            "error_type": type(exc).__name__,
            "error": str(exc),
            "certified_pair_count": len(certificates),
        }

    support_calls = sum(oracle.calls for oracle in oracles) - calls0
    try:
        check_compilation_deadline(deadline, "interval_cover_before_assembly")
        cover = assemble_slice(
            scene, interval.midpoint, cfg, certificates,
            support_calls=support_calls, ledger=ledger, deadline=deadline,
        )
        check_compilation_deadline(deadline, "interval_cover_after_assembly")
    except _INTERVAL_CERTIFICATE_FAILURES as exc:
        return None, {
            **base,
            "reason": "interval_cover_assembly_failed",
            "evidence_scope": "interval_pair_certificates_only",
            "error_type": type(exc).__name__,
            "error": str(exc),
            "certified_pair_count": len(certificates),
        }
    if cover.status is not CertStatus.CERTIFIED:
        return None, {
            **base,
            "reason": "interval_cover_projection_not_certified",
            "evidence_scope": "interval_pair_certificates_only",
            "certified_pair_count": len(certificates),
            "projection_status": cover.status.name,
        }
    return cover, {
        **base,
        "status": CertStatus.CERTIFIED.name,
        "reason": "all_interval_pair_certificates_assembled",
        "evidence_scope": "entire_orientation_interval",
        "certified_pair_count": len(certificates),
        "support_calls": int(support_calls),
    }


def _set_global_cover_status(decomposition: SlabDecomposition, cfg) -> None:
    """Recompute the aggregate theorem status from the actual slab objects."""
    slabs = decomposition.slabs
    full_circle = _full_circle_tiling(slabs)
    certified_cover_count = sum(
        int(s.cover_slice is not None
            and s.cover_slice.status is CertStatus.CERTIFIED
            and s.cover_provenance.get("status") == CertStatus.CERTIFIED.name)
        for s in slabs
    )
    theorem_mode = cfg.pair_approx.certificate_mode == "theorem"
    decomposition.global_possible_cover_status = (
        CertStatus.CERTIFIED
        if theorem_mode and full_circle and certified_cover_count == len(slabs)
        else CertStatus.UNKNOWN
    )
    decomposition.global_possible_cover_provenance = {
        "reason": (
            "all_orientation_intervals_have_certified_pair_covers"
            if decomposition.global_possible_cover_status is CertStatus.CERTIFIED
            else (
                "incomplete_orientation_interval_pair_cover"
                if theorem_mode
                else "no_interval_or_global_possible_cover_certificate"
            )
        ),
        "evidence_scope": (
            "certified_full_circle_interval_cover"
            if decomposition.global_possible_cover_status is CertStatus.CERTIFIED
            else (
                "incomplete_certified_interval_cover"
                if theorem_mode
                else "finite_left_mid_right_samples_only"
            )
        ),
        "slab_count": len(slabs),
        "certified_interval_cover_count": certified_cover_count,
        "event_free_interval_count": sum(
            int(s.predicates.certifies_no_event) for s in slabs),
        "full_circle_tiling": full_circle,
        "certificate_mode": cfg.pair_approx.certificate_mode,
        "uncertified_slab_ids": [
            slab.slab_id for slab in slabs
            if slab.cover_slice is None
            or slab.cover_slice.status is not CertStatus.CERTIFIED
        ],
        "revision": int(decomposition.revision),
    }


def decomposition_structure_failures(decomposition, cfg) -> tuple[str, ...]:
    """Validate that slabs, cached slices, covers, and aggregate proof agree."""
    failures = []
    slabs = tuple(getattr(decomposition, "slabs", ()))
    if not slabs:
        return ("empty_decomposition",)
    if [getattr(slab, "slab_id", None) for slab in slabs] != list(
            range(len(slabs))):
        failures.append("slab_id_index_alignment")
    if not _full_circle_tiling(list(slabs)):
        failures.append("orientation_interval_tiling")

    def same_angle(value, expected):
        value = float(value) % TWO_PI
        expected = float(expected) % TWO_PI
        return value == expected

    binding = getattr(decomposition, "input_binding", None)
    expected_pair_ids = tuple(
        record[0] for record in getattr(binding, "oracle_records", ())
    )
    ordered = sorted(slabs, key=lambda slab: slab.interval.lo)
    for position, slab in enumerate(ordered):
        if getattr(decomposition, "construction_mode", None) == \
                _CONNECTIVITY_CONSTRUCTION:
            if not isinstance(slab.mid_slice, PairSandwichSlice):
                failures.append(
                    f"slab_{slab.slab_id}_connectivity_midpoint_type")
            if (slab.status is not CertStatus.UNKNOWN
                    or slab.kind != "uncertain"
                    or slab.predicates.regular_candidate
                    or slab.predicates.certifies_no_event):
                failures.append(
                    f"slab_{slab.slab_id}_sampled_regularity_claim")
        if not same_angle(slab.left_slice.theta, slab.interval.lo):
            failures.append(f"slab_{slab.slab_id}_left_slice_binding")
        if not same_angle(slab.mid_slice.theta, slab.interval.midpoint):
            failures.append(f"slab_{slab.slab_id}_mid_slice_binding")
        if not same_angle(slab.right_slice.theta, slab.interval.hi):
            failures.append(f"slab_{slab.slab_id}_right_slice_binding")
        nxt = ordered[(position + 1) % len(ordered)]
        if slab.right_slice is not nxt.left_slice:
            failures.append(f"slab_{slab.slab_id}_endpoint_cache_binding")

        cover = slab.cover_slice
        provenance = slab.cover_provenance
        if cover is None:
            if provenance.get("status") == CertStatus.CERTIFIED.name:
                failures.append(f"slab_{slab.slab_id}_missing_certified_cover")
            continue
        if any(cover is sample for sample in (
                slab.left_slice, slab.mid_slice, slab.right_slice)):
            failures.append(f"slab_{slab.slab_id}_cover_aliases_sample")
        if not same_angle(cover.theta, slab.interval.midpoint):
            failures.append(f"slab_{slab.slab_id}_cover_theta_binding")
        certificates = tuple(getattr(cover, "sandwiches", ()))
        midpoint_pairs = tuple(
            sandwich.pair_id for sandwich in slab.mid_slice.sandwiches
        )
        if (midpoint_pairs != expected_pair_ids
                or len(midpoint_pairs) != len(set(midpoint_pairs))):
            failures.append(f"slab_{slab.slab_id}_midpoint_pair_binding")
        if not all(isinstance(certificate, IntervalPairCertificate)
                   for certificate in certificates):
            failures.append(f"slab_{slab.slab_id}_cover_certificate_type")
        if any(isinstance(certificate, IntervalPairCertificate)
               and certificate.interval != slab.interval
               for certificate in certificates):
            failures.append(f"slab_{slab.slab_id}_cover_certificate_interval")
        if any(isinstance(certificate, IntervalPairCertificate)
               and certificate.status is not CertStatus.CERTIFIED
               for certificate in certificates):
            failures.append(f"slab_{slab.slab_id}_cover_certificate_status")
        certificate_pairs = tuple(
            certificate.pair_id for certificate in certificates
            if isinstance(certificate, IntervalPairCertificate)
        )
        if (len(certificates) != len(midpoint_pairs)
                or certificate_pairs != midpoint_pairs
                or len(certificate_pairs) != len(set(certificate_pairs))):
            failures.append(f"slab_{slab.slab_id}_cover_pair_binding")
        inner_union = shapely.union_all([
            certificate.inner_common
            for certificate in certificates
            if isinstance(certificate, IntervalPairCertificate)
        ])
        outer_union = shapely.union_all([
            certificate.outer_cover
            for certificate in certificates
            if isinstance(certificate, IntervalPairCertificate)
        ])
        if not outer_union.difference(cover.C_plus).is_empty:
            failures.append(f"slab_{slab.slab_id}_cover_outer_union_binding")
        if not cover.C_minus.difference(inner_union).is_empty:
            failures.append(f"slab_{slab.slab_id}_cover_inner_union_binding")
        if any(component.status is not CertStatus.CERTIFIED
               for component in (*cover.D_safe, *cover.D_possible)):
            failures.append(f"slab_{slab.slab_id}_cover_component_status")
        workspace = getattr(getattr(binding, "scene", None), "workspace", None)
        if workspace is None:
            failures.append(f"slab_{slab.slab_id}_cover_workspace_binding")
        else:
            expected_safe = workspace.difference(cover.C_plus)
            safe_union = shapely.union_all([
                component.geometry for component in cover.D_safe
            ])
            # Safe extraction may conservatively drop tiny components, so only
            # the subset direction is required.  Extra component area would be
            # an unsound lower-bound expansion.
            if not safe_union.difference(expected_safe).is_empty:
                failures.append(
                    f"slab_{slab.slab_id}_safe_components_sound"
                )
            expected_possible = workspace.difference(cover.C_minus)
            possible_union = shapely.union_all([
                component.geometry for component in cover.D_possible
            ])
            if (not expected_possible.difference(possible_union).is_empty
                    or not possible_union.difference(expected_possible).is_empty):
                failures.append(
                    f"slab_{slab.slab_id}_possible_components_complete"
                )
        expected_interval = [
            float(slab.interval.lo), float(slab.interval.hi),
        ]
        if provenance.get("interval") != expected_interval:
            failures.append(f"slab_{slab.slab_id}_cover_interval_binding")
        if provenance.get("midpoint") != float(slab.interval.midpoint):
            failures.append(f"slab_{slab.slab_id}_cover_midpoint_binding")
        if (provenance.get("pair_count") != len(certificates)
                or provenance.get("certified_pair_count")
                != len(certificates)):
            failures.append(f"slab_{slab.slab_id}_cover_pair_count_binding")
        if (cover.status is CertStatus.CERTIFIED
                and (provenance.get("status") != CertStatus.CERTIFIED.name
                     or provenance.get("evidence_scope")
                     != "entire_orientation_interval")):
            failures.append(f"slab_{slab.slab_id}_cover_proof_binding")

    global_status = getattr(
        decomposition, "global_possible_cover_status", CertStatus.UNKNOWN,
    )
    if global_status is CertStatus.CERTIFIED:
        provenance = getattr(
            decomposition, "global_possible_cover_provenance", {},
        )
        if cfg.pair_approx.certificate_mode != "theorem":
            failures.append("prototype_global_cover_claim")
        if any(slab.cover_slice is None
               or slab.cover_slice.status is not CertStatus.CERTIFIED
               or slab.cover_provenance.get("status")
               != CertStatus.CERTIFIED.name
               for slab in slabs):
            failures.append("global_cover_missing_slab_proof")
        if (provenance.get("evidence_scope")
                != "certified_full_circle_interval_cover"):
            failures.append("global_cover_scope_binding")
        if provenance.get("slab_count") != len(slabs):
            failures.append("global_cover_count_binding")
        if provenance.get("full_circle_tiling") != True:
            failures.append("global_cover_tiling_binding")
        if provenance.get("certificate_mode") != "theorem":
            failures.append("global_cover_mode_binding")
        if provenance.get("revision", 0) != int(
                getattr(decomposition, "revision", 0)):
            failures.append("global_cover_revision_binding")
    return tuple(dict.fromkeys(failures))


def _connectivity_only_predicates(left, mid, right) -> PredicateReport:
    """Record that sampled topology was intentionally not evaluated.

    The interval pair certificates remain sufficient for the connectivity
    sandwich.  This placeholder is deliberately non-regular and cannot be
    consumed as evidence that the interval contains no topology event.
    """
    return PredicateReport(
        pair_set_constant=True,
        counts_match=False,
        lineage_bijective_safe=False,
        lineage_bijective_possible=False,
        details={
            "sample_thetas": (
                float(left.theta), float(mid.theta), float(right.theta),
            ),
            "regularity_evaluated": False,
            "evidence_scope": (
                "midpoint_pair_envelopes_and_interval_pair_certificates_only"
            ),
        },
        interval_certified=False,
        uncertainty_sources=(
            "left_mid_right_regularity_not_evaluated",
        ),
    )


def build_connectivity_slabs(scene, robot, cfg, oracles=None, *,
                             ledger=None, deadline=None) -> SlabDecomposition:
    """Build the minimal full-S1 decomposition for fixed-point connectivity.

    Every validated candidate pair is approximated at each configured initial
    interval's midpoint and lifted to an interval-wide theorem cover.  Unlike
    :func:`build_slabs`, this API neither assembles endpoint/midpoint fixed
    free spaces nor recursively splits on sampled regularity.  The configured
    initial interval count and all pair/certificate settings are unchanged.

    ``deadline`` is an absolute :func:`time.perf_counter` timestamp.  Expiry
    raises ``CompilationDeadlineExceeded`` before returning any partially
    built decomposition.
    """
    check_compilation_deadline(deadline, "connectivity_slabs_entry")
    validate_models(scene, robot, cfg)
    if oracles is None:
        oracles = candidate_pairs(scene, robot, scene.workspace)
    else:
        oracles = validate_candidate_oracles(
            scene, robot, scene.workspace, oracles)
    # Keep the provenance-bound CandidateOracleList.  Converting it to a
    # plain tuple here would force every midpoint build to reconstruct and
    # compare a fresh BVH candidate set merely to re-establish provenance.
    effective_ledger = (ledger if ledger is not None else next(
        (getattr(oracle, "ledger", None) for oracle in oracles
         if getattr(oracle, "ledger", None) is not None),
        None,
    ))
    decomposition = SlabDecomposition(
        slabs=[], ledger=effective_ledger,
        input_binding=capture_decomposition_binding(
            scene, robot, cfg, oracles,
        ),
        construction_mode=_CONNECTIVITY_CONSTRUCTION,
    )
    midpoint_cache = decomposition.slice_cache
    boundary_cache = {}

    def boundary_at(theta: float):
        key = float(theta) % TWO_PI
        if key not in boundary_cache:
            boundary_cache[key] = _OrientationBoundarySample(key)
        return boundary_cache[key]

    def midpoint_at(theta: float) -> PairSandwichSlice:
        key = float(theta) % TWO_PI
        if key not in midpoint_cache:
            check_compilation_deadline(
                deadline, "connectivity_slabs_before_midpoint")
            sample = build_pair_sandwich_slice(
                scene, robot, key, cfg, oracles,
                ledger=ledger, deadline=deadline,
            )
            check_compilation_deadline(
                deadline, "connectivity_slabs_after_midpoint")
            midpoint_cache[key] = sample
            decomposition.n_slices += 1
        return midpoint_cache[key]

    slabs = []
    for slab_id, interval in enumerate(initial_partition(
            cfg.orientation.initial_intervals)):
        check_compilation_deadline(
            deadline, "connectivity_slabs_before_interval")
        left = boundary_at(interval.lo)
        mid = midpoint_at(interval.midpoint)
        right = boundary_at(interval.hi % TWO_PI)
        cover_slice, cover_provenance = _build_interval_cover(
            scene, cfg, oracles, interval, mid,
            ledger=mid.ledger, deadline=deadline,
        )
        slabs.append(Slab(
            interval=interval,
            status=CertStatus.UNKNOWN,
            kind="uncertain",
            left_slice=left,
            mid_slice=mid,
            right_slice=right,
            predicates=_connectivity_only_predicates(left, mid, right),
            slab_id=slab_id,
            cover_slice=cover_slice,
            cover_provenance=cover_provenance,
            depth=0,
        ))
        check_compilation_deadline(
            deadline, "connectivity_slabs_after_interval")
    decomposition.slabs = slabs
    _set_global_cover_status(decomposition, cfg)
    check_compilation_deadline(deadline, "connectivity_slabs_complete")
    return decomposition


def refine_connectivity_slab(scene, robot, cfg, oracles,
                             decomposition: SlabDecomposition, slab_id: int,
                             *, ledger=None, deadline=None):
    """Transactionally bisect one connectivity-only interval-cover slab.

    Only the two new child midpoints receive pair envelopes and only the two
    child interval covers receive global arrangement assembly.  Parent samples,
    unchanged covers, and the input decomposition remain untouched if either
    child fails or the absolute deadline expires.
    """
    check_compilation_deadline(deadline, "connectivity_refinement_entry")
    validate_models(scene, robot, cfg)
    oracles = validate_candidate_oracles(
        scene, robot, scene.workspace, oracles,
    )
    validate_decomposition_binding(
        decomposition, scene, robot, cfg, oracles,
    )
    if getattr(decomposition, "construction_mode", None) != \
            _CONNECTIVITY_CONSTRUCTION:
        raise ValueError(
            "refine_connectivity_slab requires a connectivity-only "
            "decomposition"
        )
    try:
        slab_id = int(slab_id)
    except (TypeError, ValueError) as exc:
        raise ValueError("slab_id must be an integer") from exc
    matches = [slab for slab in decomposition.slabs
               if slab.slab_id == slab_id]
    if len(matches) != 1:
        raise ValueError(
            f"slab_id {slab_id} does not identify exactly one slab"
        )
    parent = matches[0]
    depth = int(getattr(parent, "depth", 0))
    interval = parent.interval
    if (interval.width <= cfg.orientation.theta_min
            or depth >= cfg.orientation.max_depth
            or not interval.lo < interval.midpoint < interval.hi):
        return decomposition, {
            "changed": False,
            "reason": "interval_not_refinable",
            "revision_before": int(getattr(decomposition, "revision", 0)),
            "revision_after": int(getattr(decomposition, "revision", 0)),
            "parent_slab_id": slab_id,
            "parent_interval": [float(interval.lo), float(interval.hi)],
            "parent_depth": depth,
        }

    cache = dict(decomposition.slice_cache)
    effective_ledger = (ledger if ledger is not None
                        else getattr(decomposition, "ledger", None))
    built = 0

    def midpoint_at(theta: float) -> PairSandwichSlice:
        nonlocal built
        key = float(theta) % TWO_PI
        if key not in cache:
            check_compilation_deadline(
                deadline, "connectivity_refinement_before_midpoint")
            sample = build_pair_sandwich_slice(
                scene, robot, key, cfg, oracles,
                ledger=effective_ledger, deadline=deadline,
            )
            check_compilation_deadline(
                deadline, "connectivity_refinement_after_midpoint")
            cache[key] = sample
            built += 1
        sample = cache[key]
        if not isinstance(sample, PairSandwichSlice):
            raise ValueError(
                "connectivity midpoint cache contains a full or invalid slice"
            )
        return sample

    def make_child(child_interval: Interval, left, right) -> Slab:
        mid = midpoint_at(child_interval.midpoint)
        cover_slice, cover_provenance = _build_interval_cover(
            scene, cfg, oracles, child_interval, mid,
            ledger=effective_ledger, deadline=deadline,
        )
        return Slab(
            interval=child_interval,
            status=CertStatus.UNKNOWN,
            kind="uncertain",
            left_slice=left,
            mid_slice=mid,
            right_slice=right,
            predicates=_connectivity_only_predicates(left, mid, right),
            cover_slice=cover_slice,
            cover_provenance=cover_provenance,
            depth=depth + 1,
        )

    left_child = make_child(
        interval.left_half, parent.left_slice, parent.mid_slice,
    )
    check_compilation_deadline(
        deadline, "connectivity_refinement_between_children")
    right_child = make_child(
        interval.right_half, parent.mid_slice, parent.right_slice,
    )
    check_compilation_deadline(
        deadline, "connectivity_refinement_before_commit")
    if (not left_child.interval.width < interval.width
            or not right_child.interval.width < interval.width):
        raise RuntimeError("binary slab split made no interval progress")

    raw = []
    for slab in sorted(decomposition.slabs,
                       key=lambda item: item.interval.lo):
        raw.extend((left_child, right_child) if slab is parent else (slab,))
    slabs = [replace(slab, slab_id=index)
             for index, slab in enumerate(raw)]
    previous_revision = int(getattr(decomposition, "revision", 0))
    revision = previous_revision + 1
    child_ids = [
        slab.slab_id for slab in slabs
        if (slab.interval == left_child.interval
            or slab.interval == right_child.interval)
    ]
    record = {
        "changed": True,
        "reason": "binary_interval_split",
        "revision_before": previous_revision,
        "revision_after": revision,
        "parent_slab_id": slab_id,
        "parent_interval": [float(interval.lo), float(interval.hi)],
        "parent_depth": depth,
        "child_slab_ids": child_ids,
        "child_intervals": [
            [float(left_child.interval.lo), float(left_child.interval.hi)],
            [float(right_child.interval.lo), float(right_child.interval.hi)],
        ],
        "child_depth": depth + 1,
        "new_slices_built": built,
        "slice_cache_entries": len(cache),
        "construction_mode": _CONNECTIVITY_CONSTRUCTION,
    }
    refined = SlabDecomposition(
        slabs=slabs,
        slice_cache=cache,
        n_slices=int(getattr(decomposition, "n_slices", 0)) + built,
        revision=revision,
        refinement_history=[
            *list(getattr(decomposition, "refinement_history", ())),
            record,
        ],
        ledger=effective_ledger,
        input_binding=decomposition.input_binding,
        construction_mode=_CONNECTIVITY_CONSTRUCTION,
    )
    _set_global_cover_status(refined, cfg)
    check_compilation_deadline(
        deadline, "connectivity_refinement_complete")
    return refined, record


def build_slabs(scene, robot, cfg, oracles=None,
                slice_builder=None, ledger=None) -> SlabDecomposition:
    """Build the adaptive periodic decomposition.

    ``slice_builder`` is the P5 extension point for a revisioned pair-envelope
    cache.  Derived slabs are always rebuilt; only the independent pair work
    inside each fixed-orientation slice may be reused.
    """
    validate_models(scene, robot, cfg)
    if oracles is None:
        oracles = candidate_pairs(scene, robot, scene.workspace)
    else:
        oracles = validate_candidate_oracles(
            scene, robot, scene.workspace, oracles)
    dec = SlabDecomposition(
        slabs=[], slice_builder=slice_builder, ledger=ledger,
        input_binding=capture_decomposition_binding(
            scene, robot, cfg, oracles,
        ),
    )

    def slice_at(theta: float) -> CertifiedSlice:
        # Exact canonical angle: scale-independent cache semantics.  Decimal
        # rounding can merge distinct slices whose geometry differs greatly
        # for a body primitive with a large local offset.
        key = float(theta) % TWO_PI
        if key not in dec.slice_cache:
            dec.slice_cache[key] = (
                slice_builder(key) if slice_builder is not None
                else build_slice(scene, robot, key, cfg, oracles,
                                 ledger=ledger)
            )
            dec.n_slices += 1
        return dec.slice_cache[key]

    def refine(I: Interval, left, right, depth: int) -> list:
        mid = slice_at(I.midpoint)
        pred = collect_predicates(left, mid, right, ledger=ledger)
        if pred.regular_candidate:
            return [Slab(I, CertStatus.EMPIRICALLY_VALIDATED, "regular",
                         left, mid, right, pred, depth=depth)]
        if I.width <= cfg.orientation.theta_min or depth >= cfg.orientation.max_depth:
            return [Slab(I, CertStatus.UNKNOWN, "uncertain",
                         left, mid, right, pred, depth=depth)]
        return (refine(I.left_half, left, mid, depth + 1)
                + refine(I.right_half, mid, right, depth + 1))

    slabs = []
    for I in initial_partition(cfg.orientation.initial_intervals):
        slabs += refine(I, slice_at(I.lo), slice_at(I.hi % TWO_PI), 0)
    for k, s in enumerate(slabs):
        cover_slice, cover_provenance = _build_interval_cover(
            scene, cfg, oracles, s.interval, s.mid_slice, ledger=ledger,
        )
        slabs[k] = Slab(
            s.interval, s.status, s.kind, s.left_slice, s.mid_slice,
            s.right_slice, s.predicates, slab_id=k,
            cover_slice=cover_slice, cover_provenance=cover_provenance,
            depth=s.depth,
        )
    dec.slabs = slabs
    _set_global_cover_status(dec, cfg)
    return dec


def refine_slab(scene, robot, cfg, oracles,
                decomposition: SlabDecomposition, slab_id: int,
                *, ledger=None):
    """Deterministically bisect one existing slab and reuse its slice cache.

    The returned decomposition is a new graph revision.  The input
    decomposition and its graph remain usable until the caller has rebuilt and
    validated a replacement mobility graph, which gives query refinement
    commit-on-success semantics.  Only the two new quarter-angle slices are
    compiled; parent endpoints and midpoint are reused by object identity.

    Returns ``(decomposition, record)``.  A non-refinable interval returns the
    original object with ``record["changed"] == False`` and never advances the
    revision.
    """
    validate_models(scene, robot, cfg)
    oracles = validate_candidate_oracles(
        scene, robot, scene.workspace, oracles,
    )
    validate_decomposition_binding(
        decomposition, scene, robot, cfg, oracles,
    )
    try:
        slab_id = int(slab_id)
    except (TypeError, ValueError) as exc:
        raise ValueError("slab_id must be an integer") from exc
    matches = [slab for slab in decomposition.slabs
               if slab.slab_id == slab_id]
    if len(matches) != 1:
        raise ValueError(
            f"slab_id {slab_id} does not identify exactly one slab"
        )
    parent = matches[0]
    depth = int(getattr(parent, "depth", 0))
    interval = parent.interval
    if (interval.width <= cfg.orientation.theta_min
            or depth >= cfg.orientation.max_depth
            or not interval.lo < interval.midpoint < interval.hi):
        return decomposition, {
            "changed": False,
            "reason": "interval_not_refinable",
            "revision_before": int(getattr(decomposition, "revision", 0)),
            "revision_after": int(getattr(decomposition, "revision", 0)),
            "parent_slab_id": slab_id,
            "parent_interval": [float(interval.lo), float(interval.hi)],
            "parent_depth": depth,
        }

    # Work on a shallow cache copy.  Slice objects are immutable and can be
    # reused by identity, but inserting newly built quarter slices into the
    # caller's dictionary before both children and all certificates succeed
    # would violate the advertised commit-on-success semantics.  In
    # particular, a hard support-budget exception after the first child must
    # leave the original decomposition byte-for-byte usable.
    cache = dict(decomposition.slice_cache)
    builder = getattr(decomposition, "slice_builder", None)
    effective_ledger = (ledger if ledger is not None
                        else getattr(decomposition, "ledger", None))
    built = 0

    def slice_at(theta: float) -> CertifiedSlice:
        nonlocal built
        key = float(theta) % TWO_PI
        if key not in cache:
            cache[key] = (
                builder(key) if builder is not None
                else build_slice(
                    scene, robot, key, cfg, oracles,
                    ledger=effective_ledger,
                )
            )
            built += 1
        return cache[key]

    def make_child(child_interval: Interval, left, right) -> Slab:
        mid = slice_at(child_interval.midpoint)
        predicates = collect_predicates(
            left, mid, right, ledger=effective_ledger,
        )
        regular = predicates.regular_candidate
        cover_slice, cover_provenance = _build_interval_cover(
            scene, cfg, oracles, child_interval, mid,
            ledger=effective_ledger,
        )
        return Slab(
            interval=child_interval,
            status=(CertStatus.EMPIRICALLY_VALIDATED
                    if regular else CertStatus.UNKNOWN),
            kind="regular" if regular else "uncertain",
            left_slice=left,
            mid_slice=mid,
            right_slice=right,
            predicates=predicates,
            cover_slice=cover_slice,
            cover_provenance=cover_provenance,
            depth=depth + 1,
        )

    left_child = make_child(
        interval.left_half, parent.left_slice, parent.mid_slice,
    )
    right_child = make_child(
        interval.right_half, parent.mid_slice, parent.right_slice,
    )
    if (not left_child.interval.width < interval.width
            or not right_child.interval.width < interval.width):
        raise RuntimeError("binary slab split made no interval progress")

    raw = []
    for slab in sorted(decomposition.slabs,
                       key=lambda item: item.interval.lo):
        raw.extend((left_child, right_child) if slab is parent else (slab,))
    slabs = [replace(slab, slab_id=k) for k, slab in enumerate(raw)]
    previous_revision = int(getattr(decomposition, "revision", 0))
    revision = previous_revision + 1
    child_ids = [
        slab.slab_id for slab in slabs
        if (slab.interval == left_child.interval
            or slab.interval == right_child.interval)
    ]
    record = {
        "changed": True,
        "reason": "binary_interval_split",
        "revision_before": previous_revision,
        "revision_after": revision,
        "parent_slab_id": slab_id,
        "parent_interval": [float(interval.lo), float(interval.hi)],
        "parent_depth": depth,
        "child_slab_ids": child_ids,
        "child_intervals": [
            [float(left_child.interval.lo), float(left_child.interval.hi)],
            [float(right_child.interval.lo), float(right_child.interval.hi)],
        ],
        "child_depth": depth + 1,
        "new_slices_built": built,
        "slice_cache_entries": len(cache),
    }
    refined = SlabDecomposition(
        slabs=slabs,
        slice_cache=cache,
        n_slices=int(getattr(decomposition, "n_slices", 0)) + built,
        revision=revision,
        refinement_history=[
            *list(getattr(decomposition, "refinement_history", ())),
            record,
        ],
        slice_builder=builder,
        ledger=effective_ledger,
        input_binding=decomposition.input_binding,
    )
    _set_global_cover_status(refined, cfg)
    return refined, record


def adjacent_slab_pairs(slabs, periodic: bool = True):
    """Consecutive slabs in angle order, including the 2*pi -> 0 wrap (I6)."""
    order = sorted(range(len(slabs)), key=lambda k: slabs[k].interval.lo)
    pairs = [(slabs[order[i]], slabs[order[i + 1]])
             for i in range(len(order) - 1)]
    if periodic and len(order) > 1:
        pairs.append((slabs[order[-1]], slabs[order[0]]))
    return pairs
