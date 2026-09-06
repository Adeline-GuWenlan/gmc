"""Certified orientation intervals for fixed-translation connectivity.

For two fixed translations ``a`` and ``b`` let ``T(theta)`` mean that they
belong to the same connected component of the exact free translation space
at orientation ``theta``.  A theorem-mode :class:`~gmc.orientation.slab_builder.Slab`
contains two interval-wide translation-space bounds::

    D_safe(I)  subset  F(theta)  subset  D_possible(I),  theta in I.

Consequently, one component of ``D_safe`` that *strictly contains* both
translations certifies ``T`` on all of ``I``.  Conversely, if the translations
cannot be joined even in ``D_possible`` (with boundary-touching components
joined through their closures), ``not T`` is certified on all of ``I``.
Everything else remains UNKNOWN and may be bisected deterministically.

The returned angular sandwich has the paper-facing semantics

    theta_safe  subset  {theta: T(theta)}  subset  theta_possible.

UNKNOWN intervals are deliberately retained in ``theta_possible``.  Opposite
certified verdicts on the two sides of an unresolved run imply at least one
truth-value transition in that run; they never identify or count an isolated
event.
"""

from dataclasses import dataclass, field, replace
from enum import Enum
from numbers import Integral
import time
from contextlib import contextmanager

import numpy as np
import shapely

from ..budget import BudgetExceeded
from ..spatial.bvh import validate_candidate_oracles
from ..spatial.slice_compiler import CompilationDeadlineExceeded
from ..types import CertStatus
from .intervals import TWO_PI, Interval
from .slab_builder import (
    decomposition_structure_failures,
    refine_connectivity_slab,
    refine_slab,
)


class ConnectivityVerdict(str, Enum):
    """Three-valued verdict over a complete orientation interval."""

    OPEN = "OPEN"
    CLOSED = "CLOSED"
    UNKNOWN = "UNKNOWN"


# Paper/design spelling retained as a public alias.  ``Verdict`` is used by
# the compact API below; both names have identical enum identity.
ConnectivityValue = ConnectivityVerdict


class ConnectivityIntervalInvariantError(RuntimeError):
    """An internal contradiction in the certified lower/upper bounds."""

    def __init__(self, failures):
        self.failures = tuple(failures)
        super().__init__(
            "connectivity interval invariant failed: "
            + ", ".join(self.failures)
        )


@dataclass(frozen=True)
class FixedTranslationConnectivityQuery:
    """The two fixed translations whose slice connectivity is certified."""

    start_xy: np.ndarray
    goal_xy: np.ndarray
    query_id: str = "fixed_translation_connectivity"

    def __post_init__(self):
        start = _xy(self.start_xy, "start_xy")
        goal = _xy(self.goal_xy, "goal_xy")
        start = np.array(start, copy=True)
        goal = np.array(goal, copy=True)
        start.flags.writeable = False
        goal.flags.writeable = False
        object.__setattr__(self, "start_xy", start)
        object.__setattr__(self, "goal_xy", goal)
        if not isinstance(self.query_id, str) or not self.query_id:
            raise ValueError("query_id must be a nonempty string")


@dataclass(frozen=True)
class CyclicInterval:
    """One unwrapped representation of a connected subset of ``S1``."""

    lo: float
    hi: float
    period: float = TWO_PI

    def __post_init__(self):
        values = np.asarray([self.lo, self.hi, self.period], dtype=float)
        if (not np.all(np.isfinite(values)) or self.period <= 0.0
                or self.hi <= self.lo or self.hi - self.lo > self.period):
            raise ValueError("invalid cyclic interval")

    @property
    def width(self) -> float:
        return float(self.hi - self.lo)

    @property
    def wraps(self) -> bool:
        return bool(self.hi > self.period)


@dataclass(frozen=True)
class IntervalConnectivityCertificate:
    """Replay-oriented leaf certificate for one fixed-translation query."""

    interval: Interval
    value: ConnectivityValue
    status: CertStatus
    slab_id: int
    decomposition_revision: int
    lower_connected: bool
    upper_connected: bool
    safe_component_id: str | None
    possible_start_components: tuple[str, ...]
    possible_goal_components: tuple[str, ...]
    evidence_scope: str = "fixed_translation_connectivity_only"
    implies_event_free_slab: bool = False
    uncertainty_sources: tuple[str, ...] = ()
    reason: str = ""
    depth: int = 0
    diagnostics: dict = field(default_factory=dict)


@dataclass(frozen=True)
class ConnectivityTransitionBracket:
    """At-least-one transition bracket; never an isolated event claim."""

    interval: CyclicInterval
    left_value: ConnectivityValue
    right_value: ConnectivityValue
    status: CertStatus = CertStatus.CERTIFIED
    multiplicity_lower_bound: int = 1
    isolated: bool = False


@dataclass(frozen=True)
class GateAngularCertificate:
    """Full-circle paper-facing fixed-translation connectivity certificate."""

    query: FixedTranslationConnectivityQuery
    period: float
    theta_safe: tuple[CyclicInterval, ...]
    theta_possible: tuple[CyclicInterval, ...]
    theta_unknown: tuple[CyclicInterval, ...]
    transition_brackets: tuple[ConnectivityTransitionBracket, ...]
    unresolved_regions: tuple[CyclicInterval, ...]
    leaf_certificates: tuple[IntervalConnectivityCertificate, ...]
    resolution_met: bool
    stop_reason: str
    refinement_trace: tuple[dict, ...]
    support_calls: int
    max_support_calls: int
    wall_seconds: float
    max_wall_seconds: float
    sweep_report: object = field(repr=False, compare=False)

    @property
    def semantics(self) -> str:
        return "theta_safe subset true_connectivity subset theta_possible"


@dataclass(frozen=True)
class ConnectivityInterval:
    """One slab's certified connectivity classification."""

    interval: Interval
    verdict: ConnectivityVerdict
    slab_id: int
    depth: int
    reason: str
    diagnostics: dict = field(default_factory=dict)


@dataclass(frozen=True)
class UnresolvedInterval:
    """A maximal periodic run that exhausted the selected resolution."""

    interval: Interval
    reasons: tuple[str, ...]
    slab_ids: tuple[int, ...]


@dataclass(frozen=True)
class TransitionBracket:
    """An unresolved run bracketed by opposite certified verdicts.

    ``minimum_transitions == 1`` is only an existence statement.  In
    particular, neither the midpoint nor any other point in the bracket is an
    isolated event estimate, and multiple transitions may lie inside.
    """

    interval: Interval
    left_verdict: ConnectivityVerdict
    right_verdict: ConnectivityVerdict
    minimum_transitions: int = 1
    isolated: bool = False


@dataclass(frozen=True)
class ConnectivityIntervalReport:
    """Certified lower/upper angular sandwich and adaptive-sweep provenance."""

    period: float
    start_xy: tuple[float, float]
    goal_xy: tuple[float, float]
    intervals: tuple[ConnectivityInterval, ...]
    theta_safe: tuple[Interval, ...]
    theta_possible: tuple[Interval, ...]
    unresolved: tuple[UnresolvedInterval, ...]
    transition_brackets: tuple[TransitionBracket, ...]
    refinement_trace: tuple[dict, ...]
    initial_revision: int
    final_revision: int
    stop_reason: str
    decomposition: object = field(repr=False, compare=False)
    support_calls: int = 0
    max_support_calls: int | None = None
    wall_seconds: float = 0.0
    max_wall_seconds: float | None = None
    resolution_met: bool = False
    event_tolerance: float | None = None
    semantics: str = (
        "theta_safe subset true_connectivity subset theta_possible"
    )

    @property
    def open_intervals(self) -> tuple[Interval, ...]:
        return self.theta_safe

    @property
    def possible_intervals(self) -> tuple[Interval, ...]:
        return self.theta_possible

    @property
    def n_refinements(self) -> int:
        return len(self.refinement_trace)


def _xy(value, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (2,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite length-two translation")
    return result


def _compiler_input_failures(scene, robot, cfg, oracles, decomposition):
    """Cross-check ownership and structure before consuming a cover theorem."""
    from .provenance import decomposition_binding_failures

    failures = list(decomposition_binding_failures(
        decomposition, scene, robot, cfg, tuple(oracles),
    ))
    failures.extend(decomposition_structure_failures(decomposition, cfg))
    return tuple(dict.fromkeys(failures))


@dataclass
class _SupportMeter:
    limit: int
    used: int = 0

    def charge(self, kind: str, amount: int = 1) -> None:
        if kind not in ("support_value_evals", "support_point_evals"):
            return
        amount = int(amount)
        if self.used + amount > self.limit:
            raise BudgetExceeded(
                "connectivity_sweep_support_evals",
                self.limit, self.used, amount,
            )
        self.used += amount


class _MeteredLedger:
    """Hard support cap in front of one oracle's existing ledger."""

    def __init__(self, meter: _SupportMeter, delegate):
        self.meter = meter
        self.delegate = delegate

    def charge(self, kind: str, amount: int = 1, *, phase=None) -> None:
        # PairOracle charges before evaluating a direction batch, so a rejected
        # batch cannot increment ``oracle.calls`` or perform hidden work.
        self.meter.charge(kind, amount)
        if self.delegate is not None:
            if phase is None:
                self.delegate.charge(kind, amount)
            else:
                self.delegate.charge(kind, amount, phase=phase)


def _budget_stop_reason(exc: BudgetExceeded) -> str:
    """Preserve the exhausted work currency in the query stop reason.

    WorkLedger support-value and support-point exhaustion share the established
    public spelling.  Every other currency remains distinguishable so callers
    cannot mistake (for example) an overlay or sweep-local budget failure for
    ledger support exhaustion.
    """
    kind = str(exc.kind)
    if kind in {"support_value_evals", "support_point_evals"}:
        return "support_budget_exhausted"
    return f"{kind}_budget_exhausted"


def _is_budget_stop_reason(reason: str | None) -> bool:
    return isinstance(reason, str) and reason.endswith("_budget_exhausted")


@contextmanager
def _bounded_refinement_support(oracles, decomposition, remaining: int):
    """Temporarily hard-meter support batches without losing run accounting.

    ``refine_slab`` normally forwards ``decomposition.ledger`` to every
    oracle, which would overwrite per-oracle budget proxies.  Temporarily
    clearing that field lets each oracle retain its own forwarding proxy; both
    it and every original ledger are restored before returning.
    """
    meter = _SupportMeter(int(remaining))
    originals = [(oracle, getattr(oracle, "ledger", None))
                 for oracle in oracles]
    persistent_ledger = getattr(decomposition, "ledger", None)
    original_ledgers = [original for _, original in originals]
    one_delegate = bool(original_ledgers) and all(
        original is original_ledgers[0] for original in original_ledgers
    )
    shared_delegate = (
        persistent_ledger if persistent_ledger is not None
        else original_ledgers[0] if one_delegate else None
    )
    effective_ledger = None
    if persistent_ledger is not None or one_delegate:
        effective_ledger = _MeteredLedger(meter, shared_delegate)
        for oracle, _ in originals:
            oracle.ledger = effective_ledger
    else:
        # Preserve unusual independently-ledgered oracle sets.  Fixed-slice
        # support work is still forwarded to each owner; only derived overlay
        # accounting lacks a unique delegate in this legacy configuration.
        for oracle, original in originals:
            oracle.ledger = _MeteredLedger(meter, original)
    decomposition.ledger = None
    try:
        yield meter, persistent_ledger, effective_ledger
    finally:
        decomposition.ledger = persistent_ledger
        for oracle, original in originals:
            oracle.ledger = original


def _certified_cover(slab):
    """Use the central cover validator without introducing an import cycle.

    ``gmc.orientation`` is imported while ``gmc.mobility.lineage`` itself is
    being initialised in some callers, so the dependency must remain local.
    A raw object that claims certification but fails the validator is an
    internal contradiction, not ordinary UNKNOWN evidence.
    """
    from ..mobility.lineage import certified_cover_slice

    cover = certified_cover_slice(slab)
    if cover is not None:
        return cover
    raw = getattr(slab, "cover_slice", None)
    provenance = getattr(slab, "cover_provenance", {})
    if (raw is not None
            and (getattr(raw, "status", None) is CertStatus.CERTIFIED
                 or provenance.get("status") == CertStatus.CERTIFIED.name)):
        raise ConnectivityIntervalInvariantError((
            f"slab_{getattr(slab, 'slab_id', -1)}_malformed_certified_cover",
        ))
    return None


def _covering_components(components, point: shapely.Point,
                         predicate: str) -> tuple[int, ...]:
    hits = []
    for index, component in enumerate(components):
        geometry = component.geometry
        if (geometry.is_empty or not geometry.is_valid
                or not np.isfinite(float(geometry.area))):
            raise ConnectivityIntervalInvariantError((
                f"invalid_{predicate}_component_{index}",
            ))
        relation = (geometry.contains(point) if predicate == "contains"
                    else geometry.covers(point))
        if relation:
            hits.append(index)
    return tuple(hits)


def _closure_labels(components) -> tuple[int, ...]:
    """Connected-component labels after joining intersecting closures.

    Polygon components can be emitted separately when they meet only at a
    point or along a boundary.  Treating that contact as disconnected could
    create a false CLOSED verdict, so the upper-bound test joins every
    nonempty closure intersection.
    """
    parent = list(range(len(components)))

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)

    for left in range(len(components)):
        a = components[left].geometry
        for right in range(left + 1, len(components)):
            b = components[right].geometry
            if a.intersects(b):
                union(left, right)
    return tuple(find(index) for index in range(len(components)))


def classify_translation_interval(slab, start_xy, goal_xy) \
        -> ConnectivityInterval:
    """Classify fixed-translation connectivity throughout one slab.

    OPEN requires strict point containment in one interval-wide safe
    component.  CLOSED requires separation in the closure-connected upper
    free-space components, or exclusion of an endpoint from that upper bound.
    Missing interval certification always yields UNKNOWN.
    """
    start = _xy(start_xy, "start_xy")
    goal = _xy(goal_xy, "goal_xy")
    base = dict(
        interval=slab.interval,
        slab_id=int(getattr(slab, "slab_id", -1)),
        depth=int(getattr(slab, "depth", 0)),
    )
    cover = _certified_cover(slab)
    if cover is None:
        provenance = getattr(slab, "cover_provenance", {})
        return ConnectivityInterval(
            **base,
            verdict=ConnectivityVerdict.UNKNOWN,
            reason="interval_cover_not_certified",
            diagnostics={
                "cover_reason": provenance.get("reason", "missing"),
                "evidence_scope": provenance.get("evidence_scope", "none"),
            },
        )

    p_start = shapely.Point(tuple(start))
    p_goal = shapely.Point(tuple(goal))
    safe_start = _covering_components(cover.D_safe, p_start, "contains")
    safe_goal = _covering_components(cover.D_safe, p_goal, "contains")
    safe_common = tuple(sorted(set(safe_start).intersection(safe_goal)))

    possible_start = _covering_components(
        cover.D_possible, p_start, "covers")
    possible_goal = _covering_components(
        cover.D_possible, p_goal, "covers")
    labels = _closure_labels(cover.D_possible)
    possible_connected = bool(
        possible_start and possible_goal
        and {labels[index] for index in possible_start}.intersection(
            labels[index] for index in possible_goal)
    )

    # This is the local form of theta_safe subset theta_possible.  Checking it
    # before returning OPEN makes corrupted component extraction an internal
    # error rather than a false lower-bound claim.
    if safe_common and not possible_connected:
        raise ConnectivityIntervalInvariantError((
            f"slab_{base['slab_id']}_lower_exceeds_upper",
        ))
    if safe_common:
        return ConnectivityInterval(
            **base,
            verdict=ConnectivityVerdict.OPEN,
            reason="same_strict_interval_safe_component",
            diagnostics={
                "safe_component_ids": safe_common,
                "strict_endpoint_containment": True,
                "start_upper_components": possible_start,
                "goal_upper_components": possible_goal,
            },
        )
    if not possible_start or not possible_goal:
        return ConnectivityInterval(
            **base,
            verdict=ConnectivityVerdict.CLOSED,
            reason="endpoint_excluded_by_upper_cover",
            diagnostics={
                "start_upper_components": possible_start,
                "goal_upper_components": possible_goal,
            },
        )
    if not possible_connected:
        return ConnectivityInterval(
            **base,
            verdict=ConnectivityVerdict.CLOSED,
            reason="separate_upper_component_closures",
            diagnostics={
                "start_upper_components": possible_start,
                "goal_upper_components": possible_goal,
                "closure_intersection_joined": True,
            },
        )
    return ConnectivityInterval(
        **base,
        verdict=ConnectivityVerdict.UNKNOWN,
        reason="lower_and_upper_connectivity_bounds_disagree",
        diagnostics={
            "start_safe_components": safe_start,
            "goal_safe_components": safe_goal,
            "start_upper_components": possible_start,
            "goal_upper_components": possible_goal,
        },
    )


def _component_name(component, index: int) -> str:
    return str(getattr(component, "component_id", f"component_{index}"))


def classify_fixed_translation_interval(
        slab, query: FixedTranslationConnectivityQuery, *,
        decomposition_revision: int = 0) -> IntervalConnectivityCertificate:
    """Replay-oriented form of :func:`classify_translation_interval`."""
    if not isinstance(query, FixedTranslationConnectivityQuery):
        raise TypeError("query must be FixedTranslationConnectivityQuery")
    compact = classify_translation_interval(
        slab, query.start_xy, query.goal_xy,
    )
    cover = _certified_cover(slab)
    safe_component_id = None
    possible_start = ()
    possible_goal = ()
    if cover is not None:
        safe_indices = tuple(compact.diagnostics.get(
            "safe_component_ids", ()))
        if safe_indices:
            safe_component_id = _component_name(
                cover.D_safe[safe_indices[0]], safe_indices[0],
            )
        start_indices = tuple(compact.diagnostics.get(
            "start_upper_components", ()))
        goal_indices = tuple(compact.diagnostics.get(
            "goal_upper_components", ()))
        possible_start = tuple(
            _component_name(cover.D_possible[index], index)
            for index in start_indices
        )
        possible_goal = tuple(
            _component_name(cover.D_possible[index], index)
            for index in goal_indices
        )

    lower = compact.verdict is ConnectivityVerdict.OPEN
    upper = compact.verdict is not ConnectivityVerdict.CLOSED
    if lower and not upper:
        raise ConnectivityIntervalInvariantError((
            f"slab_{compact.slab_id}_lower_exceeds_upper",
        ))
    certified = compact.verdict is not ConnectivityVerdict.UNKNOWN
    return IntervalConnectivityCertificate(
        interval=compact.interval,
        value=compact.verdict,
        status=(CertStatus.CERTIFIED if certified else CertStatus.UNKNOWN),
        slab_id=compact.slab_id,
        decomposition_revision=int(decomposition_revision),
        lower_connected=lower,
        upper_connected=upper,
        safe_component_id=safe_component_id,
        possible_start_components=possible_start,
        possible_goal_components=possible_goal,
        uncertainty_sources=(() if certified else (compact.reason,)),
        reason=compact.reason,
        depth=compact.depth,
        diagnostics=dict(compact.diagnostics),
    )


def _require_tiling(intervals) -> None:
    failures = []
    if not intervals:
        failures.append("empty_orientation_classification")
    else:
        ordered = sorted(intervals, key=lambda item: item.interval.lo)
        if ordered[0].interval.lo != 0.0:
            failures.append("orientation_classification_missing_zero")
        if ordered[-1].interval.hi != TWO_PI:
            failures.append("orientation_classification_missing_two_pi")
        for left, right in zip(ordered, ordered[1:]):
            if left.interval.hi != right.interval.lo:
                failures.append("orientation_classification_gap_or_overlap")
                break
        if any(not np.isfinite([item.interval.lo, item.interval.hi]).all()
               or item.interval.hi <= item.interval.lo
               for item in ordered):
            failures.append("invalid_orientation_classification_interval")
    if failures:
        raise ConnectivityIntervalInvariantError(tuple(dict.fromkeys(failures)))


def _periodic_merge(intervals) -> tuple[Interval, ...]:
    """Merge a subset of a [0, 2*pi] tiling, including through the seam."""
    ordered = sorted(intervals, key=lambda item: item.lo)
    if not ordered:
        return ()
    merged = []
    for interval in ordered:
        if (not np.isfinite([interval.lo, interval.hi]).all()
                or interval.hi <= interval.lo):
            raise ConnectivityIntervalInvariantError((
                "invalid_bound_interval",
            ))
        if merged and interval.lo == merged[-1].hi:
            merged[-1] = Interval(merged[-1].lo, interval.hi)
        else:
            merged.append(interval)
    if (len(merged) > 1 and merged[0].lo == 0.0
            and merged[-1].hi == TWO_PI):
        seam = Interval(merged[-1].lo, merged[0].hi + TWO_PI)
        merged = [*merged[1:-1], seam]
        merged.sort(key=lambda item: item.lo)
    return tuple(merged)


def _linear_pieces(intervals, period: float = TWO_PI):
    pieces = []
    for interval in intervals:
        width = float(interval.width)
        if (not np.isfinite(width) or width <= 0.0
                or width > period + 64.0 * np.finfo(float).eps * period):
            raise ConnectivityIntervalInvariantError((
                "invalid_periodic_bound_width",
            ))
        if width >= period:
            pieces.append((0.0, period))
            continue
        lo = float(interval.lo) % period
        hi = lo + width
        if hi <= period:
            pieces.append((lo, hi))
        else:
            pieces.extend(((lo, period), (0.0, hi - period)))
    return sorted(pieces)


def validate_connectivity_bounds(theta_safe, theta_possible,
                                 period: float = TWO_PI) -> None:
    """Raise INTERNAL-error type if the certified lower bound exceeds upper."""
    upper = _linear_pieces(theta_possible, period)
    failures = []
    for lo, hi in _linear_pieces(theta_safe, period):
        cursor = lo
        for upper_lo, upper_hi in upper:
            if upper_hi < cursor:
                continue
            if upper_lo > cursor:
                break
            cursor = max(cursor, upper_hi)
            if cursor >= hi:
                break
        if cursor < hi:
            failures.append("theta_safe_not_subset_theta_possible")
            break
    if failures:
        raise ConnectivityIntervalInvariantError(tuple(failures))


def _unknown_runs(intervals, *, resolution_reason: str):
    """Return periodic maximal UNKNOWN runs and opposite-neighbour brackets."""
    n_items = len(intervals)
    known = [index for index, item in enumerate(intervals)
             if item.verdict is not ConnectivityVerdict.UNKNOWN]
    if not known:
        reasons = tuple(dict.fromkeys(
            [item.reason for item in intervals] + [resolution_reason]
        ))
        unresolved = UnresolvedInterval(
            Interval(0.0, TWO_PI), reasons,
            tuple(item.slab_id for item in intervals),
        )
        return (unresolved,), ()

    unresolved = []
    brackets = []
    anchor = known[0]
    offset = 1
    while offset < n_items:
        index = (anchor + offset) % n_items
        if intervals[index].verdict is not ConnectivityVerdict.UNKNOWN:
            offset += 1
            continue
        run = []
        while offset < n_items:
            index = (anchor + offset) % n_items
            if intervals[index].verdict is not ConnectivityVerdict.UNKNOWN:
                break
            run.append(index)
            offset += 1
        left = intervals[(run[0] - 1) % n_items]
        right = intervals[(run[-1] + 1) % n_items]
        lo = intervals[run[0]].interval.lo
        hi = intervals[run[-1]].interval.hi
        if run[-1] < run[0] or hi <= lo:
            hi += TWO_PI
        run_interval = Interval(float(lo), float(hi))
        reasons = tuple(dict.fromkeys(
            [intervals[i].reason for i in run] + [resolution_reason]
        ))
        unresolved.append(UnresolvedInterval(
            run_interval, reasons,
            tuple(intervals[i].slab_id for i in run),
        ))
        if left.verdict is not right.verdict:
            brackets.append(TransitionBracket(
                interval=run_interval,
                left_verdict=left.verdict,
                right_verdict=right.verdict,
            ))
    unresolved.sort(key=lambda item: item.interval.lo)
    brackets.sort(key=lambda item: item.interval.lo)
    return tuple(unresolved), tuple(brackets)


def _wall_unknown(slab) -> ConnectivityInterval:
    """Return a conservative leaf without inspecting its expensive cover."""
    return ConnectivityInterval(
        interval=slab.interval,
        verdict=ConnectivityVerdict.UNKNOWN,
        slab_id=int(getattr(slab, "slab_id", -1)),
        depth=int(getattr(slab, "depth", 0)),
        reason="wall_budget_exhausted",
        diagnostics={"classification_skipped_after_deadline": True},
    )


def _leaf_cache_key(slab) -> tuple[float, float]:
    return float(slab.interval.lo), float(slab.interval.hi)


def _same_classification_evidence(previous, current) -> bool:
    """Whether a renumbered leaf retains the exact immutable proof objects."""
    if previous.interval != current.interval:
        return False
    # ``refine_connectivity_slab`` retains every leaf before the insertion and
    # uses dataclasses.replace only to renumber later leaves.  All evidence
    # fields below therefore retain identity.  Requiring each one prevents a
    # caller-mutated/rebound cover from inheriting an earlier classification.
    return all(
        getattr(previous, name, None) is getattr(current, name, None)
        for name in (
            "left_slice", "mid_slice", "right_slice",
            "cover_slice", "cover_provenance",
        )
    )


def _classify_leaves_with_cache(slabs, start, goal, deadline, previous_cache):
    """Classify only new leaves, abstaining when the wall deadline expires."""
    rows = []
    cache = {}
    deadline_exhausted = False
    for slab in sorted(slabs, key=lambda item: item.interval.lo):
        key = _leaf_cache_key(slab)
        cached = previous_cache.get(key)
        if cached is not None and _same_classification_evidence(
                cached[0], slab):
            item = replace(
                cached[1],
                interval=slab.interval,
                slab_id=int(getattr(slab, "slab_id", -1)),
                depth=int(getattr(slab, "depth", 0)),
            )
            rows.append(item)
            cache[key] = (slab, item)
            continue

        # Classification validates the interval cover and can perform many
        # polygon unions/intersections.  Never start it after the deadline.
        if deadline_exhausted or time.perf_counter() >= deadline:
            deadline_exhausted = True
            rows.append(_wall_unknown(slab))
            continue
        item = classify_translation_interval(slab, start, goal)
        rows.append(item)
        cache[key] = (slab, item)
        if time.perf_counter() >= deadline:
            deadline_exhausted = True

    _require_tiling(rows)
    if time.perf_counter() >= deadline:
        deadline_exhausted = True
    return tuple(rows), cache, deadline_exhausted


def _refinement_children(current, refined, record):
    """Validate a transactional split record and return its two new leaves."""
    failures = []
    before = int(getattr(current, "revision", 0))
    after = int(getattr(refined, "revision", 0))
    parent_id = record.get("parent_slab_id")
    parent_interval = tuple(record.get("parent_interval", ()))
    child_intervals = tuple(
        tuple(item) for item in record.get("child_intervals", ())
        if isinstance(item, (tuple, list))
    )
    midpoint_partition = bool(
        len(parent_interval) == 2
        and parent_interval[1] > parent_interval[0]
        and len(child_intervals) == 2
        and all(len(child) == 2 and child[1] > child[0]
                and child[1] - child[0]
                < parent_interval[1] - parent_interval[0]
                for child in child_intervals)
        and child_intervals[0][0] == parent_interval[0]
        and child_intervals[0][1] == child_intervals[1][0]
        and child_intervals[1][1] == parent_interval[1]
    )
    if (after != before + 1
            or record.get("revision_before") != before
            or record.get("revision_after") != after
            or not midpoint_partition):
        failures.append("non_monotone_refinement_revision_or_width")

    parent_by_id = {
        getattr(slab, "slab_id", None): slab
        for slab in getattr(current, "slabs", ())
    }
    parent = parent_by_id.get(parent_id)
    if parent is None or _leaf_cache_key(parent) != parent_interval:
        failures.append("refinement_parent_binding")

    raw_child_ids = record.get("child_slab_ids", ())
    child_ids = tuple(raw_child_ids) \
        if isinstance(raw_child_ids, (tuple, list)) else ()
    slabs = tuple(getattr(refined, "slabs", ()))
    slab_ids = tuple(getattr(slab, "slab_id", None) for slab in slabs)
    if (len(child_ids) != 2 or len(set(child_ids)) != 2
            or any(isinstance(value, (bool, np.bool_))
                   or not isinstance(value, Integral)
                   for value in child_ids)
            or slab_ids != tuple(range(len(slabs)))):
        failures.append("refinement_child_id_binding")
        children = ()
    else:
        by_id = {slab.slab_id: slab for slab in slabs}
        children = tuple(by_id.get(int(value)) for value in child_ids)
        if any(child is None for child in children):
            failures.append("refinement_child_missing")
            children = ()
        elif tuple(_leaf_cache_key(child) for child in children) != \
                child_intervals:
            failures.append("refinement_child_interval_binding")

    if (getattr(current, "input_binding", None) is not None
            and getattr(refined, "input_binding", None)
            is not getattr(current, "input_binding", None)):
        failures.append("refinement_input_binding_drift")
    if failures:
        raise ConnectivityIntervalInvariantError(tuple(dict.fromkeys(failures)))
    return children


def certified_connectivity_intervals(
        scene, robot, cfg, oracles, decomposition, start_xy, goal_xy, *,
        max_refinements: int | None = None,
        max_support_calls: int | None = None,
        max_wall_seconds: float | None = None,
        event_tolerance: float | None = None,
        deadline: float | None = None) -> ConnectivityIntervalReport:
    """Adaptively sandwich fixed-translation connectivity over full ``S1``.

    The widest unresolved refinable slab is bisected first, with lower angle
    as the deterministic tie breaker.  ``max_refinements`` counts successful
    binary splits.  Exhausting that budget, ``theta_min``, or ``max_depth``
    leaves an explicit unresolved interval; it never manufactures a verdict.

    The input decomposition is not replaced.  The final refined decomposition
    is returned in the report so callers can choose whether to retain it.
    """
    start = _xy(start_xy, "start_xy")
    goal = _xy(goal_xy, "goal_xy")
    started = time.perf_counter()
    if max_refinements is None:
        max_refinements = int(getattr(cfg.query, "max_refinement_rounds", 0))
    if (isinstance(max_refinements, (bool, np.bool_))
            or not isinstance(max_refinements, Integral)
            or int(max_refinements) < 0):
        raise ValueError("max_refinements must be a non-negative integer")
    max_refinements = int(max_refinements)

    if max_support_calls is None:
        max_support_calls = int(getattr(cfg.query, "max_support_calls", 0))
    if (isinstance(max_support_calls, (bool, np.bool_))
            or not isinstance(max_support_calls, Integral)
            or int(max_support_calls) < 0):
        raise ValueError("max_support_calls must be a non-negative integer")
    max_support_calls = int(max_support_calls)
    if max_wall_seconds is None:
        max_wall_seconds = float(getattr(cfg.query, "max_wall_seconds", 0.0))
    max_wall_seconds = float(max_wall_seconds)
    if not np.isfinite(max_wall_seconds) or max_wall_seconds < 0.0:
        raise ValueError("max_wall_seconds must be finite and non-negative")
    if event_tolerance is not None:
        event_tolerance = float(event_tolerance)
        if not np.isfinite(event_tolerance) or event_tolerance <= 0.0:
            raise ValueError("event_tolerance must be finite and positive")
    wall_deadline = started + max_wall_seconds
    if deadline is not None:
        deadline = float(deadline)
        if not np.isfinite(deadline):
            raise ValueError("deadline must be a finite perf_counter timestamp")
        wall_deadline = min(wall_deadline, deadline)

    current = decomposition
    initial_revision = int(getattr(current, "revision", 0))
    calls_before = sum(int(getattr(oracle, "calls", 0))
                       for oracle in oracles)
    trace = []
    classification_cache = {}
    fully_validated_revision = None
    stop_reason = None

    # The public entry point still rejects a foreign or corrupt decomposition,
    # but an already-expired query must abstain before starting this expensive
    # full structural replay.  No unvalidated cover is consumed in that path.
    if time.perf_counter() >= wall_deadline:
        classified = tuple(_wall_unknown(slab) for slab in sorted(
            current.slabs, key=lambda item: item.interval.lo,
        ))
        _require_tiling(classified)
        stop_reason = "wall_budget_exhausted"
    else:
        # Preserve the provenance-bearing CandidateOracleList returned here.
        # Converting it to tuple would make every later refinement independently
        # rebuild the candidate BVH merely to re-establish the same binding.
        oracles = validate_candidate_oracles(
            scene, robot, getattr(scene, "workspace", None), oracles,
        )
        if time.perf_counter() >= wall_deadline:
            classified = tuple(_wall_unknown(slab) for slab in sorted(
                current.slabs, key=lambda item: item.interval.lo,
            ))
            _require_tiling(classified)
            stop_reason = "wall_budget_exhausted"
        else:
            input_failures = _compiler_input_failures(
                scene, robot, cfg, oracles, current,
            )
            if input_failures:
                raise ConnectivityIntervalInvariantError(input_failures)
            fully_validated_revision = initial_revision
            classified, classification_cache, expired = \
                _classify_leaves_with_cache(
                    current.slabs, start, goal, wall_deadline, {},
                )
            if expired:
                stop_reason = "wall_budget_exhausted"

    while stop_reason is None:
        if time.perf_counter() >= wall_deadline:
            stop_reason = "wall_budget_exhausted"
            break
        unknown = [item for item in classified
                   if item.verdict is ConnectivityVerdict.UNKNOWN]
        merged_unknown = ()
        success_reason = "resolved" if not unknown else None
        if unknown:
            merged_unknown, _ = _unknown_runs(
                classified, resolution_reason="adaptive_selection",
            )
            if (event_tolerance is not None and merged_unknown
                    and max(item.interval.width for item in merged_unknown)
                    <= event_tolerance):
                success_reason = "event_tolerance_met"

        if success_reason is not None:
            # Child-local validation is sufficient while deciding what to
            # refine.  Before promoting a refined revision to a successful
            # sweep, replay the complete public corruption detector exactly
            # once for that revision.
            revision = int(getattr(current, "revision", 0))
            if fully_validated_revision != revision:
                if time.perf_counter() >= wall_deadline:
                    stop_reason = "wall_budget_exhausted"
                    break
                final_failures = _compiler_input_failures(
                    scene, robot, cfg, oracles, current,
                )
                if final_failures:
                    raise ConnectivityIntervalInvariantError(final_failures)
                fully_validated_revision = revision
                if time.perf_counter() >= wall_deadline:
                    stop_reason = "wall_budget_exhausted"
                    break
            stop_reason = success_reason
            break
        if len(trace) >= max_refinements:
            stop_reason = "refinement_budget_exhausted"
            break
        support_calls = (
            sum(int(getattr(oracle, "calls", 0)) for oracle in oracles)
            - calls_before
        )
        if support_calls >= max_support_calls:
            stop_reason = "support_budget_exhausted"
            break
        if time.perf_counter() >= wall_deadline:
            stop_reason = "wall_budget_exhausted"
            break
        # Refine the largest periodic UNKNOWN run first; within it choose the
        # widest leaf and then the lowest angle.  This deterministic order is
        # stable under slab-id renumbering.
        target_run = min(
            merged_unknown,
            key=lambda item: (-item.interval.width, item.interval.lo),
        )
        run_ids = set(target_run.slab_ids)
        run_unknown = [item for item in unknown if item.slab_id in run_ids]
        refinable = [
            item for item in unknown
            if item.slab_id in run_ids
            if (item.interval.width > cfg.orientation.theta_min
                and item.depth < cfg.orientation.max_depth
                and item.interval.lo < item.interval.midpoint
                < item.interval.hi)
        ]
        if not refinable:
            at_depth = any(
                item.depth >= cfg.orientation.max_depth
                for item in run_unknown)
            at_width = any(
                item.interval.width <= cfg.orientation.theta_min
                for item in run_unknown)
            stop_reason = (
                "max_depth_reached" if at_depth and not at_width
                else "theta_min_reached" if at_width and not at_depth
                else "resolution_limit_reached"
            )
            break
        target = min(
            refinable,
            key=lambda item: (-item.interval.width, item.interval.lo),
        )
        remaining_support = max_support_calls - support_calls
        persistent_ledger = getattr(current, "ledger", None)
        if time.perf_counter() >= wall_deadline:
            stop_reason = "wall_budget_exhausted"
            break
        try:
            with _bounded_refinement_support(
                    oracles, current, remaining_support) as bounded:
                _, _, effective_ledger = bounded
                refine_kwargs = {"ledger": effective_ledger}
                if getattr(current, "construction_mode", None) == \
                        "fixed_translation_connectivity":
                    refined, record = refine_connectivity_slab(
                        scene, robot, cfg, oracles, current, target.slab_id,
                        # The specialized compiler checks the same absolute
                        # deadline throughout pair and cover construction.
                        deadline=wall_deadline,
                        **refine_kwargs,
                    )
                else:
                    # Backward-compatible public path for callers holding a
                    # generic adaptive decomposition.  The surrounding checks
                    # still reject a split that finishes after the deadline.
                    refined, record = refine_slab(
                        scene, robot, cfg, oracles, current, target.slab_id,
                        **refine_kwargs,
                    )
        except CompilationDeadlineExceeded:
            stop_reason = "wall_budget_exhausted"
            break
        except BudgetExceeded as exc:
            stop_reason = _budget_stop_reason(exc)
            break
        except (FloatingPointError, OverflowError, np.linalg.LinAlgError,
                shapely.GEOSException):
            # Expected numerical non-resolution is certificate data.  API,
            # type, provenance, and other programming failures deliberately
            # remain uncaught internal errors.
            stop_reason = "numeric_certificate_unresolved"
            break
        # A successfully constructed revision should retain the caller's
        # persistent work ledger, never the temporary hard-budget proxy.
        refined.ledger = persistent_ledger
        calls_after_split = (
            sum(int(getattr(oracle, "calls", 0)) for oracle in oracles)
            - calls_before
        )
        # A synchronously completed split is accepted only if it completed
        # inside both independent sweep budgets.  Its shared cache may contain
        # reusable work, but its children are not certificate leaves here.
        if calls_after_split > max_support_calls:
            stop_reason = "support_budget_exhausted"
            break
        if time.perf_counter() >= wall_deadline:
            stop_reason = "wall_budget_exhausted"
            break
        if not record.get("changed", False):
            stop_reason = "refinement_no_progress"
            break
        _refinement_children(current, refined, record)
        # Cached classifications are reused only for leaves retaining the
        # exact immutable cover evidence.  Consequently this call performs
        # the central cover/component validation on the two new children and
        # does not replay every unchanged leaf after each split.
        next_classified, next_cache, expired = \
            _classify_leaves_with_cache(
                refined.slabs, start, goal, wall_deadline,
                classification_cache,
            )
        trace.append(dict(record))
        current = refined
        classified = next_classified
        classification_cache = next_cache
        if expired:
            stop_reason = "wall_budget_exhausted"

    # ``classified`` is already the exact current-revision leaf tiling.  Do
    # not repeat its expensive polygon work merely to construct the report.
    if time.perf_counter() >= wall_deadline:
        stop_reason = "wall_budget_exhausted"
    theta_safe = _periodic_merge([
        item.interval for item in classified
        if item.verdict is ConnectivityVerdict.OPEN
    ])
    theta_possible = _periodic_merge([
        item.interval for item in classified
        if item.verdict is not ConnectivityVerdict.CLOSED
    ])
    validate_connectivity_bounds(theta_safe, theta_possible)
    unresolved, brackets = _unknown_runs(
        classified, resolution_reason=stop_reason,
    ) if any(item.verdict is ConnectivityVerdict.UNKNOWN
             for item in classified) else ((), ())

    finished = time.perf_counter()
    elapsed = finished - started
    if finished >= wall_deadline and stop_reason != "wall_budget_exhausted":
        stop_reason = "wall_budget_exhausted"
        if unresolved:
            unresolved, brackets = _unknown_runs(
                classified, resolution_reason=stop_reason,
            )
    support_calls = (
        sum(int(getattr(oracle, "calls", 0)) for oracle in oracles)
        - calls_before
    )
    resolution_met = bool(
        not _is_budget_stop_reason(stop_reason)
        and elapsed <= max_wall_seconds
        and finished < wall_deadline
        and support_calls <= max_support_calls
        and (
            not unresolved
            or (event_tolerance is not None
                and all(item.interval.width <= event_tolerance
                        for item in unresolved))
        )
    )
    return ConnectivityIntervalReport(
        period=TWO_PI,
        start_xy=(float(start[0]), float(start[1])),
        goal_xy=(float(goal[0]), float(goal[1])),
        intervals=classified,
        theta_safe=theta_safe,
        theta_possible=theta_possible,
        unresolved=unresolved,
        transition_brackets=brackets,
        refinement_trace=tuple(trace),
        initial_revision=initial_revision,
        final_revision=int(getattr(current, "revision", 0)),
        stop_reason=stop_reason,
        decomposition=current,
        support_calls=int(support_calls),
        max_support_calls=max_support_calls,
        wall_seconds=float(elapsed),
        max_wall_seconds=max_wall_seconds,
        resolution_met=resolution_met,
        event_tolerance=event_tolerance,
    )


def _cyclic(interval: Interval) -> CyclicInterval:
    return CyclicInterval(
        float(interval.lo), float(interval.hi), TWO_PI,
    )


def certify_fixed_translation_sweep(
        scene, robot, cfg, oracles, decomposition,
        query: FixedTranslationConnectivityQuery, *,
        event_tolerance: float,
        max_splits: int,
        max_support_calls: int,
        max_wall_seconds: float) -> GateAngularCertificate:
    """Certify the query-specific angular sandwich to a target resolution.

    This is the explicit-budget, artifact-oriented API.  Reaching the target
    means every remaining periodic UNKNOWN run is at most
    ``event_tolerance`` wide.  Budget, depth, numeric, or certificate limits
    instead leave ``resolution_met=False`` and retain those regions in the
    upper bound.
    """
    if not isinstance(query, FixedTranslationConnectivityQuery):
        raise TypeError("query must be FixedTranslationConnectivityQuery")
    max_wall_seconds = float(max_wall_seconds)
    if not np.isfinite(max_wall_seconds) or max_wall_seconds < 0.0:
        raise ValueError("max_wall_seconds must be finite and non-negative")
    wrapper_started = time.perf_counter()
    wrapper_setup_seconds = time.perf_counter() - wrapper_started
    remaining_wall_seconds = max(
        0.0, max_wall_seconds - wrapper_setup_seconds,
    )
    report = certified_connectivity_intervals(
        scene, robot, cfg, oracles, decomposition,
        query.start_xy, query.goal_xy,
        max_refinements=max_splits,
        max_support_calls=max_support_calls,
        max_wall_seconds=remaining_wall_seconds,
        event_tolerance=event_tolerance,
        deadline=wrapper_started + max_wall_seconds,
    )
    revision = report.final_revision
    slabs = {
        slab.slab_id: slab for slab in report.decomposition.slabs
    }
    leaves = tuple(
        classify_fixed_translation_interval(
            slabs[item.slab_id], query,
            decomposition_revision=revision,
        )
        for item in report.intervals
    )
    brackets = tuple(
        ConnectivityTransitionBracket(
            interval=_cyclic(item.interval),
            left_value=item.left_verdict,
            right_value=item.right_verdict,
            # Existence follows from opposite certified neighbours.  It does
            # not certify topology constancy inside either general slab.
            status=CertStatus.CERTIFIED,
            multiplicity_lower_bound=item.minimum_transitions,
            isolated=False,
        )
        for item in report.transition_brackets
    )
    unknown = tuple(_cyclic(item.interval) for item in report.unresolved)
    wrapper_elapsed = time.perf_counter() - wrapper_started
    outer_wall_exhausted = wrapper_elapsed > max_wall_seconds
    if outer_wall_exhausted:
        report = replace(
            report,
            stop_reason="wall_budget_exhausted",
            wall_seconds=float(wrapper_elapsed),
            max_wall_seconds=max_wall_seconds,
            resolution_met=False,
        )
    else:
        report = replace(
            report,
            wall_seconds=float(wrapper_elapsed),
            max_wall_seconds=max_wall_seconds,
        )
    return GateAngularCertificate(
        query=query,
        period=TWO_PI,
        theta_safe=tuple(_cyclic(item) for item in report.theta_safe),
        theta_possible=tuple(
            _cyclic(item) for item in report.theta_possible),
        theta_unknown=unknown,
        transition_brackets=brackets,
        unresolved_regions=unknown,
        leaf_certificates=leaves,
        resolution_met=report.resolution_met,
        stop_reason=report.stop_reason,
        refinement_trace=report.refinement_trace,
        support_calls=report.support_calls,
        max_support_calls=int(max_support_calls),
        wall_seconds=report.wall_seconds,
        max_wall_seconds=max_wall_seconds,
        sweep_report=report,
    )


__all__ = [
    "ConnectivityTransitionBracket",
    "ConnectivityValue",
    "ConnectivityInterval",
    "ConnectivityIntervalInvariantError",
    "ConnectivityIntervalReport",
    "ConnectivityVerdict",
    "CyclicInterval",
    "FixedTranslationConnectivityQuery",
    "GateAngularCertificate",
    "IntervalConnectivityCertificate",
    "TransitionBracket",
    "UnresolvedInterval",
    "certified_connectivity_intervals",
    "certify_fixed_translation_sweep",
    "classify_fixed_translation_interval",
    "classify_translation_interval",
    "validate_connectivity_bounds",
]
