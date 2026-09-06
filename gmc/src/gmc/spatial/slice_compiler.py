"""M4 fixed-orientation compilation (Guide §7.1): one slice produces the
obstacle-side combinatorics (nerve) and the planning-side safe/possible
free-space components."""
from dataclasses import dataclass, field
import time

import numpy as np
import shapely

from ..types import CertStatus
from ..geometry.envelopes import ConvexSandwich, approximate_pair
from ..geometry.predicates import clean, set_precision_inner, set_precision_outer
from ..io.gs_io import validate_models
from .arrangement import FreeComponent, extract_components
from .bvh import candidate_pairs, validate_candidate_oracles
from .nerve import Nerve, build_contact_nerve
from .union_tree import IncrementalUnionTree


class CompilationDeadlineExceeded(TimeoutError):
    """An absolute compiler wall deadline expired at a safe work boundary.

    The exception is intentionally distinct from work-ledger exhaustion.  A
    caller can discard a staged slice/decomposition while retaining an exact
    account of all work completed before this boundary.
    """

    def __init__(self, stage: str, deadline: float):
        self.stage = str(stage)
        self.deadline = float(deadline)
        super().__init__(f"compilation wall deadline exceeded at {self.stage}")


def check_compilation_deadline(deadline, stage: str) -> None:
    """Raise before starting more synchronous work after ``deadline``.

    ``deadline`` is an absolute value in :func:`time.perf_counter`'s clock
    domain.  ``None`` disables wall checks for the existing compiler APIs.
    """
    if deadline is None:
        return
    value = float(deadline)
    if not np.isfinite(value):
        raise ValueError("deadline must be a finite perf_counter timestamp")
    if time.perf_counter() >= value:
        raise CompilationDeadlineExceeded(stage, value)


def _checked_polygonal_boolean(geometry, operation: str):
    """Normalize/repair one overlay result, then enforce Guide §7.2.

    Polygon differences should remain polygonal.  Silently discarding a line,
    dangle, invalid ring, NaN area, or non-finite bound could alter component
    topology, so such outputs are program errors rather than empty space.
    """
    try:
        result = clean(geometry)
        if not result.is_valid:
            raise FloatingPointError("result remains invalid after make_valid")
        area = float(result.area)
        if not np.isfinite(area) or area < 0.0:
            raise FloatingPointError("result area is not finite")
        if not result.is_empty:
            bounds = np.asarray(result.bounds, dtype=float)
            if bounds.shape != (4,) or not np.all(np.isfinite(bounds)):
                raise FloatingPointError("result bounds are not finite")
            parts = getattr(result, "geoms", (result,))
            if any(not isinstance(part, (shapely.Polygon,
                                         shapely.MultiPolygon))
                   for part in parts if not part.is_empty):
                raise FloatingPointError(
                    "result contains non-polygonal fragments")
        return result
    except (FloatingPointError, TypeError, ValueError) as exc:
        raise FloatingPointError(
            f"{operation} produced an unresolved polygonal result") from exc


@dataclass(frozen=True)
class CertifiedSlice:
    theta: float
    D_safe: tuple              # tuple[FreeComponent, ...]
    D_possible: tuple
    nerve: Nerve
    sandwiches: tuple          # tuple[ConvexSandwich, ...]
    C_plus: object             # union of outer polygons (shapely)
    C_minus: object
    status: CertStatus
    support_calls: int

    def locate(self, xy, side: str = "safe"):
        comps = self.D_safe if side == "safe" else self.D_possible
        pt = shapely.Point(xy)
        for c in comps:
            if c.geometry.covers(pt):
                return c
        return None


@dataclass(frozen=True)
class PairSandwichSlice:
    """A fixed-angle pair-envelope set without global arrangement assembly.

    Connectivity interval certificates need the midpoint pair sandwiches, but
    not a second fixed-angle union, workspace difference, component extraction,
    or contact nerve.  Keeping that restricted object explicit prevents it from
    being confused with a :class:`CertifiedSlice`.
    """

    theta: float
    sandwiches: tuple          # tuple[ConvexSandwich, ...]
    status: CertStatus
    support_calls: int
    ledger: object = field(default=None, repr=False, compare=False)


def build_pair_sandwich_slice(scene, robot, theta: float, cfg,
                              oracles=None, *, ledger=None,
                              deadline=None) -> PairSandwichSlice:
    """Build every candidate-pair envelope at one orientation, and no more.

    Pair order is the validated candidate-oracle order, exactly as in
    :func:`build_slice`.  Deadline checks occur before and after each complete
    pair approximation, so no partial pair is ever exposed to a caller.
    """
    check_compilation_deadline(deadline, "pair_sandwich_slice_entry")
    validate_models(scene, robot, cfg)
    if oracles is None:
        oracles = candidate_pairs(scene, robot, scene.workspace)
    else:
        oracles = validate_candidate_oracles(
            scene, robot, scene.workspace, oracles)
    oracles = tuple(oracles)
    if ledger is None:
        ledger = next((getattr(o, "ledger", None) for o in oracles
                       if getattr(o, "ledger", None) is not None), None)
    else:
        for oracle in oracles:
            oracle.ledger = ledger

    calls0 = sum(o.calls for o in oracles)
    sandwiches = []
    for oracle in oracles:
        check_compilation_deadline(
            deadline, "pair_sandwich_before_pair")
        sandwiches.append(approximate_pair(
            oracle, theta, cfg.pair_approx,
        ))
        check_compilation_deadline(
            deadline, "pair_sandwich_after_pair")
    sandwiches = tuple(sandwiches)
    status = (
        CertStatus.CERTIFIED
        if all(item.status is CertStatus.CERTIFIED for item in sandwiches)
        else CertStatus.APPROX_UNCERTIFIED
    )
    check_compilation_deadline(deadline, "pair_sandwich_slice_complete")
    return PairSandwichSlice(
        theta=float(theta),
        sandwiches=sandwiches,
        status=status,
        support_calls=sum(o.calls for o in oracles) - calls0,
        ledger=ledger,
    )


def assemble_slice(scene, theta: float, cfg, sandwiches,
                   build_nerve: bool = False,
                   support_calls: int = 0,
                   C_plus=None, C_minus=None, *,
                   ledger=None, deadline=None) -> CertifiedSlice:
    """Assemble a slice from pair envelopes.

    Keeping arrangement assembly separate from envelope construction lets the
    P5 incremental compiler reuse every unchanged pair sandwich while still
    rebuilding the global union/complement exactly.
    """
    check_compilation_deadline(deadline, "slice_assembly_entry")
    sandwiches = tuple(sandwiches)
    grid = cfg.geometry.workspace_precision
    if C_plus is None:
        check_compilation_deadline(deadline, "slice_assembly_before_outer_union")
        C_plus = (clean(IncrementalUnionTree(
            ((s.pair_id, set_precision_outer(s.outer, grid))
             for s in sandwiches),
            ledger=ledger,
        ).geometry)
                  if sandwiches else shapely.Polygon())
        check_compilation_deadline(deadline, "slice_assembly_after_outer_union")
    else:
        # The incremental union is already assembled from direction-preserving
        # leaves.  Re-snapping the final union could shrink C_plus again.
        check_compilation_deadline(
            deadline, "slice_assembly_before_outer_clean")
        C_plus = clean(C_plus)
        check_compilation_deadline(
            deadline, "slice_assembly_after_outer_clean")
    if C_minus is None:
        check_compilation_deadline(deadline, "slice_assembly_before_inner_union")
        C_minus = (clean(IncrementalUnionTree(
            ((s.pair_id, set_precision_inner(s.inner, grid))
             for s in sandwiches),
            ledger=ledger,
        ).geometry)
                   if sandwiches else shapely.Polygon())
        check_compilation_deadline(deadline, "slice_assembly_after_inner_union")
    else:
        # Likewise, re-snapping C_minus could expand the certified inner set.
        check_compilation_deadline(
            deadline, "slice_assembly_before_inner_clean")
        C_minus = clean(C_minus)
        check_compilation_deadline(
            deadline, "slice_assembly_after_inner_clean")
    # Workspace rounding can move the physical boundary outward.  The input
    # polygon is already validated by M0, so retain it verbatim for containment.
    ws = scene.workspace
    check_compilation_deadline(deadline, "slice_assembly_before_safe_difference")
    if ledger is not None:
        ledger.charge("difference_operations")
    F_safe = _checked_polygonal_boolean(
        ws.difference(C_plus), "workspace_minus_C_plus")
    check_compilation_deadline(deadline, "slice_assembly_after_safe_difference")
    check_compilation_deadline(
        deadline, "slice_assembly_before_possible_difference")
    if ledger is not None:
        ledger.charge("difference_operations")
    F_possible = _checked_polygonal_boolean(
        ws.difference(C_minus), "workspace_minus_C_minus")
    check_compilation_deadline(
        deadline, "slice_assembly_after_possible_difference")
    status = (CertStatus.CERTIFIED
              if sandwiches and all(s.status is CertStatus.CERTIFIED
                                    for s in sandwiches)
              else (CertStatus.CERTIFIED if not sandwiches
                    else CertStatus.APPROX_UNCERTIFIED))
    area_min = (100.0 * grid) ** 2
    check_compilation_deadline(deadline, "slice_assembly_before_safe_components")
    D_safe = tuple(extract_components(F_safe, sandwiches, "safe", theta,
                                      area_min, status))
    check_compilation_deadline(deadline, "slice_assembly_after_safe_components")
    # Dropping a tiny certified-safe component is conservative.  Dropping a
    # possible/free component is not: M_possible is an upper bound on truth,
    # so every positive-area component must remain regardless of sliver size.
    D_possible = tuple(extract_components(F_possible, sandwiches, "possible",
                                          theta, 0.0, status))
    check_compilation_deadline(
        deadline, "slice_assembly_after_possible_components")
    check_compilation_deadline(deadline, "slice_assembly_before_nerve")
    nerve = (build_contact_nerve(sandwiches, side="outer", ledger=ledger)
             if build_nerve else Nerve(simplices=(), edges=(), witness_points={}))
    check_compilation_deadline(deadline, "slice_assembly_after_nerve")
    return CertifiedSlice(theta=float(theta), D_safe=D_safe,
                          D_possible=D_possible, nerve=nerve,
                          sandwiches=sandwiches, C_plus=C_plus, C_minus=C_minus,
                          status=status,
                          support_calls=int(support_calls))


def build_slice(scene, robot, theta: float, cfg,
                oracles=None, build_nerve: bool = False, *,
                ledger=None, deadline=None) -> CertifiedSlice:
    """Build one fixed-orientation slice from scratch.

    ``build_nerve`` defaults to False because the v0 maximal-clique nerve is
    meant for small active sets (Guide §7.4, §16.2 commit 9 "debug only
    first").  The slab predicates do not consume the nerve.
    """
    pair_slice = build_pair_sandwich_slice(
        scene, robot, theta, cfg, oracles,
        ledger=ledger, deadline=deadline,
    )
    return assemble_slice(
        scene, theta, cfg, pair_slice.sandwiches, build_nerve=build_nerve,
        support_calls=pair_slice.support_calls,
        ledger=pair_slice.ledger, deadline=deadline,
    )
