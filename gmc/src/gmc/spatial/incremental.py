"""P5 incremental fixed-slice compilation.

The expensive pair envelopes are independent across ``PairID``.  This module
keeps them in a revisioned cache, invalidates only pairs whose scene/body
support changed, and then rebuilds the global union/complement from the mixed
set of reused and recomputed envelopes.  Reassembly is deliberately exact:
incremental output has the same semantics as :func:`build_slice`.
"""
from bisect import bisect_left, insort
from dataclasses import dataclass, replace

import numpy as np

from ..budget import WORK_KINDS, WorkLedger
from ..geometry.envelopes import approximate_pair
from ..geometry.predicates import set_precision_inner, set_precision_outer
from ..io.gs_io import validate_models
from ..types import PairID
from .bvh import candidate_pairs
from .slice_compiler import CertifiedSlice, assemble_slice
from .union_tree import IncrementalUnionTree


TWO_PI = 2.0 * np.pi


def _support_signature(support) -> tuple:
    """Value signature robust to replacement and in-place ndarray edits."""
    return (
        int(support.primitive_id),
        np.asarray(support.mean, dtype=np.float64).tobytes(),
        np.asarray(support.covariance, dtype=np.float64).tobytes(),
        float(support.level),
    )


def _oracle_signature(oracle) -> tuple:
    return (_support_signature(oracle.scene), _support_signature(oracle.body))


def _pair_cfg_signature(cfg) -> tuple:
    pair = cfg.pair_approx
    return (int(pair.initial_directions), int(pair.max_directions),
            float(pair.eps_pair), str(pair.certificate_mode),
            float(cfg.geometry.workspace_precision))


def _circular_distance(a: float, b: float) -> float:
    delta = abs(float(a) - float(b)) % TWO_PI
    return min(delta, TWO_PI - delta)


def _theta_invariant(oracle) -> bool:
    """Whether the body primitive is unchanged by in-plane rotation."""
    body = oracle.body
    covariance = body.covariance
    # Zero-cost reuse has no error envelope, so this gate must be exact.  A
    # tolerance-based test is unsafe at large covariance scales.
    return bool(
        np.all(body.mean == 0.0)
        and covariance[0, 0] == covariance[1, 1]
        and covariance[0, 1] == 0.0
        and covariance[1, 0] == 0.0
    )


@dataclass
class IncrementalStats:
    builds: int = 0
    failed_builds: int = 0
    model_updates: int = 0
    config_invalidations: int = 0
    envelopes_computed: int = 0
    envelopes_reused: int = 0
    envelope_support_calls: int = 0
    orientation_seeded: int = 0
    orientation_seeds_bypassed: int = 0
    theta_invariant_reuses: int = 0
    union_tree_builds: int = 0
    union_tree_clones: int = 0
    union_leaf_updates: int = 0
    union_operations: int = 0
    seed_lookup_candidates: int = 0
    union_lookup_candidates: int = 0
    last_computed: int = 0
    last_reused: int = 0
    last_envelope_support_calls: int = 0
    last_orientation_seeded: int = 0
    last_orientation_seeds_bypassed: int = 0
    last_theta_invariant_reuses: int = 0
    last_union_leaf_updates: int = 0
    last_union_operations: int = 0
    last_seed_lookup_candidates: int = 0
    last_union_lookup_candidates: int = 0

    @property
    def support_calls(self) -> int:
        """Backward-compatible alias for envelope-only support work."""
        return self.envelope_support_calls

    @property
    def last_support_calls(self) -> int:
        return self.last_envelope_support_calls


@dataclass
class _UnionState:
    pair_ids: tuple[PairID, ...]
    # Strong references make identity comparison safe: integer ``id`` tokens
    # alone can alias after an invalidated sandwich is garbage-collected.
    geometry_refs: dict[PairID, tuple[object, object]]
    grid: float
    plus: IncrementalUnionTree
    minus: IncrementalUnionTree


class IncrementalSliceCompiler:
    """Revisioned cache for morphology/scene edits at fixed orientations.

    The public contract is intentionally small: call :meth:`compile_slice`,
    replace either model with :meth:`update_models`, and compile the same or
    other orientations again.  Candidate-pair discovery is rerun after every
    model update, so moved/added/removed supports cannot leave stale pairs.
    """

    def __init__(self, scene, robot, cfg, *, ledger=None,
                 max_cached_orientations: int = 4096):
        self.scene = scene
        self.robot = robot
        self.cfg = cfg
        self.ledger = ledger if ledger is not None else WorkLedger()
        if int(max_cached_orientations) != max_cached_orientations \
                or max_cached_orientations < 1:
            raise ValueError("max_cached_orientations must be positive")
        self.max_cached_orientations = int(max_cached_orientations)
        self.stats = IncrementalStats()
        self._oracles: dict[PairID, object] = {}
        self._signatures: dict[PairID, tuple] = {}
        self._revisions: dict[PairID, int] = {}
        # Per-pair/revision buckets make exact lookup O(1) and circular-nearest
        # lookup O(log K), rather than scanning all N*K cached sandwiches for
        # every one of N pairs.
        self._sandwich_index: dict[tuple[PairID, int], dict[float, object]] = {}
        self._sandwich_angles: dict[tuple[PairID, int], list[float]] = {}
        self._union_cache: dict[float, _UnionState] = {}
        self._union_angles: dict[
            tuple[tuple[PairID, ...], float], list[float]
        ] = {}
        self._cfg_signature = _pair_cfg_signature(cfg)
        self._refresh_oracles()

    @property
    def oracles(self) -> tuple:
        return tuple(self._oracles.values())

    @property
    def cache_size(self) -> int:
        return sum(len(bucket) for bucket in self._sandwich_index.values())

    def _refresh_oracles(self) -> set[PairID]:
        old = self._oracles
        fresh = candidate_pairs(self.scene, self.robot, self.scene.workspace)
        current: dict[PairID, object] = {}
        signatures: dict[PairID, tuple] = {}
        changed: set[PairID] = set()

        for candidate in fresh:
            candidate.ledger = self.ledger
            candidate.max_cached_frames = self.max_cached_orientations
            prior = old.get(candidate.pair_id)
            signature = _oracle_signature(candidate)
            signatures[candidate.pair_id] = signature
            if (prior is not None
                    and self._signatures.get(candidate.pair_id) == signature):
                # A model update commonly replaces support objects even when
                # their numeric geometry is unchanged.  Reusing the cached
                # oracle is sound in that case, but it must be rebound to the
                # *current* model objects.  Otherwise downstream provenance
                # validation quite rightly rejects a stale oracle (and future
                # in-place edits would be observed through the wrong object).
                prior.scene = candidate.scene
                prior.body = candidate.body
                prior.ledger = self.ledger
                prior.max_cached_frames = self.max_cached_orientations
                current[candidate.pair_id] = prior
            else:
                candidate.revision = self._revisions.get(candidate.pair_id, -1) + 1
                self._revisions[candidate.pair_id] = candidate.revision
                current[candidate.pair_id] = candidate
                changed.add(candidate.pair_id)

        changed.update(set(old) - set(current))
        if changed:
            for key in list(self._sandwich_index):
                if key[0] in changed:
                    self._sandwich_index.pop(key, None)
                    self._sandwich_angles.pop(key, None)
        self._oracles = current
        self._signatures = signatures
        return changed

    def _ensure_config_current(self) -> None:
        signature = _pair_cfg_signature(self.cfg)
        if signature != self._cfg_signature:
            self._sandwich_index.clear()
            self._sandwich_angles.clear()
            self._union_cache.clear()
            self._union_angles.clear()
            self._cfg_signature = signature
            self.stats.config_invalidations += 1

    @staticmethod
    def _index_key(oracle) -> tuple[PairID, int]:
        return oracle.pair_id, oracle.revision

    def _cached_sandwich(self, theta: float, oracle):
        return self._sandwich_index.get(
            self._index_key(oracle), {}
        ).get(theta)

    def _nearest_sandwich(self, theta: float, oracle):
        key = self._index_key(oracle)
        angles = self._sandwich_angles.get(key, ())
        if not angles:
            return None, 0
        position = bisect_left(angles, theta)
        candidate_angles = tuple(dict.fromkeys((
            angles[position % len(angles)], angles[position - 1]
        )))
        nearest = min(candidate_angles,
                      key=lambda value: (_circular_distance(theta, value),
                                         value))
        return self._sandwich_index[key][nearest], len(candidate_angles)

    def _commit_sandwich(self, theta: float, oracle, sandwich) -> None:
        key = self._index_key(oracle)
        bucket = self._sandwich_index.setdefault(key, {})
        angles = self._sandwich_angles.setdefault(key, [])
        if theta not in bucket:
            insort(angles, theta)
        bucket[theta] = sandwich
        while len(bucket) > self.max_cached_orientations:
            # FIFO eviction bounds memory; correctness never depends on a hit.
            evicted = next(iter(bucket))
            del bucket[evicted]
            angles.pop(bisect_left(angles, evicted))

    def _nearest_union_state(self, theta: float,
                             pair_ids: tuple[PairID, ...], grid: float):
        key = pair_ids, grid
        angles = self._union_angles.get(key, ())
        if not angles:
            return None, 0
        position = bisect_left(angles, theta)
        candidates = tuple(dict.fromkeys((
            angles[position % len(angles)], angles[position - 1]
        )))
        nearest = min(candidates,
                      key=lambda value: (_circular_distance(theta, value),
                                         value))
        return self._union_cache[nearest], len(candidates)

    def _remove_union_angle(self, theta: float, state: _UnionState) -> None:
        key = state.pair_ids, state.grid
        angles = self._union_angles.get(key)
        if angles is None:
            return
        position = bisect_left(angles, theta)
        if position < len(angles) and angles[position] == theta:
            angles.pop(position)
        if not angles:
            self._union_angles.pop(key, None)

    def _commit_union_state(self, theta: float, state: _UnionState) -> None:
        previous = self._union_cache.get(theta)
        if previous is not None:
            self._remove_union_angle(theta, previous)
        self._union_cache[theta] = state
        key = state.pair_ids, state.grid
        angles = self._union_angles.setdefault(key, [])
        insort(angles, theta)
        while len(self._union_cache) > self.max_cached_orientations:
            evicted_theta = next(iter(self._union_cache))
            evicted_state = self._union_cache.pop(evicted_theta)
            self._remove_union_angle(evicted_theta, evicted_state)

    def update_models(self, *, scene=None, robot=None) -> set[PairID]:
        """Replace models and return the pair IDs invalidated by the change."""
        if scene is not None:
            self.scene = scene
        if robot is not None:
            self.robot = robot
        self.stats.model_updates += 1
        return self._refresh_oracles()

    def invalidate_pairs(self, pair_ids) -> None:
        """Explicit invalidation for callers managing external revisions."""
        pair_ids = set(pair_ids)
        for pair_id in pair_ids:
            oracle = self._oracles.get(pair_id)
            if oracle is not None:
                oracle.bump_revision()
                self._revisions[pair_id] = oracle.revision
        for key in list(self._sandwich_index):
            if key[0] in pair_ids:
                self._sandwich_index.pop(key, None)
                self._sandwich_angles.pop(key, None)

    def compile_slice(self, theta: float,
                      build_nerve: bool = False) -> CertifiedSlice:
        validate_models(self.scene, self.robot, self.cfg)
        self._ensure_config_current()
        # Cache identity must be exact and scale independent.  A tiny angular
        # difference times a large body offset can move a C-obstacle by an
        # arbitrary distance, so decimal rounding is unsound.
        theta_key = float(theta) % TWO_PI
        computed = reused = support_calls = seeded = invariant_reuses = 0
        seeds_bypassed = 0
        lookup_candidates = union_lookup_candidates = 0
        sandwiches = []
        pending_cache = []

        try:
            for oracle in self._oracles.values():
                sandwich = self._cached_sandwich(theta_key, oracle)
                if sandwich is None:
                    neighbour, inspected = self._nearest_sandwich(
                        theta_key, oracle
                    )
                    lookup_candidates += inspected
                    if neighbour is not None and _theta_invariant(oracle):
                        # Geometry is exactly theta-independent; only metadata
                        # changes.  Retain the same immutable polygons.
                        sandwich = replace(neighbour, theta=theta_key,
                                           support_calls=0)
                        reused += 1
                        invariant_reuses += 1
                        self.ledger.charge("cache_hits")
                    else:
                        # A neighbouring adaptive direction set makes the
                        # approximation depend on cache history.  That is not
                        # an admissible incremental optimization: compiling a
                        # theta after cache eviction must have exactly the
                        # same geometry/status as a clean build.  Keep the
                        # indexed lookup for instrumentation, but deliberately
                        # bypass the seed until a canonical continuation rule
                        # is available.
                        seeds_bypassed += int(neighbour is not None)
                        sandwich = approximate_pair(
                            oracle, theta_key, self.cfg.pair_approx,
                        )
                        computed += 1
                        support_calls += sandwich.support_calls
                    pending_cache.append((theta_key, oracle, sandwich))
                else:
                    reused += 1
                    self.ledger.charge("cache_hits")
                sandwiches.append(sandwich)

            pair_ids = tuple(self._oracles)
            grid = float(self.cfg.geometry.workspace_precision)
            geometry_refs = {
                sandwich.pair_id: (sandwich.outer, sandwich.inner)
                for sandwich in sandwiches
            }
            source = self._union_cache.get(theta_key)
            if not (source is not None and source.pair_ids == pair_ids
                    and source.grid == grid):
                source, inspected = self._nearest_union_state(
                    theta_key, pair_ids, grid
                )
                union_lookup_candidates += inspected

            union_builds = union_clones = leaf_updates = union_operations = 0
            if source is None:
                plus = IncrementalUnionTree(
                    ((s.pair_id, set_precision_outer(s.outer, grid))
                     for s in sandwiches),
                    ledger=self.ledger,
                )
                minus = IncrementalUnionTree(
                    ((s.pair_id, set_precision_inner(s.inner, grid))
                     for s in sandwiches),
                    ledger=self.ledger,
                )
                union_builds = 2
                union_operations = plus.build_operations + minus.build_operations
            else:
                replacements = [
                    s for s in sandwiches
                    if (source.geometry_refs.get(s.pair_id) is None
                        or source.geometry_refs[s.pair_id][0] is not s.outer
                        or source.geometry_refs[s.pair_id][1] is not s.inner)
                ]
                if replacements:
                    plus = source.plus.clone()
                    minus = source.minus.clone()
                    union_clones = 2
                    leaf_updates = len(replacements)
                    union_operations = plus.update({
                        s.pair_id: set_precision_outer(s.outer, grid)
                        for s in replacements
                    })
                    union_operations += minus.update({
                        s.pair_id: set_precision_inner(s.inner, grid)
                        for s in replacements
                    })
                else:
                    plus, minus = source.plus, source.minus

            union_state = _UnionState(
                pair_ids=pair_ids, geometry_refs=geometry_refs, grid=grid,
                plus=plus, minus=minus,
            )
            result = assemble_slice(
                self.scene, theta_key, self.cfg, sandwiches,
                build_nerve=build_nerve, support_calls=support_calls,
                C_plus=plus.geometry, C_minus=minus.geometry,
                ledger=self.ledger,
            )
        except Exception:
            # Pair and union caches are commit-on-success.  The ledger remains
            # authoritative for work attempted before a hard-budget failure.
            self.stats.failed_builds += 1
            raise

        for cached_theta, oracle, sandwich in pending_cache:
            self._commit_sandwich(cached_theta, oracle, sandwich)
        self._commit_union_state(theta_key, union_state)
        self.stats.builds += 1
        self.stats.envelopes_computed += computed
        self.stats.envelopes_reused += reused
        self.stats.envelope_support_calls += support_calls
        self.stats.orientation_seeded += seeded
        self.stats.orientation_seeds_bypassed += seeds_bypassed
        self.stats.theta_invariant_reuses += invariant_reuses
        self.stats.union_tree_builds += union_builds
        self.stats.union_tree_clones += union_clones
        self.stats.union_leaf_updates += leaf_updates
        self.stats.union_operations += union_operations
        self.stats.seed_lookup_candidates += lookup_candidates
        self.stats.union_lookup_candidates += union_lookup_candidates
        self.stats.last_computed = computed
        self.stats.last_reused = reused
        self.stats.last_envelope_support_calls = support_calls
        self.stats.last_orientation_seeded = seeded
        self.stats.last_orientation_seeds_bypassed = seeds_bypassed
        self.stats.last_theta_invariant_reuses = invariant_reuses
        self.stats.last_union_leaf_updates = leaf_updates
        self.stats.last_union_operations = union_operations
        self.stats.last_seed_lookup_candidates = lookup_candidates
        self.stats.last_union_lookup_candidates = union_lookup_candidates
        return result

    def compile_orientations(self, thetas) -> tuple[CertifiedSlice, ...]:
        return tuple(self.compile_slice(theta) for theta in thetas)


@dataclass(frozen=True)
class IncrementalCompilationState:
    scene: object
    robot: object
    decomposition: object
    mobility: object
    changed_pairs: tuple[PairID, ...]
    envelopes_computed: int
    envelopes_reused: int
    envelope_support_calls: int
    support_calls: int
    work_delta: dict


def _work_delta(before: dict, after: dict) -> dict:
    phases = set(before["by_phase"]) | set(after["by_phase"])
    by_phase = {
        phase: {
            kind: (after["by_phase"].get(phase, {}).get(kind, 0)
                   - before["by_phase"].get(phase, {}).get(kind, 0))
            for kind in WORK_KINDS
        }
        for phase in sorted(phases)
    }
    totals = {
        kind: after["totals"][kind] - before["totals"][kind]
        for kind in WORK_KINDS
    }
    return {"totals": totals, "by_phase": by_phase}


class IncrementalMobilityCompiler:
    """Safe whole-pipeline incremental rebuild.

    Pair sandwiches are reused revision-by-revision.  Slab predicates,
    component lineage, mobility edges, and witnesses are deliberately derived
    again after every retained-pair change because one local obstacle can alter
    global topology.
    """

    def __init__(self, scene, robot, cfg, *, ledger=None):
        self.slices = IncrementalSliceCompiler(
            scene, robot, cfg, ledger=ledger
        )
        self._pending_changed = set(self.slices._oracles)

    def update_models(self, *, scene=None, robot=None) -> set[PairID]:
        changed = self.slices.update_models(scene=scene, robot=robot)
        self._pending_changed.update(changed)
        return changed

    def compile(self) -> IncrementalCompilationState:
        from ..mobility.graph import compile_mobility
        from ..orientation.slab_builder import build_slabs

        before_computed = self.slices.stats.envelopes_computed
        before_reused = self.slices.stats.envelopes_reused
        before_envelope_calls = self.slices.stats.envelope_support_calls
        before_work = self.slices.ledger.snapshot()
        scene, robot, cfg = self.slices.scene, self.slices.robot, self.slices.cfg
        with self.slices.ledger.phase("slab_compile"):
            decomposition = build_slabs(
                scene, robot, cfg, self.slices.oracles,
                slice_builder=self.slices.compile_slice,
                ledger=self.slices.ledger,
            )
        with self.slices.ledger.phase("mobility_witness"):
            mobility = compile_mobility(
                scene, robot, cfg, list(self.slices.oracles), decomposition,
                ledger=self.slices.ledger,
            )
        work_delta = _work_delta(before_work, self.slices.ledger.snapshot())
        state = IncrementalCompilationState(
            scene=scene,
            robot=robot,
            decomposition=decomposition,
            mobility=mobility,
            changed_pairs=tuple(sorted(
                self._pending_changed,
                key=lambda p: (p.scene_id, p.body_id),
            )),
            envelopes_computed=(self.slices.stats.envelopes_computed
                                - before_computed),
            envelopes_reused=(self.slices.stats.envelopes_reused
                              - before_reused),
            envelope_support_calls=(
                self.slices.stats.envelope_support_calls
                - before_envelope_calls
            ),
            support_calls=(
                work_delta["totals"]["support_value_evals"]
                + work_delta["totals"]["support_point_evals"]
            ),
            work_delta=work_delta,
        )
        self._pending_changed.clear()
        return state
