"""Conservative scene-support BVH for pair pruning (Guide section 6.1).

Each scene Gaussian is represented by a covariance-aware bounding disc whose
radius is ``level * sqrt(lambda_max(covariance))``.  An internal-node disc
contains every child support disc.  For a robot primitive, that node disc is
expanded by the primitive's full rotational extent before it is tested against
the workspace.  Consequently, pruning an internal node cannot discard a pair
that the original flat predicate would retain.

``candidate_pairs`` deliberately keeps its original three-argument API and
ordering.  Callers that want to reuse the hierarchy or inspect pruning work can
construct :class:`SceneSupportBVH` and use :meth:`SceneSupportBVH.query`.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from shapely.geometry import Point

from ..geometry.support import PairOracle
from ..types import PairID


_ROUNDING_SLACK = 32.0 * np.finfo(np.float64).eps


@dataclass(frozen=True, slots=True)
class BoundingDisc:
    """A conservative Euclidean disc used by the hierarchy."""

    center: np.ndarray
    radius: float

    def __post_init__(self):
        center = np.asarray(self.center, dtype=np.float64).copy()
        if center.shape != (2,):
            raise ValueError("bounding-disc center must have shape (2,)")
        center.flags.writeable = False
        object.__setattr__(self, "center", center)
        object.__setattr__(self, "radius", float(self.radius))


@dataclass(frozen=True, slots=True)
class BVHQueryStats:
    """Work counters for one robot/workspace query.

    Counts for nodes and leaves are accumulated over all robot primitives.
    ``pair_tests`` is the number of exact legacy leaf predicates evaluated;
    ``pruned_pairs`` counts the predicates skipped by internal-node pruning.
    """

    scene_supports: int
    body_supports: int
    total_pairs: int
    tree_nodes: int
    tree_depth: int
    nodes_visited: int
    nodes_pruned: int
    leaves_visited: int
    pair_tests: int
    pruned_pairs: int
    candidate_pairs: int

    @property
    def pair_test_fraction(self) -> float:
        return self.pair_tests / self.total_pairs if self.total_pairs else 0.0


@dataclass(frozen=True, slots=True)
class CandidatePairQuery:
    """Candidate oracles together with the work needed to obtain them."""

    oracles: tuple[PairOracle, ...]
    stats: BVHQueryStats
    decisions: tuple["PairPruningRecord", ...] = ()


@dataclass(frozen=True, slots=True)
class PairPruningRecord:
    """One replayable decision for one scene/body Cartesian pair.

    An internal-node rejection records the conservative node disc and the
    selected descendant support's containment in that disc.  A leaf decision
    records the support disc itself.  Thus every pair is represented exactly
    once without rerunning all rejected leaf predicates merely for logging.
    """

    record_id: str
    scene_index: int
    body_index: int
    scene_id: int
    body_id: int
    decision: str                 # "RETAINED" | "REJECTED"
    proof_scope: str              # "internal_node" | "leaf_support_disc"
    bound_center: tuple[float, float]
    bound_radius: float
    descendant_center_distance: float
    descendant_radius: float
    descendant_containment_margin: float
    workspace_expansion: float
    body_extent: float
    workspace_distance: float
    threshold: float
    coordinate_scale: float
    coordinate_ulp: float
    rounding_slack: float
    comparison_rhs: float
    finite_comparison: bool
    reason: str


class CandidateOracleList(list):
    """List-compatible candidate set bound to the models that produced it.

    The public compatibility API historically returned a mutable list.  Keep
    that surface, but retain enough provenance to detect deletion, injection,
    or reuse with a different scene/robot before any certified compilation.
    """

    def __init__(self, values, scene, robot, workspace, *, decisions=(),
                 stats=None):
        super().__init__(values)
        self._scene = scene
        self._robot = robot
        self._workspace_wkb = bytes(workspace.wkb)
        self._expected_pair_ids = tuple(oracle.pair_id for oracle in self)
        self._pruning_decisions = tuple(decisions)
        self._bvh_stats = stats

    def matches(self, scene, robot, workspace) -> bool:
        if (scene is not self._scene or robot is not self._robot
                or bytes(workspace.wkb) != self._workspace_wkb
                or tuple(oracle.pair_id for oracle in self)
                != self._expected_pair_ids):
            return False
        scene_by_id = {item.primitive_id: item for item in scene.supports}
        body_by_id = {item.primitive_id: item for item in robot.supports}
        return all(
            scene_by_id.get(oracle.pair_id.scene_id) is oracle.scene
            and body_by_id.get(oracle.pair_id.body_id) is oracle.body
            for oracle in self
        )


@dataclass(frozen=True, slots=True)
class _BVHNode:
    bounds: BoundingDisc
    count: int
    indices: tuple[int, ...] = ()
    left: _BVHNode | None = None
    right: _BVHNode | None = None

    @property
    def is_leaf(self) -> bool:
        return self.left is None and self.right is None


def _cluster_bounding_disc(means_all: np.ndarray, radii_all: np.ndarray,
                           indices: tuple[int, ...]) -> BoundingDisc:
    """Return a disc containing every covariance-aware support disc.

    The center starts at the midpoint of the covariance-inflated axis-aligned
    bounds.  The final radius is recomputed against every child disc, so the
    AABB is only a center heuristic and cannot weaken conservativeness.
    """
    selected = np.fromiter(indices, dtype=np.intp, count=len(indices))
    means = means_all[selected]
    radii = radii_all[selected]
    low = np.min(means - radii[:, None], axis=0)
    high = np.max(means + radii[:, None], axis=0)
    center = low + 0.5 * (high - low)
    radius = float(np.max(np.linalg.norm(means - center, axis=1) + radii))
    # Inflate by one representable step so a rounded construction remains an
    # over-bound rather than accidentally becoming a one-ulp under-bound.
    radius = float(np.nextafter(radius, np.inf))
    return BoundingDisc(center, radius)


def _strictly_disjoint(workspace, bounds: BoundingDisc,
                       extra_radius: float) -> bool:
    """Whether ``workspace`` is provably outside an expanded node disc."""
    return bool(_disjoint_audit(workspace, bounds, extra_radius)["disjoint"])


def _disjoint_audit(workspace, bounds: BoundingDisc,
                    extra_radius: float) -> dict:
    """Return the complete directed-rounding decision used by BVH pruning."""
    distance = float(workspace.distance(Point(bounds.center)))
    threshold = bounds.radius + float(extra_radius)
    if not np.isfinite(distance) or not np.isfinite(threshold):
        return {
            "disjoint": False,
            "distance": distance,
            "threshold": float(threshold),
            "coordinate_scale": float("nan"),
            "coordinate_ulp": float("nan"),
            "slack": float("nan"),
            "comparison_rhs": float("nan"),
            "finite": False,
            "reason": "nonfinite_distance_or_threshold_fail_include",
        }
    # GEOS evaluates distance in the supplied world frame.  Its absolute
    # roundoff therefore also depends on the coordinate magnitude, not merely
    # on the (possibly tiny) distance/threshold.  Include a conservative
    # coordinate-quantisation term; when a separation is not resolvable at
    # this scale the only sound pruning verdict is "keep".
    workspace_bounds = np.asarray(workspace.bounds, dtype=float)
    coordinate_scale = max(
        1.0,
        float(np.max(np.abs(workspace_bounds), initial=0.0)),
        float(np.max(np.abs(bounds.center), initial=0.0)),
    )
    coordinate_ulp = float(abs(np.spacing(coordinate_scale)))
    if not np.isfinite(coordinate_ulp):
        return {
            "disjoint": False,
            "distance": distance,
            "threshold": float(threshold),
            "coordinate_scale": float(coordinate_scale),
            "coordinate_ulp": coordinate_ulp,
            "slack": float("nan"),
            "comparison_rhs": float("nan"),
            "finite": False,
            "reason": "nonfinite_coordinate_ulp_fail_include",
        }
    # Point/segment distance uses coordinate differences, products and a
    # square root.  Sixty-four coordinate ulps safely cover those fixed-size
    # operations and the two-dimensional norm while remaining negligible in
    # ordinary-scale scenes.
    coordinate_slack = 64.0 * np.sqrt(2.0) * coordinate_ulp
    slack = max(
        _ROUNDING_SLACK * max(1.0, abs(distance), abs(threshold)),
        coordinate_slack,
    )
    rhs = float(np.nextafter(threshold + slack, np.inf))
    disjoint = bool(distance > rhs)
    return {
        "disjoint": disjoint,
        "distance": distance,
        "threshold": float(threshold),
        "coordinate_scale": float(coordinate_scale),
        "coordinate_ulp": coordinate_ulp,
        "slack": float(slack),
        "comparison_rhs": rhs,
        "finite": bool(np.all(np.isfinite([
            distance, threshold, coordinate_scale, coordinate_ulp,
            slack, rhs,
        ]))),
        "reason": (
            "strict_distance_exceeds_conservative_threshold"
            if disjoint else "not_provably_disjoint_fail_include"
        ),
    }


def pair_bounding_disc(oracle: PairOracle):
    """Theta-independent disc containing ``O_ij^theta`` for every theta.

    The center is ``mu_i`` and the radius is the body-center orbit plus the
    covariance-aware circumradii of the scene and body supports.  The tuple
    return type is retained for compatibility with the v0 implementation.
    """
    radius = (float(np.linalg.norm(oracle.body.mean))
              + oracle.scene.bounding_radius()
              + oracle.body.bounding_radius())
    return oracle.scene.mean, radius


class SceneSupportBVH:
    """Reusable hierarchy over the supports of one scene.

    Splits use covariance-inflated cluster extents, while every node bound is
    independently tightened to contain all of its descendant support discs.
    Query results implement the legacy predicate algebraically, in the same
    scene-major/body-minor order, without asking GEOS to materialise a tiny
    buffer around a large-coordinate workspace.
    """

    def __init__(self, supports, *, leaf_size: int = 8):
        if int(leaf_size) != leaf_size or leaf_size < 1:
            raise ValueError("leaf_size must be a positive integer")
        self.supports = tuple(supports)
        self.leaf_size = int(leaf_size)
        self._means = np.asarray(
            [support.mean for support in self.supports], dtype=np.float64
        ).reshape((-1, 2))
        # Covariance eigenvalues are scene-support invariants.  Precomputing
        # them once avoids repeating eigvalsh at every ancestor in the tree.
        self._radii = np.asarray(
            [support.bounding_radius() for support in self.supports],
            dtype=np.float64,
        )
        self._points = tuple(Point(mean) for mean in self._means)
        self._node_count = 0
        self._depth = 0
        indices = tuple(range(len(self.supports)))
        self._root = self._build(indices, depth=1) if indices else None

    @classmethod
    def from_scene(cls, scene, *, leaf_size: int = 8) -> SceneSupportBVH:
        return cls(scene.supports, leaf_size=leaf_size)

    @property
    def node_count(self) -> int:
        return self._node_count

    @property
    def depth(self) -> int:
        return self._depth

    @property
    def bounds(self) -> BoundingDisc | None:
        return self._root.bounds if self._root is not None else None

    def matches_scene(self, scene) -> bool:
        """Return whether this index was built from the scene's same objects."""
        supports = tuple(scene.supports)
        return (len(supports) == len(self.supports)
                and all(a is b for a, b in zip(supports, self.supports)))

    def _build(self, indices: tuple[int, ...], depth: int) -> _BVHNode:
        self._node_count += 1
        self._depth = max(self._depth, depth)
        bounds = _cluster_bounding_disc(self._means, self._radii, indices)
        if len(indices) <= self.leaf_size:
            return _BVHNode(bounds=bounds, count=len(indices), indices=indices)

        selected = np.fromiter(indices, dtype=np.intp, count=len(indices))
        means = self._means[selected]
        radii = self._radii[selected]
        inflated_low = np.min(means - radii[:, None], axis=0)
        inflated_high = np.max(means + radii[:, None], axis=0)
        axis = int(np.argmax(inflated_high - inflated_low))
        ordered = tuple(sorted(indices,
                               key=lambda i: (float(self.supports[i].mean[axis]),
                                              i)))
        middle = len(ordered) // 2
        left = self._build(ordered[:middle], depth + 1)
        right = self._build(ordered[middle:], depth + 1)
        return _BVHNode(bounds=bounds, count=len(indices), indices=indices,
                        left=left, right=right)

    def query(self, robot, workspace) -> CandidatePairQuery:
        """Return conservative pair candidates and detailed pruning counters."""
        body_supports = tuple(robot.supports)
        total_pairs = len(self.supports) * len(body_supports)
        selected: list[tuple[int, int]] = []
        nodes_visited = 0
        nodes_pruned = 0
        leaves_visited = 0
        pair_tests = 0
        pruned_pairs = 0
        decisions: list[PairPruningRecord] = []

        if self._root is not None and body_supports:
            # dist(W (+) rB, p) <= q is equivalent to dist(W, p) <= q+r.
            # Evaluate the latter.  GEOS can collapse ``workspace.buffer(r)``
            # to EMPTY when coordinates are large relative to r; its distance
            # then becomes NaN and the old comparison silently dropped every
            # colliding pair.  `_strictly_disjoint` fail-includes every
            # non-finite/unresolved distance.
            workspace_expansion = float(robot.max_rotational_radius())

            for body_index, body in enumerate(body_supports):
                body_extent = (float(np.linalg.norm(body.mean))
                               + body.bounding_radius())

                def record(scene_index: int, node: _BVHNode, audit: dict,
                           *, proof_scope: str) -> None:
                    support = self.supports[scene_index]
                    descendant_distance = float(np.linalg.norm(
                        np.asarray(support.mean, dtype=float)
                        - np.asarray(node.bounds.center, dtype=float)
                    ))
                    descendant_radius = float(self._radii[scene_index])
                    containment_margin = float(np.nextafter(
                        node.bounds.radius
                        - descendant_distance - descendant_radius,
                        -np.inf,
                    ))
                    decisions.append(PairPruningRecord(
                        record_id=(
                            f"pair_pruning[{scene_index},{body_index}]"
                        ),
                        scene_index=int(scene_index),
                        body_index=int(body_index),
                        scene_id=int(support.primitive_id),
                        body_id=int(body.primitive_id),
                        decision=(
                            "REJECTED" if audit["disjoint"]
                            else "RETAINED"
                        ),
                        proof_scope=proof_scope,
                        bound_center=(float(node.bounds.center[0]),
                                      float(node.bounds.center[1])),
                        bound_radius=float(node.bounds.radius),
                        descendant_center_distance=descendant_distance,
                        descendant_radius=descendant_radius,
                        descendant_containment_margin=containment_margin,
                        workspace_expansion=workspace_expansion,
                        body_extent=float(body_extent),
                        workspace_distance=float(audit["distance"]),
                        threshold=float(audit["threshold"]),
                        coordinate_scale=float(audit["coordinate_scale"]),
                        coordinate_ulp=float(audit["coordinate_ulp"]),
                        rounding_slack=float(audit["slack"]),
                        comparison_rhs=float(audit["comparison_rhs"]),
                        finite_comparison=bool(audit["finite"]),
                        reason=str(audit["reason"]),
                    ))

                def visit(node: _BVHNode):
                    nonlocal nodes_visited, nodes_pruned, leaves_visited
                    nonlocal pair_tests, pruned_pairs
                    nodes_visited += 1
                    node_audit = _disjoint_audit(
                        workspace, node.bounds,
                        workspace_expansion + body_extent,
                    )
                    if node_audit["disjoint"]:
                        nodes_pruned += 1
                        pruned_pairs += node.count
                        for scene_index in node.indices:
                            record(
                                scene_index, node, node_audit,
                                proof_scope="internal_node",
                            )
                        return
                    if node.is_leaf:
                        leaves_visited += 1
                        for scene_index in node.indices:
                            pair_tests += 1
                            scene_support = self.supports[scene_index]
                            leaf = BoundingDisc(
                                scene_support.mean,
                                float(self._radii[scene_index]),
                            )
                            leaf_node = _BVHNode(
                                bounds=leaf, count=1,
                                indices=(scene_index,),
                            )
                            leaf_audit = _disjoint_audit(
                                workspace, leaf,
                                workspace_expansion + body_extent,
                            )
                            record(
                                scene_index, leaf_node, leaf_audit,
                                proof_scope="leaf_support_disc",
                            )
                            if not leaf_audit["disjoint"]:
                                selected.append((scene_index, body_index))
                        return
                    # Internal nodes are always binary by construction.
                    visit(node.left)  # type: ignore[arg-type]
                    visit(node.right)  # type: ignore[arg-type]

                visit(self._root)

        selected.sort()
        oracles = tuple(
            PairOracle(PairID(self.supports[scene_index].primitive_id,
                              body_supports[body_index].primitive_id),
                       self.supports[scene_index], body_supports[body_index])
            for scene_index, body_index in selected
        )
        stats = BVHQueryStats(
            scene_supports=len(self.supports),
            body_supports=len(body_supports),
            total_pairs=total_pairs,
            tree_nodes=self.node_count,
            tree_depth=self.depth,
            nodes_visited=nodes_visited,
            nodes_pruned=nodes_pruned,
            leaves_visited=leaves_visited,
            pair_tests=pair_tests,
            pruned_pairs=pruned_pairs,
            candidate_pairs=len(oracles),
        )
        decisions.sort(key=lambda item: (item.scene_index, item.body_index))
        if len(decisions) != total_pairs:
            raise RuntimeError(
                "BVH did not emit exactly one decision per Cartesian pair"
            )
        return CandidatePairQuery(
            oracles=oracles, stats=stats, decisions=tuple(decisions),
        )


def query_candidate_pairs(scene, robot, workspace, *,
                          index: SceneSupportBVH | None = None,
                          leaf_size: int = 8) -> CandidatePairQuery:
    """Query candidates with statistics, optionally reusing a scene index."""
    if index is None:
        index = SceneSupportBVH.from_scene(scene, leaf_size=leaf_size)
    elif not index.matches_scene(scene):
        raise ValueError("the supplied BVH was built for a different scene")
    return index.query(robot, workspace)


def candidate_pairs(scene, robot, workspace) -> CandidateOracleList:
    """Compatibility wrapper returning only pair oracles."""
    result = query_candidate_pairs(scene, robot, workspace)
    return CandidateOracleList(
        result.oracles, scene, robot, workspace,
        decisions=result.decisions, stats=result.stats,
    )


def validate_candidate_oracles(scene, robot, workspace, oracles):
    """Return a provenance-bound complete candidate set or raise.

    A caller-supplied empty/subset/stale list can otherwise compile obstacles
    away and fool the downstream verifier using the same incomplete list.
    Unbound legacy sequences are accepted only after an independent BVH
    recomputation proves exact ordered PairID and support-object identity.
    """
    if isinstance(oracles, CandidateOracleList) and oracles.matches(
            scene, robot, workspace):
        return oracles
    supplied = list(oracles)
    expected = candidate_pairs(scene, robot, workspace)
    if tuple(oracle.pair_id for oracle in supplied) != tuple(
            oracle.pair_id for oracle in expected):
        raise ValueError(
            "candidate oracle set is incomplete, stale, or non-canonical")
    for actual, reference in zip(supplied, expected):
        if actual.scene is not reference.scene or actual.body is not reference.body:
            raise ValueError("candidate oracle provenance does not match models")
    return CandidateOracleList(
        supplied, scene, robot, workspace,
        decisions=expected._pruning_decisions,
        stats=expected._bvh_stats,
    )
