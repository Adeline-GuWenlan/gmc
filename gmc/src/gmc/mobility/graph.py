"""M6 dual mobility graph compilation (Guide §9.4).

Nodes: (slab_id, component_index) per side. A SAFE edge exists only with a
certified rotate-in-place witness across the shared boundary angle (I3/I4).
Legacy midpoint slabs retain the event-regularity gate; a certified interval
cover can own SAFE nodes independently because its geometry is valid over the
whole slab (I5)."""
from dataclasses import dataclass, field

import networkx as nx
import numpy as np

from ..types import CertStatus
from ..orientation.provenance import validate_decomposition_binding
from ..orientation.slab_builder import (adjacent_slab_pairs,
                                        decomposition_structure_failures)
from .lineage import (candidate_links, candidate_links_direct,
                      component_slice, slab_has_safe_components)
from .witness import rotate_witness


def node_id(slab_id: int, comp: int, side: str = "safe") -> str:
    """Side-qualified node ids: safe and possible components are sorted
    independently, so the two graphs must NOT share a namespace (an index
    collision here once fused left/right rooms in a sealed-door scene)."""
    return f"slab{slab_id:03d}_{side[0]}{comp}"


@dataclass
class MobilityCompiler:
    scene: object
    robot: object
    cfg: object
    oracles: list
    decomposition: object
    M_safe: nx.Graph = field(default_factory=nx.Graph)
    M_possible: nx.Graph = field(default_factory=nx.Graph)
    safe_to_possible: dict = field(default_factory=dict)
    # Complete candidate-link decisions, including SAFE witness failures and
    # already-covered POSSIBLE relations.  Graph edges alone cannot replay
    # those negative/non-promoted decisions.
    lineage_records: list = field(default_factory=list)
    graph_revision: int = 0
    # Only graphs produced by ``compile_mobility`` carry the full structural
    # contract.  Lightweight hand-built test doubles remain supported.
    structural_contract: bool = False

    def slab(self, slab_id: int):
        return self.decomposition.slabs[slab_id]

    def locate(self, pose, side: str = "safe"):
        """Deterministic half-open owner of ``pose``, or ``None``.

        No angular tolerance is admissible: it can jump across an arbitrarily
        narrow slab.  Formal cut reasoning uses :meth:`locate_all` instead so
        exact angular/spatial boundary memberships are all retained.
        """
        from ..orientation.intervals import TWO_PI
        th = pose.theta % TWO_PI
        for slab in self.decomposition.slabs:
            if not (slab.interval.lo <= th < slab.interval.hi):
                continue
            if side == "safe" and not slab_has_safe_components(slab):
                continue
            sl = component_slice(slab)
            comps = sl.D_safe if side == "safe" else sl.D_possible
            for k, c in enumerate(comps):
                if c.geometry.covers(shapely_point(pose.xy)):
                    return node_id(slab.slab_id, k, side), slab, k
        return None

    def locate_all(self, pose, side: str = "safe"):
        """All exact closed-set memberships used by formal connectivity cuts."""
        from ..orientation.intervals import TWO_PI

        if side not in {"safe", "possible"}:
            raise ValueError("side must be 'safe' or 'possible'")
        th = float(pose.theta) % TWO_PI
        # 0 and 2*pi are the same orientation.  At that exact periodic seam,
        # both the first and last closed interval covers are admissible.
        angles = (th, TWO_PI) if th == 0.0 else (th,)
        located = []
        for slab in sorted(
                self.decomposition.slabs,
                key=lambda item: item.interval.lo):
            if not any(slab.interval.lo <= angle <= slab.interval.hi
                       for angle in angles):
                continue
            if side == "safe" and not slab_has_safe_components(slab):
                continue
            sl = component_slice(slab)
            comps = sl.D_safe if side == "safe" else sl.D_possible
            point = shapely_point(pose.xy)
            for k, component in enumerate(comps):
                if component.geometry.covers(point):
                    located.append((
                        node_id(slab.slab_id, k, side), slab, k,
                    ))
        return located

    def possible_of_safe(self, slab_id: int, safe_comp: int) -> str:
        """Possible node containing a safe component (safe subset possible)."""
        return self.safe_to_possible[(slab_id, safe_comp)]


def shapely_point(xy):
    import shapely
    return shapely.Point(tuple(xy))


def compile_mobility(scene, robot, cfg, oracles, decomposition,
                     *, ledger=None, validate_oracles: bool = True) -> MobilityCompiler:
    if validate_oracles:
        from ..spatial.bvh import validate_candidate_oracles
        oracles = validate_candidate_oracles(
            scene, robot, scene.workspace, oracles)
    # ``validate_oracles=False`` is an explicit test/debug escape hatch for
    # synthetic lineage objects.  Real decompositions always carry an input
    # binding and remain validated even when refinement already validated the
    # canonical oracle set immediately before this rebuild.
    if validate_oracles or getattr(decomposition, "input_binding", None) is not None:
        validate_decomposition_binding(
            decomposition, scene, robot, cfg, oracles,
        )
        structure_failures = decomposition_structure_failures(
            decomposition, cfg,
        )
        if structure_failures:
            raise ValueError(
                "decomposition structure mismatch: "
                + ", ".join(structure_failures)
            )
    mc = MobilityCompiler(
        scene=scene, robot=robot, cfg=cfg, oracles=oracles,
        decomposition=decomposition,
        graph_revision=int(getattr(decomposition, "revision", 0)),
    )
    for slab in decomposition.slabs:
        sl = component_slice(slab)
        for k, _ in enumerate(sl.D_possible):
            mc.M_possible.add_node(node_id(slab.slab_id, k, "possible"),
                                   slab=slab.slab_id, comp=k,
                                   interval_cover=(sl is not slab.mid_slice))
        if slab_has_safe_components(slab):
            for k, comp in enumerate(sl.D_safe):
                mc.M_safe.add_node(node_id(slab.slab_id, k, "safe"),
                                   slab=slab.slab_id, comp=k,
                                   interval_cover=(sl is not slab.mid_slice))
                for j, pc in enumerate(sl.D_possible):
                    if pc.geometry.covers(shapely_point(comp.representative)):
                        mc.safe_to_possible[(slab.slab_id, k)] = \
                            node_id(slab.slab_id, j, "possible")
                        break

    # event edges: bridge the regular slabs flanking each maximal uncertain
    # run (Guide §9.1 "Event edge"); the rotate-in-place witness over the
    # whole span carries the safety burden independently of the uncertain
    # combinatorics in between.
    order = sorted(range(len(decomposition.slabs)),
                   key=lambda k: decomposition.slabs[k].interval.lo)
    slabs_sorted = [decomposition.slabs[k] for k in order]
    n = len(slabs_sorted)
    event_pairs = []
    for i, s in enumerate(slabs_sorted):
        if not slab_has_safe_components(s):
            continue
        j = (i + 1) % n
        crossed_uncertain = False
        while not slab_has_safe_components(slabs_sorted[j]) and j != i:
            crossed_uncertain = True
            j = (j + 1) % n
        if crossed_uncertain and j != i:
            event_pairs.append((s, slabs_sorted[j]))

    all_pairs = ([(A, B, False) for A, B in
                  adjacent_slab_pairs(decomposition.slabs, periodic=True)]
                 + [(A, B, True) for A, B in event_pairs])
    for A, B, is_event in all_pairs:
        for side in ("safe", "possible"):
            if side == "safe" and (not slab_has_safe_components(A)
                                    or not slab_has_safe_components(B)):
                continue
            maker = candidate_links_direct if is_event else candidate_links
            for link in maker(A, B, side, ledger=ledger):
                na = node_id(link.slab_a, link.comp_a, side)
                nb = node_id(link.slab_b, link.comp_b, side)
                th_a, th_b = A.interval.midpoint, B.interval.midpoint
                if th_b < th_a:                        # periodic wrap (I6)
                    th_b += 2.0 * np.pi
                witness = None
                if side == "safe" and link.common_points:
                    anchors = [t for t in link.common_points
                               if rotate_witness(t, th_a, th_b, mc.oracles,
                                                 cfg.orientation.theta_min
                                                 ).status is CertStatus.CERTIFIED]
                    if anchors:
                        witness = (anchors, th_a, th_b)
                weight = (abs(th_b - th_a)
                          * robot_radius(robot))
                record = {
                    "record_index": len(mc.lineage_records),
                    "relation_scope": (
                        "event_direct" if is_event
                        else "adjacent_orientation_intervals"
                    ),
                    "side": side,
                    "slab_a": int(link.slab_a),
                    "component_a": int(link.comp_a),
                    "node_a": na,
                    "slab_b": int(link.slab_b),
                    "component_b": int(link.comp_b),
                    "node_b": nb,
                    "common_points": tuple(link.common_points),
                    "reasons": tuple(link.reasons),
                    "geometrically_excluded": bool(
                        link.is_geometrically_excluded),
                    "theta_start": float(th_a),
                    "theta_end": float(th_b),
                    "weight": float(weight),
                }
                if side == "safe":
                    if witness is not None:
                        mc.M_safe.add_edge(na, nb, witness=witness,
                                           weight=weight, status="SAFE")
                        pa = mc.safe_to_possible.get(
                            (link.slab_a, link.comp_a))
                        pb = mc.safe_to_possible.get(
                            (link.slab_b, link.comp_b))
                        if pa is not None and pb is not None:
                            mc.M_possible.add_edge(pa, pb, witness=witness,
                                                   weight=weight,
                                                   status="SAFE")
                        record["outcome"] = "SAFE_EDGE_ADDED"
                        record["certified_anchor_count"] = len(witness[0])
                        record["possible_node_a"] = pa
                        record["possible_node_b"] = pb
                    else:
                        record["outcome"] = "SAFE_WITNESS_UNRESOLVED"
                        record["certified_anchor_count"] = 0
                else:
                    if not link.is_geometrically_excluded and \
                            not mc.M_possible.has_edge(na, nb):
                        mc.M_possible.add_edge(na, nb, witness=None,
                                               weight=weight,
                                               status="POSSIBLE",
                                               reasons=link.reasons)
                        record["outcome"] = "POSSIBLE_EDGE_ADDED"
                    elif link.is_geometrically_excluded:
                        record["outcome"] = "GEOMETRICALLY_EXCLUDED"
                    else:
                        record["outcome"] = "EXISTING_EDGE_PRESERVED"
                mc.lineage_records.append(record)
    mc.structural_contract = True
    return mc


def robot_radius(robot) -> float:
    return robot.max_rotational_radius()
