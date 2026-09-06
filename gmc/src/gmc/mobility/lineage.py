"""Component lineage links between adjacent slabs (Guide §9.3).

Adjacent slabs share their boundary slice object (cached by angle), so a
candidate link is a composition: mid(A) component -> boundary component ->
mid(B) component with positive overlap along the chain. Rotate-in-place
candidate points come from the eroded common region."""
from dataclasses import dataclass

import numpy as np
import shapely

from ..spatial.arrangement import overlap_matrix
from ..orientation.interval_certificate import IntervalPairCertificate
from ..types import CertStatus


@dataclass(frozen=True)
class CandidateLink:
    slab_a: int
    comp_a: int
    slab_b: int
    comp_b: int
    side: str                       # "safe" | "possible"
    common_points: tuple            # candidate rotate-in-place anchors
    reasons: tuple = ()
    geometrically_excluded: bool = False

    @property
    def is_geometrically_excluded(self) -> bool:
        """Whether geometry proves that this transition cannot exist.

        Missing a *single fixed rotation anchor* is only a witness failure.
        In particular, A may overlap a connected boundary component X and X
        may overlap B even when A ∩ X ∩ B is empty.  Such a lineage must
        remain in the possible graph.
        """
        return self.geometrically_excluded


def certified_cover_slice(slab):
    """Return the slab-wide interval cover only when it is certified.

    A cover slice has different semantics from the legacy midpoint slice:
    ``D_safe`` is free for *every* orientation in the slab and
    ``D_possible`` contains the free space at *every* orientation in the
    slab.  An incomplete/UNKNOWN cover must therefore never replace the
    midpoint prototype geometry.
    """
    cover = getattr(slab, "cover_slice", None)
    provenance = getattr(slab, "cover_provenance", {})
    certificates = tuple(getattr(cover, "sandwiches", ())) \
        if cover is not None else ()
    midpoint_pairs = tuple(
        sandwich.pair_id
        for sandwich in getattr(slab.mid_slice, "sandwiches", ())
    )
    certificate_pairs = tuple(
        certificate.pair_id for certificate in certificates
        if isinstance(certificate, IntervalPairCertificate)
    )
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
    valid = bool(
        cover is not None
        and cover is not slab.left_slice
        and cover is not slab.mid_slice
        and cover is not slab.right_slice
        and getattr(cover, "status", None) is CertStatus.CERTIFIED
        and provenance.get("status") == CertStatus.CERTIFIED.name
        and provenance.get("evidence_scope") == "entire_orientation_interval"
        and provenance.get("interval") == [
            float(slab.interval.lo), float(slab.interval.hi),
        ]
        and provenance.get("midpoint") == float(slab.interval.midpoint)
        and provenance.get("pair_count") == len(certificates)
        and provenance.get("certified_pair_count") == len(certificates)
        and len(certificates) == len(midpoint_pairs)
        and all(isinstance(certificate, IntervalPairCertificate)
                and certificate.status is CertStatus.CERTIFIED
                and certificate.interval == slab.interval
                for certificate in certificates)
        and certificate_pairs == midpoint_pairs
        and len(certificate_pairs) == len(set(certificate_pairs))
        # C_plus may be more conservative than the per-pair union after the
        # final outward precision pass, but it may never omit any certified
        # outer-cover point.  Omitting one would enlarge D_safe and could turn
        # a corrupted/rebound cover into a false SAFE claim.
        and outer_union.difference(cover.C_plus).is_empty
        # Conversely, C_minus may be smaller than the certified inner union
        # (which only enlarges the upper free-space bound), but must never add
        # obstacle area not owned by a pair certificate.
        and cover.C_minus.difference(inner_union).is_empty
        and all(component.status is CertStatus.CERTIFIED
                for component in (*cover.D_safe, *cover.D_possible))
    )
    return cover if valid else None


def component_slice(slab):
    """Geometry source used by mobility nodes for this slab."""
    return certified_cover_slice(slab) or slab.mid_slice


def slab_has_safe_components(slab) -> bool:
    """Whether the slab may own SAFE graph nodes.

    Certified interval covers carry their own all-theta safety proof, so they
    do not depend on the event detector's regularity classification.  With no
    cover, retain the original prototype rule exactly.
    """
    return certified_cover_slice(slab) is not None or slab.kind == "regular"


def _comps(slab, side):
    sl = component_slice(slab)
    return sl.D_safe if side == "safe" else sl.D_possible


def _anchor_candidates(common, erosions, max_anchors: int = 3):
    """One anchor per connected piece of the eroded common region (§9.3):
    a dumbbell-shaped overlap (two rooms + thin corridor) yields one anchor
    per lobe, which lets the lift place the crossing traverse at a wide
    orientation instead of only at event-adjacent slabs."""
    if common.is_empty:
        return []
    for ero in erosions:
        core = common.buffer(-ero) if ero > 0 else common
        if core.is_empty:
            continue
        pieces = sorted(getattr(core, "geoms", [core]),
                        key=lambda g: -g.area)[:max_anchors]
        return [tuple(np.asarray(g.representative_point().coords[0]))
                for g in pieces if not g.is_empty]
    return []


def _boundary_slice(slab_a, slab_b):
    """The shared endpoint slice (identical cached object when adjacent)."""
    if slab_a.right_slice is slab_b.left_slice:
        return slab_a.right_slice
    if slab_a.left_slice is slab_b.right_slice:
        return slab_a.left_slice
    return slab_a.right_slice


def candidate_links(slab_a, slab_b, side: str,
                    erosions=(0.05, 0.02, 0.0), *,
                    ledger=None) -> list[CandidateLink]:
    # Consecutive certified interval covers already bound the geometry over
    # their complete angle ranges.  A true path crossing their shared angle
    # must lie in both possible covers, so direct component intersection is
    # the sound upper-graph lineage.  Requiring a single fixed rotation
    # anchor here would be unsound for POSSIBLE: a continuously translating
    # path need not rotate in place at any one point.
    if (certified_cover_slice(slab_a) is not None
            and certified_cover_slice(slab_b) is not None):
        return _cover_links(slab_a, slab_b, side, erosions, ledger=ledger)

    ca, cb = _comps(slab_a, side), _comps(slab_b, side)
    boundary = _boundary_slice(slab_a, slab_b)
    cx = (boundary.D_safe if side == "safe" else boundary.D_possible)
    M_ax = overlap_matrix(ca, cx, ledger=ledger)
    M_xb = overlap_matrix(cx, cb, ledger=ledger)
    links = []
    for i in range(len(ca)):
        for j in range(len(cb)):
            through = [x for x in range(len(cx))
                       if M_ax[i, x] > 0.0 and M_xb[x, j] > 0.0]
            if not through:
                continue
            pts = []
            for x in through:
                if ledger is not None:
                    # The common-region chain performs two additional
                    # intersections beyond the A-X and X-B overlap matrices.
                    ledger.charge("intersection_tests", 2)
                common = (ca[i].geometry
                          .intersection(cx[x].geometry)
                          .intersection(cb[j].geometry))
                pts.extend(_anchor_candidates(common, erosions))
            no_anchor = not pts
            links.append(CandidateLink(
                slab_a=slab_a.slab_id, comp_a=i,
                slab_b=slab_b.slab_id, comp_b=j, side=side,
                common_points=tuple(map(tuple, pts)),
                reasons=(
                    "lineage_via_boundary_without_single_rotation_anchor",
                ) if no_anchor else (),
                # ``through`` is direct positive-overlap evidence for a
                # possible transition.  No fixed anchor is needed in the
                # upper-bound graph.
                geometrically_excluded=False))
    return links


def _cover_links(slab_a, slab_b, side: str, erosions, *, ledger=None):
    """Direct lineage between adjacent slab-wide certified covers."""
    ca, cb = _comps(slab_a, side), _comps(slab_b, side)
    links = []
    for i in range(len(ca)):
        for j in range(len(cb)):
            if ledger is not None:
                ledger.charge("intersection_tests")
            common = ca[i].geometry.intersection(cb[j].geometry)
            # Boundary-only contact is retained in M_possible.  It can be a
            # genuine zero-clearance connection under the closed collision
            # convention, and discarding it could create false UNREACHABLE.
            if common.is_empty:
                continue
            pts = _anchor_candidates(common, erosions)
            links.append(CandidateLink(
                slab_a=slab_a.slab_id, comp_a=i,
                slab_b=slab_b.slab_id, comp_b=j, side=side,
                common_points=tuple(map(tuple, pts)),
                reasons=(
                    "certified_interval_cover_component_intersection",
                    *(() if pts else (
                        "interval_lineage_without_single_rotation_anchor",
                    )),
                ),
                # Nonempty component intersection is positive lineage
                # evidence.  Anchor certification is deliberately separate.
                geometrically_excluded=False,
            ))
    return links


def candidate_links_direct(slab_a, slab_b, side: str,
                           erosions=(0.05, 0.02, 0.0), *,
                           ledger=None) -> list[CandidateLink]:
    """Event-edge candidates across an uncertain run (Guide §9.1 Event edge):
    direct mid-component intersection, no boundary-slice composition — the
    in-between combinatorics is exactly what is uncertain. The witness (a
    rotate-in-place over the whole theta span, certified independently by the
    support oracles) carries all the safety burden."""
    ca, cb = _comps(slab_a, side), _comps(slab_b, side)
    links = []
    for i in range(len(ca)):
        for j in range(len(cb)):
            if ledger is not None:
                ledger.charge("intersection_tests")
            common = ca[i].geometry.intersection(cb[j].geometry)
            if common.is_empty or common.area <= 0.0:
                continue
            pts = _anchor_candidates(common, erosions)
            links.append(CandidateLink(
                slab_a=slab_a.slab_id, comp_a=i,
                slab_b=slab_b.slab_id, comp_b=j, side=side,
                common_points=tuple(pts),
                reasons=("event_edge_candidate",)))
    return links
