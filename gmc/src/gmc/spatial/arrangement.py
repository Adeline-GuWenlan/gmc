"""Free-space components with descriptors (Guide §7.3)."""
from dataclasses import dataclass

import numpy as np
import shapely
from shapely.geometry import Polygon
from shapely.strtree import STRtree

from ..types import CertStatus, PairID
from ..geometry.predicates import polygon_components


@dataclass(frozen=True)
class FreeComponent:
    component_id: str
    geometry: Polygon
    representative: np.ndarray
    area: float
    boundary_signature: tuple
    causing_pairs: tuple            # tuple[PairID, ...]
    clearance_lb: float | None
    status: CertStatus


def extract_components(free_geom, sandwiches, side: str, theta: float,
                       area_min: float, status: CertStatus) -> list[FreeComponent]:
    parts = polygon_components(free_geom, area_min)
    if sandwiches:
        polys = [s.outer if side == "safe" else s.inner for s in sandwiches]
        tree = STRtree(polys)
    comps = []
    for k, p in enumerate(parts):
        causing = ()
        if sandwiches:
            hits = tree.query(p.boundary, predicate="dwithin", distance=1e-6)
            causing = tuple(sorted({sandwiches[i].pair_id for i in hits},
                                   key=lambda pid: (pid.scene_id, pid.body_id)))
        rep = p.representative_point()
        comps.append(FreeComponent(
            # 17 significant digits round-trip every binary64 value; shorter
            # formatting aliases distinct slices once theta_min is small.
            component_id=f"th{float(theta):.17g}_{side}_{k}",
            geometry=p,
            representative=np.array([rep.x, rep.y]),
            area=float(p.area),
            boundary_signature=tuple((c.scene_id, c.body_id) for c in causing),
            causing_pairs=causing,
            clearance_lb=None,
            status=status))
    return comps


def overlap_matrix(comps_a, comps_b, *, ledger=None) -> np.ndarray:
    """Pairwise intersection areas between two component lists (§9.3)."""
    M = np.zeros((len(comps_a), len(comps_b)))
    for i, a in enumerate(comps_a):
        for j, b in enumerate(comps_b):
            if ledger is not None:
                ledger.charge("intersection_tests")
            if a.geometry.intersects(b.geometry):
                M[i, j] = a.geometry.intersection(b.geometry).area
    return M


def match_components(comps_a, comps_b, rel_tol: float = 0.0, *,
                     ledger=None):
    """Greedy area matching. Returns (matches, unmatched_a, unmatched_b,
    bijective) where matches is a list of (i, j). Bijective means every
    component on both sides has exactly one significant partner — the
    lineage-unambiguous case (§8.2 P2)."""
    M = overlap_matrix(comps_a, comps_b, ledger=ledger)
    if not np.isfinite(rel_tol) or rel_tol < 0.0:
        raise ValueError("component overlap tolerance must be non-negative")
    sig = M > rel_tol
    matches = [(i, int(np.argmax(M[i]))) for i in range(len(comps_a))
               if sig[i].any()]
    unmatched_a = [i for i in range(len(comps_a)) if not sig[i].any()]
    unmatched_b = [j for j in range(len(comps_b)) if not sig[:, j].any()]
    bijective = (not unmatched_a and not unmatched_b
                 and all(sig[i].sum() == 1 for i in range(len(comps_a)))
                 and all(sig[:, j].sum() == 1 for j in range(len(comps_b))))
    return matches, unmatched_a, unmatched_b, bijective
