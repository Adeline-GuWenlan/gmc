"""Pose curves and SAFE witness primitives (Guide §9.2, §11.1).

The witness family (translate at fixed theta / rotate in place / TRT chains)
is sound but knowingly incomplete: candidates without a witness stay in
M_possible — they are never deleted from the truth bound (§9.2).
"""
from dataclasses import dataclass
from enum import Enum, auto

import numpy as np
import shapely

from ..types import CertStatus, Pose2, Result
from ..verification.continuous import rotation_interval_safe, translation_safe


class SegmentKind(Enum):
    TRANSLATION = auto()
    ROTATION = auto()
    LOCAL_STEERING = auto()


@dataclass(frozen=True)
class PoseSegment:
    kind: SegmentKind
    q0: Pose2
    q1: Pose2
    control_points: tuple = ()
    certificate_ids: tuple = ()


@dataclass(frozen=True)
class PoseCurve:
    segments: tuple

    @property
    def length(self) -> float:
        total = 0.0
        for s in self.segments:
            if s.kind is SegmentKind.TRANSLATION:
                total += float(np.linalg.norm(s.q1.xy - s.q0.xy))
            else:
                total += abs(s.q1.theta - s.q0.theta)
        return total


def _raster_polyline(poly, p0, p1, res: float):
    """Grid A* inside the polygon + line-of-sight smoothing (§9.2:
    visibility-graph polyline). Returns list of points or None."""
    import heapq
    minx, miny, maxx, maxy = poly.bounds
    xs = np.arange(minx + res / 2, maxx, res)
    ys = np.arange(miny + res / 2, maxy, res)
    if len(xs) == 0 or len(ys) == 0:
        return None
    X, Y = np.meshgrid(xs, ys)
    free = shapely.contains_xy(poly, X, Y)

    def nearest_cell(p):
        j = int(np.clip(round((p[0] - xs[0]) / res), 0, len(xs) - 1))
        i = int(np.clip(round((p[1] - ys[0]) / res), 0, len(ys) - 1))
        if free[i, j]:
            return (i, j)
        cand = np.argwhere(free)
        if len(cand) == 0:
            return None
        d = np.abs(cand[:, 0] - i) + np.abs(cand[:, 1] - j)
        i2, j2 = cand[int(np.argmin(d))]
        return (int(i2), int(j2)) if d.min() <= 3 else None

    a, b = nearest_cell(p0), nearest_cell(p1)
    if a is None or b is None:
        return None
    heap = [(0.0, a)]
    dist = {a: 0.0}
    prev = {}
    steps = [(-1, 0), (1, 0), (0, -1), (0, 1),
             (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while heap:
        d, cur = heapq.heappop(heap)
        if cur == b:
            break
        if d > dist.get(cur, np.inf):
            continue
        for di, dj in steps:
            i2, j2 = cur[0] + di, cur[1] + dj
            if not (0 <= i2 < len(ys) and 0 <= j2 < len(xs)) or not free[i2, j2]:
                continue
            if di and dj and not (free[cur[0], j2] and free[i2, cur[1]]):
                continue                       # no corner cutting
            nd = d + np.hypot(di, dj)
            if nd < dist.get((i2, j2), np.inf):
                dist[(i2, j2)] = nd
                prev[(i2, j2)] = cur
                heapq.heappush(heap, (nd, (i2, j2)))
    if b not in dist:
        return None
    cells = [b]
    while cells[-1] != a:
        cells.append(prev[cells[-1]])
    pts = [tuple(p0)] + [(xs[j], ys[i]) for i, j in reversed(cells)] + [tuple(p1)]
    # line-of-sight smoothing against the polygon
    out = [pts[0]]
    k = 0
    while k < len(pts) - 1:
        j = len(pts) - 1
        while j > k + 1 and not poly.covers(
                shapely.LineString([pts[k], pts[j]])):
            j -= 1
        out.append(pts[j])
        k = j
    return out


def translate_witness(component_geom, p0, p1, theta: float,
                      oracles, floor: float = 0.0) -> Result[PoseSegment]:
    """Straight segment, then a raster/visibility polyline inside the free
    component (§9.2 Translate). Polygon coverage only routes; soundness of
    every leg comes from translation_safe on the support oracles."""
    oracles = tuple(oracles)
    candidates = [[tuple(p0), tuple(p1)]]
    if not component_geom.covers(shapely.LineString(candidates[0])):
        candidates = []
        for res in (0.12, 0.06):
            pts = _raster_polyline(component_geom, p0, p1, res)
            if pts is not None:
                candidates.append(pts)
                break
    for pts in candidates:
        ok = all(translation_safe(oracles, np.array(a), np.array(b), theta,
                                  floor=floor)[0]
                 for a, b in zip(pts[:-1], pts[1:]))
        if ok:
            seg = PoseSegment(SegmentKind.TRANSLATION,
                              Pose2(np.array(pts[0]), theta),
                              Pose2(np.array(pts[-1]), theta),
                              control_points=tuple(Pose2(np.array(p), theta)
                                                   for p in pts[1:-1]))
            return Result(seg, CertStatus.CERTIFIED, "translate_ok")
    return Result(None, CertStatus.UNKNOWN, "no_translate_witness",
                  uncertainty_sources=("translate_candidates_exhausted",))


def rotate_witness(t, th0: float, th1: float, oracles,
                   theta_min: float, floor: float = 0.0) -> Result[PoseSegment]:
    ok, margin = rotation_interval_safe(oracles, t, th0, th1, theta_min,
                                        floor=floor)
    if ok:
        seg = PoseSegment(SegmentKind.ROTATION,
                          Pose2(np.asarray(t, float), th0),
                          Pose2(np.asarray(t, float), th1))
        return Result(seg, CertStatus.CERTIFIED, "rotate_ok",
                      diagnostics={"min_margin_lb": margin})
    return Result(None, CertStatus.UNKNOWN, "no_rotate_witness",
                  uncertainty_sources=("rotation_interval_unresolved",),
                  diagnostics={"min_margin_lb": margin})
