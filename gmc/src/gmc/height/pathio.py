"""PoseCurve <-> path.json, and conservative sampling of the swept motion."""
import json
import math
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, Point

from ..mobility.witness import PoseCurve, PoseSegment, SegmentKind
from ..types import Pose2


def _pose(v) -> Pose2:
    return Pose2(np.asarray(v[:2], dtype=float), float(v[2]))


def _vals(p: Pose2) -> list:
    return [*map(float, p.xy), float(p.theta)]


def curve_to_dict(curve, query_id="") -> dict:
    return {"schema_version": 2, "query_id": query_id, "segments": [{
        "kind": s.kind.name, "q0": _vals(s.q0), "q1": _vals(s.q1),
        "control_points": [_vals(p) for p in s.control_points],
        "certificate_ids": list(s.certificate_ids)} for s in curve.segments]}


def curve_from_dict(d) -> PoseCurve:
    segs = []
    for s in d["segments"]:
        kind = SegmentKind[s["kind"]]
        if kind is SegmentKind.LOCAL_STEERING:
            raise ValueError("LOCAL_STEERING segments are not supported by height replay")
        segs.append(PoseSegment(kind, _pose(s["q0"]), _pose(s["q1"]),
                                tuple(_pose(p) for p in s.get("control_points", [])),
                                tuple(s.get("certificate_ids", []))))
    return PoseCurve(tuple(segs))


def save_path_json(curve, path, query_id=""):
    Path(path).write_text(json.dumps(curve_to_dict(curve, query_id), indent=2))


def load_path_json(path) -> PoseCurve:
    return curve_from_dict(json.loads(Path(path).read_text()))


def sample_curve(curve, max_step, radius) -> np.ndarray:
    rows = []
    for s in curve.segments:
        a, b = np.array(_vals(s.q0)), np.array(_vals(s.q1))
        move = (float(np.linalg.norm(b[:2] - a[:2]))
                + abs(b[2] - a[2]) * max(float(radius), 0.0))
        n = max(1, math.ceil(move / float(max_step)))
        t = np.linspace(0.0, 1.0, n + 1)[:, None]
        pts = a + t * (b - a)
        rows.append(pts if not rows else pts[1:])
    return np.vstack(rows)


def polyline_xy(curve) -> np.ndarray:
    pts = [curve.segments[0].q0.xy] + [s.q1.xy for s in curve.segments]
    return np.asarray(pts, dtype=float)


def path_crosses(curve, polygon) -> bool:
    xy = polyline_xy(curve)
    geom = LineString(xy) if len(np.unique(xy, axis=0)) > 1 else Point(xy[0])
    return bool(geom.intersects(polygon))


def crossing_thetas(curve, x0) -> list:
    out = []
    for s in curve.segments:
        if s.kind is SegmentKind.TRANSLATION:
            xa, xb = float(s.q0.xy[0]), float(s.q1.xy[0])
            if min(xa, xb) <= x0 <= max(xa, xb) and xa != xb:
                out.append(float(s.q0.theta))
    return out
