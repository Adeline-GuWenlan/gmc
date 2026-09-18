"""Amendment 3 P4: the pure part of the stage-timing report.

The user asked for the time of each stage "从降噪处理到真实的算法进行路径选择，再到证明路径的可行性、连通性，
再到证明算法本身的效率" -- scene prep, path selection, proving the path, and the efficiency of the algorithm.
:func:`stage_group` maps the nine P1e stages onto those questions. Path selection is split into
``compile`` and ``query`` because that split is the one that says how the cost is paid: once per map, or
once per start/goal pair.

Honesty rules this module enforces rather than leaves to the report text:

* :func:`power_fit` and :func:`linear_fit` refuse to fit fewer than :data:`MIN_FIT_POINTS` points and say
  so; the caller gives the table instead of a curve.
* every fit carries ``n`` and the x range it rests on, and :func:`day_capacity` reports how far beyond
  the largest measured point its answer lies and marks itself an extrapolation.
* :func:`row_from_run` keeps Amendment 2's runs (no ``timing`` block, only ``compile_seconds`` /
  ``query_seconds``) visibly different from timed runs, and marks an UNKNOWN run's query as a lower bound.
"""
import math

import numpy as np

from .timing import STAGES

MIN_FIT_POINTS = 4

_GROUPS = {"scene_load": "prep", "floor_replace": "prep", "project": "prep",
           "compile_pairs": "compile", "compile_slabs": "compile", "compile_mobility": "compile",
           "query": "query", "verify_curve": "prove", "replay3d": "prove"}
assert set(_GROUPS) == set(STAGES)

COMPILE_STAGES = ("compile_pairs", "compile_slabs", "compile_mobility")


def stage_group(stage):
    """``prep`` (per scene), ``compile`` (per map), ``query`` (per pair) or ``prove`` (per route)."""
    if stage not in _GROUPS:
        raise ValueError(f"unknown stage {stage!r}")
    return _GROUPS[stage]


# ------------------------------------------------------------------------------------ windows
def grow_window(base, area, bounds):
    """A window that contains ``base``, has area ``area`` and lies inside ``bounds``.

    Grows every side by the same margin; where that crosses a bound the excess is moved to the opposite
    side, so a window near a wall is shifted rather than shrunk. ``None`` when ``bounds`` cannot hold
    ``area`` with the base inside. Returns ``base`` unchanged when ``area`` is not larger than it.
    """
    x0, y0, x1, y1 = map(float, base)
    w0, h0 = x1 - x0, y1 - y0
    if area <= w0 * h0:
        return [x0, y0, x1, y1]
    # (w0 + 2d)(h0 + 2d) = area
    d = (-(w0 + h0) + math.sqrt((w0 + h0) ** 2 - 4.0 * (w0 * h0 - area))) / 4.0
    bx0, by0, bx1, by1 = map(float, bounds)

    def fit(lo, hi, blo, bhi):
        if hi - lo > bhi - blo + 1e-9:
            return None
        if lo < blo:
            hi, lo = hi + (blo - lo), blo
        if hi > bhi:
            lo, hi = lo - (hi - bhi), bhi
        return lo, hi

    fx = fit(x0 - d, x1 + d, bx0, bx1)
    fy = fit(y0 - d, y1 + d, by0, by1)
    if fx is None or fy is None:
        return None
    return [round(fx[0], 4), round(fy[0], 4), round(fx[1], 4), round(fy[1], 4)]


# ------------------------------------------------------------------------------------ fits
def _clean(x, y):
    x, y = np.asarray(x, float).ravel(), np.asarray(y, float).ravel()
    ok = np.isfinite(x) & np.isfinite(y) & (x > 0) & (y > 0)
    return x[ok], y[ok]


def power_fit(x, y, min_points=MIN_FIT_POINTS):
    """Least squares ``log y = log a + b log x``. Refuses (``exponent=None``) below ``min_points``."""
    x, y = _clean(x, y)
    n = int(len(x))
    out = {"n": n, "model": "y = coef * x^exponent (log-log least squares)",
           "x_range": [float(x.min()), float(x.max())] if n else None,
           "y_range": [float(y.min()), float(y.max())] if n else None}
    if n < min_points or np.unique(x).size < 2:
        out.update(exponent=None, coef=None, r2=None,
                   refused=f"n = {n} < {min_points}: no fit, give the table instead")
        return out
    lx, ly = np.log(x), np.log(y)
    b, la = np.polyfit(lx, ly, 1)
    pred = la + b * lx
    ss_res = float(((ly - pred) ** 2).sum())
    ss_tot = float(((ly - ly.mean()) ** 2).sum())
    out.update(exponent=float(b), coef=float(math.exp(la)),
               r2=float(1.0 - ss_res / ss_tot) if ss_tot > 0 else 1.0, refused=None)
    return out


def linear_fit(x, y, min_points=MIN_FIT_POINTS):
    """Ordinary least squares ``y = intercept + slope * x``. Refuses (``slope=None``) below ``min_points``."""
    x, y = _clean(x, y)
    n = int(len(x))
    out = {"n": n, "model": "y = intercept + slope * x",
           "x_range": [float(x.min()), float(x.max())] if n else None}
    if n < min_points or np.unique(x).size < 2:
        out.update(slope=None, intercept=None, r2=None,
                   refused=f"n = {n} < {min_points}: no fit, give the table instead")
        return out
    slope, icpt = np.polyfit(x, y, 1)
    pred = icpt + slope * x
    ss_res = float(((y - pred) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    out.update(slope=float(slope), intercept=float(icpt),
               r2=float(1.0 - ss_res / ss_tot) if ss_tot > 0 else 1.0, refused=None)
    return out


def day_capacity(fit, seconds=86400.0):
    """The ``x`` at which a :func:`power_fit` reaches ``seconds``. Always labelled an extrapolation."""
    out = {"seconds": float(seconds), "extrapolation": True, "fit_n": fit.get("n"),
           "fit_x_range": fit.get("x_range")}
    if fit.get("exponent") is None:
        out.update(n_supports=None, beyond_largest_measured_x=None,
                   note="no fit (too few points), so no capacity")
        return out
    n = (float(seconds) / fit["coef"]) ** (1.0 / fit["exponent"])
    out.update(n_supports=float(n), beyond_largest_measured_x=float(n / fit["x_range"][1]))
    return out


# ------------------------------------------------------------------------------------ run rows
def _area(w):
    return float((w[2] - w[0]) * (w[3] - w[1]))


def row_from_run(d, *, source, label, path=None):
    """One normalized row from a ``showcase_run.py`` result JSON (timed or Amendment 2)."""
    case = d.get("case") or {}
    res = d.get("result") or {}
    ver = res.get("verify") or {}
    rep = d.get("replay3d") or res.get("replay3d") or {}
    timing = d.get("timing")
    st, gl = case.get("start"), case.get("goal")
    status = res.get("status")
    certified = bool(ver.get("certified", False))
    replay_ok = bool(rep.get("passed", False))
    return {
        "source": source, "label": label, "path": path, "robot": d.get("robot"), "mode": "single",
        "scene": d.get("scene", "processed"), "budget_scale": d.get("budget_scale", 1),
        "window": case.get("window"),
        "window_area_m2": _area(case["window"]) if case.get("window") else None,
        "start": st, "goal": gl,
        "ab_dist_m": float(math.hypot(gl[0] - st[0], gl[1] - st[1])) if st and gl else None,
        "n_supports": res.get("n_supports", (d.get("projection") or {}).get("kept")),
        "status": status, "reason": res.get("reason"),
        "certified": certified, "replay3d_passed": replay_ok,
        "success": bool(status == "REACHABLE" and certified and replay_ok),
        "clearance_lb": res.get("clearance_lb"),
        "replay3d_lb": rep.get("min_clearance_lb"),
        "compile_seconds": res.get("compile_seconds"), "query_seconds": res.get("query_seconds"),
        "query_is_lower_bound": status == "UNKNOWN",
        "has_stage_timing": timing is not None,
        "stages": dict(timing["by_stage"]) if timing is not None else None,
    }


def compile_total(stages):
    """``compile_pairs + compile_slabs + compile_mobility``: what ``compile_seconds`` measures."""
    if not stages or not any(k in stages for k in COMPILE_STAGES):
        return None
    return float(sum(stages.get(k, 0.0) for k in COMPILE_STAGES))


def row_from_sweep(u):
    """One normalized row from a ``plane_timing.py`` sweep unit (same fields as :func:`row_from_run`).

    A warm query has no compile of its own (``compile_seconds`` is ``None``): its map was compiled once,
    in the unit whose ``mode`` is ``warm_compile``.
    """
    st = u.get("stages") or {}
    qr = u.get("query_report") or {}
    return {
        "source": u.get("source", "P4-sweep"), "label": f"{u.get('experiment')}:{u.get('mode')}",
        "path": None, "robot": u.get("robot"), "mode": u.get("mode"), "experiment": u.get("experiment"),
        "lane": u.get("lane"), "scene": "planefloor", "budget_scale": 1,
        "window": u.get("window"), "window_area_m2": u.get("window_area_m2"),
        "start": u.get("start"), "goal": u.get("goal"), "ab_dist_m": u.get("ab_dist_m"),
        "n_supports": u.get("n_supports"), "status": u.get("status"), "reason": u.get("reason"),
        "certified": bool(u.get("certified", False)), "replay3d_passed": bool(u.get("replay3d_passed", False)),
        "success": bool(u.get("success", False)), "clearance_lb": u.get("clearance_lb"),
        "replay3d_lb": u.get("replay3d_lb"),
        "compile_seconds": compile_total(st), "query_seconds": st.get("query"),
        "query_is_lower_bound": bool(u.get("query_is_lower_bound", False)),
        "has_stage_timing": True, "stages": dict(st),
        "refinement_rounds": qr.get("refinement_rounds"),
        "query_support_calls": qr.get("query_support_calls"),
        "order": u.get("order"),
    }
