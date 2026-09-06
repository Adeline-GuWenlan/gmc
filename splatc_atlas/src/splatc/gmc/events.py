"""G2 event layer: certified gate-interval finder over orientation.

Principle. The slice system rotates rigidly (slice.robot_support_poly), so its
forbidden set is Hausdorff-Lipschitz in theta with exact rate
L = max robot-polygon vertex radius. build_slice(perturb=+eps) shrinks free
space by a uniform eps margin (obstacles dilated, workspace eroded);
perturb=-eps grows it. Hence:

  verdict OPEN   survives perturb=+eps  =>  OPEN   for all |dtheta| <= eps/L
  verdict CLOSED survives perturb=-eps  =>  CLOSED for all |dtheta| <= eps/L

Each probe therefore certifies a verdict-constant interval around it. Covering
the period with such intervals squeezes every topology event into a bracket of
width <= tol — with a certificate that NO event hides inside any certified
interval. A uniform sweep offers no such certificate at any budget.

Certificates hold for the polygon system; its distance to ground truth is the
G1 two-sided sandwich (see slice.py / worklog gmc_G1.md).

Cost accounting: every build_slice call counts one "slice-equivalent" — the
budget unit shared with uniform-sweep baselines.
"""
from dataclasses import dataclass, field

import numpy as np

from .geometry import circum_factor
from .slice import build_slice, probes_connected

DEFAULT_EPS_LEVELS = (0.08, 0.04, 0.02, 0.01, 0.005)


@dataclass
class GateEventReport:
    period: float
    tol: float
    intervals: list = field(default_factory=list)   # (lo, hi, open: bool)
    brackets: list = field(default_factory=list)    # (lo, hi, v_lo, v_hi)
    unresolved: list = field(default_factory=list)  # (lo, hi) same-verdict, < tol
    n_slices: int = 0

    def open_intervals(self):
        return merge_intervals([(a, b) for a, b, v in self.intervals if v])

    def event_angles(self):
        return [0.5 * (a + b) for a, b, *_ in self.brackets]


def merge_intervals(iv):
    iv = sorted(iv)
    out = []
    for a, b in iv:
        if out and a <= out[-1][1] + 1e-12:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return out


def certified_gate_intervals(scene, robot, probes, tol=np.deg2rad(0.1),
                             ndir=48, eps_levels=DEFAULT_EPS_LEVELS,
                             period=np.pi, max_slices=20000, side="outer"):
    """Certified verdict intervals + event brackets for probe connectivity.

    side="outer": obstacles over-approximated -> events shift conservatively
    (closures earlier). side="inner": the opposite. True events lie between
    the two systems' brackets (G1 sandwich); run both for a certified
    truth-containing bracket."""
    L = robot.a * circum_factor(ndir)
    rep = GateEventReport(period=period, tol=tol)

    def verdict(th, perturb=0.0):
        rep.n_slices += 1
        if rep.n_slices > max_slices:
            raise RuntimeError("slice budget exhausted")
        sl = build_slice(scene, robot, th % period, ndir, side,
                         perturb=perturb)
        return probes_connected(sl, probes)

    min_r = min(r for r, _ in scene.disc_groups)
    stack = [(0.0, period)]
    while stack:
        lo, hi = stack.pop()
        if hi - lo < tol:
            # crumb: try to certify the whole sliver with one tailored-eps
            # probe (eps = L * halfwidth, usually tiny) before giving up
            th = 0.5 * (lo + hi)
            v0 = verdict(th)
            eps = L * (hi - lo) / 2 * 1.001
            if eps < min_r and verdict(th, perturb=(eps if v0 else -eps)) == v0:
                rep.intervals.append((lo, hi, v0))
                continue
            v_lo, v_hi = verdict(lo), verdict(hi)
            if v_lo != v_hi:
                rep.brackets.append((lo, hi, v_lo, v_hi))
            else:
                rep.unresolved.append((lo, hi))
            continue
        th = 0.5 * (lo + hi)
        v0 = verdict(th)
        radius = 0.0
        for eps in eps_levels:
            if eps >= min_r and not v0:
                continue                     # erosion capped by disc radius
            if verdict(th, perturb=(eps if v0 else -eps)) == v0:
                radius = eps / L
                break
        if radius == 0.0:
            stack += [(lo, th), (th, hi)]
            continue
        a, b = th - radius, th + radius
        rep.intervals.append((max(lo, a), min(hi, b), v0))
        if a > lo:
            stack.append((lo, a))
        if b < hi:
            stack.append((b, hi))
    rep.intervals.sort()
    rep.brackets.sort()
    rep.unresolved.sort()
    return rep
