"""Paired support-oracle event isolation under an atomic equal budget."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..budget import BudgetExceeded, WorkLedger
from ..geometry.support import PairOracle
from ..types import PairID
from .atlas import AtlasGateInstance


@dataclass(frozen=True)
class EventSearchResult:
    brackets: tuple[tuple[float, float], ...]
    budget_exhausted: bool
    evaluated_angles: int
    unresolved_intervals: int


class GateSupportProbe:
    """Two exact pair-resolved jamb gaps sharing one support-eval ledger."""

    def __init__(self, instance: AtlasGateInstance, ledger: WorkLedger):
        body = instance.robot.supports[0]
        top, bottom = instance.jamb_supports
        self._pairs = (
            PairOracle(PairID(top.primitive_id, body.primitive_id),
                       top, body, ledger=ledger),
            PairOracle(PairID(bottom.primitive_id, body.primitive_id),
                       bottom, body, ledger=ledger),
        )
        t = instance.gate_tangent
        self._directions = (-t, t)
        self._center = instance.gate_center
        self.pair_lipschitz = np.asarray(
            [pair.theta_lipschitz() for pair in self._pairs], dtype=float
        )
        self.lipschitz = float(np.max(self.pair_lipschitz))

    def pair_gaps(self, theta: float) -> np.ndarray:
        gaps = []
        for pair, direction in zip(self._pairs, self._directions):
            h = float(pair.support_values(theta, direction[None, :])[0])
            # t is free of this jamb exactly when u.t > h; negate the
            # configuration-obstacle halfspace residual to make free positive.
            gaps.append(float(direction @ self._center - h))
        return np.asarray(gaps, dtype=float)

    def scalar_gap(self, theta: float) -> np.ndarray:
        # Strong scalar arm sees the exact aggregate min of the same two pair
        # gaps, not a weakened grid proxy.  It pays the same two support calls.
        return np.asarray([float(np.min(self.pair_gaps(theta)))])


def _merge_intervals(intervals: list[tuple[float, float]],
                     atol: float = 1e-12,
                     period: float | None = None
                     ) -> tuple[tuple[float, float], ...]:
    if not intervals:
        return ()
    ordered = sorted((float(a), float(b)) for a, b in intervals)
    out = [ordered[0]]
    for lo, hi in ordered[1:]:
        if lo <= out[-1][1] + atol:
            out[-1] = (out[-1][0], max(out[-1][1], hi))
        else:
            out.append((lo, hi))
    # The search domain is a circle.  Regions touching 0 and ``period`` are
    # one event bracket, represented unwrapped as (lo, hi + period).
    if (period is not None and len(out) > 1
            and out[0][0] <= atol and out[-1][1] >= period - atol):
        wrapped = (out[-1][0], out[0][1] + period)
        out = [wrapped, *out[1:-1]]
    return tuple(out)


def adaptive_isolate_events(
    evaluate,
    *,
    lipschitz: float | np.ndarray,
    tolerance: float,
    initial_intervals: int,
    period: float = np.pi,
    aggregation: str = "independent",
) -> EventSearchResult:
    """Certify root-free intervals and bracket every remaining event region.

    A scalar channel ``f`` is root-free on a midpoint interval of radius ``r``
    whenever ``abs(f(mid)) > L*r``.  For ``aggregation="minimum"`` the event
    function is ``min_i f_i`` and vector bounds are used: the minimum is
    positive everywhere if every lower bound is positive, and negative
    everywhere if any upper bound is negative.  Individual pair roots that are
    dominated by another negative pair are therefore not aggregate events.
    """
    if (not np.isfinite(tolerance) or tolerance <= 0.0
            or not np.isfinite(period) or period <= 0.0
            or int(initial_intervals) != initial_intervals
            or initial_intervals < 1):
        raise ValueError("invalid adaptive event search configuration")
    if aggregation not in {"independent", "minimum"}:
        raise ValueError(f"unknown event aggregation: {aggregation}")
    lipschitz_values = np.asarray(lipschitz, dtype=float)
    if lipschitz_values.ndim == 0:
        lipschitz_values = lipschitz_values.reshape(1)
    if (lipschitz_values.ndim != 1 or lipschitz_values.size == 0
            or not np.all(np.isfinite(lipschitz_values))
            or np.any(lipschitz_values <= 0.0)):
        raise ValueError("lipschitz bounds must be finite and positive")
    cache: dict[float, np.ndarray] = {}
    exhausted = False

    def cached(theta: float) -> np.ndarray:
        key = float(theta % period)
        if key not in cache:
            values = np.asarray(evaluate(key), dtype=float)
            if values.ndim == 0:
                values = values.reshape(1)
            if (values.ndim != 1 or values.size == 0
                    or not np.all(np.isfinite(values))):
                raise ValueError(
                    "event evaluator must return a nonempty finite vector"
                )
            if lipschitz_values.size not in (1, values.size):
                raise ValueError(
                    "lipschitz vector must be scalar or match evaluator output"
                )
            if cache and values.shape != next(iter(cache.values())).shape:
                raise ValueError(
                    "event evaluator output shape changed across angles"
                )
            cache[key] = values
        return cache[key]

    def channel_bounds(values: np.ndarray, radius: float) -> np.ndarray:
        if lipschitz_values.size == 1:
            return np.full(values.shape, lipschitz_values[0] * radius)
        return lipschitz_values * radius

    def possibly_has_root(values: np.ndarray, radius: float) -> bool:
        bounds = channel_bounds(values, radius)
        if aggregation == "minimum":
            lower = values - bounds
            upper = values + bounds
            root_free = bool(np.all(lower > 0.0) or np.any(upper < 0.0))
            return not root_free
        return bool(np.any(np.abs(values) <= bounds + 1e-14))

    def possible_zero_plateau(lo: float, hi: float,
                              mid_values: np.ndarray) -> bool:
        """Conservatively flag a sampled continuous-zero event region.

        An isolated root may land exactly at the midpoint, so a zero midpoint
        alone remains a valid bracket.  If the same aggregate channel is zero
        at both endpoints too, the interval cannot be represented as an
        isolated event at this resolution and must remain unresolved.
        """
        zero_atol = 1e-14
        if aggregation == "minimum":
            if abs(float(np.min(mid_values))) > zero_atol:
                return False
            return bool(
                abs(float(np.min(cached(lo)))) <= zero_atol
                and abs(float(np.min(cached(hi)))) <= zero_atol
            )
        zero_channels = np.abs(mid_values) <= zero_atol
        if not np.any(zero_channels):
            return False
        return bool(np.any(
            zero_channels
            & (np.abs(cached(lo)) <= zero_atol)
            & (np.abs(cached(hi)) <= zero_atol)
        ))

    edges = np.linspace(0.0, period, initial_intervals + 1)
    stack = [(float(edges[i]), float(edges[i + 1]))
             for i in reversed(range(initial_intervals))]
    brackets: list[tuple[float, float]] = []
    unresolved = 0
    while stack:
        lo, hi = stack.pop()
        mid = 0.5 * (lo + hi)
        try:
            values = cached(mid)
        except BudgetExceeded:
            exhausted = True
            unresolved += 1 + len(stack)
            break
        radius = (hi - lo) / 2.0
        if not possibly_has_root(values, radius):
            continue
        if hi - lo <= tolerance:
            try:
                if possible_zero_plateau(lo, hi, values):
                    unresolved += 1
                    continue
            except BudgetExceeded:
                exhausted = True
                unresolved += 1 + len(stack)
                break
            brackets.append((lo, hi))
            continue
        stack.append((mid, hi))
        stack.append((lo, mid))
    return EventSearchResult(
        brackets=_merge_intervals(brackets, period=period),
        budget_exhausted=exhausted,
        evaluated_angles=len(cache),
        unresolved_intervals=unresolved,
    )


def run_event_arm(instance: AtlasGateInstance, arm: str, *,
                  support_eval_cap: int, tolerance: float,
                  initial_intervals: int) -> tuple[EventSearchResult, dict]:
    ledger = WorkLedger(
        limits={"support_value_evals": int(support_eval_cap)},
        default_phase="event_search",
    )
    probe = GateSupportProbe(instance, ledger)
    if arm == "gmc_pair_resolved":
        evaluate = probe.pair_gaps
        lipschitz = probe.pair_lipschitz
        aggregation = "minimum"
    elif arm == "strong_adaptive_scalar":
        evaluate = probe.scalar_gap
        lipschitz = probe.lipschitz
        aggregation = "independent"
    else:
        raise ValueError(f"unknown event benchmark arm: {arm}")
    result = adaptive_isolate_events(
        evaluate,
        lipschitz=lipschitz,
        tolerance=tolerance,
        initial_intervals=initial_intervals,
        aggregation=aggregation,
    )
    return result, ledger.snapshot()
