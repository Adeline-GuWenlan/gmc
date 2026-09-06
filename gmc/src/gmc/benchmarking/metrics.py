"""Metrics for periodic event-bracket benchmarks."""
from __future__ import annotations

import numpy as np


def circular_distance(a: float, b: float, period: float) -> float:
    d = abs(float(a) - float(b)) % period
    return float(min(d, period - d))


def _periodic_contains(event: float, lo: float, hi: float,
                       period: float, atol: float = 1e-12) -> bool:
    """Whether a canonical event lies in an optionally unwrapped bracket."""
    event = float(event % period)
    return any(lo - atol <= shifted <= hi + atol
               for shifted in (event - period, event, event + period))


def _canonical_periodic_brackets(brackets, period: float,
                                 atol: float = 1e-12):
    ordered = sorted((float(lo), float(hi)) for lo, hi in brackets)
    if not ordered:
        return ()
    merged = [ordered[0]]
    for lo, hi in ordered[1:]:
        if lo <= merged[-1][1] + atol:
            merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
        else:
            merged.append((lo, hi))
    if (len(merged) > 1 and merged[0][0] <= atol
            and merged[-1][1] >= period - atol):
        merged = [
            (merged[-1][0], merged[0][1] + period),
            *merged[1:-1],
        ]
    return tuple(merged)


def evaluate_event_brackets(truth_events, brackets, *,
                            period: float = np.pi) -> dict:
    truth = tuple(float(t % period) for t in truth_events)
    pred = _canonical_periodic_brackets(brackets, period)
    # Maximum-cardinality one-to-one truth<->bracket matching.  Counting each
    # truth independently lets one broad bracket claim arbitrarily many events
    # and can report perfect recall with fewer predictions than truths.
    adjacency = {
        event_index: sorted(
            (
                bracket_index
                for bracket_index, (lo, hi) in enumerate(pred)
                if _periodic_contains(event, lo, hi, period)
            ),
            key=lambda bracket_index: circular_distance(
                event,
                0.5 * (pred[bracket_index][0] + pred[bracket_index][1]),
                period,
            ),
        )
        for event_index, event in enumerate(truth)
    }
    bracket_to_event: dict[int, int] = {}

    def augment(event_index: int, seen: set[int]) -> bool:
        for bracket_index in adjacency[event_index]:
            if bracket_index in seen:
                continue
            seen.add(bracket_index)
            previous = bracket_to_event.get(bracket_index)
            if previous is None or augment(previous, seen):
                bracket_to_event[bracket_index] = event_index
                return True
        return False

    # Constrained truths first reduces arbitrary rematching while retaining
    # maximum cardinality.
    for event_index in sorted(adjacency, key=lambda i: (len(adjacency[i]), i)):
        augment(event_index, set())

    errors = [
        circular_distance(
            truth[event_index], 0.5 * (pred[bracket_index][0]
                                       + pred[bracket_index][1]), period
        )
        for bracket_index, event_index in bracket_to_event.items()
    ]
    matched = len(bracket_to_event)
    spurious = len(pred) - matched
    widths = [hi - lo for lo, hi in pred]
    return {
        "truth_count": len(truth),
        "predicted_bracket_count": len(pred),
        "matched_truth_count": int(matched),
        "event_recall": float(matched / len(truth)) if truth else 1.0,
        "spurious_bracket_count": int(spurious),
        "mean_bracket_width_deg": (
            float(np.rad2deg(np.mean(widths))) if widths else None
        ),
        "max_bracket_width_deg": (
            float(np.rad2deg(max(widths))) if widths else None
        ),
        "mean_midpoint_error_deg": (
            float(np.rad2deg(np.mean(errors))) if errors else None
        ),
        "max_midpoint_error_deg": (
            float(np.rad2deg(max(errors))) if errors else None
        ),
    }


def aggregate_arm(case_rows: list[dict], arm: str) -> dict:
    rows = [row["arms"][arm] for row in case_rows]
    truth = sum(row["metrics"]["truth_count"] for row in rows)
    predicted = sum(row["metrics"]["predicted_bracket_count"] for row in rows)
    matched = sum(row["metrics"]["matched_truth_count"] for row in rows)
    spurious = sum(row["metrics"]["spurious_bracket_count"] for row in rows)
    widths = [row["metrics"]["mean_bracket_width_deg"] for row in rows
              if row["metrics"]["mean_bracket_width_deg"] is not None]
    max_widths = [row["metrics"]["max_bracket_width_deg"] for row in rows
                  if row["metrics"]["max_bracket_width_deg"] is not None]
    midpoint_errors = [
        row["metrics"]["mean_midpoint_error_deg"] for row in rows
        if row["metrics"]["mean_midpoint_error_deg"] is not None
    ]
    max_midpoint_errors = [
        row["metrics"]["max_midpoint_error_deg"] for row in rows
        if row["metrics"]["max_midpoint_error_deg"] is not None
    ]
    unknown = sum(
        bool(row["search"]["budget_exhausted"])
        or int(row["search"]["unresolved_intervals"]) > 0
        for row in rows
    )
    return {
        "n_cases": len(rows),
        "truth_events": int(truth),
        "predicted_brackets": int(predicted),
        "matched_truth_events": int(matched),
        "event_recall": float(matched / truth) if truth else 1.0,
        "spurious_brackets": int(spurious),
        "budget_exhausted_cases": int(sum(
            bool(row["search"]["budget_exhausted"]) for row in rows
        )),
        "unresolved_intervals": int(sum(
            int(row["search"]["unresolved_intervals"]) for row in rows
        )),
        "unknown_cases": int(unknown),
        "unknown_rate": float(unknown / len(rows)) if rows else 0.0,
        "certified_complete_cases": int(len(rows) - unknown),
        "support_value_evals": int(sum(
            row["ledger"]["totals"]["support_value_evals"] for row in rows
        )),
        "mean_case_bracket_width_deg": (
            float(np.mean(widths)) if widths else None
        ),
        "max_bracket_width_deg": (
            float(max(max_widths)) if max_widths else None
        ),
        "mean_case_midpoint_error_deg": (
            float(np.mean(midpoint_errors)) if midpoint_errors else None
        ),
        "max_midpoint_error_deg": (
            float(max(max_midpoint_errors)) if max_midpoint_errors else None
        ),
        "wall_seconds": float(sum(row["wall_seconds"] for row in rows)),
    }
