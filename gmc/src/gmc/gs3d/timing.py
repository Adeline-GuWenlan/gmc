"""Wall-clock instrumentation for GS3D planning.

This module deliberately measures elapsed monotonic time, not simulated control
time, render frame time, scheduler queue time, or a sum of nested stages.  A
``TimingSink`` can be passed directly to :class:`LatticePlanner`; callers then
finalize a plan result with :meth:`PlanTiming.finalize`.
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field
from math import ceil
from time import perf_counter
from typing import Callable, Iterator


Clock = Callable[[], float]


@dataclass
class _Record:
    stage: str
    call_id: str
    started_s: float
    seconds: float = 0.0
    failed: bool = False
    sizes: dict[str, int | float | str] = field(default_factory=dict)

    def json(self) -> dict:
        return {"stage": self.stage, "call_id": self.call_id,
                "seconds": self.seconds, "failed": self.failed,
                "sizes": dict(self.sizes)}


class PlanTiming:
    """A monotonic stage collector for one cold preparation and many replans.

    Stage durations are individual spans.  They are intentionally not added to
    make a total because hooks such as ``edge_validation`` can be nested in
    ``search``.  ``algorithm_wall_s`` always comes from the planner's own
    algorithm-entry measurement when available.
    """

    def __init__(self, *, clock: Clock = perf_counter):
        self.clock = clock
        self._records: list[_Record] = []
        self._emitted_records = 0
        self._replans: list[float] = []
        self._preparation_started: float | None = None
        self.preparation_wall_s = 0.0

    @contextmanager
    def stage(self, name: str, *, call_id: str = "", **sizes: int | float | str) -> Iterator[None]:
        if not name or not isinstance(name, str):
            raise ValueError("timing stage must be a nonempty string")
        started = self.clock()
        record = _Record(name, call_id, started, sizes=dict(sizes))
        self._records.append(record)
        try:
            yield
        except BaseException:
            record.failed = True
            raise
        finally:
            record.seconds = max(0.0, float(self.clock() - started))

    @contextmanager
    def preparation(self) -> Iterator[None]:
        if self._preparation_started is not None:
            raise RuntimeError("preparation timing is already active")
        started = self.clock()
        self._preparation_started = started
        try:
            yield
        finally:
            self.preparation_wall_s += max(0.0, float(self.clock() - started))
            self._preparation_started = None

    def for_call(self, call_id: str):
        """Return a sink that assigns an identifier to every planner hook."""
        parent = self

        class _CallSink:
            @contextmanager
            def stage(self, name: str, *, call_id: str = "", **sizes):
                with parent.stage(name, call_id=call_id or str(call_id_outer), **sizes):
                    yield

        call_id_outer = str(call_id)
        return _CallSink()

    def finalize(self, result: dict, *, mode: str, call_id: str,
                 preparation_wall_s: float | None = None) -> dict:
        """Attach contract timing without changing planning status or geometry."""
        if mode not in ("cold", "warm"):
            raise ValueError("mode must be cold or warm")
        timing = dict(result.get("timings", {}))
        algorithm = float(timing.get("algorithm_wall_s", 0.0))
        # Planner starts this interval at its actual algorithm entry.  Do not
        # replace it with an outer wrapper measurement that includes setup.
        if algorithm < 0:
            raise ValueError("algorithm wall time must be nonnegative")
        self._replans.append(algorithm)
        records = [r.json() for r in self._records[self._emitted_records:]]
        self._emitted_records = len(self._records)
        timing.update({"algorithm_wall_s": algorithm,
                       "preparation_wall_s": float(self.preparation_wall_s if preparation_wall_s is None else preparation_wall_s),
                       "mode": mode, "records": records,
                       "replans_s": list(self._replans),
                       "call_id": str(call_id),
                       "stage_total_is_not_wall": True})
        result["timings"] = timing
        return result


def latency_statistics(samples: list[float]) -> dict[str, float | int]:
    """Deterministic nearest-rank summary; reports all samples separately too."""
    if not samples:
        return {"count": 0, "p50_s": 0.0, "p95_s": 0.0, "max_s": 0.0}
    ordered = sorted(float(x) for x in samples)
    def percentile(p: float) -> float:
        index = int(ceil(p * len(ordered)) - 1)
        return ordered[max(0, min(len(ordered) - 1, index))]
    return {"count": len(ordered), "p50_s": percentile(.50), "p95_s": percentile(.95), "max_s": ordered[-1]}
