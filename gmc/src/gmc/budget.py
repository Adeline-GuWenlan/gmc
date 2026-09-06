"""P5 oracle-work accounting and atomic budget enforcement.

Different oracle families are intentionally kept in separate currencies.
Equal-budget experiments may compare like with like without pretending that a
support evaluation and an SLSQP collision solve have unit-equivalent cost.
"""
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass, field
from threading import RLock


WORK_KINDS = (
    "support_value_evals",
    "support_point_evals",
    "pair_collision_evals",
    "pose_collision_queries",
    "intersection_tests",
    # Keep geometry work in independent currencies.  A Boolean/topological
    # overlay is not cost-equivalent to a support evaluation or index query.
    "union_operations",
    "difference_operations",
    "spatial_index_builds",
    "spatial_index_queries",
    "cache_hits",
)


class BudgetExceeded(RuntimeError):
    def __init__(self, kind: str, limit: int, used: int, requested: int):
        self.kind = kind
        self.limit = int(limit)
        self.used = int(used)
        self.requested = int(requested)
        super().__init__(
            f"{kind} budget exhausted: used={used}, requested={requested}, "
            f"limit={limit}"
        )


@dataclass
class WorkLedger:
    limits: dict[str, int] = field(default_factory=dict)
    default_phase: str = "compile"
    _by_phase: dict[str, dict[str, int]] = field(default_factory=dict,
                                                         init=False,
                                                         repr=False)
    _lock: RLock = field(default_factory=RLock, init=False, repr=False)
    _phase: ContextVar = field(init=False, repr=False)

    def __post_init__(self):
        self.limits = {str(k): int(v) for k, v in self.limits.items()}
        unknown = set(self.limits) - set(WORK_KINDS)
        if unknown:
            raise KeyError(f"unknown work kinds: {sorted(unknown)!r}")
        if any(value < 0 for value in self.limits.values()):
            raise ValueError("work limits must be non-negative")
        self._phase = ContextVar(
            f"gmc_work_ledger_phase_{id(self)}", default=self.default_phase
        )

    def _phase_counts(self, phase: str) -> dict[str, int]:
        return self._by_phase.setdefault(
            phase, {kind: 0 for kind in WORK_KINDS}
        )

    def total(self, kind: str) -> int:
        if kind not in WORK_KINDS:
            raise KeyError(kind)
        with self._lock:
            return sum(counts.get(kind, 0)
                       for counts in self._by_phase.values())

    def charge(self, kind: str, amount: int = 1, *, phase: str | None = None) -> None:
        if kind not in WORK_KINDS:
            raise KeyError(kind)
        amount = int(amount)
        if amount < 0:
            raise ValueError("work charge must be non-negative")
        with self._lock:
            limit = self.limits.get(kind)
            used = sum(counts.get(kind, 0)
                       for counts in self._by_phase.values())
            if limit is not None and used + amount > int(limit):
                raise BudgetExceeded(kind, int(limit), used, amount)
            counts = self._phase_counts(phase or self._phase.get())
            counts[kind] += amount

    @contextmanager
    def phase(self, name: str):
        token = self._phase.set(str(name))
        try:
            yield self
        finally:
            self._phase.reset(token)

    def snapshot(self) -> dict:
        with self._lock:
            by_phase = {
                phase: {kind: int(counts.get(kind, 0))
                        for kind in WORK_KINDS}
                for phase, counts in sorted(self._by_phase.items())
            }
        totals = {
            kind: int(sum(row[kind] for row in by_phase.values()))
            for kind in WORK_KINDS
        }
        return {
            "limits": {k: int(v) for k, v in sorted(self.limits.items())},
            "totals": totals,
            "by_phase": by_phase,
        }
