"""Periodic orientation intervals (Guide §8, invariant I6)."""
from dataclasses import dataclass

import numpy as np

TWO_PI = 2.0 * np.pi


@dataclass(frozen=True)
class Interval:
    lo: float
    hi: float                  # hi > lo; angles are NOT wrapped inside

    @property
    def width(self) -> float:
        return self.hi - self.lo

    @property
    def midpoint(self) -> float:
        return 0.5 * (self.lo + self.hi)

    @property
    def left_half(self) -> "Interval":
        return Interval(self.lo, self.midpoint)

    @property
    def right_half(self) -> "Interval":
        return Interval(self.midpoint, self.hi)


def initial_partition(n: int) -> list[Interval]:
    edges = np.linspace(0.0, TWO_PI, n + 1)
    return [Interval(float(a), float(b)) for a, b in zip(edges[:-1], edges[1:])]
