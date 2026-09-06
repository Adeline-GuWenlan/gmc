"""M9 directed edge certification interface (Guide §12.2).

Geometric and dynamic graphs stay separate: M_geom provides candidate global
connectivity, a certifier turns candidates into directed executable edges.
Deferred by design until holonomic P0–P4 pass (§12 "Deferred by design").

The concrete reversible-DD checker requires a corridor verification context,
not an unbound raw polygon: corridor containment alone cannot prove collision
safety. A graph-level ``M_dyn`` builder remains outside this v0 checker."""
from typing import Protocol

from ..mobility.witness import PoseCurve
from ..types import Pose2, Result


class DynamicsModel(Protocol):
    name: str


class LocalBudget(Protocol):
    max_wall_seconds: float


class EdgeCertifier(Protocol):
    def certify(self, corridor, q_start: Pose2, q_goal: Pose2,
                dynamics: DynamicsModel, budget: LocalBudget) -> Result[PoseCurve]:
        ...
