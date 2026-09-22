"""GS3D v1 interchange contracts (A0).

Units: metres, radians, seconds; world z is up. See docs/gs3d_architecture.md
for invariants, ownership, numerical requirements and acceptance fixtures.
These are interfaces, not implementations or certificates. Implementations must
validate external values, including finite numbers and array dimensions.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, ContextManager, Literal, Protocol, TypedDict

if TYPE_CHECKING:
    from gmc.height.ply3d import GaussianScene3D

Vec3 = tuple[float, float, float]
PoseRow = tuple[float, float, float, float]  # x, y, z, yaw (body centre)
Occupancy = Literal["free", "occupied", "unknown"]
Safety = Literal["continuous_bound", "sampled_only", "unresolved", "invalid"]
PlanStatus = Literal[
    "success", "invalid_input", "start_invalid", "goal_invalid",
    "map_unknown", "budget_exhausted", "no_path_on_lattice", "verification_failed",
]


@dataclass(frozen=True)
class Pose3:
    xyz: Vec3
    yaw: float = 0.0


@dataclass(frozen=True)
class BodySpec:
    name: str
    radius_m: float
    half_height_m: float
    motion: Literal["uav_translation", "ground_unicycle"]
    ground_clearance_m: float = 0.02  # chassis bottom above support; ground only


@dataclass(frozen=True)
class GoalRegion:
    original: Pose3
    position_tolerance_m: float = 0.0  # cylinder runner explicitly opts into 0.25
    yaw_tolerance_rad: float = 0.05


@dataclass(frozen=True)
class SearchBudget:
    max_wall_s: float = 120.0
    max_expansions: int = 100_000
    max_oracle_calls: int = 1_000_000
    max_narrowphase_pairs: int = 5_000_000


@dataclass(frozen=True)
class PlannerConfig:
    resolution_m: float = 0.10
    margin_m: float = 0.05  # UAV default; ground runner explicitly selects 0.001
    seed: int = 0
    budget: SearchBudget = SearchBudget()


@dataclass(frozen=True)
class SceneSpec:
    scene_id: str  # content identity, not display name
    gaussians: GaussianScene3D
    bounds_min: Vec3
    bounds_max: Vec3
    tau: float
    level: float
    known_space: KnownSpace
    support: SupportSurface | None
    provenance: dict[str, Any]


class KnownSpace(Protocol):
    """Coverage authority separate from Gaussian occupancy. Outside is unknown."""

    def contains_aabb(self, lower: Vec3, upper: Vec3) -> bool: ...


class SupportSurface(Protocol):
    """Ground support height; None means absent/unknown support."""

    def height(self, x: float, y: float) -> float | None: ...

    def height_bounds(self, lower_xy: tuple[float, float],
                      upper_xy: tuple[float, float]) -> tuple[float, float] | None:
        """Conservative min/max over the whole closed footprint box, or unknown."""
        ...

    def supports_segment(self, a: Vec3, b: Vec3, radius_m: float) -> bool: ...


@dataclass(frozen=True)
class OracleReport:
    occupancy: Occupancy
    safety: Safety
    clearance_lower_m: float | None
    reason: str
    primitive_ids: tuple[int, ...] = ()
    candidate_count: int = 0
    narrowphase_pairs: int = 0


class BodyOracle(Protocol):
    def pose(self, q: Pose3, body: BodySpec, *, margin_m: float) -> OracleReport: ...

    def edge(self, a: Pose3, b: Pose3, body: BodySpec,
             *, margin_m: float) -> OracleReport:
        """Linear xyz/yaw interpolation; continuous bound or fail closed."""
        ...


class TimingRecord(TypedDict):
    stage: str
    call_id: str
    seconds: float
    failed: bool
    sizes: dict[str, int | float | str]


class TimingSummary(TypedDict):
    algorithm_wall_s: float
    preparation_wall_s: float
    mode: Literal["cold", "warm"]
    records: list[TimingRecord]
    replans_s: list[float]


class TimingSink(Protocol):
    def stage(self, name: str, *, call_id: str = "",
              **sizes: int | float | str) -> ContextManager[Any]: ...


class Trajectory(TypedDict):
    poses: list[PoseRow]
    time_s: list[float]
    interpolation: Literal["linear_xyz_yaw", "piecewise_bezier"]
    segments: list[dict[str, Any]]  # required control points/durations for Bezier
    control_dt_s: float


class PlanResult(TypedDict):
    schema_version: Literal["gs3d.v1"]
    scene_id: str
    robot: dict[str, Any]  # serialized BodySpec + kinematic limits
    status: PlanStatus
    reason: str
    safety: Safety
    original_goal: PoseRow
    attained_goal: PoseRow | None
    position_tolerance_m: float
    yaw_tolerance_rad: float
    trajectory: Trajectory | None
    clearance_lower_m: float | None
    endpoint_reports: dict[str, Any]  # start, original, attained; candidate counts
    diagnostics: dict[str, Any]
    timings: TimingSummary
    provenance: dict[str, Any]  # commit, config, seeds, hashes, host/environment


class Planner(Protocol):
    def plan(self, scene: SceneSpec, body: BodySpec, start: Pose3,
             goal: GoalRegion, config: PlannerConfig,
             *, timer: TimingSink | None = None) -> PlanResult: ...
