"""Core immutable schemas and enums (Implementation Guide §3.1–§3.2).

Every certified object carries a CertStatus; geometric uncertainty travels as
data (Result), never as a swallowed None/exception (failure-as-data, §3.2).
"""
from dataclasses import dataclass, field
from enum import Enum, auto
import math
from numbers import Integral, Real

import numpy as np
from numpy.typing import NDArray
from shapely.geometry import Polygon

Vec2 = NDArray[np.float64]
Mat2 = NDArray[np.float64]


class CertStatus(Enum):
    APPROX_UNCERTIFIED = auto()
    EMPIRICALLY_VALIDATED = auto()
    CERTIFIED = auto()
    UNKNOWN = auto()


class PlanStatus(Enum):
    REACHABLE = auto()
    UNREACHABLE = auto()
    UNKNOWN = auto()
    INTERNAL_ERROR = auto()
    INVALID_GEOMETRY = auto()


@dataclass(frozen=True)
class GaussianSupport2D:
    mean: Vec2
    covariance: Mat2
    level: float
    primitive_id: int

    def __post_init__(self):
        mean = np.array(self.mean, dtype=np.float64, copy=True)
        covariance = np.array(self.covariance, dtype=np.float64, copy=True)
        if mean.shape != (2,):
            raise ValueError("support mean must have shape (2,)")
        if covariance.shape != (2, 2):
            raise ValueError("support covariance must have shape (2, 2)")
        if not np.all(np.isfinite(mean)):
            raise ValueError("support mean must be finite")
        if not np.all(np.isfinite(covariance)):
            raise ValueError("support covariance must be finite")
        if not np.allclose(covariance, covariance.T,
                           rtol=1e-10, atol=1e-12):
            raise ValueError("support covariance must be symmetric")
        # Matrix products such as R @ diag @ R.T can differ by one ulp across
        # the diagonal.  Canonicalise only after the tight symmetry check.
        covariance = 0.5 * (covariance + covariance.T)
        if float(np.linalg.eigvalsh(covariance)[0]) <= 0.0:
            raise ValueError("support covariance must be positive definite")
        if isinstance(self.level, (bool, np.bool_)) or not isinstance(
                self.level, Real):
            raise ValueError("support level must be a real number")
        level = float(self.level)
        if not np.isfinite(level) or level <= 0.0:
            raise ValueError("support level must be finite and positive")
        if isinstance(self.primitive_id, (bool, np.bool_)) or not isinstance(
                self.primitive_id, Integral):
            raise ValueError("primitive_id must be an integer")
        mean.flags.writeable = False
        covariance.flags.writeable = False
        object.__setattr__(self, "mean", mean)
        object.__setattr__(self, "covariance", covariance)
        object.__setattr__(self, "level", level)
        object.__setattr__(self, "primitive_id", int(self.primitive_id))

    def bounding_radius(self) -> float:
        return float(self.level * np.sqrt(np.linalg.eigvalsh(self.covariance)[-1]))


@dataclass(frozen=True)
class PairID:
    scene_id: int
    body_id: int


@dataclass(frozen=True)
class Pose2:
    xy: Vec2
    theta: float

    def __post_init__(self):
        xy = np.array(self.xy, dtype=np.float64, copy=True)
        theta = float(self.theta)
        if xy.shape != (2,):
            raise ValueError("pose xy must have shape (2,)")
        if not np.all(np.isfinite(xy)) or not np.isfinite(theta):
            raise ValueError("pose coordinates and theta must be finite")
        xy.flags.writeable = False
        object.__setattr__(self, "xy", xy)
        object.__setattr__(self, "theta", theta)


@dataclass(frozen=True)
class RobotModel2D:
    """Union of rigid body ellipsoid supports in the robot frame (§1.1)."""
    supports: tuple            # tuple[GaussianSupport2D, ...], means are nu_j
    name: str = "robot"

    def __post_init__(self):
        supports = tuple(self.supports)
        if not supports:
            raise ValueError("robot must contain at least one support")
        if not all(isinstance(s, GaussianSupport2D) for s in supports):
            raise ValueError("robot supports must be GaussianSupport2D values")
        ids = [s.primitive_id for s in supports]
        if len(ids) != len(set(ids)):
            raise ValueError("robot primitive_id values must be unique")
        object.__setattr__(self, "supports", supports)

    def max_rotational_radius(self) -> float:
        return max(float(np.linalg.norm(s.mean)) + s.bounding_radius()
                   for s in self.supports)


@dataclass(frozen=True)
class SceneModel2D:
    supports: tuple            # tuple[GaussianSupport2D, ...]
    workspace: object          # shapely Polygon (may contain holes)
    name: str = "scene"

    def __post_init__(self):
        supports = tuple(self.supports)
        if not all(isinstance(s, GaussianSupport2D) for s in supports):
            raise ValueError("scene supports must be GaussianSupport2D values")
        ids = [s.primitive_id for s in supports]
        if len(ids) != len(set(ids)):
            raise ValueError("scene primitive_id values must be unique")
        if not isinstance(self.workspace, Polygon):
            raise ValueError("workspace must be a Polygon")
        bounds = np.asarray(self.workspace.bounds, dtype=float)
        if (self.workspace.is_empty or not self.workspace.is_valid
                or not np.all(np.isfinite(bounds))
                or not np.isfinite(self.workspace.area)
                or self.workspace.area <= 0.0):
            raise ValueError("workspace must be finite, valid, and non-empty")
        object.__setattr__(self, "supports", supports)


@dataclass(frozen=True)
class Result[T]:
    value: T | None
    status: CertStatus
    reason_code: str
    uncertainty_sources: tuple[str, ...] = ()
    diagnostics: dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.value is not None


def canonical_angle(theta: float) -> float:
    """Scale-independent representative in ``(-pi, pi]``.

    ``math.remainder`` preserves tiny angles near zero while mapping exact
    floating-point multiples of ``2*pi`` to zero.  This avoids both decimal
    cache quantisation and the amplified ``sin(2*pi)`` residual produced by a
    body support with a large local offset.
    """
    if not np.isfinite(theta):
        raise ValueError("theta must be finite")
    result = float(math.remainder(float(theta), 2.0 * math.pi))
    if result <= -math.pi:
        result = math.pi
    if result == 0.0:
        result = 0.0  # canonicalise negative zero for cache identity
    return result


def rotation2(theta: float) -> Mat2:
    theta = canonical_angle(theta)
    # Exact values at the quadrantal representatives prevent a libm residual
    # from becoming macroscopic after multiplication by a large support mean.
    if theta == 0.0:
        c, s = 1.0, 0.0
    elif theta == np.pi:
        c, s = -1.0, 0.0
    elif theta == np.pi / 2.0:
        c, s = 0.0, 1.0
    elif theta == -np.pi / 2.0:
        c, s = 0.0, -1.0
    else:
        c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def wrap_angle(theta: float) -> float:
    """Canonical angle in [0, 2*pi) (I6 periodicity convention)."""
    result = canonical_angle(theta)
    return result if result >= 0.0 else float(result + 2.0 * np.pi)
