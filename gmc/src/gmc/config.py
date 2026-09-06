"""Frozen run configuration loaded from YAML (Guide §B.1)."""
from dataclasses import dataclass
from numbers import Integral, Real
from pathlib import Path

import numpy as np
import yaml


def _finite_float(value, name: str) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a real number")
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _integer(value, name: str) -> int:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be an integer")
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real) and np.isfinite(value) and float(value).is_integer():
        return int(value)
    raise ValueError(f"{name} must be an integer")


def _strict_bool(value, name: str) -> bool:
    if not isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be a YAML boolean")
    return bool(value)


@dataclass(frozen=True)
class GeometryCfg:
    workspace_precision: float
    min_cov_eigenvalue: float
    support_level_scene: float
    support_level_robot: float

    def __post_init__(self):
        wp = _finite_float(self.workspace_precision, "workspace_precision")
        me = _finite_float(self.min_cov_eigenvalue,
                           "min_cov_eigenvalue")
        ss = _finite_float(self.support_level_scene,
                           "support_level_scene")
        sr = _finite_float(self.support_level_robot,
                           "support_level_robot")
        if wp <= 0.0:
            raise ValueError("workspace_precision must be positive")
        if me < 0.0:
            raise ValueError("min_cov_eigenvalue must be non-negative")
        if ss <= 0.0 or sr <= 0.0:
            raise ValueError("support levels must be positive")
        object.__setattr__(self, "workspace_precision", wp)
        object.__setattr__(self, "min_cov_eigenvalue", me)
        object.__setattr__(self, "support_level_scene", ss)
        object.__setattr__(self, "support_level_robot", sr)


@dataclass(frozen=True)
class PairApproxCfg:
    initial_directions: int
    max_directions: int
    eps_pair: float
    certificate_mode: str          # "prototype" | "theorem"

    def __post_init__(self):
        initial = _integer(self.initial_directions, "initial_directions")
        maximum = _integer(self.max_directions, "max_directions")
        eps = _finite_float(self.eps_pair, "eps_pair")
        if initial < 3:
            raise ValueError("initial_directions must be at least 3")
        if maximum < initial:
            raise ValueError(
                "max_directions must be at least initial_directions")
        if eps <= 0.0:
            raise ValueError("eps_pair must be positive")
        if (not isinstance(self.certificate_mode, str)
                or self.certificate_mode not in {"prototype", "theorem"}):
            raise ValueError(
                "certificate_mode must be 'prototype' or 'theorem'")
        object.__setattr__(self, "initial_directions", initial)
        object.__setattr__(self, "max_directions", maximum)
        object.__setattr__(self, "eps_pair", eps)


@dataclass(frozen=True)
class OrientationCfg:
    initial_intervals: int
    theta_min: float
    max_depth: int

    def __post_init__(self):
        initial = _integer(self.initial_intervals, "initial_intervals")
        theta_min = _finite_float(self.theta_min, "theta_min")
        depth = _integer(self.max_depth, "max_depth")
        if initial < 1:
            raise ValueError("initial_intervals must be positive")
        if theta_min <= 0.0:
            raise ValueError("theta_min must be positive")
        if depth < 0:
            raise ValueError("max_depth must be non-negative")
        object.__setattr__(self, "initial_intervals", initial)
        object.__setattr__(self, "theta_min", theta_min)
        object.__setattr__(self, "max_depth", depth)


@dataclass(frozen=True)
class QueryCfg:
    max_support_calls: int
    max_wall_seconds: float
    eps_clear: float
    # Query-local orientation refinement is deliberately bounded separately
    # from the support/wall budgets.  Keeping a default preserves every
    # existing programmatic ``QueryCfg(a, b, c)`` construction while making
    # the retry limit part of the frozen configuration for new runs.
    max_refinement_rounds: int = 0

    def __post_init__(self):
        calls = _integer(self.max_support_calls, "max_support_calls")
        wall = _finite_float(self.max_wall_seconds, "max_wall_seconds")
        eps = _finite_float(self.eps_clear, "eps_clear")
        rounds = _integer(self.max_refinement_rounds,
                          "max_refinement_rounds")
        if calls < 0:
            raise ValueError("max_support_calls must be non-negative")
        if wall < 0.0:
            raise ValueError("max_wall_seconds must be non-negative")
        if eps < 0.0:
            raise ValueError("eps_clear must be non-negative")
        if rounds < 0:
            raise ValueError("max_refinement_rounds must be non-negative")
        object.__setattr__(self, "max_support_calls", calls)
        object.__setattr__(self, "max_wall_seconds", wall)
        object.__setattr__(self, "eps_clear", eps)
        object.__setattr__(self, "max_refinement_rounds", rounds)


@dataclass(frozen=True)
class LoggingCfg:
    save_intermediate_geometry: bool
    save_failed_cases: bool

    def __post_init__(self):
        object.__setattr__(
            self, "save_intermediate_geometry",
            _strict_bool(self.save_intermediate_geometry,
                         "save_intermediate_geometry"))
        object.__setattr__(
            self, "save_failed_cases",
            _strict_bool(self.save_failed_cases, "save_failed_cases"))


@dataclass(frozen=True)
class Config:
    length_unit: str
    geometry: GeometryCfg
    pair_approx: PairApproxCfg
    orientation: OrientationCfg
    query: QueryCfg
    logging: LoggingCfg
    source_path: str = ""

    def __post_init__(self):
        if not isinstance(self.length_unit, str) or not self.length_unit.strip():
            raise ValueError("length_unit must be a non-empty string")
        expected = ((self.geometry, GeometryCfg, "geometry"),
                    (self.pair_approx, PairApproxCfg, "pair_approx"),
                    (self.orientation, OrientationCfg, "orientation"),
                    (self.query, QueryCfg, "query"),
                    (self.logging, LoggingCfg, "logging"))
        for value, cls, name in expected:
            if not isinstance(value, cls):
                raise ValueError(f"{name} must be a {cls.__name__}")
        if not isinstance(self.source_path, str):
            raise ValueError("source_path must be a string")


def load_config(path: str | Path) -> Config:
    raw = yaml.safe_load(Path(path).read_text())
    if not isinstance(raw, dict):
        raise ValueError("configuration root must be a mapping")
    return Config(
        length_unit=raw["units"]["length"],
        geometry=GeometryCfg(**raw["geometry"]),
        pair_approx=PairApproxCfg(
            initial_directions=raw["pair_approx"]["initial_directions"],
            max_directions=raw["pair_approx"]["max_directions"],
            eps_pair=raw["pair_approx"]["eps_pair"],
            certificate_mode=raw["pair_approx"]["certificate_mode"],
        ),
        orientation=OrientationCfg(
            initial_intervals=raw["orientation"]["initial_intervals"],
            theta_min=raw["orientation"]["theta_min"],
            max_depth=raw["orientation"]["max_depth"],
        ),
        query=QueryCfg(
            max_support_calls=raw["query"]["max_support_calls"],
            max_wall_seconds=raw["query"]["max_wall_seconds"],
            eps_clear=raw["query"]["eps_clear"],
            max_refinement_rounds=raw["query"].get(
                "max_refinement_rounds", 0),
        ),
        logging=LoggingCfg(**raw["logging"]),
        source_path=str(path),
    )
