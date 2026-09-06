"""Common record schemas (SplatC-Bench spec 03 §9, generalized per audit 04 §5:
RepresentationRecord/GateRecord/QueryRecord instead of operator-specific ones).

These are deliberately plain dataclasses serialized to JSON — every experiment
must be replayable from its records alone (plan 00 §9 question 2/4).
"""
from __future__ import annotations

import dataclasses
import json
import platform
import subprocess
from dataclasses import dataclass, field


def _git_commit():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return "no-git"


@dataclass
class PlanningQuery:
    query_id: str
    scene_id: str
    robot_id: str
    start_pose: list          # [x, y, theta]
    goal_position: list       # [x, y]
    goal_radius: float
    final_orientation: str = "free"
    cost_config_id: str = "J_ellR0.5"


@dataclass
class GateRecord:
    gate_id: str
    scene_id: str
    robot_id: str
    orientation_half_angle: float     # radians, around theta=0 mod pi
    truth_type: str                   # "analytic" | "converged_numeric"
    critical_parameter: dict = field(default_factory=dict)


@dataclass
class OracleRecord:
    """Episode-level oracle record — the single canonical schema for what
    label_manifests.py emits (schema v2: multi-resolution per episode;
    the v1 per-resolution form was never produced)."""
    query_id: str
    split: str
    scene_id: str
    robot_id: str
    collision_checker_id: str
    theta_periodic: bool
    analytic: dict                    # physically_open / gate half-angle / center
    resolutions: dict                 # res -> {grid_shape, n_components,
    #                                   start_free, reachable_per_goal, seconds}
    measured_gate_half_angle_deg: float
    reference_path: dict | None       # goal_index/cost/n_poses/certified/margin
    convergence_status: str           # "converged" | "ORACLE_UNRESOLVED_GRID"
    uncertainty_flags: list = field(default_factory=list)


ORACLE_RES_KEYS = {"grid_shape", "n_components", "start_free",
                   "reachable_per_goal", "seconds"}


def validate_oracle_record(rec: dict):
    """Assert a record dict conforms to the OracleRecord schema before it is
    written — the drift guard between the driver and this module."""
    required = {f.name for f in dataclasses.fields(OracleRecord)
                if f.default is dataclasses.MISSING
                and f.default_factory is dataclasses.MISSING}
    missing = required - rec.keys()
    assert not missing, f"OracleRecord missing fields: {missing}"
    for res, r in rec["resolutions"].items():
        gap = ORACLE_RES_KEYS - r.keys()
        assert not gap, f"resolution '{res}' missing keys: {gap}"


@dataclass
class MethodRunRecord:
    method_id: str
    method_version: str
    scene_id: str
    robot_id: str
    query_ids: list
    compile_time_s: float
    query_times_s: list
    collision_queries: int
    retained_states_or_charts: int
    representation_bytes: int
    success: list
    commit_hash: str = field(default_factory=_git_commit)
    hardware: str = field(default_factory=platform.platform)
    random_seed: int | None = None
    failures: list = field(default_factory=list)


def dump(records, path):
    payload = [dataclasses.asdict(r) if dataclasses.is_dataclass(r) else r
               for r in records]
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)
