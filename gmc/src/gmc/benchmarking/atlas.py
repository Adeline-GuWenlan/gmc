"""Read-only adapter for the frozen SplatC-Atlas G1 assets.

Primary truth is reconstructed from a pinned split seal, the exact bytes of a
split manifest, and the pinned Atlas robot registry.  The round-4 package
checksum inventory is also checked for every package-resident input used at
runtime.  The inventory file is reported with its own digest, but is not
misrepresented as an externally authenticated signature: the split-seal and
source digests below are the independent roots of trust.

Only the sealed G1 dataset generator and its data types are imported.  A
delta-based import guard rejects any ``splatc.gmc`` legacy module loaded by an
adapter operation, without blaming unrelated modules that were already
present in the host interpreter.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import importlib
import json
from pathlib import Path
from pathlib import PurePosixPath
import sys

import numpy as np
import shapely

from ..io.robot_io import ellipse_robot
from ..types import GaussianSupport2D, RobotModel2D, SceneModel2D


TWO_PI = 2.0 * np.pi


# These two digests are the stable, independently reviewed roots.  The
# package-level MANIFEST.sha256 is intentionally not pinned because legitimate
# local and HPC packages contain different non-input report files.
_PINNED_SPLIT_SEALS_SHA256 = (
    "2010fc188f3f4325e51db2f7c77fee55047fdd70f9a0b02b3a05aed7511ad433"
)
_PINNED_ATLAS_SOURCES = {
    "src_snapshot/src/splatc/__init__.py":
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "src_snapshot/src/splatc/common/__init__.py":
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "src_snapshot/src/splatc/common/se2.py":
        "b1e3fa20648891a501ca5b8f581b289e8f5ba951630b08df39f39d5164735b0d",
    "src_snapshot/src/splatc/datasets/__init__.py":
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "src_snapshot/src/splatc/datasets/g1_gate.py":
        "f3dbd542098079c18f4b2fbec53c12cdfcaf82e283fa0c8c26df953f94897424",
    "src_snapshot/src/splatc/gaussian_geometry/__init__.py":
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "src_snapshot/src/splatc/gaussian_geometry/contact.py":
        "179a01d1227bb4f077bdf3551f516d98d886ee0c79f93b3e3c82397124a5094d",
    "src_snapshot/src/splatc/gaussian_geometry/primitives.py":
        "b622b98b127173ba628f24cd65b7cef9ef6b4305dda97819d972f8f176fad819",
}
_ATLAS_MODULE_SOURCES = {
    "splatc": "src_snapshot/src/splatc/__init__.py",
    "splatc.common": "src_snapshot/src/splatc/common/__init__.py",
    "splatc.common.se2": "src_snapshot/src/splatc/common/se2.py",
    "splatc.datasets": "src_snapshot/src/splatc/datasets/__init__.py",
    "splatc.datasets.g1_gate":
        "src_snapshot/src/splatc/datasets/g1_gate.py",
    "splatc.gaussian_geometry":
        "src_snapshot/src/splatc/gaussian_geometry/__init__.py",
    "splatc.gaussian_geometry.contact":
        "src_snapshot/src/splatc/gaussian_geometry/contact.py",
    "splatc.gaussian_geometry.primitives":
        "src_snapshot/src/splatc/gaussian_geometry/primitives.py",
}
_SPLIT_SEALS_RELATIVE = (
    "src_snapshot/results/manifests/split_seals.json"
)


class AtlasAssetError(RuntimeError):
    """The frozen Atlas inputs are absent, inconsistent, or malformed."""


class BlindSplitForbidden(AtlasAssetError):
    """Blind inputs require an explicit, auditable opt-in."""


@dataclass(frozen=True)
class AtlasCase:
    query_id: str
    split: str
    episode: dict
    record: dict | None
    gate_center_deg: float
    gate_half_angle_deg: float
    measured_gate_half_angle_deg: float | None
    convergence_status: str
    fine_reachable_per_goal: tuple[bool, ...] | None

    @property
    def family(self) -> str:
        return str(self.episode["scene"]["family"])

    @property
    def robot_id(self) -> str:
        return str(self.episode["robot_id"])

    @property
    def is_partial_gate(self) -> bool:
        return 0.0 < self.gate_half_angle_deg < 90.0

    def truth_events_rad(self, period: float = np.pi) -> tuple[float, ...]:
        """Analytic open/closed boundaries on the requested angular period.

        A centred ellipse has period pi.  The historical event benchmark uses
        that quotient and therefore has two events.  The full compiler works
        on S1=[0,2*pi), where the same two events repeat after pi.
        """
        if not self.is_partial_gate:
            return ()
        period = float(period)
        if not np.isfinite(period) or period <= 0.0:
            raise ValueError("truth event period must be finite and positive")
        center = np.deg2rad(self.gate_center_deg)
        half = np.deg2rad(self.gate_half_angle_deg)
        seeds = (center - half, center + half)
        if np.isclose(period, TWO_PI, rtol=0.0, atol=1e-14):
            seeds = (*seeds, center + np.pi - half,
                     center + np.pi + half)
        return tuple(sorted({float(value % period) for value in seeds}))

    def truth_open_intervals_rad(
            self, period: float = TWO_PI) -> tuple[tuple[float, float], ...]:
        """Canonical linear pieces of the sealed-manifest G1 open set.

        The returned pieces lie in ``[0, period]`` and never wrap.  Endpoints
        have zero measure; collision-at-contact semantics treat them as
        closed, while all measure metrics remain independent of that choice.
        This helper is valid for the full S1 period used by GMC and for the
        legacy pi quotient.
        """
        period = float(period)
        if (not np.isfinite(period) or period <= 0.0
                or not (np.isclose(period, np.pi, rtol=0.0, atol=1e-14)
                        or np.isclose(period, TWO_PI, rtol=0.0,
                                      atol=1e-14))):
            raise ValueError("G1 truth intervals require period pi or 2*pi")
        half = float(np.deg2rad(self.gate_half_angle_deg))
        if half <= 0.0:
            return ()
        if half >= np.pi / 2.0:
            return ((0.0, period),)

        center = float(np.deg2rad(self.gate_center_deg))
        centers = (center,) if np.isclose(period, np.pi) else (
            center, center + np.pi,
        )
        pieces: list[tuple[float, float]] = []
        for arc_center in centers:
            lo = float((arc_center - half) % period)
            hi = lo + 2.0 * half
            if hi <= period:
                pieces.append((lo, hi))
            else:
                pieces.extend(((lo, period), (0.0, hi - period)))
        pieces.sort()
        merged: list[tuple[float, float]] = []
        for lo, hi in pieces:
            if merged and lo <= merged[-1][1]:
                merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
            else:
                merged.append((lo, hi))
        return tuple(merged)


@dataclass(frozen=True)
class AtlasGateInstance:
    case: AtlasCase
    scene: SceneModel2D
    robot: RobotModel2D
    gate_center: np.ndarray
    gate_tangent: np.ndarray
    jamb_supports: tuple[GaussianSupport2D, GaussianSupport2D]
    robot_axes: tuple[float, float]


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _json_from_bytes(data: bytes, *, label: str):
    try:
        return json.loads(data)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise AtlasAssetError(f"invalid Atlas JSON in {label}: {exc}") from exc


def _parse_checksum_inventory(data: bytes, *, label: str) -> dict[str, str]:
    """Parse a sha256sum inventory without trusting paths or duplicates."""
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise AtlasAssetError(f"invalid UTF-8 checksum inventory: {label}") from exc
    entries: dict[str, str] = {}
    for line_number, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line:
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            raise AtlasAssetError(
                f"malformed checksum inventory line {line_number}: {label}"
            )
        digest, relative = parts
        relative = relative.lstrip("*").strip()
        rel_path = PurePosixPath(relative)
        if (len(digest) != 64
                or any(ch not in "0123456789abcdef" for ch in digest)
                or rel_path.is_absolute() or ".." in rel_path.parts):
            raise AtlasAssetError(
                f"unsafe checksum inventory line {line_number}: {label}"
            )
        if relative in entries:
            raise AtlasAssetError(
                f"duplicate checksum inventory path {relative}: {label}"
            )
        entries[relative] = digest
    return entries


class AtlasAssetStore:
    """Resolve and validate frozen Atlas assets without writing to them."""

    VALID_SPLITS = ("dev", "validation", "blind")
    RECORD_FIELDS = (
        "query_id",
        "split",
        "analytic.physically_open",
        "analytic.gate_half_angle_deg",
        "analytic.gate_center_deg",
        "measured_gate_half_angle_deg",
        "resolutions.fine.reachable_per_goal",
        "convergence_status",
    )
    MANIFEST_FIELDS = (
        "query_id",
        "scene.family",
        "scene.door_width",
        "scene.door_offset",
        "scene.door_tilt_deg",
        "robot_id",
        "start_pose",
        "goals",
        "goal_radius",
        "final_orientation",
    )

    def __init__(self, atlas_root: str | Path):
        self.root = Path(atlas_root).resolve()
        self.package_root, self.snapshot_root, self.live_root = (
            self._resolve_package_layout(self.root)
        )
        self.data_root = self.snapshot_root / "data" / "splatc_gates"
        self.records_root = self.snapshot_root / "outputs" / "oracle_records"
        self.source_root = self.snapshot_root / "src"

        inventory_path = self.package_root / "MANIFEST.sha256"
        try:
            inventory_bytes = inventory_path.read_bytes()
        except OSError as exc:
            raise AtlasAssetError(
                f"missing Atlas package checksum inventory: {inventory_path}"
            ) from exc
        self.package_manifest_path = inventory_path
        self.package_manifest_sha256 = _sha256_bytes(inventory_bytes)
        self.package_inventory = _parse_checksum_inventory(
            inventory_bytes, label=str(inventory_path),
        )

        seals_bytes, seals_sha, seals_path = self._read_package_file(
            _SPLIT_SEALS_RELATIVE,
            pinned_sha256=_PINNED_SPLIT_SEALS_SHA256,
        )
        self.seals = _json_from_bytes(seals_bytes, label=str(seals_path))
        if not isinstance(self.seals, dict) or not isinstance(
                self.seals.get("seals"), dict):
            raise AtlasAssetError(f"malformed Atlas split seals: {seals_path}")
        self._split_seals_integrity = {
            "path": str(seals_path),
            "package_relative_path": _SPLIT_SEALS_RELATIVE,
            "sha256": seals_sha,
            "package_inventory_checked": True,
            "independently_pinned": True,
        }
        self._atlas_module = None
        self._robot_registry = None
        self._source_integrity: dict | None = None

    @staticmethod
    def _resolve_package_layout(
            root: Path) -> tuple[Path, Path, Path]:
        """Accept a live Atlas root, package root, or sealed snapshot root."""
        if (root / "outputs" / "round4_final_package"
                / "MANIFEST.sha256").is_file():
            package = root / "outputs" / "round4_final_package"
            return package.resolve(), (package / "src_snapshot").resolve(), root
        if (root / "MANIFEST.sha256").is_file() and (
                root / "src_snapshot").is_dir():
            package = root
            live = package.parent.parent
            return package.resolve(), (package / "src_snapshot").resolve(), live
        if root.name == "src_snapshot" and (
                root.parent / "MANIFEST.sha256").is_file():
            package = root.parent
            live = package.parent.parent
            return package.resolve(), root, live
        raise AtlasAssetError(
            "Atlas root does not contain the sealed round4_final_package: "
            f"{root}"
        )

    def _read_package_file(
        self,
        relative: str,
        *,
        pinned_sha256: str | None = None,
    ) -> tuple[bytes, str, Path]:
        expected = self.package_inventory.get(relative)
        if expected is None:
            raise AtlasAssetError(
                f"Atlas package checksum inventory omits required input: {relative}"
            )
        path = self.package_root.joinpath(*PurePosixPath(relative).parts)
        try:
            data = path.read_bytes()
        except OSError as exc:
            raise AtlasAssetError(f"missing sealed Atlas input: {path}") from exc
        actual = _sha256_bytes(data)
        if actual != expected:
            raise AtlasAssetError(
                f"Atlas package checksum mismatch for {relative}: "
                f"{actual} != {expected}"
            )
        if pinned_sha256 is not None and actual != pinned_sha256:
            raise AtlasAssetError(
                f"Atlas pinned checksum mismatch for {relative}: "
                f"{actual} != {pinned_sha256}"
            )
        return data, actual, path

    def manifest_path(self, split: str) -> Path:
        if split not in self.VALID_SPLITS:
            raise AtlasAssetError(f"unsupported Atlas split: {split}")
        packaged = self.data_root / split / "manifest.json"
        package_relative = self._manifest_package_relative(split)
        if package_relative in self.package_inventory and packaged.is_file():
            return packaged
        # The release package deliberately withholds the blind manifest.  Its
        # live copy is still bound indirectly by the independently pinned
        # split_seals.json and remains behind the explicit allow_blind gate.
        return self.live_root / "data" / "splatc_gates" / split / "manifest.json"

    @staticmethod
    def _manifest_package_relative(split: str) -> str:
        return f"src_snapshot/data/splatc_gates/{split}/manifest.json"

    def _verify_split_payload(self, split: str) -> tuple[dict, dict]:
        path = self.manifest_path(split)
        try:
            manifest_bytes = path.read_bytes()
        except OSError as exc:
            raise AtlasAssetError(f"missing Atlas split manifest: {path}") from exc
        # The hash and JSON parse deliberately consume these same bytes.  The
        # caller receives the payload and must not reopen the manifest.
        actual_sha = _sha256_bytes(manifest_bytes)
        payload = _json_from_bytes(manifest_bytes, label=str(path))
        try:
            expected = self.seals["seals"][split]
        except KeyError as exc:
            raise AtlasAssetError(f"split seal omits Atlas split: {split}") from exc
        if actual_sha != expected["sha256"]:
            raise AtlasAssetError(
                f"{split} manifest seal mismatch: {actual_sha} != "
                f"{expected['sha256']}"
            )
        package_relative = self._manifest_package_relative(split)
        package_checked = package_relative in self.package_inventory
        if package_checked and actual_sha != self.package_inventory[package_relative]:
            raise AtlasAssetError(
                f"Atlas package checksum mismatch for {package_relative}: "
                f"{actual_sha} != {self.package_inventory[package_relative]}"
            )
        if not isinstance(payload, dict) or not isinstance(
                payload.get("episodes"), list):
            raise AtlasAssetError(f"malformed Atlas split manifest: {path}")
        actual_n = len(payload["episodes"])
        if actual_n != int(expected["n_episodes"]):
            raise AtlasAssetError(
                f"{split} episode count mismatch: {actual_n} != "
                f"{expected['n_episodes']}"
            )
        return payload, {
            "path": str(path),
            "sha256": actual_sha,
            "n_episodes": actual_n,
            "sealed_on": self.seals["sealed_on"],
            "sealed_artifact": (
                "pinned_split_seal+round4_package_inventory"
                if package_checked else "pinned_split_seal"
            ),
            "package_relative_path": (
                package_relative if package_checked else None
            ),
            "package_inventory_checked": package_checked,
            "package_manifest": {
                "path": str(self.package_manifest_path),
                "sha256": self.package_manifest_sha256,
                "externally_authenticated": False,
            },
            "split_seals": dict(self._split_seals_integrity),
            "oracle_records_sealed": False,
            "oracle_records_package_inventory_checked": False,
            "generator_source_sealed": False,
            "robot_registry_source_sealed": False,
        }

    def verify_split(self, split: str) -> dict:
        _, integrity = self._verify_split_payload(split)
        return integrity

    def load_cases(
        self,
        splits: list[str] | tuple[str, ...],
        *,
        case_ids: set[str] | None = None,
        families: set[str] | None = None,
        robot_ids: set[str] | None = None,
        partial_gate_only: bool = True,
        allow_blind: bool = False,
    ) -> tuple[list[AtlasCase], dict[str, dict]]:
        cases: list[AtlasCase] = []
        integrity: dict[str, dict] = {}
        for split in splits:
            if split == "blind" and not allow_blind:
                raise BlindSplitForbidden(
                    "Atlas blind split is sealed; pass allow_blind=True only "
                    "for a declared post-freeze evaluation"
                )
            manifest, integrity[split] = self._verify_split_payload(split)
            record_checks: list[dict] = []
            for episode in manifest["episodes"]:
                qid = str(episode["query_id"])
                if case_ids is not None and qid not in case_ids:
                    continue
                if families is not None and episode["scene"]["family"] not in families:
                    continue
                if robot_ids is not None and episode["robot_id"] not in robot_ids:
                    continue
                case = self._make_case(
                    split, episode, allow_blind=allow_blind,
                    record_checks=record_checks,
                )
                if partial_gate_only and not case.is_partial_gate:
                    if record_checks and record_checks[-1]["query_id"] == qid:
                        record_checks.pop()
                    continue
                cases.append(case)
            integrity[split]["selected_oracle_records"] = record_checks
            integrity[split]["oracle_records_package_inventory_checked"] = (
                split != "blind" and bool(record_checks)
                and all(row["package_inventory_checked"]
                        for row in record_checks)
            )
            # The checksum inventory is useful integrity evidence, but its own
            # digest is reported rather than externally pinned.  Keep the old
            # `sealed` boolean conservative and expose the precise stronger
            # per-file statement above.
            integrity[split]["oracle_records_sealed"] = False
        cases.sort(key=lambda c: (self.VALID_SPLITS.index(c.split), c.query_id))
        if case_ids is not None:
            missing = sorted(case_ids - {c.query_id for c in cases})
            if missing:
                raise AtlasAssetError(
                    "requested Atlas cases were not selected: " + ", ".join(missing)
                )
        if not cases:
            raise AtlasAssetError("Atlas filters selected zero benchmark cases")
        source_integrity = self.source_integrity()
        for split_integrity in integrity.values():
            split_integrity["runtime_source_integrity"] = source_integrity
            split_integrity["generator_source_sealed"] = True
            split_integrity["robot_registry_source_sealed"] = True
        return cases, integrity

    def _make_case(self, split: str, episode: dict,
                   *, allow_blind: bool,
                   record_checks: list[dict] | None = None) -> AtlasCase:
        qid = str(episode["query_id"])
        if split != "blind":
            relative = (
                f"src_snapshot/outputs/oracle_records/{qid}.json"
            )
            record_bytes, record_sha, record_path = self._read_package_file(
                relative
            )
            record = _json_from_bytes(record_bytes, label=str(record_path))
            if record_checks is not None:
                record_checks.append({
                    "query_id": qid,
                    "path": str(record_path),
                    "package_relative_path": relative,
                    "sha256": record_sha,
                    "package_inventory_checked": True,
                    "independently_pinned": False,
                })
            if record.get("query_id") != qid:
                raise AtlasAssetError(f"OracleRecord query_id mismatch for {qid}")
            if record.get("split") != split:
                raise AtlasAssetError(f"OracleRecord split mismatch for {qid}")
            # OracleRecord is deliberately secondary and unsealed.  Validate
            # it, then discard its analytic values and independently rebuild
            # the primary truth from the hash-sealed manifest episode.
            self._validate_record_analytic(episode, record["analytic"])
            analytic = self._manifest_analytic(episode)
            fine = tuple(bool(v) for v in
                         record["resolutions"]["fine"]["reachable_per_goal"])
            measured = float(record["measured_gate_half_angle_deg"])
            status = str(record["convergence_status"])
        else:
            if not allow_blind:
                raise BlindSplitForbidden(qid)
            record = None
            analytic = self._manifest_analytic(episode)
            fine = None
            measured = None
            status = "SEALED_BLIND_ANALYTIC_OPT_IN"
        return AtlasCase(
            query_id=qid,
            split=split,
            episode=episode,
            record=record,
            gate_center_deg=float(analytic["gate_center_deg"]),
            gate_half_angle_deg=float(analytic["gate_half_angle_deg"]),
            measured_gate_half_angle_deg=measured,
            convergence_status=status,
            fine_reachable_per_goal=fine,
        )

    def _manifest_analytic(self, episode: dict) -> dict:
        """Recompute G1 analytic truth from the sealed manifest fields."""
        a, b = self._robot_axes(str(episode["robot_id"]))
        width = float(episode["scene"]["door_width"])
        return {
            "physically_open": bool(width > 2.0 * b),
            "gate_half_angle_deg": float(np.rad2deg(
                self._gate_half_angle(a, b, width)
            )),
            "gate_center_deg": float(episode["scene"]["door_tilt_deg"]),
        }

    def _validate_record_analytic(self, episode: dict, analytic: dict) -> None:
        """Reject OracleRecord truth that drifts from its sealed episode."""
        expected = self._manifest_analytic(episode)
        if bool(analytic["physically_open"]) != expected["physically_open"]:
            raise AtlasAssetError("OracleRecord analytic open flag mismatch")
        for field in ("gate_half_angle_deg", "gate_center_deg"):
            if not np.isclose(float(analytic[field]), expected[field],
                              rtol=0.0, atol=1e-12):
                raise AtlasAssetError(
                    f"OracleRecord analytic {field} mismatch: "
                    f"{analytic[field]} != {expected[field]}"
                )

    @staticmethod
    def _gate_half_angle(a: float, b: float, w: float) -> float:
        if w <= 2.0 * b:
            return 0.0
        if w >= 2.0 * a:
            return np.pi / 2.0
        s2 = (w * w / 4.0 - b * b) / (a * a - b * b)
        return float(np.arcsin(np.sqrt(s2)))

    @staticmethod
    def _legacy_modules() -> set[str]:
        return {
            name for name in sys.modules
            if name == "splatc.gmc" or name.startswith("splatc.gmc.")
        }

    def _guarded_atlas_call(self, label: str, function, *args, **kwargs):
        """Reject only legacy modules newly loaded by this adapter call."""
        before = self._legacy_modules()
        try:
            result = function(*args, **kwargs)
        finally:
            loaded = sorted(self._legacy_modules() - before)
            if loaded:
                for name in loaded:
                    sys.modules.pop(name, None)
                raise AtlasAssetError(
                    f"Atlas legacy import guard rejected {label}: "
                    + ", ".join(loaded)
                )
        return result

    def _robot_axes(self, robot_id: str) -> tuple[float, float]:
        if self._robot_registry is None:
            module = self._load_atlas_generator()
            normal = self._guarded_atlas_call(
                "robot_library", module.robot_library,
            )
            blind = self._guarded_atlas_call(
                "blind_robot_library", module.blind_robot_library,
            )
            if set(normal).intersection(blind):
                raise AtlasAssetError("Atlas robot registries contain duplicates")
            registry = {**normal, **blind}
            checked = {}
            for name, robot in registry.items():
                a, b = float(robot.a), float(robot.b)
                if (not np.isfinite(a) or not np.isfinite(b)
                        or a <= 0.0 or b <= 0.0 or a < b):
                    raise AtlasAssetError(
                        f"malformed Atlas robot registry entry: {name}"
                    )
                checked[str(name)] = (a, b)
            self._robot_registry = checked
        try:
            return self._robot_registry[robot_id]
        except KeyError as exc:
            raise AtlasAssetError(f"unknown Atlas robot: {robot_id}") from exc

    def _load_atlas_generator(self):
        if self._atlas_module is not None:
            return self._atlas_module
        if not self.source_root.is_dir():
            raise AtlasAssetError(
                f"missing sealed Atlas source tree: {self.source_root}"
            )

        # Reject a cached splatc package from another checkout rather than
        # silently combining live and sealed modules in one import graph.
        for module_name, relative in _ATLAS_MODULE_SOURCES.items():
            cached = sys.modules.get(module_name)
            if cached is None:
                continue
            cached_file = getattr(cached, "__file__", None)
            expected_path = self.package_root.joinpath(
                *PurePosixPath(relative).parts
            ).resolve()
            if cached_file is None or Path(cached_file).resolve() != expected_path:
                raise AtlasAssetError(
                    f"cached Atlas module {module_name} is outside the sealed "
                    f"snapshot: {cached_file} != {expected_path}"
                )

        # Preflight source bytes before Python executes any of them.  A second
        # pass below rejects a file changed across the import window.
        preflight: dict[str, str] = {}
        for relative, pinned in _PINNED_ATLAS_SOURCES.items():
            _, digest, _ = self._read_package_file(
                relative, pinned_sha256=pinned,
            )
            preflight[relative] = digest

        src_s = str(self.source_root)
        inserted = not sys.path or sys.path[0] != src_s
        if inserted:
            sys.path.insert(0, src_s)
        previous = sys.dont_write_bytecode
        sys.dont_write_bytecode = True
        try:
            module = self._guarded_atlas_call(
                "splatc.datasets.g1_gate import",
                importlib.import_module,
                "splatc.datasets.g1_gate",
            )
        finally:
            sys.dont_write_bytecode = previous
            if inserted and sys.path and sys.path[0] == src_s:
                sys.path.pop(0)

        modules = {}
        for module_name, relative in _ATLAS_MODULE_SOURCES.items():
            loaded = sys.modules.get(module_name)
            if loaded is None:
                raise AtlasAssetError(
                    f"sealed generator did not load dependency: {module_name}"
                )
            module_file = getattr(loaded, "__file__", None)
            expected_path = self.package_root.joinpath(
                *PurePosixPath(relative).parts
            ).resolve()
            if module_file is None or Path(module_file).resolve() != expected_path:
                raise AtlasAssetError(
                    f"imported Atlas module path mismatch for {module_name}: "
                    f"{module_file} != {expected_path}"
                )
            _, post_digest, _ = self._read_package_file(
                relative, pinned_sha256=_PINNED_ATLAS_SOURCES[relative],
            )
            if post_digest != preflight[relative]:
                raise AtlasAssetError(
                    f"Atlas source changed during import: {relative}"
                )
            modules[module_name] = {
                "path": str(expected_path),
                "package_relative_path": relative,
                "sha256": post_digest,
                "package_inventory_checked": True,
                "independently_pinned": True,
            }
        self._source_integrity = {
            "source_root": str(self.source_root),
            "generator_module": "splatc.datasets.g1_gate",
            "generator_source_sealed": True,
            "robot_registry_source": "splatc.datasets.g1_gate:robot_library",
            "robot_registry_source_sealed": True,
            "modules": modules,
            "legacy_import_guard": {
                "forbidden_namespace": "splatc.gmc and descendants",
                "scope": "modules newly loaded by each adapter call",
                "preexisting_modules_ignored": True,
            },
        }
        self._atlas_module = module
        return module

    def source_integrity(self) -> dict:
        """Return an exact, JSON-safe account of imported Atlas sources."""
        self._load_atlas_generator()
        # Round-trip copy prevents callers mutating the store's evidence.
        return json.loads(json.dumps(self._source_integrity, sort_keys=True))

    def materialize(self, case: AtlasCase) -> AtlasGateInstance:
        """Convert the exact frozen G1 hard supports into GMC primitives."""
        if case.family != "G1":
            raise AtlasAssetError(
                f"v0 Atlas adapter supports only G1, got {case.family}"
            )
        module = self._load_atlas_generator()
        spec = case.episode["scene"]
        tilt = float(np.deg2rad(spec["door_tilt_deg"]))
        offset = float(spec["door_offset"])
        width = float(spec["door_width"])
        atlas_scene = self._guarded_atlas_call(
            "make_g1_scene",
            module.make_g1_scene,
            width,
            door_offset=offset,
            door_tilt=tilt,
        )
        a, b = self._robot_axes(case.robot_id)
        robot = ellipse_robot(a, b, level=1.0, name=case.robot_id)

        supports: list[GaussianSupport2D] = []
        group_supports: list[tuple[float, np.ndarray, int]] = []
        pid = 0
        for radius, centers in atlas_scene.disc_groups:
            for center in np.asarray(centers, dtype=float):
                support = GaussianSupport2D(
                    mean=center,
                    covariance=np.eye(2) * float(radius) ** 2,
                    level=1.0,
                    primitive_id=pid,
                )
                supports.append(support)
                group_supports.append((float(radius), center, pid))
                pid += 1

        xmin, xmax, ymin, ymax = atlas_scene.workspace
        scene = SceneModel2D(
            supports=tuple(supports),
            workspace=shapely.box(xmin, ymin, xmax, ymax),
            name=atlas_scene.scene_id,
        )
        gate_center = np.array([0.0, offset], dtype=float)
        gate_tangent = np.array([-np.sin(tilt), np.cos(tilt)], dtype=float)

        jambs = []
        for sign in (+1.0, -1.0):
            best = min(
                group_supports,
                key=lambda item: float(np.linalg.norm(
                    item[1] - (gate_center + sign * gate_tangent
                               * (width / 2.0 + item[0]))
                )),
            )
            radius, center, selected_pid = best
            target = gate_center + sign * gate_tangent * (width / 2.0 + radius)
            if np.linalg.norm(center - target) > 1e-9:
                raise AtlasAssetError(
                    f"could not identify exact jamb support for {case.query_id}"
                )
            jambs.append(scene.supports[selected_pid])

        return AtlasGateInstance(
            case=case,
            scene=scene,
            robot=robot,
            gate_center=gate_center,
            gate_tangent=gate_tangent,
            jamb_supports=(jambs[0], jambs[1]),
            robot_axes=(a, b),
        )
