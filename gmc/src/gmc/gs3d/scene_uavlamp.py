"""Deterministic 'UAV under the lamp' derivative of the plane-floor hall (stage L1).

Edits are additions only: dense grids of flat Gaussians forming panels (partition,
drop ceiling, bulkhead, lamp housing).  A panel is a rectangle of sample centres
with spacing <= ``spacing_m``; every sample has in-plane semiaxis ``spacing_m``
and normal semiaxis ``half_thickness_m`` at ``level = 2``.  Because the in-plane
semiaxis is at least the step, the union of 2-sigma supports is a slab with no
holes; ``panel_min_half_thickness`` is its proven minimum half thickness.

Geometry is authored in a site ("route") frame: ``world = local @ R + origin``,
with ``origin[2]`` the plane floor, so local z is height above the floor.
The planner and renderer must read the same archive this module writes.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Any
import zipfile

import numpy as np

from gmc.height.ply3d import GaussianScene3D

TAU = 0.3
LEVEL = 2.0
_AXES = {"u": 0, "v": 1, "z": 2}


def sha256(path) -> str:
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(8 << 20), b""):
            h.update(block)
    return h.hexdigest()


@dataclass(frozen=True)
class Frame:
    origin: np.ndarray
    R: np.ndarray  # world_to_route rows

    @classmethod
    def from_dict(cls, d) -> "Frame":
        R = np.asarray(d["world_to_route"], float)
        if not np.allclose(R @ R.T, np.eye(3), atol=1e-10):
            raise ValueError("world_to_route must be orthonormal")
        return cls(np.asarray(d["origin_world_m"], float), R)

    def to_world(self, local):
        return np.asarray(local, float) @ self.R + self.origin

    def to_route(self, world):
        return (np.asarray(world, float) - self.origin) @ self.R.T


def _grid(size: float, spacing: float) -> np.ndarray:
    n = int(np.ceil(size / spacing - 1e-9)) + 1 if size > 0 else 1
    return np.linspace(-size / 2, size / 2, n)


def _panel_steps(edit) -> tuple[float, float]:
    steps = []
    for size in edit["size_m"]:
        g = _grid(float(size), float(edit["spacing_m"]))
        steps.append(float(g[1] - g[0]) if len(g) > 1 else 0.)
    return steps[0], steps[1]


def panel_min_half_thickness(edit) -> float:
    """Minimum half thickness of the union slab over the sample rectangle.

    The worst in-plane point is a grid-cell centre at (da/2, db/2) from its
    nearest sample; that sample's support there has normal half-extent
    ``t * sqrt(1 - ((da/2)^2 + (db/2)^2) / s^2)``.
    """
    da, db = _panel_steps(edit)
    s, t = float(edit["spacing_m"]), float(edit["half_thickness_m"])
    q = ((da / 2) ** 2 + (db / 2) ** 2) / s ** 2
    if q >= 1:
        raise ValueError("panel samples too sparse: union has holes")
    return t * float(np.sqrt(1 - q))


def edit_gaussians(edit, frame: Frame) -> tuple[np.ndarray, np.ndarray]:
    """World means and covariances of one panel edit, in deterministic order."""
    if edit["kind"] != "panel":
        raise ValueError(f"unsupported edit kind {edit['kind']!r}")
    plane = edit["plane"]
    a, b = _AXES[plane[0]], _AXES[plane[1]]
    n = 3 - a - b
    ga = _grid(float(edit["size_m"][0]), float(edit["spacing_m"]))
    gb = _grid(float(edit["size_m"][1]), float(edit["spacing_m"]))
    A, B = np.meshgrid(ga, gb, indexing="ij")
    local = np.tile(np.asarray(edit["center_route"], float), (A.size, 1))
    local[:, a] += A.ravel()
    local[:, b] += B.ravel()
    semi = np.empty(3)
    semi[a] = semi[b] = float(edit["spacing_m"])
    semi[n] = float(edit["half_thickness_m"])
    cov_local = np.diag((semi / LEVEL) ** 2)
    cov_world = frame.R.T @ cov_local @ frame.R
    means = frame.to_world(local)
    return means, np.repeat(cov_world[None], len(means), axis=0)


def max_closed_gap(axis: str, body, margin: float, resolution: float) -> float:
    """Largest gap (between sample rows of two panels) the design treats as closed.

    Body+margin envelope is ``2(r+m)`` laterally, ``2(h+m)`` vertically; a gap one
    lattice cell narrower than that is closed with a cell of safety.
    """
    extent = body.radius_m if axis in ("u", "v") else body.half_height_m
    return 2 * (extent + margin) - resolution


def lamp_box_edits(edit_id: str, *, center_uv, size_uv, underside_z, height, spacing,
                   half_thickness, housing_rgb=(.12, .12, .13), diffuser_rgb=(1., .93, .72),
                   opacity=.95, justification="", contact="") -> list[dict[str, Any]]:
    """Closed rectangular light-box shell; outer solid underside exactly at ``underside_z``."""
    cu, cv = map(float, center_uv)
    su_, sv = map(float, size_uv)
    t = float(half_thickness)
    zb, zt = float(underside_z) + t, float(underside_z) + float(height) - t
    zc, hz = (zb + zt) / 2, zt - zb

    def p(suffix, center, plane, size, rgb):
        return {"edit_id": f"{edit_id}_{suffix}", "parent": edit_id, "role": "lamp", "kind": "panel",
                "center_route": [float(x) for x in center], "plane": plane,
                "size_m": [float(x) for x in size], "spacing_m": float(spacing),
                "half_thickness_m": t, "color_rgb": list(map(float, rgb)), "opacity": float(opacity),
                "justification": justification, "contact": contact}

    return [p("diffuser", (cu, cv, zb), "uv", (su_ - 2 * t, sv - 2 * t), diffuser_rgb),
            p("top", (cu, cv, zt), "uv", (su_ - 2 * t, sv - 2 * t), housing_rgb),
            p("side_umin", (cu - su_ / 2 + t, cv, zc), "vz", (sv - 2 * t, hz), housing_rgb),
            p("side_umax", (cu + su_ / 2 - t, cv, zc), "vz", (sv - 2 * t, hz), housing_rgb),
            p("end_vmin", (cu, cv - sv / 2 + t, zc), "uz", (su_ - 2 * t, hz), housing_rgb),
            p("end_vmax", (cu, cv + sv / 2 - t, zc), "uz", (su_ - 2 * t, hz), housing_rgb)]


def expand_edits(edits) -> list[dict[str, Any]]:
    out = []
    for e in edits:
        if e["kind"] == "lamp_box":
            kw = {k: e[k] for k in ("center_uv", "size_uv", "underside_z", "height", "spacing",
                                    "half_thickness")}
            for k in ("housing_rgb", "diffuser_rgb", "opacity", "justification", "contact"):
                if k in e:
                    kw[k] = e[k]
            out.extend(lamp_box_edits(e["edit_id"], **kw))
        else:
            out.append(dict(e))
    return out


def _solid_aabb_route(edit) -> dict[str, list[float]]:
    a, b = _AXES[edit["plane"][0]], _AXES[edit["plane"][1]]
    half = np.full(3, float(edit["half_thickness_m"]))
    half[a] = edit["size_m"][0] / 2 + edit["spacing_m"]
    half[b] = edit["size_m"][1] / 2 + edit["spacing_m"]
    c = np.asarray(edit["center_route"], float)
    return {"lower": (c - half).tolist(), "upper": (c + half).tolist()}


def build_test_only(item, frame) -> GaussianScene3D:
    """Rows for in-memory test-only variants (plugs); never written to an archive."""
    frame = frame if isinstance(frame, Frame) else Frame(frame.origin, frame.R)
    means, covs = [], []
    for e in expand_edits([item]):
        m, c = edit_gaussians(e, frame)
        means.append(m); covs.append(c)
    means, covs = np.vstack(means), np.concatenate(covs)
    return GaussianScene3D(means, covs, np.full(len(means), float(item.get("opacity", .95))),
                           np.arange(len(means)), "test_only")


def _savez_deterministic(path, **arrays):
    """``np.savez`` layout with fixed zip timestamps, so equal inputs give equal bytes."""
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_STORED, allowZip64=True) as zf:
        for name, value in arrays.items():
            info = zipfile.ZipInfo(name + ".npy", date_time=(1980, 1, 1, 0, 0, 0))
            with zf.open(info, "w", force_zip64=True) as f:
                np.lib.format.write_array(f, np.asanyarray(value), allow_pickle=False)


def build_uavlamp_derivative(config, output, manifest) -> dict[str, Any]:
    """Append the configured edits to the read-only source; write archive + manifest."""
    output, manifest = Path(output), Path(manifest)
    source = Path(config["source"])
    before = {"path": str(source), "bytes": source.stat().st_size, "sha256": sha256(source)}
    if config.get("source_sha256") and config["source_sha256"] != before["sha256"]:
        raise ValueError("source hash differs from config")
    with np.load(source, allow_pickle=False) as d:
        means, covs, opacity, ids = d["means"], d["covs"], d["opacity"], d["ids"]
        base_meta = json.loads(str(d["meta"])) if "meta" in d.files else {}
    frame = Frame.from_dict(config["frame"])
    edits = expand_edits(config["edits"])
    next_id = int(ids.max()) + 1
    add_m, add_c, add_o, add_i, entries = [], [], [], [], []
    for e in edits:
        m, c = edit_gaussians(e, frame)
        k = len(m)
        add_m.append(m); add_c.append(c)
        add_o.append(np.full(k, float(e["opacity"])))
        add_i.append(np.arange(next_id, next_id + k, dtype=np.int64))
        entries.append({**e, "id_range": [next_id, next_id + k - 1], "count": k,
                        "actual_step_m": list(_panel_steps(e)),
                        "min_half_thickness_m": panel_min_half_thickness(e),
                        "solid_aabb_route_m": _solid_aabb_route(e), "manual_edit": True})
        next_id += k
    out_means = np.vstack([means] + add_m)
    out_covs = np.concatenate([covs] + add_c)
    out_opacity = np.concatenate([opacity] + add_o)
    out_ids = np.concatenate([ids] + add_i)
    config_sha = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()
    archive_meta = {**base_meta, "uavlamp_scene_id": config["scene_id"], "uavlamp_source": str(source),
                    "uavlamp_source_sha256": before["sha256"], "uavlamp_config_sha256": config_sha,
                    "uavlamp_first_edit_id": int(ids.max()) + 1}
    output.parent.mkdir(parents=True, exist_ok=True)
    tmp = output.with_name(output.name + ".tmp.npz")
    _savez_deterministic(tmp, means=out_means, covs=out_covs, opacity=out_opacity, ids=out_ids,
                         meta=np.array(json.dumps(archive_meta, sort_keys=True)))
    tmp.replace(output)
    after = {"path": str(source), "bytes": source.stat().st_size, "sha256": sha256(source)}
    if before != after:
        raise RuntimeError("read-only source changed during build")
    doc = {"schema_version": "uavlamp.scene.v1", "scene_id": config["scene_id"],
           "units": "metres; covariance m^2; world xyz, +z up; route/site frame local z = height above floor",
           "source": before, "source_unchanged_after_build": True,
           "derivative": {"path": str(output), "bytes": output.stat().st_size, "sha256": sha256(output),
                          "n_base_gaussians": int(len(ids)), "n_output_gaussians": int(len(out_ids)),
                          "n_added": int(len(out_ids) - len(ids)),
                          "builder_code_sha256": sha256(Path(__file__))},
           "config_sha256": config_sha, "frame": config["frame"], "tau": TAU, "level": LEVEL,
           "edits": entries, "query": config.get("query"),
           "real_geometry": config.get("real_geometry"), "notes": config.get("notes", [])}
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(json.dumps(doc, indent=1) + "\n")
    return doc


def load_uavlamp_derivative(path, manifest) -> tuple[GaussianScene3D, dict[str, Any]]:
    """Load the exact planning/render archive; fail closed on hash or row mismatch."""
    doc = json.loads(Path(manifest).read_text())
    if doc["derivative"]["sha256"] != sha256(path):
        raise ValueError("derivative hash differs from its manifest")
    with np.load(path, allow_pickle=False) as d:
        scene = GaussianScene3D(d["means"], d["covs"], d["opacity"], d["ids"], doc["scene_id"])
    return scene, doc
