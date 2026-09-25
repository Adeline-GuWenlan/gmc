"""Shared pieces of the ground 5000-pair benchmark (stage G1): config, scene, regions, crop.

Both planners (GMC and the gs3d lattice A*) and the shared gs3d replay see the same
query region ``R`` and the same cropped Gaussians.  See ``docs/ground5k_design.md`` §1.

Geometry of one query
---------------------
* ``Q`` is an axis-aligned xy box: the bounding box of start, goal and both robots'
  connectivity witnesses, padded by ``crop.pad_m`` and clipped to the scene extent.
* ``S`` is the contact strip: points where the fitted floor plane lies within
  ``contact.max_height_error_m`` of the constant floor reference ``z_floor`` (the gs3d
  ground-support rule of ``gs3d_run._constant_floor_support``).  It is two half-planes.
* ``R = Q ∩ S`` is a convex polygon.  The robot *body* stays inside ``R``: for A* it is the
  known space and the support evidence; for GMC it becomes the centre workspace
  ``R ⊖ square(r + margin)``.  Gaussians are cropped by full 3D ellipsoid AABB overlap
  with ``Q × [z_floor - z_below, z_floor + z_above]``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np

from gmc.gs3d.contracts import BodySpec, Pose3, SceneSpec
from gmc.gs3d.robots import CYLINDER, SWEEPER, EvidenceBoundedPlaneSupport, crop_by_support_aabb

CONFIG = Path(__file__).resolve().parents[1] / "configs" / "ground5k.json"
BODIES = {"sweeper": SWEEPER, "cylinder": CYLINDER}
COMBOS = ("gmc_sweeper", "gmc_cylinder", "astar_sweeper", "astar_cylinder")
OUTCOMES = ("SUCCESS_VERIFIED", "FAIL_NO_PATH", "FAIL_UNKNOWN", "FAIL_BUDGET", "FAIL_REPLAY", "ERROR")


def load_config(path=CONFIG) -> dict:
    return json.loads(Path(path).read_text())


def sha256(path) -> str:
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(8 << 20), b""):
            h.update(block)
    return h.hexdigest()


def write_json_atomic(path, payload) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp{os.getpid()}")
    tmp.write_text(json.dumps(payload, indent=1, sort_keys=True, allow_nan=False) + "\n")
    tmp.replace(path)
    return path


# ------------------------------------------------------------------------------ scene --

def load_scene(cfg: dict, name: str):
    """``(GaussianScene3D, floor)`` for a configured scene, hash-checked, read-only."""
    spec = cfg["scenes"][name]
    if spec["loader"] == "uavlamp":
        from gmc.gs3d.scene_uavlamp import load_uavlamp_derivative
        scene, doc = load_uavlamp_derivative(spec["archive"], spec["manifest"])
        if doc["derivative"]["sha256"] != spec["sha256"]:
            raise ValueError("scene_v2 manifest hash differs from the frozen config")
        with np.load(spec["archive"], allow_pickle=False) as d:
            meta = json.loads(str(d["meta"]))
    elif spec["loader"] == "planefloor":
        from gmc.height.planefloor import load_plane_scene
        if sha256(spec["archive"]) != spec["sha256"]:
            raise ValueError("plane-floor archive hash differs from the frozen config")
        scene, meta = load_plane_scene(spec["archive"])
    else:
        raise ValueError(f"unknown loader {spec['loader']!r}")
    return scene, floor_info(meta)


def floor_info(meta: dict) -> dict:
    floor = meta["floor"]
    return {"z_floor": float(meta["z_floor"]), "normal": [float(v) for v in floor["normal"]],
            "centroid": [float(v) for v in floor["centroid"]],
            "extent": [float(v) for v in meta["extent"]],
            "ceiling_height_m": float(meta["ceiling_height_m"])}


# ------------------------------------------------------------------ half-plane regions --

@dataclass(frozen=True)
class ContactStrip:
    """``|fitted_z(x, y) - z_floor| <= max_err`` as ``dev = ax*x + ay*y + c``."""

    ax: float
    ay: float
    c: float
    max_err: float

    @classmethod
    def from_floor(cls, floor: dict, max_err: float) -> "ContactStrip":
        n, p = np.asarray(floor["normal"], float), np.asarray(floor["centroid"], float)
        # fitted_z = p_z - (n_x (x - p_x) + n_y (y - p_y)) / n_z
        ax, ay = -n[0] / n[2], -n[1] / n[2]
        c = p[2] - float(floor["z_floor"]) - ax * p[0] - ay * p[1]
        return cls(float(ax), float(ay), float(c), float(max_err))

    def dev(self, x, y):
        return self.ax * np.asarray(x, float) + self.ay * np.asarray(y, float) + self.c

    INSET_M = 1e-6   # vertices on the strip edge must stay below the limit after roundoff

    def halfplanes(self):
        """Rows ``(a_x, a_y, b)`` meaning ``a . p <= b`` (inset by 1 um in deviation)."""
        e = self.max_err - self.INSET_M
        return [(self.ax, self.ay, e - self.c), (-self.ax, -self.ay, e + self.c)]


class Region:
    """Convex xy polygon ``{p : a . p <= b}`` times a closed z interval.

    Implements the gs3d ``KnownSpace`` protocol (``contains_aabb``) and the ``lower`` /
    ``upper`` attributes ``EvidenceBoundedPlaneSupport`` reads.  ``faces`` names every
    half-plane; ``rejections`` counts, per face, how often a box was refused because of
    it.  That counter is observation only and never changes an answer.
    """

    def __init__(self, halfplanes, faces, z_range, box):
        self.A = np.asarray([h[:2] for h in halfplanes], float)
        self.b = np.asarray([h[2] for h in halfplanes], float)
        self.faces = tuple(faces)
        self.z = (float(z_range[0]), float(z_range[1]))
        self.box = tuple(map(float, box))
        self.lower = (self.box[0], self.box[1], self.z[0])
        self.upper = (self.box[2], self.box[3], self.z[1])
        self.rejections = {f: 0 for f in (*self.faces, "z")}

    @classmethod
    def from_box(cls, box, z_range, strip: ContactStrip | None = None) -> "Region":
        x0, y0, x1, y1 = map(float, box)
        hp = [(-1., 0., -x0), (1., 0., x1), (0., -1., -y0), (0., 1., y1)]
        faces = ["x_min", "x_max", "y_min", "y_max"]
        if strip is not None:
            hp += strip.halfplanes()
            faces += ["contact_hi", "contact_lo"]
        return cls(hp, faces, z_range, box)

    def contains_points(self, xy, tol=1e-12) -> np.ndarray:
        xy = np.atleast_2d(np.asarray(xy, float))
        return np.all(xy @ self.A.T <= self.b + tol, axis=1)

    def contains_aabb(self, lower, upper) -> bool:
        lo, hi = np.asarray(lower, float), np.asarray(upper, float)
        if lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([*lo, *hi]).all() or np.any(lo > hi):
            return False
        if lo[2] < self.z[0] or hi[2] > self.z[1]:
            self.rejections["z"] += 1
            return False
        corners = np.array([[lo[0], lo[1]], [lo[0], hi[1]], [hi[0], lo[1]], [hi[0], hi[1]]])
        bad = np.any(corners @ self.A.T > self.b + 1e-12, axis=0)
        if bad.any():
            for k in np.flatnonzero(bad):
                self.rejections[self.faces[k]] += 1
            return False
        return True

    def polygon(self, erode: float = 0.0):
        """Shapely polygon of the region, each half-plane moved inwards so that a centred
        axis-aligned square of half-side ``erode`` stays inside (Minkowski erosion)."""
        from shapely.geometry import Polygon, box as sbox
        x0, y0, x1, y1 = self.box
        big = max(x1 - x0, y1 - y0) * 4 + 100.
        poly = sbox(x0 - big, y0 - big, x1 + big, y1 + big)
        for (ax, ay), b in zip(self.A, self.b):
            b_e = b - erode * (abs(ax) + abs(ay))
            poly = poly.intersection(_halfplane_polygon(ax, ay, b_e, poly.bounds))
            if poly.is_empty:
                return poly
        return Polygon(poly.exterior.coords) if poly.geom_type == "Polygon" else poly

    def vertices(self) -> np.ndarray:
        return np.asarray(self.polygon().exterior.coords[:-1], float)


def _halfplane_polygon(ax, ay, b, bounds):
    """A polygon equal to ``{a . p <= b}`` inside ``bounds`` (a large box)."""
    from shapely.geometry import Polygon
    x0, y0, x1, y1 = bounds
    corners = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)
    val = corners @ np.array([ax, ay]) - b
    pts = []
    for i in range(4):
        p, q = corners[i], corners[(i + 1) % 4]
        vp, vq = val[i], val[(i + 1) % 4]
        if vp <= 0:
            pts.append(p)
        if (vp < 0 < vq) or (vq < 0 < vp):
            t = vp / (vp - vq)
            pts.append(p + t * (q - p))
    return Polygon(pts) if len(pts) >= 3 else Polygon()


class RasterRegion:
    """Hall-wide known floor: an evidence raster intersected with the contact strip.

    ``contains_aabb`` accepts a box only if every raster cell its xy extent touches is
    observed floor, all four xy corners lie in the contact strip and z is in range.
    Used by the sampler and the connectivity witness only; the planners get ``Region``.
    """

    def __init__(self, mask, origin, cell, z_range, strip: ContactStrip):
        self.mask = np.asarray(mask, bool)
        self.origin = (float(origin[0]), float(origin[1]))
        self.cell = float(cell)
        self.z = (float(z_range[0]), float(z_range[1]))
        self.strip = strip
        nx, ny = self.mask.shape
        self.lower = (self.origin[0], self.origin[1], self.z[0])
        self.upper = (self.origin[0] + nx * self.cell, self.origin[1] + ny * self.cell, self.z[1])
        self._hp = Region.from_box((*self.lower[:2], *self.upper[:2]), self.z, strip)

    def contains_aabb(self, lower, upper) -> bool:
        if not self._hp.contains_aabb(lower, upper):
            return False
        i0, j0 = self._cell(lower[0], lower[1])
        i1, j1 = self._cell(upper[0], upper[1])
        nx, ny = self.mask.shape
        if i0 < 0 or j0 < 0 or i1 >= nx or j1 >= ny:
            return False
        return bool(self.mask[i0:i1 + 1, j0:j1 + 1].all())

    def _cell(self, x, y):
        return (int(math.floor((float(x) - self.origin[0]) / self.cell)),
                int(math.floor((float(y) - self.origin[1]) / self.cell)))

    def contains_points(self, xy) -> np.ndarray:
        xy = np.atleast_2d(np.asarray(xy, float))
        i = np.floor((xy[:, 0] - self.origin[0]) / self.cell).astype(int)
        j = np.floor((xy[:, 1] - self.origin[1]) / self.cell).astype(int)
        nx, ny = self.mask.shape
        ok = (i >= 0) & (j >= 0) & (i < nx) & (j < ny)
        out = np.zeros(len(xy), bool)
        out[ok] = self.mask[i[ok], j[ok]]
        return out & self._hp.contains_points(xy)


# -------------------------------------------------------------------- support + poses --

def z_range(cfg: dict, floor: dict):
    c = cfg["crop"]
    return floor["z_floor"] - c["z_below_floor_m"], floor["z_floor"] + c["z_above_floor_m"]


def make_support(region, floor: dict, cfg: dict, height_error: float):
    """Constant floor reference ``z_floor`` (the GMC band origin and gs3d A5 manifold),
    bounded by ``region`` as floor evidence.  ``height_error`` is the fitted-plane
    deviation over that region; the gs3d constructor refuses it above 0.05 m."""
    ct = cfg["contact"]
    return EvidenceBoundedPlaneSupport(
        (0., 0., 1.), (0., 0., floor["z_floor"]), region,
        height_error_m=float(height_error), max_height_error_m=ct["max_height_error_m"],
        max_travel_m=ct["max_travel_m"], max_slope_deg=ct["max_slope_deg"])


def ground_pose(x, y, body: BodySpec, floor: dict, yaw: float = 0.0) -> Pose3:
    return Pose3((float(x), float(y), floor["z_floor"] + body.ground_clearance_m + body.half_height_m),
                 float(yaw))


# ------------------------------------------------------------------------ query crop --

def query_box(points_xy, pad: float, extent) -> tuple:
    pts = np.asarray(points_xy, float)
    x0, y0 = pts.min(0) - pad
    x1, y1 = pts.max(0) + pad
    return (max(x0, extent[0]), max(y0, extent[1]), min(x1, extent[2]), min(y1, extent[3]))


def query_region(box, cfg: dict, floor: dict) -> Region:
    strip = ContactStrip.from_floor(floor, cfg["contact"]["max_height_error_m"])
    return Region.from_box(box, z_range(cfg, floor), strip)


def region_height_error(region: Region, floor: dict, cfg: dict) -> float:
    strip = ContactStrip.from_floor(floor, cfg["contact"]["max_height_error_m"])
    v = region.vertices()
    return float(np.max(np.abs(strip.dev(v[:, 0], v[:, 1]))))


def crop_query(scene3d, region: Region, floor: dict, cfg: dict, scene_id: str):
    """SceneSpec for one query: the same Gaussians for both planners and the replay."""
    lo = (region.box[0], region.box[1], region.z[0])
    hi = (region.box[2], region.box[3], region.z[1])
    cropped, crop = crop_by_support_aabb(scene3d, lo, hi, level=cfg["level"], tau=cfg["tau"])
    support = make_support(region, floor, cfg, region_height_error(region, floor, cfg))
    spec = SceneSpec(scene_id, cropped, lo, hi, cfg["tau"], cfg["level"], region, support,
                     {"coverage_policy": "assumed_map_domain_query_region", "crop": crop,
                      "region_faces": list(region.faces), "seed": 0,
                      "tau": cfg["tau"], "level": cfg["level"]})
    return spec, crop
