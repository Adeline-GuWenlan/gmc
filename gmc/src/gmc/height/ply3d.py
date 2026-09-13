"""3D Gaussian splat scenes: PLY decoding, cropping, floor fit (spec §3.1, §5.3).

Standard 3DGS PLY: ``scale_*`` are log standard deviations, ``rot_0..3`` a
(w, x, y, z) quaternion, ``opacity`` a logit.  Only the fields listed in
``REQUIRED`` are read; spherical-harmonic colour is ignored.
"""
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.special import expit

REQUIRED = ("x", "y", "z", "opacity", "scale_0", "scale_1", "scale_2",
            "rot_0", "rot_1", "rot_2", "rot_3")


@dataclass(frozen=True)
class GaussianScene3D:
    means: np.ndarray
    covs: np.ndarray
    opacity: np.ndarray
    ids: np.ndarray
    name: str = "scene3d"

    def __post_init__(self):
        means = np.asarray(self.means, dtype=np.float64)
        covs = np.asarray(self.covs, dtype=np.float64)
        opacity = np.asarray(self.opacity, dtype=np.float64)
        ids = np.asarray(self.ids, dtype=np.int64)
        n = len(means)
        if means.shape != (n, 3) or covs.shape != (n, 3, 3):
            raise ValueError("means must be (N,3) and covs (N,3,3)")
        if opacity.shape != (n,) or ids.shape != (n,):
            raise ValueError("opacity and ids must be (N,)")
        if not (np.isfinite(means).all() and np.isfinite(covs).all()
                and np.isfinite(opacity).all()):
            raise ValueError("scene arrays must be finite")
        if len(np.unique(ids)) != n:
            raise ValueError("ids must be unique")
        for key, value in (("means", means), ("covs", covs),
                           ("opacity", opacity), ("ids", ids)):
            object.__setattr__(self, key, value)

    def __len__(self):
        return len(self.means)

    def subset(self, sel) -> "GaussianScene3D":
        return GaussianScene3D(self.means[sel], self.covs[sel],
                               self.opacity[sel], self.ids[sel], self.name)

    def aabb(self, level: float):
        half = float(level) * np.sqrt(np.einsum("nii->ni", self.covs))
        return self.means - half, self.means + half


def covariances_from_scale_rot(log_scale, quat_wxyz) -> np.ndarray:
    s2 = np.exp(2.0 * np.asarray(log_scale, dtype=np.float64))
    q = np.asarray(quat_wxyz, dtype=np.float64)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - w * z)
    R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y)
    R[:, 2, 1] = 2 * (y * z + w * x)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    C = (R * s2[:, None, :]) @ R.transpose(0, 2, 1)
    return 0.5 * (C + C.transpose(0, 2, 1))


def _read_header(path: Path):
    with open(path, "rb") as f:
        head = f.read(65536)
    end = head.find(b"end_header\n")
    if not head.startswith(b"ply\n") or end < 0:
        raise ValueError(f"{path}: not a PLY file")
    lines = head[:end].decode("ascii").splitlines()
    if "format binary_little_endian 1.0" not in lines:
        raise ValueError(f"{path}: only binary_little_endian PLY is supported")
    n = next(int(l.split()[2]) for l in lines if l.startswith("element vertex"))
    props = []
    for l in lines:
        if l.startswith("property"):
            _, typ, name = l.split()
            if typ != "float":
                raise ValueError(f"{path}: property {name} is {typ}, not float")
            props.append(name)
    missing = [p for p in REQUIRED if p not in props]
    if missing:
        raise ValueError(f"{path}: missing PLY fields {missing}")
    return n, props, end + len(b"end_header\n")


def load_3dgs_ply(path, name=None):
    path = Path(path)
    n, props, offset = _read_header(path)
    raw = np.fromfile(path, dtype=[(p, "<f4") for p in props], count=n,
                      offset=offset)
    cols = {p: raw[p].astype(np.float64) for p in REQUIRED}
    finite = np.logical_and.reduce([np.isfinite(cols[p]) for p in REQUIRED])
    idx = np.flatnonzero(finite)
    means = np.column_stack([cols["x"], cols["y"], cols["z"]])[idx]
    log_scale = np.column_stack([cols[f"scale_{k}"] for k in range(3)])[idx]
    quat = np.column_stack([cols[f"rot_{k}"] for k in range(4)])[idx]
    scene = GaussianScene3D(means, covariances_from_scale_rot(log_scale, quat),
                            expit(cols["opacity"][idx]), idx.astype(np.int64),
                            name or path.stem)
    return scene, {"rows": int(n), "nonfinite_dropped": int(n - idx.size)}


def fit_floor(scene, *, min_opacity=0.5, bin_width=0.02, band=0.15,
              iters=400, tol=0.02, seed=0) -> dict:
    """Lowest dense horizontal plane of opaque splat means (RANSAC + SVD)."""
    pts = scene.means[scene.opacity > min_opacity]
    if len(pts) < 100:
        raise ValueError("fit_floor needs at least 100 opaque splats")
    z = pts[:, 2]
    lo, hi = np.percentile(z, [0.1, 99.9])
    counts, edges = np.histogram(z, bins=np.arange(lo, hi + bin_width,
                                                   bin_width))
    thresh = 0.2 * counts.max()
    peaks = [i for i in range(len(counts))
             if counts[i] >= thresh
             and counts[i] >= counts[max(i - 1, 0)]
             and counts[i] >= counts[min(i + 1, len(counts) - 1)]]
    i0 = peaks[0]
    peak_z = 0.5 * (edges[i0] + edges[i0 + 1])
    cand = pts[np.abs(z - peak_z) <= band]
    rng = np.random.default_rng(seed)
    best_k, best_n, best_p = -1, None, None
    for _ in range(iters):
        s = cand[rng.choice(len(cand), 3, replace=False)]
        nrm = np.cross(s[1] - s[0], s[2] - s[0])
        norm = np.linalg.norm(nrm)
        if norm < 1e-12:
            continue
        nrm = nrm / norm
        nrm = nrm if nrm[2] >= 0 else -nrm
        if nrm[2] < np.cos(np.deg2rad(20.0)):
            continue
        k = int((np.abs((cand - s[0]) @ nrm) <= tol).sum())
        if k > best_k:
            best_k, best_n, best_p = k, nrm, s[0]
    if best_n is None:
        raise ValueError("no near-horizontal floor plane found")
    inl = cand[np.abs((cand - best_p) @ best_n) <= tol]
    centroid = inl.mean(axis=0)
    nrm = np.linalg.svd(inl - centroid, full_matrices=False)[2][-1]
    nrm = nrm if nrm[2] >= 0 else -nrm
    return {"z_floor": float(centroid[2]), "normal": nrm.tolist(),
            "centroid": centroid.tolist(),
            "tilt_deg": float(np.degrees(np.arccos(np.clip(nrm[2], -1, 1)))),
            "n_inliers": int(len(inl)), "peak_z": float(peak_z)}


def gravity_rotation(normal) -> np.ndarray:
    n = np.asarray(normal, dtype=np.float64)
    n = n / np.linalg.norm(n)
    v = np.cross(n, [0.0, 0.0, 1.0])
    s, c = np.linalg.norm(v), float(n[2])
    if s < 1e-15:
        return np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - c) / s ** 2)


def rotate_scene(scene, R, origin) -> GaussianScene3D:
    R = np.asarray(R, dtype=np.float64)
    o = np.asarray(origin, dtype=np.float64)
    means = (scene.means - o) @ R.T + o
    covs = R @ scene.covs @ R.T
    return GaussianScene3D(means, 0.5 * (covs + covs.transpose(0, 2, 1)),
                           scene.opacity, scene.ids, scene.name)


def crop_box(scene, lo, hi):
    keep = np.all((scene.means >= lo) & (scene.means <= hi), axis=1)
    return scene.subset(keep), int((~keep).sum())
