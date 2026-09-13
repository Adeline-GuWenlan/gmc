# Height-Band Robots Implementation Plan

> **For agentic workers:** this plan is executed by the unattended ClaudeJobs chain
> (A1 → C1 → A2 → A3 → C2, Task 13), as the user requested, not by in-session
> subagents. Inside a stage, work task by task with superpowers:executing-plans
> discipline (TDD, one commit per task). Steps use checkbox (`- [ ]`) syntax; tick
> them in your stage's state file, not in this document.

**Goal:** Project one 3DGS scene into a separate 2D obstacle map per robot height
band, run the unchanged certified GMC compiler on each map, check every path
independently in 3D, and show with videos whether and how a sweeper, a cylinder
and a UAV get through, first on toy scenes and then on the showcase gallery GS.

**Architecture:** New package `gmc/src/gmc/height/`. Each 3D splat becomes a
certified outer ellipse of its band-clipped xy-shadow (`band_shadow`), assembled
into a `SceneModel2D` per robot (`project`). GMC compiles and queries that map
unchanged (`run`). `replay3d` re-checks the path against the original 3D
ellipsoids with separating-direction certificates. `viz` renders top-down + side
videos and a 3D PLY.

**Tech Stack:** Python 3.13 (`gmc-venv`), numpy, scipy, shapely, matplotlib,
pytest; SLURM; Claude Code CLI for the agent chain.

**Spec:** `docs/superpowers/specs/2026-09-12-height-band-robots-design.md`. Read it
in full before any task; the plan argues from it.

## Global Constraints

- Python: `/scratch/wg2381/.conda/envs/gmc-venv/bin/python` (3.13.5; numpy 2.1.3,
  scipy 1.15.3, shapely 2.1.2, matplotlib 3.10.0, imageio 2.37.0, pytest 8.3.4).
  Install nothing. MP4 output only if `import imageio_ffmpeg` succeeds; otherwise GIF.
- Run everything from `gmc/` with `PYTHONPATH=src:experiments MPLBACKEND=Agg`.
  Test command: `PYTHONPATH=src:experiments $PY -m pytest -q <path>`.
- Units are metres. Scene level `ρ = 2.0`. Opacity threshold `τ = 0.3`
  (showcase sensitivity arm `τ = 0.1`). Robot ground clearance `z_lo = 0.02`.
  Replay motion bound `δ = 0.01`. Outer-ellipse inflation: relative `1e-9`, then
  covariance eigenvalue floor `1e-9`.
- Robots (verbatim from spec §3.3): `ellipse_toy` ellipse a=0.50 b=0.20, 2D only;
  `sweeper` disc r=0.175, band [0.02, 0.10]; `quadruped` ellipse a=0.35 b=0.16,
  band [0.02, 0.45]; `cylinder` disc r=0.30, band [0.02, 1.75]; `uav` disc r=0.25,
  band [z_c−0.10, z_c+0.10], z_c=1.20 on the toy.
- Read-only code: `gmc/src/gmc/{geometry,orientation,mobility,verification,spatial,reporting,coreset,dynamics,benchmarking}/`,
  `types.py`, `config.py`, `synth.py`, `io/`. A bug found there is written to the
  worklog and escalated in the standup, never patched.
- Read-only data: `/scratch/sy2366/...`. Write only inside `/scratch/wg2381/`.
- Paths: worktree `/scratch/wg2381/splathjb-height` (branch `height-bands`);
  showcase data `/scratch/wg2381/splathjb/splatc_atlas/data/gs_scenes/showcase/`;
  large outputs `/scratch/wg2381/splathjb/gmc/outputs/height/`; committed results
  `gmc/results/height/` (JSON, PNG, videos ≤ 20 MB).
- Compute: tests, toy runs and rendering may run inside the agent's own allocation.
  Showcase compile+query runs only as sbatch jobs (Task 11). `scancel` only IDs in
  your stage's `jobids` file.
- Gates are never loosened; scene semantics (τ, ρ, filters, robots, window once
  chosen) are never changed to rescue a run. A failed gate is recorded with evidence
  and the stage moves on to what does not depend on it.
- Test files are named `test_height_*.py` (spec §4 lists them without the prefix); the prefix
  keeps them from colliding with the existing suite.
- Visualize before metrics: open generated PNG frames with the image reader and
  describe what they show before writing numbers about them.
- Commits: in the worktree only, never push. Every message ends with
  ```
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01YEzhU2YV6QG66txmeSSB8Y
  ```

## Stage bookkeeping (every agent)

- State file `/scratch/wg2381/claude_jobs/height/state/<STAGE>.json`:
  `{"stage": "A1", "passed": ["task1", "task2"], "failed": {"task7": "reason"}, "updated": "<ISO time>"}`.
  Read it first; skip tasks already in `passed` after confirming their commit exists
  (`git log --oneline | grep "<task tag>"`). Update it after every task.
- Job IDs you submit: append one per line to
  `/scratch/wg2381/claude_jobs/height/jobids/<STAGE>.txt` as `<jobid> <purpose>`.
- Worklog `docs/worklog/height_bands.md`: append a dated entry for every problem hit
  and how it was resolved, as it happens.
- Commit messages start with the task tag, e.g. `[height T3] band-shadow projection`.

## File map

| File | Responsibility | Task |
|---|---|---|
| `gmc/src/gmc/height/__init__.py` | package docstring only | 1 |
| `gmc/src/gmc/height/ply3d.py` | `GaussianScene3D`; PLY decode; subset/crop; floor fit; gravity rotation | 1 |
| `gmc/src/gmc/height/band_shadow.py` | exclusion, shadow support function, certified outer ellipse | 2 |
| `gmc/src/gmc/height/prism.py` | `PrismRobot`, robot table, absolute band | 3 |
| `gmc/src/gmc/height/project.py` | 3D scene + prism + window → `SceneModel2D` + stats | 3 |
| `gmc/src/gmc/height/pathio.py` | `PoseCurve` ↔ path.json dict, polyline sampling | 4 |
| `gmc/src/gmc/height/replay3d.py` | independent 3D separating-direction replay | 5 |
| `gmc/src/gmc/height/synth3d.py` | synthetic 3D table scene (open/closed) | 6 |
| `gmc/src/gmc/height/run.py` | compile → query → verify on a `SceneModel2D`, result dict | 7 |
| `gmc/src/gmc/height/viz.py` | top-down/side animation, comparison video, 3D PLY | 8 |
| `gmc/experiments/height_toy.py` | T1 and T2 gates, JSON + videos | 9 |
| `gmc/experiments/showcase_scene.py` | G0 copy, checks, band maps, case selection | 10 |
| `gmc/experiments/showcase_run.py` | G1 per-robot job body, G2 replay | 11 |
| `gmc/experiments/height_viz.py` | G3 videos, PLY, figures from saved results | 12 |
| `gmc/hpc/height_toy.sbatch`, `height_showcase_robot.sbatch` | batch wrappers | 9, 11 |
| `gmc/tests/unit/test_height_*.py`, `gmc/tests/integration/test_height_table.py` | tests | 1–7, 9 |
| `/scratch/wg2381/claude_jobs/height/*` | agent prompts, wrappers, state | 13 |

---

## Stage A1 — core package and toy gates (Tasks 1–9)

### Task 1: `ply3d` — 3D splat scene, PLY decode, floor fit

**Files:**
- Create: `gmc/src/gmc/height/__init__.py`, `gmc/src/gmc/height/ply3d.py`
- Test: `gmc/tests/unit/test_height_ply3d.py`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `GaussianScene3D(means: (N,3) f64, covs: (N,3,3) f64, opacity: (N,) f64, ids: (N,) i64, name: str)`
    with `__len__`, `subset(mask_or_index) -> GaussianScene3D`, `aabb(level) -> (lo (N,3), hi (N,3))`.
  - `covariances_from_scale_rot(log_scale (N,3), quat_wxyz (N,4)) -> (N,3,3)`
  - `load_3dgs_ply(path, name=None) -> tuple[GaussianScene3D, dict]` (stats: `rows`, `nonfinite_dropped`)
  - `fit_floor(scene, *, min_opacity=0.5, bin_width=0.02, band=0.15, iters=400, tol=0.02, seed=0) -> dict`
    keys `z_floor, normal, centroid, tilt_deg, n_inliers, peak_z`
  - `gravity_rotation(normal) -> (3,3)`; `rotate_scene(scene, R, origin) -> GaussianScene3D`
  - `crop_box(scene, lo (3,), hi (3,)) -> tuple[GaussianScene3D, int]` (kept scene, dropped count)

- [ ] **Step 1: Write the failing tests**

```python
# gmc/tests/unit/test_height_ply3d.py
import numpy as np
import pytest

from gmc.height.ply3d import (GaussianScene3D, covariances_from_scale_rot,
                              crop_box, fit_floor, gravity_rotation,
                              load_3dgs_ply, rotate_scene)

PROPS = ["x", "y", "z", "nx", "ny", "nz", "f_dc_0", "f_dc_1", "f_dc_2",
         "opacity", "scale_0", "scale_1", "scale_2",
         "rot_0", "rot_1", "rot_2", "rot_3"]


def write_ply(path, rows):
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(rows)}\n"
              + "".join(f"property float {p}\n" for p in PROPS)
              + "end_header\n")
    arr = np.zeros(len(rows), dtype=[(p, "<f4") for p in PROPS])
    for i, r in enumerate(rows):
        for k, v in r.items():
            arr[i][k] = v
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        arr.tofile(f)


def test_load_decodes_opacity_scale_and_rotation(tmp_path):
    half = np.sqrt(0.5)
    rows = [
        dict(x=1, y=2, z=3, opacity=0.0, scale_0=np.log(0.1),
             scale_1=np.log(0.2), scale_2=np.log(0.3), rot_0=1.0),
        # 90 deg about z: local x axis -> world y axis
        dict(x=0, y=0, z=0, opacity=2.0, scale_0=np.log(0.4),
             scale_1=np.log(0.1), scale_2=np.log(0.1), rot_0=half, rot_3=half),
    ]
    p = tmp_path / "t.ply"
    write_ply(p, rows)
    scene, stats = load_3dgs_ply(p)
    assert stats["rows"] == 2 and stats["nonfinite_dropped"] == 0
    np.testing.assert_allclose(scene.means[0], [1, 2, 3], atol=1e-6)
    np.testing.assert_allclose(scene.opacity, [0.5, 1 / (1 + np.exp(-2.0))],
                               rtol=1e-6)
    np.testing.assert_allclose(scene.covs[0], np.diag([0.01, 0.04, 0.09]),
                               rtol=1e-5)
    np.testing.assert_allclose(scene.covs[1], np.diag([0.01, 0.16, 0.01]),
                               rtol=1e-5, atol=1e-7)
    assert scene.ids.tolist() == [0, 1]


def test_load_rejects_missing_field(tmp_path):
    p = tmp_path / "bad.ply"
    p.write_bytes(b"ply\nformat binary_little_endian 1.0\nelement vertex 0\n"
                  b"property float x\nend_header\n")
    with pytest.raises(ValueError, match="missing"):
        load_3dgs_ply(p)


def test_covariances_are_spd_and_normalise_quaternion():
    rng = np.random.default_rng(0)
    q = rng.normal(size=(50, 4)) * 3.0          # deliberately unnormalised
    s = rng.normal(size=(50, 3)) - 3.0
    C = covariances_from_scale_rot(s, q)
    assert np.all(np.linalg.eigvalsh(C) > 0)
    np.testing.assert_allclose(np.linalg.det(C), np.exp(2 * s.sum(axis=1)),
                               rtol=1e-8)


def _floor_scene(tilt_deg=1.0, z0=-1.0, seed=1):
    rng = np.random.default_rng(seed)
    n = 5000
    xy = rng.uniform(-5, 5, size=(n, 2))
    z = z0 + np.tan(np.deg2rad(tilt_deg)) * xy[:, 0] + rng.normal(0, 0.003, n)
    floor = np.column_stack([xy, z])
    wall = np.column_stack([np.full(1500, 5.0), rng.uniform(-5, 5, 1500),
                            rng.uniform(z0, z0 + 3, 1500)])
    floaters = np.column_stack([rng.uniform(-5, 5, (40, 2)),
                                rng.uniform(-12, -8, 40)])
    means = np.vstack([floor, wall, floaters])
    m = len(means)
    covs = np.tile(np.eye(3) * 1e-4, (m, 1, 1))
    return GaussianScene3D(means, covs, np.full(m, 0.9), np.arange(m), "floor")


def test_fit_floor_ignores_sparse_floaters_and_measures_tilt():
    info = fit_floor(_floor_scene())
    assert abs(info["z_floor"] - (-1.0)) < 0.01
    assert abs(info["tilt_deg"] - 1.0) < 0.2
    assert info["n_inliers"] > 4000


def test_gravity_rotation_levels_the_floor():
    scene = _floor_scene(tilt_deg=3.0)
    info = fit_floor(scene)
    R = gravity_rotation(np.asarray(info["normal"]))
    np.testing.assert_allclose(R @ np.asarray(info["normal"]), [0, 0, 1],
                               atol=1e-12)
    level = rotate_scene(scene, R, np.asarray(info["centroid"]))
    assert fit_floor(level)["tilt_deg"] < 0.2
    np.testing.assert_allclose(np.linalg.det(level.covs),
                               np.linalg.det(scene.covs), rtol=1e-9)


def test_crop_box_and_subset_keep_provenance():
    scene = _floor_scene()
    kept, dropped = crop_box(scene, np.array([-6, -6, -2]),
                             np.array([6, 6, 4]))
    assert dropped == 40 and len(kept) == len(scene) - 40
    assert kept.ids.max() < len(scene) - 40
    lo, hi = kept.aabb(2.0)
    np.testing.assert_allclose(hi - lo, 4 * np.sqrt(1e-4), rtol=1e-9)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_ply3d.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'gmc.height'`.

- [ ] **Step 3: Write the implementation**

```python
# gmc/src/gmc/height/__init__.py
"""Height-band projection of 3D Gaussian splat scenes onto per-robot 2D maps.

See docs/superpowers/specs/2026-09-12-height-band-robots-design.md.
"""
```

```python
# gmc/src/gmc/height/ply3d.py
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
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_ply3d.py`
Expected: `6 passed`.

- [ ] **Step 5: Commit**

```bash
git add gmc/src/gmc/height/__init__.py gmc/src/gmc/height/ply3d.py gmc/tests/unit/test_height_ply3d.py
git commit -m "[height T1] 3D splat scene, PLY decoding and floor fit"   # plus the trailer
```

### Task 2: `band_shadow` — exact exclusion and certified outer ellipse

**Files:**
- Create: `gmc/src/gmc/height/band_shadow.py`
- Test: `gmc/tests/unit/test_height_band_shadow.py`

**Interfaces:**
- Consumes: nothing (pure numpy).
- Produces:
  - `band_overlap_mask(means_z (N,), s_zz (N,), z_lo_abs, z_hi_abs, level) -> bool (N,)` — touching counts as overlap.
  - `shadow_support(mean3 (3,), cov3 (3,3), z_lo_abs, z_hi_abs, level, U (K,2)) -> (K,)` — exact support of the band-clipped xy-shadow.
  - `outer_shadow_ellipses(means (M,3), covs (M,3,3), z_lo_abs, z_hi_abs, level) -> dict` with
    `centre (M,2)`, `Q (M,2,2)` (support `uᵀcentre + √(uᵀQu)`), `candidate (M,) int` (1 = full projection, 2 = segment ⊕ ellipse), `floored (M,) bool`.
    Rows must already overlap the band.
  - Constants `REL_INFLATE = 1e-9`, `COV_EIG_FLOOR = 1e-9`.

Math (spec §3.2): blocks `A = Σ_xy`, `c = Σ_{xy,z}`, `s = Σ_zz`, `S = A − c cᵀ/s`;
`t = (h − μ_z)/√s`; cross-section centre `m(t) = μ_xy + c t/√s`, shape `S`,
squared radius `ρ² − t²`; `[t_a, t_b]` = band in t-units clipped to `[−ρ, ρ]`.

- [ ] **Step 1: Write the failing tests**

```python
# gmc/tests/unit/test_height_band_shadow.py
import numpy as np

from gmc.height.band_shadow import (COV_EIG_FLOOR, band_overlap_mask,
                                    outer_shadow_ellipses, shadow_support)

RHO = 2.0


def unit_dirs(k):
    a = 2 * np.pi * np.arange(k) / k
    return np.column_stack([np.cos(a), np.sin(a)])


def random_splats(n, seed):
    rng = np.random.default_rng(seed)
    q, _ = np.linalg.qr(rng.normal(size=(n, 3, 3)))
    sig = np.exp(rng.uniform(-9.0, 0.0, size=(n, 3)))    # axis ratio up to ~8e3
    covs = (q * sig[:, None, :] ** 2) @ q.transpose(0, 2, 1)
    covs = 0.5 * (covs + covs.transpose(0, 2, 1))
    means = rng.uniform(-3, 3, size=(n, 3))
    half = RHO * np.sqrt(covs[:, 2, 2])
    zc = means[:, 2] + rng.uniform(-1, 1, n) * half
    hw = rng.uniform(0.0, 1.5, n) * half
    return means, covs, zc - hw, zc + hw


def test_overlap_is_exact_and_touching_counts():
    mz, szz = np.array([1.0]), np.array([0.01])          # extent [0.8, 1.2]
    assert band_overlap_mask(mz, szz, 1.2, 1.5, RHO)[0]
    assert not band_overlap_mask(mz, szz, np.nextafter(1.2, 2), 1.5, RHO)[0]
    assert band_overlap_mask(mz, szz, 0.0, 0.8, RHO)[0]
    assert not band_overlap_mask(mz, szz, 0.0, 0.79, RHO)[0]


def test_unclipped_band_is_the_full_projection():
    means, covs, _, _ = random_splats(50, 1)
    U = unit_dirs(360)
    for m, C in zip(means, covs):
        h = shadow_support(m, C, -1e9, 1e9, RHO, U)
        full = U @ m[:2] + RHO * np.sqrt(np.einsum("ki,ij,kj->k", U, C[:2, :2], U))
        np.testing.assert_allclose(h, full, rtol=1e-9, atol=1e-11)


def test_outer_ellipse_dominates_shadow_support():
    means, covs, lo, hi = random_splats(300, 2)
    U = unit_dirs(720)
    for i in range(300):
        out = outer_shadow_ellipses(means[i:i + 1], covs[i:i + 1], lo[i], hi[i], RHO)
        c, Q = out["centre"][0], out["Q"][0]
        h_e = U @ c + np.sqrt(np.einsum("ki,ij,kj->k", U, Q, U))
        h_s = shadow_support(means[i], covs[i], lo[i], hi[i], RHO, U)
        assert np.all(h_e - h_s >= -1e-10 * (1 + np.abs(h_s))), i


def test_points_inside_ellipsoid_and_band_project_inside():
    rng = np.random.default_rng(3)
    means, covs, lo, hi = random_splats(60, 4)
    for i in range(60):
        out = outer_shadow_ellipses(means[i:i + 1], covs[i:i + 1], lo[i], hi[i], RHO)
        c, Q = out["centre"][0], out["Q"][0]
        L = np.linalg.cholesky(covs[i])
        w = rng.normal(size=(20000, 3))
        w *= (RHO * rng.uniform(0, 1, (20000, 1)) ** (1 / 3)
              / np.linalg.norm(w, axis=1, keepdims=True))
        p = means[i] + w @ L.T
        p = p[(p[:, 2] >= lo[i]) & (p[:, 2] <= hi[i])]
        if len(p) == 0:
            continue
        d = p[:, :2] - c
        q = np.einsum("ni,ij,nj->n", d, np.linalg.inv(Q), d)
        assert q.max() <= 1 + 1e-9, (i, q.max())


def test_clipping_a_tilted_needle_shrinks_the_bound():
    t = np.deg2rad(45)
    R = np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])
    C = R @ np.diag([0.01 ** 2, 0.01 ** 2, 0.5 ** 2]) @ R.T
    m = np.array([[0.0, 0.0, 1.0]])
    out = outer_shadow_ellipses(m, C[None], 1.0, 1.05, RHO)
    assert out["candidate"][0] == 2
    full_area = np.sqrt(np.linalg.det(RHO ** 2 * C[:2, :2]))
    assert np.sqrt(np.linalg.det(out["Q"][0])) < 0.5 * full_area


def test_degenerate_needle_is_floored_not_dropped():
    C = np.diag([1e-14, 1e-14, 0.04])
    out = outer_shadow_ellipses(np.zeros((1, 3)), C[None], -0.1, 0.1, RHO)
    assert out["floored"][0]
    assert np.linalg.eigvalsh(out["Q"][0] / RHO ** 2).min() >= COV_EIG_FLOOR * (1 - 1e-12)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_band_shadow.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'gmc.height.band_shadow'`.

- [ ] **Step 3: Write the implementation**

```python
# gmc/src/gmc/height/band_shadow.py
"""Band-clipped xy-shadow of a 3D Gaussian iso-ellipsoid (spec §3.2).

Soundness is analytic, not numeric: candidate 1 is the projection of the whole
ellipsoid (a superset of any clipped part); candidate 2 contains the shadow
because every cross-section centre lies on the segment [m(t_a), m(t_b)] and
every cross-section radius is at most r_max, and
(sqrt(a) + sqrt(b))^2 <= (1 + 1/k) a + (1 + k) b for every k > 0.
Inflation and the eigenvalue floor only grow the result.
"""
import numpy as np

REL_INFLATE = 1e-9
COV_EIG_FLOOR = 1e-9


def band_overlap_mask(means_z, s_zz, z_lo_abs, z_hi_abs, level):
    half = float(level) * np.sqrt(np.asarray(s_zz, dtype=np.float64))
    mz = np.asarray(means_z, dtype=np.float64)
    return (mz - half <= float(z_hi_abs)) & (mz + half >= float(z_lo_abs))


def _t_range(mz, szz, z_lo_abs, z_hi_abs, level):
    sq = np.sqrt(szz)
    ta = np.clip((z_lo_abs - mz) / sq, -level, level)
    tb = np.clip((z_hi_abs - mz) / sq, -level, level)
    return ta, tb, sq


def shadow_support(mean3, cov3, z_lo_abs, z_hi_abs, level, U):
    m = np.asarray(mean3, dtype=np.float64)
    C = np.asarray(cov3, dtype=np.float64)
    U = np.asarray(U, dtype=np.float64)
    ta, tb, sq = _t_range(m[2], C[2, 2], float(z_lo_abs), float(z_hi_abs),
                          float(level))
    if ta > tb:
        raise ValueError("band does not overlap the splat")
    c = C[:2, 2]
    S = C[:2, :2] - np.outer(c, c) / C[2, 2]
    beta = (U @ c) / sq
    gamma = np.sqrt(np.maximum(np.einsum("ki,ij,kj->k", U, S, U), 0.0))
    denom = np.sqrt(beta ** 2 + gamma ** 2)
    tstar = np.where(denom > 0, level * beta / np.where(denom > 0, denom, 1), 0.0)
    t = np.clip(tstar, ta, tb)
    return U @ m[:2] + beta * t + gamma * np.sqrt(np.maximum(level ** 2 - t ** 2, 0.0))


def outer_shadow_ellipses(means, covs, z_lo_abs, z_hi_abs, level):
    means = np.asarray(means, dtype=np.float64)
    covs = np.asarray(covs, dtype=np.float64)
    level = float(level)
    mz, szz = means[:, 2], covs[:, 2, 2]
    ta, tb, sq = _t_range(mz, szz, float(z_lo_abs), float(z_hi_abs), level)
    if np.any(ta > tb):
        raise ValueError("every row must overlap the band")
    A = covs[:, :2, :2]
    c = covs[:, :2, 2]
    S = A - np.einsum("ni,nj->nij", c, c) / szz[:, None, None]
    S = 0.5 * (S + S.transpose(0, 2, 1))

    Q1 = level ** 2 * A
    ctr1 = means[:, :2]

    tmin = np.where((ta <= 0) & (tb >= 0), 0.0,
                    np.minimum(np.abs(ta), np.abs(tb)))
    R = (level ** 2 - tmin ** 2)[:, None, None] * S
    d = c * ((tb - ta) / (2 * sq))[:, None]
    D = np.einsum("ni,nj->nij", d, d)
    trD = np.einsum("ni,ni->n", d, d)
    trR = np.einsum("nii->n", R)
    has_d = (trD > 0) & (trR > 0)
    k = np.sqrt(np.where(has_d, trD / np.where(trR > 0, trR, 1.0), 1.0))
    Q2 = np.where(has_d[:, None, None],
                  (1 + 1 / k)[:, None, None] * D + (1 + k)[:, None, None] * R,
                  R + D)
    ctr2 = means[:, :2] + c * ((0.5 * (ta + tb)) / sq)[:, None]

    use2 = np.linalg.det(Q2) < np.linalg.det(Q1)
    Q = np.where(use2[:, None, None], Q2, Q1) * (1.0 + REL_INFLATE)
    centre = np.where(use2[:, None], ctr2, ctr1)

    cov = 0.5 * (Q + Q.transpose(0, 2, 1)) / level ** 2
    w, v = np.linalg.eigh(cov)
    floored = w.min(axis=1) < COV_EIG_FLOOR
    w = np.maximum(w, COV_EIG_FLOOR)
    cov = (v * w[:, None, :]) @ v.transpose(0, 2, 1)
    Q = level ** 2 * 0.5 * (cov + cov.transpose(0, 2, 1))
    return {"centre": centre, "Q": Q,
            "candidate": np.where(use2, 2, 1), "floored": floored}
```

Note for the implementer: eigen-reconstruction `v diag(max(w, floor)) vᵀ` can
lose a few ulps relative to `Q` on the unfloored rows. That is covered by
`REL_INFLATE`, which is applied first and is ~10⁷ ulps; do not remove it.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_band_shadow.py`
Expected: `6 passed`.

- [ ] **Step 5: Commit**

```bash
git add gmc/src/gmc/height/band_shadow.py gmc/tests/unit/test_height_band_shadow.py
git commit -m "[height T2] band-clipped shadow support and certified outer ellipse"   # plus the trailer
```

### Task 3: `prism` and `project` — per-robot 2D obstacle maps

**Files:**
- Create: `gmc/src/gmc/height/prism.py`, `gmc/src/gmc/height/project.py`
- Test: `gmc/tests/unit/test_height_project.py`

**Interfaces:**
- Consumes: `GaussianScene3D` (Task 1); `band_overlap_mask`, `outer_shadow_ellipses` (Task 2);
  `gmc.io.robot_io.ellipse_robot(a, b, level=1.0, name=...)`; `gmc.types.GaussianSupport2D`, `SceneModel2D`.
- Produces:
  - `PrismRobot(footprint: RobotModel2D, z_lo: float | None, z_hi: float | None, name: str)`
    with `is_2d: bool`, `band_abs(z_floor) -> (float, float)`, `max_radius() -> float`.
  - `robot_table(z_c: float = 1.20) -> dict[str, PrismRobot]` with keys
    `ellipse_toy, sweeper, quadruped, cylinder, uav`.
  - `project_scene(scene3d, robot, window, *, z_floor, tau=0.3, level=2.0, name=None) -> tuple[SceneModel2D, dict]`
    where `window = (xmin, ymin, xmax, ymax)`; each support's `primitive_id` is the splat's `ids` value.
    Stats keys: `n_input, dropped_opacity, excluded_band, outside_window, deduplicated, kept, floored_3d, floored_2d, candidate2, band_abs, tau, level, window, dilate_radius`.
    Bit-identical shadows (same centre and Q) are kept once, under the first splat's id; the union is unchanged.

- [ ] **Step 1: Write the failing tests**

```python
# gmc/tests/unit/test_height_project.py
import numpy as np
import pytest

from gmc.height.ply3d import GaussianScene3D
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.io.gs_io import validate_models

from ..conftest import make_cfg

Z_FLOOR = -1.0


def mini_scene():
    """Floor at z=-1: leg needle, tabletop slab, faint splat, far splat."""
    means = np.array([
        [0.0, 0.0, Z_FLOOR + 0.36],    # 0 leg, extent 0.0..0.72 above floor
        [0.0, 0.0, Z_FLOOR + 0.74],    # 1 tabletop, extent 0.72..0.76
        [0.5, 0.0, Z_FLOOR + 0.05],    # 2 faint (opacity 0.1)
        [9.0, 9.0, Z_FLOOR + 0.05],    # 3 far outside the window
    ])
    covs = np.array([
        np.diag([0.015 ** 2, 0.015 ** 2, 0.18 ** 2]),
        np.diag([0.15 ** 2, 0.15 ** 2, 0.01 ** 2]),
        np.eye(3) * 0.01 ** 2,
        np.eye(3) * 0.01 ** 2,
    ])
    return GaussianScene3D(means, covs, np.array([0.9, 0.9, 0.1, 0.9]),
                           np.array([10, 11, 12, 13]), "mini")


WINDOW = (-2.0, -2.0, 2.0, 2.0)


def ids(scene2d):
    return {s.primitive_id for s in scene2d.supports}


def test_robot_table_bands():
    t = robot_table(z_c=1.4)
    assert t["ellipse_toy"].is_2d
    assert t["sweeper"].band_abs(Z_FLOOR) == pytest.approx((-0.98, -0.90))
    assert t["uav"].band_abs(0.0) == pytest.approx((1.3, 1.5))
    assert t["cylinder"].max_radius() == pytest.approx(0.30)
    with pytest.raises(ValueError):
        t["ellipse_toy"].band_abs(0.0)


def test_sweeper_sees_leg_not_tabletop():
    s2, st = project_scene(mini_scene(), robot_table()["sweeper"], WINDOW,
                           z_floor=Z_FLOOR)
    assert ids(s2) == {10}
    assert st["dropped_opacity"] == 1 and st["excluded_band"] == 1
    assert st["outside_window"] == 1 and st["kept"] == 1


def test_cylinder_sees_leg_and_tabletop():
    s2, _ = project_scene(mini_scene(), robot_table()["cylinder"], WINDOW,
                          z_floor=Z_FLOOR)
    assert ids(s2) == {10, 11}


def test_uav_above_table_sees_nothing():
    s2, st = project_scene(mini_scene(), robot_table(z_c=1.20)["uav"], WINDOW,
                           z_floor=Z_FLOOR)
    assert ids(s2) == set() and st["kept"] == 0


def test_lower_tau_only_adds_obstacles():
    hi, _ = project_scene(mini_scene(), robot_table()["sweeper"], WINDOW,
                          z_floor=Z_FLOOR, tau=0.3)
    lo, _ = project_scene(mini_scene(), robot_table()["sweeper"], WINDOW,
                          z_floor=Z_FLOOR, tau=0.05)
    assert ids(hi) < ids(lo) and 12 in ids(lo)


def test_stacked_identical_shadows_are_kept_once():
    base = mini_scene()
    means = np.vstack([base.means[:1], base.means[:1] + [0, 0, 0.1]])
    covs = np.stack([base.covs[0], base.covs[0]])
    s3 = GaussianScene3D(means, covs, np.array([0.9, 0.9]), np.array([5, 6]), "stack")
    s2, st = project_scene(s3, robot_table()["cylinder"], WINDOW, z_floor=Z_FLOOR)
    assert ids(s2) == {5} and st["deduplicated"] == 1


def test_projected_maps_pass_gmc_input_validation():
    cfg = make_cfg()
    for key in ("sweeper", "cylinder", "quadruped"):
        robot = robot_table()[key]
        s2, _ = project_scene(mini_scene(), robot, WINDOW, z_floor=Z_FLOOR)
        validate_models(s2, robot.footprint, cfg)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_project.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'gmc.height.prism'`.

- [ ] **Step 3: Write the implementation**

```python
# gmc/src/gmc/height/prism.py
"""Robots as vertical prisms: a 2D footprint over a height band (spec §3.3)."""
from dataclasses import dataclass

from ..io.robot_io import ellipse_robot
from ..types import RobotModel2D


@dataclass(frozen=True)
class PrismRobot:
    footprint: RobotModel2D
    z_lo: float | None
    z_hi: float | None
    name: str

    @property
    def is_2d(self) -> bool:
        return self.z_lo is None

    def band_abs(self, z_floor: float) -> tuple[float, float]:
        if self.is_2d:
            raise ValueError(f"{self.name} is a 2D-only robot with no band")
        return (float(z_floor) + self.z_lo, float(z_floor) + self.z_hi)

    def max_radius(self) -> float:
        return float(self.footprint.max_rotational_radius())


def ellipse_prism(a, b, z_lo, z_hi, name) -> PrismRobot:
    return PrismRobot(ellipse_robot(a, b, name=name), z_lo, z_hi, name)


def robot_table(z_c: float = 1.20) -> dict[str, PrismRobot]:
    return {
        "ellipse_toy": ellipse_prism(0.50, 0.20, None, None, "ellipse_toy"),
        "sweeper": ellipse_prism(0.175, 0.175, 0.02, 0.10, "sweeper"),
        "quadruped": ellipse_prism(0.35, 0.16, 0.02, 0.45, "quadruped"),
        "cylinder": ellipse_prism(0.30, 0.30, 0.02, 1.75, "cylinder"),
        "uav": ellipse_prism(0.25, 0.25, z_c - 0.10, z_c + 0.10, "uav"),
    }
```

```python
# gmc/src/gmc/height/project.py
"""3D splat scene -> the 2D obstacle map one prism robot sees (spec §3.1-§3.2).

Every step either removes obstacle mass and is counted (opacity, band, window)
or only grows obstacles (3D/2D eigenvalue floors, outer ellipses).
"""
import numpy as np
from shapely.geometry import box as shapely_box

from ..types import GaussianSupport2D, SceneModel2D
from .band_shadow import band_overlap_mask, outer_shadow_ellipses

EIG3_FLOOR = 1e-12


def project_scene(scene3d, robot, window, *, z_floor, tau=0.3, level=2.0,
                  name=None):
    xmin, ymin, xmax, ymax = map(float, window)
    z_lo, z_hi = robot.band_abs(z_floor)
    r = robot.max_radius()
    stats = {"n_input": len(scene3d), "tau": float(tau), "level": float(level),
             "window": [xmin, ymin, xmax, ymax], "band_abs": [z_lo, z_hi],
             "dilate_radius": r}

    opaque = scene3d.opacity > float(tau)
    stats["dropped_opacity"] = int((~opaque).sum())
    sub = scene3d.subset(opaque)

    covs = sub.covs.copy()
    w, v = np.linalg.eigh(covs)
    low = w.min(axis=1) < EIG3_FLOOR
    stats["floored_3d"] = int(low.sum())
    if low.any():
        wl = np.maximum(w[low], EIG3_FLOOR)
        covs[low] = (v[low] * wl[:, None, :]) @ v[low].transpose(0, 2, 1)

    inband = band_overlap_mask(sub.means[:, 2], covs[:, 2, 2], z_lo, z_hi, level)
    stats["excluded_band"] = int((~inband).sum())
    means, covs, pid = sub.means[inband], covs[inband], sub.ids[inband]

    # Cheap superset test: the whole ellipsoid's xy box.
    half = level * np.sqrt(np.stack([covs[:, 0, 0], covs[:, 1, 1]], axis=1))
    near = ((means[:, 0] - half[:, 0] <= xmax + r)
            & (means[:, 0] + half[:, 0] >= xmin - r)
            & (means[:, 1] - half[:, 1] <= ymax + r)
            & (means[:, 1] + half[:, 1] >= ymin - r))
    out_far = int((~near).sum())
    means, covs, pid = means[near], covs[near], pid[near]

    supports = []
    floored_2d = cand2 = out_box = dedup = 0
    if len(means):
        oe = outer_shadow_ellipses(means, covs, z_lo, z_hi, level)
        c, Q = oe["centre"], oe["Q"]
        eh = np.sqrt(np.stack([Q[:, 0, 0], Q[:, 1, 1]], axis=1))
        inside = ((c[:, 0] - eh[:, 0] <= xmax + r) & (c[:, 0] + eh[:, 0] >= xmin - r)
                  & (c[:, 1] - eh[:, 1] <= ymax + r) & (c[:, 1] + eh[:, 1] >= ymin - r))
        out_box = int((~inside).sum())
        rows = np.ascontiguousarray(np.concatenate([c, Q.reshape(-1, 4)], axis=1))
        first = np.zeros(len(rows), dtype=bool)
        cand_idx = np.flatnonzero(inside)
        _, keep_pos = np.unique(rows[cand_idx].view(np.dtype((np.void, 48))),
                                return_index=True)
        first[cand_idx[np.sort(keep_pos)]] = True
        dedup = int(inside.sum() - first.sum())
        inside = first
        floored_2d = int(oe["floored"][inside].sum())
        cand2 = int((oe["candidate"][inside] == 2).sum())
        for ci, Qi, idi in zip(c[inside], Q[inside], pid[inside]):
            supports.append(GaussianSupport2D(
                mean=ci, covariance=Qi / level ** 2, level=level,
                primitive_id=int(idi)))
    stats.update(outside_window=out_far + out_box, deduplicated=dedup,
                 kept=len(supports),
                 floored_2d=floored_2d, candidate2=cand2)
    scene2d = SceneModel2D(supports=tuple(supports),
                           workspace=shapely_box(xmin, ymin, xmax, ymax),
                           name=name or f"{scene3d.name}__{robot.name}")
    return scene2d, stats
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_project.py tests/unit/test_height_band_shadow.py`
Expected: `13 passed`.

- [ ] **Step 5: Commit**

```bash
git add gmc/src/gmc/height/prism.py gmc/src/gmc/height/project.py gmc/tests/unit/test_height_project.py
git commit -m "[height T3] prism robots and per-band scene projection"   # plus the trailer
```

### Task 4: `pathio` — path.json round trip and swept sampling

**Files:**
- Create: `gmc/src/gmc/height/pathio.py`
- Test: `gmc/tests/unit/test_height_pathio.py`

**Interfaces:**
- Consumes: `gmc.mobility.witness.PoseCurve, PoseSegment, SegmentKind`; `gmc.types.Pose2`.
- Produces:
  - `curve_to_dict(curve, query_id="") -> dict` — same schema as `reporting/artifacts.py` path.json (`schema_version: 2`).
  - `curve_from_dict(d) -> PoseCurve` — rejects `LOCAL_STEERING` with `ValueError`.
  - `save_path_json(curve, path, query_id="")`, `load_path_json(path) -> PoseCurve`
  - `sample_curve(curve, max_step, radius) -> np.ndarray (K,3)` rows `(x, y, theta)`; between consecutive rows no
    point of a footprint of rotational radius `radius` moves more than `max_step`.
  - `polyline_xy(curve) -> np.ndarray (M,2)`; `path_crosses(curve, polygon) -> bool`;
    `crossing_thetas(curve, x0) -> list[float]` (theta of every TRANSLATION segment that crosses `x = x0`).

- [ ] **Step 1: Write the failing tests**

```python
# gmc/tests/unit/test_height_pathio.py
import numpy as np
import pytest
from shapely.geometry import box

from gmc.height.pathio import (crossing_thetas, curve_from_dict, curve_to_dict,
                               load_path_json, path_crosses, polyline_xy,
                               sample_curve, save_path_json)

DOOR_PATH = {"schema_version": 2, "query_id": "q_1788388783567711000",
             "segments": [
                 {"kind": "ROTATION", "q0": [-2.0, 0.0, 0.3],
                  "q1": [-2.0, 0.0, 0.19634954084936207],
                  "control_points": [], "certificate_ids": ["a"]},
                 {"kind": "TRANSLATION", "q0": [-2.0, 0.0, 0.19634954084936207],
                  "q1": [2.0, 0.0, 0.19634954084936207],
                  "control_points": [], "certificate_ids": ["b"]},
                 {"kind": "ROTATION", "q0": [2.0, 0.0, 0.19634954084936207],
                  "q1": [2.0, 0.0, 0.3], "control_points": [],
                  "certificate_ids": ["c"]}]}


def test_round_trip_matches_artifact_schema(tmp_path):
    curve = curve_from_dict(DOOR_PATH)
    assert curve_to_dict(curve, DOOR_PATH["query_id"]) == DOOR_PATH
    save_path_json(curve, tmp_path / "path.json", "q")
    again = load_path_json(tmp_path / "path.json")
    assert curve_to_dict(again, "q")["segments"] == DOOR_PATH["segments"]


def test_local_steering_is_rejected():
    bad = {"schema_version": 2, "segments": [dict(DOOR_PATH["segments"][0],
                                                  kind="LOCAL_STEERING")]}
    with pytest.raises(ValueError, match="LOCAL_STEERING"):
        curve_from_dict(bad)


def test_sampling_bounds_footprint_motion():
    curve = curve_from_dict(DOOR_PATH)
    poses = sample_curve(curve, max_step=0.01, radius=0.5)
    np.testing.assert_allclose(poses[0], [-2.0, 0.0, 0.3])
    np.testing.assert_allclose(poses[-1], [2.0, 0.0, 0.3])
    step = (np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
            + np.abs(np.diff(poses[:, 2])) * 0.5)
    assert step.max() <= 0.01 + 1e-12
    assert len(poses) >= 400


def test_polyline_crossing_and_thetas():
    curve = curve_from_dict(DOOR_PATH)
    assert polyline_xy(curve).shape == (4, 2)
    assert path_crosses(curve, box(-0.3, -0.6, 0.3, 0.6))
    assert not path_crosses(curve, box(-0.3, 0.5, 0.3, 0.6))
    assert crossing_thetas(curve, 0.0) == [pytest.approx(0.19634954084936207)]
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_pathio.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'gmc.height.pathio'`.

- [ ] **Step 3: Write the implementation**

```python
# gmc/src/gmc/height/pathio.py
"""PoseCurve <-> path.json, and conservative sampling of the swept motion."""
import json
import math
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, Point

from ..mobility.witness import PoseCurve, PoseSegment, SegmentKind
from ..types import Pose2


def _pose(v) -> Pose2:
    return Pose2(np.asarray(v[:2], dtype=float), float(v[2]))


def _vals(p: Pose2) -> list:
    return [*map(float, p.xy), float(p.theta)]


def curve_to_dict(curve, query_id="") -> dict:
    return {"schema_version": 2, "query_id": query_id, "segments": [{
        "kind": s.kind.name, "q0": _vals(s.q0), "q1": _vals(s.q1),
        "control_points": [_vals(p) for p in s.control_points],
        "certificate_ids": list(s.certificate_ids)} for s in curve.segments]}


def curve_from_dict(d) -> PoseCurve:
    segs = []
    for s in d["segments"]:
        kind = SegmentKind[s["kind"]]
        if kind is SegmentKind.LOCAL_STEERING:
            raise ValueError("LOCAL_STEERING segments are not supported by height replay")
        segs.append(PoseSegment(kind, _pose(s["q0"]), _pose(s["q1"]),
                                tuple(_pose(p) for p in s.get("control_points", [])),
                                tuple(s.get("certificate_ids", []))))
    return PoseCurve(tuple(segs))


def save_path_json(curve, path, query_id=""):
    Path(path).write_text(json.dumps(curve_to_dict(curve, query_id), indent=2))


def load_path_json(path) -> PoseCurve:
    return curve_from_dict(json.loads(Path(path).read_text()))


def sample_curve(curve, max_step, radius) -> np.ndarray:
    rows = []
    for s in curve.segments:
        a, b = np.array(_vals(s.q0)), np.array(_vals(s.q1))
        move = (float(np.linalg.norm(b[:2] - a[:2]))
                + abs(b[2] - a[2]) * max(float(radius), 0.0))
        n = max(1, math.ceil(move / float(max_step)))
        t = np.linspace(0.0, 1.0, n + 1)[:, None]
        pts = a + t * (b - a)
        rows.append(pts if not rows else pts[1:])
    return np.vstack(rows)


def polyline_xy(curve) -> np.ndarray:
    pts = [curve.segments[0].q0.xy] + [s.q1.xy for s in curve.segments]
    return np.asarray(pts, dtype=float)


def path_crosses(curve, polygon) -> bool:
    xy = polyline_xy(curve)
    geom = LineString(xy) if len(np.unique(xy, axis=0)) > 1 else Point(xy[0])
    return bool(geom.intersects(polygon))


def crossing_thetas(curve, x0) -> list:
    out = []
    for s in curve.segments:
        if s.kind is SegmentKind.TRANSLATION:
            xa, xb = float(s.q0.xy[0]), float(s.q1.xy[0])
            if min(xa, xb) <= x0 <= max(xa, xb) and xa != xb:
                out.append(float(s.q0.theta))
    return out
```

The step bound holds because each row is a linear interpolation: the distance
from any footprint point between two rows is at most `|Δxy| + |Δθ|·radius`,
which is `move / n ≤ max_step`.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_pathio.py`
Expected: `4 passed`.

- [ ] **Step 5: Commit**

```bash
git add gmc/src/gmc/height/pathio.py gmc/tests/unit/test_height_pathio.py
git commit -m "[height T4] path.json round trip and swept-motion sampling"   # plus the trailer
```

### Task 5: `replay3d` — independent 3D check with separating directions

**Files:**
- Create: `gmc/src/gmc/height/replay3d.py`
- Test: `gmc/tests/unit/test_height_replay3d.py`

**Interfaces:**
- Consumes: `GaussianScene3D` (Task 1), `PrismRobot` (Task 3), `sample_curve` (Task 4).
  Must NOT import `band_shadow`, `project`, or anything under `gmc.mobility`, `gmc.geometry`,
  `gmc.orientation`, `gmc.verification`, `gmc.spatial` (spec §3.4 independence).
- Produces:
  - `sphere_dirs(n) -> (n,3)` unit vectors (Fibonacci sphere).
  - `pair_gaps(V, mu (M,3), Sig (M,3,3), level, centre (2,), F (2,2), z_lo, z_hi) -> (M,)` —
    best gap over directions `V`; a positive value is a proof of disjointness and a lower bound on distance.
  - `refine_gap(mu (3,), Sig (3,3), level, centre, F, z_lo, z_hi, v0 (3,)) -> float`
  - `replay_curve(scene3d, robot, curve, *, z_floor, tau=0.3, level=2.0, delta=0.01, n_coarse=256, n_fine=2048, margin=0.05, big_extent=0.5) -> dict`
    keys `passed, n_samples, n_pairs_checked, n_refined, min_clearance_lb, worst, collisions, params`.

Certificate (independent of the band-shadow derivation): for unit `v`,
`h_E(v) = vᵀμ + ρ√(vᵀΣv)` (ellipsoid) and
`h_P(v) = v_xyᵀc + √(v_xyᵀ F v_xy) + max(v_z z_lo, v_z z_hi)` (prism).
`gap(v) = −h_E(−v) − h_P(v) = vᵀμ − ρ√(vᵀΣv) − h_P(v)`. If `gap(v) > 0` the two sets
are separated by distance at least `gap(v)`. An optimizer that finds a poor `v`
can only cause a false collision, never a false pass.

- [ ] **Step 1: Write the failing tests**

```python
# gmc/tests/unit/test_height_replay3d.py
from pathlib import Path

import numpy as np

from gmc.height.pathio import curve_from_dict
from gmc.height.ply3d import GaussianScene3D
from gmc.height.prism import robot_table
from gmc.height.replay3d import pair_gaps, refine_gap, replay_curve, sphere_dirs

RHO = 2.0


def disc(r):
    return np.eye(2) * r * r


def test_gap_is_a_lower_bound_on_true_distance():
    mu = np.array([[1.0, 0.0, 0.5]])
    Sig = (np.eye(3) * 0.1 ** 2)[None]                    # radius 0.2 at level 2
    g = pair_gaps(sphere_dirs(2048), mu, Sig, RHO, np.zeros(2), disc(0.3), 0.0, 1.0)
    assert 0.49 <= g[0] <= 0.5 + 1e-12


def test_overlap_never_certifies_even_after_refinement():
    mu = np.array([0.35, 0.0, 0.5])
    Sig = np.eye(3) * 0.1 ** 2
    g = pair_gaps(sphere_dirs(2048), mu[None], Sig[None], RHO, np.zeros(2), disc(0.3), 0.0, 1.0)
    assert g[0] <= 0
    assert refine_gap(mu, Sig, RHO, np.zeros(2), disc(0.3), 0.0, 1.0,
                      np.array([1.0, 0.0, 0.0])) <= 0


def test_splat_above_band_is_separated_vertically():
    mu = np.array([[0.0, 0.0, 1.5]])
    Sig = (np.eye(3) * 0.1 ** 2)[None]
    g = pair_gaps(sphere_dirs(2048), mu, Sig, RHO, np.zeros(2), disc(0.3), 0.0, 1.0)
    assert 0.29 <= g[0] <= 0.3 + 1e-12


def pillar_scene(y_centre):
    zs = np.linspace(0.1, 1.9, 10)
    means = np.column_stack([np.zeros(10), np.full(10, y_centre), zs])
    covs = np.tile(np.eye(3) * 0.1 ** 2, (10, 1, 1))     # radius 0.2 blobs
    return GaussianScene3D(means, covs, np.full(10, 0.9), np.arange(10), "pillar")


LINE = {"schema_version": 2, "segments": [
    {"kind": "TRANSLATION", "q0": [-1.0, 0.0, 0.0], "q1": [1.0, 0.0, 0.0]}]}


def test_straight_path_through_pillar_fails():
    rep = replay_curve(pillar_scene(0.0), robot_table()["cylinder"],
                       curve_from_dict(LINE), z_floor=0.0)
    assert not rep["passed"] and rep["collisions"]


def test_clearance_below_delta_is_flagged_conservatively():
    # true clearance 0.005 < delta: the dilated footprint must still collide
    rep = replay_curve(pillar_scene(0.3 + 0.2 + 0.005), robot_table()["cylinder"],
                       curve_from_dict(LINE), z_floor=0.0, delta=0.01)
    assert not rep["passed"]


def test_clear_path_passes_with_bounded_clearance():
    rep = replay_curve(pillar_scene(0.3 + 0.2 + 0.05), robot_table()["cylinder"],
                       curve_from_dict(LINE), z_floor=0.0, delta=0.01)
    assert rep["passed"]
    assert 0.03 <= rep["min_clearance_lb"] <= 0.05 + 1e-9


def test_replay_imports_nothing_from_the_projection_or_core():
    src = Path(__import__("gmc.height.replay3d", fromlist=["x"]).__file__).read_text()
    for banned in ("from .band_shadow", "from .project", "from ..mobility",
                   "from ..geometry", "from ..orientation", "from ..verification",
                   "from ..spatial", "gmc.mobility", "gmc.geometry",
                   "gmc.orientation", "gmc.verification", "gmc.spatial"):
        assert banned not in src.split('"""', 2)[-1], banned
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_replay3d.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'gmc.height.replay3d'`.

- [ ] **Step 3: Write the implementation**

```python
# gmc/src/gmc/height/replay3d.py
"""Independent 3D replay of a returned path (spec §3.4).

Disjointness of a prism and a splat is certified only by exhibiting a unit
direction with a positive support gap; see the plan's Task 5 header.
"""
import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree

from .pathio import sample_curve


def sphere_dirs(n):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    th = np.pi * (1 + 5 ** 0.5) * i
    return np.column_stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi),
                            np.cos(phi)])


def _prism_support(V, centre, F, z_lo, z_hi):
    vxy = V[:, :2]
    return (vxy @ centre
            + np.sqrt(np.maximum(np.einsum("ki,ij,kj->k", vxy, F, vxy), 0.0))
            + np.maximum(V[:, 2] * z_lo, V[:, 2] * z_hi))


def pair_gaps(V, mu, Sig, level, centre, F, z_lo, z_hi):
    hp = _prism_support(V, np.asarray(centre, float), np.asarray(F, float),
                        float(z_lo), float(z_hi))
    quad = np.einsum("ki,mij,kj->mk", V, Sig, V)
    gaps = mu @ V.T - level * np.sqrt(np.maximum(quad, 0.0)) - hp[None, :]
    return gaps.max(axis=1)


def refine_gap(mu, Sig, level, centre, F, z_lo, z_hi, v0):
    def neg_gap(ang):
        v = np.array([np.cos(ang[0]) * np.sin(ang[1]),
                      np.sin(ang[0]) * np.sin(ang[1]), np.cos(ang[1])])
        return -pair_gaps(v[None], mu[None], Sig[None], level, centre, F,
                          z_lo, z_hi)[0]
    v0 = np.asarray(v0, float) / np.linalg.norm(v0)
    a0 = [np.arctan2(v0[1], v0[0]), np.arccos(np.clip(v0[2], -1, 1))]
    res = minimize(neg_gap, a0, method="Nelder-Mead",
                   options={"xatol": 1e-9, "fatol": 1e-12, "maxiter": 600})
    return float(max(-res.fun, -neg_gap(a0)))


def _footprints(robot, pose, delta):
    """World-frame (centre, F) per footprint support, dilated to cover delta."""
    x, y, th = map(float, pose)
    c, s = np.cos(th), np.sin(th)
    R = np.array([[c, -s], [s, c]])
    out = []
    for sup in robot.footprint.supports:
        shape = sup.level ** 2 * np.asarray(sup.covariance, float)
        b = float(np.sqrt(np.linalg.eigvalsh(shape)[0]))
        lam = 1.0 + float(delta) / b
        out.append((np.array([x, y]) + R @ np.asarray(sup.mean, float),
                    lam ** 2 * (R @ shape @ R.T)))
    return out


def _dilated_radius(robot, delta):
    r = 0.0
    for sup in robot.footprint.supports:
        w = np.linalg.eigvalsh(sup.level ** 2 * np.asarray(sup.covariance, float))
        lam = 1.0 + float(delta) / float(np.sqrt(w[0]))
        r = max(r, float(np.linalg.norm(sup.mean)) + lam * float(np.sqrt(w[-1])))
    return r


def replay_curve(scene3d, robot, curve, *, z_floor, tau=0.3, level=2.0,
                 delta=0.01, n_coarse=256, n_fine=2048, margin=0.05,
                 big_extent=0.5):
    if robot.is_2d:
        raise ValueError("replay_curve needs a prism robot with a height band")
    z_lo, z_hi = robot.band_abs(z_floor)
    sub = scene3d.subset(scene3d.opacity > float(tau))
    lo, hi = sub.aabb(level)
    ext = 0.5 * (hi - lo)[:, :2].max(axis=1)
    zmask = (lo[:, 2] <= z_hi + margin) & (hi[:, 2] >= z_lo - margin)
    big = np.flatnonzero(zmask & (ext > big_extent))
    small = np.flatnonzero(zmask & (ext <= big_extent))
    tree = cKDTree(sub.means[small, :2]) if len(small) else None
    Vc, Vf = sphere_dirs(n_coarse), sphere_dirs(n_fine)
    poses = sample_curve(curve, delta, robot.max_radius())
    reach = _dilated_radius(robot, delta) + margin + big_extent

    n_pairs = n_refined = 0
    min_lb, worst, collisions = margin, None, []
    for k, pose in enumerate(poses):
        near = big
        if tree is not None:
            idx = tree.query_ball_point(pose[:2], r=reach)
            near = np.concatenate([big, small[np.asarray(idx, dtype=int)]])
        for centre, F in _footprints(robot, pose, delta):
            ph = np.sqrt(np.diag(F))
            sel = near[(lo[near, 0] <= centre[0] + ph[0] + margin)
                       & (hi[near, 0] >= centre[0] - ph[0] - margin)
                       & (lo[near, 1] <= centre[1] + ph[1] + margin)
                       & (hi[near, 1] >= centre[1] - ph[1] - margin)
                       & (lo[near, 2] <= z_hi + margin)
                       & (hi[near, 2] >= z_lo - margin)]
            if not len(sel):
                continue
            n_pairs += len(sel)
            mu, Sig = sub.means[sel], sub.covs[sel]
            g = pair_gaps(Vc, mu, Sig, level, centre, F, z_lo, z_hi)
            low = g <= margin
            if low.any():
                g[low] = np.maximum(g[low], pair_gaps(Vf, mu[low], Sig[low], level,
                                                      centre, F, z_lo, z_hi))
            for j in np.flatnonzero(g <= 0):
                n_refined += 1
                v0 = np.append(mu[j, :2] - centre, 0.0)
                if not np.any(v0):
                    v0 = np.array([1.0, 0.0, 0.0])
                g[j] = max(g[j], refine_gap(mu[j], Sig[j], level, centre, F,
                                            z_lo, z_hi, v0))
            j = int(np.argmin(g))
            if g[j] < min_lb:
                min_lb = float(g[j])
                worst = {"sample_index": k, "pose": pose.tolist(),
                         "splat_id": int(sub.ids[sel[j]]), "gap": float(g[j])}
            for jj in np.flatnonzero(g <= 0)[:max(0, 20 - len(collisions))]:
                collisions.append({"sample_index": k, "pose": pose.tolist(),
                                   "splat_id": int(sub.ids[sel[jj]]),
                                   "gap": float(g[jj])})
    return {"passed": not collisions, "n_samples": int(len(poses)),
            "n_pairs_checked": int(n_pairs), "n_refined": int(n_refined),
            "min_clearance_lb": float(min_lb), "worst": worst,
            "collisions": collisions,
            "params": {"tau": tau, "level": level, "delta": delta,
                       "n_coarse": n_coarse, "n_fine": n_fine,
                       "margin": margin, "band_abs": [z_lo, z_hi]}}
```

`reach` bounds the xy distance from the pose to any splat centre whose AABB can
meet the dilated footprint AABB plus margin, for splats with xy half-extent at
most `big_extent`; larger splats are always checked, so the KD-tree prefilter
never drops a pair the AABB test would keep.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_replay3d.py`
Expected: `7 passed`.

- [ ] **Step 5: Commit**

```bash
git add gmc/src/gmc/height/replay3d.py gmc/tests/unit/test_height_replay3d.py
git commit -m "[height T5] independent 3D replay with separating-direction certificates"   # plus the trailer
```

### Task 6: `synth3d` — the synthetic 3D table scene

**Files:**
- Create: `gmc/src/gmc/height/synth3d.py`
- Test: `gmc/tests/integration/test_height_table.py`

**Interfaces:**
- Consumes: `GaussianScene3D` (Task 1); `robot_table`, `project_scene` (Task 3); `curve_from_dict` (Task 4); `replay_curve` (Task 5).
- Produces:
  - `table_scene(variant: str) -> tuple[GaussianScene3D, dict]`; `variant in {"open", "closed"}`;
    meta keys `groups` (`{"wall": [ids], "tabletop": [ids], "legs": [ids]}`), `variant`.
  - Constants: `WORKSPACE = (-3.0, -2.0, 3.0, 2.0)` (xmin, ymin, xmax, ymax), `Z_FLOOR = 0.0`,
    `TABLETOP_XY = (-0.3, -0.6, 0.3, 0.6)`, `START = (-2.2, 0.0, 0.0)`, `GOAL = (2.2, 0.0, 0.0)`.

Geometry (spec §5.2, support boundaries at ρ = 2): wall of spheres (σ = 0.04, radius 0.08,
spacing 0.10) centred on x = 0 from z = 0.08 to 2.48, filling y ∈ [−2.5, −0.6] and [0.6, 1.2]
(`closed` also [1.2, 2.5]); tabletop of flat splats (σ = 0.04, 0.04, 0.01) covering exactly
x ∈ [−0.3, 0.3] × y ∈ [−0.6, 0.6], z ∈ [0.72, 0.76]; four legs of vertical needles
(σ = 0.015, 0.015, 0.05) at (±0.25, ±0.55), stacked to cover z ∈ [0, 0.72]. Opacity 0.9.

- [ ] **Step 1: Write the failing tests**

```python
# gmc/tests/integration/test_height_table.py
import numpy as np

from gmc.height.pathio import curve_from_dict
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.height.replay3d import replay_curve
from gmc.height.synth3d import GOAL, START, WORKSPACE, Z_FLOOR, table_scene


def map_ids(variant, robot_key, z_c=1.20):
    s3, meta = table_scene(variant)
    s2, _ = project_scene(s3, robot_table(z_c)[robot_key], WORKSPACE, z_floor=Z_FLOOR)
    return {s.primitive_id for s in s2.supports}, meta["groups"]


def test_scene_boundaries_match_the_spec():
    s3, meta = table_scene("closed")
    top = s3.subset(np.isin(s3.ids, meta["groups"]["tabletop"]))
    lo, hi = top.aabb(2.0)
    np.testing.assert_allclose([lo[:, 0].min(), lo[:, 1].min(), hi[:, 0].max(), hi[:, 1].max()],
                               [-0.3, -0.6, 0.3, 0.6], atol=1e-12)
    np.testing.assert_allclose([lo[:, 2].min(), hi[:, 2].max()], [0.72, 0.76], atol=1e-12)
    legs = s3.subset(np.isin(s3.ids, meta["groups"]["legs"]))
    llo, lhi = legs.aabb(2.0)
    np.testing.assert_allclose([llo[:, 2].min(), lhi[:, 2].max()], [0.0, 0.72], atol=1e-12)
    wall_open = table_scene("open")[1]["groups"]["wall"]
    assert len(meta["groups"]["wall"]) > len(wall_open)


def test_T2_7_maps_by_provenance():
    ids, g = map_ids("closed", "sweeper")
    assert ids & set(g["legs"]) and not ids & set(g["tabletop"])
    ids, g = map_ids("closed", "cylinder")
    assert ids & set(g["legs"]) and ids & set(g["tabletop"])
    ids, g = map_ids("closed", "uav")
    assert ids & set(g["wall"]) and not ids & (set(g["tabletop"]) | set(g["legs"]))


def test_T2_6_replay_rejects_cylinder_through_the_table():
    s3, _ = table_scene("closed")
    line = {"schema_version": 2, "segments": [
        {"kind": "TRANSLATION", "q0": list(START), "q1": list(GOAL)}]}
    rep = replay_curve(s3, robot_table()["cylinder"], curve_from_dict(line), z_floor=Z_FLOOR)
    assert not rep["passed"]
    top_or_wall = {c["splat_id"] for c in rep["collisions"]}
    assert top_or_wall


def test_sweeper_straight_line_under_the_table_replays_clean():
    s3, _ = table_scene("closed")
    line = {"schema_version": 2, "segments": [
        {"kind": "TRANSLATION", "q0": list(START), "q1": list(GOAL)}]}
    rep = replay_curve(s3, robot_table()["sweeper"], curve_from_dict(line), z_floor=Z_FLOOR)
    assert rep["passed"], rep["worst"]
```

The last test is a sanity check of the scene itself (the straight line under the
table is physically free for the sweeper). It is not a planning result.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/integration/test_height_table.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'gmc.height.synth3d'`.

- [ ] **Step 3: Write the implementation**

```python
# gmc/src/gmc/height/synth3d.py
"""Synthetic 3D table scene for the T2 gates (spec §5.2).

Every boundary is placed so the rho=2 support surfaces land exactly on the
nominal numbers: sphere radius R = 2*sigma, centres inset by R.
"""
import numpy as np

from .ply3d import GaussianScene3D

LEVEL = 2.0
WORKSPACE = (-3.0, -2.0, 3.0, 2.0)
Z_FLOOR = 0.0
TABLETOP_XY = (-0.3, -0.6, 0.3, 0.6)
START = (-2.2, 0.0, 0.0)
GOAL = (2.2, 0.0, 0.0)

WALL_SIGMA, WALL_SPACING = 0.04, 0.10
TOP_SIGMA = (0.04, 0.04, 0.01)
LEG_SIGMA = (0.015, 0.015, 0.05)
LEG_XY = [(0.25, 0.55), (0.25, -0.55), (-0.25, 0.55), (-0.25, -0.55)]


def _span(lo, hi, spacing):
    n = max(1, int(np.ceil((hi - lo) / spacing - 1e-9))) + 1
    return np.linspace(lo, hi, n)


def _wall_segment(y_lo, y_hi):
    R = LEVEL * WALL_SIGMA
    ys = _span(y_lo + R, y_hi - R, WALL_SPACING)
    zs = _span(R, 2.5 - R + 0.06, WALL_SPACING)        # top row centre 2.48
    yy, zz = np.meshgrid(ys, zs, indexing="ij")
    pts = np.column_stack([np.zeros(yy.size), yy.ravel(), zz.ravel()])
    return pts, np.tile(np.eye(3) * WALL_SIGMA ** 2, (len(pts), 1, 1))


def table_scene(variant: str):
    if variant not in ("open", "closed"):
        raise ValueError("variant must be 'open' or 'closed'")
    parts, groups = [], {"wall": [], "tabletop": [], "legs": []}

    segments = [(-2.5, -0.6), (0.6, 1.2)] + ([(1.2, 2.5)] if variant == "closed" else [])
    for y_lo, y_hi in segments:
        parts.append(("wall",) + _wall_segment(y_lo, y_hi))

    sx, sy, sz = TOP_SIGMA
    xs = _span(TABLETOP_XY[0] + LEVEL * sx, TABLETOP_XY[2] - LEVEL * sx, 0.10)
    ys = _span(TABLETOP_XY[1] + LEVEL * sy, TABLETOP_XY[3] - LEVEL * sy, 0.10)
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    top = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, 0.74)])
    parts.append(("tabletop", top, np.tile(np.diag([sx ** 2, sy ** 2, sz ** 2]), (len(top), 1, 1))))

    lx, ly, lz = LEG_SIGMA
    zc = _span(LEVEL * lz, 0.72 - LEVEL * lz, 0.10)
    for x, y in LEG_XY:
        pts = np.column_stack([np.full(len(zc), x), np.full(len(zc), y), zc])
        parts.append(("legs", pts, np.tile(np.diag([lx ** 2, ly ** 2, lz ** 2]), (len(pts), 1, 1))))

    means, covs, next_id = [], [], 0
    for group, pts, cv in parts:
        groups[group].extend(range(next_id, next_id + len(pts)))
        next_id += len(pts)
        means.append(pts)
        covs.append(cv)
    means, covs = np.vstack(means), np.vstack(covs)
    scene = GaussianScene3D(means, covs, np.full(len(means), 0.9),
                            np.arange(len(means)), f"table_{variant}")
    return scene, {"groups": groups, "variant": variant}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/integration/test_height_table.py`
Expected: `4 passed`. If `test_scene_boundaries_match_the_spec` fails on the wall top row, fix
`_wall_segment` so the top row centre is 2.42–2.48; do not change the tabletop or leg numbers.

- [ ] **Step 5: Commit**

```bash
git add gmc/src/gmc/height/synth3d.py gmc/tests/integration/test_height_table.py
git commit -m "[height T6] synthetic 3D table scene with provenance groups"   # plus the trailer
```

### Task 7: `run` — compile → query → verify on a projected map

**Files:**
- Create: `gmc/src/gmc/height/run.py`
- Test: `gmc/tests/unit/test_height_run.py`

**Interfaces:**
- Consumes: `PrismRobot` (Task 3); `curve_to_dict`, `save_path_json` (Task 4); from the unchanged core:
  `gmc.budget.WorkLedger`, `gmc.spatial.bvh.query_candidate_pairs(scene, robot, workspace) -> obj with .oracles`,
  `gmc.orientation.slab_builder.build_slabs(scene, robot, cfg, oracles, ledger=)`,
  `gmc.mobility.graph.compile_mobility(scene, robot, cfg, oracles, dec, ledger=)`,
  `gmc.mobility.query.query(start: Pose2, goal: Pose2, mc) -> PlanResult`,
  `gmc.verification.path.verify_curve(oracles, workspace, curve, eps_clear, theta_min, *, expected_start, expected_goal) -> VerifyReport`,
  `gmc.mobility.lineage.component_slice(slab)`, `slab_has_safe_components(slab)`.
- Produces:
  - `with_overrides(cfg, *, initial_intervals=None, max_depth=None, max_refinement_rounds=None, max_wall_seconds=None, max_support_calls=None) -> Config`
  - `compile_and_query(scene2d, robot, cfg, start, goal, *, out_dir=None) -> tuple[dict, MobilityCompiler]`
    where `start`, `goal` are `(x, y, theta)`. Result dict keys: `status, clearance_lb, reason,
    n_supports, n_pairs, n_slabs, compile_seconds, query_seconds, safe_nodes, safe_edges,
    possible_nodes, possible_edges, verify` (`{"ran", "certified", "min_clearance", "reason"}`), `curve` (dict or None).
    With `out_dir`, writes `result.json` and (if a curve exists) `path.json` there.
  - `safe_areas(mc) -> list[dict]` with `lo, hi, area` per slab.

- [ ] **Step 1: Write the failing tests**

```python
# gmc/tests/unit/test_height_run.py
import json

import pytest

from gmc.height.prism import robot_table
from gmc.height.run import compile_and_query, safe_areas, with_overrides
from gmc.synth import single_door

from ..conftest import SMALL_WS, make_cfg


@pytest.fixture(scope="module")
def door_run(tmp_path_factory):
    cfg = make_cfg(theta_min=5e-3, initial_intervals=8)
    out = tmp_path_factory.mktemp("door")
    res, mc = compile_and_query(single_door(0.6, workspace=SMALL_WS),
                                robot_table()["ellipse_toy"], cfg,
                                (-1.7, 0.0, 0.1), (1.7, 0.0, 0.1), out_dir=out)
    return res, mc, out


def test_door_is_reachable_and_independently_verified(door_run):
    res, _, out = door_run
    assert res["status"] == "REACHABLE"
    assert res["verify"]["ran"] and res["verify"]["certified"]
    assert (out / "path.json").is_file()
    assert json.loads((out / "result.json").read_text())["status"] == "REACHABLE"


def test_safe_areas_cover_every_slab(door_run):
    _, mc, _ = door_run
    areas = safe_areas(mc)
    assert len(areas) == len(mc.decomposition.slabs)
    assert all(a["area"] >= 0 for a in areas) and any(a["area"] > 0 for a in areas)


def test_overrides_touch_only_named_fields():
    cfg = make_cfg()
    new = with_overrides(cfg, initial_intervals=1, max_depth=0, max_wall_seconds=999.0)
    assert new.orientation.initial_intervals == 1 and new.orientation.max_depth == 0
    assert new.query.max_wall_seconds == 999.0
    assert new.orientation.theta_min == cfg.orientation.theta_min
    assert new.pair_approx == cfg.pair_approx
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_run.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'gmc.height.run'`.

- [ ] **Step 3: Write the implementation**

```python
# gmc/src/gmc/height/run.py
"""Run the unchanged GMC pipeline on one projected map (mirrors k2_door_gate Stage 4)."""
import json
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from ..budget import WorkLedger
from ..mobility.graph import compile_mobility
from ..mobility.lineage import component_slice, slab_has_safe_components
from ..mobility.query import query
from ..orientation.slab_builder import build_slabs
from ..spatial.bvh import query_candidate_pairs
from ..types import Pose2
from ..verification.path import verify_curve
from .pathio import curve_to_dict, save_path_json


def with_overrides(cfg, *, initial_intervals=None, max_depth=None,
                   max_refinement_rounds=None, max_wall_seconds=None,
                   max_support_calls=None):
    orient = cfg.orientation
    if initial_intervals is not None:
        orient = replace(orient, initial_intervals=int(initial_intervals))
    if max_depth is not None:
        orient = replace(orient, max_depth=int(max_depth))
    q = cfg.query
    changes = {k: v for k, v in (("max_refinement_rounds", max_refinement_rounds),
                                 ("max_wall_seconds", max_wall_seconds),
                                 ("max_support_calls", max_support_calls)) if v is not None}
    if changes:
        q = replace(q, **changes)
    return replace(cfg, orientation=orient, query=q)


def _pose(v):
    return Pose2(np.asarray(v[:2], dtype=float), float(v[2]))


def compile_and_query(scene2d, robot, cfg, start, goal, *, out_dir=None):
    body = robot.footprint
    ledger = WorkLedger()
    t0 = time.time()
    oracles = list(query_candidate_pairs(scene2d, body, scene2d.workspace).oracles)
    for o in oracles:
        o.ledger = ledger
    dec = build_slabs(scene2d, body, cfg, oracles, ledger=ledger)
    mc = compile_mobility(scene2d, body, cfg, oracles, dec, ledger=ledger)
    compile_s = time.time() - t0
    s, g = _pose(start), _pose(goal)
    t1 = time.time()
    result = query(s, g, mc)
    query_s = time.time() - t1
    verify = {"ran": False}
    if result.curve is not None:
        rep = verify_curve(tuple(oracles), scene2d.workspace, result.curve,
                           cfg.query.eps_clear, cfg.orientation.theta_min,
                           expected_start=s, expected_goal=g)
        verify = {"ran": True, "certified": bool(rep.certified),
                  "min_clearance": float(rep.min_clearance), "reason": rep.reason}
    res = {"status": result.status.name,
           "clearance_lb": result.clearance_lower_bound,
           "reason": result.report.get("reason"),
           "n_supports": len(scene2d.supports), "n_pairs": len(oracles),
           "n_slabs": len(dec.slabs), "compile_seconds": compile_s,
           "query_seconds": query_s,
           "safe_nodes": mc.M_safe.number_of_nodes(),
           "safe_edges": mc.M_safe.number_of_edges(),
           "possible_nodes": mc.M_possible.number_of_nodes(),
           "possible_edges": mc.M_possible.number_of_edges(),
           "verify": verify,
           "curve": curve_to_dict(result.curve) if result.curve is not None else None,
           "start": list(map(float, start)), "goal": list(map(float, goal))}
    if out_dir is not None:
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        if result.curve is not None:
            save_path_json(result.curve, out / "path.json")
        (out / "result.json").write_text(json.dumps(res, indent=2, default=str))
    return res, mc


def safe_areas(mc):
    out = []
    for slab in mc.decomposition.slabs:
        area = 0.0
        if slab_has_safe_components(slab):
            area = float(sum(c.geometry.area for c in component_slice(slab).D_safe))
        out.append({"lo": float(slab.interval.lo), "hi": float(slab.interval.hi),
                    "area": area})
    return out
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_run.py`
Expected: `3 passed` (the door compile takes well under a few minutes).

- [ ] **Step 5: Commit**

```bash
git add gmc/src/gmc/height/run.py gmc/tests/unit/test_height_run.py
git commit -m "[height T7] compile-query-verify runner for projected maps"   # plus the trailer
```

### Task 8: `viz` — top-down + side videos, comparison video, 3D PLY

**Files:**
- Create: `gmc/src/gmc/height/viz.py`
- Test: `gmc/tests/unit/test_height_viz.py`

**Interfaces:**
- Consumes: `GaussianScene3D` (Task 1), `PrismRobot` (Task 3), `curve_from_dict`, `sample_curve`, `polyline_xy` (Task 4).
- Produces:
  - `save_animation(fig, update, n_frames, out_stem, fps=20) -> Path` — `.mp4` if `imageio_ffmpeg` imports, else `.gif`.
  - `robot_video(scene2d, robot, result, out_stem, *, scene3d=None, z_floor=0.0, title="", max_frames=300, tau=0.3, key_fracs=(0.0, 0.5, 1.0)) -> dict`
    with `video` (Path or None), `frames` (list of PNG Paths). `result` is a Task 7 result dict. `scene3d=None` → top-down panel only.
    No curve → one PNG with the status, `video=None`.
  - `comparison_video(runs, out_stem, *, scene3d, z_floor, max_frames=300) -> dict` — `runs` = list of
    `{"label", "scene2d", "robot", "result"}`; top-down panels side by side, synchronized by normalized progress.
  - `corridor_profile(scene3d, poly_xy (M,2), half_width, z_floor, tau=0.3) -> (s, z, w)`
  - `write_scene_ply(path, scene3d, window, z_floor, tracks, tau=0.3) -> int` — `tracks` = list of `(robot, poses (K,3), rgb)`; returns vertex count.

- [ ] **Step 1: Write the failing tests**

```python
# gmc/tests/unit/test_height_viz.py
import numpy as np
from matplotlib.image import imread
from shapely.geometry import box

from gmc.height.pathio import curve_from_dict
from gmc.height.ply3d import GaussianScene3D
from gmc.height.prism import robot_table
from gmc.height.viz import corridor_profile, robot_video, write_scene_ply
from gmc.types import GaussianSupport2D, SceneModel2D

LINE = {"schema_version": 2, "segments": [
    {"kind": "TRANSLATION", "q0": [-1.0, 0.0, 0.0], "q1": [1.0, 0.0, 0.0]}]}


def tiny():
    means = np.array([[0.0, 0.5, 0.74], [0.0, 3.0, 0.2]])
    s3 = GaussianScene3D(means, np.tile(np.eye(3) * 1e-3, (2, 1, 1)),
                         np.array([0.9, 0.9]), np.array([0, 1]), "tiny")
    s2 = SceneModel2D((GaussianSupport2D(np.array([0.0, 0.5]), np.eye(2) * 1e-3, 2.0, 0),),
                      box(-2, -2, 2, 2), "tiny2d")
    return s3, s2


def test_robot_video_writes_video_and_readable_frames(tmp_path):
    s3, s2 = tiny()
    res = {"status": "REACHABLE", "curve": LINE, "verify": {"certified": True}}
    out = robot_video(s2, robot_table()["sweeper"], res, tmp_path / "sweeper",
                      scene3d=s3, z_floor=0.0, title="t", max_frames=12)
    assert out["video"].exists() and out["video"].stat().st_size > 0
    assert len(out["frames"]) == 3
    assert imread(out["frames"][1]).ndim == 3


def test_robot_video_without_curve_renders_status_png(tmp_path):
    _, s2 = tiny()
    out = robot_video(s2, robot_table()["cylinder"], {"status": "UNKNOWN", "curve": None},
                      tmp_path / "cyl")
    assert out["video"] is None and out["frames"][0].exists()


def test_corridor_profile_keeps_only_nearby_splats():
    s3, _ = tiny()
    s, z, w = corridor_profile(s3, np.array([[-1, 0], [1, 0]]), 0.6, 0.0)
    assert len(s) == 1 and abs(z[0] - 0.74) < 1e-12 and abs(s[0] - 1.0) < 1e-12


def test_scene_ply_vertex_count_matches_header(tmp_path):
    s3, _ = tiny()
    poses = np.array([[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    n = write_scene_ply(tmp_path / "s.ply", s3, (-2, -2, 2, 2), 0.0,
                        [(robot_table()["sweeper"], poses, (255, 0, 0))])
    head = (tmp_path / "s.ply").read_bytes()[:400].decode("ascii", "ignore")
    assert f"element vertex {n}" in head and n > 2
    body = (tmp_path / "s.ply").stat().st_size - (head.index("end_header\n") + 11)
    assert body == n * 15
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_viz.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'gmc.height.viz'`.

- [ ] **Step 3: Write the implementation**

```python
# gmc/src/gmc/height/viz.py
"""Videos and 3D exports that show whether and how a robot got through (spec §6)."""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.patches import Ellipse, Rectangle

from .pathio import curve_from_dict, polyline_xy, sample_curve


def save_animation(fig, update, n_frames, out_stem, fps=20):
    out_stem = Path(out_stem)
    out_stem.parent.mkdir(parents=True, exist_ok=True)
    anim = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    try:
        import imageio_ffmpeg
        plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
        path = out_stem.with_suffix(".mp4")
        anim.save(path, writer=animation.FFMpegWriter(fps=fps, bitrate=2400))
    except ImportError:
        path = out_stem.with_suffix(".gif")
        anim.save(path, writer=animation.PillowWriter(fps=fps))
    return path


def _support_patch(sup, **kw):
    shape = sup.level ** 2 * np.asarray(sup.covariance)
    w, v = np.linalg.eigh(shape)
    ang = np.degrees(np.arctan2(v[1, 1], v[0, 1]))
    return Ellipse(sup.mean, 2 * np.sqrt(w[1]), 2 * np.sqrt(w[0]), angle=ang, **kw)


def _footprint_patches(robot, pose, **kw):
    x, y, th = pose
    c, s = np.cos(th), np.sin(th)
    R = np.array([[c, -s], [s, c]])
    out = []
    for sup in robot.footprint.supports:
        shape = R @ (sup.level ** 2 * np.asarray(sup.covariance)) @ R.T
        w, v = np.linalg.eigh(shape)
        ang = np.degrees(np.arctan2(v[1, 1], v[0, 1]))
        out.append(Ellipse(np.array([x, y]) + R @ np.asarray(sup.mean),
                           2 * np.sqrt(w[1]), 2 * np.sqrt(w[0]), angle=ang, **kw))
    return out


def corridor_profile(scene3d, poly_xy, half_width, z_floor, tau=0.3):
    from shapely.geometry import LineString, Point
    sub = scene3d.subset(scene3d.opacity > tau)
    line = LineString(poly_xy)
    lo = np.min(poly_xy, axis=0) - half_width
    hi = np.max(poly_xy, axis=0) + half_width
    cand = np.flatnonzero(np.all((sub.means[:, :2] >= lo) & (sub.means[:, :2] <= hi), axis=1))
    s, z, w = [], [], []
    for i in cand:
        p = Point(sub.means[i, :2])
        if line.distance(p) <= half_width:
            s.append(line.project(p))
            z.append(sub.means[i, 2] - z_floor)
            w.append(sub.opacity[i])
    return np.asarray(s), np.asarray(z), np.asarray(w)


def _density(ax, scene3d, extent, tau):
    sub = scene3d.subset(scene3d.opacity > tau)
    H, _, _ = np.histogram2d(sub.means[:, 0], sub.means[:, 1], bins=400,
                             range=[[extent[0], extent[2]], [extent[1], extent[3]]],
                             weights=sub.opacity)
    ax.imshow(np.log1p(H.T), origin="lower", cmap="Greys",
              extent=[extent[0], extent[2], extent[1], extent[3]])


def _frame_poses(result, robot, max_frames):
    curve = curve_from_dict(result["curve"])
    poses = sample_curve(curve, 0.02, robot.max_radius())
    idx = np.unique(np.linspace(0, len(poses) - 1, min(max_frames, len(poses))).astype(int))
    return curve, poses[idx]


def robot_video(scene2d, robot, result, out_stem, *, scene3d=None, z_floor=0.0,
                title="", max_frames=300, tau=0.3, key_fracs=(0.0, 0.5, 1.0)):
    out_stem = Path(out_stem)
    out_stem.parent.mkdir(parents=True, exist_ok=True)
    ext = scene2d.workspace.bounds
    side = scene3d is not None and not robot.is_2d
    fig, axes = plt.subplots(1, 2 if side else 1, figsize=(13 if side else 7, 6),
                             squeeze=False)
    ax = axes[0, 0]
    if scene3d is not None:
        _density(ax, scene3d, ext, tau)
    for sup in scene2d.supports:
        ax.add_patch(_support_patch(sup, fc=(0.85, 0.2, 0.2, 0.25), ec="none"))
    ax.set_xlim(ext[0], ext[2]); ax.set_ylim(ext[1], ext[3]); ax.set_aspect("equal")
    head = f"{title}  {robot.name}: {result['status']}"
    if result.get("curve") is None:
        ax.set_title(head)
        png = out_stem.with_name(out_stem.name + "_status.png")
        fig.savefig(png, dpi=110); plt.close(fig)
        return {"video": None, "frames": [png]}
    curve, poses = _frame_poses(result, robot, max_frames)
    xy = polyline_xy(curve)
    ax.plot(xy[:, 0], xy[:, 1], "b--", lw=1)
    trail, = ax.plot([], [], "b-", lw=2)
    feet = []
    rep3 = result.get("replay3d") or {}
    ax.set_title(head + (f"  replay3d={rep3.get('passed')} lb={rep3.get('min_clearance_lb')}"
                         if rep3 else ""))
    if side:
        axs = axes[0, 1]
        s, z, w = corridor_profile(scene3d, xy, robot.max_radius(), z_floor, tau)
        axs.scatter(s, z, s=2, c="k", alpha=np.clip(w, 0.05, 0.6))
        z_lo, z_hi = robot.z_lo, robot.z_hi
        band = Rectangle((0, z_lo), 2 * robot.max_radius(), z_hi - z_lo,
                         fc=(0.1, 0.4, 0.9, 0.5), ec="b")
        axs.add_patch(band)
        L = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
        axs.set_xlim(-0.5, L + 0.5); axs.set_ylim(0, max(2.2, z_hi + 0.3))
        axs.set_xlabel("distance along path (m)"); axs.set_ylabel("height above floor (m)")
        cum = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1))])

    def update(k):
        for p in feet:
            p.remove()
        feet.clear()
        for p in _footprint_patches(robot, poses[k], fc=(0.1, 0.4, 0.9, 0.6), ec="b"):
            ax.add_patch(p); feet.append(p)
        trail.set_data(poses[:k + 1, 0], poses[:k + 1, 1])
        if side:
            band.set_x(cum[k] - robot.max_radius())
        return feet

    frames = []
    for f in key_fracs:
        k = int(round(f * (len(poses) - 1)))
        update(k)
        png = out_stem.with_name(f"{out_stem.name}_f{int(100 * f):03d}.png")
        fig.savefig(png, dpi=110)
        frames.append(png)
    video = save_animation(fig, update, len(poses), out_stem)
    plt.close(fig)
    return {"video": video, "frames": frames}


def comparison_video(runs, out_stem, *, scene3d, z_floor, max_frames=300):
    fig, axes = plt.subplots(1, len(runs), figsize=(6 * len(runs), 6), squeeze=False)
    states = []
    for ax, run in zip(axes[0], runs):
        ext = run["scene2d"].workspace.bounds
        _density(ax, scene3d, ext, 0.3)
        for sup in run["scene2d"].supports:
            ax.add_patch(_support_patch(sup, fc=(0.85, 0.2, 0.2, 0.25), ec="none"))
        ax.set_xlim(ext[0], ext[2]); ax.set_ylim(ext[1], ext[3]); ax.set_aspect("equal")
        ax.set_title(f"{run['label']}: {run['result']['status']}")
        poses = None
        if run["result"].get("curve") is not None:
            _, poses = _frame_poses(run["result"], run["robot"], max_frames)
        states.append({"ax": ax, "robot": run["robot"], "poses": poses, "feet": []})

    def update(k):
        t = k / max(1, max_frames - 1)
        for st in states:
            for p in st["feet"]:
                p.remove()
            st["feet"].clear()
            if st["poses"] is None:
                continue
            j = int(round(t * (len(st["poses"]) - 1)))
            for p in _footprint_patches(st["robot"], st["poses"][j], fc=(0.1, 0.4, 0.9, 0.6), ec="b"):
                st["ax"].add_patch(p); st["feet"].append(p)
        return []

    update(max_frames // 2)
    png = Path(out_stem).with_name(Path(out_stem).name + "_f050.png")
    Path(png).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(png, dpi=100)
    video = save_animation(fig, update, max_frames, out_stem)
    plt.close(fig)
    return {"video": video, "frames": [png]}


def write_scene_ply(path, scene3d, window, z_floor, tracks, tau=0.3):
    xmin, ymin, xmax, ymax = window
    sub = scene3d.subset((scene3d.opacity > tau)
                         & (scene3d.means[:, 0] >= xmin) & (scene3d.means[:, 0] <= xmax)
                         & (scene3d.means[:, 1] >= ymin) & (scene3d.means[:, 1] <= ymax))
    hz = np.clip((sub.means[:, 2] - z_floor) / 2.5, 0, 1)
    rgb = (plt.get_cmap("viridis")(hz)[:, :3] * 255).astype(np.uint8)
    pts, cols = [sub.means], [rgb]
    for robot, poses, colour in tracks:
        ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
        z_lo, z_hi = robot.band_abs(z_floor)
        zs = np.linspace(z_lo, z_hi, 6)
        r = robot.max_radius()
        for x, y, _ in poses[:: max(1, len(poses) // 60)]:
            ring = np.column_stack([x + r * np.cos(ang), y + r * np.sin(ang)])
            for z in zs:
                pts.append(np.column_stack([ring, np.full(len(ring), z)]))
                cols.append(np.tile(np.asarray(colour, np.uint8), (len(ring), 1)))
    P, C = np.vstack(pts).astype("<f4"), np.vstack(cols)
    arr = np.zeros(len(P), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                  ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    arr["x"], arr["y"], arr["z"] = P[:, 0], P[:, 1], P[:, 2]
    arr["red"], arr["green"], arr["blue"] = C[:, 0], C[:, 1], C[:, 2]
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(P)}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\n"
              "end_header\n")
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        arr.tofile(f)
    return int(len(P))
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_viz.py`
Expected: `4 passed`.

- [ ] **Step 5: Open the frames and look at them**

Open `tmp`-rendered frames from a quick manual call (same inputs as the first test,
output to `/scratch/wg2381/splathjb/gmc/outputs/height/viz_smoke/`) with the image reader.
Confirm: grey density, red obstacle ellipse near (0, 0.5), dashed blue path, blue footprint
moving left to right across the three frames, and in the side panel a black dot at height
0.74 above the blue band. Record what you saw in the worklog.

- [ ] **Step 6: Commit**

```bash
git add gmc/src/gmc/height/viz.py gmc/tests/unit/test_height_viz.py
git commit -m "[height T8] top-down and side videos, comparison video, 3D PLY export"   # plus the trailer
```

### Task 9: `height_toy.py` — T1 and T2 gates as batch jobs

**Files:**
- Create: `gmc/experiments/height_toy.py`, `gmc/configs/height_toy.yaml`, `gmc/hpc/height_toy.sbatch`
- Output: `gmc/results/height/toy/<gate>.json`, videos + PNG frames in `gmc/results/height/toy/media/`;
  raw run dirs in `/scratch/wg2381/splathjb/gmc/outputs/height/toy/<gate>/`

**Interfaces:**
- Consumes: Tasks 1–8; `gmc.config.load_config`; `gmc.synth.single_door`, `gate_half_angle`.
- Produces: one JSON per gate with keys `gate, pass, criteria` (dict of named booleans), `result` (Task 7 dict or null),
  `replay3d` (Task 5 dict or null), `extra`. T2-8 adds `extra.showcase_initial_intervals` (1 or 16).
  CLI: `height_toy.py --gate <GATE>` with `GATE ∈ {T1a, T1b, T2-1, T2-2, T2-3, T2-4, T2-5, T2-6, T2-7, T2-8}`.

- [ ] **Step 1: Create the config**

`gmc/configs/height_toy.yaml` is a byte copy of `gmc/configs/toy.yaml` with this first line added:
`# height-band toy gates (spec §5.1-5.2); identical to toy.yaml, kept separate so T2-8 cannot edit toy.yaml`.

```bash
cd gmc && { echo "# height-band toy gates (spec §5.1-5.2); identical to toy.yaml, kept separate so T2-8 cannot edit toy.yaml"; cat configs/toy.yaml; } > configs/height_toy.yaml
```

- [ ] **Step 2: Write the experiment script**

```python
# gmc/experiments/height_toy.py
"""T1/T2 gates for height-band projection (spec §5.1-§5.2). One gate per call."""
import argparse
import json
from pathlib import Path

import numpy as np
from shapely.geometry import box

from gmc.config import load_config
from gmc.height.pathio import crossing_thetas, curve_from_dict, path_crosses
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.height.replay3d import replay_curve
from gmc.height.run import compile_and_query, safe_areas, with_overrides
from gmc.height.synth3d import GOAL, START, TABLETOP_XY, WORKSPACE, Z_FLOOR, table_scene
from gmc.height.viz import robot_video
from gmc.synth import gate_half_angle, single_door

RESULTS = Path("results/height/toy")
RAW = Path("/scratch/wg2381/splathjb/gmc/outputs/height/toy")
TABLE = box(*TABLETOP_XY)


def _cfg():
    return load_config("configs/height_toy.yaml")


def _video(gate, scene2d, robot, res, scene3d=None):
    out = robot_video(scene2d, robot, res, RESULTS / "media" / gate, scene3d=scene3d,
                      z_floor=Z_FLOOR, title=gate)
    return {"video": str(out["video"]) if out["video"] else None,
            "frames": [str(p) for p in out["frames"]]}


def gate_T1(gate):
    width = 0.6 if gate == "T1a" else 0.35
    robot = robot_table()["ellipse_toy"]
    scene = single_door(width)
    res, _ = compile_and_query(scene, robot, _cfg(), (-2.0, 0.0, np.pi / 2),
                               (2.0, 0.0, np.pi / 2), out_dir=RAW / gate)
    if gate == "T1b":
        return {"criteria": {"not_reachable": res["status"] != "REACHABLE"}, "result": res}
    half = gate_half_angle(0.50, 0.20, width)
    thetas = crossing_thetas(curve_from_dict(res["curve"]), 0.0) if res["curve"] else []
    folded = [min(t % np.pi, np.pi - t % np.pi) for t in thetas]
    crit = {"reachable": res["status"] == "REACHABLE",
            "verify_certified": bool(res["verify"].get("certified")),
            "crossing_inside_gate": bool(folded) and all(f < half for f in folded)}
    return {"criteria": crit, "result": res,
            "extra": {"gate_half_angle": half, "crossing_thetas": thetas,
                      "media": _video(gate, scene, robot, res)}}


T2_CASES = {  # gate: (variant, robot key, expectation)
    "T2-1": ("closed", "sweeper", "reach_cross"),
    "T2-2": ("closed", "quadruped", "reach"),
    "T2-3": ("closed", "cylinder", "blocked"),
    "T2-4": ("open", "cylinder", "reach_avoid"),
    "T2-5": ("closed", "uav", "reach_cross"),
}


def gate_T2_run(gate):
    variant, key, expect = T2_CASES[gate]
    s3, _ = table_scene(variant)
    robot = robot_table(1.20)[key]
    s2, stats = project_scene(s3, robot, WORKSPACE, z_floor=Z_FLOOR)
    res, _ = compile_and_query(s2, robot, _cfg(), START, GOAL, out_dir=RAW / gate)
    rep = None
    if res["curve"] is not None:
        rep = replay_curve(s3, robot, curve_from_dict(res["curve"]), z_floor=Z_FLOOR)
        res["replay3d"] = rep
    if expect == "blocked":
        crit = {"not_reachable": res["status"] != "REACHABLE"}
    else:
        crit = {"reachable": res["status"] == "REACHABLE",
                "verify_certified": bool(res["verify"].get("certified")),
                "replay3d_passed": bool(rep and rep["passed"])}
        crosses = bool(res["curve"]) and path_crosses(curve_from_dict(res["curve"]), TABLE)
        if expect == "reach_cross":
            crit["crosses_tabletop"] = crosses
        if expect == "reach_avoid":
            crit["avoids_tabletop"] = bool(res["curve"]) and not crosses
    return {"criteria": crit, "result": res, "replay3d": rep,
            "extra": {"projection": stats, "media": _video(gate, s2, robot, res, s3)}}


def gate_T2_6():
    s3, _ = table_scene("closed")
    line = {"schema_version": 2, "segments": [
        {"kind": "TRANSLATION", "q0": list(START), "q1": list(GOAL)}]}
    rep = replay_curve(s3, robot_table()["cylinder"], curve_from_dict(line), z_floor=Z_FLOOR)
    return {"criteria": {"replay3d_rejects": not rep["passed"]}, "replay3d": rep}


def gate_T2_7():
    s3, meta = table_scene("closed")
    g = {k: set(v) for k, v in meta["groups"].items()}
    ids = {}
    for key in ("sweeper", "cylinder", "uav"):
        s2, _ = project_scene(s3, robot_table(1.20)[key], WORKSPACE, z_floor=Z_FLOOR)
        ids[key] = {s.primitive_id for s in s2.supports}
    crit = {"sweeper_legs_no_top": bool(ids["sweeper"] & g["legs"]) and not ids["sweeper"] & g["tabletop"],
            "cylinder_legs_and_top": bool(ids["cylinder"] & g["legs"]) and bool(ids["cylinder"] & g["tabletop"]),
            "uav_walls_only": bool(ids["uav"] & g["wall"]) and not ids["uav"] & (g["legs"] | g["tabletop"])}
    return {"criteria": crit, "extra": {k: len(v) for k, v in ids.items()}}


def gate_T2_8():
    s3, _ = table_scene("closed")
    robot = robot_table()["sweeper"]
    s2, _ = project_scene(s3, robot, WORKSPACE, z_floor=Z_FLOOR)
    runs = {}
    for n in (1, 16):
        res, mc = compile_and_query(s2, robot, with_overrides(_cfg(), initial_intervals=n),
                                    START, GOAL, out_dir=RAW / f"T2-8_n{n}")
        runs[n] = {"status": res["status"], "areas": safe_areas(mc),
                   "compile_seconds": res["compile_seconds"]}
    ref = runs[1]["areas"][0]["area"]
    all_areas = [a["area"] for n in (1, 16) for a in runs[n]["areas"]]
    rel = max(abs(a - ref) for a in all_areas) / ref if ref > 0 else float("inf")
    same = runs[1]["status"] == runs[16]["status"] and rel <= 1e-6
    return {"criteria": {"same_status": runs[1]["status"] == runs[16]["status"],
                         "areas_within_1e-6": rel <= 1e-6},
            "extra": {"runs": runs, "max_rel_area_diff": rel,
                      "showcase_initial_intervals": 1 if same else 16}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gate", required=True,
                    choices=["T1a", "T1b", "T2-1", "T2-2", "T2-3", "T2-4", "T2-5",
                             "T2-6", "T2-7", "T2-8"])
    gate = ap.parse_args().gate
    RESULTS.mkdir(parents=True, exist_ok=True)
    if gate.startswith("T1"):
        out = gate_T1(gate)
    elif gate in T2_CASES:
        out = gate_T2_run(gate)
    else:
        out = {"T2-6": gate_T2_6, "T2-7": gate_T2_7, "T2-8": gate_T2_8}[gate]()
    out["gate"] = gate
    out["pass"] = all(out["criteria"].values())
    (RESULTS / f"{gate}.json").write_text(json.dumps(out, indent=2, default=str))
    print(f"{gate} pass={out['pass']} criteria={out['criteria']}", flush=True)


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Run the two cheap gates inline**

Run: `cd gmc && PYTHONPATH=src:experiments $PY experiments/height_toy.py --gate T2-7 && PYTHONPATH=src:experiments $PY experiments/height_toy.py --gate T2-6`
Expected: `T2-7 pass=True ...` then `T2-6 pass=True ...`. If either fails, stop and debug with
superpowers:systematic-debugging before submitting anything; these two need no compile.

- [ ] **Step 4: Write the batch wrapper**

```bash
#!/bin/bash
# gmc/hpc/height_toy.sbatch -- one T1/T2 gate per job; submit with --export=ALL,GATE=<gate>
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=height_toy
#SBATCH --time=12:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=4
#SBATCH --output=/scratch/wg2381/splathjb-height/gmc/logs/%x-%j.out
# No --partition line: >6 h jobs must route to cs (see k2_door_gate_stage4.sbatch).
set -euo pipefail
: "${GATE:?set GATE}"
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb-height/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=4 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
echo "=== height_toy GATE=${GATE} $(date -Is) host=$(hostname) job=${SLURM_JOB_ID} ==="
$PY experiments/height_toy.py --gate "${GATE}"
echo "=== HEIGHT_TOY_DONE ${GATE} ==="
```

- [ ] **Step 5: Validate and submit the eight compile gates**

```bash
mkdir -p /scratch/wg2381/splathjb-height/gmc/logs
cd /scratch/wg2381/splathjb-height/gmc
sbatch --test-only --export=ALL,GATE=T1a hpc/height_toy.sbatch
for G in T1a T1b T2-1 T2-2 T2-3 T2-4 T2-5 T2-8; do
  J=$(sbatch --parsable --export=ALL,GATE=$G hpc/height_toy.sbatch)
  echo "$J height_toy $G" >> /scratch/wg2381/claude_jobs/height/jobids/A1.txt
done
cat /scratch/wg2381/claude_jobs/height/jobids/A1.txt
```

Expected: `--test-only` prints an estimated start without error; eight job IDs recorded.
Eight jobs × 4 CPU / 16 G is inside the pre-approved compute envelope (spec §7).

- [ ] **Step 6: Commit (results are collected later by C1)**

```bash
git add gmc/experiments/height_toy.py gmc/configs/height_toy.yaml gmc/hpc/height_toy.sbatch gmc/results/height/toy/T2-6.json gmc/results/height/toy/T2-7.json
git commit -m "[height T9] toy gate script, config and batch wrapper; T2-6/T2-7 results"   # plus the trailer
```

---

## Stage A2 — showcase scene and compute submission (Tasks 10–11)

Before starting: confirm C1's standup exists and every T1/T2 gate JSON in
`gmc/results/height/toy/` has `"pass": true`. If any gate failed, do not start the
showcase; write that to the worklog and your standup and stop (C2 will report it).
Read `showcase_initial_intervals` from `gmc/results/height/toy/T2-8.json`.

### Task 10: `showcase_scene.py` — G0 copy, checks, band maps, case selection

**Files:**
- Create: `gmc/experiments/showcase_scene.py`
- Output (data, gitignored): `/scratch/wg2381/splathjb/splatc_atlas/data/gs_scenes/showcase/{raw/point_cloud.ply, meta.json, processed.npz}`
- Output (committed): `gmc/results/height/showcase/{g0.json, case.json, scale_check.json}`, `gmc/results/height/showcase/figs/*.png`

**Interfaces:**
- Consumes: Tasks 1–3 (`load_3dgs_ply`, `fit_floor`, `gravity_rotation`, `rotate_scene`, `crop_box`, `GaussianScene3D`, `robot_table`, `band_overlap_mask`).
- Produces:
  - `load_processed() -> tuple[GaussianScene3D, dict]` (scene, `g0.json` content) — used by Tasks 11–12.
  - `case.json` keys: `window [xmin,ymin,xmax,ymax]`, `start [x,y,theta]`, `goal [x,y,theta]`, `z_floor`, `z_c`,
    `overhang {top, underside, s_interval}`, `criteria {…}`, `pass`, `notes`.
  - CLI steps: `--step copy | decode | maps | section --seg x0,y0,x1,y1 --name N | case --window … --start … --goal … --zc Z --glass-clear yes|no`.

- [ ] **Step 1: Write the script**

```python
# gmc/experiments/showcase_scene.py
"""G0 for the showcase gallery GS (spec §5.3). Run one --step at a time."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage

from gmc.height.band_shadow import band_overlap_mask
from gmc.height.ply3d import (GaussianScene3D, crop_box, fit_floor, gravity_rotation,
                              load_3dgs_ply, rotate_scene)
from gmc.height.prism import robot_table

SRC = Path("/scratch/sy2366/Project/LccStudio-stage-archive/point_cloud.ply")
SRC_BYTES = 499_121_449
DATA = Path("/scratch/wg2381/splathjb/splatc_atlas/data/gs_scenes/showcase")
RES = Path("results/height/showcase")
FIGS = RES / "figs"
RHO, TAU, CELL = 2.0, 0.3, 0.05
BANDS = [(0.02, 0.10), (0.10, 0.55), (0.55, 1.00), (1.00, 1.75), (1.75, 2.50)]


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 24), b""):
            h.update(chunk)
    return h.hexdigest()


def step_copy():
    assert SRC.stat().st_size == SRC_BYTES, "source size changed; stop and report"
    dst = DATA / "raw" / "point_cloud.ply"
    dst.parent.mkdir(parents=True, exist_ok=True)
    if not dst.exists():
        shutil.copyfile(SRC, dst)
    a, b = sha256(SRC), sha256(dst)
    assert a == b, f"sha256 mismatch {a} != {b}"
    meta = {"raw_file": "raw/point_cloud.ply", "raw_sha256": b, "bytes": SRC_BYTES,
            "source": str(SRC), "lcc_capture": "19017822682139126", "lod": 0,
            "note": "art gallery; reception counter, plinths, benches, glass doors"}
    (DATA / "meta.json").write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta, indent=2))


def _dense_peaks(z, bin_width=0.02, frac=0.2):
    counts, edges = np.histogram(z, bins=np.arange(z.min(), z.max() + bin_width, bin_width))
    thr = frac * counts.max()
    return [0.5 * (edges[i] + edges[i + 1]) for i in range(len(counts))
            if counts[i] >= thr and counts[i] >= counts[max(i - 1, 0)]
            and counts[i] >= counts[min(i + 1, len(counts) - 1)]]


def step_decode():
    scene, st = load_3dgs_ply(DATA / "raw" / "point_cloud.ply", name="showcase")
    g0 = {"decode": st, "n": len(scene)}
    f0 = fit_floor(scene)
    g0["floor_raw"] = f0
    if f0["tilt_deg"] > 0.5:
        scene = rotate_scene(scene, gravity_rotation(f0["normal"]), f0["centroid"])
        g0["gravity_rotation"] = gravity_rotation(f0["normal"]).tolist()
    else:
        g0["gravity_rotation"] = np.eye(3).tolist()
    f1 = fit_floor(scene)
    z_f = f1["z_floor"]
    g0["floor"] = f1
    opq = scene.means[scene.opacity > 0.5, 2]
    above = opq[opq > z_f + 1.5]
    ceiling = max(_dense_peaks(above)) if len(above) else float(np.percentile(opq, 99.5))
    g0["ceiling_z"] = float(ceiling)
    g0["ceiling_height_m"] = float(ceiling - z_f)
    lo = np.array([-np.inf, -np.inf, z_f - 0.2])
    hi = np.array([np.inf, np.inf, ceiling + 0.2])
    scene, dropped = crop_box(scene, lo, hi)
    g0["crop"] = {"z_range": [lo[2], hi[2]], "dropped": dropped, "kept": len(scene)}
    lo3, hi3 = scene.aabb(RHO)
    near_floor = (np.abs(scene.means[:, 2] - z_f) < 0.05) & (scene.opacity > TAU)
    tops = hi3[near_floor, 2] - z_f
    g0["floor_splat_top_above_floor_m"] = {q: float(np.percentile(tops, q)) for q in (50, 90, 99, 99.9)}
    g0["floor_splats_reaching_z_lo_0.02"] = int((tops > 0.02).sum())
    np.savez(DATA / "processed.npz", means=scene.means, covs=scene.covs,
             opacity=scene.opacity, ids=scene.ids)
    RES.mkdir(parents=True, exist_ok=True)
    (RES / "g0.json").write_text(json.dumps(g0, indent=2))
    print(json.dumps({k: v for k, v in g0.items() if k != "gravity_rotation"}, indent=2))


def load_processed():
    d = np.load(DATA / "processed.npz")
    g0 = json.loads((RES / "g0.json").read_text())
    return GaussianScene3D(d["means"], d["covs"], d["opacity"], d["ids"], "showcase"), g0


def _grid(scene, extent):
    nx = int(np.ceil((extent[2] - extent[0]) / CELL))
    ny = int(np.ceil((extent[3] - extent[1]) / CELL))
    return nx, ny


def _occupancy(scene, z_f, band, extent):
    """Cells touched by the xy-AABB of any opaque splat overlapping the band (selection aid only)."""
    sub = scene.subset(scene.opacity > TAU)
    m = band_overlap_mask(sub.means[:, 2], sub.covs[:, 2, 2], z_f + band[0], z_f + band[1], RHO)
    lo, hi = sub.subset(m).aabb(RHO)
    nx, ny = _grid(scene, extent)
    occ = np.zeros((nx, ny), dtype=bool)
    i0 = np.clip(((lo[:, 0] - extent[0]) / CELL).astype(int), 0, nx - 1)
    i1 = np.clip(((hi[:, 0] - extent[0]) / CELL).astype(int), 0, nx - 1)
    j0 = np.clip(((lo[:, 1] - extent[1]) / CELL).astype(int), 0, ny - 1)
    j1 = np.clip(((hi[:, 1] - extent[1]) / CELL).astype(int), 0, ny - 1)
    small = (i1 - i0 <= 4) & (j1 - j0 <= 4)
    for a, b, c, d in zip(i0[~small], i1[~small], j0[~small], j1[~small]):
        occ[a:b + 1, c:d + 1] = True
    for di in range(5):
        for dj in range(5):
            sel = small & (i0 + di <= i1) & (j0 + dj <= j1)
            occ[i0[sel] + di, j0[sel] + dj] = True
    return occ


def step_maps():
    scene, g0 = load_processed()
    z_f = g0["floor"]["z_floor"]
    opq = scene.means[scene.opacity > 0.5]
    ext = [*np.percentile(opq[:, 0:2], 0.5, axis=0) - 0.5, *np.percentile(opq[:, 0:2], 99.5, axis=0) + 0.5]
    ext = [float(ext[0]), float(ext[1]), float(ext[2]), float(ext[3])]
    FIGS.mkdir(parents=True, exist_ok=True)
    occ = {}
    for band in BANDS:
        occ[band] = _occupancy(scene, z_f, band, ext)
        plt.figure(figsize=(12, 10))
        plt.imshow(occ[band].T, origin="lower", cmap="Greys",
                   extent=[ext[0], ext[2], ext[1], ext[3]])
        plt.title(f"occupied cells, band {band} m above floor (selection aid)")
        plt.grid(alpha=0.3); plt.savefig(FIGS / f"band_{band[0]:.2f}_{band[1]:.2f}.png", dpi=120); plt.close()
    overhang = ~_occupancy(scene, z_f, (0.02, 0.15), ext) & occ[(0.55, 1.00)]
    plt.figure(figsize=(12, 10))
    plt.imshow(occ[(0.02, 0.10)].T, origin="lower", cmap="Greys", alpha=0.5,
               extent=[ext[0], ext[2], ext[1], ext[3]])
    plt.imshow(np.ma.masked_where(~overhang.T, overhang.T), origin="lower", cmap="autumn",
               extent=[ext[0], ext[2], ext[1], ext[3]])
    plt.title("overhang candidates: free in [0.02,0.15], occupied in [0.55,1.00]")
    plt.grid(alpha=0.3); plt.savefig(FIGS / "overhang_candidates.png", dpi=120); plt.close()
    np.savez(DATA / "maps.npz", extent=np.array(ext), overhang=overhang,
             **{f"occ_{a:.2f}_{b:.2f}": v for (a, b), v in occ.items()})
    print("extent", ext, "overhang cells", int(overhang.sum()))


def _section(scene, z_f, p0, p1, half=0.2):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    L = float(np.linalg.norm(d))
    u = d / L
    sub = scene.subset(scene.opacity > TAU)
    rel = sub.means[:, :2] - p0
    s = rel @ u
    off = np.abs(rel @ np.array([-u[1], u[0]]))
    sel = (s >= 0) & (s <= L) & (off <= half)
    lo, hi = sub.subset(sel).aabb(RHO)
    return s[sel], lo[:, 2] - z_f, hi[:, 2] - z_f, L


def step_section(seg, name):
    scene, g0 = load_processed()
    z_f = g0["floor"]["z_floor"]
    s, zb, zt, L = _section(scene, z_f, seg[:2], seg[2:])
    plt.figure(figsize=(14, 5))
    plt.vlines(s, zb, zt, lw=0.3, color="k", alpha=0.3)
    plt.xlabel("distance along section (m)"); plt.ylabel("height above floor (m)")
    plt.ylim(-0.1, g0["ceiling_height_m"] + 0.2); plt.grid(alpha=0.3)
    plt.title(f"section {name} {seg}")
    FIGS.mkdir(parents=True, exist_ok=True)
    plt.savefig(FIGS / f"section_{name}.png", dpi=120); plt.close()
    hist, edges = np.histogram(zt, bins=np.arange(0, g0["ceiling_height_m"] + 0.05, 0.02))
    peaks = [float(edges[i]) for i in np.argsort(hist)[-8:][::-1]]
    print(json.dumps({"name": name, "length": L, "n": int(len(s)), "top_height_modes": peaks}))


def _free_dist(scene, z_f, band, ext):
    occ = _occupancy(scene, z_f, band, ext)
    return ndimage.distance_transform_edt(~occ) * CELL


def step_case(window, start, goal, z_c, glass_clear):
    scene, g0 = load_processed()
    z_f = g0["floor"]["z_floor"]
    ext = list(map(float, window))
    robots = robot_table(z_c)
    crit, notes = {}, {}
    crit["window_at_most_8x8"] = (ext[2] - ext[0] <= 8.0) and (ext[3] - ext[1] <= 8.0)
    s, zb, zt, L = _section(scene, z_f, start[:2], goal[:2], half=0.2)
    free_low = np.ones(int(np.ceil(L / 0.02)) + 1, dtype=bool)
    over = np.zeros_like(free_low)
    uav_hit = np.zeros_like(free_low)
    k = np.clip((s / 0.02).astype(int), 0, len(free_low) - 1)
    lowhit = (zb <= 0.15) & (zt >= 0.02)
    free_low[k[lowhit]] = False
    top_mid = (zb > 0.15) & (zb < z_c - 0.10)
    over[k[top_mid]] = True
    uav_hit[k[(zb <= z_c + 0.10) & (zt >= z_c - 0.10)]] = True
    runs = np.flatnonzero(free_low & over & ~uav_hit)
    crit["straight_line_under_overhang"] = bool(len(runs) >= 5)
    if len(runs):
        seg = (zb > 0.15) & (zb < z_c - 0.10) & (s >= runs.min() * 0.02) & (s <= runs.max() * 0.02)
        top = float(zt[seg].max()) if seg.any() else None
        under = float(zb[seg].min()) if seg.any() else None
        notes["overhang"] = {"top": top, "underside": under,
                             "s_interval": [runs.min() * 0.02, runs.max() * 0.02]}
        crit["zc_above_overhang_top_by_0.25"] = top is not None and z_c >= top + 0.25
        higher = zb[(zb > (top or 0) + 0.5)]
        ceil_side = float(higher.min()) if len(higher) else g0["ceiling_height_m"]
        notes["ceiling_side_obstacle_above_floor"] = ceil_side
        crit["zc_band_below_ceiling_side_by_0.30"] = z_c + 0.10 <= ceil_side - 0.30
    for key in ("sweeper", "cylinder", "uav"):
        r = robots[key]
        dist = _free_dist(scene, z_f, (r.z_lo, r.z_hi), ext)
        for tag, p in (("start", start), ("goal", goal)):
            i = int((p[0] - ext[0]) / CELL); j = int((p[1] - ext[1]) / CELL)
            crit[f"{tag}_clear_0.5m_{key}"] = bool(dist[i, j] >= 0.5)
    dist_c = _free_dist(scene, z_f, (robots["cylinder"].z_lo, robots["cylinder"].z_hi), ext)
    lab, _ = ndimage.label(dist_c > robots["cylinder"].max_radius())
    li = lambda p: lab[int((p[0] - ext[0]) / CELL), int((p[1] - ext[1]) / CELL)]
    crit["cylinder_detour_plausible_raster"] = bool(li(start) != 0 and li(start) == li(goal))
    crit["not_through_glass_adjacent_opening"] = glass_clear == "yes"
    case = {"window": ext, "start": list(start), "goal": list(goal), "z_floor": z_f,
            "z_c": z_c, "overhang": notes.get("overhang"), "criteria": crit,
            "pass": all(crit.values()), "notes": notes,
            "gravity_rotation": g0["gravity_rotation"]}
    (RES / "case.json").write_text(json.dumps(case, indent=2))
    fig, ax = plt.subplots(1, 2, figsize=(16, 6))
    occ = _occupancy(scene, z_f, (0.02, 1.75), ext)
    ax[0].imshow(occ.T, origin="lower", cmap="Greys", extent=[ext[0], ext[2], ext[1], ext[3]])
    ax[0].plot([start[0], goal[0]], [start[1], goal[1]], "r-")
    ax[0].plot(*start[:2], "go"); ax[0].plot(*goal[:2], "bs"); ax[0].set_title("case window, band 0.02-1.75")
    ax[1].vlines(s, zb, zt, lw=0.3, color="k", alpha=0.3)
    for key, colr in (("sweeper", "g"), ("cylinder", "r"), ("uav", "b")):
        r = robots[key]
        ax[1].axhspan(r.z_lo, r.z_hi, color=colr, alpha=0.12, label=key)
    ax[1].legend(); ax[1].set_title("section along start->goal with robot bands")
    plt.savefig(FIGS / "case_overview.png", dpi=120); plt.close()
    print(json.dumps(case, indent=2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", required=True, choices=["copy", "decode", "maps", "section", "case"])
    ap.add_argument("--seg"); ap.add_argument("--name")
    ap.add_argument("--window"); ap.add_argument("--start"); ap.add_argument("--goal")
    ap.add_argument("--zc", type=float); ap.add_argument("--glass-clear", choices=["yes", "no"])
    a = ap.parse_args()
    f = lambda v: [float(x) for x in v.split(",")]
    if a.step == "copy":
        step_copy()
    elif a.step == "decode":
        step_decode()
    elif a.step == "maps":
        step_maps()
    elif a.step == "section":
        step_section(f(a.seg), a.name)
    else:
        step_case(f(a.window), f(a.start), f(a.goal), a.zc, a.glass_clear)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Copy and decode** (inside the A2 allocation; ~1 GB RAM per million splats)

Run: `cd gmc && PYTHONPATH=src:experiments $PY experiments/showcase_scene.py --step copy && PYTHONPATH=src:experiments $PY experiments/showcase_scene.py --step decode`
Expected: sha256 equal; `g0.json` written with `floor.tilt_deg`, `ceiling_height_m`, crop counts,
floor-splat top percentiles. Append all numbers to the worklog. If
`floor_splats_reaching_z_lo_0.02` is large, record it as a finding (spec §5.3); do not change `z_lo`.

- [ ] **Step 3: Band maps — then look at them**

Run: `cd gmc && PYTHONPATH=src:experiments $PY experiments/showcase_scene.py --step maps`
Open every `figs/band_*.png` and `figs/overhang_candidates.png` with the image reader. Write
in the worklog what you see: walls, the reception counter, plinths, benches, glass doors, where
the overhang candidates cluster.

- [ ] **Step 4: Scale check**

Draw sections with `--step section --seg x0,y0,x1,y1 --name <name>` through: the counter,
a doorway, one bench or plinth, and the room (floor to ceiling). Open each PNG. Record in
`gmc/results/height/showcase/scale_check.json`:
`{"measurements": [{"object": "...", "measured_m": v, "typical_m": [lo, hi], "figure": "figs/section_<name>.png", "agrees_within_15pct": bool}], "metric_accepted": bool}`.
Typical ranges: counter top 0.90–1.10, desk/table top 0.70–0.78, bench/chair seat 0.40–0.50,
door height 2.00–2.40. `metric_accepted` is true iff at least two measurements agree within 15%.
If false: commit what you have, write the standup, and **stop** — G1 does not run (spec §5.3).

- [ ] **Step 5: Choose the case and validate it**

Pick a window (≤ 8 m × 8 m), start, goal and `z_c` from the maps and sections. The counter first;
another overhang if the counter has no knee space from the approach side. Then run:

`cd gmc && PYTHONPATH=src:experiments $PY experiments/showcase_scene.py --step case --window xmin,ymin,xmax,ymax --start x,y,0 --goal x,y,0 --zc Z --glass-clear yes`

Pass `--glass-clear yes` only after checking the maps and sections for glass-adjacent openings on
the straight line and on the plausible detour; write the evidence in `case.json.notes` by hand if
needed. Open `figs/case_overview.png`. Iterate positions until `pass` is true. If no position
satisfies `straight_line_under_overhang`, record that plainly, keep the best case, and continue
(spec §5.3).

- [ ] **Step 6: Commit**

```bash
git add gmc/experiments/showcase_scene.py gmc/results/height/showcase/
git commit -m "[height T10] showcase G0: copy, floor/scale checks, band maps, case"   # plus the trailer
```

### Task 11: `showcase_run.py` — G1 per-robot jobs with probe, G2 replay

**Files:**
- Create: `gmc/experiments/showcase_run.py`, `gmc/configs/height_showcase.yaml`, `gmc/hpc/height_showcase_robot.sbatch`
- Output: `gmc/results/height/showcase/<robot>[_tau0.1][_b<k>].json`; raw runs under
  `/scratch/wg2381/splathjb/gmc/outputs/height/showcase/<run_name>/`

**Interfaces:**
- Consumes: `load_processed` (Task 10), `case.json`, `project_scene`, `robot_table`, `compile_and_query`,
  `with_overrides`, `replay_curve`, `curve_from_dict`.
- Produces: result JSON keys `robot, tau, budget_scale, projection` (stats), `probe`
  (`{n_supports_full, n_supports_probe, probe_compile_seconds, projected_hours, abort}`), `result` (Task 7 dict, or null if aborted),
  `replay3d` (Task 5 dict or null), `case` (copy of case.json).

- [ ] **Step 1: Create the config**

`configs/height_showcase.yaml` = `configs/toy.yaml` with three changes and a header comment:
`orientation.initial_intervals` = `showcase_initial_intervals` from `results/height/toy/T2-8.json`;
`query.max_support_calls: 50_000_000`; `query.max_wall_seconds: 7200`. Everything else identical.

```bash
cd gmc
N=$($PY -c "import json;print(json.load(open('results/height/toy/T2-8.json'))['extra']['showcase_initial_intervals'])")
{ echo "# height showcase (spec §5.4): toy.yaml + initial_intervals from T2-8 (=${N}) + larger query budgets";
  sed -e "s/^  initial_intervals: .*/  initial_intervals: ${N}/" \
      -e "s/^  max_support_calls: .*/  max_support_calls: 50_000_000/" \
      -e "s/^  max_wall_seconds: .*/  max_wall_seconds: 7200/" configs/toy.yaml; } > configs/height_showcase.yaml
diff configs/toy.yaml configs/height_showcase.yaml
```

Expected diff: the header line and exactly those three value lines.

- [ ] **Step 2: Write the job body**

```python
# gmc/experiments/showcase_run.py
"""One robot on the showcase case: project, probe, compile+query, replay3d (spec §5.4)."""
import argparse
import json
import time
from pathlib import Path

import numpy as np

from gmc.config import load_config
from gmc.height.pathio import curve_from_dict
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.height.replay3d import replay_curve
from gmc.height.run import compile_and_query, with_overrides
from showcase_scene import RES, load_processed

RAW = Path("/scratch/wg2381/splathjb/gmc/outputs/height/showcase")
PROBE_HALF = 1.0          # probe window: 2 m x 2 m around the overhang midpoint
ABORT_HOURS = 20.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", required=True, choices=["cylinder", "sweeper", "uav"])
    ap.add_argument("--tau", type=float, default=0.3)
    ap.add_argument("--budget-scale", type=int, default=1)
    ap.add_argument("--skip-probe", action="store_true")
    a = ap.parse_args()
    name = a.robot + ("" if a.tau == 0.3 else f"_tau{a.tau:g}") + ("" if a.budget_scale == 1 else f"_b{a.budget_scale}")
    case = json.loads((RES / "case.json").read_text())
    scene3d, g0 = load_processed()
    robot = robot_table(case["z_c"])[a.robot]
    cfg = load_config("configs/height_showcase.yaml")
    k = a.budget_scale
    if k > 1:
        cfg = with_overrides(cfg, max_refinement_rounds=cfg.query.max_refinement_rounds * k,
                             max_wall_seconds=cfg.query.max_wall_seconds * k,
                             max_support_calls=cfg.query.max_support_calls * k)
    out = {"robot": a.robot, "tau": a.tau, "budget_scale": k, "case": case}
    s2, stats = project_scene(scene3d, robot, case["window"], z_floor=case["z_floor"], tau=a.tau)
    out["projection"] = stats
    print("projection", json.dumps(stats), flush=True)

    probe = {"n_supports_full": len(s2.supports), "abort": False}
    if not a.skip_probe:
        ov = case.get("overhang") or {}
        st, gl = np.array(case["start"][:2]), np.array(case["goal"][:2])
        mid_s = np.mean(ov["s_interval"]) if ov.get("s_interval") else 0.5 * np.linalg.norm(gl - st)
        c = st + (gl - st) / np.linalg.norm(gl - st) * mid_s
        pw = [c[0] - PROBE_HALF, c[1] - PROBE_HALF, c[0] + PROBE_HALF, c[1] + PROBE_HALF]
        sp, _ = project_scene(scene3d, robot, pw, z_floor=case["z_floor"], tau=a.tau)
        t0 = time.time()
        pcfg = with_overrides(cfg, initial_intervals=1, max_depth=0)
        compile_and_query(sp, robot, pcfg, (pw[0] + 0.3, c[1], 0.0), (pw[2] - 0.3, c[1], 0.0))
        dt = time.time() - t0
        n_int = cfg.orientation.initial_intervals
        ratio = len(s2.supports) / max(1, len(sp.supports))
        hours = dt * ratio * n_int / 3600.0
        probe.update(n_supports_probe=len(sp.supports), probe_compile_seconds=dt,
                     projected_hours=hours, extrapolation="linear in supports x initial_intervals (rough)",
                     abort=hours > ABORT_HOURS)
    out["probe"] = probe
    print("probe", json.dumps(probe), flush=True)
    if probe["abort"]:
        out["result"] = out["replay3d"] = None
    else:
        res, _ = compile_and_query(s2, robot, cfg, case["start"], case["goal"], out_dir=RAW / name)
        out["result"] = res
        out["replay3d"] = None
        if res["curve"] is not None:
            out["replay3d"] = replay_curve(scene3d, robot, curve_from_dict(res["curve"]),
                                           z_floor=case["z_floor"], tau=a.tau)
            res["replay3d"] = out["replay3d"]
    (RES / f"{name}.json").write_text(json.dumps(out, indent=2, default=str))
    r = out["result"] or {}
    print(f"=== {name} status={r.get('status')} verify={r.get('verify')} "
          f"replay3d={(out['replay3d'] or {}).get('passed')} abort={probe['abort']} ===", flush=True)


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Write the batch wrapper**

```bash
#!/bin/bash
# gmc/hpc/height_showcase_robot.sbatch -- submit with --export=ALL,ROBOT=<r>[,TAU=0.1][,BUDGET=2][,SKIP_PROBE=1]
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=height_showcase
#SBATCH --time=24:00:00
#SBATCH --mem=64G
#SBATCH --cpus-per-task=8
#SBATCH --output=/scratch/wg2381/splathjb-height/gmc/logs/%x-%j.out
# No --partition line on purpose (24 h jobs route to cs; see k2_door_gate_stage4.sbatch).
set -euo pipefail
: "${ROBOT:?set ROBOT}"
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb-height/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=8 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
ARGS=(--robot "${ROBOT}" --tau "${TAU:-0.3}" --budget-scale "${BUDGET:-1}")
[ "${SKIP_PROBE:-0}" = "1" ] && ARGS+=(--skip-probe)
echo "=== height_showcase ${ARGS[*]} $(date -Is) host=$(hostname) job=${SLURM_JOB_ID} ==="
$PY experiments/showcase_run.py "${ARGS[@]}"
echo "=== HEIGHT_SHOWCASE_DONE ${ROBOT} ==="
```

- [ ] **Step 4: Dry-check projection sizes inline, then submit the three jobs**

Inline (A2 allocation), for the record only:
`cd gmc && PYTHONPATH=src:experiments $PY -c "import json;from showcase_scene import RES,load_processed;from gmc.height.prism import robot_table;from gmc.height.project import project_scene;c=json.load(open(RES/'case.json'));s,g=load_processed();[print(k,project_scene(s,robot_table(c['z_c'])[k],c['window'],z_floor=c['z_floor'])[1]) for k in ('cylinder','sweeper','uav')]"`
Append the three stats lines to the worklog.

```bash
cd /scratch/wg2381/splathjb-height/gmc
sbatch --test-only --export=ALL,ROBOT=cylinder hpc/height_showcase_robot.sbatch
for R in cylinder sweeper uav; do
  J=$(sbatch --parsable --export=ALL,ROBOT=$R hpc/height_showcase_robot.sbatch)
  echo "$J height_showcase $R" >> /scratch/wg2381/claude_jobs/height/jobids/A2.txt
done
```

- [ ] **Step 5: Submit A3 gated on the three jobs**

```bash
IDS=$(awk '$2=="height_showcase"{print $1}' /scratch/wg2381/claude_jobs/height/jobids/A2.txt | paste -sd:)
J=$(sbatch --parsable --dependency=afterany:${IDS} /scratch/wg2381/claude_jobs/height/a3.slurm)
echo "$J agent_A3" >> /scratch/wg2381/claude_jobs/height/jobids/A2.txt
cat /scratch/wg2381/claude_jobs/height/jobids/A2.txt
```

- [ ] **Step 6: Commit**

```bash
git add gmc/experiments/showcase_run.py gmc/configs/height_showcase.yaml gmc/hpc/height_showcase_robot.sbatch
git commit -m "[height T11] showcase per-robot job with probe and 3D replay; G1 submitted"   # plus the trailer
```

---

## Stage A3 — showcase results, videos, report (Task 12)

### Task 12: collect G1, handle UNKNOWN/abort, G2 check, G3 videos and report

**Files:**
- Create: `gmc/experiments/height_viz.py`
- Output: `gmc/results/height/showcase/summary.json`, `gmc/results/height/showcase/media/*`
  (videos ≤ 20 MB and all PNG frames); large media and `showcase_paths.ply` under
  `/scratch/wg2381/splathjb/gmc/outputs/height/showcase/media/`;
  report sections appended to `docs/worklog/height_bands.md`; `docs/worklog/height_bands_zh.md`.

**Interfaces:**
- Consumes: result JSONs from Task 11; `load_processed`, `RES` (Task 10); `robot_video`, `comparison_video`,
  `write_scene_ply` (Task 8); `project_scene`, `robot_table`, `curve_from_dict`, `path_crosses`, `sample_curve`.
- Produces: `summary.json` with `rows` (one per run: `name, robot, tau, status, verify_certified,
  replay3d_passed, replay3d_clearance_lb, crosses_overhang_zone, video, frames`) and `hypotheses`
  (`sweeper_under, cylinder_around, uav_over`, each true/false/null with the rule used).

- [ ] **Step 1: Diagnose every G1 job from artifacts**

For each ID in `jobids/A2.txt` with purpose `height_showcase`:
`sacct -j <id> --format=JobID,State,Elapsed,ExitCode,MaxRSS`, read
`gmc/logs/height_showcase-<id>.out`, and check `gmc/results/height/showcase/<robot>.json`.
Classify each robot as one of: `DONE` (JSON with a result), `ABORT_PROBE` (JSON with
`probe.abort`), `TIMEOUT/FAILED` (no JSON). Write the table to the worklog.

- [ ] **Step 2: Follow-up submissions (only what Step 1 demands)**

- `ABORT_PROBE` or `TIMEOUT` for any robot: shrink the window symmetrically around the straight
  line, re-run `showcase_scene.py --step case` with the new window until `pass` is true again, commit
  the new `case.json` (the old one is kept in git history), then resubmit **all three** robots so they
  share one window. If no smaller window keeps the overhang and a detour, stop and escalate in the
  standup; do not change anything else. Coreset reduction (its outer guarantee is established in
  `docs/worklog/coreset_phase1.md`) is not implemented by this plan; if only a coreset would keep the
  case, escalate that choice to the user in the standup instead of improvising it.
- `DONE` with `status == "UNKNOWN"`: record `result.reason`, `safe_*`/`possible_*` counts, and the raw
  `result.json` report; resubmit that robot with `BUDGET=2 SKIP_PROBE=1`, and if still UNKNOWN later,
  `BUDGET=4`. Never change τ, ρ, the window, or the robot to resolve UNKNOWN.
- Always submit the sensitivity arm: `ROBOT=sweeper TAU=0.1 SKIP_PROBE=1`.

Every submission: `sbatch --parsable --export=ALL,ROBOT=…[,…] hpc/height_showcase_robot.sbatch`,
appended to `jobids/A3.txt` as `<id> height_showcase <robot> <variant>`. C2 waits for these.

- [ ] **Step 3: Write the G3 script**

```python
# gmc/experiments/height_viz.py
"""G3: videos, key frames, 3D PLY and summary from saved showcase results (spec §5.4, §6)."""
import argparse
import json
import shutil
from pathlib import Path

import numpy as np
from shapely.geometry import LineString

from gmc.height.pathio import curve_from_dict, path_crosses, sample_curve
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.height.viz import comparison_video, robot_video, write_scene_ply
from showcase_scene import RES, load_processed

MEDIA = RES / "media"
BIG = Path("/scratch/wg2381/splathjb/gmc/outputs/height/showcase/media")
COLOURS = {"cylinder": (230, 40, 40), "sweeper": (40, 200, 60), "uav": (40, 90, 230)}


def overhang_zone(case):
    ov = case.get("overhang") or {}
    if not ov.get("s_interval"):
        return None
    st, gl = np.array(case["start"][:2]), np.array(case["goal"][:2])
    u = (gl - st) / np.linalg.norm(gl - st)
    a, b = ov["s_interval"]
    return LineString([st + a * u, st + b * u]).buffer(0.2)


def _publish(path):
    if path is None:
        return None
    MEDIA.mkdir(parents=True, exist_ok=True)
    if Path(path).stat().st_size <= 20 * 1024 * 1024:
        dst = MEDIA / Path(path).name
        shutil.copyfile(path, dst)
        return str(dst)
    return str(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", default="cylinder,sweeper,uav")
    names = ap.parse_args().runs.split(",")
    case = json.loads((RES / "case.json").read_text())
    scene3d, _ = load_processed()
    zone = overhang_zone(case)
    rows, comp_by_robot, tracks = [], {}, []
    for name in names:
        p = RES / f"{name}.json"
        if not p.exists():
            rows.append({"name": name, "status": "MISSING"})
            continue
        d = json.loads(p.read_text())
        robot = robot_table(case["z_c"])[d["robot"]]
        s2, _ = project_scene(scene3d, robot, case["window"], z_floor=case["z_floor"], tau=d["tau"])
        res = d["result"] or {"status": "ABORT_PROBE", "curve": None}
        if d.get("replay3d"):
            res["replay3d"] = d["replay3d"]
        media = robot_video(s2, robot, res, BIG / name, scene3d=scene3d,
                            z_floor=case["z_floor"], title=name)
        frames = [_publish(f) for f in media["frames"]]
        crosses = None
        if res.get("curve") and zone is not None:
            crosses = path_crosses(curve_from_dict(res["curve"]), zone)
            tracks.append((robot, sample_curve(curve_from_dict(res["curve"]), 0.05,
                                               robot.max_radius()), COLOURS[d["robot"]]))
        rep = d.get("replay3d") or {}
        rows.append({"name": name, "robot": d["robot"], "tau": d["tau"],
                     "status": res["status"],
                     "verify_certified": (res.get("verify") or {}).get("certified"),
                     "replay3d_passed": rep.get("passed"),
                     "replay3d_clearance_lb": rep.get("min_clearance_lb"),
                     "crosses_overhang_zone": crosses,
                     "video": _publish(media["video"]), "frames": frames})
        if d["tau"] == 0.3:   # later --runs entries (e.g. budget reruns) replace earlier ones
            comp_by_robot[d["robot"]] = {"label": name, "scene2d": s2, "robot": robot, "result": res}
    comp = list(comp_by_robot.values())
    if comp:
        cv = comparison_video(comp, BIG / "compare_three_robots", scene3d=scene3d,
                              z_floor=case["z_floor"])
        compare = {"video": _publish(cv["video"]), "frames": [_publish(f) for f in cv["frames"]]}
    else:
        compare = None
    BIG.mkdir(parents=True, exist_ok=True)
    n_ply = write_scene_ply(BIG / "showcase_paths.ply", scene3d, case["window"],
                            case["z_floor"], tracks)

    def ok(r):
        return (r.get("status") == "REACHABLE" and r.get("verify_certified")
                and r.get("replay3d_passed"))
    by = {r.get("name"): r for r in rows}
    hyp = {}
    for key, want_cross in (("sweeper", True), ("cylinder", False), ("uav", True)):
        r = by.get(key)
        if r is None or r.get("status") in (None, "MISSING"):
            hyp[key] = None
        else:
            hyp[key] = bool(ok(r) and r.get("crosses_overhang_zone") is want_cross)
    summary = {"rows": rows, "compare": compare,
               "ply": {"path": str(BIG / "showcase_paths.ply"), "vertices": n_ply},
               "hypotheses": {"sweeper_under": hyp["sweeper"], "cylinder_around": hyp["cylinder"],
                              "uav_over": hyp["uav"],
                              "rule": "certified (REACHABLE + verify + replay3d) and path meets / avoids the 0.2 m buffer of the overhang interval on the straight line"},
               "case": case}
    (RES / "summary.json").write_text(json.dumps(summary, indent=2, default=str))
    for r in rows:
        print(r)
    print(summary["hypotheses"])


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Render — visualize before metrics**

Run: `cd gmc && PYTHONPATH=src:experiments $PY experiments/height_viz.py --runs <names of every result JSON present>`
Then open **every** PNG under `gmc/results/height/showcase/media/` with the image reader. For each
robot write in the worklog, before any number: where the footprint goes relative to the counter
(or chosen overhang), what the side panel shows between the robot band and the splat mass, and
whether that matches the status. A mismatch between picture and verdict is a bug to investigate
(superpowers:systematic-debugging), not a caption to write.

- [ ] **Step 5: Write the report**

Append to `docs/worklog/height_bands.md` a `## Results` section with: the toy gate table (all
gates, pass/fail, one evidence number each); the showcase table from `summary.json`; the
hypotheses with their rule; projection stats per robot (kept, excluded_band, dropped_opacity,
deduplicated, floored); probe numbers; τ = 0.1 comparison for the sweeper; scale check; floor
finding; and the claims boundary from spec §3.5 verbatim in substance (prototype certificates,
scene definition, unaudited glass/holes). Every number cites its JSON file.

Write `docs/worklog/height_bands_zh.md`: a Chinese summary of the same (≤ 1 page): 做了什么、每个机器人
是否跑通、是否从桌/台下穿过或绕行或飞越、视频路径、局限。

- [ ] **Step 6: Commit and standup**

```bash
git add gmc/experiments/height_viz.py gmc/results/height/showcase/ docs/worklog/height_bands.md docs/worklog/height_bands_zh.md
git commit -m "[height T12] showcase G2/G3: summary, videos, PLY, report"   # plus the trailer
```

Standup `/scratch/wg2381/claude_jobs/logs/height_A3_done.md`: job IDs and states, what is done,
what was submitted and is still running (C2 will finish those), commit hashes.

---

## Chain infrastructure — written and submitted by the interactive session (Tasks 13–14)

### Task 13: agent wrapper, stage scripts, prompts

**Files (all under `/scratch/wg2381/claude_jobs/height/`):**
- Create: `run_agent.sh`, `a1.slurm`, `c1.slurm`, `a2.slurm`, `a3.slurm`, `c2.slurm`,
  `prompts/{a1,c1,a2,a3,c2}.md`, directories `state/`, `jobids/`, `counters/`

**Interfaces:**
- Each `.slurm` sets `STAGE, PROMPT, HOURS, WAIT_ON, SELF_SLURM[, EXTRA_ADD_DIRS]` and `exec`s `run_agent.sh`.
- `run_agent.sh` pre-check: if any job listed in `jobids/<s>.txt` for `s ∈ WAIT_ON` is still in `squeue`,
  resubmit `SELF_SLURM` with `--dependency=afterany:<those ids>` and exit 0 (at most 6 times per stage).

- [ ] **Step 1: Write `run_agent.sh`**

```bash
#!/bin/bash
# Generic unattended Claude stage for the height-band chain (spec §7).
# Env: STAGE PROMPT HOURS WAIT_ON SELF_SLURM [EXTRA_ADD_DIRS] [MAX_RESUBMIT]
set -uo pipefail
H=/scratch/wg2381/claude_jobs/height
WT=/scratch/wg2381/splathjb-height
LOGS=/scratch/wg2381/claude_jobs/logs
mkdir -p "$H/state" "$H/jobids" "$H/counters" "$LOGS"
echo "=== agent ${STAGE} start $(date -Is) host=$(hostname) job=${SLURM_JOB_ID} ==="

if [ -n "${WAIT_ON:-}" ]; then
  pending=""
  for s in $WAIT_ON; do
    f="$H/jobids/$s.txt"; [ -f "$f" ] || continue
    for j in $(awk '{print $1}' "$f"); do
      [ "$j" = "${SLURM_JOB_ID}" ] && continue
      if [ -n "$(squeue -h -j "$j" -o %T 2>/dev/null)" ]; then pending="${pending}:${j}"; fi
    done
  done
  if [ -n "$pending" ]; then
    n=$(cat "$H/counters/${STAGE}.resubmits" 2>/dev/null || echo 0)
    if [ "$n" -lt "${MAX_RESUBMIT:-6}" ]; then
      echo $((n + 1)) > "$H/counters/${STAGE}.resubmits"
      J=$(sbatch --parsable --dependency=afterany${pending} "${SELF_SLURM}")
      echo "$J agent_${STAGE}_resubmit" >> "$H/jobids/${STAGE}.txt"
      echo "still queued/running:${pending//:/ } -> resubmitted ${STAGE} as $J; exiting without starting the agent"
      exit 0
    fi
    echo "still waiting on${pending//:/ } after $n resubmissions; starting the agent anyway"
  fi
fi

cd "$WT" || { echo "worktree $WT missing"; exit 2; }
export PATH="$HOME/.local/bin:$PATH" PYTHONUNBUFFERED=1 MPLBACKEND=Agg
AGENT_LOG="$LOGS/height_${STAGE}-${SLURM_JOB_ID}.agent.log"
DEADLINE=$(( $(date +%s) + HOURS * 3600 - 1800 ))
ATTEMPT=0; RC=1
while : ; do
  ATTEMPT=$((ATTEMPT + 1))
  echo "--- ${STAGE} attempt ${ATTEMPT} $(date -Is) ---" | tee -a "$AGENT_LOG"
  # shellcheck disable=SC2086
  claude -p "$(cat "$PROMPT")" --model claude-opus-5 --dangerously-skip-permissions \
    --add-dir "$WT" --add-dir /scratch/wg2381/splathjb --add-dir "$H" --add-dir "$LOGS" \
    ${EXTRA_ADD_DIRS:-} 2>&1 | tee -a "$AGENT_LOG"
  RC=${PIPESTATUS[0]}
  [ "$RC" -eq 0 ] && break
  if ! tail -n 200 "$AGENT_LOG" | grep -qi "session limit\|usage limit"; then
    echo "rc=${RC} for a reason other than the usage limit -- not retrying"; break
  fi
  if [ "$(date +%s)" -ge "$DEADLINE" ]; then
    echo "usage limit still in force and no allocation left; giving up after ${ATTEMPT} attempts"; break
  fi
  echo "usage limit hit; sleeping 20 min (deadline $(date -d @${DEADLINE} -Is))"
  sleep 1200
done
echo "=== agent ${STAGE} exit rc=${RC} $(date -Is) ==="
git -C "$WT" log --oneline -6
git -C "$WT" status --short | head -20
cat "$H/state/${STAGE}.json" 2>/dev/null || echo "(no state file)"
exit "$RC"
```

- [ ] **Step 2: Write the five stage scripts**

All five share this header; only the job name and the `export` line differ.

```bash
#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --mem=48G
#SBATCH --time=06:00:00
#SBATCH --job-name=agent_height_a1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
#SBATCH --error=/scratch/wg2381/claude_jobs/logs/%x-%j.err
H=/scratch/wg2381/claude_jobs/height
export STAGE=A1 PROMPT=$H/prompts/a1.md HOURS=6 WAIT_ON="" SELF_SLURM=$H/a1.slurm
exec bash $H/run_agent.sh
```

| file | `--job-name` | export line |
|---|---|---|
| `a1.slurm` | `agent_height_a1` | `export STAGE=A1 PROMPT=$H/prompts/a1.md HOURS=6 WAIT_ON="" SELF_SLURM=$H/a1.slurm` |
| `c1.slurm` | `agent_height_c1` | `export STAGE=C1 PROMPT=$H/prompts/c1.md HOURS=6 WAIT_ON="A1" SELF_SLURM=$H/c1.slurm` |
| `a2.slurm` | `agent_height_a2` | `export STAGE=A2 PROMPT=$H/prompts/a2.md HOURS=6 WAIT_ON="A1 C1" SELF_SLURM=$H/a2.slurm EXTRA_ADD_DIRS="--add-dir /scratch/sy2366/Project/LccStudio-stage-archive"` |
| `a3.slurm` | `agent_height_a3` | `export STAGE=A3 PROMPT=$H/prompts/a3.md HOURS=6 WAIT_ON="A2" SELF_SLURM=$H/a3.slurm` |
| `c2.slurm` | `agent_height_c2` | `export STAGE=C2 PROMPT=$H/prompts/c2.md HOURS=6 WAIT_ON="A1 C1 A2 A3" SELF_SLURM=$H/c2.slurm` |

- [ ] **Step 3: Write the shared rules block and the five prompts**

`prompts/_rules.md` (each prompt below starts by telling the agent to read it):

```markdown
# Rules for every height-band agent stage

You run unattended inside a Slurm CPU allocation on NYU Torch. No TTY, nobody to ask.
You may be a retry after a usage-limit death: read your state file first and resume.

Read in full before acting:
1. /scratch/wg2381/splathjb-height/docs/superpowers/specs/2026-09-12-height-band-robots-design.md
2. /scratch/wg2381/splathjb-height/docs/superpowers/plans/2026-09-12-height-band-robots.md
   (its "Global Constraints" and "Stage bookkeeping" sections bind you)
3. /scratch/wg2381/splathjb-height/README.md (meta-rules: visualize before metrics; a verified
   list is not a completeness proof; challenge verified items under pressure)

Hard rules:
- Work only in /scratch/wg2381/splathjb-height (branch height-bands). Never push. Never switch
  branches. /scratch/wg2381/splathjb is read-only for you except gmc/outputs/height/ and
  splatc_atlas/data/gs_scenes/showcase/.
- /scratch/sy2366/... is read-only. Write nothing outside /scratch/wg2381/.
- scancel only job IDs you appended to your own jobids file. Never --all, never wildcards.
- Install nothing. Do not edit the read-only code listed in the plan's Global Constraints.
- Never loosen a gate, never change tau/rho/filters/robots/window to rescue a run.
- Heavy compute (> ~30 min or > 8 CPU) goes through sbatch with the plan's wrappers.
- Append problems and resolutions to docs/worklog/height_bands.md as they happen.
- Update /scratch/wg2381/claude_jobs/height/state/<STAGE>.json after every task.
- End by writing /scratch/wg2381/claude_jobs/logs/height_<STAGE>_done.md: what is done (with
  commit hashes), what failed (with evidence paths), what is still running (job IDs), and what
  the next stage must do. Write it even if you finished nothing.
- Exit code: a predecessor recorded as FAILED 1:0 may have finished its work before dying on a
  usage limit. Judge predecessors by artifacts, never by exit codes.
```

`prompts/a1.md`:

```markdown
# Stage A1 — height package and toy gates
STAGE=A1. First read /scratch/wg2381/claude_jobs/height/prompts/_rules.md and everything it lists.
Then read state/A1.json (may not exist).

Do plan Tasks 1–9 in order, TDD exactly as written, one commit per task. Run tests with
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python from gmc/ with PYTHONPATH=src:experiments.
If a planned test fails for a reason the plan did not foresee, use superpowers:systematic-debugging;
fix the new height code, never the test's intent and never the read-only core.

Task 9 ends by submitting eight toy gate jobs; you do not wait for them. C1 collects them.
Before exiting, run: PYTHONPATH=src:experiments $PY -m pytest -q tests/unit/test_height_*.py tests/integration/test_height_table.py
and put the summary line in your standup.
```

`prompts/c1.md`:

```markdown
# Stage C1 — check A1 and close the toy gates
STAGE=C1. First read prompts/_rules.md and everything it lists, then state/A1.json, state/C1.json,
jobids/A1.txt, logs/height_A1_done.md (may be missing), and the A1 agent log(s) in
/scratch/wg2381/claude_jobs/logs/height_A1-*.agent.log.

1. Diagnose A1 from artifacts: for Tasks 1–9, does the commit exist, do the files match the plan,
   do the tests pass now? Finish any incomplete task exactly as the plan specifies.
2. Every toy gate job in jobids/A1.txt has ended (your wrapper waited). For each gate
   T1a T1b T2-1 … T2-8 check gmc/results/height/toy/<gate>.json exists and read `pass` and `criteria`.
   Missing JSON: read gmc/logs/height_toy-<id>.out and sacct; resubmit that gate once with
   hpc/height_toy.sbatch (append to jobids/C1.txt). A2 waits for your jobs.
   `pass: false`: investigate with superpowers:systematic-debugging. A bug in new height code or in
   synth3d geometry may be fixed (commit, rerun the gate by sbatch); the gate criteria may not change.
   If the failure is real, record it — do not massage it.
3. Open the key frames (T1a, T2-1, T2-4, T2-5 under gmc/results/height/toy/media/) and describe in
   the worklog what each shows.
4. Run the full suite: cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q  (expect the prior 353
   plus the new tests, all passing). Put the summary line in the worklog and standup.
5. Append a "Toy gates" table to docs/worklog/height_bands.md, commit, write logs/height_C1_done.md.
```

`prompts/a2.md`:

```markdown
# Stage A2 — showcase scene and compute submission
STAGE=A2. First read prompts/_rules.md and everything it lists, then logs/height_C1_done.md,
state/A2.json, and the toy gate JSONs.

If any toy gate is not `pass: true`, or C1's standup reports an unresolved failure: do not start the
showcase. Write the reason to the worklog and logs/height_A2_done.md and exit.

Otherwise do plan Tasks 10–11. /scratch/sy2366/Project/LccStudio-stage-archive/ is readable for the
copy only. Task 10 Step 4 may stop the chain (scale not metric) — if so, still write the standup.
Task 11 ends by submitting the three robot jobs and then A3 (a3.slurm) with afterany on them; record
all IDs in jobids/A2.txt. Do not wait for them.
```

`prompts/a3.md`:

```markdown
# Stage A3 — showcase results, videos, report
STAGE=A3. First read prompts/_rules.md and everything it lists, then logs/height_A2_done.md,
state/A3.json, jobids/A2.txt, gmc/results/height/showcase/case.json.

Do plan Task 12. Render and look at every frame before writing numbers. Follow-up jobs you submit go
in jobids/A3.txt; you do not wait for them — C2 will. Render G3 for whatever is complete now.
```

`prompts/c2.md`:

```markdown
# Stage C2 — final check: every task complete
STAGE=C2. First read prompts/_rules.md and everything it lists. Then read every standup
logs/height_{A1,C1,A2,A3}_done.md (any may be missing), every state/*.json, every jobids/*.txt, and
the agent logs logs/height_*-*.agent.log. Your wrapper has already waited for all listed jobs.

1. Build a checklist from the plan: Tasks 1–12 and every gate (T1a…T2-8, G0 scale check and case,
   G1 three robots, G2 replay3d, G3 videos/PLY/summary, report, zh summary). For each, record
   done / failed-with-evidence / missing, judged from artifacts and commits only.
2. Finish everything missing, in plan order, exactly as the plan specifies. That includes Task 12 if A3
   never ran or died, re-running height_viz.py for results that landed after A3, and collecting the
   UNKNOWN-budget and tau=0.1 follow-ups. If you must submit compute, append to jobids/C2.txt and, as
   your final action, resubmit c2.slurm with --dependency=afterany on those jobs (the wrapper's
   resubmission counter still applies).
3. Open every showcase frame and the comparison frame; check each against summary.json. Any
   picture/verdict mismatch is investigated before the report is final.
4. Run the full test suite once more; record the summary line.
5. Write logs/height_C2_done.md as the user-facing summary: Chinese first (≤ 1 page: 每个机器人是否跑通，
   扫地机是否从台/桌下穿过，柱状机器人是否绕行，UAV 是否飞越，视频和 PLY 路径，局限), then an English table of
   gates and key numbers with file paths. Commit remaining work.
```

- [ ] **Step 4: Validate the scripts**

```bash
H=/scratch/wg2381/claude_jobs/height
bash -n $H/run_agent.sh && echo "syntax ok"
for s in a1 c1 a2 a3 c2; do sbatch --test-only $H/$s.slurm; done
```

Expected: `syntax ok` and five `sbatch: Job ... to start at ...` lines, no errors.

### Task 14: worktree, submission, hand-off (interactive session only)

- [ ] **Step 1: Commit the plan on the current branch, then create the worktree from it**

```bash
cd /scratch/wg2381/splathjb
git add docs/superpowers/plans/2026-09-12-height-band-robots.md
git commit -m "Implementation plan for height-band robots and the showcase agent chain"   # plus the trailer
git worktree add /scratch/wg2381/splathjb-height -b height-bands HEAD
git -C /scratch/wg2381/splathjb-height log --oneline -2
```

- [ ] **Step 2: Tell the user exactly what will be submitted, and wait for their go**

State per job: name, partition, CPU, memory, wall time, dependency, `--begin`; the pre-approved compute
envelope (toy gates: 8 × 4 CPU/16 G/12 h; showcase: 3–5 × 8 CPU/64 G/24 h, at most 3 at once plus
follow-ups); and ask for `--begin` times aligned with their usage-limit refresh. Ask whether to install
`imageio-ffmpeg` into `gmc-venv` for MP4 (otherwise GIF). Do not submit before an explicit yes.

- [ ] **Step 3: Submit the pre-gated chain**

```bash
H=/scratch/wg2381/claude_jobs/height
A1=$(sbatch --parsable ${BEGIN_A1:+--begin=$BEGIN_A1} $H/a1.slurm)
C1=$(sbatch --parsable --dependency=afterany:$A1 ${BEGIN_C1:+--begin=$BEGIN_C1} $H/c1.slurm)
A2=$(sbatch --parsable --dependency=afterany:$C1 ${BEGIN_A2:+--begin=$BEGIN_A2} $H/a2.slurm)
C2=$(sbatch --parsable --dependency=afterany:$A2 ${BEGIN_C2:+--begin=$BEGIN_C2} $H/c2.slurm)
printf "%s agent_A1\n%s agent_C1\n%s agent_A2\n%s agent_C2\n" $A1 $C1 $A2 $C2 | tee $H/jobids/session.txt
squeue -u wg2381 -o "%.10i %.20j %.10T %.20R %.20S"
```

- [ ] **Step 4: Hand off**

Report the four job IDs, where standups and the final summary will appear
(`/scratch/wg2381/claude_jobs/logs/height_*_done.md`), and how to check progress
(`squeue -u wg2381`, `sacct -j <ids>`). `scancel` from this session only these four IDs, and only if asked.
