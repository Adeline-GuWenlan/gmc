"""M0-lite adapter for the K2 GS scene: decode -> clean -> calibrate -> SE(2) slab.

Pipeline (each stage logged with counts; see worklog gmc_M0_k2.md):
  1. parse raw 3DGS PLY (17 float32 props, SH deg 0), decode sigmoid/exp/quat
  2. junk filter: opacity < OP_JUNK, giant splats (max scale > SCALE_MAX)
  3. crop to building mass (axis-aligned box, original frame is kept —
     estimated building yaw is recorded in meta, not applied)
  4. voxel density filter: drop splats in near-empty voxels (free-space floaters)
  5. floor / ceiling z from histogram modes of near-horizontal structure
  6. z-slab conditioning: for robot slab [z0+SLAB_LO, z0+SLAB_HI], per splat take
     max-weight height among K samples; keep conditional 2D Gaussian + weight.
     Pilot semantics — the certified slab union comes with M2, not here.

Outputs under data/gs_scenes/k2/:
  processed_full.npz   cleaned 3D splats (mean, cov6, opacity)
  slab_2d.npz          2D conditional Gaussians (mean2, cov3, weight)
  meta.json            provenance (raw sha256), stage counts, calibration
  figs/                before/after + slab ellipse renders

Scale note: units are UNCALIBRATED. Internal evidence (ceiling 5.4 u, desk 1.7 u,
wall openings 1.6-2 u) favors 1 u ~= 0.5 m; recorded as hypothesis in meta.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2] / "data" / "gs_scenes" / "k2"
RAW = ROOT / "raw" / "point_cloud.ply"
FIGS = ROOT / "figs"

OP_JUNK = 0.05
SCALE_MAX = 1.0
CROP = {"x": (1.5, 43.0), "y": (-29.0, 5.0), "z": (-3.0, 8.0)}
VOXEL = 0.5
VOXEL_MIN = 3          # splats per voxel below which -> floater
SLAB_LO, SLAB_HI = 0.1, 1.6   # robot slab above floor, scene units (~0.05-0.8 m)
SLAB_K = 6             # conditioning heights sampled in the slab
W_MIN = 1e-3           # keep conditional splat if best weight above this


def load_raw():
    with open(RAW, "rb") as f:
        header = b""
        while not header.endswith(b"end_header\n"):
            header += f.readline()
        data = np.fromfile(f, dtype=np.float32).reshape(-1, 17)
    xyz = data[:, 0:3].astype(np.float64)
    op = 1.0 / (1.0 + np.exp(-data[:, 9].astype(np.float64)))
    sc = np.exp(data[:, 10:13].astype(np.float64))
    q = data[:, 13:17].astype(np.float64)          # (w, x, y, z)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return xyz, op, sc, q


def quat_to_rot(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - w * z); R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y); R[:, 2, 1] = 2 * (y * z + w * x); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def covariances(sc, q):
    R = quat_to_rot(q)
    return np.einsum("nij,nj,nkj->nik", R, sc ** 2, R)


def cov_pack6(C):
    return np.stack([C[:, 0, 0], C[:, 0, 1], C[:, 0, 2],
                     C[:, 1, 1], C[:, 1, 2], C[:, 2, 2]], axis=1)


def voxel_density_keep(xyz):
    ijk = np.floor(xyz / VOXEL).astype(np.int64)
    key = (ijk[:, 0] << 42) ^ (ijk[:, 1] << 21) ^ ijk[:, 2]
    _, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    return cnt[inv] >= VOXEL_MIN


def estimate_building_yaw(xyz, op, cov, z0):
    """Dominant wall azimuth mod 90 deg, from elongated slab splats."""
    m = (op > 0.5) & (xyz[:, 2] > z0 + 0.3) & (xyz[:, 2] < z0 + 4.0)
    C2 = cov[m][:, :2, :2]
    evals, evecs = np.linalg.eigh(C2)
    elong = evals[:, 1] / np.maximum(evals[:, 0], 1e-12)
    sel = elong > 9.0
    v = evecs[sel][:, :, 1]
    ang = np.arctan2(v[:, 1], v[:, 0]) % (np.pi / 2)
    hist, edges = np.histogram(ang, bins=180)
    return float(np.rad2deg(edges[np.argmax(hist)] + (edges[1] - edges[0]) / 2))


def slab_condition(xyz, op, cov, z0):
    hs = z0 + np.linspace(SLAB_LO, SLAB_HI, SLAB_K)
    mu_z = xyz[:, 2]
    s_zz = cov[:, 2, 2]
    best_w = np.zeros(len(xyz))
    best_h = np.zeros(len(xyz))
    for h in hs:
        w = op * np.exp(-0.5 * (h - mu_z) ** 2 / s_zz)
        upd = w > best_w
        best_w[upd] = w[upd]
        best_h[upd] = h
    keep = best_w > W_MIN
    xyzk, covk, hk, wk = xyz[keep], cov[keep], best_h[keep], best_w[keep]
    s_xz = covk[:, :2, 2]
    s_zzk = covk[:, 2, 2]
    mu2 = xyzk[:, :2] + s_xz * ((hk - xyzk[:, 2]) / s_zzk)[:, None]
    cov2 = covk[:, :2, :2] - np.einsum("ni,nj->nij", s_xz, s_xz) / s_zzk[:, None, None]
    return keep, mu2, cov2, wk


def render_topdown(ax, pts, w, extent, bins=700, title=""):
    import matplotlib
    ax.hist2d(pts[:, 0], pts[:, 1], weights=w, bins=bins,
              range=[extent[:2], extent[2:]], norm=matplotlib.colors.LogNorm(),
              cmap="gray_r")
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)


def render_window_ellipses(ax, mu2, cov2, w, center, half=3.0, rho=2.0, wmin=0.3):
    from matplotlib.patches import Ellipse
    m = (np.abs(mu2[:, 0] - center[0]) < half) & (np.abs(mu2[:, 1] - center[1]) < half) & (w > wmin)
    evals, evecs = np.linalg.eigh(cov2[m])
    for k in np.argsort(w[m]):
        ang = np.rad2deg(np.arctan2(evecs[k, 1, 1], evecs[k, 0, 1]))
        e = Ellipse(mu2[m][k], 2 * rho * np.sqrt(evals[k, 1]), 2 * rho * np.sqrt(evals[k, 0]),
                    angle=ang, alpha=min(0.75, 0.15 + 0.6 * w[m][k]), facecolor="tab:blue",
                    edgecolor="none")
        ax.add_patch(e)
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_aspect("equal")
    ax.set_title(f"conditional 2D splats (rho={rho} support), window @ {center}", fontsize=10)
    return int(m.sum())


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    FIGS.mkdir(parents=True, exist_ok=True)
    sha = hashlib.sha256(RAW.read_bytes()).hexdigest()
    stages = {}

    xyz, op, sc, q = load_raw()
    stages["raw"] = len(xyz)

    keep = (op >= OP_JUNK) & (sc.max(axis=1) <= SCALE_MAX)
    stages["after_junk_filter"] = int(keep.sum())
    xyz, op, sc, q = xyz[keep], op[keep], sc[keep], q[keep]

    keep = np.ones(len(xyz), bool)
    for i, ax_ in enumerate("xyz"):
        lo, hi = CROP[ax_]
        keep &= (xyz[:, i] >= lo) & (xyz[:, i] <= hi)
    stages["after_crop"] = int(keep.sum())
    xyz, op, sc, q = xyz[keep], op[keep], sc[keep], q[keep]

    pre_floater = xyz.copy()
    keep = voxel_density_keep(xyz)
    stages["after_floater_filter"] = int(keep.sum())
    xyz, op, sc, q = xyz[keep], op[keep], sc[keep], q[keep]

    cov = covariances(sc, q)

    zh, ze = np.histogram(xyz[op > 0.5][:, 2], bins=400, range=(-2.5, 6.0))
    zc = (ze[:-1] + ze[1:]) / 2
    # floor = strongest band well below the furniture mass; ceiling analogous
    mf = (zc > -2.0) & (zc < -0.5)
    z0 = float(zc[mf][np.argmax(zh[mf])])
    mc = (zc > 3.0) & (zc < 5.0)
    z_ceil = float(zc[mc][np.argmax(zh[mc])])
    yaw = estimate_building_yaw(xyz, op, cov, z0)

    keep_slab, mu2, cov2, w2 = slab_condition(xyz, op, cov, z0)
    stages["slab_2d_splats"] = int(keep_slab.sum())

    np.savez_compressed(ROOT / "processed_full.npz",
                        mean=xyz.astype(np.float32), cov6=cov_pack6(cov).astype(np.float32),
                        opacity=op.astype(np.float32))
    np.savez_compressed(ROOT / "slab_2d.npz",
                        mean2=mu2.astype(np.float32),
                        cov3=np.stack([cov2[:, 0, 0], cov2[:, 0, 1], cov2[:, 1, 1]], 1).astype(np.float32),
                        weight=w2.astype(np.float32))

    meta = {
        "raw_file": "raw/point_cloud.ply",
        "raw_sha256": sha,
        "source_note": "user-provided K2 scan (Telegram), original name 'point_cloud_1 (3).ply'",
        "stages": stages,
        "params": {"op_junk": OP_JUNK, "scale_max": SCALE_MAX, "crop": CROP,
                   "voxel": VOXEL, "voxel_min": VOXEL_MIN,
                   "slab": [SLAB_LO, SLAB_HI], "slab_k": SLAB_K, "w_min": W_MIN},
        "calibration": {
            "frame": "original PLY frame (no rotation applied)",
            "floor_z": z0, "ceiling_z": z_ceil,
            "building_yaw_deg_mod90": yaw,
            "scale_hypothesis": "1 unit ~= 0.5 m (ceiling 2.7 m, desk 0.85 m, "
                                "openings 0.8-1.0 m) — UNVERIFIED, needs a real "
                                "reference measurement",
        },
        "known_issues": [
            "wall gaps may include glass/scan holes (false-free) — wall-gap audit pending",
            "slab semantics are pilot-grade (max over K heights), certified union in M2",
            "low-opacity cumulative-solidity handling deferred to M0 calibration curve",
        ],
    }
    (ROOT / "meta.json").write_text(json.dumps(meta, indent=2))

    fig, axes = plt.subplots(1, 3, figsize=(24, 8.5))
    ext = [CROP["x"][0], CROP["x"][1], CROP["y"][0], CROP["y"][1]]
    m_pre = np.ones(len(pre_floater), bool)
    render_topdown(axes[0], pre_floater[m_pre], None, ext,
                   title=f"after junk+crop, n={stages['after_crop']}")
    render_topdown(axes[1], xyz, None, ext,
                   title=f"after floater filter, n={stages['after_floater_filter']}")
    render_topdown(axes[2], mu2, w2, ext,
                   title=f"SE(2) slab (weight-shaded), n={stages['slab_2d_splats']}, "
                         f"floor z0={z0:.2f}")
    fig.tight_layout()
    fig.savefig(FIGS / "pipeline_stages.png", dpi=100)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    n1 = render_window_ellipses(axes[0], mu2, cov2, w2, center=(22.3, -15.2))
    n2 = render_window_ellipses(axes[1], mu2, cov2, w2, center=(15.2, -11.0))
    fig.suptitle("door-window candidates: actual conditional 2D splat ellipses")
    fig.tight_layout()
    fig.savefig(FIGS / "door_windows.png", dpi=110)
    plt.close(fig)

    print(json.dumps({"stages": stages, "floor_z": z0, "ceiling_z": z_ceil,
                      "yaw_mod90": yaw, "win_splats": [n1, n2]}, indent=1))


if __name__ == "__main__":
    main()
