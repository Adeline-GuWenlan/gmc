"""bl B0 smoke test: SplatNav's Splat-Plan on a tiny synthetic Gaussian scene, on the GPU.

The repo's trained scenes (flight/statues/stonehenge/old_union) live on Google Drive and are not fetched. This runs
the same calls as `run_splatplan.py` (SplatPlan(gsplat, robot_config, voxel_config, SplinePlanner(spline_deg=6)),
then `generate_path(x0, xf)` per query), but the `gsplat` object is a duck-typed stand-in with exactly the fields
the repo's own `DummyGSplatLoader` / `load_gsplat_from_json` set (means, rots [wxyz], scales, covs, covs_inv,
colors, opacities) -- `splat/splat_utils.py` itself imports nerfstudio, which this env deliberately does not have.

Scene: a wall of Gaussians across x = 0 with one gap, plus a few pillars, inside a 2 x 2 x 0.6 m box. Robot: sphere,
radius 0.05; vmax = amax = 0.1 as in run_splatplan.py. Independent check: every trajectory sample must lie outside
every Gaussian's `scales` ellipsoid (Splat-Plan's obstacle) inflated by the robot radius; we report the smallest
clearance to the ellipsoid surfaces measured on a dense surface sample.

Usage (splatnav env, GPU node): python -B bl_smoke_splatnav.py --repo .../splatnav --out DIR
"""
import argparse
import json
import os
import sys
import time

sys.dont_write_bytecode = True

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402


class SyntheticGSplat:
    """Same attributes as splat.splat_utils.DummyGSplatLoader.initialize_attributes (+ covs_inv, opacities)."""

    def __init__(self, means, rots, scales, device, compute_cov):
        self.device = device
        self.means = means.to(device)
        self.rots = rots.to(device)
        self.scales = scales.to(device)
        self.covs_inv = compute_cov(self.rots, 1.0 / self.scales)
        self.covs = compute_cov(self.rots, self.scales)
        self.colors = 0.5 * torch.ones(means.shape[0], 3, device=device)
        self.opacities = torch.ones(means.shape[0], device=device)


def build_scene(rng):
    means, scales, quats = [], [], []
    # wall across x = 0, y in [-1, 1], gap y in [0.25, 0.65]; three layers in z to fill 0..0.6
    for y in np.arange(-0.95, 0.96, 0.1):
        if 0.25 < y < 0.65:
            continue
        for z in (0.1, 0.3, 0.5):
            # world extents (0.04, 0.07, 0.12) written as a 90-degree yaw of local (0.07, 0.04, 0.12): the repo's
            # quaternion_to_rotation_matrix returns NaN for an exact identity quaternion (zero vector part), which
            # silently removed this wall from the planner in job 19506846 (kept under splatnav_invalid_identity_quat/)
            means.append([0.0, y, z]); scales.append([0.07, 0.04, 0.12])
            quats.append([np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)])
    # pillars (rotated ellipsoids)
    for (x, y) in [(-0.5, 0.0), (0.5, -0.4), (0.45, 0.55), (-0.45, -0.6)]:
        for z in (0.15, 0.45):
            ang = rng.uniform(0, np.pi)
            means.append([x, y, z]); scales.append([0.12, 0.05, 0.2])
            quats.append([np.cos(ang / 2), 0, 0, np.sin(ang / 2)])  # yaw about z, wxyz
    return (torch.tensor(means, dtype=torch.float32), torch.tensor(quats, dtype=torch.float32),
            torch.tensor(scales, dtype=torch.float32))


def min_clearance(traj, means, R, scales, radius, n_surf=2000):
    """Smallest distance from trajectory samples to any ellipsoid surface; and whether any sample is inside an
    ellipsoid inflated by `radius` (tested on the surface sample + an inside test)."""
    k = torch.arange(n_surf, device=traj.device, dtype=torch.float32) + 0.5
    phi = torch.acos(1 - 2 * k / n_surf); theta = np.pi * (1 + 5 ** 0.5) * k
    unit = torch.stack([torch.cos(theta) * torch.sin(phi), torch.sin(theta) * torch.sin(phi), torch.cos(phi)], -1)
    best = torch.tensor(float("inf"), device=traj.device)
    inside = False
    for i in range(means.shape[0]):
        surf = means[i] + (unit * scales[i]) @ R[i].T
        d = torch.cdist(traj, surf).min()
        best = torch.minimum(best, d)
        local = (traj - means[i]) @ R[i] / scales[i]
        inside |= bool((local.norm(dim=-1) <= 1.0).any())
    return float(best), inside


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    summary = {"python": sys.version.split()[0], "torch": torch.__version__, "torch_cuda_build": torch.version.cuda,
               "cuda_available": torch.cuda.is_available()}
    if torch.cuda.is_available():
        summary["gpu"] = torch.cuda.get_device_name(0)
        summary["gpu_capability"] = list(torch.cuda.get_device_capability(0))
        x = torch.randn(2048, 2048, device="cuda"); summary["matmul_ok"] = bool(torch.isfinite(x @ x).all())
    print(json.dumps(summary, indent=1))
    if not torch.cuda.is_available():
        sys.exit("CUDA not available: Splat-Plan calls torch.cuda.synchronize() unconditionally")

    sys.path.insert(0, a.repo)
    os.chdir(a.out)  # the planner writes 'infeasible.obj' to cwd on failure; keep the clone clean
    from ellipsoids.covariance_utils import compute_cov, quaternion_to_rotation_matrix
    from splatplan.splatplan import SplatPlan
    from splatplan.spline_utils import SplinePlanner

    dev = torch.device("cuda")
    torch.manual_seed(0); rng = np.random.default_rng(0)
    means, quats, scales = build_scene(rng)
    gs = SyntheticGSplat(means, quats, scales, dev, compute_cov)
    Rm = quaternion_to_rotation_matrix(gs.rots)
    assert torch.isfinite(Rm).all() and torch.isfinite(gs.covs).all() and torch.isfinite(gs.covs_inv).all(), \
        "non-finite rotations/covariances: the planner would ignore those Gaussians"
    radius, vmax, amax = 0.05, 0.1, 0.1
    robot_config = {"radius": radius, "vmax": vmax, "amax": amax}
    voxel_config = {"lower_bound": torch.tensor([-1.0, -1.0, 0.0], device=dev),
                    "upper_bound": torch.tensor([1.0, 1.0, 0.6], device=dev), "resolution": 100}
    t0 = time.time()
    planner = SplatPlan(gs, robot_config, voxel_config, SplinePlanner(spline_deg=6, device=dev), dev)
    setup_s = time.time() - t0

    queries = [((-0.8, -0.5, 0.3), (0.8, -0.5, 0.3)),
               ((-0.8, 0.8, 0.3), (0.8, -0.8, 0.3)),
               ((0.8, 0.0, 0.3), (-0.8, 0.5, 0.3))]
    rows = []
    # negative control for the clearance check: the straight line of query 0 crosses the wall
    seg = torch.linspace(0, 1, 200, device=dev)[:, None]
    q0s, q0g = torch.tensor(queries[0][0], device=dev), torch.tensor(queries[0][1], device=dev)
    _, straight_inside = min_clearance(q0s + seg * (q0g - q0s), gs.means, Rm, gs.scales, radius)
    summary["negative_control_straight_q0_hits_ellipsoid"] = straight_inside
    print("negative control (straight q0 line inside an ellipsoid, expect True):", straight_inside)
    for q0, q1 in queries:
        x0 = torch.tensor(q0, device=dev, dtype=torch.float32)
        xf = torch.tensor(q1, device=dev, dtype=torch.float32)
        torch.cuda.synchronize(); t1 = time.time()
        try:
            out = planner.generate_path(x0, xf)
            err = None
        except Exception as e:  # the repo raises bare on infeasible QPs
            out, err = None, f"{type(e).__name__}: {e}"
        torch.cuda.synchronize()
        row = {"start": q0, "goal": q1, "plan_time_s": time.time() - t1, "error": err}
        if out is not None:
            traj = torch.tensor(out["traj"], device=dev, dtype=torch.float32)[:, :3]
            clr, inside = min_clearance(traj, gs.means, Rm, gs.scales, radius)
            row.update({"feasible": bool(out["feasible"]), "num_polytopes": out["num_polytopes"],
                        "traj_points": len(out["traj"]), "astar_points": len(out["path"]),
                        "times": {k: out[k] for k in ("times_astar", "times_collision_set", "times_polytope", "times_opt")},
                        "end_error_m": float((traj[-1] - xf).norm()), "start_error_m": float((traj[0] - x0).norm()),
                        "min_clearance_to_ellipsoids_m": clr, "clearance_ge_radius": clr >= radius - 1e-4,
                        "any_sample_inside_ellipsoid": inside,
                        "traj_xyz": traj.cpu().numpy().round(4).tolist(), "astar_xyz": np.asarray(out["path"]).round(4).tolist()})
        rows.append(row)
        print(json.dumps({k: v for k, v in row.items() if not k.endswith("_xyz")}, indent=1))

    # top-down figure: Gaussian xy footprints at their 1-sigma (`scales`) level + paths
    fig, ax = plt.subplots(figsize=(7, 7))
    th = np.linspace(0, 2 * np.pi, 60)
    Rn, Sn, Mn = Rm.cpu().numpy(), gs.scales.cpu().numpy(), gs.means.cpu().numpy()
    for i in range(len(Mn)):
        pts = Mn[i] + (np.stack([np.cos(th), np.sin(th), np.zeros_like(th)], -1) * Sn[i]) @ Rn[i].T
        ax.fill(pts[:, 0], pts[:, 1], color="0.55", alpha=0.5, lw=0)
    for j, r in enumerate(rows):
        c = f"C{j}"
        if "traj_xyz" in r:
            t = np.array(r["traj_xyz"]); p = np.array(r["astar_xyz"])
            ax.plot(p[:, 0], p[:, 1], ":", color=c, lw=1)
            ax.plot(t[:, 0], t[:, 1], "-", color=c, lw=2, label=f"q{j}: {r['num_polytopes']} polytopes, "
                    f"clear {r['min_clearance_to_ellipsoids_m']:.3f} m")
        ax.plot(*r["start"][:2], "o", color=c); ax.plot(*r["goal"][:2], "*", color=c, ms=12)
    ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.set_aspect("equal")
    ax.set_title("Splat-Plan on a synthetic Gaussian scene (top view)\nsolid: Bezier traj, dotted: A* seed; r = 0.05")
    ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout(); fig.savefig("splatplan_synthetic.png", dpi=110)

    summary.update({"setup_s": setup_s, "n_gaussians": int(means.shape[0]), "robot": robot_config,
                    "voxel_resolution": 100, "queries": rows,
                    "figure": os.path.join(a.out, "splatplan_synthetic.png")})
    with open("summary.json", "w") as f:
        json.dump(summary, f, indent=1)


if __name__ == "__main__":
    main()
