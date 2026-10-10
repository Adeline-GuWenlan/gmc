"""bl B1: SplatNav (Splat-Plan, github.com/chengine/splatnav @ e996e22, pristine clone) adapter for ``bl_worker``.

Runs in the ``splatnav`` env on a GPU (``generate_path`` calls ``torch.cuda.synchronize`` unconditionally). Calls the
repo's own classes exactly as ``run_splatplan.py`` does -- ``SplatPlan(gsplat, {radius, vmax, amax}, {lower_bound,
upper_bound, resolution}, SplinePlanner(spline_deg, N_sec), device).generate_path(x0, xf)`` -- with the nerfstudio
loader (``splat/splat_utils.py``) replaced by a stand-in carrying the same fields (means, rots wxyz, scales, covs,
covs_inv, colors, opacities), as B0's smoke did. Nothing in the clone is changed.

Body cover (plan §3.2) -- Splat-Plan's robot is a sphere. The vertical cylinder (r, h) is covered through a linear
change of coordinates, which leaves every collision test invariant (A x ∩ A y = A(x ∩ y) for invertible A):
  * the planner's space is the plan frame recentred on the region box and squashed in z about the body centre,
        x' = (u - u_c, v - v_c, eps (z - z_c));
    Gaussians map to Gaussians (mean' = A mean, cov' = A cov A^T), so Splat-Plan still reads Gaussians;
  * the squashed body is a cylinder (r + m, eps (h + m)); Splat-Plan's sphere of radius
        R = sqrt((r + m)^2 + (eps (h + m) + dz')^2)
    covers it for any centre within dz' of the plane z' = 0, where dz' = sqrt(2) vmax^2 / (2 amax) bounds how far
    the Bezier curve can leave the plane (its safe corridor around each A* segment is the repo's local box of
    half-width vmax^2/(2 amax), randomly rotated about the segment);
  * so the cover, mapped back to the real frame, is the ellipsoid with semi-axes (R, R, R / eps): laterally only
    R / r times the body (1.0x-1.1x for the configurations tuned), vertically very tall;
  * the tall extent is harmless only because the obstacle set is first cut to the Gaussians that can touch the
    body at all: those whose judge-level ellipsoid meets the body's slab [z_c - h - m, z_c + h + m] (exact: an
    ellipsoid's z-extent is mean_z +- level sqrt(cov_zz)). Removing the others cannot make a path unsafe for the
    real body. A kept Gaussian that pokes out of the slab is still seen over its whole height: that is the cover's
    conservatism (measured in ``setup_info``: ``kept_extending_beyond_slab``);
  * the voxel grid (A* initialisation) is one layer thick (z' in [-R, R]) and the result is projected back onto
    z = z_c by the harness (``max_abs_dz_m`` in every row).
eps = 1 is the plain enclosing sphere of the cylinder (radius sqrt((r+m)^2 + (h+m)^2) = 0.917 m for the cylinder).

Obstacle contract (plan §3.1): Splat-Plan's obstacle is the ellipsoid with semi-axes ``scales``; ``sigma_level:
"judge"`` sets scales = level * sqrt(eig(cov)) -- the judge's level-sigma ellipsoid ("matched"); ``"native"`` uses
1 sigma. Opacity: Splat-Plan ignores it; the export already holds only the judge's Gaussians (opacity > tau).
Margin: added to r and h (matched). The repo's quaternion-to-rotation returns NaN for a zero vector part
(B0 hazard): such rotations are rewritten as an equivalent 90-degree yaw with swapped scales, and every rotation /
covariance is asserted finite.

Config (setup keys; ``q_*`` keys are query-time only): eps (z squash), cell_m (voxel edge in x, y), vmax, amax,
spline_deg, n_sec, sigma_level ("judge" | "native"), shrink_bounds (keep the A* grid inside the known box shrunk by
the body radius + corridor), q_seed.
Claimed = A* found a path and the QP was feasible (the repo's ``feasible``). FAIL reasons: astar_no_path,
qp_infeasible. Output path = the evaluated Bezier positions, unsquashed (z kept for the dz record).
"""
from __future__ import annotations

import math
import sys
import time

import numpy as np

REPO = "/scratch/wg2381/ext_repos/splatnav"
DEFAULTS = {"eps": .05, "cell_m": .02, "vmax": .1, "amax": .1, "spline_deg": 6, "n_sec": 10, "sigma_level": "judge",
            "shrink_bounds": True, "q_seed": 0}


def _quat_wxyz(R):
    """Rotation matrices (n,3,3), det +1 -> unit quaternions wxyz (Shepperd)."""
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(R).as_quat()           # xyzw
    return np.c_[q[:, 3], q[:, :3]]


def prepare_gaussians(scene, body, cfg):
    """Judge Gaussians -> Splat-Plan's (means, quats wxyz, scales) in the squashed, recentred frame + cover info."""
    meta = scene["meta"]
    level, m = float(meta["level"]), float(meta["margin_m"])
    r, h, z_c = float(body["radius_m"]), float(body["half_height_m"]), float(meta["z_c"])
    eps = float(cfg["eps"])
    w = cfg["vmax"] ** 2 / (2 * cfg["amax"])
    dz = math.sqrt(2) * w
    R = math.sqrt((r + m) ** 2 + (eps * (h + m) + dz) ** 2)
    means, covs = np.asarray(scene["means"], float), np.asarray(scene["covs"], float)
    ext = level * np.sqrt(np.einsum("nii->ni", covs))            # judge-level AABB half extents
    zlo, zhi = z_c - h - m, z_c + h + m
    box = np.asarray(meta["known_route_lower_m"][:2] + meta["known_route_upper_m"][:2], float)
    reach = R + 1e-6
    keep = ((means[:, 2] + ext[:, 2] >= zlo) & (means[:, 2] - ext[:, 2] <= zhi)
            & (means[:, 0] + ext[:, 0] >= box[0] - reach) & (means[:, 0] - ext[:, 0] <= box[2] + reach)
            & (means[:, 1] + ext[:, 1] >= box[1] - reach) & (means[:, 1] - ext[:, 1] <= box[3] + reach))
    beyond = int(np.count_nonzero(keep & ((means[:, 2] - ext[:, 2] < zlo) | (means[:, 2] + ext[:, 2] > zhi))))
    centre = np.array([(box[0] + box[2]) / 2, (box[1] + box[3]) / 2, z_c])
    S = np.diag([1., 1., eps])
    mu = (means[keep] - centre) @ S
    C = np.einsum("ij,njk,kl->nil", S, covs[keep], S)
    ev, V = np.linalg.eigh(C)
    ev = np.maximum(ev, 0.)
    V = np.where(np.linalg.det(V)[:, None, None] < 0, V * np.array([1., 1., -1.]), V)
    lev = level if cfg["sigma_level"] == "judge" else 1.
    scales = lev * np.sqrt(ev)
    q = _quat_wxyz(V)
    zero_vec = np.linalg.norm(q[:, 1:], axis=1) == 0
    if zero_vec.any():                           # B0 NaN hazard: rewrite as a 90-degree yaw, x/y scales swapped
        c = math.cos(math.pi / 4)
        Vz = V[zero_vec] @ np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        q[zero_vec] = _quat_wxyz(Vz)
        scales[zero_vec] = scales[zero_vec][:, [1, 0, 2]]
        _ = c
    lo = np.r_[box[:2] - centre[:2], -R]
    hi = np.r_[box[2:] - centre[:2], R]
    inset = (r + m + dz) if cfg.get("shrink_bounds", True) else 0.
    lo[:2] += inset
    hi[:2] -= inset
    res = [max(1, int(math.ceil((hi[i] - lo[i]) / cfg["cell_m"]))) for i in range(2)] + [1]
    info = {"n_input": int(len(means)), "n_kept_slab_and_box": int(keep.sum()),
            "kept_extending_beyond_slab": beyond, "zero_vector_quaternions_rewritten": int(zero_vec.sum()),
            "slab_z": [zlo, zhi], "centre_plan": centre.tolist(), "eps": eps, "corridor_half_width": w,
            "dz_squashed_max": dz, "dz_real_max_m": dz / eps,
            "sphere_radius_squashed": R, "cover_semi_axes_real_m": [R, R, R / eps],
            "cover_lateral_ratio_vs_body": R / r, "cover_lateral_excess_m": R - r,
            "grid_lower": lo.tolist(), "grid_upper": hi.tolist(), "grid_resolution": res,
            "grid_inset_m": inset, "scale_level": lev, "judge_level": level, "margin_m": m}
    return {"means": mu.astype(np.float32), "quats": q.astype(np.float32), "scales": scales.astype(np.float32),
            "centre": centre, "eps": eps, "z_c": z_c, "R": R, "lo": lo, "hi": hi, "res": res, "info": info}


class _GS:
    """Same fields as splat.splat_utils.DummyGSplatLoader (+ covs_inv, opacities)."""

    def __init__(self, means, quats, scales, device, compute_cov):
        import torch
        self.device = device
        self.means = torch.tensor(means, device=device)
        self.rots = torch.tensor(quats, device=device)
        self.scales = torch.tensor(scales, device=device)
        self.covs_inv = compute_cov(self.rots, 1.0 / torch.clamp(self.scales, min=1e-12))
        self.covs = compute_cov(self.rots, self.scales)
        self.colors = 0.5 * torch.ones(self.means.shape[0], 3, device=device)
        self.opacities = torch.ones(self.means.shape[0], device=device)


class Adapter:
    def __init__(self):
        self.setup_info, self.instance_info = {}, {}

    def build_setup(self, scene, body, config):
        cfg = dict(DEFAULTS, **config)
        st = prepare_gaussians(scene, body, cfg)
        st["cfg"] = cfg
        self.setup_info = st["info"]
        return st

    def instantiate(self, state, config):
        import torch
        sys.path.insert(0, REPO)
        from ellipsoids.covariance_utils import compute_cov, quaternion_to_rotation_matrix
        from splatplan.splatplan import SplatPlan
        from splatplan.spline_utils import SplinePlanner
        cfg = dict(state["cfg"], **{k: v for k, v in config.items() if k.startswith("q_")})
        dev = torch.device("cuda")
        gs = _GS(state["means"], state["quats"], state["scales"], dev, compute_cov)
        Rm = quaternion_to_rotation_matrix(gs.rots)
        if not (torch.isfinite(Rm).all() and torch.isfinite(gs.covs).all() and torch.isfinite(gs.covs_inv).all()):
            raise ValueError("non-finite rotations/covariances: Splat-Plan would silently drop those Gaussians")
        t0 = time.perf_counter()
        planner = SplatPlan(gs, {"radius": float(state["R"]), "vmax": float(cfg["vmax"]), "amax": float(cfg["amax"])},
                            {"lower_bound": torch.tensor(state["lo"], dtype=torch.float32, device=dev),
                             "upper_bound": torch.tensor(state["hi"], dtype=torch.float32, device=dev),
                             "resolution": torch.tensor(state["res"], device=dev)},
                            SplinePlanner(spline_deg=int(cfg["spline_deg"]), N_sec=int(cfg["n_sec"]), device=dev), dev)
        torch.cuda.synchronize()
        grid = planner.gsplat_voxel.non_navigable_grid
        self.instance_info = dict(state["info"], splatplan_init_s=time.perf_counter() - t0,
                                  grid_shape=list(grid.shape), grid_occupied_fraction=float(grid.float().mean()),
                                  gpu=torch.cuda.get_device_name(0), torch=torch.__version__)
        return {"planner": planner, "state": state, "cfg": cfg, "torch": torch, "dev": dev}

    def plan(self, live, start_uv, goal_uv, query):
        torch, st, planner = live["torch"], live["state"], live["planner"]
        c = st["centre"]
        x0 = torch.tensor([start_uv[0] - c[0], start_uv[1] - c[1], 0.], dtype=torch.float32, device=live["dev"])
        xf = torch.tensor([goal_uv[0] - c[0], goal_uv[1] - c[1], 0.], dtype=torch.float32, device=live["dev"])
        torch.manual_seed(int(live["cfg"].get("q_seed", 0)))          # compute_bounding_box draws a random axis
        rec = {}
        vox = planner.gsplat_voxel
        real = type(vox).create_path

        def create_path(x0_, xf_):                       # instance-level wrapper: records A*'s answer only
            p = real(vox, x0_, xf_)
            rec["astar"] = p
            return p
        vox.create_path = create_path
        try:
            out = planner.generate_path(x0, xf)
        except Exception as exc:
            if "astar" in rec and rec["astar"] is None:
                return {"claimed": False, "claimed_reason": "astar_no_path", "path_uv": None}
            if isinstance(exc, RuntimeError) and "reraise" in str(exc):
                return {"claimed": False, "claimed_reason": "qp_infeasible", "path_uv": None,
                        "info": {"astar_points": None if rec.get("astar") is None else len(rec["astar"])}}
            raise
        finally:
            del vox.create_path
        traj = np.asarray(out["traj"], float)[:, :3]
        P = np.c_[traj[:, 0] + c[0], traj[:, 1] + c[1], st["z_c"] + traj[:, 2] / st["eps"]]
        stages = {k: float(out[k]) for k in ("times_astar", "times_collision_set", "times_polytope", "times_opt")}
        astar = np.asarray(out["path"], float)
        info = {"num_polytopes": int(out["num_polytopes"]), "astar_points": int(len(astar)),
                "traj_points": int(len(traj)),
                "astar_start_offset_m": float(np.linalg.norm(astar[0, :2] - x0.cpu().numpy()[:2])),
                "astar_goal_offset_m": float(np.linalg.norm(astar[-1, :2] - xf.cpu().numpy()[:2]))}
        if not out["feasible"]:
            return {"claimed": False, "claimed_reason": "qp_infeasible", "path_uv": None, "stages": stages,
                    "info": info}
        return {"claimed": True, "claimed_reason": "feasible", "path_uv": P.tolist(), "stages": stages, "info": info}
