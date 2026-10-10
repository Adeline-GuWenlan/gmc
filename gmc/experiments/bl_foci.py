"""bl B1: FOCI (github.com/leggedrobotics/foci @ 79d8ddc) adapter for ``bl_worker``.

Runs in the ``foci`` env on a GPU (warp ``cuda:0`` at import) against ``ext_repos/foci_bl`` = the clone + the committed
``gmc/baselines/patches/foci_mumps.patch`` (IPOPT linear solver MA27 -> MUMPS; MA27 needs an HSL licence we do not
have: a disclosed deviation). Calls the repo's ``Planner(means, covs, robot_cov, num_control_points, num_samples)``
and ``Planner.plan(start, end)`` exactly as ``demos/stonehenge.py`` does; FOCI keeps its own initial guess (its A* on
a 0.25 m voxel grid of Gaussian means + a spline fit), never our A* route.

Frame: plan frame recentred on the region box in (u, v); z shifted so that the body centre z_c sits at 0.5, the
middle of FOCI's hard-coded ``z_range = (0, 1)`` (solver hull bounds and A* bounds), i.e. FOCI may move the body
within z_c +- 0.5 m. It plans in 3D; the harness projects its curve onto z = z_c and records ``max_abs_dz_m``.

Obstacles: the judge's Gaussians (opacity > tau already). Cut to those that can touch the body at all (judge-level
ellipsoid meets the body slab [z_c - h - m, z_c + h + m] and lies within reach of the known box) -- removing the
others cannot make a path unsafe for the real body. Contract: FOCI has no sigma level, threshold or margin: its
obstacle term is a soft Gaussian-overlap cost, exp(-d^T (S_o + S_r)^-1 d / 2) * 1000 / det(S_o + S_r), summed over
body points and curve samples, traded against jerk and a soft goal cost. ``sigma_level: "native"`` feeds the 1-sigma
covariances as the demo does; ``"judge"`` feeds level^2 * cov, so FOCI's 1-sigma shape is the judge's level-sigma
ellipsoid (the closest available "matched" setting). Mismatch recorded either way: no hard constraint, no margin.

Body: FOCI's robot is three body points (centre and +-0.2 m along the heading, ``Planner.kinematics``), each a
Gaussian ``robot_cov``. Our cover: robot_cov = diag(a^2, a^2, c^2) / level^2 * cov_scale^2, where (a, a, c) is the
minimum-volume ellipsoid enclosing the cylinder (r + m, h + m): a = sqrt(3/2) (r + m), c = sqrt(3) (h + m); so with
cov_scale 1 the level-sigma ellipsoid of each body point encloses the cylinder. ``kin_scale`` (default None = the
repo's 0.2 m) optionally changes the body-point spacing by subclassing ``Planner.__init__`` (same code, other
scale): a deviation, used only if tuning shows it matters, and disclosed.

Claimed = IPOPT reported success (``solver.stats()['success']``: Solve_Succeeded or Solved_To_Acceptable_Level).
FAIL reasons: astar_no_path (FOCI's ``ValueError("No path found")``), ipopt_<return_status>.
"""
from __future__ import annotations

import math
import time

import numpy as np

REPO = "/scratch/wg2381/ext_repos/foci_bl"
DEFAULTS = {"num_control_points": 10, "num_samples": 40, "sigma_level": "native", "cov_scale": 1.0,
            "kin_scale": None, "heading": "start_to_goal"}


def prepare(scene, body, cfg):
    meta = scene["meta"]
    level, m = float(meta["level"]), float(meta["margin_m"])
    r, h, z_c = float(body["radius_m"]), float(body["half_height_m"]), float(meta["z_c"])
    a, c = math.sqrt(1.5) * (r + m), math.sqrt(3.) * (h + m)
    s = float(cfg["cov_scale"])
    robot_cov = np.diag([a * a, a * a, c * c]) / level ** 2 * s ** 2
    means, covs = np.asarray(scene["means"], float), np.asarray(scene["covs"], float)
    ext = level * np.sqrt(np.einsum("nii->ni", covs))
    zlo, zhi = z_c - h - m, z_c + h + m
    box = np.asarray(meta["known_route_lower_m"][:2] + meta["known_route_upper_m"][:2], float)
    reach = r + m + 0.2 + 1e-6
    keep = ((means[:, 2] + ext[:, 2] >= zlo) & (means[:, 2] - ext[:, 2] <= zhi)
            & (means[:, 0] + ext[:, 0] >= box[0] - reach) & (means[:, 0] - ext[:, 0] <= box[2] + reach)
            & (means[:, 1] + ext[:, 1] >= box[1] - reach) & (means[:, 1] - ext[:, 1] <= box[3] + reach))
    centre = np.array([(box[0] + box[2]) / 2, (box[1] + box[3]) / 2, z_c - .5])
    mu = means[keep] - centre
    C = covs[keep] * (level ** 2 if cfg["sigma_level"] == "judge" else 1.)
    info = {"n_input": int(len(means)), "n_kept_slab_and_box": int(keep.sum()), "slab_z": [zlo, zhi],
            "centre_offset_plan": centre.tolist(), "robot_cov_diag": np.diag(robot_cov).tolist(),
            "cover_level_sigma_semi_axes_m": (level * np.sqrt(np.diag(robot_cov))).tolist(),
            "cylinder_r_h_plus_margin": [r + m, h + m], "obstacle_cov_factor": level ** 2 if
            cfg["sigma_level"] == "judge" else 1., "z_range_real": [z_c - .5, z_c + .5]}
    return {"means": mu, "covs": C, "robot_cov": robot_cov, "centre": centre, "z_c": z_c, "info": info}


def _planner_cls(kin_scale):
    from foci.planners.planner import Planner
    if kin_scale is None:
        return Planner
    import casadi as cas
    from foci.optim.solvers import create_solver

    class ScaledPlanner(Planner):
        """``Planner.__init__`` verbatim except the body-point spacing (0.2 m upstream)."""

        def __init__(self, obstacle_positions, obstacle_covs, robot_cov, num_control_points, num_samples,
                     obstacle_colors=None):
            self.num_control_points = num_control_points
            self.num_samples = num_samples
            self.obstacle_positions = obstacle_positions
            self.obstacle_covs = obstacle_covs
            self.robot_cov = robot_cov
            self.obstacle_colors = obstacle_colors
            pose = cas.MX.sym("pose", 4)
            theta = pose[3]
            middle = pose[:3]
            scale = float(kin_scale)
            left = middle - cas.vertcat(cas.cos(theta) * scale, cas.sin(theta) * scale, 0)
            right = middle + cas.vertcat(cas.cos(theta) * scale, cas.sin(theta) * scale, 0)
            self.kinematics = cas.Function("kinematics", [pose], [cas.horzcat(left, middle, right)])
            covs_sum = obstacle_covs + robot_cov
            covs_det = np.zeros(len(covs_sum))
            covs_inv = np.zeros_like(covs_sum)
            for i in range(len(covs_sum)):
                covs_det[i] = np.linalg.det(covs_sum[i])
                covs_inv[i] = np.linalg.inv(covs_sum[i])
            self.solver, self.lbg, self.ubg, self.convolution_functor = create_solver(
                num_control_points, obstacle_positions, covs_det, covs_inv, self.kinematics, dim_control_points=3,
                num_body_parts=3, num_samples=num_samples, z_range=(0, 1))
    return ScaledPlanner


class Adapter:
    def __init__(self):
        self.setup_info, self.instance_info = {}, {}

    def build_setup(self, scene, body, config):
        cfg = dict(DEFAULTS, **config)
        st = prepare(scene, body, cfg)
        st["cfg"] = cfg
        self.setup_info = st["info"]
        return st

    def instantiate(self, state, config):
        import foci
        if not foci.__file__.startswith(REPO):
            raise ValueError(f"foci imported from {foci.__file__}, not the MUMPS copy {REPO}")
        cfg = dict(state["cfg"], **{k: v for k, v in config.items() if k.startswith("q_")})
        t0 = time.perf_counter()
        planner = _planner_cls(cfg["kin_scale"])(state["means"], state["covs"], state["robot_cov"],
                                                 num_control_points=int(cfg["num_control_points"]),
                                                 num_samples=int(cfg["num_samples"]))
        self.instance_info = dict(state["info"], planner_init_s=time.perf_counter() - t0, foci_file=foci.__file__)
        return {"planner": planner, "state": state, "cfg": cfg}

    def plan(self, live, start_uv, goal_uv, query):
        st, planner = live["state"], live["planner"]
        c = st["centre"]
        s = np.array([start_uv[0] - c[0], start_uv[1] - c[1], .5])
        g = np.array([goal_uv[0] - c[0], goal_uv[1] - c[1], .5])
        yaw = math.atan2(g[1] - s[1], g[0] - s[0]) if live["cfg"]["heading"] == "start_to_goal" else math.pi / 2
        t0 = time.perf_counter()
        try:
            opt_curve, init = planner.plan(np.r_[s, yaw], np.r_[g, yaw])
        except ValueError as exc:
            if "No path found" in str(exc):
                return {"claimed": False, "claimed_reason": "astar_no_path", "path_uv": None,
                        "stages": {"plan_s": time.perf_counter() - t0}}
            raise
        stats = planner.solver.stats()
        stages = {"plan_s": time.perf_counter() - t0, "ipopt_iterations": stats.get("iter_count"),
                  "t_wall_total_solver": stats.get("t_wall_total")}
        P = np.asarray(opt_curve, float)[:, :3]
        P = np.c_[P[:, 0] + c[0], P[:, 1] + c[1], st["z_c"] + (P[:, 2] - .5)]
        info = {"ipopt_return_status": stats.get("return_status"), "ipopt_success": bool(stats.get("success")),
                "curve_samples": int(len(P)),
                "start_error_m": float(np.linalg.norm(P[0, :2] - np.asarray(start_uv))),
                "goal_error_m": float(np.linalg.norm(P[-1, :2] - np.asarray(goal_uv))),
                "init_points": int(len(init))}
        if not stats.get("success"):
            return {"claimed": False, "claimed_reason": f"ipopt_{stats.get('return_status')}", "path_uv": None,
                    "stages": stages, "info": info}
        return {"claimed": True, "claimed_reason": f"ipopt_{stats.get('return_status')}", "path_uv": P.tolist(),
                "stages": stages, "info": info}
