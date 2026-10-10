"""bl B2: cust_fields (github.com/Shuaikang-Wang/cust_fields @ 5ca178e) adapter for ``bl_worker`` -- plain NF.

Runs in the ``cust_fields`` env (numpy, PyYAML; CPU).  The main result is the repo's plain harmonic navigation
function (``NF/``, exactly as ``test_nf.py``): ``NavigationFunction(World(yaml), goal, NF_LAMBDA, NF_MU)`` and
``test_nf.py``'s own path loop (normalised -grad(phi) steps of DT * tanh(2 d), ``safe_advance`` backtracking, 16
random-direction escapes, stall limit 60, MAX_STEPS, GOAL_TOL), with ``neg_gradient`` / ``safe_advance`` imported from
the pristine ``test_nf.py``.  The homotopy customisation (``TOPO/``) needs a per-pair target homotopy class, which this
benchmark does not define, so it is not used (plan §4).

World construction from the shared rasteriser's C-space map (``bl_raster``; judge-conservative):
* workspace = the region box shrunk by r + m (the body centre's box), as the repo's squircle ``Workspace``
  (s = 0.9999, nearly the rectangle and inside it);
* obstacles = the 8-connected components of the map's Gaussian layer inside that workspace (``pieces``); each piece is
  covered by ONE rotated squircle (``Rectangular``, squareness ``s``): its convex hull's minimum-area rectangle, scaled
  about the centre until the squircle contains every hull vertex (so the squircle contains the piece's closed cells);
  covers that overlap (or come closer than ``merge_gap_m``) are merged -- union of the pieces, re-fitted -- until all
  covers are pairwise disjoint, because the NF's forest world needs disjoint obstacles.  Every merge of pieces that the
  map kept apart can close a passage: counted, and passage closure is measured per pair (``bl_custfields_check``).
* The repo's ``Rectangular`` silently adds 0.1 m to every width and height (``geometry.py``: ``width + 0.1``) and so
  does ``Workspace`` (it inherits that ``__init__``).  ``native_pad: false`` (matched contract) passes width - 0.1 so
  the squircle the NF sees is the cover we computed (when the cover is < 0.1 m wide the pad cannot be undone: the
  repo's squircle is then larger, recorded); ``native_pad: true`` keeps the repo's 5 cm pad per side.
* Pieces that touch the workspace boundary (walls bulging into the region) give covers that cross the workspace's
  boundary.  The NF's forest world needs obstacles strictly inside the workspace; the repo's mechanism for obstacles
  attached to the workspace (a workspace ``StarTree``) cannot run: ``ForestToStar.compute_virtual_ws`` calls
  ``Rectangular(_type, center, w, h)`` without the required ``theta`` and ``s`` (TypeError; shown by the probe).  We
  keep such covers as ordinary obstacles and record how many cross the boundary -- a declared violation of the
  method's assumption, forced by the code.
Endpoints inside a cover (or in a map-occupied cell) are moved to the nearest point that is free in both (grid
search at the raster resolution, up to ``snap_max_m``; recorded) -- else FAIL ``endpoint_in_obstacle_cover``.  The
harness adds the exact start / goal segments; the judge checks them like the rest.
"""
from __future__ import annotations

import importlib.util
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

import bl_raster as R

REPO = "/scratch/wg2381/ext_repos/cust_fields"
GMC = Path(__file__).resolve().parents[1]
DEFAULTS = {"raster_res_m": 0.005, "s": 0.85, "ws_s": 0.9999, "native_pad": False, "merge_gap_m": 0.0,
            "nf_lambda": 1e3, "nf_mu": [1e10, 1e8, 1e6, 1e4, 1e2, 1e1], "dt": 0.05, "max_steps": 3000,
            "goal_tol": 0.05, "snap_max_m": 0.10, "seed": 0}
REPO_PAD = 0.1


# ============================================================================================ geometry (numpy)
def label8(mask):
    """8-connected component labels of a boolean image (row runs + union-find; numpy + Python, no scipy)."""
    ny, nx = mask.shape
    parent = []

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    runs = []                                    # (row, c0, c1 inclusive, id)
    prev = []
    for y in range(ny):
        row = mask[y]
        if not row.any():
            prev = []
            continue
        d = np.diff(np.r_[0, row.astype(np.int8), 0])
        starts, ends = np.nonzero(d == 1)[0], np.nonzero(d == -1)[0] - 1
        cur = []
        j = 0
        for c0, c1 in zip(starts, ends):
            rid = len(parent)
            parent.append(rid)
            while j < len(prev) and prev[j][2] < c0 - 1:
                j += 1
            k = j
            while k < len(prev) and prev[k][1] <= c1 + 1:
                ra, rb = find(rid), find(prev[k][3])
                if ra != rb:
                    parent[max(ra, rb)] = min(ra, rb)
                k += 1
            cur.append((y, int(c0), int(c1), rid))
        runs.extend(cur)
        prev = cur
    lab = np.zeros((ny, nx), np.int32)
    roots, n = {}, 0
    for y, c0, c1, rid in runs:
        r = find(rid)
        if r not in roots:
            n += 1
            roots[r] = n
        lab[y, c0:c1 + 1] = roots[r]
    return lab, n


def convex_hull(P):
    """Andrew's monotone chain; P (n, 2) -> hull vertices counter-clockwise."""
    P = np.unique(np.asarray(P, float), axis=0)
    if len(P) <= 2:
        return P
    P = P[np.lexsort((P[:, 1], P[:, 0]))]

    def half(pts):
        h = []
        for p in pts:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (p[1] - h[-2][1])
                                   - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(p)
        return h
    lower, upper = half(P), half(P[::-1])
    return np.array(lower[:-1] + upper[:-1])


def min_area_rect(H):
    """Minimum-area enclosing rectangle of a convex polygon: (centre, half sizes (a, b), theta) with the a-axis along
    (cos theta, sin theta)."""
    if len(H) == 1:
        return H[0], np.array([0., 0.]), 0.
    E = np.roll(H, -1, axis=0) - H
    best = None
    for th in np.unique(np.mod(np.arctan2(E[:, 1], E[:, 0]), np.pi / 2)):
        c, s = math.cos(th), math.sin(th)
        X = H @ np.array([[c, -s], [s, c]])          # coordinates along (c, s) and (-s, c)
        lo, hi = X.min(axis=0), X.max(axis=0)
        area = float(np.prod(hi - lo))
        if best is None or area < best[0]:
            mid = (lo + hi) / 2
            best = (area, np.array([c * mid[0] - s * mid[1], s * mid[0] + c * mid[1]]), (hi - lo) / 2, th)
    return best[1], best[2], best[3]


def squircle_level(q, center, a, b, theta, s):
    """The repo's ``Rectangular.check_point_inside`` potential (<= 0 inside) for full sizes 2a x 2b (no pad), vector."""
    q = np.atleast_2d(q)
    x0, y0 = center
    dx, dy = q[:, 0] - x0, q[:, 1] - y0
    x = dx * math.cos(theta) + dy * math.sin(theta)
    y = -dx * math.sin(theta) + dy * math.cos(theta)
    A, B = 2 * a, 2 * b
    X = (B / A) * x
    return (1 / (B / 2) ** 2) * (X ** 2 + y ** 2 + np.sqrt(X ** 4 + y ** 4 + (2 - 4 * s ** 2) * X ** 2 * y ** 2)) / 2 - 1


def fit_cover(H, s, min_half=1e-3):
    """Squircle (centre, a, b, theta) containing the convex polygon H: min-area rectangle, scaled about its centre."""
    c, ab, th = min_area_rect(H)
    a, b = max(ab[0], min_half), max(ab[1], min_half)
    # the squircle's level is homogeneous: scaling (a, b) by lam maps level(q) through q -> c + (q - c) / lam
    lo, hi = 1., 4.
    while squircle_level(H, c, a * hi, b * hi, th, s).max() > 0:
        hi *= 2
    for _ in range(60):
        mid = (lo + hi) / 2
        if squircle_level(H, c, a * mid, b * mid, th, s).max() > 0:
            lo = mid
        else:
            hi = mid
    lam = hi * (1 + 1e-9)
    return c, a * lam, b * lam, th


def squircle_boundary(c, a, b, th, s, n=256):
    """Points on the squircle boundary (radial bisection on the level), for overlap tests and plots."""
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    u = np.c_[np.cos(ang), np.sin(ang)]
    lo, hi = np.zeros(n), np.full(n, 2 * max(a, b) + 1e-6)
    for _ in range(50):
        mid = (lo + hi) / 2
        inside = squircle_level(c + u * mid[:, None], c, a, b, th, s) <= 0
        lo, hi = np.where(inside, mid, lo), np.where(inside, hi, mid)
    return c + u * hi[:, None]


def covers_overlap(A, B, s, gap):
    """Conservative overlap test of two convex squircles inflated by gap/2 each: boundary samples of one inside the
    other (centres included).  Sampled, with a tolerance of the sample spacing."""
    if np.linalg.norm(np.asarray(A[0]) - np.asarray(B[0])) > math.hypot(A[1], A[2]) + math.hypot(B[1], B[2]) + gap + .01:
        return False                                  # bounding circles apart (a squircle lies in its rectangle)
    for P, Q in ((A, B), (B, A)):
        cP, aP, bP, tP = P
        cQ, aQ, bQ, tQ = Q
        pad = gap / 2 + 2 * math.pi * max(aP, bP) / 256 + 1e-4
        pts = np.r_[squircle_boundary(cP, aP + pad, bP + pad, tP, s), [cP]]
        if (squircle_level(pts, cQ, aQ + gap / 2, bQ + gap / 2, tQ, s) <= 0).any():
            return True
    return False


def build_obstacles(gauss, ws_mask, info, s, gap):
    """Pieces -> disjoint squircle covers (merging); returns covers, piece ids per cover, stats."""
    pieces, n = label8(gauss & ws_mask)
    res, u0, v0 = info["res_m"], info["u0"], info["v0"]
    pts = {}
    ys, xs = np.nonzero(pieces)
    lab = pieces[ys, xs]
    # the convex hull of a piece's closed cells only needs each row's first and last cell (their 4 corners)
    key = lab.astype(np.int64) * (gauss.shape[0] + 1) + ys
    o2 = np.lexsort((xs, key))
    key, xs2, ys2, lab2 = key[o2], xs[o2], ys[o2], lab[o2]
    first = np.r_[True, key[1:] != key[:-1]]
    last = np.r_[key[1:] != key[:-1], True]
    ex = np.r_[np.nonzero(first)[0], np.nonzero(last)[0]]
    ex_lab, ex_x, ex_y = lab2[ex], xs2[ex], ys2[ex]
    o3 = np.argsort(ex_lab, kind="stable")
    ex_lab, ex_x, ex_y = ex_lab[o3], ex_x[o3], ex_y[o3]
    cuts = np.r_[0, np.nonzero(np.diff(ex_lab))[0] + 1, len(ex_lab)]
    for k in range(len(cuts) - 1):
        xx, yy = ex_x[cuts[k]:cuts[k + 1]], ex_y[cuts[k]:cuts[k + 1]]
        corners = np.concatenate([np.c_[u0 + (xx + dx) * res, v0 + (yy + dy) * res] for dx in (0, 1) for dy in (0, 1)])
        pts[int(ex_lab[cuts[k]])] = convex_hull(corners)
    groups = [[k] for k in sorted(pts)]
    covers = [fit_cover(pts[k], s) for k in sorted(pts)]
    merges = 0
    changed = True
    while changed:
        changed = False
        for i in range(len(covers)):
            for j in range(i + 1, len(covers)):
                if covers_overlap(covers[i], covers[j], s, gap):
                    groups[i] = groups[i] + groups[j]
                    covers[i] = fit_cover(convex_hull(np.concatenate([pts[k] for k in groups[i]])), s)
                    del groups[j], covers[j]
                    merges += 1
                    changed = True
                    break
            if changed:
                break
    return covers, groups, {"pieces": int(n), "covers": len(covers), "merges": merges,
                            "pieces_in_merged_covers": int(sum(len(g) for g in groups if len(g) > 1))}


def cover_mask(covers, s, info, shape):
    """Raster of the union of covers on the map grid (a cell is marked if its centre is inside a cover)."""
    ny, nx = shape
    xs = info["u0"] + (np.arange(nx) + .5) * info["res_m"]
    ys = info["v0"] + (np.arange(ny) + .5) * info["res_m"]
    out = np.zeros(shape, bool)
    for c, a, b, th in covers:
        r = max(a, b) * math.sqrt(2) + info["res_m"]
        ix = np.nonzero((xs >= c[0] - r) & (xs <= c[0] + r))[0]
        iy = np.nonzero((ys >= c[1] - r) & (ys <= c[1] + r))[0]
        if not len(ix) or not len(iy):
            continue
        X, Y = np.meshgrid(xs[ix], ys[iy])
        inside = squircle_level(np.c_[X.ravel(), Y.ravel()], c, a, b, th, s).reshape(X.shape) <= 0
        out[np.ix_(iy, ix)] |= inside
    return out


def _test_nf_module():
    spec = importlib.util.spec_from_file_location("cf_test_nf", os.path.join(REPO, "test_nf.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)                 # defines functions only (main() is guarded)
    return mod


class Adapter:
    def __init__(self):
        self.setup_info, self.instance_info = {}, {}

    def build_setup(self, scene, body, config):
        cfg = dict(DEFAULTS, **config)
        meta = scene["meta"]
        path = GMC / R.raster_path(meta["region"], meta["robot"], float(cfg["raster_res_m"]))
        arrays, info = R.load(path)
        if info["scene_export_sha256"] != R._sha(GMC / "outputs/baselines/scene" /
                                                  f"{meta['region']}_{meta['robot']}.npz"):
            raise ValueError("raster was not built from this scene export")
        t0 = time.perf_counter()
        lo, hi = meta["known_route_lower_m"], meta["known_route_upper_m"]
        rm = float(body["radius_m"]) + float(meta["margin_m"])
        ws_lo, ws_hi = np.array([lo[0] + rm, lo[1] + rm]), np.array([hi[0] - rm, hi[1] - rm])
        free = arrays["occ"] == 0
        cx, cy = R.cell_centres(info["u0"], info["v0"], info["res_m"], info["nx"], info["ny"])
        X, Y = np.meshgrid(cx, cy)
        h = info["res_m"] / 2
        ws_mask = (X - h >= ws_lo[0]) & (X + h <= ws_hi[0]) & (Y - h >= ws_lo[1]) & (Y + h <= ws_hi[1])
        covers, groups, st = build_obstacles(arrays["gauss"].astype(bool), ws_mask, info, float(cfg["s"]),
                                             float(cfg["merge_gap_m"]))
        cmask = cover_mask(covers, float(cfg["s"]), info, free.shape)
        ws_c, ws_size = (ws_lo + ws_hi) / 2, ws_hi - ws_lo
        # a cover crosses the workspace boundary if a boundary sample lies outside the workspace rectangle
        crossing = 0
        for c, a, b, th in covers:
            P = squircle_boundary(c, a, b, th, float(cfg["s"]), 128)
            crossing += int(((P < ws_lo) | (P > ws_hi)).any())
        pad = 0. if cfg["native_pad"] else REPO_PAD
        yaml_obs = []
        unpadded = 0
        for c, a, b, th in covers:
            w, hh = 2 * a - pad, 2 * b - pad
            if w <= 1e-3 or hh <= 1e-3:
                unpadded += 1
                w, hh = max(w, 1e-3), max(hh, 1e-3)
            yaml_obs.append({"shape": "Star", "star": [{"type": "Rectangular", "center": [float(c[0]), float(c[1])],
                                                        "width": float(w), "height": float(hh), "theta": float(th),
                                                        "s": float(cfg["s"])}]})
        world = {"obstacles": yaml_obs,
                 "workspace": [{"shape": "StarTree", "stars": [{
                     "type": "Workspace", "center": [float(ws_c[0]), float(ws_c[1])],
                     "width": float(ws_size[0] - pad), "height": float(ws_size[1] - pad), "theta": 0.0,
                     "s": float(cfg["ws_s"])}]}]}
        ws_box = ((X >= ws_lo[0]) & (X <= ws_hi[0]) & (Y >= ws_lo[1]) & (Y <= ws_hi[1]))
        nf_free = ws_box & ~cmask & free                 # free in the NF world's covers and in the map
        sinfo = dict(st, raster=str(path.relative_to(GMC)), raster_sha256=info["sha256"],
                     raster_build_wall_s=info["build_wall_s"], raster_res_m=info["res_m"],
                     covers_crossing_workspace_boundary=crossing, covers_below_repo_pad=unpadded,
                     native_pad=bool(cfg["native_pad"]), squareness=float(cfg["s"]),
                     map_free_frac_in_ws=float((free & ws_box).sum() / max(1, ws_box.sum())),
                     nf_free_frac_in_ws=float(nf_free.sum() / max(1, ws_box.sum())),
                     map_free_cells_lost_to_covers=int((free & ws_box & cmask).sum()),
                     map_occupied_cells_outside_covers_in_ws=int((~free & ws_mask & ~cmask &
                                                                   arrays["gauss"].astype(bool)).sum()),
                     build_world_wall_s=time.perf_counter() - t0)
        self.setup_info = sinfo
        return {"world": world, "covers": [(c.tolist(), a, b, th) for c, a, b, th in covers], "groups": groups,
                "occ": arrays["occ"], "nf_free": nf_free.astype(np.uint8),
                "grid": {"u0": info["u0"], "v0": info["v0"], "res_m": info["res_m"]},
                "cfg": cfg, "info": sinfo}

    def instantiate(self, state, config):
        import yaml
        cfg = dict(state["cfg"], **{k: v for k, v in config.items() if k.startswith("q_")})
        p = GMC / state["info"]["raster"]
        if R._sha(p) != state["info"]["raster_sha256"]:
            raise ValueError(f"raster {p} changed since this setup was built (stale setup artifact)")
        sys.path.insert(0, REPO)
        from NF.geometry import World
        from NF.navigation import NavigationFunction
        t0 = time.perf_counter()
        yml = Path(os.getcwd()) / f"bl_world_{os.getpid()}.yaml"
        yml.write_text(yaml.safe_dump(state["world"]))
        world = World(str(yml))
        tnf = _test_nf_module()
        self.instance_info = dict(state["info"], world_yaml=str(yml), n_obstacles=len(world.obstacles),
                                  world_load_s=time.perf_counter() - t0)
        return {"world": world, "NF": NavigationFunction, "tnf": tnf, "state": state, "cfg": cfg}

    def _snap(self, live, uv):
        """Nearest point (cell centre at the raster resolution) free in the NF world and in the map; distance."""
        world, st, cfg = live["world"], live["state"], live["cfg"]
        p = np.asarray(uv, float)
        g = st["grid"]
        iy, ix = (int(v[0]) for v in R.to_index(g, p))
        ok_here = (0 <= iy < st["occ"].shape[0] and 0 <= ix < st["occ"].shape[1] and st["occ"][iy, ix] == 0
                   and world.check_point_in_free_space(p, threshold=0.0))
        if ok_here:
            return p, 0.
        hit = R.nearest_free({"occ": 1 - st["nf_free"]}, g, p, max_m=float(cfg["snap_max_m"]))
        if hit is None:
            return None, None
        c = np.asarray(hit[2], float)
        if not world.check_point_in_free_space(c, threshold=0.0):      # cover raster is by cell centre: re-check
            return None, None
        return c, hit[3]

    def plan(self, live, start_uv, goal_uv, query):
        world, tnf, cfg = live["world"], live["tnf"], live["cfg"]
        t0 = time.perf_counter()
        s, s_d = self._snap(live, start_uv)
        g, g_d = self._snap(live, goal_uv)
        info = {"start_snap_m": s_d, "goal_snap_m": g_d, "n_obstacles": len(world.obstacles)}
        stages = {"snap_s": time.perf_counter() - t0}
        if s is None or g is None:
            return {"claimed": False, "claimed_reason": "endpoint_in_obstacle_cover", "path_uv": None,
                    "stages": stages, "info": info}
        t1 = time.perf_counter()
        nf = live["NF"](world, np.array([g[0], g[1], 0.0]), float(cfg["nf_lambda"]), list(cfg["nf_mu"]))
        traj, reason, steps, evals = self._descend(nf, world, tnf, s, g, cfg)
        stages["nf_descend_s"] = time.perf_counter() - t1
        info.update(steps=steps, gradient_evals=evals, final_dist_m=float(np.linalg.norm(traj[-1] - g)))
        if reason != "goal_reached":
            return {"claimed": False, "claimed_reason": reason, "path_uv": None, "stages": stages, "info": info}
        return {"claimed": True, "claimed_reason": reason, "path_uv": np.asarray(traj).tolist(), "stages": stages,
                "info": info}

    @staticmethod
    def _descend(nf, world, tnf, start, goal, cfg):
        """``test_nf.py`` main loop, verbatim logic (parameters from cfg); returns (trajectory, reason, steps, evals)."""
        DT, MAX_STEPS, GOAL_TOL = float(cfg["dt"]), int(cfg["max_steps"]), float(cfg["goal_tol"])
        rng = np.random.default_rng(int(cfg["seed"]))
        q = np.asarray(start, float).copy()
        GOAL = np.asarray(goal, float)
        trajectory = [q.copy()]
        stall, evals = 0, 0
        for step in range(MAX_STEPS):
            dist = np.linalg.norm(q - GOAL)
            if dist < GOAL_TOL:
                return np.array(trajectory), "goal_reached", step, evals
            grad_dir, grad_norm = tnf.neg_gradient(nf, q)
            evals += 1
            speed = float(np.tanh(2.0 * dist))
            nominal = DT * speed
            moved = False
            if grad_dir is not None and 1e-3 < grad_norm < 1e4:
                cand, ok = tnf.safe_advance(world, q, grad_dir, nominal)
                if ok and np.linalg.norm(cand - q) > 0.1 * nominal:
                    q = cand
                    moved = True
            if not moved:
                phi0 = nf.compute_potential_at_point(q)
                best, best_phi = None, phi0
                for _ in range(16):
                    ang = rng.uniform(-np.pi, np.pi)
                    d = np.array([np.cos(ang), np.sin(ang)])
                    c, ok = tnf.safe_advance(world, q, d, nominal)
                    if ok and np.linalg.norm(c - q) > 0.5 * nominal:
                        phi = nf.compute_potential_at_point(c)
                        if phi < best_phi:
                            best, best_phi = c, phi
                if best is not None:
                    q = best
                    moved = True
            if not moved:
                stall += 1
                if stall > 60:
                    return np.array(trajectory), "nf_stuck", step, evals
                continue
            stall = 0
            trajectory.append(q.copy())
        return np.array(trajectory), "nf_max_steps", MAX_STEPS, evals
