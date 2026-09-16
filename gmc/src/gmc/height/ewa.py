"""Anisotropic EWA splatting: a real 3D-Gaussian rasteriser, on the CPU, JIT-compiled by numba.

This is the algorithm of the CUDA rasterisers (Kerbl et al. 2023 "3D Gaussian Splatting";
nerfstudio's ``gsplat``), not a scatter plot of the splat means: every 3D Gaussian is pushed
through the local affine (Jacobian) approximation of the perspective map to a screen-space conic,
16x16 pixel tiles gather the Gaussians whose 3-sigma disc touches them **in depth order**, and each
pixel alpha-composites its tile's list front to back.  Anisotropy, opacity and ordering are all
reproduced, which is what makes the result look like the photographs the capture was built from.

Why the CPU: this cluster ships no CUDA toolkit module and no ``nvcc``, and the render environment
has no torch, so every GPU rasteriser would need a new environment built from scratch.  numba is
already installed, ``prange`` uses every core of the job, and the picture a correct EWA rasteriser
produces does not depend on where it ran -- only how long it took.

The robot is composited *inside* the same front-to-back loop through a per-pixel occluder depth
buffer, so it is hidden exactly by whatever is really in front of it.  Where the occluder covers a
pixel, splats nearer than it are scaled by ``xray`` so a robot driving under a table stays visible.

Conventions: camera space is x right, y down, z forward (OpenCV); ``Camera.R`` maps world to camera
(``t = R (x - pos)``) so a camera-space z *is* the depth used for ordering and occlusion.
"""
import itertools
import math
import os
from dataclasses import dataclass

import numpy as np

# numba's default threading layer here is TBB, and when the cgroup gives the process very few CPUs TBB
# reports "workers currently limited to 0" and the interpreter then hangs in a futex at shutdown -- after
# the video is already written, so a Slurm job would sit there until its time limit. numba reads this at
# import, and setdefault leaves an explicit choice (e.g. NUMBA_THREADING_LAYER=omp) alone.
os.environ.setdefault("NUMBA_THREADING_LAYER", "workqueue")

from numba import njit, prange  # noqa: E402  (must follow the threading-layer choice above)

TILE = 16              # pixels per tile side
MAX_RADIUS_PX = 192    # clamp: one Gaussian may not claim more than (2*192/16+1)^2 tiles
BLUR = 0.3             # screen-space dilation added to the 2D covariance (px^2), as in the reference
CUTOFF = 8.0           # skip a Gaussian at a pixel below exp(-CUTOFF) ~ 3.4e-4
ALPHA_MIN = 1.0 / 255.0
ALPHA_MAX = 0.995
T_EPS = 1e-4           # a pixel is finished once this little light is left
FAR = 1e30


# ----------------------------------------------------------------------------- camera

@dataclass(frozen=True)
class Camera:
    pos: np.ndarray        # (3,) world position
    R: np.ndarray          # (3,3) world -> camera rotation, rows are right / down / forward
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    near: float = 0.05
    far: float = 1.0e6

    @property
    def fov_y_deg(self):
        return math.degrees(2.0 * math.atan(0.5 * self.height / self.fy))

    def as_dict(self):
        return {"pos": [float(v) for v in self.pos], "fov_y_deg": round(self.fov_y_deg, 3),
                "size": [int(self.width), int(self.height)], "near": self.near}


def look_at(target, azim_deg, elev_deg, dist):
    """Camera position and world->camera rotation for a camera ``dist`` away on the (azim, elev) ray."""
    a, e = math.radians(float(azim_deg)), math.radians(float(elev_deg))
    back = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    pos = np.asarray(target, dtype=float) + float(dist) * back
    fwd = -back
    up = np.array([0.0, 0.0, 1.0])
    down = -up + float(up @ fwd) * fwd          # world -up projected into the image plane
    n = float(np.linalg.norm(down))
    if n < 1e-9:                                 # straight down: pick a stable roll from the azimuth
        down = np.array([math.cos(a), math.sin(a), 0.0])
        n = 1.0
    down = down / n
    right = np.cross(down, fwd)                  # right-handed: x = y cross z
    return pos, np.stack([right, down, fwd])


def camera(target, azim_deg, elev_deg, dist, width, height, fov_y_deg=36.0, near=0.05, far=1.0e6):
    pos, R = look_at(target, azim_deg, elev_deg, dist)
    fy = 0.5 * float(height) / math.tan(0.5 * math.radians(float(fov_y_deg)))
    return Camera(pos, R, fy, fy, 0.5 * float(width), 0.5 * float(height), int(width), int(height),
                  float(near), float(far))


def project_points(cam, pts):
    """(u, v, depth) in pixels for world points ``pts`` (N,3); depth is camera-space z."""
    t = (np.asarray(pts, dtype=float) - cam.pos) @ cam.R.T
    z = t[:, 2]
    safe = np.where(np.abs(z) < 1e-9, 1e-9, z)
    return cam.fx * t[:, 0] / safe + cam.cx, cam.fy * t[:, 1] / safe + cam.cy, z


def frame_distance(hx, hy, hz, elev_deg, width, height, *, fov_y_deg=50.0, cover=0.85):
    """Distance at which a box of horizontal half-extents ``hx, hy`` and vertical half-extent ``hz``
    about the look point covers ``cover`` of the frame.

    Corner-exact fitting is the wrong tool at these ranges: under strong perspective the near corner of a
    3.5 m box swings far off axis, so demanding that every corner land inside the frame pushes the camera
    metres too far back -- in this 5.3 m gallery, up through the ceiling and out of the room.  The
    angular estimate below is what a camera operator would do, and ``fit_scale`` is the knob on top.
    """
    e = math.radians(float(elev_deg))
    tan_y = math.tan(0.5 * math.radians(float(fov_y_deg)))
    tan_x = tan_y * float(width) / float(height)
    r = math.hypot(float(hx), float(hy))                     # horizontal half-diagonal of the box
    v = r * math.sin(e) + float(hz) * math.cos(e)            # its apparent vertical half-extent
    return max(r / (cover * tan_x), v / (cover * tan_y))


# ----------------------------------------------------------------------------- scene packing

def pack(means, covs, opacity, rgb):
    """Render-ready float32 arrays: means, upper-triangle covariance, opacity, colour, 3-sigma radius.

    Done once; the per-frame cost is then a matmul over the means plus work on what survives culling.
    """
    means = np.ascontiguousarray(np.asarray(means, dtype=np.float32))
    C = np.asarray(covs, dtype=np.float64)
    if means.ndim != 2 or means.shape[1] != 3 or C.shape != (len(means), 3, 3):
        raise ValueError("means must be (N,3) and covs (N,3,3)")
    cov6 = np.empty((len(means), 6), dtype=np.float32)
    for k, (i, j) in enumerate([(0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2)]):
        cov6[:, k] = C[:, i, j]
    diag = np.maximum(np.einsum("nii->ni", C), 0.0)
    r3 = (3.0 * np.sqrt(diag.max(axis=1))).astype(np.float32)
    rgb = np.asarray(rgb, dtype=np.float32)
    if rgb.shape != means.shape:
        raise ValueError("rgb must be (N,3)")
    return {"means": means, "cov6": cov6, "r3": r3,
            "opacity": np.ascontiguousarray(np.asarray(opacity, dtype=np.float32)),
            "rgb": np.ascontiguousarray(np.clip(rgb, 0.0, 1.0))}


# ----------------------------------------------------------------------------- kernels

@njit(cache=True, parallel=True, fastmath=True)
def _conics(t, cov6, R, fx, fy, cx, cy, lim_x, lim_y, blur, cull_px, uv, conic, radius):
    """Screen-space conic of each Gaussian: Sigma_2D = J (R Sigma R^T) J^T + blur I, conic = inv."""
    for i in prange(t.shape[0]):
        tx, ty, tz = t[i, 0], t[i, 1], t[i, 2]
        rz = 1.0 / tz
        jx = min(lim_x, max(-lim_x, tx * rz)) * tz      # clamp off-frustum Gaussians (reference trick)
        jy = min(lim_y, max(-lim_y, ty * rz)) * tz
        a = fx * rz
        b = -fx * jx * rz * rz
        c = fy * rz
        d = -fy * jy * rz * rz

        s0, s1, s2, s3, s4, s5 = cov6[i, 0], cov6[i, 1], cov6[i, 2], cov6[i, 3], cov6[i, 4], cov6[i, 5]
        # M = R Sigma R^T, Sigma = [[s0,s1,s2],[s1,s3,s4],[s2,s4,s5]]
        m00 = m01 = m02 = m11 = m12 = m22 = 0.0
        for p in range(3):
            r0, r1, r2 = R[p, 0], R[p, 1], R[p, 2]
            w0 = r0 * s0 + r1 * s1 + r2 * s2            # (R Sigma) row p
            w1 = r0 * s1 + r1 * s3 + r2 * s4
            w2 = r0 * s2 + r1 * s4 + r2 * s5
            if p == 0:
                m00 = w0 * r0 + w1 * r1 + w2 * r2
                m01 = w0 * R[1, 0] + w1 * R[1, 1] + w2 * R[1, 2]
                m02 = w0 * R[2, 0] + w1 * R[2, 1] + w2 * R[2, 2]
            elif p == 1:
                m11 = w0 * r0 + w1 * r1 + w2 * r2
                m12 = w0 * R[2, 0] + w1 * R[2, 1] + w2 * R[2, 2]
            else:
                m22 = w0 * r0 + w1 * r1 + w2 * r2

        g00 = a * m00 + b * m02                          # (J M) row 0
        g01 = a * m01 + b * m12
        g02 = a * m02 + b * m22
        h11 = c * m11 + d * m12                          # (J M) row 1, columns 1 and 2
        h12 = c * m12 + d * m22
        A = g00 * a + g02 * b + blur
        B = g01 * c + g02 * d
        Cc = h11 * c + h12 * d + blur
        det = A * Cc - B * B
        if det <= 1e-12:
            radius[i] = 0
            continue
        idet = 1.0 / det
        conic[i, 0] = Cc * idet
        conic[i, 1] = -B * idet
        conic[i, 2] = A * idet
        uv[i, 0] = fx * tx * rz + cx
        uv[i, 1] = fy * ty * rz + cy
        mid = 0.5 * (A + Cc)
        disc = math.sqrt(max(mid * mid - det, 1e-9))
        rad = int(math.ceil(3.0 * math.sqrt(max(mid + disc, 1e-9))))
        if cull_px > 0 and rad > cull_px:
            radius[i] = 0                # near-camera Gaussians smear across the frame; drop them
            continue
        radius[i] = min(rad, MAX_RADIUS_PX) if rad > 0 else 1


@njit(cache=True)
def _tile_count(uv, radius, tx_n, ty_n, counts):
    total = 0
    for i in range(uv.shape[0]):
        r = radius[i]
        if r <= 0:
            continue
        i0 = int(math.floor((uv[i, 0] - r) / TILE))
        i1 = int(math.floor((uv[i, 0] + r) / TILE))
        j0 = int(math.floor((uv[i, 1] - r) / TILE))
        j1 = int(math.floor((uv[i, 1] + r) / TILE))
        if i0 < 0:
            i0 = 0
        if j0 < 0:
            j0 = 0
        if i1 > tx_n - 1:
            i1 = tx_n - 1
        if j1 > ty_n - 1:
            j1 = ty_n - 1
        for jy in range(j0, j1 + 1):
            base = jy * tx_n
            for jx in range(i0, i1 + 1):
                counts[base + jx + 1] += 1
                total += 1
    return total


@njit(cache=True)
def _tile_fill(uv, radius, tx_n, ty_n, cursor, lists):
    """Gaussians are visited in depth order, so each tile's slice comes out depth-ordered for free."""
    for i in range(uv.shape[0]):
        r = radius[i]
        if r <= 0:
            continue
        i0 = int(math.floor((uv[i, 0] - r) / TILE))
        i1 = int(math.floor((uv[i, 0] + r) / TILE))
        j0 = int(math.floor((uv[i, 1] - r) / TILE))
        j1 = int(math.floor((uv[i, 1] + r) / TILE))
        if i0 < 0:
            i0 = 0
        if j0 < 0:
            j0 = 0
        if i1 > tx_n - 1:
            i1 = tx_n - 1
        if j1 > ty_n - 1:
            j1 = ty_n - 1
        for jy in range(j0, j1 + 1):
            base = jy * tx_n
            for jx in range(i0, i1 + 1):
                t = base + jx
                lists[cursor[t]] = i
                cursor[t] += 1


@njit(cache=True, parallel=True, fastmath=True)
def _blend(uv, conic, opacity, rgb, depth, offs, lists, tx_n, ty_n, width, height,
           occ_rgb, occ_a, occ_d, xray, bg, out, out_depth):
    for tile in prange(tx_n * ty_n):
        jx = tile % tx_n
        jy = tile // tx_n
        s, e = offs[tile], offs[tile + 1]
        y1 = min((jy + 1) * TILE, height)
        x1 = min((jx + 1) * TILE, width)
        for py in range(jy * TILE, y1):
            for px in range(jx * TILE, x1):
                fpx = px + 0.5
                fpy = py + 0.5
                T = 1.0
                cr = cg = cb = 0.0
                oa = occ_a[py, px]
                od = occ_d[py, px]
                odone = oa <= 0.0
                med = FAR
                mdone = False
                for q in range(s, e):
                    i = lists[q]
                    dz = depth[i]
                    if (not odone) and dz >= od:
                        cr += T * oa * occ_rgb[py, px, 0]
                        cg += T * oa * occ_rgb[py, px, 1]
                        cb += T * oa * occ_rgb[py, px, 2]
                        T *= 1.0 - oa
                        odone = True
                        if (not mdone) and T < 0.5:
                            med = od
                            mdone = True
                        if T < T_EPS:
                            break
                    dx = fpx - uv[i, 0]
                    dy = fpy - uv[i, 1]
                    power = -0.5 * (conic[i, 0] * dx * dx + conic[i, 2] * dy * dy) - conic[i, 1] * dx * dy
                    if power < -CUTOFF:
                        continue
                    al = opacity[i] * math.exp(power)
                    if (not odone) and oa > 0.0:
                        al *= xray                      # x-ray: fade what stands between us and the robot
                    if al < ALPHA_MIN:
                        continue
                    if al > ALPHA_MAX:
                        al = ALPHA_MAX
                    w = T * al
                    cr += w * rgb[i, 0]
                    cg += w * rgb[i, 1]
                    cb += w * rgb[i, 2]
                    T *= 1.0 - al
                    if (not mdone) and T < 0.5:
                        med = dz
                        mdone = True
                    if T < T_EPS:
                        break
                if (not odone) and oa > 0.0 and T > T_EPS:
                    cr += T * oa * occ_rgb[py, px, 0]
                    cg += T * oa * occ_rgb[py, px, 1]
                    cb += T * oa * occ_rgb[py, px, 2]
                    T *= 1.0 - oa
                    if (not mdone) and T < 0.5:
                        med = od
                        mdone = True
                out[py, px, 0] = cr + T * bg[0]
                out[py, px, 1] = cg + T * bg[1]
                out[py, px, 2] = cb + T * bg[2]
                out_depth[py, px] = med


# ----------------------------------------------------------------------------- render

def render(scene, cam, *, occ=None, bg=(1.0, 1.0, 1.0), xray=0.15, blur=BLUR, clip=None,
           cull_radius_px=0, max_pairs=200_000_000):
    """Rasterise ``scene`` (from :func:`pack`) through ``cam``.

    ``occ`` is an optional occluder layer ``(rgb (H,W,3), alpha (H,W), depth (H,W))`` composited in
    depth order with the splats.  ``clip`` is an optional world AABB ``(x0,x1,y0,y1,z0,z1)`` that keeps
    only the Gaussians centred inside it.  Returns the image, the median-transmittance depth map (the
    visible surface, for occluding 2D overlays) and counters.

    ``cam.near`` matters far more here than in a synthetic scene: a camera placed inside a captured room
    sits *within* the geometry, and without a near cull the wall it is embedded in smears over the whole
    frame.  Callers set it from the shot (see ``near_frac`` in :mod:`gmc.height.viz3d`).
    """
    W, H = cam.width, cam.height
    R32 = np.ascontiguousarray(cam.R, dtype=np.float32)
    means = scene["means"]

    t = means @ R32.T
    t -= (R32 @ np.asarray(cam.pos, dtype=np.float32))
    tz = t[:, 2]
    ok = (tz > np.float32(cam.near)) & (tz < np.float32(cam.far))
    if clip is not None:
        x0, x1, y0, y1, z0, z1 = (float(c) for c in clip)
        ok &= ((means[:, 0] >= x0) & (means[:, 0] <= x1) & (means[:, 1] >= y0) & (means[:, 1] <= y1)
               & (means[:, 2] >= z0) & (means[:, 2] <= z1))
    inv = np.zeros_like(tz)
    np.divide(np.float32(1.0), tz, out=inv, where=ok)
    m = np.float32(cam.fx) * scene["r3"] * inv
    u = np.float32(cam.fx) * t[:, 0] * inv + np.float32(cam.cx)
    v = np.float32(cam.fy) * t[:, 1] * inv + np.float32(cam.cy)
    ok &= (u > -m) & (u < W + m) & (v > -m) & (v < H + m)
    idx = np.flatnonzero(ok)
    del inv, m, u, v, ok

    n_vis = int(len(idx))
    out = np.empty((H, W, 3), dtype=np.float32)
    out_depth = np.empty((H, W), dtype=np.float32)
    o_rgb, o_a, o_d = _occ_arrays(occ, H, W)
    if n_vis == 0:
        out[:] = np.asarray(bg, dtype=np.float32)
        out_depth[:] = FAR
        return {"rgb": out, "depth": out_depth, "n_visible": 0, "n_drawn": 0, "n_pairs": 0}

    tv = np.ascontiguousarray(t[idx])
    del t
    uv = np.zeros((n_vis, 2), dtype=np.float32)
    conic = np.zeros((n_vis, 3), dtype=np.float32)
    radius = np.zeros(n_vis, dtype=np.int32)
    _conics(tv, np.ascontiguousarray(scene["cov6"][idx]), R32, np.float32(cam.fx), np.float32(cam.fy),
            np.float32(cam.cx), np.float32(cam.cy), np.float32(1.3 * cam.cx / cam.fx),
            np.float32(1.3 * cam.cy / cam.fy), np.float32(blur), int(cull_radius_px),
            uv, conic, radius)

    keep = np.flatnonzero(radius > 0)
    dv = np.ascontiguousarray(tv[keep, 2])
    order = keep[np.argsort(dv, kind="stable")]
    uv = np.ascontiguousarray(uv[order])
    conic = np.ascontiguousarray(conic[order])
    depth = np.ascontiguousarray(tv[order, 2])
    radius = np.ascontiguousarray(radius[order])
    src = idx[order]
    op = np.ascontiguousarray(scene["opacity"][src])
    rgb = np.ascontiguousarray(scene["rgb"][src])
    del tv, keep, dv, order, src, idx

    tx_n = (W + TILE - 1) // TILE
    ty_n = (H + TILE - 1) // TILE
    counts = np.zeros(tx_n * ty_n + 1, dtype=np.int64)
    total = _tile_count(uv, radius, tx_n, ty_n, counts)
    if total > max_pairs:
        raise MemoryError(f"ewa.render: {total} tile-Gaussian pairs exceeds max_pairs={max_pairs}; "
                          f"raise it or narrow the view")
    offs = np.cumsum(counts)
    lists = np.empty(int(total), dtype=np.int32)
    _tile_fill(uv, radius, tx_n, ty_n, offs[:-1].copy(), lists)

    _blend(uv, conic, op, rgb, depth, offs, lists, tx_n, ty_n, W, H, o_rgb, o_a, o_d,
           np.float32(xray), np.asarray(bg, dtype=np.float32), out, out_depth)
    return {"rgb": out, "depth": out_depth, "n_visible": n_vis, "n_drawn": int(len(uv)),
            "n_pairs": int(total)}


def _occ_arrays(occ, H, W):
    if occ is None:
        return (np.zeros((H, W, 3), dtype=np.float32), np.zeros((H, W), dtype=np.float32),
                np.full((H, W), FAR, dtype=np.float32))
    o_rgb, o_a, o_d = occ
    return (np.ascontiguousarray(o_rgb, dtype=np.float32), np.ascontiguousarray(o_a, dtype=np.float32),
            np.ascontiguousarray(o_d, dtype=np.float32))


# ----------------------------------------------------------------------------- robot occluder

@njit(cache=True, parallel=True, fastmath=True)
def _cylinders(Rt, pos, fx, fy, cxp, cyp, x0, x1, y0, y1, cyl, light, amb, depth, shade):
    """Ray-cast vertical cylinders ``cyl`` = (cx, cy, r, z_lo, z_hi); nearest hit wins per pixel.

    The prism is analytic, so there is no triangle mesh and no z-fighting, and the surface normal
    comes out of the intersection for free -- which is what gives the robot its shading.
    Camera-space z of a hit is the ray parameter itself, because the ray direction is built with a
    camera-space z of exactly 1.
    """
    for py in prange(y0, y1):
        dcy = (py + 0.5 - cyp) / fy
        for px in range(x0, x1):
            dcx = (px + 0.5 - cxp) / fx
            dx = Rt[0, 0] * dcx + Rt[0, 1] * dcy + Rt[0, 2]
            dy = Rt[1, 0] * dcx + Rt[1, 1] * dcy + Rt[1, 2]
            dz = Rt[2, 0] * dcx + Rt[2, 1] * dcy + Rt[2, 2]
            best = FAR
            nx = ny = nz = 0.0
            for m in range(cyl.shape[0]):
                ccx, ccy, r, z_lo, z_hi = cyl[m, 0], cyl[m, 1], cyl[m, 2], cyl[m, 3], cyl[m, 4]
                ex = pos[0] - ccx
                ey = pos[1] - ccy
                A = dx * dx + dy * dy
                B = 2.0 * (dx * ex + dy * ey)
                Cq = ex * ex + ey * ey - r * r
                if A > 1e-12:
                    disc = B * B - 4.0 * A * Cq
                    if disc >= 0.0:
                        sq = math.sqrt(disc)
                        ta = (-B - sq) / (2.0 * A)
                        tb = (-B + sq) / (2.0 * A)
                        for s in range(2):
                            t = ta if s == 0 else tb
                            if 1e-6 < t < best:
                                wz = pos[2] + t * dz
                                if z_lo <= wz <= z_hi:
                                    best = t
                                    nx = (pos[0] + t * dx - ccx) / r
                                    ny = (pos[1] + t * dy - ccy) / r
                                    nz = 0.0
                if abs(dz) > 1e-12:
                    for s in range(2):
                        zc = z_lo if s == 0 else z_hi
                        t = (zc - pos[2]) / dz
                        if 1e-6 < t < best:
                            wx = pos[0] + t * dx - ccx
                            wy = pos[1] + t * dy - ccy
                            if wx * wx + wy * wy <= r * r:
                                best = t
                                nx = 0.0
                                ny = 0.0
                                nz = -1.0 if s == 0 else 1.0
            if best < FAR:
                depth[py, px] = best
                lam = nx * light[0] + ny * light[1] + nz * light[2]
                if lam < 0.0:
                    lam = 0.0
                shade[py, px] = amb + (1.0 - amb) * lam


@njit(cache=True, fastmath=True)
def _aa_lines(segs, half_w, colour, rgb, alpha):
    """Composite anti-aliased segments (pixel coords, (m,2,2)) over an RGBA layer, straight alpha."""
    H, W = alpha.shape
    for s in range(segs.shape[0]):
        ax, ay = segs[s, 0, 0], segs[s, 0, 1]
        bx, by = segs[s, 1, 0], segs[s, 1, 1]
        if not (math.isfinite(ax) and math.isfinite(ay) and math.isfinite(bx) and math.isfinite(by)):
            continue
        pad = half_w + 1.5
        x0 = max(int(math.floor(min(ax, bx) - pad)), 0)
        x1 = min(int(math.ceil(max(ax, bx) + pad)), W - 1)
        y0 = max(int(math.floor(min(ay, by) - pad)), 0)
        y1 = min(int(math.ceil(max(ay, by) + pad)), H - 1)
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        for py in range(y0, y1 + 1):
            for px in range(x0, x1 + 1):
                qx, qy = px + 0.5 - ax, py + 0.5 - ay
                t = 0.0 if L2 <= 1e-12 else min(1.0, max(0.0, (qx * vx + qy * vy) / L2))
                ddx = qx - t * vx
                ddy = qy - t * vy
                cov = half_w + 0.5 - math.sqrt(ddx * ddx + ddy * ddy)
                if cov <= 0.0:
                    continue
                if cov > 1.0:
                    cov = 1.0
                a0 = alpha[py, px]
                na = cov + a0 * (1.0 - cov)
                if na <= 0.0:
                    continue
                for k in range(3):
                    rgb[py, px, k] = (cov * colour[k] + a0 * (1.0 - cov) * rgb[py, px, k]) / na
                alpha[py, px] = na


def prism_occluder(cam, prisms, *, face_rgba, edge_rgb, edges=(), line_width=1.4, ambient=0.45,
                   dilate=7):
    """Occluder layer for vertical prisms: ``prisms`` rows are (x, y, r, z_lo, z_hi) in world metres.

    Returns ``(rgb, alpha, depth)`` ready for :func:`render`, or ``None`` when nothing is on screen.
    ``edges`` are world segments (m,2,3) drawn anti-aliased on top; the depth buffer is dilated by
    ``dilate`` pixels so those strokes inherit the prism's depth instead of falling behind the scene.
    """
    cyl = np.ascontiguousarray(np.atleast_2d(np.asarray(prisms, dtype=np.float64)))
    if cyl.size == 0:
        return None
    W, H = cam.width, cam.height
    corners = []
    for x, y, r, z_lo, z_hi in cyl:
        for sx, sy, sz in itertools.product((-r, r), (-r, r), (z_lo, z_hi)):
            corners.append((x + sx, y + sy, sz))
    u, v, z = project_points(cam, np.array(corners))
    good = z > cam.near
    if not good.any():
        return None
    pad = line_width + 3.0
    x0 = max(int(math.floor(u[good].min() - pad)), 0)
    x1 = min(int(math.ceil(u[good].max() + pad)) + 1, W)
    y0 = max(int(math.floor(v[good].min() - pad)), 0)
    y1 = min(int(math.ceil(v[good].max() + pad)) + 1, H)
    if x1 <= x0 or y1 <= y0 or not good.all():
        x0, y0, x1, y1 = 0, 0, W, H          # partly behind the camera: be safe, sweep the frame

    depth = np.full((H, W), FAR, dtype=np.float64)
    shade = np.zeros((H, W), dtype=np.float64)
    fwd = cam.R[2]
    light = -fwd + 0.75 * np.array([0.0, 0.0, 1.0]) + 0.35 * cam.R[0]
    light = light / np.linalg.norm(light)
    _cylinders(np.ascontiguousarray(cam.R.T), np.ascontiguousarray(cam.pos), cam.fx, cam.fy,
               cam.cx, cam.cy, x0, x1, y0, y1, cyl, np.ascontiguousarray(light), float(ambient),
               depth, shade)

    hit = depth < FAR
    rgb = np.zeros((H, W, 3), dtype=np.float32)
    alpha = np.zeros((H, W), dtype=np.float32)
    if hit.any():
        rgb[hit] = np.asarray(face_rgba[:3], dtype=np.float32) * shade[hit].astype(np.float32)[:, None]
        alpha[hit] = float(face_rgba[3])
    segs = np.asarray(edges, dtype=float).reshape(-1, 2, 3)
    if len(segs):
        su, sv, sz = project_points(cam, segs.reshape(-1, 3))
        behind = sz <= cam.near
        su = np.where(behind, np.nan, su)
        sv = np.where(behind, np.nan, sv)
        _aa_lines(np.ascontiguousarray(np.stack([su, sv], axis=1).reshape(-1, 2, 2)),
                  0.5 * float(line_width), np.asarray(edge_rgb, dtype=np.float64), rgb, alpha)
    if not hit.any() and alpha.max() <= 0.0:
        return None
    if dilate and dilate > 1:
        from scipy.ndimage import minimum_filter
        depth = minimum_filter(depth, size=int(dilate), mode="nearest")
    return rgb, alpha, depth.astype(np.float32)
