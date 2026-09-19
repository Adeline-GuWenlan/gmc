# gmc/experiments/plane_camera_scan.py
"""Amendment 3 P5: which camera azimuth lets a 3D video actually see its robot.

P2-SW-0's videos were rendered at the default azimuth (travel direction - 50 deg). Every frame put a tall
square enclosure between the camera and the route: the robot is drawn in x-ray on a gallery wall and table
B -- the thing the sweeper passes under -- never appears. The route is fine; the camera is not.

This scan replays the renderer's own follow camera (``viz3d._cameras`` and ``ewa.frame_distance``, same
elevation 35 deg, orbit 50 deg, fov 50 deg, near-cull 0.32 x distance) over 60 poses of each certified
curve, and counts the frames whose horizontal sight line from the look point to the camera crosses a
0.5 m bin whose uav-band density exceeds ``BLOCK`` supports per m² -- i.e. something at least 1.1 m tall, which at
35 deg elevation hides a floor robot within ~2.5 m of it. The density raster is P3's hall-wide
``p3_density_0p5m.npz``; nothing here loads the 7.3 M-splat scene, so it runs on the login node.

It is a selection aid for ``percase_render.py --azim``, checked against the frames it predicts: P2's
default camera scores 60/60 blocked, and every P2 3D key frame shows a wall; P3's long sweeper scores
8/60, and its mid-clip frame (pose 7.22, 12.42) shows the robot behind the long display wall.

Run from gmc/: PYTHONPATH=src:experiments python experiments/plane_camera_scan.py
"""
import json
import math
from pathlib import Path

import numpy as np

from gmc.height import ewa
from gmc.height.pathio import curve_from_dict, sample_curve
from gmc.height.prism import robot_table
from gmc.height.viz3d import SPLAT_OPTS, _cameras, _default_azim

DENSITY = Path("results/height/plane/long/p3_density_0p5m.npz")
OUT = Path("results/height/plane/p5_camera_scan.json")
VIEW_HALF = {"sweeper": 1.75, "uav": 2.5, "cylinder": 2.5}    # percase_render.VIEW_HALF
VIEW_PAD, ELEV, ORBIT, W, H = 0.5, 35.0, 50.0, 1280, 720      # percase_render / pointcloud_video defaults
BLOCK = 500          # uav-band supports per m² (0.5 m bins, as stored) that count as a wall
N_POSES = 60
AZIMS = (-150, -120, -90, -60, -30, -20, 0, 30, 60, 65, 90, 120, 150, 180)
RUNS = [("P2-SW-0", "results/height/plane/shared/sweeper.json", "sweeper"),
        ("P2-SW-0", "results/height/plane/shared/uav.json", "uav"),
        ("P3-long-0", "results/height/plane/long/sweeper.json", "sweeper"),
        ("P3-long-0", "results/height/plane/long/uav.json", "uav"),
        ("P3-cyl-rung2", "results/height/plane/long/cyl_ladder/rung2/cylinder.json", "cylinder"),
        ("P5-A-0", "results/height/plane/shared_a/sweeper.json", "sweeper"),
        ("P5-A-0", "results/height/plane/shared_a/uav.json", "uav"),
        ("P5-A-0", "results/height/plane/shared_a/cylinder.json", "cylinder")]


def sight_blocked(density, extent, cell, cx, cy, azim_deg, dist, elev=ELEV,
                  near_frac=SPLAT_OPTS["near_frac"], block=BLOCK, step=0.25):
    """True if the horizontal sight line from (cx, cy) toward the camera meets a cell over ``block``.

    Only the stretch the renderer keeps is scanned: from 0.5 m out (the robot's own neighbourhood) to the
    near-cull distance short of the camera, where splats are dropped anyway.
    """
    horiz = dist * math.cos(math.radians(elev))
    a = math.radians(azim_deg)
    for s in np.arange(0.5, horiz - near_frac * dist, step):
        x, y = cx + s * math.cos(a), cy + s * math.sin(a)
        i, j = int((x - extent[0]) // cell), int((y - extent[1]) // cell)
        if 0 <= i < density.shape[0] and 0 <= j < density.shape[1] and density[i, j] > block:
            return True
    return False


def blocked_frames(poses, window, robot_key, r, azim0, density, extent, cell):
    vw = [window[0] - VIEW_PAD, window[1] - VIEW_PAD, window[2] + VIEW_PAD, window[3] + VIEW_PAD]
    hz = max(abs(-0.02 - SPLAT_OPTS["look_z"]), abs(SPLAT_OPTS["fit_z"] - SPLAT_OPTS["look_z"]))
    cams = _cameras(poses, vw, VIEW_HALF[robot_key], ELEV, ORBIT, azim0, margin=0.6 + r)
    n = 0
    for cx, cy, hx, hy, az, _ in cams:
        dist = ewa.frame_distance(hx, hy, hz, ELEV, W, H, fov_y_deg=SPLAT_OPTS["fov_y_deg"],
                                  cover=SPLAT_OPTS["fit_cover"])
        n += sight_blocked(density, extent, cell, cx, cy, az, dist)
    return n


def main():
    d = np.load(DENSITY)
    extent, cell, dens = d["extent"], float(d["cell"]), d["uav"]
    out = {"what": "frames of the 3D follow camera whose sight line crosses a >= 1.1 m obstacle "
                   f"(> {BLOCK} uav-band supports per m² in a 0.5 m bin), out of {N_POSES}",
           "density": str(DENSITY), "elev": ELEV, "orbit_deg": ORBIT, "runs": []}
    for label, path, key in RUNS:
        if not Path(path).exists():
            continue
        run = json.loads(Path(path).read_text())
        res, case = run["result"], run["case"]
        if res.get("curve") is None:
            continue
        r = robot_table(case["z_c"])[key].max_radius()
        xy = sample_curve(curve_from_dict(res["curve"]), 0.02, r)
        poses = xy[np.linspace(0, len(xy) - 1, N_POSES).astype(int)]
        row = {"case": label, "robot": key, "run": path, "default_azim0": round(_default_azim(poses), 1),
               "blocked": {"default": blocked_frames(poses, case["window"], key, r, None, dens, extent, cell)}}
        for az in AZIMS:
            row["blocked"][str(az)] = blocked_frames(poses, case["window"], key, r, az, dens, extent, cell)
        out["runs"].append(row)
        print(label, key, json.dumps(row["blocked"]), flush=True)
    OUT.write_text(json.dumps(out, indent=1))
    print("wrote", OUT)


if __name__ == "__main__":
    main()
