"""Whole-scene rigid transforms (P4a kill-test harness, round-6).

The G1 scenes are unions of disc primitives, so a rigid transform is
EXACT: rotate + translate every disc center, radius unchanged.  The
workspace becomes the AABB of the transformed workspace corners.  The
returned scene drops the side-tag oracle cache (frame-stale) and records
the transform in meta for provenance; no other meta field is rewritten —
consumers that read G1-frame door fields from meta would silently get the
OLD frame, which is exactly the leak the kill test exists to catch.
"""
from __future__ import annotations

import numpy as np

from ..gaussian_geometry.primitives import SceneGeometry


def rigid_transform_scene(scene, phi, t=(0.0, 0.0)):
    c, s = np.cos(phi), np.sin(phi)
    R = np.array([[c, -s], [s, c]])
    t = np.asarray(t, dtype=float)
    groups = [(r, centers @ R.T + t) for r, centers in scene.disc_groups]
    xmin, xmax, ymin, ymax = scene.workspace
    corners = np.array([[xmin, ymin], [xmin, ymax],
                        [xmax, ymin], [xmax, ymax]]) @ R.T + t
    ws = (float(corners[:, 0].min()), float(corners[:, 0].max()),
          float(corners[:, 1].min()), float(corners[:, 1].max()))
    meta = {k: v for k, v in scene.meta.items() if k != "_tag_cache"}
    meta["rigid_transform"] = {"phi_rad": float(phi),
                               "t": [float(t[0]), float(t[1])]}
    return SceneGeometry(f"{scene.scene_id}_T", ws, groups, meta)


def rigid_transform_pose(pose, phi, t=(0.0, 0.0)):
    c, s = np.cos(phi), np.sin(phi)
    x, y = c * pose[0] - s * pose[1] + t[0], s * pose[0] + c * pose[1] + t[1]
    if len(pose) == 2:
        return (x, y)
    return (x, y, pose[2] + phi)
