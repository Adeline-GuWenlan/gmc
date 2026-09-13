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
