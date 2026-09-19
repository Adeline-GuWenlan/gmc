"""Amendment 3 P5: the 3D camera's azimuth must be choosable, and P2's videos need their own caption.

P2-SW-0's 3D videos put the camera at the default azimuth (travel direction - 50 deg), which on that case is
south of a tall square enclosure: every frame shows the route as an occluded ghost and the robot in x-ray
through a gallery wall, and table B -- the object the sweeper passes under -- never appears. The routes
are fine (certified maps, the 2-panel side view and replay3d agree); the camera is not. ``--azim`` lets a
re-render look from the open side. The default stays None, so every existing render is unchanged.
"""
import numpy as np
import pytest

percase_render = pytest.importorskip("percase_render")


@pytest.fixture
def fakes(monkeypatch, tmp_path):
    seen = {}
    poses = np.zeros((3, 3))

    def pv(*a, **k):
        seen["video"] = k
        return {"video": tmp_path / "v.mp4", "n_frames": 3, "frames": [], "render_seconds_per_frame": 0.0,
                "n_points": 0, "workers": 1, "size": (1, 1), "view_half": k.get("view_half"),
                "elev": k.get("elev"), "orbit_deg": 50.0, "renderer": "splat", "splat_opts": {},
                "azim0": k.get("azim0")}

    def ov(*a, **k):
        seen["overview"] = k
        return tmp_path / "o.png"

    monkeypatch.setattr(percase_render, "_frame_poses", lambda res, robot, frames: (None, poses))
    monkeypatch.setattr(percase_render, "sample_curve", lambda c, s, r: np.zeros((2, 3)))
    monkeypatch.setattr(percase_render, "pointcloud_video", pv)
    monkeypatch.setattr(percase_render, "overview_png", ov)
    monkeypatch.setattr(percase_render, "robot_video", lambda *a, **k: {"video": None, "frames": []})
    monkeypatch.setattr(percase_render, "write_scene_ply", lambda *a, **k: 0)
    monkeypatch.setattr(percase_render, "frame_count", lambda p: 0)
    return seen, tmp_path


class _R:
    def max_radius(self):
        return 0.175


def call(tmp_path, **kw):
    return percase_render.render_outputs(
        "sweeper", tmp_path, pts=None, cols=None, robot=_R(), res={}, s2=None, scene_near=None, z_f=0.0,
        window=[0, 0, 1, 1], start=[0, 0, 0], goal=[1, 1, 0], frames=3, workers=1, view_half=1.75,
        title="t", title_2panel="t2", note="n", **kw)


def test_default_camera_is_unchanged(fakes):
    seen, tmp = fakes
    out, _ = call(tmp)
    assert seen["video"]["azim0"] is None and seen["overview"]["azim"] is None
    assert "elev" not in seen["video"] and "elev" not in seen["overview"]   # their own defaults stand
    assert out["render_3d"]["azim0"] is None


def test_azim_reaches_both_the_video_and_the_overview_and_the_manifest(fakes):
    seen, tmp = fakes
    out, _ = call(tmp, azim0=40.0)
    assert seen["video"]["azim0"] == 40.0 and seen["overview"]["azim"] == 40.0
    assert out["render_3d"]["azim0"] == 40.0


def test_p2_shared_preset_says_the_cylinder_has_no_route_and_fits_one_line():
    c = percase_render.claim_for("planefloor", "p2shared")
    assert "USER-APPROVED MANUAL SCENE EDIT" in c and "D1" in c
    assert "P2-SW-0" in c and "UNKNOWN" in c
    assert len(c) <= 135
