import json

import numpy as np

from gmc.gs3d.scene import (LEVEL, ManualGaussian, _candidate_enclosure, _cov_from_route_axes,
                            build_showcase_derivative, load_showcase_derivative, manual_edits,
                            route_frame)
from gmc.height.planefloor import save_plane_scene
from gmc.height.ply3d import GaussianScene3D


def _tiny_source(tmp_path):
    zf = -1.2271749593107995
    origin, _ = route_frame(zf)
    # A tiny valid source with a table-like top away from the light crossing.
    s = GaussianScene3D(np.array([origin + [1.4, 0., .90], origin + [2.8, 2.8, .1]]),
                        np.repeat(np.eye(3)[None] * .0025, 2, axis=0), np.array([.95, .95]),
                        np.array([10, 11]), "tiny")
    p = tmp_path / "source.npz"
    save_plane_scene(p, s, {"z_floor": zf, "floor": {}, "ceiling_height_m": 5.31})
    return p


def test_manual_edit_covariances_keep_route_axes_and_are_spd():
    zf = -1.2271749593107995
    _, R = route_frame(zf)
    for edit in manual_edits(zf):
        cov = _cov_from_route_axes(edit.semiaxes_route_m, R)
        assert np.all(np.linalg.eigvalsh(cov) > 0)
        assert np.allclose(np.linalg.eigvalsh(R @ cov @ R.T), np.sort((np.array(edit.semiaxes_route_m) / LEVEL) ** 2))


def test_derivative_manifest_and_archive_use_identical_manual_rows(tmp_path):
    source = _tiny_source(tmp_path)
    out, manifest = tmp_path / "edited.npz", tmp_path / "edited.json"
    doc = build_showcase_derivative(source, out, manifest)
    scene, loaded = load_showcase_derivative(out, manifest)
    assert len(scene) == 5
    assert doc["derivative"]["sha256"] == loaded["derivative"]["sha256"]
    assert loaded["source_unchanged_after_build"] is True
    assert all(x["manual_edit"] for x in loaded["manual_geometry"])
    assert loaded["existing_table_arrangement"]["manual_edit"] is False
    assert json.loads(manifest.read_text())["scene_id"] == "showcase_planefloor_airborne_v1"
