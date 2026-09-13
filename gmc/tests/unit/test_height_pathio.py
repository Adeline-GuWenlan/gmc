import numpy as np
import pytest
from shapely.geometry import box

from gmc.height.pathio import (crossing_thetas, curve_from_dict, curve_to_dict,
                               load_path_json, path_crosses, polyline_xy,
                               sample_curve, save_path_json)

DOOR_PATH = {"schema_version": 2, "query_id": "q_1788388783567711000",
             "segments": [
                 {"kind": "ROTATION", "q0": [-2.0, 0.0, 0.3],
                  "q1": [-2.0, 0.0, 0.19634954084936207],
                  "control_points": [], "certificate_ids": ["a"]},
                 {"kind": "TRANSLATION", "q0": [-2.0, 0.0, 0.19634954084936207],
                  "q1": [2.0, 0.0, 0.19634954084936207],
                  "control_points": [], "certificate_ids": ["b"]},
                 {"kind": "ROTATION", "q0": [2.0, 0.0, 0.19634954084936207],
                  "q1": [2.0, 0.0, 0.3], "control_points": [],
                  "certificate_ids": ["c"]}]}


def test_round_trip_matches_artifact_schema(tmp_path):
    curve = curve_from_dict(DOOR_PATH)
    assert curve_to_dict(curve, DOOR_PATH["query_id"]) == DOOR_PATH
    save_path_json(curve, tmp_path / "path.json", "q")
    again = load_path_json(tmp_path / "path.json")
    assert curve_to_dict(again, "q")["segments"] == DOOR_PATH["segments"]


def test_local_steering_is_rejected():
    bad = {"schema_version": 2, "segments": [dict(DOOR_PATH["segments"][0],
                                                  kind="LOCAL_STEERING")]}
    with pytest.raises(ValueError, match="LOCAL_STEERING"):
        curve_from_dict(bad)


def test_sampling_bounds_footprint_motion():
    curve = curve_from_dict(DOOR_PATH)
    poses = sample_curve(curve, max_step=0.01, radius=0.5)
    np.testing.assert_allclose(poses[0], [-2.0, 0.0, 0.3])
    np.testing.assert_allclose(poses[-1], [2.0, 0.0, 0.3])
    step = (np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
            + np.abs(np.diff(poses[:, 2])) * 0.5)
    assert step.max() <= 0.01 + 1e-12
    assert len(poses) >= 400


def test_polyline_crossing_and_thetas():
    curve = curve_from_dict(DOOR_PATH)
    assert polyline_xy(curve).shape == (4, 2)
    assert path_crosses(curve, box(-0.3, -0.6, 0.3, 0.6))
    assert not path_crosses(curve, box(-0.3, 0.5, 0.3, 0.6))
    assert crossing_thetas(curve, 0.0) == [pytest.approx(0.19634954084936207)]
