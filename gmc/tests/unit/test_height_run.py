import json

import pytest

from gmc.height.prism import robot_table
from gmc.height.run import compile_and_query, safe_areas, with_overrides
from gmc.synth import single_door

from ..conftest import SMALL_WS, make_cfg


@pytest.fixture(scope="module")
def door_run(tmp_path_factory):
    cfg = make_cfg(theta_min=5e-3, initial_intervals=8)
    out = tmp_path_factory.mktemp("door")
    res, mc = compile_and_query(single_door(0.6, workspace=SMALL_WS),
                                robot_table()["ellipse_toy"], cfg,
                                (-1.7, 0.0, 0.1), (1.7, 0.0, 0.1), out_dir=out)
    return res, mc, out


def test_door_is_reachable_and_independently_verified(door_run):
    res, _, out = door_run
    assert res["status"] == "REACHABLE"
    assert res["verify"]["ran"] and res["verify"]["certified"]
    assert (out / "path.json").is_file()
    assert json.loads((out / "result.json").read_text())["status"] == "REACHABLE"


def test_safe_areas_cover_every_slab(door_run):
    _, mc, _ = door_run
    areas = safe_areas(mc)
    assert len(areas) == len(mc.decomposition.slabs)
    assert all(a["area"] >= 0 for a in areas) and any(a["area"] > 0 for a in areas)


def test_overrides_touch_only_named_fields():
    cfg = make_cfg()
    new = with_overrides(cfg, initial_intervals=1, max_depth=0, max_wall_seconds=999.0)
    assert new.orientation.initial_intervals == 1 and new.orientation.max_depth == 0
    assert new.query.max_wall_seconds == 999.0
    assert new.orientation.theta_min == cfg.orientation.theta_min
    assert new.pair_approx == cfg.pair_approx
