import numpy as np

from gmc.height.pathio import curve_from_dict
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.height.replay3d import replay_curve
from gmc.height.synth3d import GOAL, START, WORKSPACE, Z_FLOOR, table_scene


def map_ids(variant, robot_key, z_c=1.20):
    s3, meta = table_scene(variant)
    s2, _ = project_scene(s3, robot_table(z_c)[robot_key], WORKSPACE, z_floor=Z_FLOOR)
    return {s.primitive_id for s in s2.supports}, meta["groups"]


def test_scene_boundaries_match_the_spec():
    s3, meta = table_scene("closed")
    top = s3.subset(np.isin(s3.ids, meta["groups"]["tabletop"]))
    lo, hi = top.aabb(2.0)
    np.testing.assert_allclose([lo[:, 0].min(), lo[:, 1].min(), hi[:, 0].max(), hi[:, 1].max()],
                               [-0.3, -0.6, 0.3, 0.6], atol=1e-12)
    np.testing.assert_allclose([lo[:, 2].min(), hi[:, 2].max()], [0.72, 0.76], atol=1e-12)
    legs = s3.subset(np.isin(s3.ids, meta["groups"]["legs"]))
    llo, lhi = legs.aabb(2.0)
    np.testing.assert_allclose([llo[:, 2].min(), lhi[:, 2].max()], [0.0, 0.72], atol=1e-12)
    wall_open = table_scene("open")[1]["groups"]["wall"]
    assert len(meta["groups"]["wall"]) > len(wall_open)


def test_T2_7_maps_by_provenance():
    ids, g = map_ids("closed", "sweeper")
    assert ids & set(g["legs"]) and not ids & set(g["tabletop"])
    ids, g = map_ids("closed", "cylinder")
    assert ids & set(g["legs"]) and ids & set(g["tabletop"])
    ids, g = map_ids("closed", "uav")
    assert ids & set(g["wall"]) and not ids & (set(g["tabletop"]) | set(g["legs"]))


def test_T2_6_replay_rejects_cylinder_through_the_table():
    s3, _ = table_scene("closed")
    line = {"schema_version": 2, "segments": [
        {"kind": "TRANSLATION", "q0": list(START), "q1": list(GOAL)}]}
    rep = replay_curve(s3, robot_table()["cylinder"], curve_from_dict(line), z_floor=Z_FLOOR)
    assert not rep["passed"]
    top_or_wall = {c["splat_id"] for c in rep["collisions"]}
    assert top_or_wall


def test_sweeper_straight_line_under_the_table_replays_clean():
    s3, _ = table_scene("closed")
    line = {"schema_version": 2, "segments": [
        {"kind": "TRANSLATION", "q0": list(START), "q1": list(GOAL)}]}
    rep = replay_curve(s3, robot_table()["sweeper"], curve_from_dict(line), z_floor=Z_FLOOR)
    assert rep["passed"], rep["worst"]
