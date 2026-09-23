"""L1 uav-lamp scene edits: dense leak-free sheets, gap sizes vs body, deterministic build."""
import hashlib
import json

import numpy as np
import pytest

from gmc.gs3d.contracts import GoalRegion, PlannerConfig, Pose3, SceneSpec
from gmc.gs3d.oracle import GaussianBodyOracle
from gmc.gs3d.planner import LatticePlanner
from gmc.gs3d import scene_uavlamp as su
from gmc.height.ply3d import GaussianScene3D
from gs3d_core_fixtures import UAV, KnownBox

IDENTITY = {"origin_world_m": [0., 0., 0.], "world_to_route": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}
MARGIN, RES = .05, .10


def panel(edit_id="p", role="partition", center=(0., 0., 1.), plane="vz", size=(1., 1.),
          spacing=.05, half_thickness=.02):
    return {"edit_id": edit_id, "role": role, "kind": "panel", "center_route": list(center),
            "plane": plane, "size_m": list(size), "spacing_m": spacing,
            "half_thickness_m": half_thickness, "color_rgb": [.5, .5, .5], "opacity": .95,
            "justification": "test", "contact": "test"}


def spec_from(gaussians, lower, upper):
    return SceneSpec("t", gaussians, lower, upper, .3, 2., KnownBox(lower, upper), None,
                     {"coverage_policy": "synthetic"})


def as_scene(*edits):
    rows = [su.edit_gaussians(e, su.Frame.from_dict(IDENTITY)) for e in edits]
    means = np.vstack([r[0] for r in rows])
    covs = np.concatenate([r[1] for r in rows])
    return GaussianScene3D(means, covs, np.full(len(means), .95), np.arange(len(means)))


def test_panel_samples_include_both_edges_on_regular_grid():
    means, covs = su.edit_gaussians(panel(size=(1., .5)), su.Frame.from_dict(IDENTITY))
    assert len(means) == 21 * 11
    assert np.allclose(means[:, 0], 0.)                       # vz panel lies in u = 0
    assert means[:, 1].min() == pytest.approx(-.5) and means[:, 1].max() == pytest.approx(.5)
    assert means[:, 2].min() == pytest.approx(.75) and means[:, 2].max() == pytest.approx(1.25)
    eig = np.linalg.eigvalsh(covs[0])
    assert 2 * np.sqrt(eig[0]) == pytest.approx(.02)           # normal semiaxis = half thickness
    assert 2 * np.sqrt(eig[-1]) == pytest.approx(.05)          # in-plane semiaxis = spacing


def test_panel_union_is_a_solid_slab_without_holes():
    e = panel(size=(.6, .6))
    frame = su.Frame.from_dict(IDENTITY)
    means, covs = su.edit_gaussians(e, frame)
    inv = np.linalg.inv(covs)
    bound = su.panel_min_half_thickness(e)
    assert bound >= .7 * e["half_thickness_m"]
    grid = np.linspace(-.3, .3, 121)                           # includes all between-sample points
    v, z = np.meshgrid(grid, 1. + grid)
    for t in (-bound, 0., bound):
        pts = np.column_stack([np.full(v.size, t), v.ravel(), z.ravel()])
        d = pts[:, None, :] - means[None]
        maha = np.einsum("pni,nij,pnj->pn", d, inv, d).min(axis=1)
        assert maha.max() <= 4. + 1e-9                         # every point inside some 2-sigma support


@pytest.mark.parametrize("v,z", [(0., 1.), (.025, 1.025), (.2749, .9751), (-.3, 1.3)])
def test_body_cannot_cross_panel_between_or_at_samples(v, z):
    g = as_scene(panel(size=(.6, .6)))
    oracle = GaussianBodyOracle(spec_from(g, (-2, -2, -1), (2, 2, 3)))
    report = oracle.edge(Pose3((-.6, v, z)), Pose3((.6, v, z)), UAV, margin_m=MARGIN)
    assert report.occupancy == "occupied"


def gap_scene(gap, axis):
    """Two coplanar vz panels separated by `gap` along v (axis='v') or z (axis='z')."""
    if axis == "v":
        a = panel("a", center=(0, -(gap / 2 + .5), 1.), size=(1., 2.))
        b = panel("b", center=(0, gap / 2 + .5, 1.), size=(1., 2.))
    else:
        a = panel("a", center=(0, 0, 1. - gap / 2 - .5), size=(3., 1.))
        b = panel("b", center=(0, 0, 1. + gap / 2 + .5), size=(3., 1.))
    return spec_from(as_scene(a, b), (-2, -3, -1), (2, 3, 3))


@pytest.mark.parametrize("axis,closed_gap,open_gap", [("v", su.max_closed_gap("v", UAV, MARGIN, RES), .80),
                                                      ("z", su.max_closed_gap("z", UAV, MARGIN, RES), .50)])
def test_gap_rule_closes_below_body_minus_one_cell_and_opens_above(axis, closed_gap, open_gap):
    # Design rule: a gap is 'closed' if it is at least one lattice cell narrower than the body+margin
    # envelope (and the panel half-thicknesses shrink it further). The oracle agrees.
    assert closed_gap == pytest.approx((2 * (UAV.radius_m if axis == "v" else UAV.half_height_m)
                                        + 2 * MARGIN) - RES)
    oracle = GaussianBodyOracle(gap_scene(closed_gap, axis))
    for off in np.linspace(-closed_gap / 2, closed_gap / 2, 5):
        p = (0, off, 1.) if axis == "v" else (0, 0, 1. + off)
        a, b = Pose3((-.6, p[1], p[2])), Pose3((.6, p[1], p[2]))
        assert oracle.edge(a, b, UAV, margin_m=MARGIN).occupancy != "free"
    oracle = GaussianBodyOracle(gap_scene(open_gap, axis))
    assert oracle.edge(Pose3((-.6, 0, 1.)), Pose3((.6, 0, 1.)), UAV, margin_m=MARGIN).occupancy == "free"


def tiny_source(path):
    rng = np.random.default_rng(0)
    means = rng.uniform(-1, 1, (50, 3))
    covs = np.repeat(np.eye(3)[None] * 1e-4, 50, axis=0)
    meta = {"z_floor": -1.0, "ceiling_height_m": 3.0}
    np.savez(path, means=means, covs=covs, opacity=np.full(50, .9), ids=np.arange(50) + 7,
             meta=np.array(json.dumps(meta)))
    return path


def tiny_config(src):
    return {"schema_version": "uavlamp.scene.v1", "scene_id": "t", "source": str(src),
            "frame": {"origin_world_m": [1., 2., -1.], "world_to_route": [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]},
            "edits": [panel("wall", size=(1., 1.)),
                      {**panel("lamp_bottom", role="lamp", plane="uv", center=(0, 0, .5), size=(.3, .6))}],
            "query": {"start_route": [-1, 0, .3], "goal_route": [1, 0, .8],
                      "box_route": {"lower": [-2, -1, 0], "upper": [2, 1, 2]}}}


def test_builder_is_deterministic_and_records_identity(tmp_path):
    src = tiny_source(tmp_path / "src.npz")
    src_hash = hashlib.sha256(src.read_bytes()).hexdigest()
    docs = []
    for k in range(2):
        out, man = tmp_path / f"o{k}.npz", tmp_path / f"o{k}.json"
        docs.append(su.build_uavlamp_derivative(tiny_config(src), out, man))
    a, b = docs
    assert a["derivative"]["sha256"] == b["derivative"]["sha256"]
    assert a["source"]["sha256"] == src_hash and a["source_unchanged_after_build"]
    assert [e["id_range"] for e in a["edits"]] == [[57, 57 + 21 * 21 - 1], [57 + 441, 57 + 441 + 7 * 13 - 1]]
    assert a["edits"][1]["role"] == "lamp" and a["edits"][1]["count"] == 7 * 13
    assert a["query"]["start_route"] == [-1, 0, .3]
    with np.load(tmp_path / "o0.npz") as d:
        assert len(d["ids"]) == 50 + 441 + 91
        assert np.array_equal(d["means"][:50], np.load(src)["means"])     # source rows untouched
        world = d["means"][50]                                            # frame applied
    first_local = su.Frame.from_dict(tiny_config(src)["frame"]).to_route(world[None])[0]
    assert first_local == pytest.approx([0., -.5, .5])
    scene, doc = su.load_uavlamp_derivative(tmp_path / "o0.npz", tmp_path / "o0.json")
    assert len(scene) == 582 and doc["scene_id"] == "t"


def test_loader_rejects_hash_mismatch(tmp_path):
    src = tiny_source(tmp_path / "src.npz")
    su.build_uavlamp_derivative(tiny_config(src), tmp_path / "o.npz", tmp_path / "o.json")
    doc = json.loads((tmp_path / "o.json").read_text())
    doc["derivative"]["sha256"] = "0" * 64
    (tmp_path / "o.json").write_text(json.dumps(doc))
    with pytest.raises(ValueError):
        su.load_uavlamp_derivative(tmp_path / "o.npz", tmp_path / "o.json")


def opening_slice(plug=False):
    """Synthetic corridor: walls at v=+-.5, drop ceiling at z=1.8, a lamp box spanning the corridor
    with underside .8 and a bulkhead from the lamp top to the ceiling. Coarse .1 m panels keep it fast."""
    sp = dict(spacing=.1)
    edits = [panel("wl", center=(0, -.5, .9), plane="uz", size=(2.4, 1.8), **sp),
             panel("wr", center=(0, .5, .9), plane="uz", size=(2.4, 1.8), **sp),
             panel("ceil", role="drop_ceiling", center=(0, 0, 1.8), plane="uv", size=(2.4, 1.), **sp),
             panel("floor", role="floor", center=(0, 0, 0.), plane="uv", size=(2.4, 1.), **sp)]
    edits += su.lamp_box_edits("lamp", center_uv=(0., 0.), size_uv=(.3, 1.), underside_z=.8,
                               height=.15, spacing=.1, half_thickness=.02)
    edits.append(panel("bulk", role="bulkhead", center=(0, 0, (.95 + 1.8) / 2), plane="vz",
                       size=(1., 1.8 - .95), **sp))
    if plug:
        edits.append(panel("plug", role="plug", center=(0, 0, .4), plane="vz", size=(1., .8), **sp))
    return spec_from(as_scene(*edits), (-1.2, -.5, 0.), (1.2, .5, 1.8))


def test_opening_slice_single_query_goes_under_and_plug_exhausts():
    start, goal = Pose3((-.8, 0, 1.35)), GoalRegion(Pose3((.8, 0, 1.35)))
    result = LatticePlanner().plan(opening_slice(), UAV, start, goal, PlannerConfig(resolution_m=RES))
    assert result["status"] == "success", result["reason"]
    rows = np.asarray(result["trajectory"]["poses"])
    under = rows[np.abs(rows[:, 0]) <= .15]
    assert len(under) and under[:, 2].max() <= .8 - UAV.half_height_m - MARGIN + 1e-9
    plugged = LatticePlanner().plan(opening_slice(plug=True), UAV, start, goal, PlannerConfig(resolution_m=RES))
    assert plugged["status"] in ("no_path_on_lattice", "verification_failed")
    assert plugged["reason"] in ("reachable_lattice_exhausted", "reachable_frontier_has_unproven_edges")
