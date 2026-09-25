"""Synthetic acceptance (uav.md §17 strictness tests + §18 U0 benchmark rows).

Every case is ONE query (start + goal only) on a compiled complex; every REACHABLE
is additionally replayed with a fresh baseline ``gs3d`` oracle on a fresh index.
"""
import subprocess
import sys

import numpy as np
import pytest

from gmc.aerial3d.api import compile_complex, query
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.trajectory import replay_plan, sample_linear_trajectory
import aerial3d_fixtures as F

M = .05
_CACHE = {}
# proven minimum half thickness of a fixture panel slab (grid step <= spacing in both axes)
HALF_T_MIN = F.HALF_T * np.sqrt(1 - .5)


def run(case, body=F.UAV, name=None, **kw):
    key = (case.__name__, tuple(sorted(kw.items())), body.name)
    if key not in _CACHE:
        scene, queries = case(**kw)
        _CACHE[key] = (scene, queries, compile_complex(scene, body))
    scene, queries, compiled = _CACHE[key]
    start, goal = queries[name or next(iter(queries))]
    r = query(compiled, start, goal)
    if r["status"] == "REACHABLE":
        fresh = replay_plan(r["gs3d_result"], GaussianBodyOracle(PreparedScene(scene)))
        assert fresh["passed"], fresh["geometry"]["reason"]
        assert r["verification"]["own"]["status"] == "CERTIFIED"
        assert r["clearance_lower_m"] > M
        assert np.allclose(r["polyline_world"][0], start) and np.allclose(r["polyline_world"][-1], goal)
    return r, compiled, scene


def samples(r, dt=.02):
    s = sample_linear_trajectory(r["gs3d_result"]["trajectory"], dt_s=dt)
    return np.asarray(s["poses"])[:, :3]


def test_open_vertical_ascent_climbs_straight():
    r, _, _ = run(F.open_ascent)
    assert r["status"] == "REACHABLE"
    P = np.asarray(r["polyline_world"])
    assert len(P) == 2 and np.ptp(P[:, 2]) == pytest.approx(2.3)


def test_low_wall_is_overflown():
    r, _, _ = run(F.low_wall)
    assert r["status"] == "REACHABLE"
    X = samples(r)
    at_wall = X[np.abs(X[:, 0]) < F.UAV.radius_m + .1]
    wall_top = 1.0 + F.HALF_T
    assert len(at_wall) and np.all(at_wall[:, 2] - F.UAV.half_height_m - M >= wall_top - 1e-6)


def test_closed_full_height_wall_is_certified_unreachable():
    r, _, _ = run(F.full_wall, gap=False)
    assert r["status"] == "UNREACHABLE" and r["reason"] == "possible_space_cut"
    assert r["certificate"]["cut_traceable_fraction"] == 1.0


def test_full_height_wall_with_gap_is_passed_around():
    r, _, _ = run(F.full_wall, gap=True)
    assert r["status"] == "REACHABLE"
    X = samples(r)
    crossing = X[np.abs(X[:, 0]) < .05]
    # the wall's supports end at y <= .1 + spacing: the body disc passes wholly beyond them
    assert len(crossing) and np.all(crossing[:, 1] - F.UAV.radius_m >= .1 + F.SPACING - 1e-6)


def test_overhang_is_not_penetrated():
    r, _, _ = run(F.overhang)
    assert r["status"] == "REACHABLE"
    X = samples(r)
    # Over x <= .5 the slab is a hole-free sheet at least t_min thick (panel_min_half_thickness);
    # a body disc reaching into that region must be wholly below or above the sheet.
    t_min = HALF_T_MIN
    under_slab = X[:, 0] - F.UAV.radius_m <= .5
    body_lo, body_hi = X[:, 2] - F.UAV.half_height_m, X[:, 2] + F.UAV.half_height_m
    ok = (body_hi + M <= 1.5 - t_min) | (body_lo - M >= 1.5 + t_min)
    assert under_slab.any() and np.all(ok[under_slab])
    assert X[:, 0].max() > .5 + F.SPACING + F.UAV.radius_m  # flies out past the edge


def test_bridge_gives_two_distinct_z_routes_at_the_same_xy():
    under, compiled, _ = run(F.bridge, name="under")
    over, _, _ = run(F.bridge, name="over")
    assert under["status"] == over["status"] == "REACHABLE"
    zu = samples(under)[np.abs(samples(under)[:, 0]) < .05][:, 2]
    zo = samples(over)[np.abs(samples(over)[:, 0]) < .05][:, 2]
    assert len(zu) and len(zo) and zu.max() < 1.45 < zo.min()
    # both crossings are at |y| < .2 (the openings), i.e. the same xy column
    for r in (under, over):
        X = samples(r)
        assert np.all(np.abs(X[np.abs(X[:, 0]) < .05][:, 1]) < .2)
    assert set(under["graph_path"]["cell_ids"]).isdisjoint(over["graph_path"]["cell_ids"])


def test_vertical_shaft_is_climbed_through_the_hole():
    r, _, _ = run(F.shaft)
    assert r["status"] == "REACHABLE"
    X = samples(r)
    level = X[np.abs(X[:, 2] - 2.) < .05]
    assert len(level) and np.all((level[:, 0] > .45) & (level[:, 1] > .45))


def test_two_uav_sizes_take_different_routes():
    small, _, _ = run(F.window_wall, body=F.SMALL_UAV)
    big, _, _ = run(F.window_wall, body=F.BIG_UAV)
    assert small["status"] == big["status"] == "REACHABLE"
    ys = samples(small)[np.abs(samples(small)[:, 0]) < .03][:, 1]
    yb = samples(big)[np.abs(samples(big)[:, 0]) < .03][:, 1]
    assert np.all(np.abs(ys) < .3)      # small: through the narrow window A
    assert np.all(yb > .55)             # big: through the large window B
    assert big["metrics"]["path_length_m"] > small["metrics"]["path_length_m"]


def test_codex_under_over_fixture_in_one_query():
    r, _, _ = run(F.under_over_f1)
    assert r["status"] == "REACHABLE"
    X = samples(r)
    assert X[np.abs(X[:, 0] + .7) < .02][:, 2].max() < .856
    assert X[np.abs(X[:, 0] - .8) < .02][:, 2].min() > 1.395


def _under_lamp(X):
    lamp_x, underside = (-.15, .15), .9
    over_lamp = (X[:, 0] + F.UAV.radius_m > lamp_x[0]) & (X[:, 0] - F.UAV.radius_m < lamp_x[1])
    return over_lamp, X[:, 2] + F.UAV.half_height_m + M <= underside + 1e-6


@pytest.mark.parametrize("name", ["low_start", "high_start"])
def test_mini_booth_route_passes_under_the_lamp(name):
    r, _, _ = run(F.mini_booth, name=name)
    assert r["status"] == "REACHABLE"
    X = samples(r)
    footprint, under = _under_lamp(X)
    assert footprint.any() and np.all(under[footprint])


@pytest.mark.parametrize("name", ["low_start", "high_start"])
def test_mini_booth_plug_is_certified_unreachable(name):
    r, _, _ = run(F.mini_booth, plug=True, name=name)
    assert r["status"] == "UNREACHABLE" and r["reason"] == "possible_space_cut"
    cert = r["certificate"]
    assert cert["blocked_leaves_on_cut"] > 0 and cert["cut_traceable_fraction"] == 1.0


def test_graph_boundaries_trace_to_pair_ids():
    r, compiled, _ = run(F.mini_booth, name="low_start")
    for cell in r["graph_path"]["cells"]:
        k = cell["facets_by_kind"]
        assert k["pair"] + k["domain"] + k["box"] == cell["facets"]
        assert cell["kind"] != "support_plane" or k["pair"] > 0
    t = compiled.cells.traceability()
    assert t["facets_without_provenance"] == 0
    assert compiled.boundary_traceability["traceable_fraction"] == 1.0


def test_backend_never_loads_mesh_voxel_esdf_or_projection_code():
    code = r"""
import sys
sys.path[:0] = ['src', 'experiments']
before = set(sys.modules)
import types
trap = types.ModuleType('gmc.height.project')
def forbidden(*a, **k): raise AssertionError('projection called')
trap.project_scene = forbidden
sys.modules['gmc.height.project'] = trap   # any use of the 2-D projection would raise
from gmc.aerial3d.api import compile_complex, query
import aerial3d_fixtures as F
scene, q = F.low_wall()
r = query(compile_complex(scene, F.UAV), *q['over'])
assert r['status'] == 'REACHABLE', r['reason']
loaded = set(sys.modules) - before - {'gmc.height.project'}
bad = sorted(m for m in loaded if any(t in m.lower() for t in
      ('mesh', 'voxel', 'esdf', 'sdf', 'trimesh', 'open3d', 'shapely', 'slice_compiler',
       'height.project', 'heightmap', 'occupancy')))
assert not bad, bad
print('OK', len(loaded))
"""
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=600)
    assert out.returncode == 0, out.stderr[-2000:]
    assert out.stdout.startswith("OK")
