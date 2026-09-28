"""aerial3d with ground bodies (sweeper, cylinder): z-window domain, detour vs thread, compile reuse."""
import json

import numpy as np
import pytest

from gmc.aerial3d.api import (CompileConfig, compile_complex, load_compiled, query, save_compiled)
from gmc.aerial3d.pairs import domain_from_scene
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.trajectory import replay_plan
from aerial3dg_fixtures import CYLINDER, SWEEPER, ground_z, table_corridor

COMPILE_STAGES = {"scene_prepare", "pair_candidates", "envelopes", "octree", "cells", "possible_graph", "audit"}
GROUND = CompileConfig(margin_m=.001)


@pytest.fixture(scope="module")
def corridor():
    scene, queries = table_corridor()
    prep = PreparedScene(scene)
    compiled = {b.name: compile_complex(scene, b, config=GROUND, prepared=prep) for b in (SWEEPER, CYLINDER)}
    return scene, queries, compiled


def test_ground_domain_is_a_thin_slab_at_the_body_centre_height():
    scene, _ = table_corridor()
    for body in (SWEEPER, CYLINDER):
        frame, dom = domain_from_scene(scene, body, margin_m=.001, ground_slab_m=1e-3)
        z = ground_z(body)
        assert dom.bbox_lower[2] == pytest.approx(z - 1e-3, abs=1e-9)
        assert dom.bbox_upper[2] == pytest.approx(z + 1e-3, abs=1e-9)
        assert {"ground:z_window_max", "ground:z_window_min"} <= set(dom.labels)
        # the xy extent is still the known box minus the footprint
        assert dom.bbox_lower[0] == pytest.approx(-2.5 + body.radius_m + .001, abs=1e-6)


def test_ground_domain_needs_a_support_surface():
    scene, _ = table_corridor()
    from dataclasses import replace
    with pytest.raises(ValueError, match="support"):
        domain_from_scene(replace(scene, support=None), SWEEPER, margin_m=.001)


def test_band_prunes_gaussians_the_body_cannot_reach(corridor):
    _, _, compiled = corridor
    # the table top (z .47-.53) is out of the sweeper's reach (top .10 + margin); the floor (top .01)
    # is below both chassis (bottom .02 - margin): neither is a sweeper candidate
    sw = compiled["sweeper"].pairs
    assert not np.any(np.abs(sw.means[:, 2] - .5) < 1e-9)
    assert compiled["sweeper"].pair_stats["pruned_pairs"] == compiled["sweeper"].pair_stats["opacity_selected"]
    cy = compiled["cylinder"].pairs
    assert np.any(np.abs(cy.means[:, 2] - .5) < 1e-9)
    assert not np.any(cy.means[:, 2] < 0)


def test_short_body_threads_under_the_table_tall_body_detours(corridor):
    scene, queries, compiled = corridor
    out = {}
    for body in (SWEEPER, CYLINDER):
        r = query(compiled[body.name], *queries[body.name]["under"])
        assert r["status"] == "REACHABLE", (body.name, r["reason"])
        poly = np.asarray(r["polyline_world"])
        assert np.allclose(poly[:, 2], ground_z(body), atol=1e-12)          # pinned on the manifold
        assert r["verification"]["own"]["status"] == "CERTIFIED"
        assert r["verification"]["shared"]["passed"]
        out[body.name] = (r, poly)
    r_s, p_s = out["sweeper"]
    r_c, p_c = out["cylinder"]
    assert len(p_s) == 2 and r_s["metrics"]["path_length_m"] == pytest.approx(3.6)   # straight, under
    assert p_c[:, 1].min() < -1.0 + 1e-6                                             # around the table
    assert r_c["metrics"]["path_length_m"] > 3.6 + 1.0


def test_exported_ground_result_passes_a_fresh_independent_replay(corridor):
    scene, queries, compiled = corridor
    r = query(compiled["cylinder"], *queries["cylinder"]["under"])
    replay = replay_plan(r["gs3d_result"], GaussianBodyOracle(PreparedScene(scene)))
    assert replay["passed"]
    assert r["gs3d_result"]["robot"]["motion"] == "ground_unicycle"


def test_off_manifold_endpoint_is_unknown_not_planned(corridor):
    _, queries, compiled = corridor
    (s, g) = queries["sweeper"]["under"]
    r = query(compiled["sweeper"], (s[0], s[1], s[2] + .01), g)
    assert r["status"] == "UNKNOWN" and r["reason"] == "start_off_ground_manifold"


def test_repeat_query_reuses_the_compile_and_is_identical(corridor):
    _, queries, compiled = corridor
    c = compiled["cylinder"]
    before = json.dumps(c.timings["records"], sort_keys=True)
    cold = query(c, *queries["cylinder"]["under"], call_id="cold")
    warm = query(c, *queries["cylinder"]["under"], call_id="warm")
    assert warm["polyline_world"] == cold["polyline_world"]
    assert warm["status"] == cold["status"] and warm["compile_id"] == cold["compile_id"]
    assert not COMPILE_STAGES & {t["stage"] for t in warm["timings"]["records"]}
    assert json.dumps(c.timings["records"], sort_keys=True) == before       # no compile was re-run


def test_persisted_compile_reloads_and_answers_identically(corridor, tmp_path):
    _, queries, compiled = corridor
    c = compiled["sweeper"]
    path = tmp_path / "sweeper.a3c"
    meta = save_compiled(c, path)
    assert meta["compile_id"] == c.compile_id and len(meta["sha256"]) == 64
    back = load_compiled(path)
    assert back.compile_id == c.compile_id
    a = query(c, *queries["sweeper"]["under"])
    b = query(back, *queries["sweeper"]["under"])
    assert a["polyline_world"] == b["polyline_world"] and a["status"] == b["status"]
    raw = bytearray(path.read_bytes())
    raw[len(raw) // 2] ^= 0xFF
    path.write_bytes(bytes(raw))
    with pytest.raises(ValueError, match="hash"):
        load_compiled(path)


def test_uav_path_is_unchanged():
    from aerial3d_fixtures import UAV, low_wall
    scene, _ = low_wall()
    frame, dom = domain_from_scene(scene, UAV, margin_m=.05)
    assert not any(l.startswith("ground:") for l in dom.labels)


def test_export_densifies_the_shared_replay_trajectory_without_changing_the_path(corridor):
    from gmc.aerial3d.api import QueryConfig
    _, queries, compiled = corridor
    base = query(compiled["sweeper"], *queries["sweeper"]["under"])
    dense = query(compiled["sweeper"], *queries["sweeper"]["under"], config=QueryConfig(export_max_segment_m=.2))
    assert dense["polyline_world"] == base["polyline_world"]                  # same path, same metrics
    poses = np.asarray(dense["gs3d_result"]["trajectory"]["poses"], float)
    steps = np.linalg.norm(np.diff(poses[:, :3], axis=0), axis=1)
    assert steps.max() <= .2 + 1e-9 and len(poses) > len(base["gs3d_result"]["trajectory"]["poses"])
    assert np.allclose(poses[:, 1], .5) and np.allclose(poses[:, 2], ground_z(SWEEPER))   # collinear, on manifold
    assert dense["verification"]["shared"]["passed"]
    assert dense["gs3d_result"]["diagnostics"]["export_max_segment_m"] == .2
