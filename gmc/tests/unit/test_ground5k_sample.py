"""Ground-5k sampler (G1 Task 1) on a small synthetic floor.

Two rooms split by a wall at x = 5 with an optional 1.6 m door; floor observed only for
x < 9, so the strip x in [9, 10] is unknown floor.  Every filter the rules require is
exercised: uniform draws, the 3 m separation, (a) known floor, (b) certified-free
endpoints, (c) certified lattice connectivity, rejection counts, seed and shuffle.
"""
import numpy as np
import pytest
from scipy import stats

from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.height.ply3d import GaussianScene3D

import ground5k_common as gc
import ground5k_sample as gs

EXTENT = [0.0, 0.0, 10.0, 6.0]
FLAT = {"z_floor": 0.0, "normal": [0.0, 0.0, 1.0], "centroid": [5.0, 3.0, 0.0], "extent": EXTENT,
        "ceiling_height_m": 3.0}
CFG = {"tau": 0.3, "level": 2.0, "margin_m": 0.001,
       "contact": {"max_height_error_m": 0.05, "max_travel_m": 0.05, "max_slope_deg": 5.0},
       "crop": {"z_below_floor_m": 0.10, "z_above_floor_m": 2.0, "pad_m": 1.0},
       "sampler": {"min_sep_m": 3.0, "lattice_m": 0.25,
                   "evidence": {"cell_m": 0.10, "tau": 0.3, "z_lo_rel_fitted_m": -0.30,
                                "z_hi_rel_fitted_m": 0.10, "min_count": 1}}}


def _floor_splats(x_max=9.0):
    g = np.arange(0.025, x_max, 0.05)
    h = np.arange(0.025, 6.0, 0.05)
    X, Y = np.meshgrid(g, h, indexing="ij")
    means = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, -0.05)])
    return means, np.full(len(means), 0.9)


def _wall_scene(door=True):
    ys = np.arange(0.05, 6.0, 0.1)
    if door:
        ys = ys[(ys < 2.2) | (ys > 3.8)]
    zs = np.arange(0.1, 2.0, 0.2)
    Y, Z = np.meshgrid(ys, zs, indexing="ij")
    means = np.column_stack([np.full(Y.size, 5.0), Y.ravel(), Z.ravel()])
    covs = np.repeat(np.diag([0.06 ** 2] * 3)[None], len(means), 0)
    return GaussianScene3D(means, covs, np.full(len(means), 0.9), np.arange(len(means)), "synth")


def _empty_scene():
    return GaussianScene3D(np.empty((0, 3)), np.empty((0, 3, 3)), np.empty(0),
                           np.empty(0, dtype=np.int64), "empty")


def _hall(scene3d):
    means, opacity = _floor_splats()
    mask, _ = gs.floor_evidence(means, opacity, FLAT, EXTENT, CFG["sampler"]["evidence"])
    region = gs.hall_region(mask, EXTENT, CFG, FLAT)
    spec = gs.hall_scene(scene3d, region, FLAT, CFG, "synthetic-hall")
    prepared = PreparedScene(spec)
    oracles = {k: GaussianBodyOracle(prepared) for k in gc.BODIES}
    return region, oracles


def _witnesses(region, oracles):
    return {k: gs.build_witness(oracles[k], b, FLAT, region, EXTENT, 0.25, CFG["margin_m"])
            for k, b in gc.BODIES.items()}


@pytest.fixture(scope="module")
def door_world():
    region, oracles = _hall(_wall_scene(door=True))
    return region, oracles, _witnesses(region, oracles)


@pytest.fixture(scope="module")
def closed_world():
    region, oracles = _hall(_wall_scene(door=False))
    return region, oracles, _witnesses(region, oracles)


# ----------------------------------------------------------------- floor evidence --

def test_floor_evidence_marks_only_cells_with_floor_layer_splats():
    means, opacity = _floor_splats()
    high = np.array([[9.5, 3.0, 1.0]])            # an object splat, not floor
    faint = np.array([[9.5, 1.0, -0.05]])         # below the opacity cut
    means = np.vstack([means, high, faint])
    opacity = np.r_[opacity, 0.9, 0.1]
    mask, st = gs.floor_evidence(means, opacity, FLAT, EXTENT, CFG["sampler"]["evidence"])
    assert mask.shape == (100, 60)
    assert mask[:90].all() and not mask[90:].any()
    assert st["observed_cells"] == 90 * 60


def test_floor_evidence_marks_the_footprint_of_each_floor_splat():
    ev = {**CFG["sampler"]["evidence"], "footprint_level": 2.0, "max_half_extent_m": 0.5}
    means = np.array([[2.05, 2.05, -0.05], [7.05, 3.05, -0.05]])
    covs = np.array([np.diag([0.1 ** 2, 0.05 ** 2, 1e-4]),       # 2-sigma box 0.4 x 0.2 m
                     np.diag([0.4 ** 2, 0.4 ** 2, 1e-4])])       # 0.8 m half-extent: ignored
    mask, st = gs.floor_evidence(means, np.full(2, 0.9), FLAT, EXTENT, ev, covs=covs)
    assert mask[18:23, 19:22].all()                # x 1.85..2.25, y 1.95..2.15
    assert mask.sum() == 5 * 3
    assert st["too_large_ignored"] == 1


def test_floor_evidence_follows_the_tilted_fitted_plane():
    tilt = {**FLAT, "normal": [-0.01, 0.0, 1.0]}   # fitted z = 0.01 (x - 5)
    means = np.array([[1.0, 1.0, -0.04 - 0.05], [9.0, 1.0, 0.04 - 0.05], [9.0, 2.0, -0.04 - 0.35]])
    mask, _ = gs.floor_evidence(means, np.full(3, 0.9), tilt, EXTENT, CFG["sampler"]["evidence"])
    assert mask[10, 10] and mask[90, 10] and not mask[90, 20]


# ------------------------------------------------------------------- regions --

def test_contact_strip_matches_fitted_plane_deviation():
    tilt = {**FLAT, "normal": [-0.01, 0.002, 1.0]}
    s = gc.ContactStrip.from_floor(tilt, 0.05)
    n = np.array(tilt["normal"])
    for x, y in [(0, 0), (10, 6), (2.5, 4)]:
        fitted = 0.0 - (n[0] * (x - 5) + n[1] * (y - 3)) / n[2]
        assert s.dev(x, y) == pytest.approx(fitted)
        inside = all(ax * x + ay * y <= b + 1e-12 for ax, ay, b in s.halfplanes())
        assert inside == (abs(fitted) <= 0.05)


def test_region_counts_the_face_that_refused_a_box():
    r = gc.Region.from_box((0, 0, 4, 3), (-0.1, 2.0))
    assert r.contains_aabb((1, 1, 0), (2, 2, 1))
    assert not r.contains_aabb((3.5, 1, 0), (4.2, 2, 1))
    assert not r.contains_aabb((1, 1, 0), (2, 2, 2.5))
    assert r.rejections["x_max"] == 1 and r.rejections["z"] == 1 and r.rejections["x_min"] == 0


def test_region_polygon_erosion_keeps_the_body_square_inside():
    tilt = {**FLAT, "normal": [-0.02, 0.0, 1.0]}     # strip |0.02 (x-5)| <= 0.05 -> x in [2.5, 7.5]
    r = gc.Region.from_box((0, 0, 10, 6), (-0.1, 2.0), gc.ContactStrip.from_floor(tilt, 0.05))
    poly = r.polygon(erode=0.3)
    x0, y0, x1, y1 = poly.bounds
    assert (x0, y0, x1, y1) == pytest.approx((2.8, 0.3, 7.2, 5.7), abs=1e-4)   # strip inset 1e-6 m in z
    v = r.vertices()
    assert v[:, 0].min() == pytest.approx(2.5, abs=1e-4) and v[:, 0].max() == pytest.approx(7.5, abs=1e-4)


def test_support_accepts_a_region_clipped_by_the_contact_strip():
    tilt = {**FLAT, "normal": [-0.0049393908, -0.0005859098, 0.9999876295], "centroid": [8.1205873014, 19.3886684820, 0.0],
            "extent": [-4.43357959985733, -0.9637084349989891, 21.088036155700674, 37.84499740600586]}
    for box in [tilt["extent"], (-3.0, 2.0, 12.0, 30.0), (10.0, 20.0, 21.0, 37.0)]:
        r = gc.query_region(box, CFG, tilt)
        err = gc.region_height_error(r, tilt, CFG)
        assert err <= 0.05
        gc.make_support(r, tilt, CFG, err)          # the gs3d constructor must accept it


def test_raster_region_rejects_a_box_touching_an_unobserved_cell(door_world):
    region = door_world[0]
    assert region.contains_aabb((8.0, 2.0, 0.0), (8.95, 2.5, 1.0))
    assert not region.contains_aabb((8.5, 2.0, 0.0), (9.05, 2.5, 1.0))


# ------------------------------------------------------------------ witness --

def test_witness_joins_both_rooms_through_the_door(door_world):
    region, oracles, wit = door_world
    for name in gc.BODIES:
        _, labels = gs.witness_graph(wit[name])
        ny = len(wit[name]["ys"])
        west = labels[np.searchsorted(wit[name]["xs"], 2.0) * ny + np.searchsorted(wit[name]["ys"], 1.0)]
        east = labels[np.searchsorted(wit[name]["xs"], 8.0) * ny + np.searchsorted(wit[name]["ys"], 5.0)]
        assert west == east, name


def test_witness_keeps_the_rooms_apart_behind_a_closed_wall(closed_world):
    region, oracles, wit = closed_world
    for name in gc.BODIES:
        _, labels = gs.witness_graph(wit[name])
        ny = len(wit[name]["ys"])
        west = labels[np.searchsorted(wit[name]["xs"], 2.0) * ny + np.searchsorted(wit[name]["ys"], 3.0)]
        east = labels[np.searchsorted(wit[name]["xs"], 8.0) * ny + np.searchsorted(wit[name]["ys"], 3.0)]
        assert west != east, name


def test_witness_nodes_are_certified_free_and_on_known_floor(door_world):
    region, oracles, wit = door_world
    w = wit["cylinder"]
    X, Y = np.meshgrid(w["xs"], w["ys"], indexing="ij")
    free = w["free"]
    near_wall = np.abs(X[free] - 5.0) < 0.3 + 0.12              # within the wall's reach ...
    assert np.all((Y[free][near_wall] > 2.2) & (Y[free][near_wall] < 3.8))   # ... only in the door
    assert not np.any(X[free] > 9.0 - 0.3)                      # none over unknown floor
    assert free.sum() > 100


# ------------------------------------------------------------------ sampling --

def _sample(world, n, seed):
    region, oracles, wit = world
    return gs.sample_pairs(n, np.random.default_rng(seed), EXTENT, region, oracles, wit, FLAT, CFG)


def test_pairs_obey_separation_known_floor_and_free_endpoints(door_world):
    pairs, counts = _sample(door_world, 40, 1)
    region, oracles, _ = door_world
    assert len(pairs) == 40
    for p in pairs:
        assert p["sep_m"] >= 3.0
        assert np.hypot(*(p["goal"] - p["start"])) == pytest.approx(p["sep_m"])
        for e in ("start", "goal"):
            x, y = p[e]
            assert x <= 9.0 - 0.3
            assert abs(x - 5.0) >= 0.3 + 0.12 or 2.2 < y < 3.8
        for name in gc.BODIES:
            assert p["clearance"][name]["start"] > CFG["margin_m"]
            w = p["witness"][name]
            assert np.allclose(w["polyline"][0], p["start"]) and np.allclose(w["polyline"][-1], p["goal"])


def test_rejection_counts_add_up(door_world):
    pairs, c = _sample(door_world, 40, 2)
    accepted_endpoints = 2 * c["pair_draws"]
    assert c["endpoint_draws"] == (c["a_unknown_floor"] + c["b_not_free_sweeper"]
                                   + c["b_not_free_cylinder"] + accepted_endpoints)
    assert c["pair_draws"] == (c["sep_below_min"] + c["c_unattached_sweeper"] + c["c_disconnected_sweeper"]
                               + c["c_unattached_cylinder"] + c["c_disconnected_cylinder"] + c["accepted"])
    assert c["accepted"] == len(pairs) == 40
    assert c["a_unknown_floor"] > 0 and c["b_not_free_sweeper"] + c["b_not_free_cylinder"] > 0


def test_closed_wall_rejects_every_cross_room_pair(closed_world):
    pairs, c = _sample(closed_world, 30, 3)
    assert c["c_disconnected_sweeper"] + c["c_disconnected_cylinder"] > 0
    for p in pairs:
        assert (p["start"][0] < 5.0) == (p["goal"][0] < 5.0)


def test_same_seed_same_pairs_other_seed_other_pairs(door_world):
    a, _ = _sample(door_world, 10, 7)
    b, _ = _sample(door_world, 10, 7)
    c, _ = _sample(door_world, 10, 8)
    assert all(np.array_equal(p["start"], q["start"]) for p, q in zip(a, b))
    assert not all(np.array_equal(p["start"], q["start"]) for p, q in zip(a, c))


def test_endpoint_draws_are_uniform_over_the_admissible_floor():
    region, oracles = _hall(_empty_scene())
    rng = np.random.default_rng(11)
    counts = {k: 0 for k in ("endpoint_draws", "a_unknown_floor", "b_not_free_sweeper", "b_not_free_cylinder")}
    xy = np.array([gs.draw_endpoint(rng, EXTENT, region, oracles, FLAT, CFG["margin_m"], counts)[0]
                   for _ in range(400)])
    # admissible for the cylinder's 0.6 m box on 0.1 m cells: x in [0.3, 8.7], y in [0.3, 5.7]
    assert xy[:, 0].min() >= 0.3 - 1e-9 and xy[:, 0].max() <= 8.7 + 1e-9
    assert stats.kstest((xy[:, 0] - 0.3) / 8.4, "uniform").pvalue > 0.01
    assert stats.kstest((xy[:, 1] - 0.3) / 5.4, "uniform").pvalue > 0.01


def test_finalized_pairs_are_shuffled_with_crops_holding_endpoints_and_witnesses(door_world):
    pairs, _ = _sample(door_world, 12, 4)
    rows, polylines = gs.finalize_pairs(pairs, np.random.default_rng(5), FLAT, CFG, "t")
    assert [r["pair_id"] for r in rows] == [f"t-{k:05d}" for k in range(12)]
    assert sorted(tuple(r["start"]) for r in rows) == sorted(tuple(np.round(p["start"], 6)) for p in pairs)
    assert [tuple(r["start"]) for r in rows] != [tuple(np.round(p["start"], 6)) for p in pairs]
    for r in rows:
        x0, y0, x1, y1 = r["crop"]["box_xy"]
        pts = np.vstack([r["start"], r["goal"], *polylines[r["pair_id"]].values()])
        assert np.all(pts[:, 0] >= x0 + 1.0 - 1e-6) or x0 == EXTENT[0]
        assert np.all((pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (pts[:, 1] >= y0) & (pts[:, 1] <= y1))
        assert set(r["witness"]) == {"sweeper", "cylinder"}
        assert r["floor_z"]["start"]["support_z"] == 0.0


# ------------------------------------------------ staircase query region (pilot fix) --

TILTED = {**FLAT, "normal": [-0.02, -0.004, 1.0], "centroid": [5.0, 3.0, 0.0]}


def _stair(box=(0.0, 0.0, 10.0, 6.0), floor=TILTED):
    return gc.query_region(box, CFG, floor)


def test_query_region_is_rectilinear_so_gmc_overlays_stay_exact():
    """Pilot: GMC's I3 graph-nesting check failed on 5 of 7 regions cut by the slanted
    contact-strip edge and on none of 43 boxes.  The region's edges are now axis-aligned."""
    poly = _stair().polygon(erode=0.3)
    xy = np.asarray(poly.exterior.coords)
    d = np.diff(xy, axis=0)
    assert np.all((np.abs(d[:, 0]) < 1e-12) | (np.abs(d[:, 1]) < 1e-12))


def test_staircase_lies_inside_the_contact_strip_and_hugs_it():
    r = _stair()
    strip = gc.ContactStrip.from_floor(TILTED, 0.05)
    v = r.vertices()
    assert np.all(np.abs(strip.dev(v[:, 0], v[:, 1])) <= 0.05)
    exact = gc.Region.from_box((0, 0, 10, 6), r.z, strip).polygon()
    lost = exact.area - r.polygon().area
    assert 0 <= lost <= 6.0 * 0.10 * 0.2 * 2 + 1e-9        # <= height x band x |dy/dx| per side


def test_unclipped_box_stays_one_box():
    r = gc.query_region((4.0, 1.0, 6.0, 5.0), CFG, TILTED)
    assert len(r.vertices()) == 4
    assert r.polygon().equals(__import__("shapely.geometry", fromlist=["box"]).box(4, 1, 6, 5))


def test_staircase_contains_aabb_agrees_with_its_polygon():
    from shapely.geometry import box as sbox
    r = _stair()
    poly = r.polygon()
    rng = np.random.default_rng(3)
    for _ in range(400):
        c = rng.uniform((0, 0), (10, 6))
        h = rng.uniform(0.05, 0.8, 2)
        lo, hi = (c[0] - h[0], c[1] - h[1], 0.0), (c[0] + h[0], c[1] + h[1], 1.0)
        inside = poly.buffer(1e-9).contains(sbox(lo[0], lo[1], hi[0], hi[1]))
        assert r.contains_aabb(lo, hi) == inside


def test_eroded_workspace_is_exactly_where_the_body_square_fits():
    r = _stair()
    e = 0.3
    ws = r.polygon(erode=e)
    rng = np.random.default_rng(4)
    from shapely.geometry import Point
    for _ in range(400):
        c = rng.uniform((0, 0), (10, 6))
        fits = r.contains_aabb((c[0] - e, c[1] - e, 0.0), (c[0] + e, c[1] + e, 1.0))
        d = ws.exterior.distance(Point(c))
        if d > 1e-6:
            assert ws.contains(Point(c)) == fits


def test_staircase_counts_contact_rejections():
    r = _stair()
    x_lo = r.vertices()[:, 0].min()
    assert not r.contains_aabb((x_lo - 0.05, 0.1, 0.0), (x_lo + 0.3, 0.3, 1.0))
    assert r.rejections["contact"] + r.rejections["x_min"] >= 1
