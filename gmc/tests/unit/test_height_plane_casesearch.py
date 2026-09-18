"""Amendment 3 P2a: the reusable case-search machinery (gmc.height.casesearch).

These are pure array/geometry functions: the tests build rasters and sections by hand so that the
criteria can be pinned without loading the 7.3 M-splat scene. The heavy end (project_scene ->
showcase_scene._support_raster) lives in experiments/plane_case_search.py and runs under sbatch.
"""
import numpy as np
import pytest

from gmc.height.casesearch import (cell_of, clearance_at, contrast, d1_precheck, edt_clearance,
                                   free_components, geodesic_path, line_clearance, overhang_on_line,
                                   path_polyline, path_separation, same_component, with_border)

CELL = 0.025


def empty(win, cell=CELL):
    nx = int(np.ceil((win[2] - win[0]) / cell))
    ny = int(np.ceil((win[3] - win[1]) / cell))
    return np.zeros((nx, ny), dtype=bool)


def box(occ, win, lo, hi, cell=CELL):
    i0, j0 = cell_of(lo, win, occ.shape, cell)
    i1, j1 = cell_of(hi, win, occ.shape, cell)
    occ[i0:i1 + 1, j0:j1 + 1] = True
    return occ


# --------------------------------------------------------------------------- rasters and clearance
def test_cell_of_is_the_floor_index_and_is_clamped_to_the_raster():
    win = [0.0, 0.0, 2.0, 2.0]
    shape = (80, 80)
    assert cell_of((0.0, 0.0), win, shape, CELL) == (0, 0)
    assert cell_of((0.26, 1.01), win, shape, CELL) == (10, 40)
    assert cell_of((-5.0, 99.0), win, shape, CELL) == (0, 79)


def test_edt_clearance_is_distance_in_metres_to_the_nearest_occupied_cell():
    win = [0.0, 0.0, 2.0, 2.0]
    occ = empty(win)
    occ[40, 40] = True
    dist = edt_clearance(occ, CELL)
    assert dist[40, 40] == 0.0
    assert dist[44, 40] == pytest.approx(4 * CELL)
    assert dist[44, 43] == pytest.approx(5 * CELL)


def test_with_border_marks_the_window_edge_occupied_and_does_not_mutate_its_input():
    win = [0.0, 0.0, 1.0, 1.0]
    occ = empty(win)
    out = with_border(occ)
    assert not occ.any()
    assert out[0, :].all() and out[-1, :].all() and out[:, 0].all() and out[:, -1].all()
    assert not out[1:-1, 1:-1].any()


def test_clearance_at_reads_the_metre_distance_under_a_world_point():
    win = [1.0, 1.0, 3.0, 3.0]
    occ = empty(win)
    occ = box(occ, win, (2.0, 1.0), (2.1, 3.0))
    dist = edt_clearance(occ, CELL)
    assert clearance_at(dist, win, CELL, (1.5, 2.0)) == pytest.approx(0.5, abs=CELL)
    assert clearance_at(dist, win, CELL, (2.05, 2.0)) == 0.0


# --------------------------------------------------------------------------- free space, components
def test_free_components_and_same_component_split_a_wall_that_spans_the_window():
    win = [0.0, 0.0, 4.0, 2.0]
    occ = box(empty(win), win, (2.0, 0.0), (2.1, 2.0))
    dist = edt_clearance(with_border(occ), CELL)
    lab, n = free_components(dist, 0.30)
    a = cell_of((0.6, 1.0), win, occ.shape, CELL)
    b = cell_of((3.4, 1.0), win, occ.shape, CELL)
    assert n >= 2
    assert lab[a] != 0 and lab[b] != 0
    assert not same_component(lab, a, b)


def test_same_component_is_true_across_a_gap_wider_than_the_disc():
    win = [0.0, 0.0, 4.0, 2.0]
    occ = empty(win)
    occ = box(occ, win, (2.0, 0.0), (2.1, 0.6))
    occ = box(occ, win, (2.0, 1.4), (2.1, 2.0))
    dist = edt_clearance(with_border(occ), CELL)
    a = cell_of((0.6, 1.0), win, occ.shape, CELL)
    b = cell_of((3.4, 1.0), win, occ.shape, CELL)
    lab_small, _ = free_components(dist, 0.175)
    lab_big, _ = free_components(dist, 0.45)
    assert same_component(lab_small, a, b)      # a 0.8 m gap passes a r = 0.175 m disc
    assert not same_component(lab_big, a, b)    # and stops a r = 0.45 m one


# --------------------------------------------------------------------------- D1
def test_d1_precheck_requires_r_plus_0_05_clearance_at_both_endpoints():
    """D1 (user-approved): spec criterion 3's 0.5 m becomes robot.max_radius() + 0.05 m."""
    win = [0.0, 0.0, 4.0, 2.0]
    occ = box(empty(win), win, (1.9, 0.9), (2.0, 1.0))
    dist = edt_clearance(with_border(occ), CELL)
    near = d1_precheck(dist, win, CELL, (1.70, 0.95), (3.4, 1.0), 0.175)
    far = d1_precheck(dist, win, CELL, (0.60, 1.0), (3.4, 1.0), 0.175)
    assert near["start_clear_m"] < 0.225 and not near["start_clear_ok"] and not near["ok"]
    assert far["start_clear_ok"] and far["goal_clear_ok"] and far["connected"] and far["ok"]
    assert far["clearance_required_m"] == pytest.approx(0.225)


def test_d1_precheck_fails_on_connectivity_even_when_both_endpoints_are_clear():
    win = [0.0, 0.0, 4.0, 2.0]
    occ = box(empty(win), win, (2.0, 0.0), (2.1, 2.0))
    dist = edt_clearance(with_border(occ), CELL)
    out = d1_precheck(dist, win, CELL, (0.6, 1.0), (3.4, 1.0), 0.30)
    assert out["start_clear_ok"] and out["goal_clear_ok"]
    assert not out["connected"] and not out["ok"]


# --------------------------------------------------------------------------- straight line
def test_line_clearance_reports_the_minimum_along_the_segment_not_at_its_ends():
    win = [0.0, 0.0, 4.0, 2.0]
    occ = box(empty(win), win, (2.0, 0.0), (2.1, 0.8))
    dist = edt_clearance(with_border(occ), CELL)
    out = line_clearance(dist, win, CELL, (0.6, 1.0), (3.4, 1.0))
    assert out["min_clearance_m"] == pytest.approx(0.2, abs=2 * CELL)
    assert out["blocked_len_m"] == 0.0 and out["free"]
    assert line_clearance(dist, win, CELL, (0.6, 1.0), (3.4, 1.0), r=0.175)["free"]


def test_line_clearance_measures_how_much_of_the_line_a_disc_cannot_use():
    win = [0.0, 0.0, 4.0, 2.0]
    occ = box(empty(win), win, (1.8, 0.0), (2.2, 2.0))
    dist = edt_clearance(with_border(occ), CELL)
    out = line_clearance(dist, win, CELL, (0.6, 1.0), (3.4, 1.0), r=0.30)
    assert out["min_clearance_m"] == 0.0
    assert out["blocked_len_m"] > 0.9          # 0.4 m of wall + 2 x 0.30 m of disc radius
    assert not out["free"]


# --------------------------------------------------------------------------- geodesics and contrast
def test_geodesic_path_is_the_straight_line_when_nothing_is_in_the_way():
    win = [0.0, 0.0, 4.0, 2.0]
    dist = edt_clearance(with_border(empty(win)), CELL)
    a = cell_of((0.6, 1.0), win, dist.shape, CELL)
    b = cell_of((3.4, 1.0), win, dist.shape, CELL)
    ij, length = geodesic_path(dist, 0.175, a, b, CELL)
    assert length == pytest.approx(2.8, abs=0.05)
    xy = path_polyline(ij, win, CELL)
    assert np.abs(xy[:, 1] - 1.0).max() < 0.05


def test_geodesic_path_detours_around_a_blocking_table_and_reports_the_longer_route():
    win = [0.0, 0.0, 4.0, 3.0]
    occ = box(empty(win), win, (1.5, 1.2), (2.5, 1.8))
    dist = edt_clearance(with_border(occ), CELL)
    a = cell_of((0.6, 1.5), win, dist.shape, CELL)
    b = cell_of((3.4, 1.5), win, dist.shape, CELL)
    ij, length = geodesic_path(dist, 0.30, a, b, CELL)
    assert length > 3.1                         # it must leave the straight line (2.8 m) to get round
    xy = path_polyline(ij, win, CELL)
    assert np.abs(xy[:, 1] - 1.5).max() > 0.5   # and go round one end


def test_geodesic_path_returns_none_when_the_two_cells_are_disconnected():
    win = [0.0, 0.0, 4.0, 2.0]
    occ = box(empty(win), win, (2.0, 0.0), (2.1, 2.0))
    dist = edt_clearance(with_border(occ), CELL)
    a = cell_of((0.6, 1.0), win, dist.shape, CELL)
    b = cell_of((3.4, 1.0), win, dist.shape, CELL)
    assert geodesic_path(dist, 0.30, a, b, CELL) is None


def test_path_separation_is_zero_for_identical_routes_and_large_for_a_detour():
    straight = np.stack([np.linspace(0.6, 3.4, 50), np.full(50, 1.5)], axis=1)
    detour = np.concatenate([np.stack([np.linspace(0.6, 2.0, 25), np.linspace(1.5, 2.6, 25)], axis=1),
                             np.stack([np.linspace(2.0, 3.4, 25), np.linspace(2.6, 1.5, 25)], axis=1)])
    assert path_separation(straight, straight) == pytest.approx(0.0, abs=1e-9)
    assert path_separation(straight, detour) == pytest.approx(1.1, abs=0.15)


def test_contrast_prefers_a_long_detour_over_a_short_one_at_equal_separation():
    lo = contrast(sweeper_len=3.0, cylinder_len=3.3, separation_m=0.5)
    hi = contrast(sweeper_len=3.0, cylinder_len=6.0, separation_m=0.5)
    assert hi > lo > 0.0


def test_contrast_is_zero_when_the_two_routes_coincide():
    assert contrast(sweeper_len=3.0, cylinder_len=3.0, separation_m=0.0) == 0.0


# --------------------------------------------------------------------------- criterion 1 (unchanged)
def section_of(bars, L):
    """bars: (s, rho_bottom, rho_top) above the floor -> the (s, zb, zt, L) tuple _section returns."""
    s, zb, zt = (np.array([b[k] for b in bars], float) for k in range(3))
    return s, zb, zt, L


def test_overhang_on_line_finds_a_tabletop_with_free_floor_under_it():
    """A 1.2 m tabletop at 0.70-0.87 m over open floor, between two legs: criterion 1 as step_case defines it."""
    bars = [(s, 0.70, 0.87) for s in np.arange(1.0, 2.2, 0.01)]
    out = overhang_on_line(*section_of(bars, 3.0), z_c=1.20, ceiling_height_m=5.31)
    assert out["ok"] and out["runs"] >= 5
    assert out["top"] == pytest.approx(0.87)
    assert out["underside"] == pytest.approx(0.70)
    assert out["s_interval"][0] == pytest.approx(1.0, abs=0.02)
    assert out["zc_above_overhang_top_by_0.25"] and out["zc_band_below_ceiling_side_by_0.30"]


def test_overhang_on_line_rejects_a_line_whose_floor_is_blocked_under_the_overhang():
    """Same tabletop, but a leg stands in the low band all the way along it: no free-low bin coincides."""
    bars = ([(s, 0.70, 0.87) for s in np.arange(1.0, 2.2, 0.01)]
            + [(s, 0.0, 0.30) for s in np.arange(1.0, 2.2, 0.01)])
    out = overhang_on_line(*section_of(bars, 3.0), z_c=1.20, ceiling_height_m=5.31)
    assert out["runs"] == 0 and not out["ok"]


def test_overhang_on_line_rejects_an_overhang_the_uav_would_hit():
    """Mass through the uav band [z_c-0.10, z_c+0.10] on the same bins disqualifies them."""
    bars = [(s, 0.70, 1.60) for s in np.arange(1.0, 2.2, 0.01)]
    out = overhang_on_line(*section_of(bars, 3.0), z_c=1.20, ceiling_height_m=5.31)
    assert out["runs"] == 0 and not out["ok"]


def test_overhang_on_line_requires_z_c_a_quarter_metre_above_the_overhang_top():
    bars = [(s, 0.70, 1.05) for s in np.arange(1.0, 2.2, 0.01)]
    out = overhang_on_line(*section_of(bars, 3.0), z_c=1.20, ceiling_height_m=5.31)
    assert out["runs"] >= 5
    assert not out["zc_above_overhang_top_by_0.25"] and not out["ok"]


def test_overhang_on_line_requires_0_30_m_below_the_next_thing_up():
    """A soffit 0.35 m over the uav band top leaves less than 0.30 m: criterion 1's ceiling-side clause."""
    bars = ([(s, 0.70, 0.87) for s in np.arange(1.0, 2.2, 0.01)]
            + [(s, 1.45, 2.00) for s in np.arange(1.0, 2.2, 0.01)])
    out = overhang_on_line(*section_of(bars, 3.0), z_c=1.20, ceiling_height_m=5.31)
    assert out["ceiling_side_obstacle_above_floor"] == pytest.approx(1.45)
    assert not out["zc_band_below_ceiling_side_by_0.30"] and not out["ok"]


def test_overhang_on_line_matches_step_case_criterion_1_bin_for_bin():
    """Pin the extraction against showcase_scene.step_case's own inline code, on a random section."""
    rng = np.random.default_rng(0)
    n, L, z_c = 4000, 4.0, 1.20
    s = rng.uniform(0, L, n)
    zb = rng.uniform(-0.05, 2.0, n)
    zt = zb + rng.uniform(0.01, 0.9, n)
    free_low = np.ones(int(np.ceil(L / 0.02)) + 1, dtype=bool)
    over = np.zeros_like(free_low)
    uav_hit = np.zeros_like(free_low)
    k = np.clip((s / 0.02).astype(int), 0, len(free_low) - 1)
    free_low[k[(zb <= 0.15) & (zt >= 0.02)]] = False
    over[k[(zb > 0.15) & (zb < z_c - 0.10)]] = True
    uav_hit[k[(zb <= z_c + 0.10) & (zt >= z_c - 0.10)]] = True
    expect = np.flatnonzero(free_low & over & ~uav_hit)

    out = overhang_on_line(s, zb, zt, L, z_c=z_c, ceiling_height_m=5.31)
    assert out["runs"] == len(expect)
    if len(expect):
        assert out["s_interval"] == [expect.min() * 0.02, expect.max() * 0.02]


# --------------------------------------------------------------------------- graph reuse (P2 scale)
def test_free_graph_reused_over_many_sources_agrees_with_geodesic_path():
    """The search runs one Dijkstra per source over a graph built once; it must match the simple call."""
    from scipy.sparse.csgraph import dijkstra

    from gmc.height.casesearch import free_graph, path_from_predecessors
    win = [0.0, 0.0, 4.0, 3.0]
    occ = box(empty(win), win, (1.5, 1.2), (2.5, 1.8))
    dist = edt_clearance(with_border(occ), CELL)
    G, idx, flat = free_graph(dist, 0.30, CELL)
    srcs = [cell_of(p, win, dist.shape, CELL) for p in ((0.6, 1.5), (0.6, 0.6))]
    goal = cell_of((3.4, 1.5), win, dist.shape, CELL)
    d, pred = dijkstra(G, directed=False, indices=[int(idx[c]) for c in srcs],
                       return_predecessors=True)
    for row, (src, prow) in enumerate(zip(srcs, pred)):
        ij, length = geodesic_path(dist, 0.30, src, goal, CELL)
        assert d[row][int(idx[goal])] == pytest.approx(length)
        again = path_from_predecessors(prow, flat, dist.shape, int(idx[src]), int(idx[goal]))
        assert np.array_equal(again, ij)
