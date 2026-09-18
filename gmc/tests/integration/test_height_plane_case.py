"""Amendment 3 P2a: the shared-case search must find the three-route picture on a scene built to have it.

``synth3d.table_scene("open")`` is a wall at x = 0 with two gaps. The south gap is filled by a table
(top 0.74 m on four thin legs); the north gap is empty. From one start to one goal that gives the
sweeper a route **under** the table, the uav a route **over** it, and the cylinder no choice but to go
**round** — the picture the user asked for. This test runs the real pipeline (project_scene ->
showcase_scene._support_raster -> EDT -> D1 -> criterion 1 -> geodesics) on it, so a regression in the
search shows up here rather than in a 4 h Slurm job on the hall.

Run from gmc/ with PYTHONPATH=src:experiments.
"""
import numpy as np
import pytest

from gmc.height.prism import robot_table
from gmc.height.synth3d import GOAL, START, WORKSPACE, Z_FLOOR, table_scene

plane_case_search = pytest.importorskip("plane_case_search")   # needs PYTHONPATH=experiments

ROBOTS = ("sweeper", "cylinder", "uav")
Z_C = 1.20


@pytest.fixture(scope="module")
def verdict():
    scene, _ = table_scene("open")
    robots = robot_table(Z_C)
    win = list(WORKSPACE)
    st, gl = tuple(START[:2]), tuple(GOAL[:2])
    sec = plane_case_search.Sections(scene, Z_FLOOR)
    maps = {k: plane_case_search.certified(scene, robots[k], win, Z_FLOOR) for k in ROBOTS}
    ev = plane_case_search.evaluate(maps, win, robots, st, gl, Z_C, sec, 2.5)
    return ev, maps, win, robots, scene, sec, st, gl


def test_the_precomputed_section_agrees_with_showcase_scene_section(verdict):
    _, _, _, _, scene, sec, st, gl = verdict
    assert sec.selfcheck(scene, Z_FLOOR, st, gl)["agree"]


def test_the_case_passes_every_criterion_including_the_unchanged_criterion_1(verdict):
    ev = verdict[0]
    assert ev["failed_criteria"] == []
    assert ev["pass"]
    assert ev["criterion1"]["ok"] and ev["criterion1"]["top"] == pytest.approx(0.76, abs=0.01)


def test_the_straight_line_is_free_for_the_sweeper_and_blocked_for_the_cylinder(verdict):
    ev = verdict[0]
    assert ev["robots"]["sweeper"]["line"]["free"]
    assert ev["robots"]["uav"]["line"]["free"]
    assert not ev["robots"]["cylinder"]["line"]["free"]
    assert ev["robots"]["cylinder"]["line"]["blocked_len_m"] > 0.6   # the tabletop plus two radii


def test_all_three_robots_share_the_window_the_start_and_the_goal(verdict):
    ev, _, win, _, _, _, st, gl = verdict
    assert ev["window"] == list(win)
    assert tuple(ev["start"]) == st and tuple(ev["goal"]) == gl
    for k in ROBOTS:
        assert ev["robots"][k]["connected"]
        assert ev["robots"][k]["start_clear_ok"] and ev["robots"][k]["goal_clear_ok"]


def test_d1_is_what_the_endpoints_are_judged_by_and_it_is_per_robot(verdict):
    ev = verdict[0]
    need = {k: ev["robots"][k]["clearance_required_m"] for k in ROBOTS}
    assert need == pytest.approx({"sweeper": 0.225, "cylinder": 0.35, "uav": 0.30})


def test_the_three_routes_are_visibly_different(verdict):
    """The headline: same window, same start, same goal, and the cylinder's route runs metres away."""
    ev = verdict[0]
    lens = {k: ev["robots"][k]["path_len_m"] for k in ROBOTS}
    assert lens["cylinder"] > lens["sweeper"] + 1.0
    assert ev["separation_m"] > 0.5
    assert ev["contrast"] > 0.5
    sweeper = np.array(ev["robots"]["sweeper"]["path_xy"])
    cylinder = np.array(ev["robots"]["cylinder"]["path_xy"])
    assert np.abs(sweeper[:, 1]).max() < 0.2          # straight under the table
    assert cylinder[:, 1].max() > 1.0                 # round through the north gap


def test_the_cylinder_sees_the_tabletop_and_the_sweeper_does_not(verdict):
    """The maps differ because the bands differ — the whole mechanism, in one assertion."""
    _, maps, win, robots, _, _, _, _ = verdict
    from gmc.height.casesearch import cell_of
    over_table = cell_of((0.0, 0.0), win, maps["sweeper"]["occ"].shape, plane_case_search.C)
    assert not maps["sweeper"]["occ"][over_table]
    assert maps["cylinder"]["occ"][over_table]
    assert not maps["uav"]["occ"][over_table]
