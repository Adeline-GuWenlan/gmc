"""Amendment 3 P3: a video title must state the claim of the case it shows, and must not round the
certificate away.

P2's renderer captioned every plane-floor video "one window, one start, one goal, three robots". That is
P2's claim; P3's long case has two robots (the cylinder runs on a separate ladder, user decision D4), so
the caption is chosen by preset. And ``replay3d lb = 0.000 m`` was printed for a certified 0.00015 m
bound -- a title that reads as "no clearance" for a route that has some.
"""
import pytest

percase_render = pytest.importorskip("percase_render")

BOUNDARY = ("USER-APPROVED MANUAL SCENE EDIT", "D1")


def test_default_claims_are_unchanged():
    assert percase_render.claim_for("planefloor") == percase_render.CLAIM_PLANE
    assert percase_render.claim_for("processed") == percase_render.CLAIM
    assert "three robots" in percase_render.CLAIM_PLANE


@pytest.mark.parametrize("key", ["p3long", "p3rung"])
def test_p3_presets_carry_the_claims_boundary_and_not_p2s_three_robot_claim(key):
    c = percase_render.claim_for("planefloor", key)
    assert all(b in c for b in BOUNDARY)
    assert "three robots" not in c
    assert "D4" in c


def test_p3_long_preset_names_the_two_robots_it_shows():
    assert "sweeper + uav" in percase_render.claim_for("planefloor", "p3long")


def test_unknown_preset_is_an_error_not_a_silent_default():
    with pytest.raises(KeyError):
        percase_render.claim_for("planefloor", "p4")


@pytest.mark.parametrize("lb,shown", [(0.00014671492565643263, "0.000147 m"), (0.017685, "0.0177 m"),
                                      (0.05, "0.05 m")])
def test_headline_keeps_three_significant_figures_of_the_clearance_bound(lb, shown):
    res = {"status": "REACHABLE", "verify": {"certified": True},
           "replay3d": {"passed": True, "min_clearance_lb": lb}}
    head, ok = percase_render.headline("sweeper", res)
    assert ok and f"replay3d lb = {shown}" in head


@pytest.mark.parametrize("key", ["p3long", "p3rung"])
def test_p3_presets_fit_one_title_line_so_the_claims_boundary_is_not_clipped(key):
    """The 3D title band holds the headline plus ONE caption line (13 pt, 1280 px): about 135 characters.
    The first P3 render clipped its caption at both frame edges and 'D1' fell off."""
    assert len(percase_render.claim_for("planefloor", key)) <= 135
