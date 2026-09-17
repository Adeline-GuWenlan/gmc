"""Amendment 3 P1e: per-stage wall-clock timing, threaded through the height pipeline.

P4 answers "where does the wall clock actually go" from these records, so what is pinned here is
the record *shape* (stage name, seconds, input size) and the promise that adding the timer changed
nothing for existing callers -- `compile_seconds` and `query_seconds` keep their current meaning.
"""
import pytest

from gmc.height.prism import robot_table
from gmc.height.run import compile_and_query
from gmc.height.timing import STAGES, StageTimer
from gmc.synth import single_door

from ..conftest import SMALL_WS, make_cfg


class FakeClock:
    """A clock the test drives, so the assertions are exact instead of flaky."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def test_stage_list_is_the_one_amendment_3_p1e_names():
    assert STAGES == ("scene_load", "floor_replace", "project", "compile_pairs",
                      "compile_slabs", "compile_mobility", "query", "verify_curve",
                      "replay3d")


def test_stage_records_its_name_its_seconds_and_its_input_size():
    clk = FakeClock()
    t = StageTimer(clock=clk)
    with t.stage("scene_load", n_splats=7_247_831):
        clk.advance(11.7)
    rec, = t.records
    assert rec["stage"] == "scene_load"
    assert rec["seconds"] == pytest.approx(11.7)
    assert rec["sizes"] == {"n_splats": 7_247_831}
    assert rec["failed"] is False
    assert rec["label"] is None


def test_input_size_can_be_filled_in_after_the_work_returns():
    """`project`'s own output size is only known once projection has run."""
    clk = FakeClock()
    t = StageTimer(clock=clk)
    with t.stage("project", label="sweeper", n_splats=100) as rec:
        clk.advance(2.0)
        rec["sizes"]["n_supports"] = 412
    assert t.records[0]["label"] == "sweeper"
    assert t.records[0]["sizes"] == {"n_splats": 100, "n_supports": 412}


def test_an_unknown_stage_is_a_typo_not_a_new_stage():
    with pytest.raises(ValueError, match="compile_pairs"):
        StageTimer().record("compile_paris", 1.0)
    with pytest.raises(ValueError, match="compile_pairs"):
        with StageTimer().stage("kompile_pairs"):
            pass


def test_repeated_stages_are_kept_apart_and_summed_per_stage():
    clk = FakeClock()
    t = StageTimer(clock=clk)
    for key, dt in (("sweeper", 2.0), ("cylinder", 3.0)):
        with t.stage("project", label=key):
            clk.advance(dt)
    d = t.to_dict()
    assert [r["label"] for r in d["stages"]] == ["sweeper", "cylinder"]
    assert d["by_stage"]["project"] == pytest.approx(5.0)
    assert d["total_seconds"] == pytest.approx(5.0)


def test_to_dict_names_the_stages_that_never_ran():
    d = StageTimer().to_dict()
    assert d["stages"] == [] and d["total_seconds"] == 0.0
    assert d["missing_stages"] == list(STAGES)


def test_a_failed_stage_is_still_timed_and_the_error_propagates():
    clk = FakeClock()
    t = StageTimer(clock=clk)
    with pytest.raises(ZeroDivisionError):
        with t.stage("query"):
            clk.advance(4.0)
            raise ZeroDivisionError("budget")
    assert t.records[0]["seconds"] == pytest.approx(4.0)
    assert t.records[0]["failed"] is True


# ------------------------------------------------- threaded through run.compile_and_query --


@pytest.fixture(scope="module")
def door_timed():
    cfg = make_cfg(theta_min=5e-3, initial_intervals=8)
    timer = StageTimer()
    res, _ = compile_and_query(single_door(0.6, workspace=SMALL_WS),
                               robot_table()["ellipse_toy"], cfg,
                               (-1.7, 0.0, 0.1), (1.7, 0.0, 0.1), timer=timer)
    return res, timer


def test_compile_and_query_times_every_stage_it_owns(door_timed):
    res, timer = door_timed
    assert res["status"] == "REACHABLE" and res["verify"]["certified"]
    assert {r["stage"] for r in timer.records} == {
        "compile_pairs", "compile_slabs", "compile_mobility", "query", "verify_curve"}
    assert all(r["seconds"] >= 0.0 and not r["failed"] for r in timer.records)


def test_compile_seconds_and_query_seconds_keep_their_current_meaning(door_timed):
    res, timer = door_timed
    by = timer.to_dict()["by_stage"]
    three = by["compile_pairs"] + by["compile_slabs"] + by["compile_mobility"]
    # the three sub-stages are strict subintervals of the compile block, so they cannot exceed it
    assert three <= res["compile_seconds"] + 1e-3
    assert three == pytest.approx(res["compile_seconds"], abs=0.5)
    assert by["query"] == pytest.approx(res["query_seconds"], abs=0.1)


def test_each_stage_records_the_input_size_p4_will_regress_against(door_timed):
    res, timer = door_timed
    by = {r["stage"]: r["sizes"] for r in timer.records}
    assert by["compile_pairs"]["n_supports"] == res["n_supports"]
    assert by["compile_slabs"]["n_pairs"] == res["n_pairs"]
    assert by["compile_mobility"]["n_slabs"] == res["n_slabs"]
    assert by["query"]["n_slabs"] == res["n_slabs"]
    assert by["verify_curve"]["n_pairs"] == res["n_pairs"]


def test_the_result_json_carries_the_timing_block(door_timed):
    res, timer = door_timed
    assert res["timing"]["by_stage"]["query"] == pytest.approx(
        timer.to_dict()["by_stage"]["query"])
    assert "replay3d" in res["timing"]["missing_stages"]


def test_a_caller_that_passes_no_timer_is_unaffected():
    cfg = make_cfg(theta_min=5e-3, initial_intervals=4)
    res, _ = compile_and_query(single_door(0.6, workspace=SMALL_WS),
                               robot_table()["ellipse_toy"], cfg,
                               (-1.7, 0.0, 0.1), (1.7, 0.0, 0.1))
    assert res["compile_seconds"] > 0.0 and res["query_seconds"] >= 0.0
    assert "timing" not in res
