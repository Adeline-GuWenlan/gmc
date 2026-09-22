from contextlib import contextmanager

import pytest

from gmc.gs3d.timing import PlanTiming, latency_statistics


class Clock:
    def __init__(self): self.value = 0.0
    def __call__(self): return self.value
    def advance(self, seconds): self.value += seconds


def test_stage_boundaries_failure_and_no_nested_sum():
    clock = Clock()
    timing = PlanTiming(clock=clock)
    with timing.preparation():
        with timing.stage("scene_load", call_id="cold"):
            clock.advance(2)
        with timing.stage("index_build", call_id="cold"):
            with timing.stage("nested", call_id="cold"):
                clock.advance(3)
            clock.advance(1)
    with pytest.raises(RuntimeError):
        with timing.stage("search", call_id="r0"):
            clock.advance(4)
            raise RuntimeError("budget")
    result = {"timings": {"algorithm_wall_s": 7.0}}
    timing.finalize(result, mode="cold", call_id="r0")
    records = result["timings"]["records"]
    assert result["timings"]["preparation_wall_s"] == 6.0
    assert result["timings"]["algorithm_wall_s"] == 7.0
    assert [r["seconds"] for r in records] == [2.0, 4.0, 3.0, 4.0]
    assert records[-1]["failed"] is True
    assert result["timings"]["stage_total_is_not_wall"] is True


def test_call_sink_labels_hooks_and_latency_nearest_rank():
    clock = Clock()
    timing = PlanTiming(clock=clock)
    with timing.for_call("r3").stage("verify"):
        clock.advance(.25)
    result = {"timings": {"algorithm_wall_s": .25}}
    timing.finalize(result, mode="warm", call_id="r3", preparation_wall_s=0)
    assert result["timings"]["records"][0]["call_id"] == "r3"
    assert latency_statistics([1., 2., 3., 4., 5.]) == {"count": 5, "p50_s": 3., "p95_s": 5., "max_s": 5.}
