# A4 — timing and benchmark tooling (checkpoint)

Implemented the standalone GS3D timing collector and the bounded synthetic
three-robot benchmark harness.  This checkpoint is intentionally incomplete
until compute job `18243909` has produced and validated the final benchmark
artifact.

## Delivered interface

- `gmc/src/gmc/gs3d/timing.py`: monotonic `PlanTiming` stage sink, cold
  preparation accounting, per-replan records and deterministic p50/p95/max.
  Stage spans are not summed: search can contain edge validation and a nested
  sum would double-count.  The planner's own entry-to-result wall interval is
  retained as `algorithm_wall_s`.
- `gmc/experiments/gs3d_benchmark.py`: runs UAV, sweeper and cylinder with one
  cold prepared call and five warm calls each.  It writes `gs3d.benchmark.v1`,
  including sizes, seed, cache state, every warm latency sample, distribution,
  status, and separately named `control_dt_s`/trajectory duration.  It does
  not include rendering or scheduler time in algorithm timing.
- `gmc/tests/unit/test_gs3d_timing.py`: deterministic-clock boundary, failed
  stage, nested-stage and call-ID tests.
- `gmc/tests/unit/test_gs3d_benchmark.py`: one real small three-body run and
  JSON schema assertions.

## Checks completed before compute

From `gmc/`, with the required interpreter and `PYTHONPATH=src:experiments`:

```bash
/scratch/wg2381/.conda/envs/gmc-venv/bin/python -m pytest -q \
  tests/unit/test_gs3d_timing.py tests/unit/test_gs3d_benchmark.py \
  tests/unit/test_gs3d_core_planner.py \
  -k 'timer_hooks or singleton_path_timing or gs3d_timing or three_robot_benchmark'
```

Result: **5 passed**.  A one-time local tiny run also produced successful UAV,
sweeper and cylinder records; the governed reproducible run is job `18243909`.

## Required continuation

1. Inspect job `18243909` state and `logs/gs3d_compute_A4-18243909.{out,err}`.
2. Validate/open `gmc/results/gs3d/benchmark/timing.json`, then rerun the
   focused tests after any correction.
3. Commit the listed A4 paths and update runtime `state/A4.progress.json`,
   `logs/A4_done.md`, and `state/A4.done.json` if all evidence passes.
4. Tell A5 to construct `PlanTiming`, time scene load/edit/index in
   `preparation()`/`stage()`, pass `timer.for_call(id)` to every planner call,
   then call `finalize(result, mode=..., call_id=...)` before serializing.
   Keep render/export and `control_dt_s` outside `algorithm_wall_s`.
