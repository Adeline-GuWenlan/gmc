# A4 — timing and benchmark tooling, completed 2026-09-22

Delivered standalone GS3D timing instrumentation and a bounded synthetic
three-robot benchmark harness. It is deliberately independent of A3's robot
entrypoint; A5 can attach the supplied sink while merging the production runs.

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

## Verified evidence

From `gmc/`, with the required interpreter and `PYTHONPATH=src:experiments`:

```bash
/scratch/wg2381/.conda/envs/gmc-venv/bin/python -m pytest -q \
  tests/unit/test_gs3d_timing.py tests/unit/test_gs3d_benchmark.py \
  tests/unit/test_gs3d_core_planner.py \
  -k 'timer_hooks or singleton_path_timing or gs3d_timing or three_robot_benchmark'
```

Result: **5 passed**. Governed compute job **18243909** (2 CPU, 8 GiB, 1 hour
cap) completed in 6 seconds, exit `0:0`, MaxRSS 45,468 KiB. It ran
`/scratch/wg2381/codex_jobs/gs3d_20260921/compute/A4_benchmark.sh`; its stdout
and stderr are empty because JSON is written directly to the artifact.

`gmc/results/gs3d/benchmark/timing.json` is valid JSON and records successful
cold plus five warm plans for every body. The governed-run summary is:

| Robot | cold algorithm / preparation (s) | warm p50 / p95 / max (s) | control dt / trajectory duration (s) |
|---|---:|---:|---:|
| UAV | .148781 / .005290 | .144154 / .144748 / .144748 | .05 / 8.533333 |
| Sweeper | .000662 / .000331 | .000566 / .000583 / .000583 | .05 / 5.333333 |
| Cylinder | .000650 / .000241 | .000588 / .000593 / .000593 | .05 / 5.333333 |

The report records Gaussian count, expansions, oracle calls, narrow-phase
pairs, cache identity/status and all individual warm samples. `render_fps` is
null and separate; render/export/queue time is excluded. Stage records retain
failed flags and call IDs, and state `stage_total_is_not_wall=true` because
nested search/edge spans must never be added into a total. GPU synchronization
is explicitly false: this is a CPU-only benchmark.

A first post-resume pytest command was launched from the repository root and
failed to import experiment fixtures because the required working directory is
`gmc/`; it changed no source or artifact. The prescribed-directory rerun above
passed, and is the acceptance result.

## A5 integration hook

For every production call, A5 should construct `PlanTiming`, bracket scene
load/edit/index preparation in `preparation()` and named `stage()` scopes, then
pass `timer.for_call(call_id)` to `LatticePlanner.plan`. Immediately before JSON
serialization, call `timer.finalize(result, mode="cold"|"warm", call_id=...)`.
Use one prepared scene/index across the five warm replans. Keep rendering,
export, scheduler queue time and physical `control_dt_s` outside
`algorithm_wall_s`; do not sum nested stage records. `finalize` preserves failed
or timed-out plan records, so all attempts—not only successes—must be emitted.
