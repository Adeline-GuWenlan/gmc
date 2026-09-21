# GS3D independent agent jobs

Start here: [PLAN.md](PLAN.md), including the eight-agent roster, model choices, dependencies,
original requirements, acceptance matrix, Git rules and compute envelope.

This chain is based exactly on `9ff0d5c2a59c95a4444e45878ef878c8d9837108`.
The user selected **start now, automatically resume after quota refresh** on 2026-09-21.
No subagents are used. At most two independent agent sessions run concurrently.

```bash
cd /scratch/wg2381/codex_jobs/gs3d_20260921
python3 scheduler.py status
bash sessions.sh
squeue -u wg2381
```

`logs/A0_done.md` is the first handoff. `logs/A7_done.md` is the final report.
`state/publication.json` records verified GitHub refs after all stages pass.
Raw agent transcripts are local `logs/A#_JOBID.jsonl`; they are never committed.

Each agent has its own worktree/branch and persistent session ID. Resume commands are printed
by `sessions.sh`; do not interactively resume a session while its Slurm job is live.
The dispatcher reads completion artifacts and dependencies, not Slurm exit status alone.
It queues future work at actual quota reset times and uses separate afterany recovery jobs.

The plan and launcher source are versioned on `codex/gs3d-20260921/plan`. Agent branches are
`codex/gs3d-20260921/A0` ... `/A7`; completed integration is published on
`codex/gs3d-20260921/integration`. Original branches are preserved.

To stop new submissions, create `state/PAUSE`. Existing jobs continue. Remove it and run
`python3 scheduler.py dispatch` to resume. Job IDs owned by this chain are recorded in
`jobids/all.jsonl`. Never cancel jobs outside that ledger.

Infrastructure verification before launch: 20 offline tests passed, including DAG joins,
concurrency, quota/weekly reset calculation, compute waiting, crash recovery, missing-evidence
rejection and continuation precedence; all shell files passed `bash -n`.

To run those checks again without submitting jobs:
```bash
python3 -m unittest -v test_scheduler.py
```

This is the execution plan and job infrastructure. The 3D algorithm work is complete only when
the stage acceptance evidence and final publication proof say so.
