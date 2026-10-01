"""F1 Task 1: answer G2's pairs on a widened-box compile with the verdict pipeline only.

``task`` = ``aerial3dg_batch.run_task`` (load once, fsync'd JSONL, resumable) with
``QueryConfig(export_max_segment_m=0.20, shortcut=False, tighten=False, merge_corners=False)``:
the verdict machinery is G2's (endpoint location, possible-space cut, portal-graph search, lifting,
own continuous verifier, shared gs3d replay); only the path post-processing that shortens a route
after it is found is skipped (it was > 95 % of the wall time of the long detours, and it can only
replace the polyline by another certified one).  So status / reason are comparable with G2; path
length is the unshortened lifted polyline.  A full-config sample is kept for the agreement check.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from gmc.aerial3d.api import QueryConfig

from aerial3dg_batch import run_task, task_slice
from aerial3dg_run import MANIFEST, _dump, host

FAST = QueryConfig(export_max_segment_m=.20, shortcut=False, tighten=False, merge_corners=False)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--robot", required=True)
    p.add_argument("--pairs", type=Path, required=True)
    p.add_argument("--a3c", type=Path, required=True)
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument("--n-tasks", type=int, default=1)
    p.add_argument("--task", type=int, default=0)
    p.add_argument("--timeout", type=float, default=120.)
    a = p.parse_args(argv)
    doc = json.loads(a.pairs.read_text())
    pairs = [doc["pairs"][i] for i in task_slice(len(doc["pairs"]), a.n_tasks, a.task)]
    out = a.out_dir / f"fast_{a.task:02d}.jsonl"
    s = run_task(a.a3c, pairs, out, timeout_s=a.timeout, manifest=json.loads(MANIFEST.read_text()), qconfig=FAST)
    s.update(robot=a.robot, qconfig=repr(FAST), pairs_file=str(a.pairs), host=host())
    _dump(a.out_dir / f"fast_{a.task:02d}.summary.json", s)
    print(json.dumps({k: s[k] for k in ("status_counts", "answered_this_run", "query_wall_s_this_run", "peak_rss_mb")}),
          flush=True)


if __name__ == "__main__":
    main()
