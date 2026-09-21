#!/bin/bash
set -euo pipefail
export GS3D_ROOT=/scratch/wg2381/codex_jobs/gs3d_20260921
export PATH=/home/wg2381/.npm-global/bin:/home/wg2381/.local/bin:$PATH
export PYTHONUNBUFFERED=1 MPLBACKEND=Agg
exec python3 "$GS3D_ROOT/scheduler.py" run "$1"
