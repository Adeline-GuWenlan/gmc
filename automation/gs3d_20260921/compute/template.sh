#!/bin/bash
set -euo pipefail
: "${GS3D_WORKTREE:?Use the stage worktree}"
cd "$GS3D_WORKTREE/gmc"
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
# Copy this template with an explicit worktree path before submitting, then replace test target.
/scratch/wg2381/.conda/envs/gmc-venv/bin/python -m pytest tests/unit/test_height_planefloor.py
