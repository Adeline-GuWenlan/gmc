#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --job-name=ul_t1_measure_q1
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=12G
#SBATCH --time=01:00:00
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# L1 Task 1 + Task 2 measurement: real-hall measurement rasters, then the two
# pre-registered single queries on Codex's lamp-only derivative (read-only).
set -euo pipefail
cd /scratch/wg2381/splathjb-uavlamp/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=2
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
$PY experiments/uavlamp_measure.py \
  --archive /scratch/wg2381/splathjb/gmc/outputs/height/plane/processed_planefloor.npz \
  --output outputs/uavlamp/measure
$PY experiments/uavlamp_query.py \
  outputs/uavlamp/specs/Q1a_codex_start_codex_box.json \
  outputs/uavlamp/specs/Q1b_start_1p5_back_wide_box.json
