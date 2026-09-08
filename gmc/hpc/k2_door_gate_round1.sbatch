#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --job-name=gmc_k2_r1
#SBATCH --time=00:20:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=8
#SBATCH --output=/scratch/wg2381/splathjb/gmc/logs/%x-%j.out
set -x
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=8 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
# Slice-level round only: the full-pipeline stage cannot run until the
# prototype world-bound iteration cap is decided (see the worklog).
$PY experiments/k2_door_gate.py --door door_A --half 1.5 \
    --figure results/k2/figures/door_A_h1.5.png \
    -o results/k2/door_A_h1.5.json
echo "=== K2_R1_DONE ==="
