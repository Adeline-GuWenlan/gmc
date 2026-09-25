#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=g5_run
#SBATCH --time=01:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%A_%a.out
# G1/G2: one array task = one slice of pairs for one planner x robot combination.
# Arguments go to ground5k_run.py (e.g. --combo gmc_cylinder --first 0 --per-task 1).
# The runner skips pairs whose JSON already exists, so a requeued task never double-counts.
set -euo pipefail
cd /scratch/wg2381/splathjb-ground5k/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMBA_NUM_THREADS=1
echo "node $(hostname) job ${SLURM_JOB_ID} array ${SLURM_ARRAY_JOB_ID:-}_${SLURM_ARRAY_TASK_ID:-} start $(date -u +%FT%TZ) args $*"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/ground5k_run.py "$@"
echo "end $(date -u +%FT%TZ)"
