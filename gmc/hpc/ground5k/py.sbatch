#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=g5_py
#SBATCH --time=01:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# Ground-5k: run one experiments/ script with its arguments (diagnostics, summaries).
set -euo pipefail
cd /scratch/wg2381/splathjb-ground5k/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
echo "node $(hostname) job ${SLURM_JOB_ID} start $(date -u +%FT%TZ) args $*"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python "$@"
echo "end $(date -u +%FT%TZ)"
