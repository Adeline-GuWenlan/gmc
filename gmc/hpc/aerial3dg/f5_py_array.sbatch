#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f5_py
#SBATCH --time=02:00:00
#SBATCH --mem=1600M
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%A_%a.out
# F5: run line SLURM_ARRAY_TASK_ID (1-based) of a plan file; each line = python arguments (script + args).
# Usage: sbatch --job-name=a3f5_<what> --array=1-N%K f5_py_array.sbatch PLAN
set -euo pipefail
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
LINE=$(sed -n "${SLURM_ARRAY_TASK_ID}p" "$1")
echo "node $(hostname) job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ): $LINE"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python $LINE
echo "end $(date -u +%FT%TZ)"
