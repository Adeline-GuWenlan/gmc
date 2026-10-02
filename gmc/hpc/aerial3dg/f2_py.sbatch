#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f2_py
#SBATCH --time=01:00:00
#SBATCH --mem=3G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# F2 (aerial3dg failure analysis, stage 2): run one python entry point from this worktree's gmc/.
# Usage: sbatch --job-name=a3f2_<what> [--mem/--time] f1_py.sbatch experiments/<script>.py [args...]
set -euo pipefail
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
echo "node $(hostname) job ${SLURM_JOB_ID}${SLURM_ARRAY_TASK_ID:+_$SLURM_ARRAY_TASK_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python "$@"
echo "end $(date -u +%FT%TZ)"
