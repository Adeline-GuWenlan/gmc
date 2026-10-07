#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f5_trace
#SBATCH --time=04:00:00
#SBATCH --mem=2500M
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%A_%a.out
# F5 Task 1: trace shard number SLURM_ARRAY_TASK_ID (1-based line of PLAN: "REGION ROBOT SHARD_FILE").
# Usage: sbatch --array=1-N%6 f5_trace_array.sbatch PLAN [extra trace args]
set -euo pipefail
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
PLAN=$1; shift
read -r REG ROBOT SHARD <<< "$(sed -n "${SLURM_ARRAY_TASK_ID}p" "$PLAN")"
echo "node $(hostname) job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} $REG $ROBOT $SHARD commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/aerial3dg_fail5_trace.py trace --shard "$SHARD" "$@"
echo "end $(date -u +%FT%TZ)"
