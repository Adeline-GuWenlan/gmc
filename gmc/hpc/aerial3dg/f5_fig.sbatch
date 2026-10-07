#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f5_fig
#SBATCH --time=02:00:00
#SBATCH --mem=2000M
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%A_%a.out
# F5 Task 2: review figures, part SLURM_ARRAY_TASK_ID of N (array 0..N-1); task N also renders the overviews.
# Usage: sbatch --array=0-3 f5_fig.sbatch 4
set -euo pipefail
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
P=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
echo "node $(hostname) job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} start $(date -u +%FT%TZ)"
$P experiments/aerial3dg_fail5_fig.py cases --part "${SLURM_ARRAY_TASK_ID}" --parts "$1"
if [ "${SLURM_ARRAY_TASK_ID}" = "0" ]; then $P experiments/aerial3dg_fail5_fig.py overview; fi
echo "end $(date -u +%FT%TZ)"
