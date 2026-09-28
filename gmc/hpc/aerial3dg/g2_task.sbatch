#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3g_task
#SBATCH --time=01:00:00
#SBATCH --mem=2G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%A_%a.out
# One array task of the G2 5000-pair run: load the robot's persisted compile ONCE, answer a contiguous
# slice of the pair list, one fsync'd JSON line per pair (a requeued/killed task resumes mid-slice).
# Usage: sbatch --array=0-9%K --job-name=a3g_task_<robot> g2_task.sbatch ROBOT N_TASKS [extra task args]
set -euo pipefail
ROBOT=$1; NT=$2; shift 2
cd /scratch/wg2381/splathjb-aerial3dg/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
echo "node $(hostname) job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/aerial3dg_batch.py task --robot "$ROBOT" \
  --pairs results/aerial3dg/g2/pairs_5000.json --a3c outputs/aerial3dg/demo/${ROBOT}.a3c \
  --n-tasks "$NT" --out-dir results/aerial3dg/g2/runs/${ROBOT} "$@"
echo "end $(date -u +%FT%TZ)"
