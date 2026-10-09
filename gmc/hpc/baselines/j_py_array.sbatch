#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=bl_j_py
#SBATCH --time=02:00:00
#SBATCH --mem=3G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%A_%a.out
# bl J: run line SLURM_ARRAY_TASK_ID (1-based) of a plan file: "CODE <bl_judge_check.py args>", CODE = new | old.
# new = HEAD (src:experiments); old = git archive 2a68d73 under outputs/baselines/j/old/gmc.  python -P keeps HEAD's
# experiments dir (this script's dir) off sys.path, so an old run imports only old code (checked by code_label()).
# Usage: sbatch --job-name=bl_j_<what> --array=1-N%K j_py_array.sbatch PLAN
set -euo pipefail
cd /scratch/wg2381/splathjb-baselines/gmc
read -r CODE ARGS <<< "$(sed -n "${SLURM_ARRAY_TASK_ID}p" "$1")"
if [ "$CODE" = old ]; then O=outputs/baselines/j/old/gmc; export PYTHONPATH=$O/src:$O/experiments; else export PYTHONPATH=src:experiments; fi
export MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
echo "node $(hostname) job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} commit $(git rev-parse HEAD) code $CODE start $(date -u +%FT%TZ): $ARGS"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python -P experiments/bl_judge_check.py $ARGS
echo "end $(date -u +%FT%TZ)"
