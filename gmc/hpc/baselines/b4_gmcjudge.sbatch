#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=bl_b4_gmcjudge
#SBATCH --time=02:00:00
#SBATCH --mem=3G
#SBATCH --cpus-per-task=1
#SBATCH --array=0-5
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%A_%a.out
# bl B4 audit: GMC's REACHABLE routes re-judged through bl_harness.judge_row (d757729 judge), one task per region x robot.
# B3's inline-judge cust_fields tasks (same Judge, same compiles) peaked < 0.9 GB -> 3G.
set -uo pipefail
cd /scratch/wg2381/splathjb-baselines/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
CELLS=(WWEST:cylinder WWEST:sweeper GAPW1:cylinder GAPW1:sweeper S:cylinder S:sweeper)
C=${CELLS[$SLURM_ARRAY_TASK_ID]}
echo "node $(hostname) job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ) cell $C"
$PY experiments/bl_b4_gmcjudge.py --region ${C%%:*} --robot ${C##*:}
rc=$?
echo "end $(date -u +%FT%TZ) rc=$rc"
exit $rc
