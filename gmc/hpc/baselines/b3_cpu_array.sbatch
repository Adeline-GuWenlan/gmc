#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=bl_b3_cpu
#SBATCH --time=05:00:00
#SBATCH --mem=2G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%A_%a.out
# bl B3 full run of a CPU method ($1 = cust_fields): array task i = results/baselines/b3/plan.json
# methods[$1].tasks[i] (`bl_b3.py task`), judge inline, frozen config. Pilot tasks peaked at 0.9 GB -> 2G.
set -uo pipefail
M=$1
cd /scratch/wg2381/splathjb-baselines/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
ARGS=$($PY experiments/bl_b3.py task --method "$M" --index "$SLURM_ARRAY_TASK_ID")
echo "node $(hostname) job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
echo "args: $ARGS"
$PY experiments/bl_harness.py $ARGS
rc=$?
echo "end $(date -u +%FT%TZ) rc=$rc"
exit $rc
