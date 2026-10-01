#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f_diag
#SBATCH --time=01:30:00
#SBATCH --mem=2500M
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# F1 Task 2: diagnostics on the persisted G2 compile of ROBOT (read-only), several subcommands in one load each.
# Usage: sbatch --job-name=a3f_diag_<robot> f1_diag.sbatch ROBOT CMD [CMD ...]   (CMD in endpoints|replay|bridges)
set -uo pipefail
ROBOT=$1; shift
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
for CMD in "$@"; do
  EXTRA=""; [ "$CMD" = bridges ] && EXTRA="--grid"
  $PY experiments/aerial3dg_fail_diag.py $CMD --robot $ROBOT --out results/aerial3dg/f1/diag/${CMD}_${ROBOT}.json $EXTRA \
    || echo "FAILED $CMD"
done
echo "end $(date -u +%FT%TZ)"
