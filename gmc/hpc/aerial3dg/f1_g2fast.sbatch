#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f_g2fast
#SBATCH --time=01:00:00
#SBATCH --mem=2G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# F1 control: the G2 compile (read-only .a3c) answered again in verdict-only query mode, so widened-box
# results are compared like with like.  Usage: sbatch f1_g2fast.sbatch ROBOT PAIRS_JSON
set -euo pipefail
ROBOT=$1; PAIRS=$2
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/aerial3dg_fail_widen.py --robot "$ROBOT" --pairs "$PAIRS" \
  --a3c /scratch/wg2381/splathjb-aerial3dg/gmc/outputs/aerial3dg/demo/${ROBOT}.a3c --out-dir results/aerial3dg/f1/widen/g2/$ROBOT
echo "end $(date -u +%FT%TZ)"
