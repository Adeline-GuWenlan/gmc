#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f3_compile
#SBATCH --time=01:30:00
#SBATCH --mem=3500M
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# F3 Task 1.1: probe-compile = persisted compile of BOTH robots on one region box (aerial3dg_batch compile: compile
# record with supports / candidate pairs / time / peak RSS + .a3c with SHA-256 sidecar).  These are the compiles the
# pilot GMC runs (and F4, if the box is unchanged) reuse.  Usage: f3_compile.sbatch REGION U0 V0 U1 V1
set -euo pipefail
REG=$1; shift
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
D=outputs/aerial3dg/f3/$REG; R=results/aerial3dg/f3/probe/$REG; mkdir -p $D $R
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ) region $REG box $*"
for ROBOT in cylinder sweeper; do
  [ -s $D/$ROBOT.a3c ] || $PY experiments/aerial3dg_batch.py compile --robot $ROBOT --box "$@" --a3c $D/$ROBOT.a3c --out $R/compile_$ROBOT.json
done
echo "end $(date -u +%FT%TZ)"
