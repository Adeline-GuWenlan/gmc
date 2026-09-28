#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3g_variant
#SBATCH --time=01:00:00
#SBATCH --mem=3G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# G3 size sensitivity: compile ONE body with a perturbed radius on the G2 box, persist it, then answer the
# unbiased pair slice 0-499 (task 0 of 10) of pairs_5000.json from that one compile.
# Usage: sbatch --job-name=a3g_variant_<robot>_<r> g3_variant.sbatch ROBOT RADIUS
set -euo pipefail
ROBOT=$1; R=$2; TAG=${ROBOT}_r${R}
cd /scratch/wg2381/splathjb-aerial3dg/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
OUT=results/aerial3dg/g3/variants/$TAG; A3C=outputs/aerial3dg/g3/variants/$TAG.a3c
mkdir -p "$OUT" "$(dirname $A3C)"
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
$PY experiments/aerial3dg_batch.py compile --robot "$ROBOT" --radius "$R" --box -9.0 -0.35 3.7 2.75 \
  --a3c "$A3C" --out "$OUT/compile.json"
$PY experiments/aerial3dg_batch.py task --robot "$ROBOT" --pairs results/aerial3dg/g2/pairs_5000.json \
  --a3c "$A3C" --n-tasks 10 --task 0 --out-dir "$OUT"
echo "end $(date -u +%FT%TZ)"
