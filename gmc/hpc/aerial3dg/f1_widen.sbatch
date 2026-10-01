#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f_widen
#SBATCH --time=01:30:00
#SBATCH --mem=4G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# F1 Task 1: compile ONE robot on a widened route box (same archive, crop rule, config as G2), persist it,
# then answer the given pair list (G2's pairs, same indices / endpoints) from that one compile.
# Usage: sbatch --job-name=a3f_widen_<tag>_<robot> f1_widen.sbatch ROBOT TAG U0 V0 U1 V1 PAIRS_JSON
set -euo pipefail
ROBOT=$1; TAG=$2; U0=$3; V0=$4; U1=$5; V1=$6; PAIRS=$7
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
OUT=results/aerial3dg/f1/widen/$TAG/$ROBOT; A3C=outputs/aerial3dg/f1/widen/${TAG}_${ROBOT}.a3c
mkdir -p "$OUT" "$(dirname $A3C)"
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ) box $U0 $V0 $U1 $V1"
if [ ! -s "$A3C" ]; then
  $PY experiments/aerial3dg_batch.py compile --robot "$ROBOT" --box $U0 $V0 $U1 $V1 --a3c "$A3C" --out "$OUT/compile.json"
fi
$PY experiments/aerial3dg_batch.py task --robot "$ROBOT" --pairs "$PAIRS" --a3c "$A3C" --n-tasks 1 --task 0 --out-dir "$OUT"
echo "end $(date -u +%FT%TZ)"
