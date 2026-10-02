#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f2_gmc
#SBATCH --time=05:00:00
#SBATCH --mem=3500M
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# F2 Task 1c: compile ONE robot once on the F2 region box, persist it (.a3c + SHA-256 sidecar), then answer the
# confirmed-reachable pair list from that one compile with G2's full query config (resumable JSONL).
# Usage: sbatch --job-name=a3f2_gmc_<robot> f2_gmc.sbatch ROBOT U0 V0 U1 V1 PAIRS_JSON [N_TASKS TASK]
set -euo pipefail
ROBOT=$1; U0=$2; V0=$3; U1=$4; V1=$5; PAIRS=$6; NT=${7:-1}; T=${8:-0}
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
OUT=${OUT:-results/aerial3dg/f2/gmc/$ROBOT}; A3C=outputs/aerial3dg/f2/${ROBOT}.a3c
mkdir -p "$OUT" "$(dirname $A3C)"
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ) box $U0 $V0 $U1 $V1"
if [ ! -s "$A3C" ]; then
  $PY experiments/aerial3dg_batch.py compile --robot "$ROBOT" --box $U0 $V0 $U1 $V1 --a3c "$A3C" --out "$OUT/compile.json"
fi
$PY experiments/aerial3dg_batch.py task --robot "$ROBOT" --pairs "$PAIRS" --a3c "$A3C" --n-tasks $NT --task $T --out-dir "$OUT"
echo "end $(date -u +%FT%TZ)"
