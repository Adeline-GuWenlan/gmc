#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f3_gmc
#SBATCH --time=03:00:00
#SBATCH --mem=3500M
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# F3 Task 1.2/1.3: compile ONE robot once on a region box (reused if the .a3c exists; .a3c + SHA-256 sidecar), then
# answer a pair list from that one compile with G2's full QueryConfig (aerial3dg_batch task, resumable JSONL).
# Usage: sbatch --job-name=a3f3_gmc_<region>_<robot> f3_gmc.sbatch ROBOT U0 V0 U1 V1 PAIRS_JSON A3C OUT_DIR [N_TASKS TASK]
set -euo pipefail
ROBOT=$1; U0=$2; V0=$3; U1=$4; V1=$5; PAIRS=$6; A3C=$7; OUT=$8; NT=${9:-1}; T=${10:-${SLURM_ARRAY_TASK_ID:-0}}
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
mkdir -p "$OUT" "$(dirname $A3C)"
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ) box $U0 $V0 $U1 $V1"
if [ ! -s "$A3C" ]; then
  $PY experiments/aerial3dg_batch.py compile --robot "$ROBOT" --box $U0 $V0 $U1 $V1 --a3c "$A3C" --out "$(dirname $A3C)/$(basename $A3C .a3c).compile.json"
fi
$PY experiments/aerial3dg_batch.py task --robot "$ROBOT" --pairs "$PAIRS" --a3c "$A3C" --n-tasks $NT --task $T --out-dir "$OUT"
echo "end $(date -u +%FT%TZ)"
