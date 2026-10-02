#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f2_pilot
#SBATCH --time=02:00:00
#SBATCH --mem=3G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# F2 Task 1b pilot: build the shared robust-body lattice for a box once, then one audited sampling stream.
# Usage: sbatch f2_pilot.sbatch TAG U0 V0 U1 V1 N_CANDIDATES [extra run args...]
set -euo pipefail
TAG=$1; U0=$2; V0=$3; U1=$4; V1=$5; N=$6; shift 6
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
OUT=results/aerial3dg/f2/sample/$TAG
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ) box $U0 $V0 $U1 $V1"
if [ ! -s "$OUT/lattice.npz" ]; then
  $PY experiments/aerial3dg_fail2_sample.py lattice --box $U0 $V0 $U1 $V1 --out "$OUT/lattice.npz" "$@"
fi
$PY experiments/aerial3dg_fail2_sample.py run --box $U0 $V0 $U1 $V1 --lattice "$OUT/lattice.npz" \
    --max-candidates $N --audit --out-dir "$OUT" "$@"
$PY experiments/aerial3dg_fail2_sample.py collect --out-dir "$OUT" --out "$OUT/pilot_collect.json"
echo "end $(date -u +%FT%TZ)"
