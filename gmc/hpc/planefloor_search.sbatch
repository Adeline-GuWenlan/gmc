#!/bin/bash
# gmc/hpc/planefloor_search.sbatch -- Amendment 3 case search (generic, SCRIPT-driven).
#   sbatch --job-name=pf_<robot>_search --export=ALL,SCRIPT=experiments/percase_search_<robot>.py[,ARGS="..."] gmc/hpc/planefloor_search.sbatch
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=pf_search
#SBATCH --time=04:00:00
#SBATCH --mem=8G
#SBATCH --cpus-per-task=2
#SBATCH --output=/scratch/wg2381/splathjb-plane/gmc/logs/%x-%j.out
set -euo pipefail
: "${SCRIPT:?set SCRIPT}"
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb-plane/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
echo "=== planefloor_search ${SCRIPT} ${ARGS:-} $(date -Is) host=$(hostname) job=${SLURM_JOB_ID} ==="
# shellcheck disable=SC2086
$PY "${SCRIPT}" ${ARGS:-}
echo "=== PLANEFLOOR_SEARCH_DONE ${SCRIPT} ==="
