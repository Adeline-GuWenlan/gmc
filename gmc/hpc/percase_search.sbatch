#!/bin/bash
# gmc/hpc/percase_search.sbatch -- Amendment 2 case search for one robot.
#   sbatch --job-name=pc_<robot>_search --export=ALL,SCRIPT=experiments/percase_search_<robot>.py[,ARGS="..."] gmc/hpc/percase_search.sbatch
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=pc_search
#SBATCH --time=04:00:00
#SBATCH --mem=64G
#SBATCH --cpus-per-task=8
#SBATCH --output=/scratch/wg2381/splathjb-percase/gmc/logs/%x-%j.out
set -euo pipefail
: "${SCRIPT:?set SCRIPT}"
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb-percase/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=8 OMP_NUM_THREADS=8 MKL_NUM_THREADS=8
echo "=== percase_search ${SCRIPT} ${ARGS:-} $(date -Is) host=$(hostname) job=${SLURM_JOB_ID} ==="
# shellcheck disable=SC2086
$PY "${SCRIPT}" ${ARGS:-}
echo "=== PERCASE_SEARCH_DONE ${SCRIPT} ==="
