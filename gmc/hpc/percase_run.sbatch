#!/bin/bash
# gmc/hpc/percase_run.sbatch -- Amendment 2 planning run for one robot on its own case.
#   sbatch --job-name=pc_<robot>_run --export=ALL,ROBOT=<robot>[,BUDGET=2][,SKIP_PROBE=1] gmc/hpc/percase_run.sbatch
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=pc_run
#SBATCH --time=24:00:00
#SBATCH --mem=64G
#SBATCH --cpus-per-task=8
#SBATCH --output=/scratch/wg2381/splathjb-percase/gmc/logs/%x-%j.out
# No --partition line on purpose (24 h jobs route to cs; see height_showcase_robot.sbatch).
set -euo pipefail
: "${ROBOT:?set ROBOT}"
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb-percase/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=8 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
ARGS=(--robot "${ROBOT}" --tau 0.3 --budget-scale "${BUDGET:-1}"
      --case "results/height/percase/${ROBOT}/case.json"
      --out-dir "results/height/percase/${ROBOT}" --raw-subdir percase)
[ "${SKIP_PROBE:-0}" = "1" ] && ARGS+=(--skip-probe)
echo "=== percase_run ${ARGS[*]} $(date -Is) host=$(hostname) job=${SLURM_JOB_ID} ==="
$PY experiments/showcase_run.py "${ARGS[@]}"
echo "=== PERCASE_RUN_DONE ${ROBOT} ==="
