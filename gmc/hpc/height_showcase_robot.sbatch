#!/bin/bash
# gmc/hpc/height_showcase_robot.sbatch -- submit with --export=ALL,ROBOT=<r>[,TAU=0.1][,BUDGET=2][,SKIP_PROBE=1]
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=height_showcase
#SBATCH --time=24:00:00
#SBATCH --mem=64G
#SBATCH --cpus-per-task=8
#SBATCH --output=/scratch/wg2381/splathjb-height/gmc/logs/%x-%j.out
# No --partition line on purpose (24 h jobs route to cs; see k2_door_gate_stage4.sbatch).
set -euo pipefail
: "${ROBOT:?set ROBOT}"
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb-height/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=8 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
ARGS=(--robot "${ROBOT}" --tau "${TAU:-0.3}" --budget-scale "${BUDGET:-1}")
[ "${SKIP_PROBE:-0}" = "1" ] && ARGS+=(--skip-probe)
echo "=== height_showcase ${ARGS[*]} $(date -Is) host=$(hostname) job=${SLURM_JOB_ID} ==="
$PY experiments/showcase_run.py "${ARGS[@]}"
echo "=== HEIGHT_SHOWCASE_DONE ${ROBOT} ==="
