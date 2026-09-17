#!/bin/bash
# gmc/hpc/planefloor_step.sbatch -- Amendment 3 P1: one --step of experiments/planefloor_build.py.
#   sbatch --job-name=pf_p1a_diagnose --export=ALL,STEP=diagnose gmc/hpc/planefloor_step.sbatch
# Everything that loads the 7.3 M-splat showcase scene goes through here, never the login node.
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=pf_p1_step
#SBATCH --time=04:00:00
#SBATCH --mem=64G
#SBATCH --cpus-per-task=8
#SBATCH --output=/scratch/wg2381/splathjb-plane/gmc/logs/%x-%j.out
set -euo pipefail
: "${STEP:?set STEP=diagnose|build|survivors|evidence}"
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb-plane/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg PYTHONUNBUFFERED=1
export OPENBLAS_NUM_THREADS=8 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
echo "=== planefloor_step ${STEP} $(date -Is) host=$(hostname) job=${SLURM_JOB_ID} ==="
$PY experiments/planefloor_build.py --step "${STEP}" ${ARGS:-}
echo "=== PLANEFLOOR_STEP_DONE ${STEP} ==="
