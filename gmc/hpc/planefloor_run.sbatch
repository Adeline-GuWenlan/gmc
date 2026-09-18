#!/bin/bash
# gmc/hpc/planefloor_run.sbatch -- Amendment 3 planning run: one robot on one case.
#   sbatch --job-name=pf_<robot>_run \
#     --export=ALL,ROBOT=<robot>,CASE=<path>,OUTDIR=<dir>[,RAWSUB=<sub>][,BUDGET=2][,SKIP_PROBE=1][,SCENE=processed] \
#     gmc/hpc/planefloor_run.sbatch
# CASE and OUTDIR are relative to /scratch/wg2381/splathjb-plane/gmc.
# Sized from sacct on 09-17/18 runs: GMC is single-threaded (cylinder used 0.99 of 8 cores over
# 2.5 h) and peak RSS was 2.8-6.5 GB, 14.2 GB for the 156k-support cylinder map. Raise --mem
# per job with sbatch --mem=... when a bigger map needs it; check MaxRSS with sacct first.
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=pf_run
#SBATCH --time=24:00:00
#SBATCH --mem=20G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/splathjb-plane/gmc/logs/%x-%j.out
# No --partition line on purpose (24 h jobs route to cs; see height_showcase_robot.sbatch).
set -euo pipefail
: "${ROBOT:?set ROBOT}"
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb-plane/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
ARGS=(--robot "${ROBOT}" --tau "${TAU:-0.3}" --budget-scale "${BUDGET:-1}"
      --case "${CASE:?set CASE}" --scene "${SCENE:-planefloor}"
      --out-dir "${OUTDIR:?set OUTDIR}" --raw-subdir "${RAWSUB:-plane}")
[ "${SKIP_PROBE:-0}" = "1" ] && ARGS+=(--skip-probe)
echo "=== planefloor_run ${ARGS[*]} $(date -Is) host=$(hostname) job=${SLURM_JOB_ID} ==="
$PY experiments/showcase_run.py "${ARGS[@]}"
echo "=== PLANEFLOOR_RUN_DONE ${ROBOT} ==="
