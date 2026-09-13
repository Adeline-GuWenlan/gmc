#!/bin/bash
# gmc/hpc/height_toy.sbatch -- one T1/T2 gate per job; submit with --export=ALL,GATE=<gate>
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=height_toy
#SBATCH --time=12:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=4
#SBATCH --output=/scratch/wg2381/splathjb-height/gmc/logs/%x-%j.out
# No --partition line: >6 h jobs must route to cs (see k2_door_gate_stage4.sbatch).
set -euo pipefail
: "${GATE:?set GATE}"
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb-height/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=4 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
echo "=== height_toy GATE=${GATE} $(date -Is) host=$(hostname) job=${SLURM_JOB_ID} ==="
$PY experiments/height_toy.py --gate "${GATE}"
echo "=== HEIGHT_TOY_DONE ${GATE} ==="
