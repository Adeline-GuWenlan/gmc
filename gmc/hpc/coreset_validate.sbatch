#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=gmc_coreset_validate
#SBATCH --time=08:00:00
#SBATCH --mem=24G
#SBATCH --cpus-per-task=8
#SBATCH --output=/scratch/wg2381/splathjb/gmc/logs/%x-%j.out
set -x
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=8 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
echo "=== DUAL-DOOR CONDITIONING ==="
$PY experiments/coreset_conditioning_dual.py -o results/coreset/conditioning_dual.json
echo "=== SINGLE-DOOR PARETO + CROSS-ROBOT ==="
$PY experiments/coreset_robot_conditioning.py -o results/coreset/robot_conditioning_v2.json
echo "=== FILLED/SURFACE PARETO ==="
$PY experiments/coreset_pareto.py -o results/coreset/pareto_v3.json
echo "=== CORESET_VALIDATE_DONE ==="
