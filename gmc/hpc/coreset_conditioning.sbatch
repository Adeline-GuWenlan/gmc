#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=gmc_coreset_cond
#SBATCH --time=06:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=4
#SBATCH --output=/scratch/wg2381/splathjb/gmc/logs/%x-%j.out
set -x
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=4 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
$PY experiments/coreset_figures.py -o results/coreset/figures
$PY experiments/coreset_robot_conditioning.py -o results/coreset/robot_conditioning.json
$PY experiments/coreset_pareto.py -o results/coreset/pareto_v2.json
echo "=== CORESET_COND_DONE ==="
