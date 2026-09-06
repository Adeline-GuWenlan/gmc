#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=gmc_coreset_pareto
#SBATCH --time=04:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=4
#SBATCH --output=/scratch/wg2381/splathjb/gmc/logs/%x-%j.out
set -x
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb/gmc
export PYTHONPATH=src MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=4 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
$PY experiments/coreset_pareto.py -o results/coreset/pareto.json
echo "=== CORESET_PARETO_DONE ==="
