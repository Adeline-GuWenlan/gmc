#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=gmc_coreset_final
#SBATCH --time=08:00:00
#SBATCH --mem=24G
#SBATCH --cpus-per-task=8
#SBATCH --output=/scratch/wg2381/splathjb/gmc/logs/%x-%j.out
set -x
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=8 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
echo "=== FULL SUITE INCLUDING NEW CORESET GATES ==="
$PY -m pytest -q 2>&1 | tail -20
echo "=== END TO END TOY ==="
$PY experiments/coreset_end_to_end.py -o results/coreset/end_to_end.json
echo "=== FIGURES ==="
$PY experiments/coreset_figures.py -o results/coreset/figures
echo "=== CORESET_FINAL_DONE ==="
