#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=gmc_baseline
#SBATCH --time=02:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=8
#SBATCH --output=/scratch/wg2381/splathjb/gmc/logs/%x-%j.out
set -x
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb/gmc
export PYTHONPATH=src MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
echo "=== VERSIONS ==="
$PY -c "import sys,numpy,scipy,shapely,networkx;print(sys.version.split()[0],numpy.__version__,scipy.__version__,shapely.__version__,networkx.__version__)"
echo "=== PYTEST FULL SUITE ==="
$PY -m pytest -q 2>&1 | tail -40
echo "=== BASELINE_SUITE_DONE rc=$? ==="
