#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=12G
#SBATCH --time=01:00:00
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# Generic: run one gmc experiment script with arguments.
set -euo pipefail
cd /scratch/wg2381/splathjb-uavlamp/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=2 NUMBA_NUM_THREADS=2
/scratch/wg2381/.conda/envs/gmc-venv/bin/python "$@"
