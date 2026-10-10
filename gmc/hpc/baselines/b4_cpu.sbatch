#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=bl_b4_cpu
#SBATCH --time=02:00:00
#SBATCH --mem=4G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# bl B4 CPU step under gmc-venv, from gmc/: runs the bash snippet "$1" with PY=gmc-venv python.
#   sbatch --job-name=bl_b4_analyze b4_cpu.sbatch '$PY experiments/bl_analyze.py all'
set -uo pipefail
cd /scratch/wg2381/splathjb-baselines/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
echo "node $(hostname) job $SLURM_JOB_ID commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
echo "snippet: $1"
eval "$1"
rc=$?
echo "end $(date -u +%FT%TZ) rc=$rc"
exit $rc
