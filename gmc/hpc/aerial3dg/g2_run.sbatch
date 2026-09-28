#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3g_g2
#SBATCH --time=00:45:00
#SBATCH --mem=2G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# G2 single steps (sample / collect).  Usage: sbatch [overrides] g2_run.sbatch {sample|collect} ARGS...
# 2 GB: cpu_short QOS is 120 GB/user and ground5k's throttled arrays hold ~112 GB (G2 worklog).
set -euo pipefail
cd /scratch/wg2381/splathjb-aerial3dg/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
git status --short -- src experiments | head -20
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/aerial3dg_batch.py "$@"
echo "end $(date -u +%FT%TZ)"
