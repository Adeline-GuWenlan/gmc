#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=512M
#SBATCH --time=00:15:00
#SBATCH --job-name=gs3d_dispatch
#SBATCH --output=/scratch/wg2381/codex_jobs/gs3d_20260921/logs/%x-%j.out
#SBATCH --error=/scratch/wg2381/codex_jobs/gs3d_20260921/logs/%x-%j.err
set -euo pipefail
export PATH=/home/wg2381/.npm-global/bin:/home/wg2381/.local/bin:$PATH
export PYTHONUNBUFFERED=1
exec python3 /scratch/wg2381/codex_jobs/gs3d_20260921/scheduler.py dispatch
