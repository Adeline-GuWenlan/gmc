#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=4G
#SBATCH --time=02:00:00
#SBATCH --job-name=gs3d_A7
#SBATCH --output=/scratch/wg2381/codex_jobs/gs3d_20260921/logs/%x-%j.out
#SBATCH --error=/scratch/wg2381/codex_jobs/gs3d_20260921/logs/%x-%j.err
set -euo pipefail
exec bash /scratch/wg2381/codex_jobs/gs3d_20260921/run_agent.sh A7
