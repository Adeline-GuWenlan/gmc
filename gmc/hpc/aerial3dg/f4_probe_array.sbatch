#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f4_probe
#SBATCH --time=04:00:00
#SBATCH --mem=2000M
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%A_%a.out
# F4 Task 3: run line SLURM_ARRAY_TASK_ID (1-based) of a probe plan file ("MODE REGION ROBOT ROWS OUT" per line)
# through f4_probe.sbatch's logic.  Usage: sbatch --array=1-N%8 f4_probe_array.sbatch PLAN
set -euo pipefail
LINE=$(sed -n "${SLURM_ARRAY_TASK_ID}p" "$1")
echo "plan line ${SLURM_ARRAY_TASK_ID}: $LINE"
exec bash /scratch/wg2381/splathjb-aerial3dg-fail/gmc/hpc/aerial3dg/f4_probe.sbatch $LINE
