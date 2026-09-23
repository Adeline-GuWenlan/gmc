#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=ul_l2run
#SBATCH --time=04:00:00
#SBATCH --mem=8G
#SBATCH --cpus-per-task=2
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# L2: one uavlamp_run.py invocation. The defaults above fit one ~2 h query; per-query values are
# overridden on the sbatch command line (see docs/uavlamp_report.md for the exact commands).
set -euo pipefail
cd /scratch/wg2381/splathjb-uavlamp/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
echo "node $(hostname) job ${SLURM_JOB_ID} cpus ${SLURM_CPUS_PER_TASK} start $(date -u +%FT%TZ)"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/uavlamp_run.py "$@"
echo "end $(date -u +%FT%TZ)"
