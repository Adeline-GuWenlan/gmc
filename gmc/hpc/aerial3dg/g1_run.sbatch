#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3g_run
#SBATCH --time=01:00:00
#SBATCH --mem=12G
#SBATCH --cpus-per-task=4
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# aerial3d ground bodies on the real uav-lamp archive (G1).
# Usage: sbatch [--job-name/--mem/--time overrides] g1_run.sbatch {probe|screen|demo} ARGS...
set -euo pipefail
cd /scratch/wg2381/splathjb-aerial3dg/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=${SLURM_CPUS_PER_TASK:-4} OPENBLAS_NUM_THREADS=${SLURM_CPUS_PER_TASK:-4}
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
git status --short -- src experiments | head -20
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/aerial3dg_run.py "$@"
echo "end $(date -u +%FT%TZ)"
