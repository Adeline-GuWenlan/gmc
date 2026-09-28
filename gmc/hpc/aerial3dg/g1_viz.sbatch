#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3g_viz
#SBATCH --time=02:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=4
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# G1 archive-backed figures + videos (uavlamp_viz / ewa machinery). Usage: sbatch g1_viz.sbatch ARGS...
set -euo pipefail
cd /scratch/wg2381/splathjb-aerial3dg/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=4 NUMBA_NUM_THREADS=4
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/aerial3dg_viz.py "$@"
echo "end $(date -u +%FT%TZ)"
