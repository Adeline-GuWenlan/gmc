#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=ul_viz
#SBATCH --time=01:30:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=4
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# L2 visualisations (EWA keyframes, flythrough MP4, altitude profile, necessity panels, plotly).
set -euo pipefail
cd /scratch/wg2381/splathjb-uavlamp/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=4 NUMBA_NUM_THREADS=4
echo "node $(hostname) job ${SLURM_JOB_ID} start $(date -u +%FT%TZ)"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/uavlamp_viz.py "$@"
echo "end $(date -u +%FT%TZ)"
