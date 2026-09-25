#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=uc_synth
#SBATCH --time=06:00:00
#SBATCH --mem=8G
#SBATCH --cpus-per-task=2
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# C1: synthetic acceptance cases, aerial3d vs LatticePlanner preview, figures.
# Usage: sbatch c1_synthetic.sbatch OUTDIR [--only case ...]
set -euo pipefail
cd /scratch/wg2381/splathjb-uavconn/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
OUT=$1; shift
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
git status --short -- src experiments | head -30
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/aerial3d_synthetic.py --out "$OUT" "$@"
echo "end $(date -u +%FT%TZ)"
