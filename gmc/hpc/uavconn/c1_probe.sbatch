#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=uc_probe
#SBATCH --time=06:00:00
#SBATCH --mem=24G
#SBATCH --cpus-per-task=2
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# C1 scale probe: compile the aerial3d complex on the real uav-lamp archive for one spec's box
# (+ one query).  Usage: sbatch c1_probe.sbatch SPEC OUTDIR [extra probe args]
set -euo pipefail
cd /scratch/wg2381/splathjb-uavconn/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
SPEC=$1; OUT=$2; shift 2
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
git status --short -- src experiments | head -20
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/aerial3d_probe.py "$SPEC" \
    --uavlamp-root /scratch/wg2381/splathjb-uavlamp/gmc --output "$OUT" "$@"
echo "end $(date -u +%FT%TZ)"
