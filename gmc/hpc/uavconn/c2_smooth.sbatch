#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=uc_smooth
#SBATCH --time=01:00:00
#SBATCH --mem=6G
#SBATCH --cpus-per-task=2
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# C2: the same A6 smoothing on both methods' raw paths, each curve freshly re-verified.
# Usage: sbatch c2_smooth.sbatch --template SPEC --out DIR --run method:name=result.json ...
set -euo pipefail
cd /scratch/wg2381/splathjb-uavconn/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/uavconn_smooth.py "$@" \
    --uavlamp-root /scratch/wg2381/splathjb-uavlamp/gmc
echo "end $(date -u +%FT%TZ)"
