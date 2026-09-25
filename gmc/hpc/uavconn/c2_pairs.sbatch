#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=uc_pairs
#SBATCH --time=00:45:00
#SBATCH --mem=6G
#SBATCH --cpus-per-task=2
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# C2: generate the pre-registered extended start/goal set (docs/uavconn_c2_prereg.md §2).
set -euo pipefail
cd /scratch/wg2381/splathjb-uavconn/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/uavconn_pairs.py configs/uavlamp_l2/M1_main_low_start.json \
    --uavlamp-root /scratch/wg2381/splathjb-uavlamp/gmc --out results/uavconn/extended --spec-dir configs/uavconn_c2/ext
echo "end $(date -u +%FT%TZ)"
