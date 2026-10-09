#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f5_render3d
#SBATCH --time=01:00:00
#SBATCH --mem=12G
#SBATCH --cpus-per-task=2
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# F5 follow-up: 3D renders of one failure case (stills + MP4) from the scene_v2 archive. Usage: sbatch f5_render3d.sbatch [ARGS]
set -euo pipefail
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=${SLURM_CPUS_PER_TASK} NUMBA_NUM_THREADS=${SLURM_CPUS_PER_TASK}
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/aerial3dg_fail5_render3d.py "$@"
echo "end $(date -u +%FT%TZ)"
