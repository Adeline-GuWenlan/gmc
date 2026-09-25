#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=g5_sample
#SBATCH --time=02:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=2
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# G1 Task 1: one sampler step (evidence | lattice | sample | plot); see ground5k_sample.py.
set -euo pipefail
cd /scratch/wg2381/splathjb-ground5k/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
echo "node $(hostname) job ${SLURM_JOB_ID} cpus ${SLURM_CPUS_PER_TASK} start $(date -u +%FT%TZ) args $*"
for step in "$@"; do
  case "$step" in --*) continue;; esac
  /scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/ground5k_sample.py "$step" ${G5_ARGS:-}
done
echo "end $(date -u +%FT%TZ)"
