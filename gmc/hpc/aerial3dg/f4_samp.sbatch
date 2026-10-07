#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f4_samp
#SBATCH --time=05:00:00
#SBATCH --mem=2000M
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%A_%a.out
# F4 Task 1: one independent seeded sampler stream per array task (stream = SLURM_ARRAY_TASK_ID, seed = SEED +
# 1000*stream) of F3's real-body confirmed-reachable sampler (aerial3dg_fail3_sample.py run, uniform mode), on F3's
# region box and shared pilot lattice.  Resumable (the stream JSONL is appended; re-submit continues it).
# Usage: sbatch --job-name=a3f4_samp_<R> --array=101-10N f4_samp.sbatch REGION SEED QUOTA U0 V0 U1 V1
set -euo pipefail
REG=$1; SEED=$2; Q=$3; shift 3
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
echo "node $(hostname) job ${SLURM_JOB_ID}_${SLURM_ARRAY_TASK_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ) $REG box $*"
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/aerial3dg_fail3_sample.py run --box "$@" \
  --stream "$SLURM_ARRAY_TASK_ID" --seed "$SEED" --quota "$Q" --max-hours 4.8 \
  --lattice results/aerial3dg/f3/sample/$REG/pilot/stream_01.lattice.npz --out-dir results/aerial3dg/f4/sample/$REG
echo "end $(date -u +%FT%TZ)"
