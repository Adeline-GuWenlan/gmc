#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=l40s_public
#SBATCH --gres=gpu:1
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=bl_b1_gpu
#SBATCH --time=02:00:00
#SBATCH --mem=24G
#SBATCH --cpus-per-task=4
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# bl B1 GPU step: the harness (gmc-venv, judge on CPU) drives a method worker (splatnav / foci env) on the GPU.
# Runs the bash snippet "$1" from gmc/ with H="python experiments/bl_harness.py" (gmc-venv). Several harness runs
# may be started in the background inside one snippet (they share the GPU); the snippet must `wait` for them.
set -uo pipefail
cd /scratch/wg2381/splathjb-baselines/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
       XDG_CACHE_HOME=/scratch/wg2381/.cache MPLCONFIGDIR=/scratch/wg2381/.cache/mpl \
       WARP_CACHE_PATH=/scratch/wg2381/.cache/warp
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
H="$PY experiments/bl_harness.py"
echo "node $(hostname) job $SLURM_JOB_ID gpus ${CUDA_VISIBLE_DEVICES:-none} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
echo "snippet: $1"
eval "$1"
rc=$?
nvidia-smi --query-gpu=memory.used --format=csv,noheader
echo "end $(date -u +%FT%TZ) rc=$rc"
exit $rc
