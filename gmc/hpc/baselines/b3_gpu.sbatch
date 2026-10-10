#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=l40s_public
#SBATCH --gres=gpu:1
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=bl_b3_gpu
#SBATCH --time=05:00:00
#SBATCH --mem=12G
#SBATCH --cpus-per-task=4
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# bl B3 full run of one GPU method ($1 = foci | splatnav | pno): the 4 harness streams of
# results/baselines/b3/plan.json (`bl_b3.py streams`) run concurrently on one L40S, judge inline, frozen config.
# Memory: the pilots' 4-stream jobs peaked at 5.5 GB (B1) / 3.5 GB for 2 PNO streams (B2) -> 12G.
set -uo pipefail
M=$1
cd /scratch/wg2381/splathjb-baselines/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
       XDG_CACHE_HOME=/scratch/wg2381/.cache MPLCONFIGDIR=/scratch/wg2381/.cache/mpl \
       WARP_CACHE_PATH=/scratch/wg2381/.cache/warp
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
H="$PY experiments/bl_harness.py"
echo "node $(hostname) job $SLURM_JOB_ID method $M gpus ${CUDA_VISIBLE_DEVICES:-none} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
mkdir -p results/baselines/b3/gpu_mem
nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader -l 30 \
  > results/baselines/b3/gpu_mem/${M}_${SLURM_JOB_ID}.txt & NV=$!
mapfile -t STREAMS < <($PY experiments/bl_b3.py streams --method "$M")
PIDS=()
for s in "${STREAMS[@]}"; do echo "stream: $s"; ( eval "$s" ) & PIDS+=($!); done
rc=0
for p in "${PIDS[@]}"; do wait "$p" || rc=$?; done
kill $NV
echo "end $(date -u +%FT%TZ) rc=$rc"
exit $rc
