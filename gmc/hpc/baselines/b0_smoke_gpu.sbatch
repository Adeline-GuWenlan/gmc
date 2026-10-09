#!/bin/bash
#SBATCH --job-name=bl_b0_smoke_gpu
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=l40s_public
#SBATCH --gres=gpu:1
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=24G
#SBATCH --time=01:15:00
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# bl B0 GPU smokes, one allocation for all three so the l40s queue is paid once:
#   1. splatnav: torch.cuda check + Splat-Plan on a synthetic Gaussian scene (bl_smoke_splatnav.py)
#   2. foci:     demos/stonehenge.py headless on the MUMPS copy (bl_smoke_foci.py)
#   3. pno:      the CPU smoke's evaluation again on cuda (latency on the GPU, same numbers expected)
# PARTS (env var, default all three) picks which run, e.g. sbatch --export=ALL,PARTS=foci.
# Parts are independent: a failure in one does not stop the others. Nothing is served (no viser/web server).
set -o pipefail
WT=/scratch/wg2381/splathjb-baselines
ENVS=/scratch/wg2381/.conda/envs
OUT=$WT/gmc/results/baselines/b0_smoke
export MPLBACKEND=Agg PYTHONDONTWRITEBYTECODE=1 XDG_CACHE_HOME=/scratch/wg2381/.cache \
       MPLCONFIGDIR=/scratch/wg2381/.cache/mpl WARP_CACHE_PATH=/scratch/wg2381/.cache/warp
echo "=== $(date -Is) host=$(hostname) job=$SLURM_JOB_ID gpus=${CUDA_VISIBLE_DEVICES:-none}"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
cd "$WT/gmc/experiments"
rc=()
PARTS=${PARTS:-splatnav foci pno}; echo "PARTS: $PARTS"
want(){ [[ " $PARTS " == *" $1 "* ]]; }

if want splatnav; then
echo; echo "=== $(date +%T) [1] splatnav"
mkdir -p "$OUT/splatnav"
"$ENVS/splatnav/bin/python" -B bl_smoke_splatnav.py --repo /scratch/wg2381/ext_repos/splatnav \
  --out "$OUT/splatnav" 2>&1 | tee "$OUT/splatnav/log.txt"; rc+=("splatnav=$?")
fi

if want foci; then
echo; echo "=== $(date +%T) [2] foci"
mkdir -p "$OUT/foci"
"$ENVS/foci/bin/python" -B bl_smoke_foci.py --repo /scratch/wg2381/ext_repos/foci_bl \
  --out "$OUT/foci" 2>&1 | tee "$OUT/foci/log.txt"; rc+=("foci=$?")
fi

if want pno; then
echo; echo "=== $(date +%T) [3] pno on cuda"
mkdir -p "$OUT/pno_cuda"
P=$WT/gmc/outputs/baselines/pno
"$ENVS/pno/bin/python" -B bl_smoke_pno.py --repo /scratch/wg2381/ext_repos/PNO --models "$P/models" \
  --data "$P/dataset/cityData/256x256" --out "$OUT/pno_cuda" --device cuda --time_inst 100 2>&1 \
  | tee "$OUT/pno_cuda/log.txt"; rc+=("pno=$?")
fi

echo; echo "=== $(date -Is) done: ${rc[*]}"
