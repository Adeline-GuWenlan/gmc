#!/bin/bash
#SBATCH --job-name=bl_b0_smoke_pno
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=00:45:00
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# bl B0 smoke: PNO pretrained 2D models, zero-shot, the notebook's §3 evaluation on City 256x256, on CPU.
set -eo pipefail
E=/scratch/wg2381/.conda/envs/pno
WT=/scratch/wg2381/splathjb-baselines
P=$WT/gmc/outputs/baselines/pno
OUT=$WT/gmc/results/baselines/b0_smoke/pno_cpu
export MPLBACKEND=Agg PYTHONDONTWRITEBYTECODE=1 XDG_CACHE_HOME=/scratch/wg2381/.cache MPLCONFIGDIR=/scratch/wg2381/.cache/mpl
export OMP_NUM_THREADS=$SLURM_CPUS_PER_TASK
echo "=== $(date -Is) host=$(hostname) job=$SLURM_JOB_ID"
mkdir -p "$OUT"; cd "$WT/gmc/experiments"
"$E/bin/python" -B bl_smoke_pno.py --repo /scratch/wg2381/ext_repos/PNO --models "$P/models" \
  --data "$P/dataset/cityData/256x256" --out "$OUT" --device cpu 2>&1 | tee "$OUT/log.txt"
echo "=== $(date -Is) done"
