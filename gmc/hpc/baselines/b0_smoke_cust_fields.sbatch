#!/bin/bash
#SBATCH --job-name=bl_b0_smoke_cust_fields
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=2G
#SBATCH --time=00:30:00
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# bl B0 smoke: cust_fields' own test_nf.py, headless (figure saved instead of shown).
set -euo pipefail
E=/scratch/wg2381/.conda/envs/cust_fields
WT=/scratch/wg2381/splathjb-baselines
OUT=$WT/gmc/results/baselines/b0_smoke/cust_fields
export MPLBACKEND=Agg PYTHONDONTWRITEBYTECODE=1 XDG_CACHE_HOME=/scratch/wg2381/.cache MPLCONFIGDIR=/scratch/wg2381/.cache/mpl
echo "=== $(date -Is) host=$(hostname) job=$SLURM_JOB_ID"
mkdir -p "$OUT"; cd "$WT/gmc/experiments"
"$E/bin/python" -B bl_smoke_cust_fields.py --repo /scratch/wg2381/ext_repos/cust_fields --out "$OUT" 2>&1 | tee "$OUT/log.txt"
echo "=== $(date -Is) done"
