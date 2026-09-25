#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=uc_run
#SBATCH --time=01:30:00
#SBATCH --mem=6G
#SBATCH --cpus-per-task=2
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# C2: aerial3d on the real uav-lamp archive -- one compile per scene variant, then start+goal queries
# (cold + warm) with the baseline's fresh-oracle replay.  Usage: sbatch c2_run.sbatch OUTDIR SPEC [SPEC ...]
set -euo pipefail
WT=/scratch/wg2381/splathjb-uavconn/gmc
cd "$WT"
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
OUT=$1; shift
echo "node $(hostname) job ${SLURM_JOB_ID} cpus ${SLURM_CPUS_PER_TASK} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
git status --short -- src experiments configs | head -20
scontrol show node "$(hostname -s)" | grep -oE '(CPUAlloc|CPUTot|AllocMem|RealMemory)=[0-9]+' | tr '\n' ' '; echo
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/uavconn_run.py "$@" --output "$OUT" --warm 3 \
    --uavlamp-root /scratch/wg2381/splathjb-uavlamp/gmc
echo "end $(date -u +%FT%TZ)"
