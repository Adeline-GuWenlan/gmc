#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=uc_base
#SBATCH --time=05:40:00
#SBATCH --mem=8G
#SBATCH --cpus-per-task=2
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# C2: the baseline (LatticePlanner via the UNCHANGED uavlamp_run.py; this branch's gs3d/uavlamp code
# is byte-identical to uav-lamp 4357962) for one spec, one plan() call.  Same env as L2's l2_run.sbatch.
# Runs from the baseline worktree's gmc/ (read-only) so L2 specs with relative archive paths resolve;
# writes only to OUTDIR.  Usage: sbatch c2_baseline.sbatch SPEC OUTDIR
set -euo pipefail
WT=/scratch/wg2381/splathjb-uavconn/gmc
SPEC=$1; OUT=$2
cd /scratch/wg2381/splathjb-uavlamp/gmc
export PYTHONPATH=$WT/src:$WT/experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1
echo "node $(hostname) job ${SLURM_JOB_ID} cpus ${SLURM_CPUS_PER_TASK} commit $(git -C $WT rev-parse HEAD) start $(date -u +%FT%TZ)"
git -C "$WT" diff --stat 4357962 -- src/gmc/gs3d experiments/uavlamp_run.py experiments/uavlamp_query.py
scontrol show node "$(hostname -s)" | grep -oE '(CPUAlloc|CPUTot|AllocMem|RealMemory)=[0-9]+' | tr '\n' ' '; echo
/scratch/wg2381/.conda/envs/gmc-venv/bin/python $WT/experiments/uavlamp_run.py "$SPEC" --warm 1 --output "$OUT"
scontrol show node "$(hostname -s)" | grep -oE '(CPUAlloc|CPUTot)=[0-9]+' | tr '\n' ' '; echo
echo "end $(date -u +%FT%TZ)"
