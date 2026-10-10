#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=bl_b1_cpu
#SBATCH --time=02:00:00
#SBATCH --mem=4G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# bl B1 CPU step under gmc-venv, from gmc/: runs the bash snippet "$1" with H="python experiments/bl_harness.py"
# and PY=gmc-venv python. Examples (the job log echoes the snippet):
#   sbatch --job-name=bl_b1_export b1_cpu.sbatch '$H export && BL_HEAVY=1 $PY -m pytest -q tests/unit/test_bl_harness.py'
#   sbatch --job-name=bl_b1_astar  b1_cpu.sbatch '$H run --method astar_replay --region S --robot cylinder --pairs tuning --out ...'
set -uo pipefail
cd /scratch/wg2381/splathjb-baselines/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
H="$PY experiments/bl_harness.py"
echo "node $(hostname) job $SLURM_JOB_ID commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
echo "snippet: $1"
eval "$1"
rc=$?
echo "end $(date -u +%FT%TZ) rc=$rc"
exit $rc
