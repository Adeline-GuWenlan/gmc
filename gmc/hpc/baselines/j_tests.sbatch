#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=bl_j_tests
#SBATCH --time=06:00:00
#SBATCH --mem=4G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# bl J (a): the full gmc/tests suite on one code tree, junit xml for the per-test comparison.
# Usage: sbatch --job-name=bl_j_tests_<label> j_tests.sbatch LABEL GMC_ROOT
#   LABEL = head | old;  GMC_ROOT = the gmc/ directory of that tree (worktree gmc/ or outputs/baselines/j/old/gmc)
set -uo pipefail
LABEL=$1; ROOT=$2
OUT=/scratch/wg2381/splathjb-baselines/gmc/results/baselines/j/tests
mkdir -p "$OUT"
cd "$ROOT"
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
echo "node $(hostname) job ${SLURM_JOB_ID} label $LABEL root $ROOT start $(date -u +%FT%TZ)"
grep -c contains_swept_cylinder src/gmc/gs3d/oracle.py
$PY -m pytest tests -p no:cacheprovider -q -ra --durations=25 --junitxml="$OUT/junit_$LABEL.xml"
echo "pytest exit $? end $(date -u +%FT%TZ)"
