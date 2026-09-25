#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=uc_pytest
#SBATCH --time=02:00:00
#SBATCH --mem=8G
#SBATCH --cpus-per-task=2
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# C1: gs3d + uavlamp + aerial3d unit tests (synthetic fixtures only; no real archive).
set -euo pipefail
cd /scratch/wg2381/splathjb-uavconn/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
mkdir -p outputs/uavconn
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
git status --short -- src experiments tests | head -30
/scratch/wg2381/.conda/envs/gmc-venv/bin/python -m pytest -q -p no:cacheprovider \
    tests/unit/test_gs3d_*.py tests/unit/test_uavlamp_*.py tests/unit/test_aerial3d_*.py \
    --durations=25 --junitxml=outputs/uavconn/pytest_junit_${SLURM_JOB_ID}.xml || echo "PYTEST_EXIT $?"
echo "end $(date -u +%FT%TZ)"
