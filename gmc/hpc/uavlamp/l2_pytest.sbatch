#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=ul_pytest
#SBATCH --time=01:00:00
#SBATCH --mem=8G
#SBATCH --cpus-per-task=2
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# L2: gs3d + uavlamp unit tests (synthetic fixtures only).
set -euo pipefail
cd /scratch/wg2381/splathjb-uavlamp/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=2
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ)"
git status --short -- src experiments tests | head -20
/scratch/wg2381/.conda/envs/gmc-venv/bin/python -m pytest -q -p no:cacheprovider \
    tests/unit/test_gs3d_*.py tests/unit/test_uavlamp_*.py \
    --junitxml=outputs/uavlamp/l2/pytest_junit.xml
echo "end $(date -u +%FT%TZ)"
