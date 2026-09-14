#!/bin/bash
# gmc/hpc/percase_render.sbatch -- Amendment 2 videos (3D point-cloud MP4 + two-panel) for one robot.
#   sbatch --job-name=pc_<robot>_render --export=ALL,SCRIPT=experiments/percase_render.py,ARGS="--robot <robot>" gmc/hpc/percase_render.sbatch
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=pc_render
#SBATCH --time=04:00:00
#SBATCH --mem=32G
#SBATCH --cpus-per-task=4
#SBATCH --output=/scratch/wg2381/splathjb-percase/gmc/logs/%x-%j.out
set -euo pipefail
: "${SCRIPT:?set SCRIPT}"
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb-percase/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=4 OMP_NUM_THREADS=4 MKL_NUM_THREADS=4
echo "=== percase_render ${SCRIPT} ${ARGS:-} $(date -Is) host=$(hostname) job=${SLURM_JOB_ID} ==="
# shellcheck disable=SC2086
$PY "${SCRIPT}" ${ARGS:-}
echo "=== PERCASE_RENDER_DONE ${SCRIPT} ==="
