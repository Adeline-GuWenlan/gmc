#!/bin/bash
# gmc/hpc/percase_render_splat.sbatch -- Amendment 2 videos rendered with the real EWA Gaussian
# rasteriser (gmc.height.ewa): no opacity threshold, no subsample, every splat an anisotropic
# alpha-blended footprint. CPU only -- this cluster has no CUDA toolkit module and no nvcc.
#
#   preview stills for one robot (cheap, look at these first):
#     sbatch --job-name=pc_sweeper_stills --time=00:40:00 \
#       --export=ALL,ARGS="--robot sweeper --stills 6" gmc/hpc/percase_render_splat.sbatch
#   full run for one robot:
#     sbatch --job-name=pc_sweeper_splat \
#       --export=ALL,ARGS="--robot sweeper" gmc/hpc/percase_render_splat.sbatch
#
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --job-name=pc_render_splat
#SBATCH --time=02:00:00
#SBATCH --mem=64G
#SBATCH --cpus-per-task=32
#SBATCH --output=/scratch/wg2381/splathjb-percase/gmc/logs/%x-%j.out
set -euo pipefail
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb-percase/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
# numba drives the parallelism (prange over screen tiles); "workqueue" avoids the TBB shutdown hang.
export NUMBA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-8}"
export NUMBA_THREADING_LAYER=workqueue
export OPENBLAS_NUM_THREADS=4 OMP_NUM_THREADS=4 MKL_NUM_THREADS=4
echo "=== percase_render_splat ${ARGS:-} $(date -Is) host=$(hostname) job=${SLURM_JOB_ID} \
cpus=${SLURM_CPUS_PER_TASK:-?} ==="
# shellcheck disable=SC2086
$PY experiments/percase_render.py ${ARGS:-}
echo "=== PERCASE_RENDER_SPLAT_DONE ==="
