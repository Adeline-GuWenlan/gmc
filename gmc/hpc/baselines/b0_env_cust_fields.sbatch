#!/bin/bash
#SBATCH --job-name=bl_b0_env_cust_fields
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=4G
#SBATCH --time=01:00:00
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# bl B0: build the cust_fields env (README: python 3.10 + numpy matplotlib pyyaml).
# Run ALONE (no other conda create/install running): parallel creates corrupted the shared pkgs
# cache on 2026-09-24. This job uses its own fresh CONDA_PKGS_DIRS and pip cache.
set -euo pipefail
source /share/apps/anaconda3/2025.06/etc/profile.d/conda.sh
ENV_NAME=cust_fields
E=/scratch/wg2381/.conda/envs/$ENV_NAME
PK=/scratch/wg2381/.conda/pkgs_bl/$ENV_NAME/$SLURM_JOB_ID
mkdir -p "$PK/conda" "$PK/pip"
export CONDA_PKGS_DIRS=$PK/conda PIP_CACHE_DIR=$PK/pip PIP_NO_INPUT=1 XDG_CACHE_HOME=/scratch/wg2381/.cache
echo "=== $(date -Is) host=$(hostname) job=$SLURM_JOB_ID env=$E pkgs=$CONDA_PKGS_DIRS"
conda --version
[ -e "$E" ] && { echo "env $E already exists; refusing to overwrite"; exit 3; }

echo "=== $(date +%T) conda create"
conda create -y -p "$E" --override-channels -c conda-forge python=3.10 pip
echo "=== $(date +%T) pip"
"$E/bin/pip" install numpy matplotlib pyyaml
echo "=== $(date +%T) verify"
"$E/bin/python" -c "import sys, numpy, matplotlib, yaml; print(sys.version); print('numpy', numpy.__version__, 'matplotlib', matplotlib.__version__, 'pyyaml', yaml.__version__)"
"$E/bin/pip" freeze > "$E/bl_b0_freeze.txt"
echo "=== $(date -Is) done"
