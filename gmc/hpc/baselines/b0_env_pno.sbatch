#!/bin/bash
#SBATCH --job-name=bl_b0_env_pno
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=02:00:00
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# bl B0: build the PNO env.
# The repo's environment.yml (275 exact pins: py3.8, torch 2.1.0/cu121, pyrocko/qt channels) is for the 3D/4D
# experiments and is not solvable on conda-forge-strict. The README pins the 2D example (the one we use:
# pretrained 2D models, zero-shot) to "Python 3.10, PyTorch 2.5.1 and the compatible numpy". So: py3.10 +
# torch 2.5.1 (cu121 wheel; runs on CPU too) + only what examples/ and 2D_Neural_Heuristics/ import.
# Not installed: pykonal / scikit-fmm (training-data generators only), jupyter (we run headless scripts).
# Run ALONE (no other conda create/install running), own fresh CONDA_PKGS_DIRS.
set -eo pipefail   # no -u: conda's reactivation sources base activate.d scripts that use unset vars (job 19506296)
source /share/apps/anaconda3/2025.06/etc/profile.d/conda.sh
ENV_NAME=pno
E=/scratch/wg2381/.conda/envs/$ENV_NAME
PK=/scratch/wg2381/.conda/pkgs_bl/$ENV_NAME/$SLURM_JOB_ID
mkdir -p "$PK/conda" "$PK/pip"
export CONDA_PKGS_DIRS=$PK/conda PIP_CACHE_DIR=$PK/pip PIP_NO_INPUT=1 XDG_CACHE_HOME=/scratch/wg2381/.cache TMPDIR=$PK
echo "=== $(date -Is) host=$(hostname) job=$SLURM_JOB_ID env=$E pkgs=$CONDA_PKGS_DIRS"
[ -e "$E" ] && { echo "env $E already exists; refusing to overwrite"; exit 3; }

echo "=== $(date +%T) conda create"
conda create -y -p "$E" --override-channels -c conda-forge python=3.10 pip
echo "=== $(date +%T) torch 2.5.1 (README)"
"$E/bin/pip" install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu121
echo "=== $(date +%T) pip deps"
"$E/bin/pip" install "numpy<2" scipy matplotlib scikit-learn prettytable tqdm pandas h5py shapely pqdict
echo "=== $(date +%T) verify"
cd /tmp
"$E/bin/python" -c "import sys, torch, numpy, scipy, sklearn, matplotlib; print(sys.version.split()[0], 'torch', torch.__version__, 'cuda build', torch.version.cuda, 'numpy', numpy.__version__, 'scipy', scipy.__version__); print('cuda available here (CPU node, expect False):', torch.cuda.is_available())"
"$E/bin/pip" freeze > "$E/bl_b0_freeze.txt"
rm -rf "$PK/pip"   # the torch wheels are ~3 GB; the env keeps what it needs
echo "=== $(date -Is) done"
