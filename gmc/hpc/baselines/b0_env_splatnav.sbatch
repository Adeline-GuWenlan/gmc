#!/bin/bash
#SBATCH --job-name=bl_b0_env_splatnav
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=02:00:00
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# bl B0: build the splatnav env, minimal set for Splat-Plan only.
# The README builds on nerfstudio; nerfstudio's documented stack is py3.10 + torch 2.1.2+cu118 (cu118 supports the
# L40S, sm_89). Splat-Plan's own modules (splatplan/, polytopes/, initialization/, ellipsoids/, SFC/) import only:
# torch numpy scipy open3d clarabel dijkstra3d polytope cvxpy cvxopt unfoldNd sympy matplotlib. nerfstudio is
# imported only by splat/splat_utils.py -> ns_utils (GSplatLoader.load_gsplat_from_nerfstudio), which B1 bypasses
# by handing the Gaussians to the planner directly, so nerfstudio / gsplat / LightGlue / Blender / ompl / viser are
# NOT installed. open3d links libEGL/libGL (absent on the nodes): conda-forge libegl+libgl
# into the env; open3d's RUNPATH ($ORIGIN/../../../ = $E/lib) finds them, no LD_LIBRARY_PATH needed.
# Run ALONE (no other conda create/install running), own fresh CONDA_PKGS_DIRS.
set -eo pipefail   # no -u: conda's reactivation sources base activate.d scripts that use unset vars (job 19506296)
source /share/apps/anaconda3/2025.06/etc/profile.d/conda.sh
ENV_NAME=splatnav
E=/scratch/wg2381/.conda/envs/$ENV_NAME
PK=/scratch/wg2381/.conda/pkgs_bl/$ENV_NAME/$SLURM_JOB_ID
mkdir -p "$PK/conda" "$PK/pip"
export CONDA_PKGS_DIRS=$PK/conda PIP_CACHE_DIR=$PK/pip PIP_NO_INPUT=1 XDG_CACHE_HOME=/scratch/wg2381/.cache TMPDIR=$PK
echo "=== $(date -Is) host=$(hostname) job=$SLURM_JOB_ID env=$E pkgs=$CONDA_PKGS_DIRS"
[ -e "$E" ] && { echo "env $E already exists; refusing to overwrite"; exit 3; }

echo "=== $(date +%T) conda create (+ glvnd libs for open3d)"
conda create -y -p "$E" --override-channels -c conda-forge python=3.10 pip libegl libgl
echo "=== $(date +%T) torch 2.1.2+cu118 (nerfstudio's documented stack)"
"$E/bin/pip" install torch==2.1.2+cu118 torchvision==0.16.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118
echo "=== $(date +%T) Splat-Plan deps (README list minus nerfstudio/LightGlue/viser)"
"$E/bin/pip" install "numpy<2" scipy matplotlib sympy tqdm clarabel dijkstra3d polytope cvxopt cvxpy unfoldNd open3d
echo "=== $(date +%T) verify (CPU node)"
cd /scratch/wg2381/ext_repos/splatnav
export PYTHONDONTWRITEBYTECODE=1
"$E/bin/python" -B -c "
import sys, torch, numpy, open3d, clarabel, dijkstra3d, polytope, cvxpy, cvxopt, unfoldNd
print(sys.version.split()[0], 'torch', torch.__version__, 'cuda build', torch.version.cuda, 'numpy', numpy.__version__, 'open3d', open3d.__version__, 'clarabel', clarabel.__version__)
from splatplan.splatplan import SplatPlan
from splatplan.spline_utils import SplinePlanner
from SFC.corridor_utils import SafeFlightCorridor
print('Splat-Plan modules import OK (no nerfstudio)')
try:
    import splat.splat_utils
except ImportError as e:
    print('splat.splat_utils (GSplatLoader) needs nerfstudio, as expected:', e)
"
"$E/bin/pip" freeze > "$E/bl_b0_freeze.txt"
rm -rf "$PK/pip"
echo "=== $(date -Is) done"
