#!/bin/bash
#SBATCH --job-name=bl_b0_env_foci_fix
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=4G
#SBATCH --time=01:00:00
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# bl B0: repair the existing foci env (built 2026-09-24) instead of rebuilding it.
#  1. open3d (pip 0.20.0) links libEGL.so.1 + libGL.so.1, which the nodes lack. FOCI needs open3d for real
#     (initial_guess.py builds its A* occupancy as an o3d VoxelGrid), so guarding the import is not enough.
#     Fix: the glvnd dispatch libraries from conda-forge (libegl, libgl) into the env, found through
#     LD_LIBRARY_PATH=$E/lib. No system packages, nothing outside the env.
#  2. `foci` was installed editable from the pristine clone; re-point it at the patched copy
#     /scratch/wg2381/ext_repos/foci_bl (MA27 -> MUMPS, gmc/baselines/patches/foci_mumps.patch).
# Run ALONE (no other conda install running), own fresh CONDA_PKGS_DIRS.
set -eo pipefail   # no -u: conda's reactivation sources base activate.d scripts that use unset vars (job 19506296)
source /share/apps/anaconda3/2025.06/etc/profile.d/conda.sh
ENV_NAME=foci
E=/scratch/wg2381/.conda/envs/$ENV_NAME
PK=/scratch/wg2381/.conda/pkgs_bl/$ENV_NAME/$SLURM_JOB_ID
mkdir -p "$PK/conda" "$PK/pip"
export CONDA_PKGS_DIRS=$PK/conda PIP_CACHE_DIR=$PK/pip PIP_NO_INPUT=1 XDG_CACHE_HOME=/scratch/wg2381/.cache
export PYTHONDONTWRITEBYTECODE=1
echo "=== $(date -Is) host=$(hostname) job=$SLURM_JOB_ID env=$E pkgs=$CONDA_PKGS_DIRS"
"$E/bin/pip" freeze > "$PK/freeze_before.txt"

echo "=== $(date +%T) conda install libegl libgl (freeze-installed)"
conda install -y -p "$E" --override-channels -c conda-forge --freeze-installed libegl libgl
ls -l "$E"/lib/libEGL.so* "$E"/lib/libGL.so*

echo "=== $(date +%T) re-point editable foci at foci_bl"
cd /scratch/wg2381/ext_repos/foci_bl
git diff --stat   # must show exactly the 3 MUMPS hunks
"$E/bin/pip" install --no-deps --no-build-isolation -e /scratch/wg2381/ext_repos/foci_bl

echo "=== $(date +%T) verify (CPU node: warp/cuda parts are checked by the GPU smoke)"
cd /tmp
export LD_LIBRARY_PATH=$E/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
"$E/bin/python" - <<'EOF'
import sys, numpy as np
import open3d as o3d, casadi as cas
print("python", sys.version.split()[0], "numpy", np.__version__, "open3d", o3d.__version__, "casadi", cas.__version__)
pcd = o3d.geometry.PointCloud(); pcd.points = o3d.utility.Vector3dVector(np.random.rand(100, 3))
vg = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.25)
print("o3d VoxelGrid voxels:", len(vg.get_voxels()))
x = cas.MX.sym("x", 2)
s = cas.nlpsol("s", "ipopt", {"x": x, "f": (x[0] - 1) ** 2 + (x[1] + 2) ** 2, "g": x[0] + x[1]},
               {"ipopt.linear_solver": "mumps", "ipopt.print_level": 0, "print_time": 0})
r = s(x0=[0, 0], lbg=-10, ubg=10)
print("ipopt+mumps:", s.stats()["return_status"], np.array(r["x"]).ravel())
try:
    s2 = cas.nlpsol("s2", "ipopt", {"x": x, "f": (x[0] - 1) ** 2}, {"ipopt.linear_solver": "ma27", "ipopt.print_level": 0, "print_time": 0})
    s2(x0=[0, 0]); print("ma27 status:", s2.stats()["return_status"])
except Exception as e:
    print("ma27 (expected to fail, unlicensed):", type(e).__name__, str(e)[:200])
import foci, foci.optim.initial_guess as ig
print("foci from:", foci.__file__)
import inspect; src = inspect.getsource(ig)
print("initial_guess uses mumps:", '"ipopt.linear_solver": "mumps"' in src)
EOF
"$E/bin/pip" freeze > "$E/bl_b0_freeze.txt"
diff "$PK/freeze_before.txt" "$E/bl_b0_freeze.txt" || true
echo "=== $(date -Is) done"
