# Baseline environments (bl stage B0, 2026-10-09)

What B1/B2 need to run the four baselines on this cluster: env paths, versions, the exact install commands, the
smoke test each repo's own example passed, the deviations, and what to import. Plan: `docs/baselines_f4_plan.md`
§4/§6. Claims are **[E]** (evidence file named) or **[G]** (untested).

## 0. Summary

| method | env (`/scratch/wg2381/.conda/envs/…`) | code | smoke (repo's own example) | status |
|---|---|---|---|---|
| cust_fields | `cust_fields` (py 3.10.22) | `ext_repos/cust_fields` (pristine, 5ca178e) | `test_nf.py` | **RUNS** — goal reached [E] |
| PNO | `pno` | `ext_repos/PNO` (pristine, 6384751) + weights `gmc/outputs/baselines/pno/` | notebook §3 eval, City 256 | **RUNS** — reproduces paper Table 2 within 2 % [E] |
| SplatNav | `splatnav` | `ext_repos/splatnav` (pristine, e996e22) | Splat-Plan on a synthetic Gaussian scene + CUDA check | **RUNS** on L40S — 3/3 feasible, clearance ≥ r [E] |
| FOCI | `foci` (py 3.10.21) | `ext_repos/foci_bl` (copy of 79d8ddc + `gmc/baselines/patches/foci_mumps.patch`) | `demos/stonehenge.py` headless | **RUNS** on L40S — 6/6 IPOPT acceptable; soft goal, ends 0.2–4.0 m short [E] |

How they were built (fixing the 2026-09-24 failure): **one install job at a time**, each submitted only after the
previous one ended, each with its own fresh `CONDA_PKGS_DIRS=/scratch/wg2381/.conda/pkgs_bl/<env>/<jobid>/conda` and
pip cache next to it; channel conda-forge only (`--override-channels`; `~/.condarc` is conda-forge strict). Scripts
(committed, exactly what ran): `gmc/hpc/baselines/b0_env_*.sbatch`. Each env holds a `bl_b0_freeze.txt` (pip freeze
after B0). Job IDs: `/scratch/wg2381/claude_jobs/baselines/jobids/B0.txt`.

Two cluster facts every B1/B2 job must respect:
- **No libEGL/libGL on the nodes.** pip `open3d` (and `open3d-cpu`) link `libEGL.so.1`/`libGL.so.1`. Fix used in
  `foci` and `splatnav`: conda-forge `libegl` + `libgl` (libglvnd 1.7.0) inside the env. open3d's RUNPATH contains
  `$ORIGIN/../../../` = `$E/lib`, so they are found **without** `LD_LIBRARY_PATH` **[E]** (checked: `env -u
  LD_LIBRARY_PATH $E/bin/python -c "import open3d"` → ok; `ldd` resolves both to `$E/lib`). Only headless open3d
  use works (geometry, VoxelGrid, I/O); no windows are opened anywhere.
- **$HOME is at 88 % of its inode quota.** Set `XDG_CACHE_HOME=/scratch/wg2381/.cache`,
  `MPLCONFIGDIR=/scratch/wg2381/.cache/mpl`, `WARP_CACHE_PATH=/scratch/wg2381/.cache/warp` (warp kernel cache) and
  `PYTHONDONTWRITEBYTECODE=1` (keeps the pristine clones free of `__pycache__`).

Call each env's interpreter directly (`$E/bin/python`); no `conda activate` needed.

No method is SETUP_FAIL. Budget: **2.13 CPU-h of 10, 0.054 GPU-h of 2** (11 compute jobs, `sacct`), plus the
agent's own 1-CPU allocation.

Smoke evidence lives in `gmc/results/baselines/b0_smoke/<method>/` (`summary.json`, `log.txt`, figure). The runners
are `gmc/experiments/bl_smoke_<method>.py` and the jobs `gmc/hpc/baselines/b0_smoke_{cust_fields,pno,gpu}.sbatch`
(`b0_smoke_gpu.sbatch` takes `PARTS="splatnav foci pno"`). Every runner leaves the repo code unchanged and writes
no bytecode into the clones.

## 1. cust_fields

- **Env.** `/scratch/wg2381/.conda/envs/cust_fields`: Python 3.10.22, numpy 2.2.6, matplotlib 3.10.9, PyYAML 6.0.3.
  CPU only.
- **Install** (job 19506169, 3 min, `b0_env_cust_fields.sbatch`), as the README says:
  `conda create -p $E --override-channels -c conda-forge python=3.10 pip`, then
  `$E/bin/pip install numpy matplotlib pyyaml`.
- **Smoke** (job 19506295). `bl_smoke_cust_fields.py` executes the pristine `test_nf.py` unchanged; `plt.show`
  saves the figure instead. Result: on `CONFIG/world_demo.yaml`, "goal reached at step 179", path 8.150 m,
  180 points, final distance 0.0498 < tolerance 0.05, 3.5 s **[E]** (`b0_smoke/cust_fields/summary.json`).
  Figure `test_nf.png`, which I viewed: a squircle workspace, three squircle obstacles (two merged in one tree),
  the −∇φ field, and a path from start around the upper cluster to the goal.
- **Import for B2.** `sys.path.insert(0, "/scratch/wg2381/ext_repos/cust_fields")`, then
  `from NF.geometry import World`, `from NF.navigation import NavigationFunction`, then
  `NavigationFunction(World(yaml_path), np.array([gx, gy, 0.0]), nf_lambda, nf_mu)`. The demo's defaults are
  `NF_LAMBDA = 1e3` and `NF_MU = [1e10, 1e8, 1e6, 1e4, 1e2, 1e1]`. The path extraction is the demo's own loop:
  normalised −∇φ steps with `safe_advance` backtracking, random-direction escape, `DT = 0.05`,
  `MAX_STEPS = 3000`, `GOAL_TOL = 0.05`. Load `test_nf.py` as a module, as the smoke runner does, to reuse
  `neg_gradient` and `safe_advance` verbatim.
- **Constraint for B2's map adapter.** The world YAML only knows **rotated squircles**: `type: Rectangular` with
  `center`, `width`, `height`, `theta` and squareness `s` (0.85 in the demo), grouped as `Star` (one) or `StarTree`
  (overlapping squircles, a tree), inside a squircle `Workspace` (`s: 0.9999`). Plan §4 says "star-shaped
  approximations (convex hull or star decomposition)". In this code that means covering each C-space component
  with a tree of rotated squircles. Arbitrary polygons are not supported.

## 2. PNO

- **Env.** `/scratch/wg2381/.conda/envs/pno`: Python 3.10.22, torch 2.5.1+cu121 (CUDA 12.1 build; CPU works too),
  torchvision 0.20.1, numpy 1.26.4, scipy 1.15.3, scikit-learn 1.7.2, matplotlib 3.10.9, prettytable 3.18.0,
  pandas 2.3.3, h5py 3.16.0, shapely 2.1.2, pqdict 1.5.1.
- **Install** (job 19506371, 9 min, `b0_env_pno.sbatch`):
  `conda create -p $E --override-channels -c conda-forge python=3.10 pip`, then
  `$E/bin/pip install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu121`, then
  `$E/bin/pip install "numpy<2" scipy matplotlib scikit-learn prettytable tqdm pandas h5py shapely pqdict`.
- **Deviation: environment.yml not used.** It has 275 exact pins (py3.8, torch 2.1.0/cu121, pyrocko/qt channels),
  is meant for the 3D/4D experiments, and does not solve on conda-forge-strict. The README pins the 2D example,
  the one we use, to "Python 3.10, PyTorch 2.5.1"; that is what this env is. Not installed: pykonal and
  scikit-fmm (training-data generators only) and jupyter.
- **Weights** (never trained). `gmc/outputs/baselines/pno/models/{FNO,DAFNO,FNOSDF,PNO,PNOwPINN}/best_model.pt`
  come from HF `lukebhan/generalizableMotionPlanningViaOperatorLearning`, revision `36a76173…` (`HF_REVISION.txt`).
  Each has a `.sha256` sidecar, and all five equal HF's LFS oids **[E]**:
  - PNO `11756e38…`
  - PNOwPINN `30b6bff8…`
  - FNOSDF `977bd774…`
  - FNO `872bd3ac…`
  - DAFNO `25790030…`

  Inference data (only the City 256×256 test set: `mask.npy`, `goal.npy`, `output.npy`, no `dist_in`, ~71 MB)
  comes from HF dataset `lukebhan/generalizableMotionPlanning`, revision `c42f8958…`, in
  `gmc/outputs/baselines/pno/dataset/cityData/256x256/`. Sidecars equal the LFS oids. Everything is uncommitted.
- **Smoke.** CPU job 19506958, plus cuda in 19506846. `bl_smoke_pno.py` replicates notebook §3 `test_model`: the
  FNOSDF approximates the SDF, `smooth_chi(mask, sdf, 5)`, value function ×mask, `LpLoss(d=2, p=2)`, all 90
  City-256 maps, batch size 2. Relative L2 **[E]** (`b0_smoke/pno_cpu/summary.json`, `pno_cuda/summary.json`):

  | model | CPU | cuda | paper Table 2 [G: read from arXiv 2410.17547 HTML via a fetch tool] |
  |---|---|---|---|
  | PNO | 0.1733 | 0.1736 | 0.1748 |
  | PNO w PINN | 0.1636 | 0.1639 | 0.1675 |

  Latency per 256² map: on CPU (4 threads) 0.36 s SDF + 1.05 s (PINN) / 1.21 s (PNO) value function; on the L40S
  1.3 ms + 3.6 ms (PINN). The model loads in 0.16 s. Figure `pno_cpu/pno_city256_map0.png`, which I viewed: the
  predicted V has its minimum at the goal (confirms that the goal is indexed (col, row) = (x, y)) and grows like
  FMM. |error| reaches ≥ 5 cells in parts of the free space.
- **Workaround (CPU only, ours, numerically neutral).** `DEEPNORM2dMultiGoal.forward` passes `chi.expand(...)` (a
  stride-0 view) to `torch.fft.rfft2`. oneMKL rejects that layout with "DFTI ERROR: Inconsistent configuration
  parameters" (job 19506845); cuFFT, which the notebook used, accepts it. The runner wraps
  `torch.fft.rfft2(x.contiguous())` when the device is CPU. B2 must do the same, or run on GPU.
- **Import for B2.**
  - `sys.path.insert(0, ".../PNO/examples")`, then `from models.deepnormMultiGoal import DEEPNORM2dMultiGoal`,
    `from models.fno import FNO2d`.
  - Constructors exactly `DEEPNORM2dMultiGoal(4, 8, 8, 16)` and `FNO2d(4, 1, 8, 8, 16)`;
    `torch.load(..., weights_only=True)`.
  - `torch.set_default_device(dev)`: model internals create tensors on the default device.
  - Inputs: `mask` float (N, S, S, 1) with 1 = free; `goal` int (N, 2) as (col, row). Output: (N, S, S, 1), × mask.
  - Square grids. The models were trained on 64×64 and evaluated zero-shot up to 1024² ("super-resolution"), so
    the grid size is free.
- **Path extraction is B2's choice; the 2D notebook has none.** It only evaluates value functions. The paper's 2D
  planning use is A* with PNO as the heuristic (`2D_Neural_Heuristics/heuristics.py`, `astar/`; their imports are
  covered by this env). My illustrative greedy 8-neighbour descent on V̂ stuck after 32 cells at an obstacle edge
  (local minimum) **[E]** (figure), so naive descent on V̂ is not enough.

## 3. SplatNav (Splat-Plan)

- **Env.** `/scratch/wg2381/.conda/envs/splatnav`: Python 3.10.22, torch 2.1.2+cu118 (CUDA 11.8 build; L40S is
  sm_89, `cuda_available` True, 2048² matmul finite), torchvision 0.16.2, numpy 1.26.4, scipy 1.15.3,
  open3d 0.20.0 (+ conda-forge libegl/libgl), clarabel 0.11.1, dijkstra3d 1.15.2, polytope 0.2.5, cvxpy 1.7.5,
  cvxopt 1.3.3, unfoldNd 0.2.3, sympy 1.14.0, matplotlib 3.10.9.
- **Install** (job 19506844, 12 min, `b0_env_splatnav.sbatch`):
  - `conda create -p $E --override-channels -c conda-forge python=3.10 pip libegl libgl`
  - `$E/bin/pip install torch==2.1.2+cu118 torchvision==0.16.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118`
    (nerfstudio's documented torch line; the README builds on nerfstudio)
  - `$E/bin/pip install "numpy<2" scipy matplotlib sympy tqdm clarabel dijkstra3d polytope cvxopt cvxpy unfoldNd open3d`
- **Deviation: minimal set, no nerfstudio.** Not installed: nerfstudio, gsplat, LightGlue, viser, ompl, Blender.
  Splat-Plan's modules (`splatplan/`, `polytopes/`, `initialization/`, `ellipsoids/`, `SFC/`) import without them
  **[E]** (job log). Only `splat/splat_utils.py` (GSplatLoader, which loads a nerfstudio checkpoint) needs them; it
  already fails on `cv2`. B1 feeds Gaussians directly, as plan §4 asks.
- **Smoke** (job 19507292, L40S). The repo's trained scenes are on Google Drive and were not fetched.
  `bl_smoke_splatnav.py` makes the same calls as `run_splatplan.py`:
  `SplatPlan(gsplat, {radius, vmax, amax}, {lower_bound, upper_bound, resolution}, SplinePlanner(spline_deg=6), dev)`,
  then `generate_path(x0, xf)`. The scene is a 56-Gaussian synthetic one: a wall at x = 0 with one gap, plus
  rotated pillars; r = 0.05, vmax = amax = 0.1, a 100³ voxel grid. The `gsplat` object is a stand-in with the
  fields of the repo's `DummyGSplatLoader` (means, rots wxyz, scales, covs, covs_inv, colors, opacities).
  Results **[E]** (`b0_smoke/splatnav/summary.json`):
  - setup 1.86 s;
  - 3/3 queries feasible, 0.11–0.26 s each, 31–47 polytopes;
  - smallest distance from the Bézier trajectory to any ellipsoid surface 0.0517–0.0526 m ≥ r, with no sample
    inside an ellipsoid. That check is independent of the planner: a dense surface sample, plus an inside test;
  - negative control: the straight start→goal line of query 0 does hit an ellipsoid;
  - figure `splatplan_synthetic.png`, which I viewed: all three paths go through the gap.
- **HAZARD for B1 (found here).** `ellipsoids.covariance_utils.quaternion_to_rotation_matrix` returns **NaN** for
  any quaternion with an exactly zero vector part, i.e. q = ±[1, 0, 0, 0], in float32 and float64. A vector part of
  1e-12 is already fine **[E]** (`b0_smoke/splatnav/quat_nan_probe.txt`, from `gmc/experiments/bl_probe_splatnav_quat.py`). Such Gaussians get
  NaN covariances and silently vanish from the voxel grid and the collision set. My first smoke (19506846) used
  identity quaternions for the wall, and all paths went straight through it; that run is kept, marked INVALID, in
  `b0_smoke/splatnav_invalid_identity_quat/`. B1 must assert `torch.isfinite` on `quaternion_to_rotation_matrix(rots)`
  and on `covs`. For axis-aligned Gaussians (e.g. a floor support), write an equivalent rotation: a 90° yaw with
  the x/y scales swapped, as the smoke does.
- **Other facts B1 needs** (read in code; **[E]** where the smoke shows it):
  - `generate_path` calls `torch.cuda.synchronize()` unconditionally, so it is **GPU only**.
  - On an infeasible QP it writes `infeasible.obj` to the cwd and re-raises (bare `raise`). Run with cwd in a
    scratch dir and map the exception to FAIL.
  - The trajectory starts and ends at the **A\* voxel centres**, not at x0/xf (0.0145 m off here) **[E]**, so plan
    §3.3's appended segments will be needed. A start/goal inside an occupied voxel is moved to the nearest free one
    (printed by the repo).
  - Obstacle = the ellipsoid with semi-axes `scales` (a 1σ level). The judge uses 2σ (plan §3.1), so the matched
    contract is `scales × 2` [G, to be confirmed by B1 against the judge's code].
  - **Opacity is not used** by Splat-Plan: filter by the judge's τ before handing Gaussians over.
  - Robot = a sphere of `radius`. The collision set uses `rs = vmax²/(2·amax) + radius`.
  - It plans in 3D inside `[lower_bound, upper_bound]`; a thin z-slab around z_c is one way to keep it in the plane [G].
- **Import for B1.** `sys.path.insert(0, "/scratch/wg2381/ext_repos/splatnav")` (pristine), then
  `from splatplan.splatplan import SplatPlan`, `from splatplan.spline_utils import SplinePlanner`,
  `from ellipsoids.covariance_utils import compute_cov, quaternion_to_rotation_matrix`. Never `splat.splat_utils`.

## 4. FOCI

- **Env.** `/scratch/wg2381/.conda/envs/foci`, built 2026-09-24 (`ext_repos/env_setup/env_foci.sbatch`, job
  18438458: `conda create python=3.10 pip git-lfs`, then `pip install "numpy<2" open3d matplotlib viser plyfile
  scipy astar casadi warp-lang usd-core scikit-learn pandas`, then `pip install -e ext_repos/foci`, then
  `git lfs pull`). Versions: Python 3.10.21, numpy 1.26.4, casadi 3.8.1 (bundled IPOPT + MUMPS), warp-lang 1.17.0,
  open3d 0.20.0, astar 0.99, plyfile 1.1.3, scipy 1.15.3, viser 1.1.1.
- **B0 repair** (job 19506351, `b0_env_foci_fix.sbatch`; 19506296 had died after the conda step on a `set -u`
  issue in the base env's activate.d, which was then fixed):
  1. `conda install -p $E --override-channels -c conda-forge --freeze-installed libegl libgl`. This added libglvnd
     1.7.0, libglx, libxcb and xorg-libx11/xau/xdmcp, and bumped openssl 3.6.4 → 3.6.5. `import open3d` now
     works, as does `o3d.geometry.VoxelGrid` (FOCI's A* initial guess needs open3d for real) **[E]**.
  2. `pip install --no-deps --no-build-isolation -e /scratch/wg2381/ext_repos/foci_bl`, so `import foci` →
     `foci_bl` **[E]**.
- **Deviation: MA27 → MUMPS** (licence). `gmc/baselines/patches/foci_mumps.patch` sets
  `"ipopt.linear_solver": "ma27"` → `"mumps"` in `foci/optim/solvers.py` (main trajectory NLP),
  `foci/optim/initial_guess.py` (spline fit of the A* path) and `foci/convolution/gaussian_robot_warp.py` (its
  `__main__` self-test only). It is applied to the copy `ext_repos/foci_bl` (`cp -a` of the clone at 79d8ddc,
  then `git apply`; `git -C foci_bl diff --stat` = those 3 lines); the pristine `ext_repos/foci` still says ma27.
  In the env, IPOPT+MUMPS gives `Solve_Succeeded` and MA27 gives `Invalid_Option` **[E]**. **This deviates from
  the paper**, which uses HSL MA27. Iterates may differ from the authors' runs; I cannot test MA27 here [G].
- **Smoke** (job 19506397, L40S). `bl_smoke_foci.py` is `demos/stonehenge.py` up to and including its six
  `planner.plan` calls: same ply (138,829 Gaussians), ×20 scaling, `robot_cov = 0.01 I`, 10 control points, 40
  samples, 12 ring points at z = 0.5. The demo's viser server is replaced by a matplotlib top view. Results **[E]**
  (`b0_smoke/foci/summary.json`):
  - 6/6 `Solved_To_Acceptable_Level`, 48–151 IPOPT iterations, 0.39–2.08 s per plan, setup 1.45 s;
  - warp on `cuda:0`;
  - figure `foci_stonehenge.png`, which I viewed: the optimised curves leave their A* inits and **end
    0.2–4.0 m short of the goal** (plans 0, 4 and 5 by > 2 m).

  This is the method's formulation, not the environment:
  - the goal is a soft cost (`w_2 = 40`, equal to the obstacle weight `w_1`);
  - the start is a constraint `‖·‖² ≤ 0.05` (≤ 0.224 m; the observed start errors are 0.07–0.22 m);
  - IPOPT runs with `tol = acceptable_tol = 0.1` and `acceptable_iter = 1`.

  Whether MA27 would stop elsewhere is untested [G]. B1 has to measure goal error and apply plan §3.3 (append the
  goal segment, judged).
- **Facts for B1 (from code):**
  - **GPU only**: `foci/convolution/gaussian_robot_warp.py` runs `wp.set_device("cuda:0")` at import.
  - `Planner(means, covs, robot_cov, num_control_points, num_samples)`: the robot is three body points (centre and
    ±0.2 m along the heading, `Planner.kinematics`), each with the Gaussian `robot_cov`. B1 fits the cylinder cover
    through `robot_cov`, or through the kinematics, which needs another patch.
  - Hard-coded: `z_range=(0, 1)` (solver bounds and A* bounds) and A* `voxel_size=0.25`. B1 must shift z so z_c
    lies in (0, 1) (or patch).
  - The A* occupancy is a VoxelGrid of Gaussian **means** only.
  - **Opacity is not used**.
  - `plan` raises `ValueError("No path found")` when its A* fails.
  - Quirk: `plan` sets `self.ubg[-1] = astar_length`, which overwrites the bound of the last constraint, an
    acceleration-hull term whose bound is `amax² = 1` (looks unintended upstream; keep it, it is the method).
  - IPOPT `print_level 5` is verbose (64 kB of log per 6 plans); casadi has no OpenMP, so the kinematics map runs
    serially.
- **Import for B1.** `$E/bin/python`, then `from foci.planners.planner import Planner` and
  `from foci.utils.ply import extract_splat_data`. Assert `foci.__file__` is under `ext_repos/foci_bl` (the smoke
  does).

## 5. Open items for B1/B2
- SplatNav: quaternion NaN guard; 1σ vs the judge's 2σ contract; opacity filter; sphere cover of the cylinder
  (radius √(0.30² + 0.865²) = 0.916 m if the full height is enclosed: very conservative, so state it); z-slab.
- FOCI: soft goal and early IPOPT stop mean many appended goal segments are expected; z_range (0, 1); Gaussian
  robot cover; MUMPS disclosed.
- PNO: path extraction (A* with the PNO heuristic is the paper's use); CPU rfft2 workaround; map → S×S square
  grid with 1 = free; goal (col, row).
- cust_fields: C-space components must become trees of rotated squircles; record how often that closes a passage.
