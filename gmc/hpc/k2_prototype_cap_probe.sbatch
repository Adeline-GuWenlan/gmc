#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --job-name=gmc_k2_cap
#SBATCH --time=00:25:00
#SBATCH --mem=8G
#SBATCH --cpus-per-task=8
#SBATCH --output=/scratch/wg2381/splathjb/gmc/logs/%x-%j.out
set -x
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=8 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
$PY - <<'PY'
# How many iterations does the prototype world-bound fixed point ACTUALLY need
# on real 3DGS geometry?  The shipped cap is 16 and the run hits it.  Raise the
# cap only inside this probe (the compiler source is untouched) and record the
# true maximum, so a cap change can be justified by a measured margin instead
# of by a guess.
import numpy as np, sys, collections, platform
sys.path.insert(0, 'experiments')
import gmc.geometry.envelopes as env
from gmc.config import load_config
from gmc.io.robot_io import ellipse_robot
from gmc.spatial.bvh import candidate_pairs
from k2_scene import load_window
print("node:", platform.node(), "numpy", np.__version__)

CAP = 512
counts = collections.Counter()
worst = []
_outer = env._outer_vertices

def probe(oracle, theta, directions, h):
    slack = 0.0
    for it in range(1, CAP + 1):
        local = _outer(directions, np.asarray(h, dtype=np.longdouble) + slack)
        world, needed = oracle.translate_local_points(theta, local)
        if needed <= slack:
            counts[it] += 1
            if it > 16:
                s = oracle.scene
                eig = np.linalg.eigvalsh(s.covariance)
                worst.append((it, np.round(np.asarray(s.mean), 4).tolist(),
                              float(s.level * np.sqrt(max(eig[0], 0.0))),
                              float(s.level * np.sqrt(max(eig[1], 0.0)))))
            return local, world, slack, needed
        slack = max(slack, needed)
    counts["FAIL"] += 1
    raise FloatingPointError("did not converge even at cap %d" % CAP)

env._prototype_outward_world_vertices = probe

cfg = load_config('configs/toy.yaml')
for half in (1.5, 3.0):
    scene, st = load_window((22.3, -15.2), half)
    robot = ellipse_robot(1.2, 0.3)
    oracles = list(candidate_pairs(scene, robot, scene.workspace))
    for theta in (2.1293, 0.6021, 1.2086, 1.7890, 0.0, 0.7854):
        counts.clear(); worst.clear()
        for o in oracles:
            env.approximate_pair(o, theta, cfg.pair_approx)
        it_keys = [k for k in counts if k != "FAIL"]
        tot = sum(counts[k] for k in it_keys)
        over = sum(counts[k] for k in it_keys if k > 16)
        print(f"half={half} theta={theta}: n_oracles={len(oracles)} calls={tot} "
              f"max_iters={max(it_keys)} over_16={over} FAIL={counts['FAIL']}",
              flush=True)
        for w in sorted(worst, reverse=True)[:3]:
            print(f"      needed {w[0]:3d} iters  mean={w[1]} "
                  f"semi=({w[2]:.3e}, {w[3]:.5f})", flush=True)
PY
echo "=== K2_CAP_DONE ==="
