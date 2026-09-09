#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --job-name=gmc_k2_inward_cap
#SBATCH --time=01:00:00
#SBATCH --mem=8G
#SBATCH --cpus-per-task=8
#SBATCH --output=/scratch/wg2381/splathjb/gmc/logs/%x-%j.out
set -x
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=8 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
$PY - <<'PY'
# Companion to k2_prototype_cap_probe.sbatch, for the OTHER capped fixed point.
# `_inward_world_points` shrinks the certified inner points by `alpha` until the
# world-export error fits inside the pair inradius.  Its cap has the same
# shipped value of 16 as the prototype outward loop, it runs in BOTH prototype
# and theorem mode, and unlike the outward loop it has never been measured on
# real 3DGS geometry.  Raise the cap only inside this probe and record the true
# iteration distribution.
#
# The probe deliberately monkeypatches rather than reading the compiler's
# `_WORLD_EXPORT_MAX_ITERS`, so the measurement stays independent of any cap
# change made in the source.
#
# The body below must stay a FAITHFUL replica of `_inward_world_points`
# (envelopes.py), including the `_binary64_error_scale` multiply-error term,
# the `_inradius_lower_bound` call with its non-positive-inradius raise, and
# the `needed >= 1.0` early return that yields an EMPTY inner set.  Dropping
# the early-return branch would turn calls that legitimately give up into
# calls that keep iterating, and the histogram would be wrong.
import numpy as np, sys, collections, platform
sys.path.insert(0, 'experiments')
import gmc.geometry.envelopes as env
from gmc.config import load_config
from gmc.io.robot_io import ellipse_robot
from gmc.spatial.bvh import candidate_pairs
from k2_scene import load_window
print("node:", platform.node(), "numpy", np.__version__)

CAP = 512
counts = collections.Counter()      # converged calls, keyed by iteration count
empty_at = collections.Counter()    # empty-inner early returns, keyed by iter
other = collections.Counter()       # INRADIUS_RAISE / FAIL
worst = []

def probe(oracle, theta, points, local_point_error):
    inradius = env._inradius_lower_bound(oracle)
    if not np.isfinite(inradius) or inradius <= 0.0:
        other["INRADIUS_RAISE"] += 1
        raise FloatingPointError("pair inradius bound is not positive")
    alpha = 0.0
    for it in range(1, CAP + 1):
        local = (1.0 - alpha) * points
        world, export_error = oracle.translate_local_points(theta, local)
        multiply_error = float(env._binary64_error_scale(
            points, local, alpha,
        ))
        error = ((1.0 - alpha) * local_point_error
                 + multiply_error + export_error)
        needed = float(np.nextafter(error / inradius, np.inf))
        if needed <= alpha:
            counts[it] += 1
            if it > 16:
                s = oracle.scene
                eig = np.linalg.eigvalsh(s.covariance)
                worst.append((it, np.round(np.asarray(s.mean), 4).tolist(),
                              float(s.level * np.sqrt(max(eig[0], 0.0))),
                              float(s.level * np.sqrt(max(eig[1], 0.0)))))
            return local, world, alpha, error
        if not np.isfinite(needed) or needed >= 1.0:
            # Same conservative empty-inner exit as the real function.
            empty_at[it] += 1
            return np.empty((0, 2), dtype=float), \
                np.empty((0, 2), dtype=float), 1.0, error
        alpha = max(alpha, needed)
    other["FAIL"] += 1
    raise FloatingPointError("did not converge even at cap %d" % CAP)

env._inward_world_points = probe

cfg = load_config('configs/toy.yaml')
print("certificate_mode:", cfg.pair_approx.certificate_mode, flush=True)
for half in (1.5, 3.0):
    scene, st = load_window((22.3, -15.2), half)
    robot = ellipse_robot(1.2, 0.3)
    oracles = list(candidate_pairs(scene, robot, scene.workspace))
    for theta in (2.1293, 0.6021, 1.2086, 1.7890, 0.0, 0.7854):
        counts.clear(); empty_at.clear(); other.clear(); worst.clear()
        for o in oracles:
            env.approximate_pair(o, theta, cfg.pair_approx)
        it_keys = sorted(counts)
        conv = sum(counts[k] for k in it_keys)
        over = sum(counts[k] for k in it_keys if k > 16)
        empty = sum(empty_at.values())
        mx = max(it_keys) if it_keys else 0
        print(f"half={half} theta={theta}: n_oracles={len(oracles)} "
              f"converged={conv} max_iters={mx} over_16={over} "
              f"empty_inner={empty} INRADIUS_RAISE={other['INRADIUS_RAISE']} "
              f"FAIL={other['FAIL']}", flush=True)
        print("      hist:", " ".join(f"{k}x{counts[k]}" for k in it_keys),
              flush=True)
        if empty:
            print("      empty_inner at iter:",
                  " ".join(f"{k}x{empty_at[k]}" for k in sorted(empty_at)),
                  flush=True)
        for w in sorted(worst, reverse=True)[:3]:
            print(f"      needed {w[0]:3d} iters  mean={w[1]} "
                  f"semi=({w[2]:.3e}, {w[3]:.5f})", flush=True)
PY
echo "=== K2_INWARD_CAP_DONE ==="
