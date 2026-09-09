#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=gmc_k2_stage4
#SBATCH --time=24:00:00
#SBATCH --mem=32G
#SBATCH --cpus-per-task=8
#SBATCH --output=/scratch/wg2381/splathjb/gmc/logs/%x-%j.out
set -x
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=8 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1

# Stage 4 only: the one round of k2_door_gate.py that has never completed.
# Stages 0-3 and 5 already pass on real K2 3DGS geometry (job 17215125); this
# script exists so the full compile -> query -> independent-verify pipeline can
# be retried on its own, without re-running the slice-level rounds.
#
# NO --partition LINE ON PURPOSE.  Verified with `sbatch --test-only`: the
# cpu_short QoS caps wall time at 06:00:00, and naming ANY partition explicitly
# for a 24 h job is rejected by the site submit plugin with "CPU job setup is
# not valid".  With no --partition line a 24 h job is accepted and routed to
# the `cs` partition.  Do not "fix" this by adding one back.
#
# WHY 24 h.  `build_slabs` bisects orientation (initial_intervals=16,
# theta_min=5e-3, max_depth=14 in configs/toy.yaml) into potentially hundreds
# of slices, and Stage 4 has never once run to completion on real data, so its
# true cost is unmeasured.  24 h is a deliberate over-budget for the first
# successful run, not an estimate; once one run finishes, the real wall time
# can replace it.
#
# half=1.5 only.  It goes first because the point is to unblock Stage 4 at all.
# The clean atlas-matched round needs half >= 2.09 u (at half=1.5 only 51% of
# each probe segment lies inside the workspace box) and is a separate later
# job; do not bolt a half=3.0 compile round onto this script.

echo "=== STAGE 4: half=1.5, full compile + query + independent verify ==="
$PY experiments/k2_door_gate.py --door door_A --half 1.5 --with-compile \
    -o results/k2/door_A_h1.5_full.json

echo "=== K2_STAGE4_DONE ==="
