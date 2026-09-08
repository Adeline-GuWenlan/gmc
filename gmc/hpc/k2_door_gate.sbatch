#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=gmc_k2_doorA
#SBATCH --time=04:00:00
#SBATCH --mem=32G
#SBATCH --cpus-per-task=8
#SBATCH --output=/scratch/wg2381/splathjb/gmc/logs/%x-%j.out
set -x
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=8 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1

# Rounds go small -> atlas-matched -> full pipeline, and each writes its own
# JSON, so a later round timing out cannot destroy an earlier round's result.
# The cost knob between rounds 1 and 2 is the WINDOW SIZE, never a splat
# subsample: dropping splats removes obstacle mass and can only push the
# verdict towards REACHABLE, which would make the rounds incomparable.

echo "=== ROUND 1: half=1.5, slice-level only ==="
$PY experiments/k2_door_gate.py --door door_A --half 1.5 \
    --figure results/k2/figures/door_A_h1.5.png \
    -o results/k2/door_A_h1.5.json

echo "=== ROUND 2: half=3.0, atlas-matched cross-check ==="
$PY experiments/k2_door_gate.py --door door_A --half 3.0 \
    --figure results/k2/figures/door_A_h3.0.png \
    -o results/k2/door_A_h3.0.json

echo "=== ROUND 3: half=1.5, full compile + query + independent verify ==="
$PY experiments/k2_door_gate.py --door door_A --half 1.5 --with-compile \
    -o results/k2/door_A_h1.5_full.json

echo "=== K2_DOOR_GATE_DONE ==="
