#!/bin/bash
# FINAL single-tree canonical chain (round-5 P0-R: every artifact from one
# frozen source tree).  cn009 direct (user-directed); setsid + single-thread
# BLAS (shared 8-core allocation).
set -u
PROJ=/scratch/sy2366/Project/splathjb
cd "$PROJ/splatc_atlas"
mkdir -p outputs
setsid nohup nice -n 19 singularity exec \
  --overlay "$PROJ/overlays/splathjb_env.ext3:ro" \
  /share/apps/admin/singularity-images/centos-8.2.2004.sif \
  bash -c "source /ext3/env.sh && cd $PROJ/splatc_atlas && \
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
    export PYTHONPATH=src && \
    python experiments/compiler/ridge_continuation.py && \
    python experiments/compiler/p2_generalization.py && \
    python experiments/compiler/gamma_delta.py && \
    python experiments/compiler/p3_matched_budget.py && \
    python experiments/compiler/p36_fairness.py && \
    python experiments/compiler/ridge_continuation_scalar.py && \
    python experiments/compiler/witness_upgrade.py && \
    python experiments/compiler/floor_probe.py && \
    echo FULL_CHAIN_DONE" \
  > outputs/full_chain.log 2>&1 &
LPID=$!
sleep 5
if kill -0 "$LPID" 2>/dev/null || [ -s outputs/full_chain.log ]; then
  echo "launched pid $LPID on $(hostname) at $(date -u +%FT%TZ)"
else
  echo "LAUNCH FAILED"; exit 1
fi
