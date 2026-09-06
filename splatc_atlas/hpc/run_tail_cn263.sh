#!/bin/bash
# Tail of the frozen-tree canonical chain (p36 + scalar + witness + floor),
# moved to cn263 (cn009 load spiked to ~14; first four artifacts already
# written from the same frozen tree on shared /scratch).
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
    python experiments/compiler/p36_fairness.py && \
    python experiments/compiler/ridge_continuation_scalar.py && \
    python experiments/compiler/witness_upgrade.py && \
    python experiments/compiler/floor_probe.py && \
    echo TAIL_CHAIN_DONE" \
  > outputs/tail_chain.log 2>&1 &
LPID=$!
sleep 5
if kill -0 "$LPID" 2>/dev/null || [ -s outputs/tail_chain.log ]; then
  echo "launched pid $LPID on $(hostname) at $(date -u +%FT%TZ)"
else
  echo "LAUNCH FAILED"; exit 1
fi
