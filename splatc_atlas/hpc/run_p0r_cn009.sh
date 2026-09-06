#!/bin/bash
# P0-R canonical reruns + P3.6 fairness on cn009 (direct node, queue
# congested; user-directed 2026-08-14).  bash -lc required.
set -u
PROJ=/scratch/sy2366/Project/splathjb
cd "$PROJ/splatc_atlas"
mkdir -p outputs
# setsid: fully detach from the ssh session (plain nohup raced SIGHUP on
# the double-ssh exit — cn009 launch died silently with a 0-byte log)
setsid nohup nice -n 19 singularity exec \
  --overlay "$PROJ/overlays/splathjb_env.ext3:ro" \
  /share/apps/admin/singularity-images/centos-8.2.2004.sif \
  bash -c "source /ext3/env.sh && cd $PROJ/splatc_atlas && \
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
    PYTHONPATH=src python experiments/compiler/ridge_continuation.py && \
    PYTHONPATH=src python experiments/compiler/p2_generalization.py && \
    PYTHONPATH=src python experiments/compiler/p36_fairness.py && \
    echo P0R_P36_ALL_DONE" \
  > outputs/p0r_run.log 2>&1 &
LPID=$!
sleep 5
if kill -0 "$LPID" 2>/dev/null || [ -s outputs/p0r_run.log ]; then
  echo "launched pid $LPID on $(hostname) at $(date -u +%FT%TZ); log:"
  head -2 outputs/p0r_run.log
else
  echo "LAUNCH FAILED (process gone, log empty)"
  exit 1
fi
