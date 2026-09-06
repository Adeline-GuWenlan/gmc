#!/bin/bash
# P0-R2 layer-2 acceptance: CLEAN-ROOM reproduction of the review package
# (round-6).  Fresh dir, unzip the shipped package, run reproduce.sh from
# the snapshot inside the overlay env, then both gate scripts.  cn009
# direct (queue congested; user-directed).  bash -lc required.
set -u
PROJ=/scratch/sy2366/Project/splathjb
CR=$PROJ/cleanroom_p0r2
LOG=$PROJ/splatc_atlas/outputs/p0r2_cleanroom.log
ZIP=$PROJ/splatc_atlas/outputs/round4_final_package.zip
rm -rf "$CR"; mkdir -p "$CR"
# setsid: detach fully (plain nohup raced SIGHUP on double-ssh exit)
setsid nohup nice -n 19 singularity exec \
  --overlay "$PROJ/overlays/splathjb_env.ext3:ro" \
  /share/apps/admin/singularity-images/centos-8.2.2004.sif \
  bash -c "source /ext3/env.sh && \
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
    echo CLEANROOM_START \$(date -u +%FT%TZ) \$(hostname) && \
    cd $CR && python -m zipfile -e $ZIP . && \
    PKG=$CR/round4_final_package && \
    echo '== layer 1: validate_package.py' && \
    python \$PKG/validate_package.py \$PKG && \
    echo '== reproduce.sh (full chain from snapshot)' && \
    cd \$PKG/src_snapshot && bash reproduce.sh && \
    echo '== layer 2: compare_reproduction.py' && \
    cd \$PKG && python compare_reproduction.py \$PKG \$PKG/src_snapshot && \
    echo CLEANROOM_ALL_PASS" \
  > "$LOG" 2>&1 &
LPID=$!
sleep 5
if kill -0 "$LPID" 2>/dev/null || [ -s "$LOG" ]; then
  echo "launched pid $LPID on $(hostname) at $(date -u +%FT%TZ); log: $LOG"
  head -3 "$LOG"
else
  echo "LAUNCH FAILED (process gone, log empty)"
  exit 1
fi
