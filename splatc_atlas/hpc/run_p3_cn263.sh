#!/bin/bash
# P3 matched-budget baselines + hybrid-floor uncensored rerun on cn263
# (direct node, queue congested; user-directed).  bash -lc required.
set -u
PROJ=/scratch/sy2366/Project/splathjb
cd "$PROJ/splatc_atlas"
mkdir -p outputs
nohup singularity exec \
  --overlay "$PROJ/overlays/splathjb_env.ext3:ro" \
  /share/apps/admin/singularity-images/centos-8.2.2004.sif \
  bash -c "source /ext3/env.sh && cd $PROJ/splatc_atlas && \
    PYTHONPATH=src python experiments/compiler/p3_matched_budget.py && \
    PYTHONPATH=src python experiments/compiler/floor_probe.py && \
    echo P3_ALL_DONE" \
  > outputs/p3_run.log 2>&1 &
echo "launched pid $! on $(hostname) at $(date -u +%FT%TZ)"
