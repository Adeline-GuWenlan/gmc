#!/bin/bash
# P0.5 + P3.5 pack on cn263 (direct node, queue congested; user-directed):
# matched-budget with positive controls + gate-band stats + accounting,
# scalar-clearance same-architecture ablation, gamma rerun (provenance arm
# fix), oracle-tube floor rerun (provenance added).  bash -lc required.
set -u
PROJ=/scratch/sy2366/Project/splathjb
cd "$PROJ/splatc_atlas"
mkdir -p outputs
nohup singularity exec \
  --overlay "$PROJ/overlays/splathjb_env.ext3:ro" \
  /share/apps/admin/singularity-images/centos-8.2.2004.sif \
  bash -c "source /ext3/env.sh && cd $PROJ/splatc_atlas && \
    PYTHONPATH=src python experiments/compiler/p3_matched_budget.py && \
    PYTHONPATH=src python experiments/compiler/ridge_continuation_scalar.py && \
    PYTHONPATH=src python experiments/compiler/gamma_delta.py && \
    PYTHONPATH=src python experiments/compiler/floor_probe.py && \
    echo P35_ALL_DONE" \
  > outputs/p35_run.log 2>&1 &
echo "launched pid $! on $(hostname) at $(date -u +%FT%TZ)"
