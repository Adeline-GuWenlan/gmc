#!/bin/bash
# P2 generalization on a directly-accessed compute node (queue congested;
# user-directed 2026-08-13).  Invoke via:  bash -lc hpc/run_p2_cn263.sh
# (login shell needed for the singularity module, same gotcha as sbatch).
set -u
PROJ=/scratch/sy2366/Project/splathjb
cd "$PROJ/splatc_atlas"
mkdir -p outputs
nohup singularity exec \
  --overlay "$PROJ/overlays/splathjb_env.ext3:ro" \
  /share/apps/admin/singularity-images/centos-8.2.2004.sif \
  bash -c "source /ext3/env.sh && cd $PROJ/splatc_atlas && PYTHONPATH=src python experiments/compiler/p2_generalization.py" \
  > outputs/p2_run.log 2>&1 &
echo "launched pid $! on $(hostname) at $(date -u +%FT%TZ)"
