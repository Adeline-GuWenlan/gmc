#!/bin/bash
#SBATCH -J p0r2_cleanroom
#SBATCH -p compute
#SBATCH -q cair
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -c 4
#SBATCH --mem=24G
#SBATCH -t 12:00:00
#SBATCH -o /scratch/sy2366/Project/splathjb/splatc_atlas/outputs/p0r2_cleanroom_sbatch.log
# Formal single-shot P0-R2 layer-2 acceptance in an OWN allocation.
# The direct-ssh clean-room died at ~6h when its adoptive job's cgroup
# was torn down (silent SIGKILL, no traceback) — borrowed cgroups are a
# walltime lottery for multi-hour chains; sbatch is the fix.
set -u
PROJ=/scratch/sy2366/Project/splathjb
CR=$PROJ/cleanroom_p0r2_sbatch
rm -rf "$CR"; mkdir -p "$CR"
singularity exec \
  --overlay "$PROJ/overlays/splathjb_env.ext3:ro" \
  /share/apps/admin/singularity-images/centos-8.2.2004.sif \
  bash -c "source /ext3/env.sh && \
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
    echo CLEANROOM_START \$(date -u +%FT%TZ) \$(hostname) && \
    cd $CR && python -m zipfile -e $PROJ/splatc_atlas/outputs/round4_final_package.zip . && \
    PKG=$CR/round4_final_package && \
    echo '== layer 1: validate_package.py' && \
    python \$PKG/validate_package.py \$PKG && \
    echo '== reproduce.sh (full chain from snapshot)' && \
    cd \$PKG/src_snapshot && bash reproduce.sh && \
    echo '== layer 2: compare_reproduction.py' && \
    cd \$PKG && python compare_reproduction.py \$PKG \$PKG/src_snapshot && \
    echo CLEANROOM_ALL_PASS"
