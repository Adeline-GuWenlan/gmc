#!/bin/bash
#SBATCH -J p4a2_cleanroom
#SBATCH -p compute
#SBATCH -q cair
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -c 4
#SBATCH --mem=32G
#SBATCH -t 16:00:00
#SBATCH -o /scratch/sy2366/Project/splathjb/splatc_atlas/outputs/p4a2_cleanroom.log
# Layer-2 acceptance v4 (round-8): three gates with INDIVIDUALLY captured
# exit codes, then a machine receipt (write_receipt.py).  Own allocation
# (borrowed-cgroup lesson).
#
# v4.1: the clean-room working tree lives on NODE-LOCAL tmpfs, not
# /scratch — attempt 1 (job 17304460) died at the figure-writing tail
# with "Disk quota exceeded" on /scratch while the account's quota was
# being consumed by unrelated workloads; the reproduction working set is
# <1GB and ephemeral by design, so node-local storage is both safer and
# semantically cleaner.  tmpfs is RAM-backed and charged to the job
# cgroup, hence --mem=32G.  Receipt + this log (small files) still land
# on /scratch.
set -u
PROJ=/scratch/sy2366/Project/splathjb
CR=/tmp/cleanroom_p4a2_${SLURM_JOB_ID}
ZIP=$PROJ/splatc_atlas/outputs/round4_final_package.zip
RECEIPT=$PROJ/splatc_atlas/outputs/cleanroom_reproduction_receipt.json
trap 'rm -rf "$CR"' EXIT
rm -rf "$CR"; mkdir -p "$CR"
df -h /tmp | tail -1
singularity exec \
  --overlay "$PROJ/overlays/splathjb_env.ext3:ro" \
  -B /tmp:/tmp \
  /share/apps/admin/singularity-images/centos-8.2.2004.sif \
  bash -c "source /ext3/env.sh && \
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
    START=\$(date -u +%FT%T+00:00) && \
    echo CLEANROOM_START \$START \$(hostname) && \
    cd $CR && python -m zipfile -e $ZIP . && \
    PKG=$CR/round4_final_package && \
    echo '== layer 1: validate_package.py' && \
    python \$PKG/validate_package.py \$PKG; V=\$?; \
    R=1; C=1; \
    if [ \$V -eq 0 ]; then \
      echo '== reproduce.sh (full chain from snapshot)' && \
      cd \$PKG/src_snapshot && bash reproduce.sh; R=\$?; \
    fi; \
    if [ \$R -eq 0 ]; then \
      echo '== layer 2: compare_reproduction.py' && \
      cd \$PKG && python compare_reproduction.py \$PKG \$PKG/src_snapshot; C=\$?; \
    fi; \
    python \$PKG/src_snapshot/experiments/compiler/write_receipt.py \
      \$PKG $ZIP $RECEIPT \$V \$R \$C \$START; \
    if [ \$V -eq 0 ] && [ \$R -eq 0 ] && [ \$C -eq 0 ]; then \
      echo CLEANROOM_ALL_PASS; \
    else \
      echo CLEANROOM_GATE_FAIL V=\$V R=\$R C=\$C; exit 1; \
    fi"
