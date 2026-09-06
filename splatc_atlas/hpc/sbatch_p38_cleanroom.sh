#!/bin/bash
#SBATCH -J p38_cleanroom
#SBATCH -p compute
#SBATCH -q cair
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -c 4
#SBATCH --mem=24G
#SBATCH -t 16:00:00
#SBATCH -o /scratch/sy2366/Project/splathjb/splatc_atlas/outputs/p38_cleanroom.log
# Layer-2 acceptance for the round-11 (P3.8) package: three gates with
# individually captured exit codes, then the machine receipt.  Clean
# room on /scratch (user directive 2026-08-21: no tmpfs), ephemeral dir
# removed on exit.  Own allocation.
set -u
PROJ=/scratch/sy2366/Project/splathjb
CR=$PROJ/splatc_atlas/outputs/cleanroom_p38_${SLURM_JOB_ID}
ZIP=$PROJ/splatc_atlas/outputs/round4_final_package.zip
RECEIPT=$PROJ/splatc_atlas/outputs/cleanroom_reproduction_receipt.json
trap 'rm -rf "$CR"' EXIT
rm -rf "$CR"; mkdir -p "$CR"
singularity exec \
  --overlay "$PROJ/overlays/splathjb_env.ext3:ro" \
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
