#!/bin/bash
#SBATCH -J p38_chain
#SBATCH -p compute
#SBATCH -q cair
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -c 4
#SBATCH --mem=24G
#SBATCH -t 16:00:00
#SBATCH -o /scratch/sy2366/Project/splathjb/splatc_atlas/outputs/p38_chain.log
# Round-11 tree bump (P3.8 chart-factored connector benchmark): new
# baselines module + p38_connector_bench + validator 7c + report ->
# new src_tree/code_bundle -> FULL canonical chain regenerates on the
# new tree (single-tree discipline).  Own allocation (direct-ssh
# multi-hour runs die with the adoptive cgroup).
set -u
PROJ=/scratch/sy2366/Project/splathjb
singularity exec \
  --overlay "$PROJ/overlays/splathjb_env.ext3:ro" \
  /share/apps/admin/singularity-images/centos-8.2.2004.sif \
  bash -c "source /ext3/env.sh && \
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
    echo P38_CHAIN_START \$(date -u +%FT%TZ) \$(hostname) && \
    cd $PROJ/splatc_atlas && \
    bash experiments/compiler/reproduce.sh && \
    echo P38_CHAIN_ALL_DONE"
