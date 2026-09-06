#!/bin/bash
#SBATCH -J p37_chain
#SBATCH -p compute
#SBATCH -q cair
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -c 4
#SBATCH --mem=24G
#SBATCH -t 12:00:00
#SBATCH -o /scratch/sy2366/Project/splathjb/splatc_atlas/outputs/p37_chain.log
# P3.7 tree bump: src/ gained ROI/theta-quotient params (defaults
# bit-identical, A/B verified locally) -> new src_tree_sha256 -> the FULL
# canonical chain regenerates on the new tree (single-tree discipline),
# plus the new p37_matched.py matched-baseline experiment.  Own
# allocation (direct-ssh multi-hour runs die with the adoptive cgroup).
set -u
PROJ=/scratch/sy2366/Project/splathjb
singularity exec \
  --overlay "$PROJ/overlays/splathjb_env.ext3:ro" \
  /share/apps/admin/singularity-images/centos-8.2.2004.sif \
  bash -c "source /ext3/env.sh && \
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
    echo P37_CHAIN_START \$(date -u +%FT%TZ) \$(hostname) && \
    cd $PROJ/splatc_atlas && \
    bash experiments/compiler/reproduce.sh && \
    echo P37_CHAIN_ALL_DONE"
