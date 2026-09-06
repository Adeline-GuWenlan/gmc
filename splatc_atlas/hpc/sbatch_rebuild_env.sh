#!/bin/bash
#SBATCH -J rebuild_env
#SBATCH -p compute
#SBATCH -q cair
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -c 4
#SBATCH --mem=16G
#SBATCH -t 02:00:00
#SBATCH -o /scratch/sy2366/Project/splathjb/splatc_atlas/outputs/rebuild_env.log
# The splatc conda env (locked numeric stack) lived at
# /scratch/sy2366/Project/conda_envs/splatc and was deleted in the
# 2026-08-20 quota cleanup.  Rebuild it with the EXACT pinned versions
# the canonical chain is locked to, assert them, smoke the test suite,
# re-measure environment_freeze.txt from the rebuilt env, and export an
# explicit spec next to the overlay so the next loss is a 5-minute
# restore instead of a resolve.
set -u
PROJ=/scratch/sy2366/Project/splathjb
singularity exec \
  --overlay "$PROJ/overlays/splathjb_env.ext3:ro" \
  /share/apps/admin/singularity-images/centos-8.2.2004.sif \
  bash -c "source /ext3/miniforge3/etc/profile.d/conda.sh && \
    mamba create -y -n splatc \
      python=3.11.15 numpy=1.26.4 scipy=1.10.1 \
      matplotlib=3.11.1 pytest=9.1.1 && \
    conda activate splatc && \
    python -c 'import sys, numpy, scipy, matplotlib, pytest; \
print(sys.version.split()[0], numpy.__version__, scipy.__version__, \
matplotlib.__version__, pytest.__version__); \
assert sys.version.split()[0] == \"3.11.15\"; \
assert numpy.__version__ == \"1.26.4\"; \
assert scipy.__version__ == \"1.10.1\"; \
assert matplotlib.__version__ == \"3.11.1\"; \
assert pytest.__version__ == \"9.1.1\"' && \
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
    cd $PROJ/splatc_atlas && PYTHONPATH=src python -m pytest tests/ -q \
      | tail -2 && \
    { echo \"# pip freeze, locked env rebuilt after 2026-08-20 quota-\"; \
      echo \"# cleanup loss; same pinned stack: python 3.11.15\"; \
      pip freeze; } > experiments/compiler/environment_freeze.txt && \
    conda list --explicit > $PROJ/overlays/splatc_env_explicit.txt && \
    echo ENV_REBUILD_DONE"
