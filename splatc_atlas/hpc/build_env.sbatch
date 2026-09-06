#!/bin/bash
#SBATCH -J splatc_env
#SBATCH -p compute
#SBATCH -q cair
#SBATCH -c 4
#SBATCH --mem=16G
#SBATCH -t 02:00:00
#SBATCH -o logs/%x_%j.out
#SBATCH -e logs/%x_%j.err
# Build the conda-in-overlay environment for splathjb (top-level README HPC
# discipline: env creation runs on a compute node; nothing installed to the
# shared filesystem).
set -euo pipefail
cd /scratch/sy2366/Project/splathjb
module load singularity 2>/dev/null || true

SIF=/share/apps/admin/singularity-images/centos-8.2.2004.sif
OVL=overlays/splathjb_env.ext3

if [ ! -f "$OVL" ]; then
  singularity overlay create --size 10240 "$OVL"
fi

singularity exec --overlay "$OVL":rw "$SIF" bash -c '
set -euo pipefail
if [ ! -d /ext3/miniforge3 ]; then
  cd /tmp
  curl -fsSL -o mf.sh \
    https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
  bash mf.sh -b -p /ext3/miniforge3
  rm -f mf.sh
fi
source /ext3/miniforge3/etc/profile.d/conda.sh
# pin to match the local pilot exactly (python 3.11 / numpy 1.26.4 / scipy 1.10)
# — base is py3.13 where numpy 1.26 has no wheels (first build attempt failed)
if ! conda env list | grep -q "/ext3/miniforge3/envs/splatc"; then
  mamba create -y -n splatc python=3.11 "numpy=1.26.4" "scipy=1.10.*" \
    matplotlib pytest
fi
printf "source /ext3/miniforge3/etc/profile.d/conda.sh\nconda activate splatc\n" \
  > /ext3/env.sh
'

# smoke test: full analytic suite inside the sealed env (read-only overlay)
singularity exec --overlay "$OVL":ro "$SIF" bash -c '
source /ext3/env.sh
cd /scratch/sy2366/Project/splathjb/splatc_atlas
PYTHONPATH=src python -m pytest tests -q
'
echo "ENV BUILD OK"
