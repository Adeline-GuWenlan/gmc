#!/bin/bash
#SBATCH -J splatc_label
#SBATCH -p compute
#SBATCH -q cair
#SBATCH -c 4
#SBATCH --mem=8G
#SBATCH -t 00:40:00
#SBATCH --array=0-41
#SBATCH -o logs/%x_%A_%a.out
#SBATCH -e logs/%x_%A_%a.err
# Oracle labeling of dev+validation manifests (42 episodes, one per task).
# Blind stays sealed and is never labeled here.
set -euo pipefail
cd /scratch/sy2366/Project/splathjb
module load singularity 2>/dev/null || true
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4

singularity exec --overlay overlays/splathjb_env.ext3:ro \
  /share/apps/admin/singularity-images/centos-8.2.2004.sif bash -c "
source /ext3/env.sh
cd /scratch/sy2366/Project/splathjb
PYTHONPATH=splatc_atlas/src python \
  splatc_atlas/experiments/oracle/label_manifests.py --index ${SLURM_ARRAY_TASK_ID}
"
