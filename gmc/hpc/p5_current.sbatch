#!/bin/bash
#SBATCH -p compute
#SBATCH -q cair
#SBATCH -c 2
#SBATCH --mem=8G
#SBATCH -t 00:30:00
#SBATCH -J gmc_p5_current
#SBATCH -o /scratch/sy2366/Project/splathjb/gmc_current_20260902/outputs/p5-%j.out

set -euo pipefail

SING=/share/apps/NYUAD5/singularity/4.2.0/bin/singularity
SIF=/share/apps/admin/singularity-images/centos-8.2.2004.sif
OVL=/scratch/sy2366/Project/gmc/gmc_env.ext3
PROJ=/scratch/sy2366/Project/splathjb/gmc_current_20260902

"$SING" exec --overlay "$OVL":ro "$SIF" bash -c '
set -euo pipefail
cd /scratch/sy2366/Project/splathjb/gmc_current_20260902
export PYTHONPATH=src
export OPENBLAS_NUM_THREADS=2
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
PY=/ext3/miniconda3/envs/gmc/bin/python
"$PY" -m gmc.cli benchmark configs/benchmarks/p5_system_scaling.yaml \
  -o "results/p5_system_scaling_${SLURM_JOB_ID}.json"
"$PY" -m gmc.cli benchmark configs/benchmarks/atlas_g1_events.yaml \
  -o "results/p5_atlas_g1_equal_budget_${SLURM_JOB_ID}.json"
echo "=== GMC_P5_CURRENT_DONE ==="
'
