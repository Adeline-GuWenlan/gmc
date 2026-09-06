#!/bin/bash
#SBATCH -p compute
#SBATCH -q cair
#SBATCH -c 4
#SBATCH --mem=16G
#SBATCH -t 02:00:00
#SBATCH -J gmc_numeric_final
#SBATCH -o /scratch/sy2366/Project/splathjb/gmc_current_20260902/outputs/numeric-final-%j.out

set -euo pipefail

SING=/share/apps/NYUAD5/singularity/4.2.0/bin/singularity
SIF=/share/apps/admin/singularity-images/centos-8.2.2004.sif
OVL=/scratch/sy2366/Project/gmc/gmc_env.ext3
PROJ=/scratch/sy2366/Project/splathjb/gmc_current_20260902

"$SING" exec --overlay "$OVL":ro "$SIF" bash -c '
set -euo pipefail
cd /scratch/sy2366/Project/splathjb/gmc_current_20260902
export PYTHONPATH=src
export OPENBLAS_NUM_THREADS=4
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
PY=/ext3/miniconda3/envs/gmc/bin/python
"$PY" -m pytest --durations=20 \
  tests/unit/test_envelopes.py \
  tests/regression/test_p3_provenance_guards.py \
  tests/regression/test_interval_artifact_tree.py \
  tests/regression/test_large_coordinate_soundness.py \
  tests/regression/test_orientation_soundness.py \
  tests/theorem/test_connectivity_intervals.py \
  tests/theorem/test_interval_global_soundness.py \
  tests/theorem/test_interval_pair_certificate.py \
  tests/theorem/test_interval_slab_cover.py \
  tests/integration/test_atlas_gate_intervals.py \
  tests/integration/test_interval_cover_graph.py \
  tests/integration/test_query_refinement.py
'
