#!/bin/bash
#SBATCH -p compute
#SBATCH -q cair
#SBATCH -c 4
#SBATCH --mem=16G
#SBATCH -t 12:00:00
#SBATCH -J gmc_close
#SBATCH -o /scratch/sy2366/Project/splathjb/gmc_current_20260902/outputs/slurm-%j.out

set -euo pipefail
set -x

SING=/share/apps/NYUAD5/singularity/4.2.0/bin/singularity
SIF=/share/apps/admin/singularity-images/centos-8.2.2004.sif
OVL=/scratch/sy2366/Project/gmc/gmc_env.ext3
PROJ=/scratch/sy2366/Project/splathjb/gmc_current_20260902

test -x "$SING"
test -f "$SIF"
test -f "$OVL"
ATLAS_SNAPSHOT="$PROJ/../splatc_atlas/outputs/round4_final_package/src_snapshot"
test -f "$ATLAS_SNAPSHOT/../MANIFEST.sha256"
test -f "$ATLAS_SNAPSHOT/results/manifests/split_seals.json"
test -f "$ATLAS_SNAPSHOT/src/splatc/datasets/g1_gate.py"
mkdir -p "$PROJ/outputs" "$PROJ/results"

"$SING" exec --overlay "$OVL":ro "$SIF" bash -c '
set -euo pipefail
PY=/ext3/miniconda3/envs/gmc/bin/python
PROJ=/scratch/sy2366/Project/splathjb/gmc_current_20260902

test -x "$PY"
cd "$PROJ"
export PYTHONPATH=src
export MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=4
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

echo "=== VERSIONS ==="
"$PY" -c "import sys, numpy, scipy, shapely, networkx; print(sys.version.split()[0], numpy.__version__, scipy.__version__, shapely.__version__, networkx.__version__)"

echo "=== PYTEST FULL SUITE ==="
"$PY" -m pytest

echo "=== CLI TOY DEMO ==="
"$PY" -m gmc.cli synth single-door --width 0.6 --robot "a=0.50 b=0.20" -o scenes/synthetic/door_hpc.yaml
RUN="outputs/door_run_${SLURM_JOB_ID}"
"$PY" -m gmc.cli compile scenes/synthetic/door_hpc.yaml --config configs/toy.yaml -o "$RUN"
"$PY" -m gmc.cli query "$RUN" --start=-2,0,0.3 --goal=2,0,0.3
RESULT=$(find "$RUN/queries" -type f -name result.json | sort | head -1)
test -n "$RESULT"
"$PY" -m gmc.cli verify "$RESULT"

echo "=== ATLAS STAGED GATE CLOSURE ==="
ATLAS_OUT="results/atlas_g1_full_gate_intervals_${SLURM_JOB_ID}.json"
"$PY" -m gmc.cli benchmark configs/benchmarks/atlas_g1_full_gate_intervals.yaml -o "$ATLAS_OUT"
test -s "$ATLAS_OUT"

echo "=== GMC_CURRENT_CLOSURE_DONE ==="
'
