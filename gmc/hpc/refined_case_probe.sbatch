#!/bin/bash
#SBATCH -p compute
#SBATCH -q cair
#SBATCH -c 4
#SBATCH --mem=16G
#SBATCH -t 01:00:00
#SBATCH -J gmc_g1_probe
#SBATCH -o /scratch/sy2366/Project/splathjb/gmc_current_20260902/outputs/probe-%j.out

set -euo pipefail
set -x

SING=/share/apps/NYUAD5/singularity/4.2.0/bin/singularity
SIF=/share/apps/admin/singularity-images/centos-8.2.2004.sif
OVL=/scratch/sy2366/Project/gmc/gmc_env.ext3
PROJ=/scratch/sy2366/Project/splathjb/gmc_current_20260902

"$SING" exec --overlay "$OVL":ro "$SIF" bash -c '
set -euo pipefail
PY=/ext3/miniconda3/envs/gmc/bin/python
PROJ=/scratch/sy2366/Project/splathjb/gmc_current_20260902
cd "$PROJ"
export PYTHONPATH=src
export MPLBACKEND=Agg
export OPENBLAS_NUM_THREADS=4
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

"$PY" - <<'"'"'PY'"'"'
from copy import deepcopy
import json
import os
from pathlib import Path

import yaml

from gmc.benchmarking.runner import run_benchmark_spec

path = Path("configs/benchmarks/atlas_g1_full_gate_intervals.yaml")
spec = yaml.safe_load(path.read_text())
spec.pop("formal_profile", None)
stage = deepcopy(spec["stages"][1])
stage["stage"] = "refined_dev_016_diagnostic_probe"
stage["case_ids"] = ["dev_016"]
stage["expected"] = {
    "expected_n_cases": 1,
    "expected_case_ids": ["dev_016"],
    "acceptance_policy": "all_cases_accept",
}
spec["stages"] = [stage]

report = run_benchmark_spec(spec, spec_dir=path.parent)
job_id = os.environ["SLURM_JOB_ID"]
output = Path(f"results/atlas_g1_refined_dev016_probe_{job_id}.json")
output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
print(json.dumps({
    "output": str(output),
    "gate": report["gate"],
    "aggregate_by_stage": report["aggregate_by_stage"],
}, indent=2, sort_keys=True))
if not report["gate"]["pass"]:
    raise SystemExit(1)
PY

echo "=== GMC_REFINED_PROBE_DONE ==="
'
