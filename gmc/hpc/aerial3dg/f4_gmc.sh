#!/bin/bash
# F4 Task 2 (login node): GMC on one region's confirmed pairs, one robot, as an array over N_TASKS contiguous chunks,
# on F3's persisted compile of that region (outputs/aerial3dg/f3/<REGION>/<robot>.a3c; SHA-256 checked = handoff).
# Usage: [DEP=afterany:ID:...] hpc/aerial3dg/f4_gmc.sh REGION ROBOT N_TASKS MEM TIME [THROTTLE]
set -euo pipefail
REG=$1; R=$2; NT=$3; MEM=$4; TIME=$5; TH=${6:-$NT}
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
J=/scratch/wg2381/claude_jobs/aerial3dg_fail/jobids/F4.txt
BOX=$(PYTHONPATH=src:experiments /scratch/wg2381/.conda/envs/gmc-venv/bin/python -c "import json;print(*json.load(open('results/aerial3dg/f4/sample/${REG}_pairs.json'))['box_uv'])")
id=$(sbatch --parsable ${DEP:+--dependency=$DEP} --array=0-$((NT-1))%$TH --mem=$MEM --time=$TIME --job-name=a3f4_gmc_${REG}_$R \
     hpc/aerial3dg/f3_gmc.sbatch $R $BOX results/aerial3dg/f4/sample/${REG}_pairs.json \
     outputs/aerial3dg/f3/$REG/$R.a3c results/aerial3dg/f4/gmc/$REG/$R $NT)
echo "$id a3f4_gmc_${REG}_$R $(date -u +%FT%TZ) T2 GMC $R on $REG confirmed pairs, array 0-$((NT-1))%$TH x $MEM (F3 compile, G2 QCONFIG, 120 s)" >> $J
echo "$id"
