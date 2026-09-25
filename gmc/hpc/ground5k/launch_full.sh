#!/bin/bash
# G1 Task 3: submit the full run for one scene as four throttled arrays (one per combination).
# Usage: launch_full.sh <scene> <prefix N> [--dependency=afterany:ID:ID...]
# Layout from the pilot (docs/ground5k_design.md §2.3): pairs per task keep a task under the
# 6 h cpu_short QOS wall even when every query hits the 1800 s cap for GMC cylinder; memory is
# pilot sacct MaxRSS x 1.5; throttles keep <= 40 tasks and share the 120 GB per-user QOS.
set -euo pipefail
cd /scratch/wg2381/splathjb-ground5k/gmc
SCENE=$1; N=$2; DEP=${3:-}
IDS=/scratch/wg2381/claude_jobs/ground5k/jobids/G1.txt
#            combo           per_task  mem  throttle
LAYOUT="astar_sweeper    250  6G  1
        astar_cylinder   125  6G  1
        gmc_sweeper       25  8G  2
        gmc_cylinder       9  12G 8"
echo "$LAYOUT" | while read -r COMBO PER MEM THR; do
  LAST=$(( (N + PER - 1) / PER - 1 ))
  J=$(sbatch --parsable ${DEP} --job-name=g5_${SCENE}_${COMBO} --array=0-${LAST}%${THR} --time=05:55:00 \
      --mem=${MEM} --cpus-per-task=1 hpc/ground5k/run_array.sbatch \
      --scene ${SCENE} --combo ${COMBO} --first 0 --per-task ${PER} --limit ${N})
  echo "$J g5_${SCENE}_${COMBO} $(date -u +%FT%TZ) full run ${SCENE} pairs [0,${N}) ${COMBO}: ${PER}/task, ${MEM}, array 0-${LAST}%${THR} ${DEP}" >> $IDS
  echo "${COMBO} ${J} 0-${LAST}%${THR} ${PER} ${MEM}"
done
