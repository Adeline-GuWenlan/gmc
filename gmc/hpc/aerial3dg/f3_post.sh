#!/bin/bash
# F3: after a region's sampler streams finish (login node; json only): collect the accepted pairs, then submit one GMC
# job per robot on the region's persisted compile (outputs/aerial3dg/f3/<REGION>/<robot>.a3c, built by f3_compile.sbatch).
# Usage: hpc/aerial3dg/f3_post.sh REGION KIND U0 V0 U1 V1 PREFIX     (KIND = pilot | targeted)
set -euo pipefail
REG=$1; KIND=$2; U0=$3; V0=$4; U1=$5; V1=$6; PFX=$7
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments
J=/scratch/wg2381/claude_jobs/aerial3dg_fail/jobids/F3.txt
PAIRS=results/aerial3dg/f3/sample/$REG/${KIND}_pairs.json
timeout 110 /scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/aerial3dg_fail3_sample.py collect \
  --out-dir results/aerial3dg/f3/sample/$REG/$KIND --region $REG --prefix $PFX --out $PAIRS | cut -c1-400
for R in cylinder sweeper; do
  id=$(sbatch --parsable --mem=${GMEM:-2500M} ${SBX:-} --job-name=a3f3_gmc_${REG}_${KIND}_$R hpc/aerial3dg/f3_gmc.sbatch $R $U0 $V0 $U1 $V1 $PAIRS \
       outputs/aerial3dg/f3/$REG/$R.a3c results/aerial3dg/f3/gmc/$REG/$KIND/$R)
  echo "$id a3f3_gmc_${REG}_${KIND}_$R $(date -u +%FT%TZ) T1 GMC $R on $REG $KIND pairs (persisted compile, G2 QCONFIG)" >> $J
  echo "$id GMC $R $REG $KIND"
done
