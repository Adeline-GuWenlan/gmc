#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f4_probe
#SBATCH --time=04:00:00
#SBATCH --mem=2000M
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# F4 Task 3: one probe of one region/robot on F3's persisted compile (or the scene, for epclear / witness).
#   kin      1 ms turn + translation floors (aerial3dg_fail2_kin.py --full) on the shared_replay_failed rows of ROWS
#   bufzero  buffer-0 endpoint query cell (aerial3dg_fail3_probe.py bufzero) on the *_not_certified_free rows of ROWS
#   requery  re-query every row of ROWS on the same compile, 300 s limit, verdict-only for TIMEOUT rows, certificate dump
#   epclear  the robot's own endpoint clearance ladder for the pair ids listed in ROWS (aerial3dg_fail3_f4.py)
#   witness  real-body A* at margin 0.0021 for the pair ids listed in ROWS (aerial3dg_fail3_sample.py witness)
# Usage: sbatch --job-name=a3f4_probe_<what> f4_probe.sbatch MODE REGION ROBOT ROWS OUT
set -euo pipefail
MODE=$1; REG=$2; R=$3; ROWS=$4; OUT=$5
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
A3C=outputs/aerial3dg/f3/$REG/$R.a3c; PAIRS=results/aerial3dg/f4/sample/${REG}_pairs.json
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ) $MODE $REG $R $ROWS"
case $MODE in
  kin)     $PY experiments/aerial3dg_fail2_kin.py --full --a3c $A3C --rows $ROWS --out $OUT ;;
  bufzero) $PY experiments/aerial3dg_fail3_probe.py bufzero --a3c $A3C --rows $ROWS --out $OUT ;;
  requery) $PY experiments/aerial3dg_fail3_probe.py requery --a3c $A3C --rows $ROWS --pairs $PAIRS --timeout 300 --out $OUT ;;
  epclear) $PY experiments/aerial3dg_fail3_f4.py epclear --region $REG --robot $R --ids-file $ROWS --out $OUT ;;
  witness) $PY experiments/aerial3dg_fail3_sample.py witness --pairs $PAIRS --ids $(cat $ROWS) --out $OUT ;;
  *) echo "unknown mode $MODE"; exit 2 ;;
esac
echo "end $(date -u +%FT%TZ)"
