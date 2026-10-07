#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --job-name=a3f3_probe
#SBATCH --time=02:00:00
#SBATCH --mem=3500M
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
# F3 Task 1.2/1.4: probe every GMC failure of one region/kind/robot on the SAME persisted compile:
#   1 ms turn + translation floor (aerial3dg_fail2_kin.py --full) on shared_replay_failed rows,
#   buffer-0 endpoint cell (aerial3dg_fail3_probe.py bufzero) on *_not_certified_free rows,
#   re-query of every non-REACHABLE row (reproduction + certificate dump; aerial3dg_fail3_probe.py requery).
# Usage: f3_probe.sbatch REGION KIND ROBOT
set -euo pipefail
REG=$1; KIND=$2; R=$3
cd /scratch/wg2381/splathjb-aerial3dg-fail/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
A3C=outputs/aerial3dg/f3/$REG/$R.a3c; ROWS=results/aerial3dg/f3/gmc/$REG/$KIND/$R/task_00.jsonl
D=results/aerial3dg/f3/diag/$REG/$KIND; mkdir -p $D
echo "node $(hostname) job ${SLURM_JOB_ID} commit $(git rev-parse HEAD) start $(date -u +%FT%TZ) $REG $KIND $R"
grep -q '"shared_replay_failed"' $ROWS && $PY experiments/aerial3dg_fail2_kin.py --full --a3c $A3C --rows $ROWS --out $D/kin_$R.json
grep -q '_not_certified_free"' $ROWS && $PY experiments/aerial3dg_fail3_probe.py bufzero --a3c $A3C --rows $ROWS --out $D/bufzero_$R.json
grep -q -v '"status": "REACHABLE"' $ROWS && $PY experiments/aerial3dg_fail3_probe.py requery --a3c $A3C --rows $ROWS \
  --pairs results/aerial3dg/f3/sample/$REG/${KIND}_pairs.json --out $D/requery_$R.json
echo "end $(date -u +%FT%TZ)"
