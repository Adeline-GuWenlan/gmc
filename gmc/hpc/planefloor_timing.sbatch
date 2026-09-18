#!/bin/bash
# gmc/hpc/planefloor_timing.sbatch -- Amendment 3 P4: the ONE dedicated timing sweep.
#   sbatch --job-name=pf_p4_sweep gmc/hpc/planefloor_timing.sbatch
# Runs experiments/plane_timing.py --step selftest, then its four lanes side by side, one process and one
# core each (GMC is single-threaded: the cylinder used 0.99 of 8 cores over 2.5 h on 09-17).
# Memory, from sacct of the runs each lane repeats: cylinder 2.2-4.5 GB up to 149 k supports and 14.2 GB
# on P2's 156 k map (its long query); sweeper 2.8 GB; uav 2.2-2.9 GB; the P1b build 6.5 GB. The prep lane's
# compile-only units stop themselves at P4_MEM_GUARD_GB. Worst case of all four at once ~39 GB.
# Each lane writes results/height/timing/sweep/<lane>.jsonl as it goes and stops starting new units
# 20 min before the wall clock (P4_DEADLINE_EPOCH).
#SBATCH --account=torch_pr_527_general
#SBATCH --job-name=pf_p4_sweep
#SBATCH --time=06:00:00
#SBATCH --mem=48G
#SBATCH --cpus-per-task=4
#SBATCH --output=/scratch/wg2381/splathjb-plane/gmc/logs/%x-%j.out
set -euo pipefail
PY=/scratch/wg2381/.conda/envs/gmc-venv/bin/python
cd /scratch/wg2381/splathjb-plane/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg PYTHONUNBUFFERED=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export P4_DEADLINE_EPOCH=$(( $(date +%s) + 6 * 3600 - 20 * 60 ))
export P4_MEM_GUARD_GB=${P4_MEM_GUARD_GB:-18}
echo "=== planefloor_timing $(date -Is) host=$(hostname) job=${SLURM_JOB_ID} cpus=${SLURM_CPUS_PER_TASK} ==="
grep -m1 'model name' /proc/cpuinfo || true
$PY experiments/plane_timing.py --step selftest
pids=()
for lane in ${LANES:-prep cyl sweeper uav}; do
  $PY experiments/plane_timing.py --step sweep --lane "${lane}" \
    > "logs/pf_p4_sweep-${SLURM_JOB_ID}-${lane}.out" 2>&1 &
  pids+=($!)
  echo "lane ${lane} pid $!"
done
rc=0
for p in "${pids[@]}"; do wait "$p" || rc=1; done
echo "=== P4_SWEEP_DONE rc=${rc} $(date -Is) ==="
exit ${rc}
