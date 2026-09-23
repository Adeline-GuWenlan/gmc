#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --job-name=ul_site_gallery
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=8G
#SBATCH --time=00:30:00
#SBATCH --output=/scratch/wg2381/claude_jobs/logs/%x-%j.out
set -euo pipefail
cd /scratch/wg2381/splathjb-uavlamp/gmc
export PYTHONPATH=src:experiments MPLBACKEND=Agg OMP_NUM_THREADS=2
/scratch/wg2381/.conda/envs/gmc-venv/bin/python experiments/uavlamp_site.py \
  --archive /scratch/wg2381/splathjb/gmc/outputs/height/plane/processed_planefloor.npz \
  --output outputs/uavlamp/site_gallery --wall-a 7.5 22 11.6 30.5 --wall-b 10.3 22 15.9 32 --u-range -5 5
