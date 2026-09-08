#!/bin/bash
#SBATCH --account=torch_pr_527_general
#SBATCH --partition=cpu_short
#SBATCH --job-name=gmc_k2_overlay
#SBATCH --time=00:10:00
#SBATCH --mem=2G
#SBATCH --cpus-per-task=1
#SBATCH --output=/scratch/wg2381/splathjb/gmc/logs/%x-%j.out
# Read-only inventory of the fisheye_gs overlay image.
#
# --contain: no host directories are bind-mounted into the container, so
# everything visible is base image + overlay and nothing else.  An earlier
# version of this script omitted it and walked the host /scratch bind, which
# listed other users' directories; that is out of scope and must not happen.
# --overlay :ro : the image itself is never written.
set -x
OVERLAY=/scratch/wg2381/fisheye_gs/xgrid_indoor_1_k2_full_64g.ext3
BASE=/share/apps/images/centos-7.9.2009.sif

echo "=== overlay filesystem usage (the decisive number) ==="
singularity exec --contain --overlay "${OVERLAY}:ro" "$BASE" df -h /

echo "=== root listing WITH overlay ==="
singularity exec --contain --overlay "${OVERLAY}:ro" "$BASE" ls -la / > /tmp/with_overlay.txt
cat /tmp/with_overlay.txt

echo "=== root listing WITHOUT overlay (base image alone) ==="
singularity exec --contain "$BASE" ls -la / > /tmp/base_only.txt
cat /tmp/base_only.txt

echo "=== diff: what the overlay adds at / ==="
diff /tmp/base_only.txt /tmp/with_overlay.txt || true

echo "=== overlay payload (staying inside the container root only) ==="
singularity exec --contain --overlay "${OVERLAY}:ro" "$BASE" bash -c '
  for d in /ext3 /data /work /opt/data /root /var/data; do
    [ -d "$d" ] && { echo "--- $d"; ls -la "$d"; }
  done
  echo "--- files >50M anywhere on the overlay root fs"
  find / -xdev -type f -size +50M -printf "%10s  %p\n" 2>/dev/null | head -40
  echo "--- ply / npz / ckpt anywhere on the overlay root fs"
  find / -xdev \( -name "*.ply" -o -name "*.npz" -o -name "*.ckpt" -o -name "*.pth" \) -printf "%10s  %p\n" 2>/dev/null | head -40
'
echo "=== K2_OVERLAY_DONE ==="
