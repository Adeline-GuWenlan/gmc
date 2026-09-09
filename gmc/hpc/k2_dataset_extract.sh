#!/bin/bash
# Extract the fisheye_gs K2 capture out of its apptainer overlay image.
#
# WHY THIS IS NOT A MOUNT
# -----------------------
# docs/worklog/k2_real_scene_test_a.md step 0 recorded this dataset as
# unreadable: mounted read-only against the CentOS base image, the overlay adds
# exactly one directory at /, `datasets`, which reports as nfsnobody:nfsnobody
# mode 0750 and cannot be entered.  The worklog concluded that reading it would
# need a read-write mount, which mutates the image, and stopped there.
#
# That diagnosis was wrong in its particulars, and the correct one makes the
# problem go away:
#
#   * The payload is not at /datasets.  It is at /upper/datasets -- `upper` is
#     the overlayfs upper directory, mode 0777, uid 54185.
#   * /upper/datasets is mode 0750 owned by uid 3003305 gid 100 -- a different
#     REAL user, not nfsnobody.  The container only rendered it as nfsnobody
#     because uid 3003305 has no passwd entry inside the base image.  That also
#     means --fakeroot could never have worked: an unmappable owner uid defeats
#     the user-namespace root mapping, so no mount-based route gets in.
#
# debugfs reads an ext2/3/4 image as an ordinary FILE.  No mount, no loop
# device, no root, and no permission check is ever consulted -- the mode bits
# belong to a filesystem the kernel never mounts.  Invoked without -w it cannot
# write, so the source image is provably untouched.  This is why no read-write
# mount, and therefore no mutation of the delivered image, is required.
#
# VERIFIED 2026-09-08: 26 GB, 6,689 files, all 66 shipped SHA256SUMS entries OK,
# and map.ply hashing to d6f8f327...c43a2 exactly as map.ply.manifest.json
# declares.
#
# Light I/O against /scratch; safe on a login node.  No SLURM job needed.
set -euo pipefail

IMAGE=${IMAGE:-/scratch/wg2381/fisheye_gs/xgrid_indoor_1_k2_full_64g.ext3}
DEST=${DEST:-/scratch/wg2381/fisheye_gs/k2_dataset}
SRCPATH=${SRCPATH:-/upper/datasets/xgrid-indoor-1}

[ -f "$IMAGE" ] || { echo "FATAL: image not found: $IMAGE" >&2; exit 1; }

# debugfs is in /usr/sbin, which is not always on a login-node PATH.
DEBUGFS=$(command -v debugfs || echo /usr/sbin/debugfs)
[ -x "$DEBUGFS" ] || { echo "FATAL: debugfs not found" >&2; exit 1; }

mkdir -p "$DEST"
LOG="${DEST%/}/../logs/rdump_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$(dirname "$LOG")"

echo "image : $IMAGE (opened read-only; no -w flag is ever passed)"
echo "from  : $SRCPATH"
echo "to    : $DEST"

# Do NOT pipe debugfs into head/grep: it dies of SIGPIPE partway through and
# leaves a silently truncated tree behind an exit code of 0.  Redirect instead.
#
# The log fills with "Operation not permitted while changing ownership" -- that
# is rdump failing to reproduce uid 3003305, which an unprivileged user cannot
# set.  Harmless, and in fact the desired outcome: the extracted files end up
# owned by the invoking user.  Any OTHER error is real.
"$DEBUGFS" -R "rdump $SRCPATH $DEST" "$IMAGE" > "$LOG" 2>&1

echo "--- errors other than the expected ownership warnings ---"
grep -viE "changing ownership|^debugfs [0-9]" "$LOG" || echo "(none)"

echo "--- extracted ---"
du -sh "$DEST"
find "$DEST" -type f | wc -l

# The dataset ships its own checksums.  Verify rather than assume.
SUMS="$DEST/xgrid-indoor-1/SHA256SUMS"
if [ -f "$SUMS" ]; then
  echo "--- verifying $(wc -l < "$SUMS") shipped checksums ---"
  ( cd "$DEST/xgrid-indoor-1" && sha256sum -c SHA256SUMS ) \
    | grep -vE ": OK$" || echo "all checksums OK"
fi
