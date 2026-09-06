"""Layer 2 of the release gate (round-6 P0-R2): compare a CLEAN-ROOM
reproduction against the packaged canonical artifacts, number by number.

The v1 gate validated internal consistency only; round-6 showed that a
package can pass every static check and still not reproduce.  This script
closes that hole: after unzipping the package on a fresh host and running
src_snapshot/reproduce.sh, every canonical table and every witness
amendment must match the shipped ones —

  compared exactly : statuses, verdicts, event/section/station counts,
                     all query counters (c_seed/c_track/c_densify/c_cert/
                     c_connector/total), pair_ops, bp_hits, summary counts,
                     budgets, leaf counts, booleans, strings
  compared with tol: every float (margins, fit coefficients, gammas) —
                     |a-b| <= ABS_TOL or relative <= REL_TOL
  ignored          : provenance blocks, generated_at, and any key whose
                     name contains "second"/"wall"/"time" (host-dependent)

Usage:
  python compare_reproduction.py <package_dir> <reproduced_snapshot_root>

<reproduced_snapshot_root> is the src_snapshot/ directory in which
reproduce.sh was executed (it holds results/tables/ and
outputs/oracle_records_amendments/ produced by the rerun).
Exit 0 = every artifact matches; exit 1 = any missing file or mismatch.
"""
import json
import math
import os
import sys

ABS_TOL = 1e-6
REL_TOL = 1e-6
SKIP_KEYS = {"provenance", "_provenance", "generated_at"}
SKIP_SUBSTR = ("second", "wall", "time")
MAX_REPORT = 60


def skip_key(k):
    lk = str(k).lower()
    return k in SKIP_KEYS or any(s in lk for s in SKIP_SUBSTR)


def compare(a, b, path, diffs, stats):
    if isinstance(a, dict) and isinstance(b, dict):
        ka, kb = set(a) - SKIP_KEYS, set(b) - SKIP_KEYS
        ka = {k for k in ka if not skip_key(k)}
        kb = {k for k in kb if not skip_key(k)}
        for k in sorted(ka | kb):
            if k not in ka:
                diffs.append(f"{path}.{k}: only in reproduction")
            elif k not in kb:
                diffs.append(f"{path}.{k}: only in package")
            else:
                compare(a[k], b[k], f"{path}.{k}", diffs, stats)
        return
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            diffs.append(f"{path}: length {len(a)} != {len(b)}")
            return
        for i, (x, y) in enumerate(zip(a, b)):
            compare(x, y, f"{path}[{i}]", diffs, stats)
        return
    stats["leaves"] += 1
    if isinstance(a, bool) or isinstance(b, bool) \
            or isinstance(a, str) or isinstance(b, str) \
            or a is None or b is None:
        if a != b:
            diffs.append(f"{path}: {a!r} != {b!r}")
        return
    if isinstance(a, int) and isinstance(b, int):
        if a != b:
            diffs.append(f"{path}: int {a} != {b}")
        return
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if not math.isclose(a, b, rel_tol=REL_TOL, abs_tol=ABS_TOL):
            diffs.append(f"{path}: float {a} != {b} "
                         f"(|d|={abs(a - b):.3e})")
        else:
            stats["float_maxdev"] = max(stats["float_maxdev"], abs(a - b))
        return
    if a != b:
        diffs.append(f"{path}: {a!r} != {b!r} (type mismatch)")


def compare_file(pkg_p, rep_p, diffs, stats):
    if not os.path.exists(rep_p):
        diffs.append(f"{os.path.basename(pkg_p)}: reproduction output "
                     f"MISSING ({rep_p})")
        return
    a = json.load(open(pkg_p))
    b = json.load(open(rep_p))
    # a=package (reference), b=reproduction
    compare(b, a, os.path.basename(pkg_p), diffs, stats)


def main(pkg, rep_root):
    all_diffs = []
    n_files = 0
    tab_dir = os.path.join(pkg, "tables")
    for f in sorted(os.listdir(tab_dir)):
        if not f.endswith(".json"):
            continue
        diffs, stats = [], {"leaves": 0, "float_maxdev": 0.0}
        compare_file(os.path.join(tab_dir, f),
                     os.path.join(rep_root, "results", "tables", f),
                     diffs, stats)
        n_files += 1
        tag = "MATCH" if not diffs else f"MISMATCH ({len(diffs)})"
        print(f"{f}: {tag}  [{stats['leaves']} leaves, "
              f"float maxdev {stats['float_maxdev']:.2e}]")
        all_diffs += [f"tables/{f} :: {d}" for d in diffs]
    am_dir = os.path.join(pkg, "oracle_amendments")
    if os.path.isdir(am_dir):
        for f in sorted(os.listdir(am_dir)):
            if not f.endswith(".json"):
                continue
            diffs, stats = [], {"leaves": 0, "float_maxdev": 0.0}
            compare_file(
                os.path.join(am_dir, f),
                os.path.join(rep_root, "outputs",
                             "oracle_records_amendments", f),
                diffs, stats)
            n_files += 1
            tag = "MATCH" if not diffs else f"MISMATCH ({len(diffs)})"
            print(f"oracle_amendments/{f}: {tag}  "
                  f"[{stats['leaves']} leaves]")
            all_diffs += [f"oracle_amendments/{f} :: {d}" for d in diffs]

    print(f"\ncompared {n_files} artifacts")
    if all_diffs:
        print(f"REPRODUCTION: FAIL ({len(all_diffs)} difference(s)):")
        for d in all_diffs[:MAX_REPORT]:
            print("  " + d)
        if len(all_diffs) > MAX_REPORT:
            print(f"  ... and {len(all_diffs) - MAX_REPORT} more")
        return 1
    print("REPRODUCTION: MATCH (layer 2 pass)")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1], sys.argv[2]))
