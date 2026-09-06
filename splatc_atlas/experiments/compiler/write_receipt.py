"""Write the clean-room reproduction receipt (round-7 P0-R3).

Runs at the END of the layer-2 acceptance job, INSIDE the locked
environment, with the actual exit codes of the three gates.  The receipt
records the identity of exactly what was verified; the final release zip
adds ONLY this receipt on top of the verified content.

Usage:
  python write_receipt.py <pkg_dir> <zip_path> <out_path> \
      <validate_exit> <reproduce_exit> <compare_exit> <started_utc>
"""
import hashlib
import json
import os
import platform
import subprocess
import sys
from datetime import datetime, timezone


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def main(pkg, zip_path, out, ve, re_, ce, started):
    sys.path.insert(0, os.path.join(pkg, "src_snapshot",
                                    "experiments", "compiler"))
    from validate_package import sha_bundle
    freeze = subprocess.run([sys.executable, "-m", "pip", "freeze"],
                            capture_output=True, text=True).stdout
    tables = {}
    for sub in ("tables", "oracle_amendments"):
        d = os.path.join(pkg, sub)
        if os.path.isdir(d):
            for f in sorted(os.listdir(d)):
                if f.endswith(".json"):
                    tables[f"{sub}/{f}"] = sha(os.path.join(d, f))
    receipt = {
        "note": "layer-2 clean-room acceptance receipt; the final release "
                "zip adds ONLY this file on top of the verified content",
        "content_manifest_sha256": sha(os.path.join(pkg, "MANIFEST.sha256")),
        # round-8 rename (was package_zip_sha256): this is the CONTENT
        # zip verified by this acceptance run — the final release zip
        # differs (it adds this receipt); its sha256 ships as a detached
        # .sha256 file NEXT TO the zip, never inside it
        "verified_content_zip_sha256": sha(zip_path)
        if os.path.exists(zip_path) else None,
        "code_bundle_sha256": sha_bundle(os.path.join(pkg, "src_snapshot")),
        "host": platform.node(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "environment_freeze_sha256": hashlib.sha256(
            freeze.encode()).hexdigest(),
        "environment_freeze": freeze.strip().splitlines(),
        "started_utc": started,
        "finished_utc": datetime.now(timezone.utc).isoformat(
            timespec="seconds"),
        "validate_exit": int(ve),
        "reproduce_exit": int(re_),
        "compare_exit": int(ce),
        "canonical_table_sha256": tables,
    }
    with open(out, "w") as f:
        json.dump(receipt, f, indent=1)
    print(f"receipt -> {out} (gates {ve}/{re_}/{ce})")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3],
         sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7])
