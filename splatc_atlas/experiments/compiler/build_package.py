"""Assemble the external review package and run layer 1 of the release
gate (round-6 P0-R2).  One command, no hand-copying:

    PYTHONPATH=src python experiments/compiler/build_package.py

Output: outputs/round4_final_package/ + .zip, ONLY if layer-1 validation
passes.  Round-6 lesson: the v1 package shipped WITHOUT its input data
(episode manifests + oracle records), so reproduce.sh crashed on arrival.
The snapshot now contains every input the reproduction chain reads, and
validate_package.py preflights that closure; releasability additionally
requires layer 2 (clean-room reproduce.sh + compare_reproduction.py).
"""
import hashlib
import json
import os
import shutil
import subprocess
import sys

BASE = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))
PKG = os.path.join(BASE, "outputs", "round4_final_package")

FIGS = ["p3_matched_budget.png", "p36_fairness.png", "p37_matched.png",
        "gamma_delta.png",
        "ridge_diag_w0.505.png", "ridge_diag_w0.490.png"]
TABLES = ["p3_matched_budget.json", "p36_fairness.json",
          "p37_matched.json", "p38_connector.json",
          "p4a2_semantic.json", "p4a3_validation.json",
          "ridge_scalar_ablation.json", "p2_generalization.json",
          "gamma_delta.json", "ridge_continuation.json",
          "ridge_stations.json", "floor_probe.json"]
# round-8: the two-document split replaces the single "final" report —
# Gate B evidence (PASS, closed) is frozen apart from the sprint-C P4a
# work so new chart-layer results can never silently dilute it
REPORTS = ["gate_b_evidence_report.md", "sprint_c_p4a_report.md",
           "p38_connector_report.md"]
# frozen negative-result records from EARLIER trees: identity comes from
# the recorded provenance inside each file (validate_package.py exempts
# records/ from current-tree equality but still scans it for banned
# terms); the round-7 chart-count kill FAIL lives here, not in a memo
RECORDS = ["p4a_rigid_charts_round7_15k_FAIL.json",
           # round-10: the round-9 "semantic invariance PASS" table is
           # WITHDRAWN (unsound domain certificate, goal-conditioned
           # portal harness, attachment test that never located the
           # ridge endpoints, accumulated pair-op counters) — frozen
           # here, disclosed in the sprint-C report
           "p4a_semantic_invariance_round9_WITHDRAWN.json"]
SRC_DIRS = ["src", "experiments/compiler", "hpc", "tests"]
# round-6: the reproduction chain's INPUTS ship inside the snapshot.
# dev + validation manifests only — the blind split is SEALED and must
# never enter a package (validate_package.py enforces this).
DATA_SPLITS = ["dev", "validation"]


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def main():
    if os.path.exists(PKG):
        shutil.rmtree(PKG)
    os.makedirs(os.path.join(PKG, "figures"))
    os.makedirs(os.path.join(PKG, "tables"))
    os.makedirs(os.path.join(PKG, "oracle_amendments"))
    os.makedirs(os.path.join(PKG, "records"))
    for f in REPORTS:
        shutil.copy(os.path.join(BASE, "docs", "reports", f), PKG)
    for f in RECORDS:
        shutil.copy(os.path.join(BASE, "outputs", "records", f),
                    os.path.join(PKG, "records"))
    for f in FIGS:
        shutil.copy(os.path.join(BASE, "results", "figures", f),
                    os.path.join(PKG, "figures"))
        side = os.path.join(BASE, "results", "figures", f + ".prov.json")
        if os.path.exists(side):
            shutil.copy(side, os.path.join(PKG, "figures"))
    for f in TABLES:
        shutil.copy(os.path.join(BASE, "results", "tables", f),
                    os.path.join(PKG, "tables"))
    for f in sorted(os.listdir(
            os.path.join(BASE, "outputs", "oracle_records_amendments"))):
        shutil.copy(os.path.join(BASE, "outputs",
                                 "oracle_records_amendments", f),
                    os.path.join(PKG, "oracle_amendments"))
    # full source snapshot (round-5: src_tree_sha256 must be verifiable)
    snap = os.path.join(PKG, "src_snapshot")
    for d in SRC_DIRS:
        shutil.copytree(os.path.join(BASE, d),
                        os.path.join(snap, d),
                        ignore=shutil.ignore_patterns(
                            "__pycache__", "*.pyc", ".DS_Store", "archive"))
    # archive dir separately (versions v1..v4.0 for the record)
    shutil.copytree(os.path.join(BASE, "experiments", "compiler", "archive"),
                    os.path.join(snap, "experiments", "compiler", "archive"))
    # ---- reproduction inputs (round-6 P0-R2: without these the chain
    # crashed at p2_generalization / witness_upgrade on any fresh host)
    for split in DATA_SPLITS:
        dst = os.path.join(snap, "data", "splatc_gates", split)
        os.makedirs(dst)
        shutil.copy(os.path.join(BASE, "data", "splatc_gates", split,
                                 "manifest.json"), dst)
    shutil.copytree(os.path.join(BASE, "outputs", "oracle_records"),
                    os.path.join(snap, "outputs", "oracle_records"))
    os.makedirs(os.path.join(snap, "results", "manifests"))
    shutil.copy(os.path.join(BASE, "results", "manifests",
                             "split_seals.json"),
                os.path.join(snap, "results", "manifests"))
    # ---- reproduction entrypoints + environment lock
    shutil.copy(os.path.join(BASE, "experiments", "compiler",
                             "reproduce.sh"), snap)
    shutil.copy(os.path.join(BASE, "experiments", "compiler",
                             "environment.yml"), snap)
    # round-7: measured full freeze from the locked overlay (evidence for
    # the scoped environment claim; REQUIRED — capture it before building)
    shutil.copy(os.path.join(BASE, "experiments", "compiler",
                             "environment_freeze.txt"), snap)
    shutil.copy(os.path.join(BASE, "experiments", "compiler",
                             "validate_package.py"), PKG)
    shutil.copy(os.path.join(BASE, "experiments", "compiler",
                             "compare_reproduction.py"), PKG)
    # round-7 two-pass release: pass 1 builds the CONTENT zip (no
    # receipt), the layer-2 acceptance of that zip writes the receipt,
    # pass 2 (--with-receipt) adds ONLY the receipt.  Never include a
    # receipt implicitly — a stale one from an earlier round would be
    # baked into fresh content and fail its own manifest closure check.
    if "--with-receipt" in sys.argv:
        rc = os.path.join(BASE, "outputs",
                          "cleanroom_reproduction_receipt.json")
        shutil.copy(rc, PKG)
    with open(os.path.join(snap, "environment.txt"), "w") as f:
        f.write("see environment.yml (scoped lock: interpreter + numeric "
                "stack pinned; NOT a conda-lock-level full freeze — "
                "measured overlay freeze in environment_freeze.txt).  "
                "Canonical + clean-room numeric chain: python 3.11.15 / "
                "numpy 1.26.4 / scipy 1.10.1, Linux x86_64.\n"
                "no VCS in working tree: identity = src_tree_sha256 over "
                "src/*.py — validate_package.py RECOMPUTES it from this "
                "snapshot (never trusts JSON strings).\n"
                "release gate: (1) validate_package.py <pkg>  ->  (2) from "
                "src_snapshot/: bash reproduce.sh  ->  (3) "
                "compare_reproduction.py <pkg> <src_snapshot>.  All three "
                "must exit 0.\n")
    # MANIFEST.sha256 over everything except itself
    lines = []
    for root, dirs, files in os.walk(PKG):
        dirs[:] = [d for d in dirs if d != "__pycache__"]
        for f in sorted(files):
            if f == "MANIFEST.sha256":
                continue
            p = os.path.join(root, f)
            lines.append(f"{sha(p)}  {os.path.relpath(p, PKG)}")
    with open(os.path.join(PKG, "MANIFEST.sha256"), "w") as f:
        f.write("\n".join(sorted(lines, key=lambda s: s.split(None, 1)[1]))
                + "\n")
    # release gate, layer 1
    r = subprocess.run([sys.executable,
                        os.path.join(PKG, "validate_package.py"), PKG],
                       capture_output=True, text=True)
    print(r.stdout, r.stderr)
    if r.returncode != 0:
        print("BUILD ABORTED: layer-1 validation failed; no zip produced")
        return 1
    zpath = os.path.join(BASE, "outputs", "round4_final_package")
    shutil.make_archive(zpath, "zip", os.path.join(BASE, "outputs"),
                        "round4_final_package")
    # round-8: detached checksum NEXT TO the zip (the receipt inside the
    # zip records the CONTENT zip's hash, which by construction differs
    # from the final zip — the detached file is what a reviewer compares)
    zsha = sha(zpath + ".zip")
    with open(zpath + ".zip.sha256", "w") as f:
        f.write(f"{zsha}  round4_final_package.zip\n")
    print("->", zpath + ".zip")
    print("   sha256:", zsha, "(also in .zip.sha256 next to it)")
    if "--with-receipt" in sys.argv:
        print("layer 1 PASS incl. receipt closure — releasable if the "
              "receipt's acceptance run (layer 2) exited 0/0/0.")
    else:
        print("NOT RELEASABLE YET: layer 2 (clean-room reproduce + "
              "compare) must also exit 0, then rebuild with "
              "--with-receipt — see environment.txt in the snapshot.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
