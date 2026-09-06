"""Provenance stamps for experiment outputs (review round-3, P0).

No git in this working copy: identity = sha256 of the generating script +
sha256 over the src/splatc python tree, plus UTC timestamp, method arm,
checker and cost-accounting versions.  Every result JSON carries a
`provenance` block; every figure gets a footer stamp rendered from the
SAME JSON it plots — figures are never hand-picked.
"""
import hashlib
import os
from datetime import datetime, timezone

CHECKER = ("cert=certify_path_conservative(bubble, checker#2 metric_margin); "
           "diag profile=checker#2 only, billed c_diag")
ACCOUNTING = "pose-queries v3: side_rho=1 query; seed/track/densify/cert split"


def _sha_file(path, n=12):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()[:n]


def _sha_tree(root, n=12):
    h = hashlib.sha256()
    for dirpath, dirnames, filenames in sorted(os.walk(root)):
        dirnames.sort()
        if "__pycache__" in dirpath:
            continue
        for fname in sorted(filenames):
            if fname.endswith(".py"):
                p = os.path.join(dirpath, fname)
                h.update(fname.encode())
                with open(p, "rb") as f:
                    h.update(f.read())
    return h.hexdigest()[:n]


def _sha_bundle(base, roots=("src", "experiments/compiler"), n=12):
    """Round-7: full executable identity.  src_tree_sha256 misses the
    experiment scripts' transitive imports (gamma imports
    ridge_continuation; p37 imports p3_matched_budget -> p36_fairness),
    so editing an imported helper changed outputs without changing any
    recorded hash.  This bundle hashes every *.py under src/ AND
    experiments/compiler/ with its RELATIVE path (posix separators, so
    macOS and Linux agree), excluding archive/ (never imported) and
    __pycache__."""
    h = hashlib.sha256()
    for root in roots:
        top = os.path.join(base, root)
        for dirpath, dirnames, filenames in sorted(os.walk(top)):
            dirnames[:] = sorted(d for d in dirnames
                                 if d not in ("__pycache__", "archive"))
            for fname in sorted(filenames):
                if fname.endswith(".py"):
                    p = os.path.join(dirpath, fname)
                    rel = os.path.relpath(p, base).replace(os.sep, "/")
                    h.update(rel.encode())
                    with open(p, "rb") as f:
                        h.update(f.read())
    return h.hexdigest()[:n]


def make_provenance(script_path, arm, extra=None):
    script_path = os.path.abspath(script_path)
    base = os.path.normpath(os.path.join(
        os.path.dirname(script_path), "..", ".."))
    src = os.path.join(base, "src")
    prov = {
        "script": os.path.basename(script_path),
        "script_sha256": _sha_file(script_path),
        "src_tree_sha256": _sha_tree(src),
        "code_bundle_sha256": _sha_bundle(base),
        "method_arm": arm,
        "checker": CHECKER,
        "cost_accounting": ACCOUNTING,
        "generated_at": datetime.now(timezone.utc).isoformat(
            timespec="seconds"),
        "vcs": "none (rsync tree); identity via sha256 above",
    }
    if extra:
        prov.update(extra)
    return prov


def stamp_figure(fig, provs):
    """Footer stamp from the provenance blocks of the JSONs being plotted."""
    if isinstance(provs, dict):
        provs = [provs]
    parts = [f"{p.get('script','?')}@{p.get('script_sha256','?')}"
             f"[{p.get('method_arm','?')}] {p.get('generated_at','?')}"
             for p in provs]
    src = provs[0].get("src_tree_sha256", "?") if provs else "?"
    fig.text(0.005, 0.002,
             "prov: " + " | ".join(parts) + f" | src@{src}",
             fontsize=5.5, color="gray")


def write_sidecar(fig_path, provs):
    """Machine-checkable figure provenance (round-5 review §2.3): a
    <figure>.prov.json sidecar recording exactly the provenance embedded in
    the footer, so validate_package.py can compare figure vs source JSON
    without OCR.  Plot scripts MUST call this right after savefig."""
    import json
    if isinstance(provs, dict):
        provs = [provs]
    with open(fig_path + ".prov.json", "w") as f:
        json.dump({"figure": os.path.basename(fig_path),
                   "source_provenance": provs}, f, indent=1)
