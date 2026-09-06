"""Machine validation of a review package — layer 1 of the two-layer
release gate (round-6 review P0-R2).

History: human prose-vs-file checking failed twice (rounds 4, 5); the v1
validator then passed a package whose reproduction CRASHED (round-6) —
it verified internal consistency but never checked that the snapshot
could actually run (missing data manifests + oracle records), and it
trusted provenance hash STRINGS instead of recomputing them.  Hence:

  Layer 1 (this script, static):
   1. MANIFEST.sha256 mismatch or missing file
   2. any JSON that does not parse
   3. unresolved oracle truth carrying reachable != null
   4. figure sidecar provenance != its source JSON provenance
   5. canonical tables share ONE src_tree_sha256
   6. banned terminology in any .md
   7. key report numbers absent from their source JSONs
   8. RECOMPUTED src_snapshot/src tree hash == every provenance claim
      (tables, amendments, figure sidecars) — hashes are recomputed,
      never trusted from JSON strings
   9. RECOMPUTED script hash of each generator in the snapshot ==
      each table's / amendment's provenance script_sha256
  10. input-dependency preflight: every script reproduce.sh invokes
      exists in the snapshot; data manifests (dev+validation) and one
      oracle record per manifest episode are shipped; environment.yml
      and compare_reproduction.py present; the SEALED blind manifest
      must NOT be in the package
  Layer 2 (separate, dynamic): clean-room reproduce.sh from the
  snapshot, then compare_reproduction.py against the packaged tables.
  A package is releasable only if BOTH layers exit 0.

Usage:  python validate_package.py <package_dir>
Exit 0 = layer 1 pass; exit 1 = FAIL (reasons printed).
"""
import hashlib
import json
import os
import re
import sys

BANNED = ["1000×", "1000x", "false-unreachable", "independent checker",
          "独立 checker", "residual 已证明", "residual 已建立",
          "metadata-free",          # round-6: G1-frame priors exist
          "互为必要",                # round-6: implementation-level only
          "Gate B-G1 内部 PASS",    # round-6: mechanism PASS / residual HOLD
          "P4a kill test 待做",     # round-8: it RAN and the wide case
          #                           failed — "pending" hid a negative
          "精确环境锁",              # round-8: the lock is scoped
          #                           (interpreter+numeric stack), not a
          #                           conda-lock-level full freeze
          "P4a.1 semantic invariance: PASS",   # round-10: WITHDRAWN —
          #                           may only appear via the withdrawal
          #                           marker, never as a live claim
          "全程无 x 阈值",           # round-10: the witness generation
          #                           is causally x-threshold-dependent
          "0/81",                   # round-10: wrong denominator
          "«"]   # unfilled placeholder marker must never ship

FIG_SOURCES = {
    "gamma_delta.png": "tables/gamma_delta.json",
    "p3_matched_budget.png": "tables/p3_matched_budget.json",
    "p36_fairness.png": "tables/p36_fairness.json",
    "p37_matched.png": "tables/p37_matched.json",
    "ridge_diag_w0.505.png": "tables/ridge_stations.json",
    "ridge_diag_w0.490.png": "tables/ridge_stations.json",
}


def fail(msgs, m):
    msgs.append("FAIL: " + m)


def sha_file(path, n=12):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()[:n]


def sha_tree(root, n=12):
    # MUST mirror prov._sha_tree exactly (filename + bytes of *.py, sorted)
    h = hashlib.sha256()
    for dirpath, dirnames, filenames in sorted(os.walk(root)):
        dirnames.sort()
        if "__pycache__" in dirpath:
            continue
        for fname in sorted(filenames):
            if fname.endswith(".py"):
                h.update(fname.encode())
                with open(os.path.join(dirpath, fname), "rb") as f:
                    h.update(f.read())
    return h.hexdigest()[:n]


def sha_bundle(base, roots=("src", "experiments/compiler"), n=12):
    # MUST mirror prov._sha_bundle exactly (relative posix path + bytes,
    # excluding archive/ and __pycache__)
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


def main(pkg):
    msgs = []
    ok_n = 0
    snap = os.path.join(pkg, "src_snapshot")

    # 1. checksums
    man = os.path.join(pkg, "MANIFEST.sha256")
    if not os.path.exists(man):
        fail(msgs, "MANIFEST.sha256 missing")
    else:
        listed = set()
        for line in open(man):
            line = line.strip()
            if not line:
                continue
            want, path = line.split(None, 1)
            path = path.lstrip("*").strip()
            listed.add(path)
            fp = os.path.join(pkg, path)
            if not os.path.exists(fp):
                fail(msgs, f"missing file {path}")
                continue
            got = hashlib.sha256(open(fp, "rb").read()).hexdigest()
            if got != want:
                fail(msgs, f"sha256 mismatch {path}")
            else:
                ok_n += 1
        # every real file must be listed (no unlisted content can ship)
        for root, dirs, files in os.walk(pkg):
            dirs[:] = [d for d in dirs if d != "__pycache__"]
            for f in files:
                rel = os.path.relpath(os.path.join(root, f), pkg)
                if rel != "MANIFEST.sha256" and rel not in listed:
                    fail(msgs, f"file not in MANIFEST: {rel}")

    # 2. JSON parse + collect provenance
    provs = {}
    for root, _, files in os.walk(pkg):
        for f in files:
            if f.endswith(".json"):
                p = os.path.join(root, f)
                rel = os.path.relpath(p, pkg)
                try:
                    d = json.load(open(p))
                except Exception as e:
                    fail(msgs, f"JSON parse error {rel}: {e}")
                    continue
                pv = None
                if isinstance(d, dict):
                    pv = d.get("provenance") or d.get("_provenance")
                if pv:
                    provs[rel] = pv

    # 3. unresolved truth schema
    p2p = os.path.join(pkg, "tables", "p2_generalization.json")
    if os.path.exists(p2p):
        d = json.load(open(p2p))
        for r in d.get("rows", []):
            t = r.get("truth")
            if t and t.get("status") != "converged" \
                    and t.get("reachable") is not None:
                fail(msgs, f"unresolved truth with reachable="
                     f"{t['reachable']} in {r.get('query_id')}")

    # 4. figure sidecars vs source JSON
    for fig, srcrel in FIG_SOURCES.items():
        side = os.path.join(pkg, "figures", fig + ".prov.json")
        srcp = os.path.join(pkg, srcrel)
        if not os.path.exists(side):
            fail(msgs, f"figure sidecar missing for {fig}")
            continue
        if not os.path.exists(srcp):
            fail(msgs, f"figure source JSON missing {srcrel}")
            continue
        sp = json.load(open(side))["source_provenance"][0]
        d = json.load(open(srcp))
        dp = d.get("provenance") or d.get("_provenance") or {}
        for k in ("script_sha256", "src_tree_sha256", "generated_at"):
            if sp.get(k) != dp.get(k):
                fail(msgs, f"{fig}: sidecar {k}={sp.get(k)} != "
                     f"source JSON {dp.get(k)}")

    # 5. one canonical src tree across tables/
    trees = {rel: pv.get("src_tree_sha256")
             for rel, pv in provs.items() if rel.startswith("tables/")}
    if len(set(trees.values())) > 1:
        fail(msgs, f"multiple src trees in canonical tables: {trees}")

    # 6. banned terminology — in prose AND in machine provenance.
    #    Round-7: the report had withdrawn a claim while canonical
    #    provenance method_arm strings still asserted it, and the v3
    #    validator only scanned .md — so every provenance block
    #    (tables, amendments, figure sidecars) is now scanned too.
    for root, _, files in os.walk(pkg):
        for f in files:
            if f.endswith(".md"):
                txt = open(os.path.join(root, f), encoding="utf-8").read()
                for b in BANNED:
                    if b in txt:
                        fail(msgs, f"banned term '{b}' in {f}")
    prov_texts = {rel: json.dumps(pv, ensure_ascii=False)
                  for rel, pv in provs.items()}
    for root, _, files in os.walk(os.path.join(pkg, "figures")):
        for f in files:
            if f.endswith(".prov.json"):
                d = json.load(open(os.path.join(root, f)))
                prov_texts[f"figures/{f}"] = json.dumps(
                    d.get("source_provenance", []), ensure_ascii=False)
    for rel, txt in prov_texts.items():
        for b in BANNED:
            if b in txt:
                fail(msgs, f"banned term '{b}' in provenance of {rel}")

    # 7. key report numbers exist in source JSONs.  Round-8: the single
    #    "final" report is retired; the package ships TWO documents
    #    (Gate B evidence, frozen + sprint-C P4a) and both must be there
    rep = ""
    for cand in ("gate_b_evidence_report.md", "sprint_c_p4a_report.md",
                 "p38_connector_report.md"):
        p = os.path.join(pkg, cand)
        if not os.path.exists(p):
            fail(msgs, f"{cand} missing")
        else:
            rep += open(p, encoding="utf-8").read()
    if not rep:
        rep = None
    if rep is not None:
        p3 = json.load(open(os.path.join(pkg, "tables",
                                         "p3_matched_budget.json")))
        for line in p3["summary"]:
            if line["continuation_total"] and \
                    f"{line['continuation_total']:,}" not in rep \
                    and str(line["continuation_total"]) not in rep:
                fail(msgs, f"report missing continuation total "
                     f"{line['continuation_total']} (w={line['w']})")
        scp = os.path.join(pkg, "tables", "ridge_scalar_ablation.json")
        if os.path.exists(scp):
            sc = json.load(open(scp))
            n_open_cert = sum(
                1 for r in sc["rows"]
                if r["w"] > 0.5 and r.get("status") == "CERTIFIED_REACHABLE")
            if n_open_cert != 0:
                fail(msgs, "scalar ablation has certified open cases; "
                     "report claims 0/6 per K")
            if "0/6" not in rep:
                fail(msgs, "report missing the 0/6 scalar framing")

    # 7b (round-8). negative results cannot hide in JSON: the frozen
    #    round-7 FAIL record must ship, actually record the failure,
    #    and be disclosed VERBATIM in prose; the current P4a.1 machine
    #    verdict must be quoted in prose, per-case FAILs included
    frec = os.path.join(pkg, "records",
                        "p4a_rigid_charts_round7_15k_FAIL.json")
    if not os.path.exists(frec):
        fail(msgs, "records/p4a_rigid_charts_round7_15k_FAIL.json missing")
    else:
        d = json.load(open(frec))
        wide = next((c for c in d.get("cases", [])
                     if c.get("case", {}).get("w") == 1.1), None)
        if not wide or wide["case"].get("major_chart_count_invariant") \
                is not False:
            fail(msgs, "frozen FAIL record does not record the wide-door "
                 "count-invariance failure — wrong file?")
    if rep is not None and \
            "P4a chart-layer rigid kill (15k fixed budget): FAIL" not in rep:
        fail(msgs, "report does not disclose the round-7 P4a FAIL "
             "verbatim ('P4a chart-layer rigid kill (15k fixed budget): "
             "FAIL')")
    # round-10: the round-9 machine PASS is WITHDRAWN — the frozen
    # table must ship in records/ (with the PASS it wrongly recorded)
    # and the report must carry the withdrawal verbatim
    w9 = os.path.join(pkg, "records",
                      "p4a_semantic_invariance_round9_WITHDRAWN.json")
    if not os.path.exists(w9):
        fail(msgs, "records/p4a_semantic_invariance_round9_WITHDRAWN"
             ".json missing")
    else:
        d9 = json.load(open(w9))
        if d9.get("overall", {}).get("semantic_invariance") != "PASS":
            fail(msgs, "round-9 withdrawn record does not contain the "
                 "PASS that was withdrawn — wrong file?")
    if rep is not None and \
            "round-9 P4a.1 machine PASS: WITHDRAWN (FAIL-CORRECTABLE)" \
            not in rep:
        fail(msgs, "report does not carry the round-9 withdrawal "
             "verbatim ('round-9 P4a.1 machine PASS: WITHDRAWN "
             "(FAIL-CORRECTABLE)')")
    semp = os.path.join(pkg, "tables", "p4a2_semantic.json")
    if not os.path.exists(semp):
        fail(msgs, "tables/p4a2_semantic.json missing")
    elif rep is not None:
        sem = json.load(open(semp))
        verdict = sem.get("overall", {}).get(
            "chart_layer_reachability_consistency")
        if f"P4a.2 chart-layer reachability consistency: {verdict}" \
                not in rep:
            fail(msgs, f"report does not quote the machine verdict "
                 f"'P4a.2 chart-layer reachability consistency: "
                 f"{verdict}'")
        for c in sem.get("cases", []):
            if not c.get("case_pass") and \
                    f"P4a.2 case {c['case']}: FAIL" not in rep:
                fail(msgs, f"case '{c['case']}' failed in the table but "
                     f"the report does not say 'P4a.2 case "
                     f"{c['case']}: FAIL'")
    v3p = os.path.join(pkg, "tables", "p4a3_validation.json")
    if not os.path.exists(v3p):
        fail(msgs, "tables/p4a3_validation.json missing")
    elif rep is not None:
        v3 = json.load(open(v3p))
        verdict = v3.get("overall", {}).get("sealed_validation")
        if f"P4a.3 sealed validation: {verdict}" not in rep:
            fail(msgs, f"report does not quote the machine verdict "
                 f"'P4a.3 sealed validation: {verdict}'")

    # 7c (P3.8). the kill-condition outcome and the negative control are
    #    machine verdicts: the report must quote them exactly, and the
    #    continuation reference it argues against must be the measured
    #    number from the same table
    p38p = os.path.join(pkg, "tables", "p38_connector.json")
    if not os.path.exists(p38p):
        fail(msgs, "tables/p38_connector.json missing")
    elif rep is not None:
        p38 = json.load(open(p38p))
        ov = p38.get("overall", {})
        kv = ov.get("p38_kill_condition")
        if f"P3.8 kill condition: {kv}" not in rep:
            fail(msgs, f"report does not quote the machine verdict "
                 f"'P3.8 kill condition: {kv}'")
        neg_ok = bool(ov.get("negative_control_all_refused")) \
            and ov.get("false_claims") == 0
        nm = "PASS" if neg_ok else "FAIL"
        if f"P3.8 negative control (plugged, all arms refuse): {nm}" \
                not in rep:
            fail(msgs, f"report does not quote 'P3.8 negative control "
                 f"(plugged, all arms refuse): {nm}'")
        cont = p38.get("summary", {}).get("thin_0.505", {}).get(
            "contact_continuation", {})
        ref_nq = cont.get("median_nq_success")
        if ref_nq and f"{int(ref_nq):,}" not in rep \
                and str(int(ref_nq)) not in rep:
            fail(msgs, f"report missing the P3.8 continuation reference "
                 f"query count {int(ref_nq)}")
        for arm, e in p38.get("kill", {}).get("arms", {}).items():
            if e.get("within_2x_any_currency") and \
                    f"P3.8 {arm}: WITHIN 2x" not in rep:
                fail(msgs, f"arm '{arm}' is within 2x in the table but "
                     f"the report does not say 'P3.8 {arm}: WITHIN 2x'")

    # 8. recompute the snapshot src tree hash; every provenance claim in
    #    the package (tables, amendments, sidecars) must equal it
    snap_src = os.path.join(snap, "src")
    if not os.path.isdir(snap_src):
        fail(msgs, "src_snapshot/src missing — tree hash unverifiable")
    else:
        tree = sha_tree(snap_src)
        # records/ holds FROZEN negative-result artifacts from earlier
        # trees; their identity is the recorded provenance INSIDE them
        # (7b checks content), so they are exempt from current-tree
        # equality — but not from the banned-term scan above
        claims = {rel: pv for rel, pv in provs.items()
                  if not rel.startswith("records/")
                  and not rel.startswith("records" + os.sep)}
        for root, _, files in os.walk(os.path.join(pkg, "figures")):
            for f in files:
                if f.endswith(".prov.json"):
                    for pv in json.load(
                            open(os.path.join(root, f)))["source_provenance"]:
                        claims[f"figures/{f}"] = pv
        bad = {rel: pv.get("src_tree_sha256") for rel, pv in claims.items()
               if pv.get("src_tree_sha256") != tree}
        if bad:
            fail(msgs, f"recomputed snapshot tree {tree} != provenance "
                 f"claims: {bad}")
        else:
            print(f"recomputed src tree: {tree} "
                  f"({len(claims)} provenance claims match)")
        # round-7: full executable identity — the bundle covers the
        # experiment scripts' transitive imports, which src_tree misses
        bundle = sha_bundle(snap)
        badb = {rel: pv.get("code_bundle_sha256")
                for rel, pv in claims.items()
                if pv.get("code_bundle_sha256") != bundle}
        if badb:
            fail(msgs, f"recomputed code bundle {bundle} != provenance "
                 f"claims: {badb}")
        else:
            print(f"recomputed code bundle: {bundle} "
                  f"({len(claims)} provenance claims match)")

    # 9. recompute each generator script's hash from the snapshot
    scripts_dir = os.path.join(snap, "experiments", "compiler")
    for rel, pv in sorted(provs.items()):
        if not (rel.startswith("tables/")
                or rel.startswith("oracle_amendments/")):
            continue
        script = pv.get("script")
        claimed = pv.get("script_sha256")
        sp = os.path.join(scripts_dir, script) if script else None
        if not script or not os.path.exists(sp):
            fail(msgs, f"{rel}: generator {script} not in snapshot")
        elif sha_file(sp) != claimed:
            fail(msgs, f"{rel}: snapshot {script} hash {sha_file(sp)} != "
                 f"provenance claim {claimed} (stale table or edited "
                 f"script — regenerate)")

    # 10. input-dependency preflight (the round-6 crash class)
    repro = os.path.join(snap, "reproduce.sh")
    if not os.path.exists(repro):
        fail(msgs, "src_snapshot/reproduce.sh missing")
    else:
        for m in re.finditer(r"^\s*python\s+(-m\s+\S+\s+)?(\S+\.py)",
                             open(repro).read(), re.M):
            sp = os.path.join(snap, m.group(2))
            if not os.path.exists(sp):
                fail(msgs, f"reproduce.sh invokes {m.group(2)} — "
                     f"not in snapshot")
    n_eps = 0
    for split in ("dev", "validation"):
        mp = os.path.join(snap, "data", "splatc_gates", split,
                          "manifest.json")
        if not os.path.exists(mp):
            fail(msgs, f"data manifest missing: {split}")
            continue
        for e in json.load(open(mp))["episodes"]:
            n_eps += 1
            qid = e["query_id"]
            if not os.path.exists(os.path.join(
                    snap, "outputs", "oracle_records", f"{qid}.json")):
                fail(msgs, f"oracle record missing for {qid}")
    if n_eps:
        print(f"input preflight: {n_eps} episodes with oracle records")
    for req in (os.path.join(snap, "environment.yml"),
                os.path.join(pkg, "compare_reproduction.py"),
                os.path.join(pkg, "validate_package.py")):
        if not os.path.exists(req):
            fail(msgs, f"required file missing: {os.path.relpath(req, pkg)}")
    blind = os.path.join(snap, "data", "splatc_gates", "blind")
    if os.path.exists(blind):
        fail(msgs, "SEALED blind split is inside the package — must not ship")
    # clean-room receipt (round-7): OPTIONAL at layer 1 (it is produced BY
    # the layer-2 acceptance run and added to the final zip afterwards);
    # if present it must parse and carry the required fields
    rc = os.path.join(pkg, "cleanroom_reproduction_receipt.json")
    if os.path.exists(rc):
        try:
            r = json.load(open(rc))
            need = {"content_manifest_sha256", "code_bundle_sha256",
                    "verified_content_zip_sha256",   # round-8 rename
                    "host", "environment_freeze_sha256",
                    "started_utc", "finished_utc",
                    "validate_exit", "reproduce_exit", "compare_exit",
                    "canonical_table_sha256"}
            miss = need - set(r)
            if miss:
                fail(msgs, f"receipt missing fields: {sorted(miss)}")
            elif not (r["validate_exit"] == r["reproduce_exit"]
                      == r["compare_exit"] == 0):
                fail(msgs, "receipt records a non-zero gate exit code")
            else:
                # closure: this package's MANIFEST minus the receipt's own
                # line must hash to the CONTENT manifest the receipt
                # verified (i.e., the final zip adds only the receipt)
                lines = [ln for ln in open(man).read().splitlines()
                         if not ln.endswith(
                             " cleanroom_reproduction_receipt.json")]
                content = hashlib.sha256(
                    ("\n".join(lines) + "\n").encode()).hexdigest()
                if content != r["content_manifest_sha256"]:
                    fail(msgs, f"receipt verified content manifest "
                         f"{r['content_manifest_sha256'][:12]} but this "
                         f"package's non-receipt content hashes to "
                         f"{content[:12]} — receipt does not belong to "
                         f"this content")
        except Exception as e:
            fail(msgs, f"receipt unreadable: {e}")

    print(f"checksums verified: {ok_n}")
    if msgs:
        print("\n".join(msgs))
        print(f"\nPACKAGE: FAIL ({len(msgs)} problem(s))")
        return 1
    print("PACKAGE: PASS (layer 1; layer 2 = clean-room reproduce + "
          "compare_reproduction.py)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "."))
