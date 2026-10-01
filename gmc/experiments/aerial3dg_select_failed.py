"""Pick a distance-stratified sample of "failed" cylinder pairs from the G2 5000-pair run, for
manual visual inspection (routes_<name>.png via aerial3dg_viz.py, one figure per pair showing
both robots' route/certificate + straight-line evidence).

Two groups (user-chosen scope, not all of UNKNOWN/UNREACHABLE):
* gap    cylinder UNKNOWN, reason=safe_graph_disconnected_possible_connected (the u~-5.6 captured
         structure the octree can't certify with a single pair's leaf certificate; the report's
         only acknowledged method-incompleteness bucket, 1587/5000).
* unreach cylinder UNREACHABLE (certified; all cross the lamp+partition, 2928/5000) -- included as
         a sanity check on the cut certificates, not because the cause is unknown.

Sampling: sort each group by dist_m, take an evenly-spaced (systematic) subsample of size --n-gap /
--n-unreach so the picks span the full distance range rather than clustering near the mode.
Deterministic; no randomness.

Usage (from gmc/, PYTHONPATH=src:experiments):
    python experiments/aerial3dg_select_failed.py --n-gap 80 --n-unreach 40 \
        --out configs/aerial3dg/g4_failed_cylinder_pairs.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

G2 = Path("results/aerial3dg/g2/runs/cylinder")
GAP_REASON = "safe_graph_disconnected_possible_connected"


def load_rows() -> dict:
    rows = {}
    for f in sorted(G2.glob("task_*.jsonl")):
        for line in f.read_text().splitlines():
            if line.strip():
                d = json.loads(line)
                rows[d["index"]] = d
    return rows


def systematic_sample(items: list, n: int) -> list:
    """Evenly-spaced picks by rank (after sorting by dist_m), not random: spans the full range."""
    if n >= len(items):
        return items
    step = len(items) / n
    return [items[int(i * step)] for i in range(n)]


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--n-gap", type=int, default=80, help="sample size, UNKNOWN/safe_graph_disconnected group")
    p.add_argument("--n-unreach", type=int, default=40, help="sample size, UNREACHABLE (certified) group")
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)

    rows = load_rows()
    gap = sorted((r for r in rows.values() if r["status"] == "UNKNOWN" and r["reason"] == GAP_REASON),
                 key=lambda r: r["dist_m"])
    unreach = sorted((r for r in rows.values() if r["status"] == "UNREACHABLE"), key=lambda r: r["dist_m"])
    print(f"pool sizes: gap(UNKNOWN/{GAP_REASON})={len(gap)}  unreach(UNREACHABLE)={len(unreach)}  "
          f"of {len(rows)} cylinder rows", flush=True)

    picks = ([("F_gap", r) for r in systematic_sample(gap, a.n_gap)] +
             [("F_unreach", r) for r in systematic_sample(unreach, a.n_unreach)])
    pairs = []
    for prefix, r in picks:
        name = f"{prefix}_{r['index']:04d}"
        pairs.append({
            "name": name, "role": f"cylinder {r['status']}/{r['reason']}, dist {r['dist_m']:.2f} m, "
                                   f"G2 index {r['index']} ({r['pair_id']})",
            "g2_index": r["index"], "pair_id": r["pair_id"], "start_uv": r["start_uv"], "goal_uv": r["goal_uv"],
            "dist_m": r["dist_m"], "g2_cylinder_status": r["status"], "g2_cylinder_reason": r["reason"],
            "g2_cut_pairs_by_role": r.get("cut_pairs_by_role")})
    doc = {"box_route_uv": [-9.0, -0.35, 3.7, 2.75],
           "selection": "Manually-requested gallery of 'failed' cylinder pairs from the G2 5000-pair run "
                        "(gmc/results/aerial3dg/g2/runs/cylinder); scope = UNKNOWN/safe_graph_disconnected_possible_connected "
                        "(u~-5.6 gap, method incompleteness) + UNREACHABLE (certified, all cross the lamp). Systematic "
                        "sample by dist_m rank within each group, not random, not all 4515 candidates.",
           "n_gap": len(systematic_sample(gap, a.n_gap)), "n_unreach": len(systematic_sample(unreach, a.n_unreach)),
           "pool_sizes": {"gap": len(gap), "unreach": len(unreach), "total_cylinder_rows": len(rows)},
           "pairs": pairs}
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(doc, indent=1) + "\n")
    print(f"wrote {len(pairs)} pairs -> {a.out}", flush=True)


if __name__ == "__main__":
    main()
