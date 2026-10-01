# G4: visual gallery of failed cylinder pairs (G2 5000-pair run)

Purpose: manual inspection of *why* the cylinder fails on the G2 run, so the failures can be classified and the
"is it the compile box or the scene?" question can be answered by eye. Not part of the G1-G3 deliverables.

- `gallery/routes_F_gap_*.png` (80): cylinder UNKNOWN, reason `safe_graph_disconnected_possible_connected`
  (1587 in G2; every one straddles the captured structure at u~-5.6). Each figure shows both robots' route or
  certificate plus the straight-line evidence.
- `gallery/routes_F_unreach_*.png` (40): cylinder certified UNREACHABLE (`possible_space_cut`, 2928 in G2; all
  cross the lamp+bulkhead at u~-0.85). Included as a sanity check of the cut certificates.
- `gallery/viz_manifest.json`: inputs and hashes for the figures.
- Pair selection: `gmc/experiments/aerial3dg_select_failed.py` (distance-stratified, deterministic, systematic
  sample) -> `gmc/configs/aerial3dg/g4_failed_cylinder_pairs.json`. Rendered with `aerial3dg_viz.py`.

Status: figures generated 2026-09-28 in an interactive session; not yet reviewed in a written analysis. The
follow-up classification is branch `aerial3dg-fail` (docs/aerial3dg_failures.md, when finished).
