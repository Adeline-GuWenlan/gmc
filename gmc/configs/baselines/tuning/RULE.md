# bl B1 tuning rule (written and committed before any tuning result existed)

Tuning set: plan §3.5 (`bl_harness.load_pairs("tuning")`: first 25 WWEST + 15 GAPW1 + 10 S of F3's pilot pairs,
disjoint from the F4 5000), both robots, through the unchanged harness (judge inline, 120 s limit).

Candidates: 8 per method (`tuning/splatnav/S*.json`, `tuning/foci/F*.json`), equal effort, one pass each, no
re-runs on the same pairs with hand edits in between.

Selection, per robot, from the same candidate list for both methods:
1. most SUCCESS on that robot's 50 tuning pairs;
2. tie: fewest CLAIMED_* (unsafe/unsound claims);
3. tie: lowest median algorithm_wall_s.
Ineligible: configurations that break a fairness rule of plan §3 -- SplatNav `S7_eps05_native1sigma` (obstacle at 1
sigma while the judge uses level 2 sigma: the plan requires the matched contract where the method exposes it) and
FOCI `F0_repo_zband` (the repo's z band (0, 1): the plan requires planning in the plane). Both are run only to measure
what the matched contract / plane constraint costs or gains, and are reported.

The selected configuration is frozen in `configs/baselines/<method>.json` and committed before the pilot.

## B2 addendum (PNO, cust_fields) — written and committed before any B2 tuning result

Same tuning set, same harness, same selection rule (1. most SUCCESS, 2. fewest CLAIMED_*, 3. lowest median
algorithm_wall_s, per robot), ≤ 10 candidates per method, one pass each.

- **PNO** (`tuning/pno/P*.json`, 8 candidates): model grid S (1024 / 2048 / 4096), published weights (PNO /
  PNOwPINN), heuristic erosion (4 = `heuristics.py` default / 1), shared-raster resolution (10 / 5 / 2.5 mm).
  All are zero-shot (no training); all are eligible (none breaks plan §3: the map is the shared conservative raster in
  every candidate, and PNO exposes no obstacle contract of its own).
- **cust_fields** (`tuning/cust_fields/C*.json`): obstacle construction (one convex squircle cover per map piece vs
  the chain star decomposition), cover tightness (`waste`, squareness `s`), endpoint snap radius, NF step `dt`. The
  candidate keeping the repo's hidden 5 cm obstacle pad (`native_pad: true`) is run for information and is
  **ineligible** (plan §3.1: where a method exposes a safety margin it is set to the judge's; our C-space map already
  contains r + margin).
