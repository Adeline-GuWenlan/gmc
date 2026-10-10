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
