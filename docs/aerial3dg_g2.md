# aerial3d-ground G2: the 5000-pair run, one compile per robot

Stage G2 of the aerial3d-ground chain, 2026-09-28. Machine-readable version:
`gmc/results/aerial3dg/g2_handoff.json`. Code: `gmc/experiments/aerial3dg_batch.py` (sample / task / collect /
plot), tests `gmc/tests/unit/test_aerial3dg_batch.py` (6 passed), sbatch `gmc/hpc/aerial3dg/g2_{run,task}.sbatch`.

## Result

**5000/5000 pairs answered for both robots. 0 TIMEOUT. Nothing is still running.**

| robot | REACHABLE | UNREACHABLE (certified) | UNKNOWN | query median / mean / p95 / max (s) |
|---|---|---|---|---|
| sweeper  | 4699 | 0 | 301 | 0.65 / 1.12 / 3.66 / 18.1 |
| cylinder | 73 | 2928 | 1999 | 0.05 / 0.105 / 0.18 / 3.9 |

Paired on the same pair (sweeper status | cylinder status):

| pairs | sweeper | cylinder |
|---|---|---|
| 2783 | REACHABLE | UNREACHABLE (possible-space cut) |
| 1843 | REACHABLE | UNKNOWN |
| 73 | REACHABLE | REACHABLE |
| 145 | UNKNOWN | UNREACHABLE |
| 156 | UNKNOWN | UNKNOWN |

- **Morphology contrast, at scale.** On 2783 pairs the sweeper threads under the lamp while the cylinder is certified
  UNREACHABLE. Every cylinder UNREACHABLE is a pair crossing the lamp at u≈-0.85, where the lamp and bulkhead span
  the corridor above 1.07 m. On the 73 pairs both robots reach, the cylinder's detour ratio averages 1.052 (the
  sweeper's is 1.032 over its 4699 routes). The "cylinder detours / sweeper threads" pattern stays rare, as G1 found.
- **Cylinder UNKNOWN (1999):**
  - 1587 are `safe_graph_disconnected_possible_connected`. All of them cross the captured structure at u≈-5.6
    (checked in full on task 0, 168/168). G1's 0.1 m gs3d point map has no free cylinder centre anywhere in the
    u = -5.6 and -5.5 columns. So these pairs are very likely unreachable, but not certified: a free window
    narrower than 0.1 m could be missed. G1 showed that halving the octree leaf changes no verdict.
  - 412 are endpoints the oracle calls free but the 5 cm octree does not certify.
- **Sweeper UNKNOWN (301):**
  - 239 are endpoints not certified free at leaf resolution.
  - 62 are `shared_replay_failed`: the own verifier certified the route and the gs3d replay was more conservative.
  - Every one of the 4699 REACHABLE routes passed both the own verifier and the shared gs3d replay (min clearance >= margin).

`gmc/results/aerial3dg/g2/g2_overview.png` (inspected) shows endpoint coverage, the distance histogram, outcome
per robot, query time, and detour ratio.

## The sampler (Task 1)

- **Box.** Uniform over G1's extended-corridor box u[-9, 3.7] v[-0.35, 2.75] (route frame). Continuous draws are
  rounded to 0.1 mm.
- **Rules.** Straight-line xy distance >= 3 m (so start ≠ goal). Both endpoints must be free by the gs3d oracle
  (margin 0.001, `free` + `continuous_bound`) for each robot at its own z_c.
- **Rejection.** A draw is rejected whole, so pairs are i.i.d. and any contiguous array slice or prefix is an
  unbiased sub-sample.
- **Numbers.** Seed 20260928, 75,460 draws. Rejections: 28,406 for distance, 36,095 for a sweeper-blocked
  endpoint, 5,959 for a cylinder-blocked endpoint. Distance 3.0 / 5.33 / 11.6 m (min / median / max). 3.5 min
  on 1 CPU.
- **Uniform, not reachability-biased.** The planner's certified UNREACHABLE is a result in its own right. A
  connectivity witness, as ground5k used, would remove exactly the lamp-crossing pairs that show the contrast.
  So the rates above are reported as they fall.
- **One shared list.** Every pair gives a paired comparison between the robots. The cylinder's free set is
  contained in the sweeper's (G1 map), so the list is exactly the cylinder's own endpoint set. For the sweeper it
  excludes the sweeper-only lamp zone and pockets as endpoints; routes still cross them.

## Compile once (Task 2)

- **Zero compiles in G2.** The one compile per robot is G1's demo compile on the same box: jobs 18704511 and
  18704512 at cd18230, sweeper 26.1 s, cylinder 70.7 s. `gmc/src` and the runner have not changed since, so it
  is byte-identical to a fresh compile.
- **Persist and reload.** Each array task calls `load_compiled` once on the `.a3c` (180 MB sweeper, 387 MB
  cylinder, SHA-256 checked). Loads take 0.73 s and 1.06 s; task RSS is 0.65 GB and 1.06 GB. The task then
  answers its 500 pairs.
- **Proof per task.** Each `task_??.summary.json` carries a `compile_once_proof`:
  - a counter wrapped around `gmc.aerial3d.api.compile_complex` reads 0;
  - one load;
  - compile records unchanged by the queries;
  - one compile_id on all rows;
  - no compile stage inside any query call.

  All of this holds for all 20 tasks.
- **Checkpointing.** Each pair is one fsync'd JSONL line, and a restarted task skips answered pairs. A torn last
  line is cut off; a unit test covers this. Results were committed and pushed as tasks completed:
  - 39d4168: pairs + cylinder task 0
  - 373c146: cylinder 1-9
  - 9c9fa31: sweeper 0-4
  - f938414: sweeper 5-8
  - 3d90de9: sweeper 9 + aggregate
- **Sizing.** The cpu_short QOS is 32 CPU / 120 GB per user, and ground5k's throttled arrays hold about 112 GB. So
  every G2 job was 1 CPU and at most 2.2 GB, sized at 1.5× the task-0 probe MaxRSS.

## Cost vs G1's estimate

| | G1 central | G1 pessimistic | G1 p95 bound | actual |
|---|---|---|---|---|
| sweeper, CPU-h | 0.61 | 0.63 | 3.28 | 1.58 |
| cylinder, CPU-h | 0.38 | 0.92 | 2.01 | 0.17 |
| both, CPU-h | 1.00 | 1.55 | 5.29 | 1.74 (+0.06 sampler) |

Wall-clock was 34 min, from sampler start at 08:00:56Z to the last task end at 08:34:58Z.

- **The sweeper cost 2.6× the central estimate.** G1's screen pairs were short (median 3.57 m) and bucketed toward
  near-straight thread and lamp routes. Uniform pairs are longer (median 5.3 m) and have more corners (4.6 vs 2.9
  route vertices). Shortcut and tighten take 0.99 s of the 1.16 s REACHABLE mean (0.37 s in G1's screen), and
  their cost grows with route length and complexity. Even for 3-4 m pairs the mean is 0.77 s vs 0.17 s.
- **The cylinder cost 0.44× the central estimate.** 98.5% of uniform pairs are cheap: certified cuts take 0.035 s
  and UNKNOWN answers 0.15 s.
- **The total** lands between G1's pessimistic column and its p95 bound.
- **Recompiling per pair** would have added about 134 h of compile wall (5000 × (26.1 + 70.7) s), roughly 77× the
  whole run.
