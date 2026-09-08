# Worklog — Test A on the real K2 3DGS scene (2026-09-08)

Scope: does the certified compiler run at all on real 3DGS-derived geometry,
and do its verdicts agree with an independent implementation on the same data?
This is the milestone the cloud audit's action 5 asked for ("real 3DGS before
any compression claim is written up"). It is a **plumbing and cross-check**
result. No planning or compression claim follows from it — see Caveats.

Code: `gmc/experiments/k2_scene.py` (loader), `gmc/experiments/k2_door_gate.py`
(experiment), `gmc/hpc/k2_door_gate.sbatch`. Results in `gmc/results/k2/`.

## Step 0 — the fisheye_gs overlay is not usable as a scene

`/scratch/wg2381/fisheye_gs/xgrid_indoor_1_k2_full_64g.ext3` is a 64 GB
apptainer overlay, not a readable scene directory. Mounted read-only against
`/share/apps/images/centos-7.9.2009.sif`:

- `df` inside reports **26 GB used**, so it does hold content;
- the only thing it adds at `/` is a single directory `datasets`, owned
  `nfsnobody:nfsnobody` mode `0750`;
- that directory is **not readable** through a read-only fuse mount, with or
  without `--fakeroot`, even though the container's gid list contains 65534.

Reading it would need a read-write mount, which modifies the image, so it was
not attempted. Test A therefore used the slab already extracted by the atlas,
`splatc_atlas/data/gs_scenes/k2/slab_2d.npz` (656,487 conditioned 2D splats).

A first version of the inventory script (`gmc/hpc/k2_overlay_inventory.sbatch`)
walked the container's bind-mounted host `/scratch` and listed other users'
directories. That was out of scope and is fixed: the script now runs with
`--contain`, so nothing outside the base image and the overlay is visible.

## What the real geometry actually looks like

| window | in box | after weight > 0.3 | dropped (not SPD) | used |
|---|---|---|---|---|
| door_A half=1.5 u | 17,314 | 4,566 | 0 | 4,566 |
| door_A half=3.0 u | 55,514 | 14,894 | 0 | 14,894 |

Support shape, which is the number that matters for everything downstream:

| window | semi-minor min | semi-axes p50 | axis ratio p50 | axis ratio max |
|---|---|---|---|---|
| half=1.5 | 8.95e-05 u | 0.0002 x 0.0361 u | 101 | 1.80e3 |
| half=3.0 | 8.91e-05 u | 0.0007 x 0.0395 u | 39.9 | 2.21e3 |

**Real 3DGS supports are needles**: the median is ~40-100x longer than it is
wide and the worst is 2,212x. The smallest covariance eigenvalue in the window
is 1.98e-9, only about ten times the configured `min_cov_eigenvalue = 1e-10`
floor, so `validate_models` admits them but they sit close to the degenerate
end. Nothing in the synthetic families resembles this.

## The cross-check, and why it uses the atlas v3 file

`splatc_atlas/results/gmc_h2/k2_windows.json` (atlas **v3**) records, from a
completely separate implementation -- shapely polygon unions at NPOLY=24, no
support-function oracle and no pair sandwich -- whether each door is open at
each of 720 orientations for three robot lengths. Those flags are a
falsifiable prediction in both directions.

The obvious file to reach for, `k2_windows_v2.json`, is the wrong one. v2
probed with a single POINT at a fixed offset, which the configuration-space
wall swallows as the robot grows, reporting an open corridor as closed. The
two files **disagree at up to 246 of 720 orientations**, and v2 calls
door_A/L32 never open where v3 finds it open 26.5% of the time. Using v2 with
point probes would let agreement come from reproducing v2's artifact, so this
experiment adopts v3's segment probes and cross-checks against v3, recording
the v2 verdict only to show which of the two the compiler lands on.

Two independent confirmations that the two stacks are looking at the same
geometry, before any verdict is compared: this experiment's wall-bearing
estimator returns **-29.25 deg**, matching the atlas's recorded
`wall_dir_deg` exactly, and the derived probe endpoints reproduce the atlas's
recorded `probes` to the printed digits.

### Result (door_A, half=1.5, job 17215125)

| robot | support (u) | theta | gmc | atlas v3 | atlas v2 |
|---|---|---|---|---|---|
| L16 | 1.6 x 0.6 | 1.5708 | open | open | open |
| L24 | 2.4 x 0.6 | 2.1293 | open | open | open |
| L24 | 2.4 x 0.6 | 0.6021 | shut | shut | shut |
| L32 | 3.2 x 0.6 | 1.2086 | **open** | **open** | **shut** |
| L32 | 3.2 x 0.6 | 1.7890 | shut | shut | shut |

**Agreement with atlas v3: 5/5. With the superseded v2: 4/5.** The single
disagreement is the L32 row, where the certified compiler independently
confirms that v2's "never open" was the point-probe artifact the atlas
worklog suspected. Free space splits into 3 and 2 certified-safe components
respectively on the two shut rows, which is the expected qualitative
behaviour.

**Negative control passed**: a point 0.837 u deep inside forbidden mass did
not locate in any certified-safe component.

The figure (`gmc/results/k2/figures/door_A_h1.5.png`, forbidden mass clipped
to the workspace) shows why the probe geometry matters here: at
theta = 2.129 the **door centre itself lies inside configuration-space
forbidden mass**, and the passage is a narrow channel offset around the lower
edge of the wall's grown envelope. A probe placed at the door centre, or at a
fixed offset from it, can easily land in that mass and report a closed gate.
That is an independent confirmation, from this compiler's own geometry, of
why the atlas replaced point probes with segments in v3.

## The failure: the prototype world-bound iteration is capped too low for real data

The full pipeline (Stage 4: compile -> query -> independent verification) did
**not** complete. `build_slabs` raised

    FloatingPointError: prototype outer world-coordinate bound did not converge
    envelopes.py:595  _prototype_outward_world_vertices <- approximate_pair

at an orientation the slab refinement picked, on the same half=1.5 scene whose
five Stage 1-2 orientations had all succeeded.

Root cause, established by measurement rather than inspection:

1. Not a bad splat. All 14,894 supports pass individually at theta=2.1293
   (single-support scenes, verified non-vacuous: 1 oracle, 1 sandwich,
   non-zero `C_plus` area each), and so do all 400 most degenerate ones.
2. Not scene size. The whole half=3.0 scene at theta=2.1293 passes: 14,894
   oracles, 45 s, zero failures.
3. It is **orientation-dependent**, and it is a **cap**.
   `_prototype_outward_world_vertices` (`envelopes.py:577-596`) runs a fixed
   point on the world-translation error bound with a hard limit of **16**
   iterations. Instrumented on real data at theta=2.1293 (59,576 calls), the
   iteration count decays from 50.4% at 2 iterations down a long tail that
   lands exactly on the cap: 13 iters x8, 14 x1, 15 x2, **16 x1**.
4. Raising the cap to 512 inside a probe (the compiler source untouched)
   shows the tail is genuinely longer than 16. Sweeping orientations with the
   L24 robot (job 17217578, `gmc/logs/gmc_k2_cap-17217578.out`):

   | window | theta | calls | max iters | calls needing > 16 |
   |---|---|---|---|---|
   | half=1.5 | 0.0000 | 18,264 | 11 | 0 |
   | half=1.5 | 0.6021 | 18,264 | 12 | 0 |
   | half=1.5 | 0.7854 | 18,264 | 13 | 0 |
   | half=1.5 | 2.1293 | 18,264 | 14 | 0 |
   | half=1.5 | 1.7890 | 18,264 | **16** | 0 |
   | half=1.5 | 1.2086 | 18,264 | **25** | 3 |
   | half=3.0 | 0.0000 | 59,576 | **16** | 0 |
   | half=3.0 | 2.1293 | 59,576 | **16** | 0 |
   | half=3.0 | 0.7854 | 59,577 | **19** | 2 |
   | half=3.0 | 1.7890 | 59,578 | **20** | 1 |
   | half=3.0 | 0.6021 | 59,577 | **25** | 2 |
   | half=3.0 | 1.2086 | 59,576 | **25** | 5 |

   **Five of twelve orientations exceed the cap outright and three more land
   exactly on it**; only four have any margin. The worst single call is a
   needle at mean (21.978, -14.810), semi-axes (9.09e-05, 0.0324) u, needing
   25 iterations.

   (The sweep fixes the robot at L24. That is why the experiment's
   theta=1.2086 row succeeded while this table shows 25 iterations at the same
   theta: the experiment ran that orientation with L32, the sweep with L24.
   Which pairs are hard depends on the robot as well as the orientation.)

Because 16 is right at the edge of the distribution, whether a given call
converges is decided by last-bit rounding: the same slice passed on the login
node and failed on a compute node. The synthetic families never approach the
cap, which is why this only appears on real 3DGS geometry.

The failure mode is correct: it raises rather than returning an uncertified
bound. Nothing unsound was produced.

**Proposed fix, not applied.** Raise the cap in
`_prototype_outward_world_vertices` from 16 to 64 -- 2.6x the measured worst
case of 25, and the iteration is monotone in `slack`, so extra iterations can
only tighten the bound and never weaken it. It is a one-line change inside the
certified core, so it is left for an explicit decision rather than made as a
side effect of this experiment. A cap of 16 does not merely fail sometimes on
real data: it is below what two thirds of the tested orientations need.

## Caveats

- **No planning claim.** `meta.json` still lists the wall-gap audit as not
  done, so an opening measured here may be glass or a scan hole.
- **Not reproducible from the repo alone.** Both the slab
  (`splatc_atlas/data/`) and the atlas reference (`splatc_atlas/results/`) are
  gitignored; they live on HPC only.
- Round 1 used half=1.5, where the probe segments run past the workspace box:
  only **51% of each segment is inside**, so the cross-check asks a shorter
  question than the atlas asked, and the 5/5 agreement is on that shorter
  question. The experiment now measures the inside fraction and warns; exact
  comparison at this wall bearing needs **half >= 2.09 u**. The half=3.0 round
  would have been the clean one, and it is the round that crashed.
- The 0.5 m/u scale is still UNVERIFIED in `meta.json`; all lengths above are
  in scene units.

## Open

- Rerun Stage 4 once the iteration cap is decided; the full pipeline has never
  completed on real data.
- Test B (door_A -> door_B, 8.25 u apart, ~13,940 splats at pad 1.0 u) is
  still unstarted and needs Stage 4 working first.
