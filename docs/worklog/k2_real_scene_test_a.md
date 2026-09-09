# Worklog — Test A on the real K2 3DGS scene (2026-09-08)

Scope: does the certified compiler run at all on real 3DGS-derived geometry,
and do its verdicts agree with an independent implementation on the same data?
This is the milestone the cloud audit's action 5 asked for ("real 3DGS before
any compression claim is written up"). It is a **plumbing and cross-check**
result. No planning or compression claim follows from it — see Caveats.

Code: `gmc/experiments/k2_scene.py` (loader), `gmc/experiments/k2_door_gate.py`
(experiment), `gmc/hpc/k2_door_gate.sbatch`. Results in `gmc/results/k2/`.

## Step 0 — the fisheye_gs overlay is not usable as a scene

> **Superseded.** The diagnosis in this section is wrong and the
> overlay has since been read and extracted without mounting it at all. See
> correction 2 in the update at the end of this file.

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
is 1.98e-9, only about ten times [**wrong: it is 19.8x, see correction 4**] the configured `min_cov_eigenvalue = 1e-10`
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
only tighten the bound and never weaken it [**stated backwards — see correction
1 in the update at the end of this file; the true argument is that extra iterations are
monotonically more conservative, not tighter**]. It is a one-line change inside the
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

---

## Update — later on 2026-09-08, written up 09-09: cap raised, pipeline completed once, four corrections

Jobs `17225249` (inward-loop probe, 16:30:35-16:36:52) and `17225251` (Stage 4,
16:30:35-18:14:26), both on 2026-09-08.

### The change

`_WORLD_EXPORT_MAX_ITERS = 64` in `gmc/src/gmc/geometry/envelopes.py`, applied to
both `range(16)` loops: `_prototype_outward_world_vertices` and
`_inward_world_points`. Theorem-mode `_outward_world_vertices` keeps its
`range(24)` and is untouched. Every call that already converged within 16
iterations returns bit-identical results; the only behavioural change is that
calls which previously exhausted the budget now get more attempts at the same
exit condition.

### The inward loop was measured, and it was not the problem

The inward cap runs in *both* certificate modes and had never been instrumented.
Job `17225249` probed it at cap 512 over the same twelve (window, theta) cells:

| | outer (prototype) | inner |
|---|---|---|
| worst case | 25 iters | **15 iters** |
| cells exceeding 16 | 5 of 12 | **0 of 12** |
| cells landing exactly on 16 | 3 | 0 |
| empty-inner early returns / inradius raises / FAILs | — | 0 / 0 / 0 |

So 16 was sufficient for the inward fixed point — by exactly one iteration at
worst (theta=0.7854 reaches 15 in both windows). It had no margin and nobody had
checked. The outward loop was the only actual blocker.

### Stage 4 completed for the first time — with verdict UNKNOWN

Job `17225251`, 1 h 43 m 51 s, exit 0. `build_slabs` no longer raises.

    n_supports 4,566   n_pairs 4,566
    compile 5,707.4 s  query 394.1 s
    M_safe     203 nodes / 203 edges
    M_possible 283 nodes / 339 edges
    status UNKNOWN     clearance_lb None

**The query returned no curve, so `independent_verification.ran` is false.** The
crash is fixed and the pipeline runs end to end, but the safety property Stage 4
exists to test — re-verifying the returned path against all original supports —
is still unexercised. This is a plumbing milestone, not a verification result.
The next question is why the query is UNKNOWN: M_possible carries 339 edges
against M_safe's 203, which is consistent with the start and goal being joined
only through slabs that never reached a certified status, but that has not been
confirmed.

### Corrections to what is written above

1. **The soundness argument was stated backwards.** This document said extra
   iterations "can only tighten the bound and never weaken it" (and the Chinese
   version said 多迭代只会让界更紧). That is the wrong direction: the loops only
   ever grow `slack` and `alpha`, so extra iterations produce a *larger* outer
   hull and a *smaller* inner hull — a looser over-approximation of the
   C-obstacle and therefore less free space. The change is sound because it is
   monotonically **more conservative**, never less.

2. **Step 0's diagnosis of the fisheye_gs overlay was wrong**, and the correct
   one dissolves the problem. The payload is at `/upper/datasets`, not
   `/datasets` — `upper` is the overlayfs upper directory. `/upper/datasets` is
   mode 0750 owned by **uid 3003305, gid 100**, a different real user; the
   container rendered it as `nfsnobody` only because uid 3003305 has no passwd
   entry in the base image. `--fakeroot` could therefore never have worked
   either, since an unmappable owner uid defeats the namespace root mapping.
   No read-write mount is needed at all: `debugfs` reads the ext3 image as a
   file, consulting no permission bits, and without `-w` cannot modify it. See
   `gmc/hpc/k2_dataset_extract.sh`. Extracted 2026-09-08 to
   `/scratch/wg2381/fisheye_gs/k2_dataset/`: 26 GB, 6,689 files, **all 66
   shipped SHA256SUMS entries OK**, `map.ply` hashing to `d6f8f327...c43a2`
   exactly as its manifest declares.

3. **"The full pipeline has never completed on real data" no longer holds** — it
   completed on 2026-09-08 (job 17225251, 16:30:35-18:14:26), as above. The stronger claim it was standing in for
   does still hold: the independent verification arm has never run.

4. **"only about ten times the configured `min_cov_eigenvalue = 1e-10` floor" is
   wrong.** The measured minimum is 1.983e-9, which is **19.8x** the floor, not
   ~10x. Re-measured directly from `slab_2d.npz` on 2026-09-08 while building
   the talk figures. The qualitative point survives -- these supports sit close
   to the degenerate end and `validate_models` admits them -- but the margin is
   twice what this document claimed.

### One implementation difference worth recording

`gmc/experiments/k2_scene.py::load_window` takes supports from a box of
half-width `half + 0.75` while the workspace stays at `half`, because a splat
whose mean sits just outside the workspace can still intrude into it and
dropping it would silently open free space at the boundary. The atlas
(`h2_k2_windows.py::load_window`) uses `half + 0.5`. The two stacks therefore
admit slightly different support sets at the window edge; ours is the more
conservative of the two. This is not a discrepancy to fix, but it is a reason
the two implementations are not expected to agree bit-for-bit at the boundary,
and it was not recorded anywhere before now.

### What the overlay actually contains

Not a trained 3DGS — the raw Lixel K2 capture, `verification.json` all `pass`:
2,130 three-camera frames (710 each left/centre/right; left and right fisheye
`kb4`/`OPENCV_FISHEYE`, centre pinhole), Livox Mid-360 lidar at 46,303,401
points over 2,335 scans, `map.ply` at 38,882,391 points, 2,310 trajectory poses,
47,600 IMU samples, 7 calibration files, and a known-pose COLMAP rig (3 cameras,
2,064 images, 0 points3D).

This bears on two standing caveats. The LAS header is **metric at 1 mm scale**,
min `[-2.301, -25.505, -2.025]` max `[52.194, 18.502, 10.082]`, a
54.5 x 44.0 x 12.1 m extent — enough to settle the UNVERIFIED 0.5 m/u scale
hypothesis by comparison against the slab's extent. And a 38.9 M-point lidar map
is independent geometry for the pending wall-gap audit. Neither has been done.
