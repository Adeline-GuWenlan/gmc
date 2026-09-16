# Why the shared three-robot case search returned nothing — diagnosis

Asked 2026-09-16, after the user questioned whether the per-robot routes are too simple to be
meaningful, and whether the A4 failure indicts the GMC approach itself.

Reproduce with `python gmc/experiments/height_map_diagnosis.py`.
Artefacts: `gmc/results/height/diagnosis/{map_diagnosis.json,fig1_phantom_floor.png,fig2_hall_bands.png}`.

## Short answer

It does not indict the planner or the certifier. Two separate causes, one in the data and one in
our own case criteria, and each is sufficient on its own.

## 1. The failure is confined to the near-floor bands

A4's search over 8 table/axis combinations (`results/height/showcase/case_search_a4.json`):

| robot | combinations passing criterion 3 | best min clearance over pairs |
|---|---|---|
| sweeper | 0 of 8 | 0.00 – 0.45 m |
| cylinder | 0 of 8 | 0.00 – 0.45 m |
| uav | 1500 / 2065 / 2468 / 3813 hits | 0.45 – 2.39 m |

Same machinery, same scene: the robot that flies is unobstructed, both ground robots are blocked
everywhere. The scene-wide pair search agrees — `pairs_dist_ok 1296 → crit1 490 → crit2 171 →
all three 0` — and the cylinder's free space is shattered into 65 components, the largest only
185 m² of a 990 m² hall.

## 2. The sweeper band of the map is not trustworthy

Band occupancy does not decrease monotonically toward the floor, which it should:

```
0.02–0.10 m  40.7%      <- 2.3x the band above it
0.10–0.55 m  17.4%
0.55–1.00 m  13.3%
1.00–1.75 m  11.7%
1.75–2.50 m  11.1%
```

Discriminating test: take cells that are free in **every** band from 0.10 m up to 2.5 m (78.1% of
the hall — genuinely open floor). Of those, **30.6% are still occupied in the 0.02–0.10 m band**:
237 m², 24% of the whole hall, and 58.7% of all low-band occupancy. A 2–10 cm obstacle with
nothing at all above it, repeated over 237 m² of an art gallery, is not furniture.

Morphology confirms it. The phantom layer has 367 connected components, median blob 0.04 m², 30%
of blobs ≤ 100 cm². The real furniture band (0.55–1.00 m) has 60 components, median 0.21 m², 1.7%
of blobs ≤ 100 cm². Speckle versus outlines — see `fig1_phantom_floor.png`, panels 1 and 2.

Ruled out as causes:

- **Scale.** `scale_check.json` has 5 independent measurements agreeing within 15% (bench seat
  0.43 m, table A top 0.80 m, reception counter 1.03 m, glass door clear height 2.37 m);
  `metric_accepted: true`.
- **The Amendment 1 floor rule fixing it.** It removes 71,594 of 7,319,425 splats (0.98%), and
  near-floor intruders reaching 0.02 m only fall 50,305 → 45,295 (−10%). Its 15° tilt test is the
  binding one: 31,218 intruders fail only that test.

## 3. Criteria 1 and 3 barely intersect, even on a perfect map

Spec §5.3 criterion 1 wants the straight start→goal segment to pass under an overhang; criterion 3
wants start and goal ≥ 0.5 m from any obstacle in every band. Total overhang in the hall is 26 m².

| region | as built | all phantom deleted |
|---|---|---|
| criterion-3 clear | 307 m² | 469 m² |
| …and within 0.5 m of an overhang | **0.9 m²** (0.09% of hall) | **1.2 m²** |
| …and within 1.0 m | 28.7 m² | 87.8 m² |

Deleting every phantom cell does not rescue the 0.5 m radius. The overhangs in this gallery are
table undersides, which are exactly where the legs are, so "pass under an overhang" and "half a
metre of clear floor at both endpoints" are close to mutually exclusive here. The criteria were
over-specified for this scene, independently of the data quality.

## 4. What this does mean

GMC consumes the 3DGS as ground truth and has no notion of reconstruction uncertainty. On this
scene, *sound with respect to the splats* is not *sound with respect to the room*: refusing to
route through phantom geometry is correct behaviour, but with untrustworthy input that soundness
converts into false negatives over a quarter of the floor. The Amendment 1 floor rule was the
first attempt at this and is a heuristic threshold, not an outer approximation with a guarantee.

That is the open research question, and it is a better thing to report than "no route was found" —
the three per-robot runs are all REACHABLE, certified and replay3d-passed, with clearance lower
bounds 0.05 / 0.027 / 0.0032 m.
