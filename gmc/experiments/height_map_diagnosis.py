"""Why the shared three-robot case search on the showcase scene returned nothing.

Two competing explanations are separated here, from the artefacts only:

  (a) the 3DGS reconstruction puts phantom geometry in the sweeper band, so the
      obstacle map is wrong near the floor;
  (b) criteria 1 and 3 of spec 5.3 (pass under an overhang; 0.5 m clearance at
      start and goal in every band) have an almost empty intersection in this
      hall, independently of (a).

Both turn out to hold, and (b) survives a perfect repair of (a). The test for
(a) is that a cell occupied in [0.02, 0.10] m with nothing above it anywhere up
to 2.5 m is not a plausible piece of gallery furniture.

Inputs : showcase maps.npz (bands + overhang), results/height/showcase/*.json
Output : gmc/results/height/diagnosis/
"""
import json
from pathlib import Path

import numpy as np
from scipy import ndimage
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D

DATA = Path("/scratch/wg2381/splathjb/splatc_atlas/data/gs_scenes/showcase")
ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "gmc/results/height/diagnosis"
OUT.mkdir(parents=True, exist_ok=True)

SURFACE = "#fcfcfb"
INK, INK2 = "#0b0b0b", "#52514e"
REAL = "#3a3a38"     # low-band occupancy that has support above it
PHANTOM = "#d03b3b"  # low-band occupancy with nothing above it
OVERH = "#8cb9f0"
VIABLE = "#0ca30c"

plt.rcParams.update({
    "font.family": "sans-serif", "font.sans-serif": ["DejaVu Sans"],
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE, "text.color": INK, "axes.labelcolor": INK2,
    "xtick.color": INK2, "ytick.color": INK2, "axes.edgecolor": "#c3c2b7",
})

BANDS = ["occ_0.02_0.10", "occ_0.10_0.55", "occ_0.55_1.00",
         "occ_1.00_1.75", "occ_1.75_2.50"]
CELL = 0.05          # m, from maps.npz grid vs extent
AREA = CELL * CELL

d = np.load(DATA / "maps.npz")
xmin, ymin, xmax, ymax = (float(v) for v in d["extent"])
extent = [xmin, xmax, ymin, ymax]
hall_m2 = (xmax - xmin) * (ymax - ymin)

low = d[BANDS[0]]
overhang = d["overhang"]
above = np.zeros_like(low)
for k in BANDS[1:]:
    above |= d[k]

# (a) phantom = occupied in the sweeper band, free everywhere above it.
phantom = low & ~above
real_low = low & above
open_floor = ~above


def clearance(mask):
    """Metric distance from every cell to the nearest occupied cell."""
    return ndimage.distance_transform_edt(~mask) * CELL


def blobs(mask):
    lab, n = ndimage.label(mask)
    if n == 0:
        return n, np.zeros(0)
    return n, np.bincount(lab.ravel())[1:] * AREA


n_ph, sz_ph = blobs(phantom)
n_fur, sz_fur = blobs(d["occ_0.55_1.00"])

report = {
    "hall": {"extent": [xmin, ymin, xmax, ymax], "cell_m": CELL,
             "footprint_m": [xmax - xmin, ymax - ymin], "area_m2": hall_m2},
    "band_occupancy_frac": {k: float(d[k].mean()) for k in BANDS},
    "phantom_test": {
        "definition": "occupied in [0.02,0.10] m and free in every band above it",
        "open_floor_cells": int(open_floor.sum()),
        "open_floor_frac_of_hall": float(open_floor.mean()),
        "phantom_cells": int(phantom.sum()),
        "phantom_m2": float(phantom.sum() * AREA),
        "phantom_frac_of_open_floor": float(phantom.sum() / open_floor.sum()),
        "phantom_frac_of_all_low_band": float(phantom.sum() / low.sum()),
        "phantom_components": int(n_ph),
        "phantom_median_blob_m2": float(np.median(sz_ph)),
        "phantom_frac_blobs_le_100cm2": float((sz_ph <= 4 * AREA).mean()),
        "furniture_band_components": int(n_fur),
        "furniture_band_median_blob_m2": float(np.median(sz_fur)),
        "furniture_band_frac_blobs_le_100cm2": float((sz_fur <= 4 * AREA).mean()),
    },
}

# (b) criterion 3 wants >= 0.5 m clear in every band; criterion 1 wants the
# straight segment to pass under an overhang. Intersect them, as built and
# after deleting all phantom occupancy.
dist_overhang = clearance(overhang)


def crit3_mask(low_band):
    clr = None
    for k in BANDS:
        c = clearance(low_band if k == BANDS[0] else d[k])
        clr = c if clr is None else np.minimum(clr, c)
    return clr >= 0.5


joint = {}
for tag, lowmap in (("as_built", low), ("phantom_removed", low & ~phantom)):
    ok = crit3_mask(lowmap)
    joint[tag] = {
        "crit3_m2": float(ok.sum() * AREA),
        "crit3_frac_of_hall": float(ok.mean()),
        "crit3_and_near_overhang_m2": {
            f"{r:.1f}": float((ok & (dist_overhang <= r)).sum() * AREA)
            for r in (0.5, 1.0, 1.5, 2.0, 3.0)},
    }
    if tag == "as_built":
        viable = ok & (dist_overhang <= 0.5)
report["criteria_intersection"] = joint
report["criteria_intersection"]["overhang_m2"] = float(overhang.sum() * AREA)
report["criteria_intersection"]["note"] = (
    "spec 5.3 criterion 1 = straight segment passes under an overhang; "
    "criterion 3 = start and goal >= 0.5 m from any obstacle in every band. "
    "Repairing the map does not rescue the 0.5 m radius, so the criteria are "
    "over-specified for this hall, not only the data.")

# What A4 actually reported, carried through so the figure and the text agree.
a4_path = ROOT / "gmc/results/height/showcase/case_search_a4.json"
if a4_path.exists():
    a4 = json.loads(a4_path.read_text())
    report["a4_case_search"] = {
        "pass": a4.get("pass"),
        "per_table": {t: {"lines": v.get("lines"),
                          "crit3_sweeper": v.get("crit3_sweeper"),
                          "crit3_cylinder": v.get("crit3_cylinder"),
                          "crit3_uav": v.get("crit3_uav"),
                          "best_min_clearance": v.get("best_min_clearance_over_pairs")}
                      for t, v in a4["tables"].items()},
        "pairs": a4.get("pairs", {}).get("counts"),
    }

(OUT / "map_diagnosis.json").write_text(json.dumps(report, indent=2) + "\n")

# ------------------------------------------------------------------ figures --
fig, ax = plt.subplots(1, 3, figsize=(16.5, 9.5), constrained_layout=True)

rgb = np.ones(low.shape + (3,))
rgb[real_low] = matplotlib.colors.to_rgb(REAL)
rgb[phantom] = matplotlib.colors.to_rgb(PHANTOM)
ax[0].imshow(np.transpose(rgb, (1, 0, 2)), origin="lower", extent=extent,
             aspect="equal", interpolation="nearest")
ax[0].set_title(
    "sweeper band 0.02-0.10 m\nwhat is actually there", fontsize=11)
ax[0].legend(handles=[
    Line2D([], [], color=REAL, lw=6,
           label=f"has support above it   {real_low.sum() * AREA:.0f} m2"),
    Line2D([], [], color=PHANTOM, lw=6,
           label=f"nothing above it to 2.5 m   {phantom.sum() * AREA:.0f} m2\n"
                 f"= {100 * phantom.sum() / open_floor.sum():.0f}% of open floor")],
    loc="upper left", fontsize=8.5, framealpha=0.92)

ax[1].imshow(d["occ_0.55_1.00"].T, origin="lower", extent=extent, cmap="Greys",
             vmin=0, vmax=1.35, aspect="equal", interpolation="nearest")
ax[1].set_title(
    "same hall at table height 0.55-1.00 m\nreal furniture, for contrast", fontsize=11)
ax[1].legend(handles=[
    Line2D([], [], color=REAL, lw=6,
           label=f"{n_fur} components, median {np.median(sz_fur):.2f} m2\n"
                 f"vs {n_ph} / {np.median(sz_ph):.2f} m2 in the red layer")],
    loc="upper left", fontsize=8.5, framealpha=0.92)

rgb2 = np.ones(low.shape + (3,))
rgb2[overhang] = matplotlib.colors.to_rgb(OVERH)
rgb2[viable] = matplotlib.colors.to_rgb(VIABLE)
ax[2].imshow(np.transpose(rgb2, (1, 0, 2)), origin="lower", extent=extent,
             aspect="equal", interpolation="nearest")
ax[2].set_title(
    "where a shared three-robot case could start\ncriteria 1 and 3 intersected",
    fontsize=11)
ax[2].legend(handles=[
    Line2D([], [], color=OVERH, lw=6,
           label=f"overhang, free low blocked high   {overhang.sum() * AREA:.0f} m2"),
    Line2D([], [], color=VIABLE, lw=6,
           label=f"also criterion-3 clear   {viable.sum() * AREA:.1f} m2\n"
                 f"= {100 * viable.mean():.2f}% of the hall")],
    loc="upper left", fontsize=8.5, framealpha=0.92)

for a in ax:
    a.set_xlabel("x (m)")
    a.set_xlim(xmin, xmax)
    a.set_ylim(ymin, ymax)
ax[0].set_ylabel("y (m)")
fig.suptitle(
    f"showcase hall {xmax - xmin:.1f} x {ymax - ymin:.1f} m = {hall_m2:.0f} m2   |   "
    "why the shared three-robot case search returned nothing", fontsize=13)
p1 = OUT / "fig1_phantom_floor.png"
fig.savefig(p1, dpi=110)
plt.close(fig)

# Global context: every band, with the three per-robot cases that did succeed.
cases = {}
for robot, col in (("uav", "#2a78d6"), ("sweeper", "#d03b3b"), ("cylinder", "#0ca30c")):
    cj = ROOT / f"gmc/results/height/percase/{robot}/case.json"
    mj = ROOT / f"gmc/results/height/percase/{robot}/video/manifest.json"
    if cj.exists() and mj.exists():
        cases[robot] = (json.loads(cj.read_text()),
                        json.loads(mj.read_text())["selection"]["window"], col)

panels = [("occ_0.02_0.10", "0.02-0.10 m   sweeper band"),
          ("occ_0.55_1.00", "0.55-1.00 m   table tops"),
          ("occ_1.00_1.75", "1.00-1.75 m   cylinder body"),
          ("overhang", "overhang   free low, blocked high")]
fig, axes = plt.subplots(1, len(panels), figsize=(19, 11), constrained_layout=True)
for a, (key, name) in zip(axes, panels):
    a.imshow(d[key].T, origin="lower", extent=extent, cmap="Greys", vmin=0,
             vmax=1.35, aspect="equal", interpolation="nearest")
    for robot, (c, w, col) in cases.items():
        a.add_patch(Rectangle((w[0], w[1]), w[2] - w[0], w[3] - w[1],
                              fill=False, ec=col, lw=1.8, zorder=3))
        s, g = c["start"], c["goal"]
        a.plot([s[0], g[0]], [s[1], g[1]], color=col, lw=2.0, zorder=4)
        a.plot(*s[:2], marker="^", ms=7, color=col, mec="k", mew=0.6, zorder=5)
        a.plot(*g[:2], marker="*", ms=12, color=col, mec="k", mew=0.6, zorder=5)
    a.set_title(f"{name}\noccupied {d[key].mean() * 100:.1f}% of {hall_m2:.0f} m2",
                fontsize=11)
    a.set_xlabel("x (m)")
    a.set_xlim(xmin, xmax)
    a.set_ylim(ymin, ymax)
axes[0].set_ylabel("y (m)")
for robot, (c, w, col) in cases.items():
    area = (w[2] - w[0]) * (w[3] - w[1])
    axes[0].plot([], [], color=col, lw=2.2,
                 label=f"{robot}: {c.get('behaviour')}  "
                       f"({area:.1f} m2, {100 * area / hall_m2:.1f}% of hall)")
axes[0].legend(loc="upper left", fontsize=9, framealpha=0.92)
fig.suptitle(
    f"showcase hall, whole floor {xmax - xmin:.1f} x {ymax - ymin:.1f} m = "
    f"{hall_m2:.0f} m2   |   the three per-robot cases in context", fontsize=13)
p2 = OUT / "fig2_hall_bands.png"
fig.savefig(p2, dpi=110)
plt.close(fig)

print(f"hall {hall_m2:.0f} m2, cell {CELL} m")
print(f"phantom {phantom.sum() * AREA:.0f} m2 = "
      f"{100 * phantom.sum() / open_floor.sum():.1f}% of open floor, "
      f"{100 * phantom.sum() / low.sum():.1f}% of all low-band occupancy")
print(f"crit3 & within 0.5 m of overhang: as built "
      f"{joint['as_built']['crit3_and_near_overhang_m2']['0.5']:.1f} m2, "
      f"phantom removed "
      f"{joint['phantom_removed']['crit3_and_near_overhang_m2']['0.5']:.1f} m2")
for p in (OUT / "map_diagnosis.json", p1, p2):
    print(p)
