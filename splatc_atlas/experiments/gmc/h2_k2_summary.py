"""Summary figure for the K2 window sweeps: morphology-closure ladder.

Left: gate open intervals (theta in [0,180)) per door per robot length —
the orientation-gate structure narrowing as the robot grows.
Right: open fraction + S3 event count vs robot support length.
Reads results/gmc_h2/k2_windows.json (v3).
"""
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parents[2] / "results" / "gmc_h2"
runs = json.loads((OUT / "k2_windows.json").read_text())["runs"]

fig, axes = plt.subplots(1, 2, figsize=(15, 5.2))
ax = axes[0]
rows = [(w, r) for w in ("door_A", "door_B") for r in ("L16", "L24", "L32")]
for y, (w, r) in enumerate(rows):
    res = next(x for x in runs if x["window"] == w and x["robot"] == r)
    flags = np.array(res["open_flags"], dtype=bool)
    th = np.linspace(0, 180, len(flags), endpoint=False)
    ax.fill_between(th, y + 0.08, y + 0.92, where=flags, color="tab:green", alpha=0.6)
    ax.fill_between(th, y + 0.08, y + 0.92, where=~flags, color="tab:red", alpha=0.25)
ax.set_yticks([y + 0.5 for y in range(len(rows))],
              [f"{w} / {r} (sup {next(x for x in runs if x['window']==w and x['robot']==r)['robot_support'][0]:.1f}u)"
               for w, r in rows], fontsize=8)
ax.set_xlabel("theta (deg)")
ax.set_xlim(0, 180)
ax.set_title("orientation-gate intervals at two real K2 doorways\n(green=passable) — gates narrow as the robot grows")

ax = axes[1]
lens = {"L16": 1.6, "L24": 2.4, "L32": 3.2}
for w, c in (("door_A", "tab:blue"), ("door_B", "tab:orange")):
    xs, of, ev = [], [], []
    for r in ("L16", "L24", "L32"):
        res = next(x for x in runs if x["window"] == w and x["robot"] == r)
        xs.append(lens[r]); of.append(res["gate_open_fraction"]); ev.append(res["n_s3_events"])
    ax.plot(xs, of, "o-", color=c, label=f"{w} open fraction")
    ax2 = ax.twinx() if w == "door_A" else ax2
    ax2.plot(xs, ev, "s--", color=c, alpha=0.5)
ax.axvline(2.0, color="0.6", ls=":", label="door gap ~2.0u")
ax.set_xlabel("robot support length (u)")
ax.set_ylabel("gate open fraction (solid)")
ax2.set_ylabel("S3 events per pi sweep (dashed)")
ax.set_ylim(-0.03, 1.05)
ax.legend(fontsize=8, loc="center left")
ax.set_title("closure ladder: open fraction vs robot length")
fig.tight_layout()
fig.savefig(OUT / "figs" / "k2_closure_ladder.png", dpi=120)
print("saved", OUT / "figs" / "k2_closure_ladder.png")
