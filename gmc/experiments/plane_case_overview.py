# gmc/experiments/plane_case_overview.py
"""Amendment 3 P2: the scene-wide version of the argument, from p2a_search.json alone.

Where in the hall does a shared three-robot case exist, and where does it not? One figure over the whole
25.5 x 38.8 m gallery, so the chosen 4.3 x 5.6 m window is not read as a lucky corner.

This reads only rasters and JSON, never the 7.3 M-splat scene, so it runs anywhere. The backdrop is the
cached 0.55-1.00 m band of the **unedited** scene (`showcase/maps.npz`): the P1b edit only removes splats
whose rho-top is below floor + 0.10 m, and such a splat cannot appear in a band that starts at 0.55 m, so
that map is the same before and after the edit -- which is why it is honest to use it here. P1d measured
the same thing directly on the certified raster: the 0.10-0.18 and 0.18-0.26 m bands came out
bit-identical before and after.

CLAIMS BOUNDARY: the plane floor is a USER-APPROVED MANUAL SCENE EDIT (Amendment 3 P1); results are
sound with respect to the edited scene only. Criterion 3 is relaxed by user decision D1 to r + 0.05 m
(criteria 1 and 3 intersect in 0.9 m2 of the 990 m2 hall as built and 1.2 m2 with every phantom cell
deleted, height_map_diagnosis.md section 3). Criterion 1 is unchanged.

Run from gmc/: PYTHONPATH=src:experiments MPLBACKEND=Agg python experiments/plane_case_overview.py
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = Path("results/height/plane")
FIGS = OUT / "figs"
MAPS = Path("/scratch/wg2381/splathjb/splatc_atlas/data/gs_scenes/showcase/maps.npz")
CLAIMS = ("plane floor = USER-APPROVED MANUAL SCENE EDIT (Amendment 3 P1), sound w.r.t. the edited scene "
          "only · criterion 3 relaxed by user decision D1 to r + 0.05 m (crit1 & crit3 = 0.9 m² as built "
          "/ 1.2 m² with every phantom cell deleted) · criterion 1 unchanged")
# A4's scene-wide funnel on the UNEDITED scene, for the comparison that matters (map_diagnosis.md §1).
A4 = {"pairs_dist_ok": 1296, "crit1": 490, "crit2": 171, "all_three": 0}


def main():
    search = json.loads((OUT / "p2a_search.json").read_text())
    case = json.loads((OUT / "case_shared.json").read_text())
    regions = search["regions"]
    mp = np.load(MAPS)
    ext = mp["extent"]

    fig = plt.figure(figsize=(21, 13))
    ax = fig.add_axes((0.04, 0.06, 0.46, 0.84))
    ax.imshow(mp["occ_0.55_1.00"].T, origin="lower", cmap="Greys", alpha=0.65,
              extent=[ext[0], ext[2], ext[1], ext[3]], interpolation="nearest")
    best = max((r.get("pairs_connected_and_contrasting", 0) for r in regions.values()), default=1)
    for name, r in regions.items():
        b = r["box"]
        n = r.get("pairs_connected_and_contrasting", 0)
        k = r.get("pairs_passing_criterion1", 0)
        col = plt.cm.viridis(0.15 + 0.8 * (n / max(best, 1)))
        ax.add_patch(plt.Rectangle((b[0], b[1]), b[2] - b[0], b[3] - b[1], fill=True, alpha=0.22,
                                   color=col, ec=col, lw=2.5))
        ax.text(b[0] + 0.15, b[3] - 0.65,
                f"{name}: {n} contrasting pairs\n{k}/120 pass criterion 1\n"
                f"cylinder {r['supports_per_m2']['cylinder']:.0f} supports/m²",
                fontsize=8.5, va="top", color="black",
                bbox=dict(fc="white", ec=col, alpha=0.82, lw=1.2, boxstyle="round,pad=0.25"))
    w = case["window"]
    ax.add_patch(plt.Rectangle((w[0], w[1]), w[2] - w[0], w[3] - w[1], fill=False, ec="red", lw=3.0))
    ax.plot(case["start"][0], case["start"][1], "go", ms=11)
    ax.plot(case["goal"][0], case["goal"][1], "bs", ms=11)
    ax.annotate(f"the shared case {case['selection']['label']}\n"
                f"{w[2] - w[0]:.2f} × {w[3] - w[1]:.2f} m of a 990 m² hall",
                xy=(w[0], w[1]), xytext=(w[0] - 1.0, w[1] - 4.5), fontsize=10, color="red",
                bbox=dict(fc="white", ec="red", alpha=0.9, boxstyle="round,pad=0.25"),
                arrowprops=dict(arrowstyle="->", color="red", lw=2))
    ax.axhspan(32.0, ext[3], color="orange", alpha=0.14)
    ax.text(ext[0] + 0.4, 34.8, "y > 32 excluded by P1: the round-tables cluster holds 18 m² of the\n"
                                "28.5 m² of residual near-floor over-approximation", fontsize=8.5,
            color="darkorange")
    ax.set_xlim(ext[0], ext[2])
    ax.set_ylim(ext[1], ext[3])
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("where a shared three-robot case exists in the showcase art-gallery hall\n"
                 "grey = the 0.55–1.00 m band (furniture outlines; unchanged by the P1b edit)",
                 fontsize=11)

    names = list(regions)
    ax2 = fig.add_axes((0.56, 0.56, 0.40, 0.34))
    xs = np.arange(len(names))
    ax2.bar(xs - 0.22, [regions[n].get("lattice_points_passing_D1_for_all_three", 0) for n in names],
            0.44, label="lattice points passing D1 for all three robots", color="tab:blue")
    ax2.bar(xs + 0.22, [regions[n].get("pairs_connected_and_contrasting", 0) for n in names],
            0.44, label="pairs: all three connected, line free for sweeper, blocked for cylinder",
            color="tab:red")
    ax2.set_yscale("log")
    ax2.set_xticks(xs)
    ax2.set_xticklabels(names)
    ax2.set_ylabel("count (log)")
    ax2.grid(alpha=0.3, axis="y")
    ax2.legend(fontsize=8, loc="lower left")
    ax2.set_title("per region, on the CERTIFIED map (project_scene → _support_raster 0.025 m → EDT)\n"
                  f"A4 on the unedited scene, scene-wide: {A4['pairs_dist_ok']} → crit1 {A4['crit1']} → "
                  f"crit2 {A4['crit2']} → all three **{A4['all_three']}**", fontsize=10)

    ax3 = fig.add_axes((0.56, 0.08, 0.40, 0.38))
    ax3.axis("off")
    sup = case["n_supports"]
    pre = case["precheck"]
    rows = [["", "sweeper", "cylinder", "uav"],
            ["band above floor (m)", "0.02–0.10", "0.02–1.75", "1.10–1.30"],
            ["disc radius r (m)", "0.175", "0.300", "0.250"],
            ["supports in the window", f"{sup['sweeper']:,}", f"{sup['cylinder']:,}", f"{sup['uav']:,}"],
            ["supports per m²", f"{case['supports_per_m2']['sweeper']:.0f}",
             f"{case['supports_per_m2']['cylinder']:.0f}", f"{case['supports_per_m2']['uav']:.0f}"],
            ["start clearance (m)", f"{pre['sweeper']['start_clear_m']:.3f}",
             f"{pre['cylinder']['start_clear_m']:.3f}", f"{pre['uav']['start_clear_m']:.3f}"],
            ["goal clearance (m)", f"{pre['sweeper']['goal_clear_m']:.3f}",
             f"{pre['cylinder']['goal_clear_m']:.3f}", f"{pre['uav']['goal_clear_m']:.3f}"],
            ["D1 needs (r + 0.05 m)", "0.225", "0.350", "0.300"],
            ["straight line unusable (m)", f"{pre['sweeper']['line']['blocked_len_m']:.2f}",
             f"{pre['cylinder']['line']['blocked_len_m']:.2f}",
             f"{pre['uav']['line']['blocked_len_m']:.2f}"],
            ["free-space route (m)", f"{pre['sweeper']['path_len_m']:.2f}",
             f"{pre['cylinder']['path_len_m']:.2f}", f"{pre['uav']['path_len_m']:.2f}"]]
    t = ax3.table(cellText=rows[1:], colLabels=rows[0], loc="center", cellLoc="center")
    t.auto_set_font_size(False)
    t.set_fontsize(9)
    t.scale(1, 1.55)
    for c in range(4):
        t[(0, c)].set_facecolor("#dddddd")
    ax3.set_title(f"the shared case: window {w}, start {tuple(round(v, 2) for v in case['start'][:2])}, "
                  f"goal {tuple(round(v, 2) for v in case['goal'][:2])} — identical for all three\n"
                  f"detour ratio {case['detour_ratio']:.2f}×, routes separate by "
                  f"{case['separation_m']:.2f} m", fontsize=10)

    fig.suptitle("Amendment 3 P2 — the shared three-robot case, scene-wide\n" + CLAIMS, fontsize=10)
    FIGS.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGS / "p2_scene_wide.png", dpi=85)
    plt.close(fig)

    summary = {"claims_boundary": CLAIMS,
               "task": "P2: scene-wide context for the shared case",
               "backdrop": "showcase/maps.npz occ_0.55_1.00, unchanged by the P1b edit (it removes only "
                           "splats whose rho-top is below floor + 0.10 m, which cannot reach a band "
                           "starting at 0.55 m; P1d measured the 0.10-0.18 and 0.18-0.26 m certified "
                           "bands bit-identical before and after)",
               "a4_scene_wide_funnel_on_the_unedited_scene": A4,
               "regions": {n: {k: regions[n].get(k) for k in
                               ("box", "area_m2", "supports_per_m2", "free_components",
                                "lattice_points_passing_D1_for_all_three",
                                "pairs_connected_and_contrasting", "pairs_with_two_routes",
                                "pairs_passing_criterion1")} for n in names},
               "chosen": case["selection"] | {"window": w, "start": case["start"], "goal": case["goal"],
                                              "window_m2": round((w[2] - w[0]) * (w[3] - w[1]), 2),
                                              "hall_m2": 990},
               "figure": str(FIGS / "p2_scene_wide.png")}
    (OUT / "p2_scene_wide.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary["regions"], indent=1))
    print("wrote", FIGS / "p2_scene_wide.png")


if __name__ == "__main__":
    main()
