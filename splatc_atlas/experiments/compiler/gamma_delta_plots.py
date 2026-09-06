"""Figure for the Gamma(delta) experiment (reads gamma_delta.json).

Left: log-log query cost vs delta — continuation (total and minus-seed),
cold-start, cert-only, plus the hybrid-floor volume reference (from
floor_probe_hybrid.json, not re-run).  Fitted exponents in the legend.
Right: Gamma(delta) = C / C_cert-only.

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/gamma_delta_plots.py
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = os.path.join(os.path.dirname(__file__), "..", "..")
TABDIR = os.path.join(BASE, "results", "tables")
FIGDIR = os.path.join(BASE, "results", "figures")

# hybrid floor reference (results/tables/floor_probe_hybrid.json, worklog
# 08-13): certified-discovery floor under oracle scheduling
HYBRID_REF = {40.0: 4096, 20.0: 16384, 10.0: 131072}
HYBRID_FAIL = {2.5: 131072}          # >=128k failed at delta<=2.5mm


def main():
    with open(os.path.join(TABDIR, "gamma_delta.json")) as f:
        g = json.load(f)
    fits = g["fits"]
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13.5, 5.5))

    cont = [r for r in g["continuation"] if r["status"] == "CERTIFIED_REACHABLE"]
    cold = [r for r in g["cold_start"] if r["all_solved"]]
    cert = [r for r in g["cert_only"] if r["certified"]]

    ax.loglog([r["delta_mm"] for r in cont], [r["total"] for r in cont],
              "o-", color="tab:red",
              label=f"continuation total (α={fits['continuation_total']}, "
                    "seed-dominated; NOT the robust statistic)")
    ax.loglog([r["delta_mm"] for r in cont],
              [r["c_track"] for r in cont], "o-", color="tab:purple",
              label=f"c_track (α={fits['continuation_track']}, narrow-3 "
                    f"{g.get('fits_narrow3', {}).get('continuation_track', '?')}"
                    ", R²=0.999)")
    ax.loglog([r["delta_mm"] for r in cont],
              [r["total"] - r["c_seed"] for r in cont], "o--", color="tab:red",
              alpha=0.6,
              label=f"continuation − seed (α={fits['continuation_minus_seed']})")
    f3 = g.get("fits_narrow3", {})
    ax.loglog([r["delta_mm"] for r in cold],
              [r["total_queries"] for r in cold], "s-", color="tab:orange",
              label=f"cold-start stationwise ablation, oracle-positioned "
                    f"(α={fits['cold_start']})")
    ax.loglog([r["delta_mm"] for r in cert], [r["checks"] for r in cert],
              "^-", color="tab:green",
              label=f"cert-only oracle witness (α={fits['cert_only']}, "
                    f"narrow-3 {f3.get('cert_only', '?')})")
    dh = sorted(HYBRID_REF)
    ax.loglog(dh, [HYBRID_REF[d] for d in dh], "d-", color="tab:gray",
              label="hybrid floor: CENSORED lower bound (old run, "
                    "last pt at budget cap)")
    for d, v in HYBRID_FAIL.items():
        ax.loglog([d], [v], "x", color="tab:gray", ms=10, mew=2)
        ax.annotate("fail ≥128k (cap)", (d, v), fontsize=8, color="tab:gray",
                    textcoords="offset points", xytext=(6, 4))
    # mark continuation failures if any
    for r in g["continuation"]:
        if r["status"] != "CERTIFIED_REACHABLE":
            ax.axvline(r["delta_mm"], color="tab:red", lw=0.8, ls=":")
            ax.annotate(f"cont {r['status']}", (r["delta_mm"], 1e3),
                        rotation=90, fontsize=7, color="tab:red")
    ax.invert_xaxis()
    ax.set_xlabel("delta = (w-2b)/2 [mm]  (narrower →)")
    ax.set_ylabel("pose queries / margin checks")
    ax.set_title("discovery vs certification cost, G1 de-aligned")
    ax.grid(True, which="both", alpha=0.25)
    ax.legend(fontsize=8)

    gm = g["gamma"]
    ax2.semilogx([r["delta_mm"] for r in gm], [r["gamma_total"] for r in gm],
                 "o-", color="tab:red",
                 label="Γ_total = pipeline / oracle-witness cert")
    ax2.semilogx([r["delta_mm"] for r in gm],
                 [r["gamma_amortized"] for r in gm], "o--", color="tab:red",
                 alpha=0.6,
                 label="Γ_amortized (excl. one-time seed; valid only "
                       "once reuse is shown)")
    ax2.semilogx([r["delta_mm"] for r in gm],
                 [r["gamma_self"] for r in gm], "o-", color="tab:blue",
                 alpha=0.8, label="Γ_self = pipeline / cert of own output")
    ax2.invert_xaxis()
    ax2.set_xlabel("delta [mm]  (narrower →)")
    ax2.set_ylabel("Γ(δ)")
    # two lines: the single-line title ran past the figure's right edge
    # (round-6 P0-R2 figure fix)
    ax2.set_title("discovery premium over certification\n"
                  "(finite-range evidence; no asymptotic claim)", fontsize=10)
    ax2.grid(True, which="both", alpha=0.25)
    ax2.legend(fontsize=8)

    fig.suptitle("Γ(δ): ridge-guided continuation vs stationwise ablation "
                 "vs cert-only — closed w=0.49: SAFE ABSTENTION "
                 "(closure not certified); ablation "
                 f"{g['closed_door']['cold_start']['solved']}/"
                 f"{g['closed_door']['cold_start']['stations']} stations, "
                 "outside-wall funnel only", fontsize=9, y=0.995)
    fig.tight_layout(rect=(0, 0.012, 1, 0.955))
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from prov import stamp_figure
    stamp_figure(fig, g.get("provenance", {}))
    out = os.path.join(FIGDIR, "gamma_delta.png")
    fig.savefig(out, dpi=110)
    from prov import write_sidecar
    write_sidecar(out, g.get("provenance", {}))
    print(out)


if __name__ == "__main__":
    main()
