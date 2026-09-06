"""P3.7 figure: symmetry/domain-matched arms vs continuation (reads
p37_matched.json + ridge_continuation.json; sidecar carries both
provenances, p37 first).

Left: pose-query first-success per width — all four matched arms NEVER at
cap (staggered x markers), continuation certified totals below.
Right: the pair-op currency at cap (w=0.505..0.58), matched arms vs
continuation's core+certificate pair_ops (diagnostic excluded, round-6
billing split); continuation's full (incl. diag) value shown hollow.

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/p37_plots.py
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
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

COLORS = {"iso_uniform_q": "tab:red", "aniso_uniform_q": "tab:orange",
          "aniso_generic_q": "tab:olive", "aniso_pair_q": "tab:blue"}
LABELS = {"iso_uniform_q": "iso uniform (matched ROI+quotient)",
          "aniso_uniform_q": "aniso uniform (matched)",
          "aniso_generic_q": "aniso generic |rho| (matched)",
          "aniso_pair_q": "aniso pair-aware (matched)"}


def main():
    with open(os.path.join(TABDIR, "p37_matched.json")) as f:
        g = json.load(f)
    with open(os.path.join(TABDIR, "ridge_continuation.json")) as f:
        rc = json.load(f)
    cap = max(g["budgets"])
    arms = list(COLORS)
    ws = [l["w"] for l in g["summary"]]

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13.0, 5.2))

    never_y = {"iso_uniform_q": 1.35, "aniso_uniform_q": 1.55,
               "aniso_generic_q": 1.78, "aniso_pair_q": 2.05}
    for arm in arms:
        xs, ys, nx = [], [], []
        for line in g["summary"]:
            b = line[arm]
            (nx if b is None else xs).append(line["w"])
            if b is not None:
                ys.append(b)
        if xs:
            ax.plot(xs, ys, "o-", color=COLORS[arm], label=LABELS[arm])
        if nx:
            ax.scatter(nx, [cap * never_y[arm]] * len(nx), marker="x",
                       s=90, color=COLORS[arm],
                       label=(LABELS[arm] + ": never ≤ cap"
                              if not xs else None))
    ax.plot(ws, [l["continuation_total"] for l in g["summary"]],
            "s-", color="tab:green", lw=2,
            label="continuation v4.1 (CERTIFIED path, total)")
    ax.axhline(cap, color="gray", ls=":", lw=0.8)
    ax.text(0.506, cap * 1.06, f"budget cap {cap}", fontsize=8,
            color="gray")
    ax.set_yscale("log")
    ax.set_xlabel("door width w [m]")
    ax.set_ylabel("pose queries")
    ax.set_title("P3.7 matched domain (ROI = seed box, [0,π) quotient):\n"
                 "all matched volumetric arms NEVER; kill condition not "
                 "hit (5/5 widths)", fontsize=9)
    ax.legend(fontsize=8, loc="center left")
    ax.grid(True, which="both", alpha=0.25)

    cont_cert = {round(r["w"], 4): r["pair_ops_cert"]
                 for r in rc["rows"]
                 if r.get("status") == "CERTIFIED_REACHABLE"}
    cont_all = {round(r["w"], 4): r["pair_ops"]
                for r in rc["rows"]
                if r.get("status") == "CERTIFIED_REACHABLE"}
    for arm in arms:
        ys = []
        for w in ws:
            r = next(x for x in g["rows"]
                     if x["kind"] == "main" and x["w"] == w
                     and x["arm"] == arm)
            ys.append(r["pair_ops"])
        ax2.plot(ws, ys, "x-", color=COLORS[arm], mew=2,
                 label=LABELS[arm] + " @cap (unfinished)")
    ax2.plot(ws, [cont_cert[round(w, 4)] for w in ws], "s-",
             color="tab:green", lw=2,
             label="continuation core+cert (diag excluded)")
    ax2.plot(ws, [cont_all[round(w, 4)] for w in ws], "s--",
             color="tab:green", alpha=0.5,
             label="continuation incl. diagnostic (30–42% diag)")
    ax2.set_yscale("log")
    ax2.set_xlabel("door width w [m]")
    ax2.set_ylabel("primitive-pair evaluations")
    ax2.set_title("pair-op currency: matched arms at cap vs continuation\n"
                  "(billing split per round-6; baselines unfinished → "
                  "lower bounds)", fontsize=9)
    ax2.legend(fontsize=8)
    ax2.grid(True, which="both", alpha=0.25)

    fig.tight_layout(rect=(0, 0.015, 1, 1))
    from prov import stamp_figure, write_sidecar
    provs = [g.get("provenance", {}), rc.get("provenance", {})]
    stamp_figure(fig, provs)
    out = os.path.join(FIGDIR, "p37_matched.png")
    os.makedirs(FIGDIR, exist_ok=True)
    fig.savefig(out, dpi=110)
    write_sidecar(out, provs)
    print(out)


if __name__ == "__main__":
    main()
