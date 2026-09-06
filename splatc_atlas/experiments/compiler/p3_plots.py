"""Gate B figure: matched-budget first-success vs width per policy, with
the continuation-v4.1 certified totals as reference (reads
p3_matched_budget.json; provenance-stamped from the same JSON).

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/p3_plots.py
"""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = os.path.join(os.path.dirname(__file__), "..", "..")
TABDIR = os.path.join(BASE, "results", "tables")
FIGDIR = os.path.join(BASE, "results", "figures")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main():
    with open(os.path.join(TABDIR, "p3_matched_budget.json")) as f:
        g = json.load(f)
    cap = max(g["budgets"])
    fig, ax = plt.subplots(figsize=(8.5, 5.2))
    colors = {"uniform": "tab:red", "generic": "tab:orange",
              "contact": "tab:olive"}
    for pol in ("uniform", "generic", "contact"):
        xs, ys, nx = [], [], []
        for line in g["summary"]:
            b = line[pol]
            if b is None:
                nx.append(line["w"])
            else:
                xs.append(line["w"])
                ys.append(b)
        ax.plot(xs, ys, "o-", color=colors[pol],
                label=f"{pol} probe (first correct REACHABLE)")
        if nx:
            ax.scatter(nx, [cap * 1.6] * len(nx), marker="x", s=90,
                       color=colors[pol])
    ax.plot([l["w"] for l in g["summary"]],
            [l["continuation_total"] for l in g["summary"]],
            "s-", color="tab:green", lw=2,
            label="continuation v4.1 (CERTIFIED path, total)")
    ax.axhline(cap, color="gray", ls=":", lw=0.8)
    ax.text(0.506, cap * 1.08, f"budget cap {cap}", fontsize=8, color="gray")
    ax.set_yscale("log")
    ax.set_xlabel("door width w [m]  (delta = (w-0.5)/2)")
    ax.set_ylabel("pose queries")
    ax.set_title("Gate B matched-budget: continuation vs volume probes, "
                 "G1 de-aligned (x = never within cap)")
    ax.legend(fontsize=9)
    ax.grid(True, which="both", alpha=0.25)
    fig.tight_layout()
    from prov import stamp_figure
    stamp_figure(fig, g.get("provenance", {}))
    out = os.path.join(FIGDIR, "p3_matched_budget.png")
    fig.savefig(out, dpi=110)
    from prov import write_sidecar
    write_sidecar(out, g.get("provenance", {}))
    print(out)


if __name__ == "__main__":
    main()
