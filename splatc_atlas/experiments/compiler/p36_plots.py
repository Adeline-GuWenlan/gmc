"""Supplemental fairness figure (round-6 P0-R2): the Gate B chart WITH the
pair-aware ProbeTree arm, plus the 2x2 wide-door control matrix.  Reads
p36_fairness.json (pair-aware mains + controls) and p3_matched_budget.json
(the other three volumetric arms + continuation); sidecar carries BOTH
provenances, p36 first.

The pair-aware arm is a DIAGNOSTIC arm (isolates pair information without
continuation), not a strongest-nearest-neighbor baseline; the title says so.

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/p36_plots.py
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

COLORS = {"uniform": "tab:red", "generic": "tab:orange",
          "contact": "tab:olive", "pair": "tab:blue"}


def main():
    with open(os.path.join(TABDIR, "p36_fairness.json")) as f:
        g36 = json.load(f)
    with open(os.path.join(TABDIR, "p3_matched_budget.json")) as f:
        g3 = json.load(f)
    cap = max(g3["budgets"])

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13.0, 5.2),
                                  gridspec_kw={"width_ratios": [1.25, 1.0]})

    # left: thin de-aligned mains, all four volumetric arms + continuation.
    # never-markers get one staggered row per policy — at a shared height
    # the later arms fully occlude the earlier ones
    never_y = {"uniform": 1.35, "generic": 1.55, "contact": 1.78,
               "pair": 2.05}
    for pol in ("uniform", "generic", "contact"):
        xs, ys, nx = [], [], []
        for line in g3["summary"]:
            b = line[pol]
            (nx if b is None else xs).append(line["w"])
            if b is not None:
                ys.append(b)
        if xs:
            ax.plot(xs, ys, "o-", color=COLORS[pol], label=f"{pol} probe")
        if nx:
            ax.scatter(nx, [cap * never_y[pol]] * len(nx), marker="x", s=90,
                       color=COLORS[pol],
                       label=f"{pol}: never ≤ cap" if not xs else None)
    pw = [r for r in g36["rows"] if r["kind"] == "pair_main"]
    nx = [r["w"] for r in pw if r["first_success_budget"] is None]
    xs = [(r["w"], r["first_success_budget"]) for r in pw
          if r["first_success_budget"] is not None]
    if xs:
        ax.plot([v[0] for v in xs], [v[1] for v in xs], "o-",
                color=COLORS["pair"], label="pair-aware probe (diagnostic)")
    if nx:
        ax.scatter(nx, [cap * never_y["pair"]] * len(nx), marker="x", s=90,
                   color=COLORS["pair"],
                   label="pair-aware (diagnostic): never ≤ cap")
    ax.plot([l["w"] for l in g3["summary"]],
            [l["continuation_total"] for l in g3["summary"]],
            "s-", color="tab:green", lw=2,
            label="continuation v4.1 (CERTIFIED path, total)")
    ax.axhline(cap, color="gray", ls=":", lw=0.8)
    ax.text(0.506, cap * 1.06, f"budget cap {cap}", fontsize=8, color="gray")
    ax.set_yscale("log")
    ax.set_xlabel("door width w [m]")
    ax.set_ylabel("pose queries")
    ax.set_title("thin de-aligned mains: all volumetric arms vs continuation\n"
                 "(vs frozen isotropic ProbeTree family; un-quotiented domain "
                 "≈2.98× continuation seed box — see report §4 domain note)",
                 fontsize=9)
    ax.legend(fontsize=8, loc="center left")
    ax.grid(True, which="both", alpha=0.25)

    # right: 2x2 wide-door control matrix, first-success budget per policy
    ctrl = [r for r in g36["rows"] if r["kind"] == "control_2x2"]
    cells = sorted({(r["w"], r["dy0"], r["tilt_deg"]) for r in ctrl})
    labels = [f"w={w:.2f}\n{'aligned' if dy == 0 and tl == 0 else 'de-al.'}"
              for w, dy, tl in cells]
    polices = ("uniform", "generic", "contact", "pair")
    width = 0.19
    for j, pol in enumerate(polices):
        xs, ys, nx = [], [], []
        for i, cell in enumerate(cells):
            r = next(r for r in ctrl if (r["w"], r["dy0"], r["tilt_deg"])
                     == cell and r["policy"] == pol)
            b = r["first_success_budget"]
            x = i + (j - 1.5) * width
            (nx if b is None else xs).append(x)
            if b is not None:
                ys.append(b)
        if xs:
            ax2.bar(xs, ys, width=width, color=COLORS[pol], label=pol)
        if nx:
            ax2.scatter(nx, [cap * 1.5] * len(nx), marker="x", s=70,
                        color=COLORS[pol],
                        label=(f"{pol}: never" if not xs else None))
    ax2.axhline(cap, color="gray", ls=":", lw=0.8)
    ax2.set_yscale("log")
    ax2.set_xticks(range(len(cells)))
    ax2.set_xticklabels(labels, fontsize=8)
    ax2.set_ylabel("first-success budget")
    ax2.set_title("2x2 wide-door controls: de-alignment alone does not\n"
                  "defeat volumetric arms; thinness is isolated", fontsize=9)
    ax2.legend(fontsize=8)
    ax2.grid(True, which="both", axis="y", alpha=0.25)

    fig.tight_layout(rect=(0, 0.015, 1, 1))
    from prov import stamp_figure, write_sidecar
    provs = [g36.get("provenance", {}), g3.get("provenance", {})]
    stamp_figure(fig, provs)
    out = os.path.join(FIGDIR, "p36_fairness.png")
    os.makedirs(FIGDIR, exist_ok=True)
    fig.savefig(out, dpi=110)
    write_sidecar(out, provs)
    print(out)


if __name__ == "__main__":
    main()
