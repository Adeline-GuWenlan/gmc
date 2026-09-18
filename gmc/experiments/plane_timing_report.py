# gmc/experiments/plane_timing_report.py
"""Amendment 3 §P4: collect every stage timing, fit what can honestly be fitted, draw the figures.

**Claims boundary:** the hall numbers are on the plane-floor scene, a **user-approved manual
scene-definition change** (sound with respect to the edited scene only); P2/P3's cases rest on criterion 3
**relaxed by user decision D1** (crit1 ∧ crit3 = 0.9 m² as built, 1.2 m² with every phantom cell
deleted). Amendment 2's runs are on the unedited scene (+ the Amendment 1 floor rule) and carry only
``compile_seconds`` / ``query_seconds``: they enter the compile and query series only, marked "A2".

Steps (both read files only; run from ``gmc/`` with ``PYTHONPATH=src:experiments``):

``collect``  -> ``results/height/timing/records.json``: one row per compile/query unit (P2, P3, A2, the P4
             sweep), one per ``project`` record (runs, searches, sweep), the per-scene prep records, the
             hosts, the ``sacct`` snapshot.
``report``   -> ``results/height/timing/timing_report.json`` + ``figs/*.png``. No fit on fewer than
             :data:`gmc.height.timingreport.MIN_FIT_POINTS` points; every fit states n and its x range;
             UNKNOWN runs are drawn hollow (their query time is a lower bound) and never fitted.
"""
import argparse
import json
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from gmc.height.timingreport import (COMPILE_STAGES, day_capacity, linear_fit, power_fit, row_from_run,
                                     row_from_sweep, stage_group)

RES = Path("results/height/timing")
FIGS = RES / "figs"
SWEEP = RES / "sweep"
LANES = ("prep", "cyl", "sweeper", "uav")

TIMED = [("results/height/plane/shared/sweeper.json", "P2 shared", "P2 sweeper"),
         ("results/height/plane/shared/uav.json", "P2 shared", "P2 uav"),
         ("results/height/plane/shared/cylinder.json", "P2 shared", "P2 cylinder"),
         ("results/height/plane/shared/cylinder_b2.json", "P2 shared", "P2 cylinder b2"),
         ("results/height/plane/long/sweeper.json", "P3 long", "P3 sweeper"),
         ("results/height/plane/long/uav.json", "P3 long", "P3 uav")] + \
        [(f"results/height/plane/long/cyl_ladder/rung{k}/cylinder.json", "P3 ladder", f"P3 rung {k}")
         for k in range(5)] + \
        [("results/height/plane/long/cyl_ladder/rung3/cylinder_b2.json", "P3 ladder", "P3 rung 3 b2")]
A2 = [(f"results/height/percase/{r}/{r}.json", "A2", f"A2 {r}") for r in ("sweeper", "cylinder", "uav")]
SEARCHES = [("results/height/plane/p2a_search.json", "P2 search"),
            ("results/height/plane/p2d_evidence.json", "P2 evidence"),
            ("results/height/plane/long/p3_search.json", "P3 search")]

CLAIMS = ("plane floor = user-approved manual scene edit (sound wrt the edited scene only); "
          "P2/P3 cases use criterion 3 relaxed by user decision D1")

# dataviz reference palette. Robots: categorical slots 1-3 (all-pairs validated) + a shape each.
# Stage groups: slots 4-7, adjacent pairs only (stacked bars).
ROBOT = {"sweeper": ("#2a78d6", "o"), "cylinder": ("#eb6834", "s"), "uav": ("#1baf7a", "^")}
GROUP_COLOR = {"prep": "#eda100", "compile": "#e87ba4", "query": "#008300", "prove": "#4a3aa7"}
MODE_COLOR = {"cold": "#2a78d6", "warm1": "#eb6834", "warm2": "#1baf7a"}
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["DejaVu Sans"], "font.size": 9,
                     "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
                     "text.color": INK, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
                     "axes.edgecolor": GRID, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "axes.axisbelow": True})


def _load(p):
    return json.loads(Path(p).read_text())


def _robot_of(label):
    return (label or "").split(":")[-1].split()[0]


def _jsonl(lane):
    p = SWEEP / f"{lane}.jsonl"
    if not p.exists():
        return []
    out = []
    for line in p.read_text().splitlines():
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:        # a lane killed mid-write leaves at most one broken line
            pass
    return out


# ------------------------------------------------------------------------------------ collect
def step_collect(args):
    sacct = _load(RES / "sacct_runs.json")["runs"]
    runs, proj, prep, hosts = [], [], [], {}

    def add_projects(timing, source):
        for rec in (timing or {}).get("stages", []):
            if rec["stage"] == "project" and rec["sizes"].get("n_supports") is not None:
                w = rec["sizes"].get("window")
                proj.append({"source": source, "robot": _robot_of(rec.get("label")), "label": rec.get("label"),
                             "n_supports": rec["sizes"]["n_supports"], "window": w,
                             "window_area_m2": (w[2] - w[0]) * (w[3] - w[1]) if w else None,
                             "seconds": rec["seconds"]})
            elif rec["stage"] in ("scene_load", "floor_replace"):
                prep.append({"source": source, "stage": rec["stage"], "label": rec.get("label"),
                             "seconds": rec["seconds"], "sizes": rec["sizes"]})

    for path, src, label in TIMED + A2:
        if not Path(path).exists():
            print("missing", path)
            continue
        d = _load(path)
        r = row_from_run(d, source=src, label=label, path=path)
        s = sacct.get(path, {})
        r.update(node=s.get("node"), maxrss_gib=s.get("maxrss_gb"), job=s.get("job"),
                 elapsed_s=s.get("elapsed_s"))
        runs.append(r)
        add_projects(d.get("timing"), src)
    for path, src in SEARCHES:
        if Path(path).exists():
            add_projects(_load(path).get("timing"), src)
    for lane in LANES:
        for u in _jsonl(lane):
            if u.get("kind") == "lane_meta":
                hosts[lane] = u.get("host_info")
            if u.get("kind") != "unit":
                continue
            add_projects(u.get("timing"), "P4 sweep")
            if u.get("experiment") == "prep":
                continue
            if u.get("status") or u.get("mode") in ("warm_compile", "compile"):
                r = row_from_sweep(u)
                r.update(node=(hosts.get(lane) or {}).get("host"), peak_growth_gib=u.get("peak_growth_gb"),
                         mem=u.get("mem"), label=f"{u.get('experiment')}:{u.get('mode')}:{u.get('label', '')}")
                runs.append(r)
    out = {"claims_boundary": CLAIMS, "runs": runs, "project": proj, "prep": prep, "hosts": hosts,
           "sacct_note": _load(RES / "sacct_runs.json")["note"],
           "counts": {"runs": len(runs), "project": len(proj), "prep": len(prep),
                      "sweep_rows": sum(1 for r in runs if r["source"] == "P4-sweep")}}
    RES.mkdir(parents=True, exist_ok=True)
    (RES / "records.json").write_text(json.dumps(out, indent=1, default=float) + "\n")
    print(json.dumps(out["counts"]))


# ------------------------------------------------------------------------------------ report
def _fit_both(x, y):
    return {"power": power_fit(x, y), "linear": linear_fit(x, y)}


def _xy(rows, xkey, ykey):
    pts = [(r[xkey], r[ykey]) for r in rows if r.get(xkey) and r.get(ykey)]
    return [p[0] for p in pts], [p[1] for p in pts]


def fits_and_tables(R):
    runs = R["runs"]
    timed = [r for r in runs if r["has_stage_timing"]]
    rep = {"claims_boundary": CLAIMS, "fits": {}, "tables": {}}
    F = rep["fits"]

    # -- compile: every unit whose compile finished (UNKNOWN runs' compile did finish)
    comp = [r for r in runs if r.get("compile_seconds") and r.get("n_supports")]
    runs_only = [r for r in comp if r["source"] != "P4-sweep"]
    F["compile_vs_supports_runs_only"] = _fit_both(*_xy(runs_only, "n_supports", "compile_seconds"))
    F["compile_vs_supports_all"] = _fit_both(*_xy(comp, "n_supports", "compile_seconds"))
    for st in COMPILE_STAGES:
        rows = [dict(r, y=r["stages"][st]) for r in timed if r["stages"].get(st) and r.get("n_supports")]
        F[f"{st}_vs_supports"] = power_fit(*_xy(rows, "n_supports", "y"))
    F["compile_per_robot"] = {rb: power_fit(*_xy([r for r in comp if r["robot"] == rb], "n_supports",
                                                  "compile_seconds")) for rb in ROBOT}
    F["compile_distinct_supports"] = int(len({r["n_supports"] for r in comp}))

    # -- project: per robot, over every recorded projection
    F["project_vs_supports"] = {rb: power_fit(*_xy([p for p in R["project"] if p["robot"] == rb],
                                                    "n_supports", "seconds")) for rb in ROBOT}
    # projection scans all 7.1 M splats whatever the window: a fixed cost plus a per-support term
    F["project_vs_supports_linear"] = {rb: linear_fit(*_xy([p for p in R["project"] if p["robot"] == rb],
                                                           "n_supports", "seconds")) for rb in ROBOT}

    # -- query: finished queries only; single runs vs sweep kept apart
    fin = [r for r in runs if r.get("query_seconds") and r.get("status") == "REACHABLE"]
    F["query_vs_supports_single_runs"] = {rb: power_fit(*_xy(
        [r for r in fin if r["robot"] == rb and r["source"] != "P4-sweep"], "n_supports", "query_seconds"))
        for rb in ROBOT}
    sw = [r for r in fin if r["source"] == "P4-sweep"]
    F["query_vs_distance_fixed_map"] = {f"{rb}:{m}": power_fit(*_xy(
        [r for r in sw if r["robot"] == rb and r["experiment"] in ("dist", "cyl_warm") and r["mode"] == m],
        "ab_dist_m", "query_seconds")) for rb in ROBOT for m in ("cold", "warm1", "warm2")}
    F["query_vs_supports_fixed_pair"] = {rb: power_fit(*_xy(
        [r for r in sw if r["robot"] == rb and r["experiment"] == "win"], "n_supports", "query_seconds"))
        for rb in ROBOT}
    # -- prove
    ver = [dict(r, y=r["stages"]["verify_curve"]) for r in timed if (r["stages"] or {}).get("verify_curve")]
    rpl = [dict(r, y=r["stages"]["replay3d"]) for r in timed if (r["stages"] or {}).get("replay3d")]
    F["verify_vs_distance_fixed_map"] = {rb: power_fit(*_xy(
        [r for r in ver if r["robot"] == rb and r.get("experiment") == "dist" and r["mode"] == "cold"],
        "ab_dist_m", "y")) for rb in ROBOT}
    # verify grows with route length, so it is fitted only where one variable moves at a time:
    # supports at a fixed pair (the window series) and distance on a fixed map (above)
    F["verify_vs_supports_fixed_pair"] = {rb: power_fit(*_xy(
        [r for r in ver if r["robot"] == rb and r.get("experiment") == "win"], "n_supports", "y")) for rb in ROBOT}
    F["replay3d_vs_distance"] = power_fit(*_xy(rpl, "ab_dist_m", "y"))

    # -- compile once, query many: per fixed map
    wc = defaultdict(lambda: {"compile": [], "rows": []})
    for r in sw + [r for r in runs if r["source"] == "P4-sweep" and r["mode"] == "warm_compile"]:
        if r.get("experiment") not in ("dist", "cyl_warm"):
            continue
        key = f"{r['experiment']}:{r['robot']}"
        if r["mode"] == "warm_compile":
            wc[key]["compile"].append(r["compile_seconds"])
        elif r["mode"] in ("cold", "warm1", "warm2"):
            wc[key]["rows"].append({"mode": r["mode"], "ab_dist_m": r["ab_dist_m"], "query_s": r["query_seconds"],
                                    "compile_s": r.get("compile_seconds"), "rounds": r.get("refinement_rounds"),
                                    "support_calls": r.get("query_support_calls"),
                                    "verify_s": (r["stages"] or {}).get("verify_curve"),
                                    "replay3d_s": (r["stages"] or {}).get("replay3d"),
                                    "success": r["success"], "status": r["status"]})
    # UNKNOWN warm/cold queries on the fixed maps, kept as lower bounds
    for r in runs:
        if r["source"] == "P4-sweep" and r.get("status") and r["status"] != "REACHABLE" and \
                r.get("experiment") in ("dist", "cyl_warm"):
            wc[f"{r['experiment']}:{r['robot']}"]["rows"].append(
                {"mode": r["mode"], "ab_dist_m": r["ab_dist_m"], "query_s": r["query_seconds"],
                 "status": r["status"], "lower_bound": True, "rounds": r.get("refinement_rounds")})
    T = rep["tables"]
    T["compile_once_query_many"] = {}
    for key, v in wc.items():
        rows = sorted(v["rows"], key=lambda z: (z["mode"], z["ab_dist_m"] or 0))
        comp_once = float(np.mean(v["compile"])) if v["compile"] else None
        cold_c = [z["compile_s"] for z in rows if z["mode"] == "cold" and z.get("compile_s")]
        q = {m: [z["query_s"] for z in rows if z["mode"] == m and z.get("success")] for m in ("cold", "warm1", "warm2")}
        T["compile_once_query_many"][key] = {
            "compile_once_s": comp_once, "cold_compile_s": cold_c,
            "cold_compile_cv": float(np.std(cold_c) / np.mean(cold_c)) if len(cold_c) > 1 else None,
            "median_query_s": {m: (float(np.median(x)) if x else None) for m, x in q.items()},
            "query_over_compile_median": {m: (float(np.median(x)) / comp_once if x and comp_once else None)
                                          for m, x in q.items()},
            "rows": rows}

    # -- calibration: the sweep's cold long-goal run against P3's own run of the same map and pair
    T["calibration"] = []
    for rb in ("sweeper", "uav"):
        p3 = next((r for r in runs if r["label"] == f"P3 {rb}"), None)
        c = [r for r in sw if r["robot"] == rb and r.get("experiment") == "dist" and r["mode"] == "cold"
             and r.get("ab_dist_m") and abs(r["ab_dist_m"] - (p3 or {}).get("ab_dist_m", -1)) < 1e-3]
        if p3 and c:
            T["calibration"].append({"robot": rb, "p3_node": p3.get("node"),
                                     "p3_compile_s": p3["compile_seconds"], "p3_query_s": p3["query_seconds"],
                                     "sweep_compile_s": c[0]["compile_seconds"], "sweep_query_s": c[0]["query_seconds"],
                                     "compile_ratio": c[0]["compile_seconds"] / p3["compile_seconds"],
                                     "query_ratio": c[0]["query_seconds"] / p3["query_seconds"]})

    # -- where the wall clock goes, per timed run (single runs + sweep cold units)
    T["where_time_goes"] = []
    for r in timed:
        if r["source"] == "P4-sweep" and r["mode"] != "cold":
            continue
        g = defaultdict(float)
        for st, s in (r["stages"] or {}).items():
            g[stage_group(st)] += s
        tot = sum(g.values())
        if tot <= 0 or not g.get("compile"):
            continue
        T["where_time_goes"].append({"label": r["label"], "source": r["source"], "robot": r["robot"],
                                     "n_supports": r["n_supports"], "ab_dist_m": r["ab_dist_m"],
                                     "status": r["status"], "total_s": tot,
                                     "seconds": dict(g), "share": {k: v / tot for k, v in g.items()},
                                     "stages": r["stages"]})

    # -- per-scene prep, from the sweep's prep lane (the only place it was timed)
    T["prep_per_scene"] = [p for p in R["prep"] if p["source"] == "P4 sweep"]
    seen = set()
    T["project_hall"] = []
    for p in R["project"]:
        if (p.get("label") or "").startswith("hall:") and p["robot"] not in seen:
            seen.add(p["robot"])
            T["project_hall"].append(p)

    # -- the day: compile capacity (extrapolation), with the hall's own support counts beside it
    cf = F["compile_vs_supports_all"]["power"]
    rep["day"] = {"compile_capacity": day_capacity(cf, 86400.0),
                  "hall_supports": {p["robot"]: p["n_supports"] for p in T["project_hall"]},
                  "hall_compile_hours_extrapolated": (
                      {p["robot"]: cf["coef"] * p["n_supports"] ** cf["exponent"] / 3600.0 for p in T["project_hall"]}
                      if cf.get("exponent") else None),
                  "compile_memory_gib_per_100k": [
                      {"robot": r["robot"], "n_supports": r["n_supports"],
                       "gib_per_100k": r["peak_growth_gib"] / r["n_supports"] * 1e5}
                      for r in runs if r.get("peak_growth_gib") and r.get("n_supports")]}
    return rep


# ------------------------------------------------------------------------------------ figures
def _mark(ax, rows, xkey, ykey, *, fit=None, fit_label=None):
    for r in rows:
        x, y = r.get(xkey), r.get(ykey)
        if not x or not y:
            continue
        col, mk = ROBOT.get(r["robot"], (INK2, "o"))
        lb = r.get("query_is_lower_bound") and ykey in ("query_seconds", "q")
        sweep = r["source"] == "P4-sweep"
        ax.plot([x], [y], marker=mk, ms=5 if sweep else 8, ls="none",
                mfc="none" if lb else col, mec=col if (sweep or lb) else INK, mew=1.4 if lb else (0.0 if sweep else 0.9),
                alpha=0.85 if sweep else 1.0, zorder=3 if sweep else 4)
        if r["source"] == "A2":
            ax.annotate("A2", (x, y), textcoords="offset points", xytext=(5, 3), fontsize=7, color=INK2)
    if fit and fit.get("exponent") is not None:
        xs = np.geomspace(*fit["x_range"], 50)
        ax.plot(xs, fit["coef"] * xs ** fit["exponent"], ls="--", lw=1.2, color=INK2, zorder=2)
        ax.text(0.03, 0.97, (fit_label or "") + f"∝ x^{fit['exponent']:.2f}  (n={fit['n']}, "
                f"x {fit['x_range'][0]:.3g}–{fit['x_range'][1]:.3g})",
                transform=ax.transAxes, va="top", fontsize=7.5, color=INK2)
    elif fit is not None:
        ax.text(0.03, 0.97, f"no fit: n={fit['n']} < 4", transform=ax.transAxes, va="top", fontsize=7.5,
                color=INK2)


def _legend_panel(ax, extra=""):
    ax.axis("off")
    hs = [plt.Line2D([], [], marker=mk, ls="none", mfc=c, mec=INK, mew=0.9, ms=8, label=rb)
          for rb, (c, mk) in ROBOT.items()]
    hs += [plt.Line2D([], [], marker="o", ls="none", mfc=INK2, mec=INK, mew=0.9, ms=8,
                      label="P2/P3 run (one compile + one query)"),
           plt.Line2D([], [], marker="o", ls="none", mfc=INK2, mec=INK2, mew=0, ms=5, label="P4 sweep unit"),
           plt.Line2D([], [], marker="o", ls="none", mfc="none", mec=INK2, mew=1.4, ms=8,
                      label="hollow: UNKNOWN, query time is a lower bound"),
           plt.Line2D([], [], ls="--", color=INK2, label="power fit (≥ 4 finished points)")]
    ax.legend(handles=hs, loc="upper left", frameon=False, fontsize=8)
    ax.text(0.0, 0.18, "'A2' = Amendment 2, unedited scene,\ncompile/query only (no stage timing)\n" + extra,
            transform=ax.transAxes, fontsize=7.5, color=INK2, va="top")


def fig_axis(R, rep, xkey, xlabel, fname, title):
    runs = R["runs"]
    fin = [r for r in runs]
    panels = [("project", "project (per robot, per window)"), ("compile_seconds", "compile (pairs + slabs + mobility)"),
              ("query_seconds", "query"), ("verify_curve", "verify_curve"), ("replay3d", "replay3d")]
    fig, axs = plt.subplots(2, 3, figsize=(13, 7.6))
    for ax, (key, lab) in zip(axs.flat, panels):
        if key == "project":
            rows = [dict(p, source=p["source"] if p["source"] == "P4 sweep" else "run", y=p["seconds"])
                    for p in R["project"]]
            rows = [dict(r, source="P4-sweep" if r["source"] == "P4 sweep" else r["source"]) for r in rows]
            _mark(ax, rows, xkey, "y")
        elif key in ("compile_seconds", "query_seconds"):
            rows = [r for r in fin if not (key == "compile_seconds" and r["mode"] in ("warm1", "warm2"))]
            fit = None
            if xkey == "n_supports" and key == "compile_seconds":
                fit = rep["fits"]["compile_vs_supports_all"]["power"]
            _mark(ax, rows, xkey, key, fit=fit, fit_label="compile ")
        else:
            rows = [dict(r, y=r["stages"][key]) for r in fin if r["has_stage_timing"] and (r["stages"] or {}).get(key)]
            _mark(ax, rows, xkey, "y")
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_title(lab, fontsize=9.5, loc="left", color=INK)
        ax.set_xlabel(xlabel)
        ax.set_ylabel("seconds (wall, one core)")
    _legend_panel(axs.flat[5])
    fig.suptitle(f"{title}\n{CLAIMS}", fontsize=10, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    FIGS.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGS / fname, dpi=130, bbox_inches="tight")
    plt.close(fig)


def fig_where(rep):
    rows = sorted(rep["tables"]["where_time_goes"], key=lambda r: (r["source"] != "P4-sweep", r["robot"],
                                                                     r["n_supports"] or 0, r["ab_dist_m"] or 0))
    if not rows:
        return
    fig, ax = plt.subplots(figsize=(11, 0.34 * len(rows) + 1.6))
    y = np.arange(len(rows))
    for g in ("prep", "compile", "query", "prove"):
        left = np.array([sum(r["share"].get(h, 0) for h in ("prep", "compile", "query", "prove")[
            :("prep", "compile", "query", "prove").index(g)]) for r in rows])
        w = np.array([r["share"].get(g, 0) for r in rows])
        ax.barh(y, w - 0.004, left=left + 0.002, color=GROUP_COLOR[g], height=0.72, label=g)
        for yi, (l, ww) in enumerate(zip(left, w)):
            if ww >= 0.09:
                ax.text(l + ww / 2, yi, f"{ww:.0%}", ha="center", va="center", fontsize=7,
                        color="#ffffff" if g in ("query", "prove") else INK)
    ax.set_yticks(y, [f"{r['label']}  ({r['robot']}, {r['n_supports']:,} sup, {r['ab_dist_m']:.1f} m)"
                      for r in rows], fontsize=7.5)
    for yi, r in enumerate(rows):
        tag = f"{r['total_s']:,.0f} s" + ("  UNKNOWN (query = lower bound)" if r["status"] == "UNKNOWN" else "")
        ax.text(1.01, yi, tag, va="center", fontsize=7.5, color=INK2, transform=ax.get_yaxis_transform())
    ax.set_xlim(0, 1)
    ax.invert_yaxis()
    ax.grid(False)
    ax.set_xlabel("share of the unit's wall clock")
    ax.legend(ncol=4, loc="lower left", bbox_to_anchor=(0, 1.0), frameon=False, fontsize=8)
    ax.set_title("Where the wall clock goes, per compile+query unit (prep = scene_load + project for this run; "
                 "per-scene denoise is separate)\n" + CLAIMS, fontsize=9, loc="left", pad=22)
    fig.tight_layout()
    fig.savefig(FIGS / "where_time_goes.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def fig_warm_cold(rep):
    T = rep["tables"]["compile_once_query_many"]
    keys = [k for k in ("dist:sweeper", "dist:uav", "cyl_warm:cylinder") if k in T]
    if not keys:
        return
    fig, axs = plt.subplots(1, len(keys), figsize=(4.6 * len(keys), 4.2), squeeze=False)
    for ax, k in zip(axs[0], keys):
        v = T[k]
        if v["compile_once_s"]:
            ax.axhline(v["compile_once_s"], color=INK2, lw=1.2, ls="-")
            ax.text(0.02, v["compile_once_s"], f" compile once: {v['compile_once_s']:.0f} s", va="bottom",
                    fontsize=7.5, color=INK2, transform=ax.get_yaxis_transform())
        for m, lab in (("cold", "query on a fresh compile"), ("warm1", "warm, pass 1"), ("warm2", "warm, pass 2")):
            pts = sorted([(z["ab_dist_m"], z["query_s"], z.get("lower_bound")) for z in v["rows"]
                          if z["mode"] == m and z.get("query_s")])
            if not pts:
                continue
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=MODE_COLOR[m], lw=2, marker="o", ms=5,
                    label=lab)
            for p in pts:
                if p[2]:
                    ax.plot([p[0]], [p[1]], marker="o", ms=9, mfc="none", mec=MODE_COLOR[m], mew=1.4)
        ax.set_yscale("log")
        ax.set_xlabel("A–B distance (m), same compiled map")
        ax.set_ylabel("seconds")
        ax.set_title(k.split(":")[1] + (" (rung 2 window)" if k.startswith("cyl") else " (long-case window)"),
                     loc="left", fontsize=9.5)
        ax.legend(frameon=False, fontsize=7.5, loc="lower right")
    fig.suptitle("Compile once, query many: the query is paid per pair, the compile once per map\n" + CLAIMS,
                 fontsize=9.5, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    fig.savefig(FIGS / "compile_once_query_many.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def fig_prep(rep):
    rows = rep["tables"]["prep_per_scene"]
    if not rows:
        return
    fig, ax = plt.subplots(figsize=(8, 0.4 * len(rows) + 1.4))
    y = np.arange(len(rows))
    ax.barh(y, [r["seconds"] for r in rows], color="#2a78d6", height=0.6)
    for yi, r in enumerate(rows):
        ax.text(r["seconds"], yi, f"  {r['seconds']:.1f} s", va="center", fontsize=8, color=INK)
    ax.set_yticks(y, [f"{r['stage']}: {r['label']}" for r in rows], fontsize=8)
    ax.invert_yaxis()
    ax.set_xlabel("seconds (wall, one core), once per scene")
    ax.set_title("Per-scene preparation (denoise / scene prep), 7.34 M splats -> 7.10 M, measured once\n" + CLAIMS,
                 fontsize=9, loc="left")
    fig.tight_layout()
    fig.savefig(FIGS / "prep_per_scene.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def _f(v, nd=1):
    if v is None:
        return "—"
    if isinstance(v, (int, np.integer)) or (isinstance(v, float) and v >= 1000):
        return f"{v:,.0f}"
    return f"{v:.{nd}f}"


def _fit_txt(f):
    if f is None:
        return "—"
    if f.get("exponent") is None:
        return f"no fit (n = {f['n']})"
    return (f"t = {f['coef']:.3g} · x^{f['exponent']:.2f}, n = {f['n']}, x {f['x_range'][0]:.3g}–"
            f"{f['x_range'][1]:.3g}, R² {f['r2']:.3f}")


def md_tables(R, rep):
    """Every table the report quotes, machine-written, so no number is hand-copied."""
    L = [f"# P4 timing tables (generated by plane_timing_report.py --step report)\n", f"_{CLAIMS}_\n"]
    T, F = rep["tables"], rep["fits"]
    L += ["## Per-scene prep (P4 sweep, prep lane, one core)\n", "| stage | step | seconds | sizes |", "|---|---|---|---|"]
    for p in T["prep_per_scene"]:
        L.append(f"| {p['stage']} | {p['label']} | {p['seconds']:.2f} | "
                 + ", ".join(f"{k}={v:,}" if isinstance(v, int) else f"{k}={v}" for k, v in p["sizes"].items()) + " |")
    L += ["", "## Hall-wide projections (P4 sweep)\n", "| robot | supports | project s |", "|---|---|---|"]
    for p in T["project_hall"]:
        L.append(f"| {p['robot']} | {p['n_supports']:,} | {p['seconds']:.1f} |")
    L += ["", "project fits (every recorded projection, per robot): "
          + "; ".join(f"{rb}: {_fit_txt(f)}" for rb, f in F["project_vs_supports"].items()), "",
          "project, linear (fixed scan + per-support term): "
          + "; ".join(f"{rb}: " + (f"t = {f['intercept']:.2f} s + {f['slope'] * 1e6:.1f} µs · N (n = {f['n']}, "
                                   f"R² {f['r2']:.3f})" if f.get("slope") is not None else f"no fit (n = {f['n']})")
                      for rb, f in F["project_vs_supports_linear"].items()), ""]
    L += ["## Every compile+query unit that ran as its own job (P2, P3, A2)\n",
          "| run | robot | supports | window m² | A–B m | pairs s | slabs s | mobility s | compile s | query s | "
          "verify s | replay3d s | status | node | MaxRSS GiB |", "|" + "---|" * 15]
    for r in [r for r in R["runs"] if r["source"] != "P4-sweep"]:
        st = r["stages"] or {}
        q = _f(r["query_seconds"]) + (" ≥" if r["query_is_lower_bound"] else "")
        L.append(f"| {r['label']} | {r['robot']} | {r['n_supports']:,} | {_f(r['window_area_m2'])} | "
                 f"{_f(r['ab_dist_m'], 2)} | {_f(st.get('compile_pairs'))} | {_f(st.get('compile_slabs'))} | "
                 f"{_f(st.get('compile_mobility'))} | {_f(r['compile_seconds'])} | {q} | {_f(st.get('verify_curve'))} | "
                 f"{_f(st.get('replay3d'))} | {r['status']}{'' if r['success'] or r['status'] != 'REACHABLE' else ' (not all gates)'} | "
                 f"{r.get('node') or '—'} | {_f(r.get('maxrss_gib'), 2)} |")
    L += ["", "## Fits\n", "| series | fit |", "|---|---|"]
    for k in ("compile_vs_supports_runs_only", "compile_vs_supports_all"):
        L.append(f"| {k} (power) | {_fit_txt(F[k]['power'])} |")
        lf = F[k]["linear"]
        if lf.get("slope") is not None:
            L.append(f"| {k} (linear) | t = {lf['intercept']:.2f} s + {lf['slope'] * 1e3:.2f} ms · N, n = {lf['n']}, "
                     f"R² {lf['r2']:.4f} |")
    for st in COMPILE_STAGES:
        L.append(f"| {st}_vs_supports | {_fit_txt(F[f'{st}_vs_supports'])} |")
    for grp in ("compile_per_robot", "query_vs_supports_single_runs", "query_vs_supports_fixed_pair",
                "verify_vs_supports_fixed_pair", "verify_vs_distance_fixed_map"):
        for rb, f in F[grp].items():
            L.append(f"| {grp} [{rb}] | {_fit_txt(f)} |")
    for k, f in F["query_vs_distance_fixed_map"].items():
        L.append(f"| query_vs_distance_fixed_map [{k}] | {_fit_txt(f)} |")
    L.append(f"| replay3d_vs_distance | {_fit_txt(F['replay3d_vs_distance'])} |")
    L += ["", "## Compile once, query many (P4 sweep)\n"]
    for k, v in T["compile_once_query_many"].items():
        L += [f"### {k}\n", f"compile once: {_f(v['compile_once_s'])} s; cold compiles of the same map: "
              + ", ".join(_f(c) for c in v["cold_compile_s"]) + f" s (CV {_f(v['cold_compile_cv'], 3)}); "
              + "median query/compile: " + ", ".join(f"{m} {_f(x, 2)}" for m, x in v["query_over_compile_median"].items()),
              "", "| mode | A–B m | query s | rounds | support calls | verify s | replay3d s | status |",
              "|---|---|---|---|---|---|---|---|"]
        for z in v["rows"]:
            L.append(f"| {z['mode']} | {_f(z['ab_dist_m'], 2)} | {_f(z.get('query_s'))}{' ≥' if z.get('lower_bound') else ''} | "
                     f"{_f(z.get('rounds'))} | {_f(z.get('support_calls'))} | {_f(z.get('verify_s'))} | "
                     f"{_f(z.get('replay3d_s'))} | {z.get('status')} |")
        L.append("")
    win = [r for r in R["runs"] if r["source"] == "P4-sweep" and r.get("experiment") == "win"]
    L += ["## One pair (rung 0, 2.43 m), growing window (P4 sweep)\n",
          "| robot | window m² | supports | project s | compile s | query s | verify s | rounds | status |",
          "|---|---|---|---|---|---|---|---|---|"]
    for r in sorted(win, key=lambda r: (r["robot"], r["window_area_m2"])):
        st = r["stages"] or {}
        L.append(f"| {r['robot']} | {_f(r['window_area_m2'])} | {r['n_supports']:,} | {_f(st.get('project'))} | "
                 f"{_f(r['compile_seconds'])} | {_f(r['query_seconds'])}{' ≥' if r['query_is_lower_bound'] else ''} | "
                 f"{_f(st.get('verify_curve'))} | {_f(r.get('refinement_rounds'))} | {r['status']} |")
    co = [r for r in R["runs"] if r["source"] == "P4-sweep" and r.get("experiment") == "compile_only"]
    L += ["", "## Compile-only on the largest maps (P4 sweep, prep lane)\n",
          "| map | robot | supports | pairs s | slabs s | mobility s | compile s | peak growth GiB |",
          "|---|---|---|---|---|---|---|---|"]
    for r in co:
        st = r["stages"] or {}
        L.append(f"| {r['label']} | {r['robot']} | {r['n_supports']:,} | {_f(st.get('compile_pairs'))} | "
                 f"{_f(st.get('compile_slabs'))} | {_f(st.get('compile_mobility'))} | {_f(r['compile_seconds'])} | "
                 f"{_f(r.get('peak_growth_gib'), 2)} |")
    L += ["", "## Calibration: the sweep's cold long-goal unit vs P3's own run\n",
          "| robot | P3 node | P3 compile s | sweep compile s | ratio | P3 query s | sweep query s | ratio |",
          "|---|---|---|---|---|---|---|---|"]
    for c in T["calibration"]:
        L.append(f"| {c['robot']} | {c['p3_node']} | {_f(c['p3_compile_s'])} | {_f(c['sweep_compile_s'])} | "
                 f"{c['compile_ratio']:.2f} | {_f(c['p3_query_s'])} | {_f(c['sweep_query_s'])} | {c['query_ratio']:.2f} |")
    d = rep["day"]
    cc = d["compile_capacity"]
    L += ["", "## The day (EXTRAPOLATION)\n",
          f"compile fit: {_fit_txt(F['compile_vs_supports_all']['power'])}; 24 h of compile ≈ "
          f"{_f(cc['n_supports'])} supports, {_f(cc['beyond_largest_measured_x'])}× beyond the largest compiled map.", "",
          "| robot | hall-wide supports | compile h (extrapolated) |", "|---|---|---|"]
    for rb, n in d["hall_supports"].items():
        h = (d["hall_compile_hours_extrapolated"] or {}).get(rb)
        L.append(f"| {rb} | {n:,} | {_f(h, 2)} |")
    (RES / "tables.md").write_text("\n".join(L) + "\n")


def step_report(args):
    R = _load(RES / "records.json")
    rep = fits_and_tables(R)
    fig_axis(R, rep, "n_supports", "supports in the robot's map", "stages_vs_supports.png",
             "Stage time vs number of supports")
    fig_axis(R, rep, "window_area_m2", "window area (m²)", "stages_vs_area.png", "Stage time vs window area")
    fig_axis(R, rep, "ab_dist_m", "A–B distance (m)", "stages_vs_distance.png", "Stage time vs A–B distance")
    fig_where(rep)
    fig_warm_cold(rep)
    fig_prep(rep)
    (RES / "timing_report.json").write_text(json.dumps(rep, indent=1, default=float) + "\n")
    md_tables(R, rep)
    print(json.dumps({k: (v.get("power") or v) if isinstance(v, dict) and "power" in v else None
                      for k, v in rep["fits"].items() if isinstance(v, dict) and "power" in v}, indent=1, default=float))
    print(json.dumps(rep["day"], indent=1, default=float))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", required=True, choices=["collect", "report"])
    args = ap.parse_args()
    {"collect": step_collect, "report": step_report}[args.step](args)


if __name__ == "__main__":
    main()
