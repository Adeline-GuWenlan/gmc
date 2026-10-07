"""F5 Tasks 1-2 (inline, json/csv only): merge the trace records into per-row facts and mechanism classes, pick the
review figures, write the review index inputs.

Run from ``gmc/`` with ``PYTHONPATH=src:experiments``.

  facts     results/aerial3dg/f5/facts.csv (one row per traced (case, robot): the three verification facts, mechanism
            class, one-line cause, code location, probes) + summary.json (counts, probe tallies, consistency with F4)
  select    results/aerial3dg/f5/fig_selection.json: every genuine / soundness / F3-targeted row, all rows of a
            by-tolerance or export class with <= 30 rows, else 30 stratified representatives (strata = region x
            robot, proportional, >= 1 each; within a stratum evenly spaced over the locus clearance) + an overview
  review    docs/aerial3dg_f5_review.md + results/aerial3dg/f5/review_verdicts.csv

Mechanism classes (each traced through the code; ``CODE`` gives the location):
  EP-FLOOR      endpoint query cell not grown: ``grow_cell``'s refined gap to a pair <= buffer + slack (the blocker's
                2-sigma top lies under the chassis bottom) and the endpoint's oracle clearance <= margin + buffer
  EP-SIDE       the same with a blocker that reaches into the body band (lateral)
  EP-SLACK      the same, but the endpoint's oracle clearance is > margin + buffer: GMC's envelope gap is more
                conservative than the oracle by more than the 1 mm buffer (genuine)
  REPLAY-AABB   own verifier CERTIFIED; the shared replay's known-space test rejects the world AABB of a swept export
                segment that leaves the route prism although every pose is inside
  REPLAY-RATE   own verifier CERTIFIED; replay geometry passes; the rate check fails by float round-off of us-scale
                turn / translation times; the 1 ms floors clear it
  POST-TIMEOUT  the 120 s limit fires inside the optional post-processing (shortcut / tighten / merge) after a
                certified safe-graph route exists (genuine)
  SIMPLIFY-ZERODIV  ``simplify`` divides by ac . ac == 0 on an a -> b -> a spike of the lifted polyline (genuine)
  CUT-WRONG     certified UNREACHABLE on a confirmed pair (soundness) -- none expected; any row is listed
  OTHER         anything the trace does not explain (genuine until explained)
"""
from __future__ import annotations

import argparse
import collections
import csv
import json
import math
from pathlib import Path

import numpy as np

F3 = Path("results/aerial3dg/f3")
F4 = Path("results/aerial3dg/f4")
F5 = Path("results/aerial3dg/f5")
DOCS = Path("../docs")
TOL_MM = 2.0
EP_SLACK_MM = .1

CODE = {
    "EP-FLOOR": "aerial3d/cells.py:121-126 (grow_cell: refine_gap <= buffer + slack -> None) -> aerial3d/api.py:330-333",
    "EP-SIDE": "aerial3d/cells.py:121-126 (grow_cell: refine_gap <= buffer + slack -> None) -> aerial3d/api.py:330-333",
    "EP-SLACK": "aerial3d/cells.py:121-126 (grow_cell) -> aerial3d/api.py:330-333; gap from envelopes.refine_gap",
    "REPLAY-AABB": "gs3d/oracle.py:165-166 (known_space.contains_aabb of the swept world AABB) -> gs3d/integration.py:47-55; "
                   "called from aerial3d/api.py:417-423",
    "REPLAY-RATE": "gs3d/validation.py:90-93 (speed / yaw-rate <= limit + 1e-9) on planner._linear_trajectory times; "
                   "aerial3d/api.py:417-423",
    "POST-TIMEOUT": "aerial3d/api.py:374-392 (shortcut / tighten / merge_corners -> query.py:225-326, each accept() = "
                    "verify_segment) under G2's 120 s alarm (aerial3dg_batch._alarm)",
    "SIMPLIFY-ZERODIV": "aerial3d/query.py:217 (simplify: t = ab.ac / ac.ac with a == c), called at aerial3d/api.py:364",
    "CUT-WRONG": "aerial3d/api.py:243-268 (_cut_certificate)",
    "OTHER": "-",
}
GENUINE = {"EP-SLACK", "POST-TIMEOUT", "SIMPLIFY-ZERODIV", "CUT-WRONG", "OTHER"}
TOLERANCE = {"EP-FLOOR", "EP-SIDE"}
EXPORT = {"REPLAY-AABB", "REPLAY-RATE"}


def group(m):
    return "genuine" if m in GENUINE else "by-tolerance" if m in TOLERANCE else "export" if m in EXPORT else "?"


def _jsonl(p):
    return [json.loads(l) for l in open(p) if l.strip()]


def load_traces(d=F5 / "trace"):
    out = {}
    for f in sorted(Path(d).glob("*.jsonl")):
        for r in _jsonl(f):
            out[(r["case_id"], r["robot"])] = r
    return out


def mm(x):
    return None if x is None else round(1e3 * float(x), 3)


def classify(r):
    """Mechanism class + the failure locus from one trace record."""
    st, rs = r["gmc_orig"].split(":", 1)
    q = r["query"]
    ep = r["endpoints"]
    if st == "UNREACHABLE":
        return "CUT-WRONG", None
    if st == "TIMEOUT":
        post = q.get("post") or {}
        if q["status"] == "TIMEOUT" and q.get("stage_at_end") in ("shortcut", "tighten", "merge_corners",
                                                                  "after_shortcut", "after_tighten"):
            return "POST-TIMEOUT", "post"
        # alarm just after post-processing (own verification / replay), post-processing ~ the whole budget
        if q["status"] == "TIMEOUT" and sum(v.get("wall_s", 0) for v in post.values()) > .5 * q["wall_s"]:
            return "POST-TIMEOUT", "post"
        # limit-edge: the re-query finished just inside the limit, with post-processing most of the time
        if q["status"] != "TIMEOUT" and sum(v.get("wall_s", 0) for v in post.values()) > .5 * q["wall_s"] \
                and q["wall_s"] > 60:
            return "POST-TIMEOUT", "post"
        return "OTHER", None
    if st == "ERROR" or rs.startswith("ZeroDivisionError"):
        if q.get("simplify_spikes"):
            return "SIMPLIFY-ZERODIV", "spike"
        return "OTHER", None
    if rs.endswith("_not_certified_free"):
        e = "start" if rs.startswith("start") else "goal"
        x = ep[e]
        c = x.get("clearance_fine_m")
        tr = x.get("grow_cell_trace") or {}
        bl = tr.get("blockers") or []
        if c is not None and c * 1e3 > TOL_MM + EP_SLACK_MM:     # F3/F4's cut: (2.0, 2.1] mm is still tolerance
            return "EP-SLACK", e
        if bl and not bl[0]["centre_below_chassis_bottom"] and tr.get("cause") == "lateral_gaussian":
            return "EP-SIDE", e
        return "EP-FLOOR", e
    if rs == "shared_replay_failed":
        rp = q.get("replay") or {}
        if rp.get("geometry_passed") is False and rp.get("geometry_reason") == "map_unknown":
            return "REPLAY-AABB", "edge"
        if rp.get("geometry_passed") and (rp.get("both_floors") or {}).get("passed"):
            return "REPLAY-RATE", "kin"
        return "OTHER", None
    return "OTHER", None


def _fmt(x, nd=2):
    return "-" if x is None else f"{x:.{nd}f}"


def locus_clearance(r, mech, end):
    ep = r["endpoints"]
    if mech.startswith("EP-") and end in ("start", "goal"):
        return ep[end].get("clearance_fine_m"), f"{end} endpoint oracle clearance (row robot, 0.01 mm bisection)"
    return r["astar"].get("lateral_fine_m"), "A* route lateral clearance (row robot, chassis +3 mm, margin 0)"


def stage_label(q):
    st = q.get("stage_at_end")
    return st or "-"


def cause_line(r, mech, end):
    q, ep, pr = r["query"], r["endpoints"], r["probes"]
    if mech in ("EP-FLOOR", "EP-SIDE", "EP-SLACK"):
        x = ep[end]
        tr = x.get("grow_cell_trace") or {}
        bl = (tr.get("blockers") or [{}])[0]
        c = x.get("clearance_fine_m")
        top = bl.get("top_z_2sigma")
        where = "floor splat" if bl.get("centre_below_chassis_bottom") else "Gaussian"
        s = (f"{end} is {_fmt(mm(c))} mm clear of {where} {bl.get('scene_id')} (2σ top {_fmt(mm(top))} mm); "
             f"GMC's refined gap {_fmt(mm(tr.get('min_refined_gap_m')), 3)} mm ≤ buffer 1 mm + slack -> no endpoint cell")
        if mech == "EP-SLACK":
            s += "; clearance > margin + buffer, so the envelope gap is the conservative part"
        return s
    if mech == "REPLAY-AABB":
        fe = (q.get("replay") or {}).get("failed_edge") or {}
        ex = fe.get("excess_beyond_prism_m") or [0, 0, 0]
        ax = int(np.argmax(ex))
        return (f"exported edge {fe.get('k')}: its swept world AABB leaves the route prism by {1e3 * ex[ax]:.1f} mm in "
                f"{'uvz'[ax]} although every exported pose is inside (own verifier CERTIFIED)")
    if mech == "REPLAY-RATE":
        rp = q.get("replay") or {}
        v = (rp.get("kin_violations") or [{}])[0]
        kind = "in-place turn" if v.get("translation_m") == 0 else "translation"
        size = abs(v.get("yaw_change_rad") or 0) * 1e6 if kind == "in-place turn" else (v.get("translation_m") or 0) * 1e6
        exc = max(v.get("yaw_rate_excess") or 0, v.get("speed_excess") or 0)
        return (f"{kind} of {size:.2f} {'µrad' if kind == 'in-place turn' else 'µm'} at export step {v.get('step')}: "
                f"rate over the limit by {exc:.1e} (> 1e-9) from float round-off; geometry passed; 1 ms floors pass")
    if mech == "POST-TIMEOUT":
        post = q.get("post") or {}
        st = stage_label(q)
        sh = post.get("shortcut") or {}
        ti = post.get("tighten") or {}
        n = sh.get("accept_calls", 0) + ti.get("accept_calls", 0) + (post.get("merge_corners") or {}).get("accept_calls", 0)
        acc_s = sum(v.get("accept_s", 0) for v in post.values())
        vo = pr.get("verdict_only") or pr.get("b0_verdict_only") or {}
        bu = pr.get("budget") or pr.get("b0_budget") or {}
        parts = ", ".join(f"{k} {v.get('accept_calls', 0)} accepts {v.get('accept_s', 0):.0f} s"
                          for k, v in post.items() if v.get("calls"))
        lim = q.get("timeout_s") or 120
        where = f"{lim:g} s alarm inside {st}" if q["status"] == "TIMEOUT" else f"re-query {q['status']} in {q['wall_s']:.0f} s"
        return (f"{where} (post-processing: {parts}); verdict-only {vo.get('status')} in "
                f"{_fmt(vo.get('wall_s'), 1)} s; 60 s post budget -> {bu.get('status')}")
    if mech == "SIMPLIFY-ZERODIV":
        qq = q if q.get("simplify_spikes") else pr.get("b0") or q
        sp = (qq.get("simplify_spikes") or [{}])[0]
        g = (pr.get("guard") or pr.get("b0_guard") or {}).get("status")
        return (f"lifted polyline has an a -> b -> a spike (|ab| = {1e9 * (sp.get('ab_m') or 0):.1f} nm, vertex {sp.get('i')}),"
                f" so ac.ac = 0 in simplify; guarded simplify -> {g}")
    return "not explained by the trace"


def fact_row(r):
    mech, end = classify(r)
    # F3 targeted rows whose G2-config failure is EP: the genuine part is the buffer-0 outcome
    pr = r["probes"]
    b0 = pr.get("b0") or {}
    sub = None
    if r["source"] == "F3-targeted" and mech.startswith("EP-") and b0:
        if b0["status"] == "TIMEOUT":
            sub = "POST-TIMEOUT"
        elif b0["status"] == "ERROR":
            sub = "SIMPLIFY-ZERODIV"
    a = r["astar"]
    loc, loc_def = locus_clearance(r, mech, end)
    q = r["query"]
    fr = {"case_id": r["case_id"], "source": r["source"], "region": r["region"], "robot": r["robot"],
          "pair_id": r["pair_id"], "f4_class": r["class_f4"], "mechanism": mech, "group": group(mech),
          "under_buffer0": sub or "", "cause": cause_line(r, mech, end), "code": CODE[mech],
          "gmc_orig": r["gmc_orig"],
          "a_astar_replay": "PASS" if a["passed"] else "FAIL", "a_replayed_from": a["replayed_from"],
          "a_clearance3d_mm": mm(a["cylinder"].get("clearance_lower_m")),
          "b_reproduced": r["reproduced"], "b_requery": f"{q['status']}:{q['reason']}"[:90],
          "b_requery_wall_s": round(q["wall_s"], 2), "compile_id": r["compile_id"],
          "c_start_mm": mm(r["endpoints"]["start"].get("clearance_fine_m")),
          "c_goal_mm": mm(r["endpoints"]["goal"].get("clearance_fine_m")),
          "c_route_lateral_mm": mm(a.get("lateral_fine_m")), "c_locus_mm": mm(loc), "c_locus_def": loc_def,
          "c_by_tolerance": None if loc is None else bool(loc * 1e3 <= TOL_MM + EP_SLACK_MM + 1e-6),
          "probe_verdict_only": (pr.get("verdict_only") or pr.get("b0_verdict_only") or {}).get("status"),
          "probe_verdict_only_wall_s": (pr.get("verdict_only") or pr.get("b0_verdict_only") or {}).get("wall_s"),
          "probe_budget60": (pr.get("budget") or pr.get("b0_budget") or {}).get("status"),
          "probe_budget60_wall_s": (pr.get("budget") or pr.get("b0_budget") or {}).get("wall_s"),
          "probe_guard": (pr.get("guard") or pr.get("b0_guard") or {}).get("status"),
          "probe_buffer0": None if not b0 else f"{b0['status']}:{b0['reason']}"[:70],
          "replay_floors": ((q.get("replay") or {}).get("both_floors") or {}).get("passed"),
          "endpoint_end": end if end in ("start", "goal") else "",
          "figure": ""}
    post = q.get("post") or {}
    for st in ("shortcut", "tighten", "merge_corners"):
        fr[f"post_{st}_accepts"] = (post.get(st) or {}).get("accept_calls")
        fr[f"post_{st}_s"] = None if st not in post else round(post[st]["accept_s"], 2)
    fr["post_stage_at_end"] = q.get("stage_at_end") if q["status"] == "TIMEOUT" else ""
    vo = pr.get("verdict_only") or pr.get("b0_verdict_only")
    fr["vo_path_cells"] = (vo or {}).get("n_path_cells")
    fr["vo_vertices"] = len((vo or {}).get("polyline_uv") or []) or None
    if vo and vo.get("replay"):
        fr["probe_verdict_only_floors"] = (vo["replay"].get("both_floors") or {}).get("passed")
    else:
        fr["probe_verdict_only_floors"] = None
    return fr


def cmd_facts(a):
    tr = load_traces(a.trace_dir)
    rows = [fact_row(r) for r in tr.values()]
    rows.sort(key=lambda x: (x["source"] != "F4", x["group"] != "genuine", x["mechanism"], x["robot"], x["case_id"]))
    keys = list(rows[0].keys())
    with open(Path(a.out_dir) / "facts.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    s = summarize(rows)
    _dump(Path(a.out_dir) / "summary.json", s)
    print(json.dumps(s, indent=1, default=str)[:6000])


def summarize(rows):
    s = {"n": len(rows), "by_source": dict(collections.Counter(r["source"] for r in rows))}
    for src in ("F4", "F3-targeted"):
        R = [r for r in rows if r["source"] == src]
        if not R:
            continue
        d = {"n": len(R),
             "mechanism_by_robot": {f"{m}/{rb}": n for (m, rb), n in
                                    sorted(collections.Counter((r["mechanism"], r["robot"]) for r in R).items())},
             "f4_class_vs_mechanism": {f"{c} -> {m}": n for (c, m), n in
                                       sorted(collections.Counter((r["f4_class"], r["mechanism"]) for r in R).items())},
             "a_astar_replay": dict(collections.Counter(r["a_astar_replay"] for r in R)),
             "a_replayed_from": dict(collections.Counter(r["a_replayed_from"] for r in R)),
             "b_reproduced": dict(collections.Counter(str(r["b_reproduced"]) for r in R)),
             "c_by_tolerance_by_mechanism": {f"{m}: {t}": n for (m, t), n in sorted(collections.Counter(
                 (r["mechanism"], str(r["c_by_tolerance"])) for r in R).items())},
             "all_three_facts": sum(1 for r in R if r["a_astar_replay"] == "PASS" and r["b_reproduced"] is True)}
        T = [r for r in R if r["mechanism"] == "POST-TIMEOUT" or r["under_buffer0"] == "POST-TIMEOUT"]
        if T:
            d["post_timeout_probes"] = {
                "verdict_only": dict(collections.Counter(str(r["probe_verdict_only"]) for r in T)),
                "verdict_only_floors_on_replay_veto": dict(collections.Counter(
                    str(r["probe_verdict_only_floors"]) for r in T if r["probe_verdict_only"] == "UNKNOWN")),
                "verdict_only_wall_s_max": max((r["probe_verdict_only_wall_s"] or 0) for r in T),
                "budget60": dict(collections.Counter(str(r["probe_budget60"]) for r in T)),
                "budget60_wall_s_max": max((r["probe_budget60_wall_s"] or 0) for r in T)}
        G = [r for r in R if r["probe_guard"]]
        if G:
            d["guard_probe"] = {r["case_id"]: r["probe_guard"] for r in G}
        s[src] = d
    s["soundness_rows"] = [r["case_id"] for r in rows if r["mechanism"] == "CUT-WRONG"]
    s["unexplained_rows"] = [f"{r['case_id']}/{r['robot']}" for r in rows if r["mechanism"] == "OTHER"]
    s["mismatch_f4_vs_f5"] = [f"{r['case_id']}/{r['robot']}: {r['f4_class']} -> {r['mechanism']}" for r in rows
                              if r["source"] == "F4" and EXPECT.get(r["f4_class"]) and
                              r["mechanism"] not in EXPECT[r["f4_class"]]]
    return s


EXPECT = {"EP-TOL": {"EP-FLOOR", "EP-SIDE"}, "EP-GENUINE": {"EP-SLACK"}, "EXPORT-DOMAIN": {"REPLAY-AABB"},
          "EXPORT-KIN": {"REPLAY-RATE"}, "METHOD-TIMEOUT": {"POST-TIMEOUT"}, "METHOD-ERROR": {"SIMPLIFY-ZERODIV"}}


def _dump(path, doc):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(doc, indent=1, default=float) + "\n")


# ---------------------------------------------------------------------------------------------- figure selection
BOXES = {"WWEST": [-10.0, -1.35, -1.3, 3.75], "GAPW1": [-8.5, -0.35, -2.8, 3.75], "S": [-5.3, -1.75, 4.5, -0.45],
         "G2MID": [-5.5, -0.35, -1.2, 2.75]}


def _dir_at(P, uv):
    P = np.asarray(P, float)
    if len(P) < 2:
        return [1., 0.]
    k = int(np.argmin(np.linalg.norm(P - np.asarray(uv, float), axis=1)))
    for a_, b_ in ((k, k + 1), (k - 1, k), (k + 1, k + 2), (k - 2, k - 1)):
        if 0 <= a_ and b_ < len(P) and np.linalg.norm(P[b_] - P[a_]) > 1e-6:
            d = P[b_] - P[a_]
            return (d / np.linalg.norm(d)).round(4).tolist()
    return [1., 0.]


def _toward(a, b):
    d = np.asarray(b, float) - np.asarray(a, float)
    n = np.linalg.norm(d)
    return [1., 0.] if n < 1e-9 else (d / n).round(4).tolist()


def _wrap(s, n=47):
    out, line = [], ""
    for w in str(s).split(" "):
        if len(line) + len(w) + 1 > n:
            out.append(line)
            line = "      " + w
        else:
            line = (line + " " + w) if line else w
    out.append(line)
    return "\n".join(out)


def case_dict(r, fr):
    mech = fr["mechanism"]
    q, pr, a = r["query"], r["probes"], r["astar"]
    astar_uv = a.get("route_uv_used") or r.get("_astar_uv")
    view = {"path_cells": q.get("path_cells"), "endpoint_cells": q.get("endpoint_cells"),
            "route_uv": q.get("polyline_uv"), "route_label": "GMC route (G2 run, own-verified)"}
    rp = q.get("replay") or {}
    if rp.get("failed_edge"):
        view["failed_edge"] = rp["failed_edge"]
    end = fr["endpoint_end"]
    profile = "body"
    zoom = .35
    blockers = None
    if mech == "POST-TIMEOUT" or fr["under_buffer0"] == "POST-TIMEOUT":
        vo = pr.get("verdict_only") or pr.get("b0_verdict_only") or {}
        view.update(path_cells=vo.get("path_cells"), endpoint_cells=vo.get("endpoint_cells") or q.get("endpoint_cells"),
                    route_uv=vo.get("polyline_uv"), route_label="GMC certified route (verdict-only re-run, unshortened)")
    if mech == "SIMPLIFY-ZERODIV" or fr["under_buffer0"] == "SIMPLIFY-ZERODIV":
        src = q if q.get("simplify_in_uv") else pr.get("b0") or {}
        view.update(path_cells=src.get("path_cells") or q.get("path_cells"), route_uv=src.get("simplify_in_uv"),
                    endpoint_cells=src.get("endpoint_cells") or q.get("endpoint_cells"),
                    route_label="GMC lifted polyline (input of simplify, before the crash)")
    if mech.startswith("EP-"):
        locus = r["endpoints"][end]["uv"]
        kind = f"{end} endpoint (no certified cell)"
        profile = "floor" if mech != "EP-SIDE" else "body"
        blockers = (r["endpoints"][end].get("grow_cell_trace") or {}).get("blockers")
        if fr["under_buffer0"]:
            b0 = pr.get("b0") or {}
            view.update(path_cells=b0.get("path_cells"), endpoint_cells=b0.get("endpoint_cells"),
                        route_uv=b0.get("polyline_uv"), route_label="GMC route under buffer 0 (probe)")
    elif mech == "REPLAY-AABB":
        fe = rp["failed_edge"]
        locus = (np.add(fe["a_uv"], fe["b_uv"]) / 2).round(4).tolist()
        kind = "export edge whose swept AABB leaves the prism"
        zoom = .55
    elif mech == "REPLAY-RATE":
        locus = rp.get("kin_locus_uv") or q["polyline_uv"][0]
        kind = "export knot with the rate round-off"
    elif mech == "SIMPLIFY-ZERODIV":
        src = q if q.get("simplify_spikes") else pr.get("b0") or {}
        locus = src["simplify_spikes"][0]["a_uv"]
        kind = "a -> b -> a spike of the lifted polyline"
        zoom = .25
    else:
        locus = (a.get("lateral_tightest_edge") or {}).get("uv")
        kind = "A* route's tightest lateral point"
        if locus is None:
            P = np.asarray(view.get("route_uv") or astar_uv)
            locus = P[len(P) // 2].tolist()
            kind = "mid-route (A* lateral clearance >= 100 mm)"
    ref = view.get("route_uv") if mech in ("REPLAY-AABB", "REPLAY-RATE", "SIMPLIFY-ZERODIV") and view.get("route_uv") \
        else astar_uv
    head = f"{r['case_id']}  {r['robot']}  [{fr['source']}]"
    title = (f"{head}   GMC {fr['gmc_orig'][:70]}   class {mech} ({fr['group']})"
             + (f", under buffer 0: {fr['under_buffer0']}" if fr["under_buffer0"] else "")
             + "\n" + _wrap(fr["cause"], 170))
    facts = [
        f"case      {r['case_id']}", f"robot     {r['robot']}",
        f"source    {fr['source']}   F4 class {fr['f4_class']}",
        _wrap(f"GMC (G2)  {fr['gmc_orig']}"),
        f"class     {mech} ({fr['group']})",
        "", "(a) A* route is a real route",
        _wrap(f"    re-replay {fr['a_astar_replay']} with the compile's gs3d oracle (real cylinder"
              + (f" + {r['robot']}" if r['robot'] != 'cylinder' else "") + f", margin 1 mm, goal region); "
              f"min 3-D clearance bound {_fmt(fr['a_clearance3d_mm'])} mm; from {fr['a_replayed_from']}"),
        "", "(b) GMC's failure reproduces on the same compile",
        _wrap(f"    {fr['b_reproduced']}: {fr['b_requery']} ({fr['b_requery_wall_s']} s), compile {fr['compile_id']}"),
        "", "(c) clearance vs margin + buffer (2 mm, +0.1)",
        f"    start {_fmt(fr['c_start_mm'])} mm, goal {_fmt(fr['c_goal_mm'])} mm",
        f"    route lateral {_fmt(fr['c_route_lateral_mm'])} mm" + (" (cap)" if fr['c_route_lateral_mm'] == 100 else ""),
        _wrap(f"    locus: {_fmt(fr['c_locus_mm'])} mm -> "
              + ("by-tolerance (<= 2.1 mm)" if fr["c_by_tolerance"] else "above 2.1 mm")),
        "", "mechanism", _wrap("    " + fr["cause"]), _wrap("    code: " + fr["code"]),
    ]
    pl = []
    for k, v in pr.items():
        extra = ""
        if v.get("replay") and not v["replay"].get("passed"):
            extra = f", 1 ms floors {'pass' if (v['replay'].get('both_floors') or {}).get('passed') else 'fail'}"
        pl.append(_wrap(f"    {k}: {v['status']} {v['reason'][:40]} ({v['wall_s']:.1f} s{extra})"))
    if rp and not rp.get("passed"):
        pl.append(_wrap(f"    replay of the G2 route: geometry {rp.get('geometry_reason')}, kinematics "
                        f"{rp['kinematics'].get('reason', 'passed')}, 1 ms floors "
                        f"{'pass' if (rp.get('both_floors') or {}).get('passed') else 'fail'}"))
    if pl:
        facts += ["", "probes (no src change)"] + pl
    p = r.get("_pair") or {}
    if p:
        facts += ["", f"pair: {p.get('dist_m', 0):.2f} m straight, A* detour {p.get('len_ratio', 0):.2f}"]
    facts += ["", "verdict -> f5/review_verdicts.csv"]
    return {"case_id": r["case_id"], "robot": r["robot"], "region": r["region"], "source": fr["source"],
            "mechanism": mech, "start_uv": r["endpoints"]["start"]["uv"], "goal_uv": r["endpoints"]["goal"]["uv"],
            "astar_uv": astar_uv, "tight_uv": (a.get("lateral_tightest_edge") or {}).get("uv"),
            "gmc_view": view, "blockers": blockers, "locus_uv": locus, "locus_kind": kind,
            "locus_dir": (_toward(locus, blockers[0]["mean_route"][:2]) if blockers else _dir_at(ref, locus)),
            "locus_dir_kind": "toward the blocker" if blockers else "along the local route direction", "profile": profile, "zoom_half": zoom, "title": title,
            "text": "\n".join(facts)}


def attach_inputs(tr):
    """A* route + pair evidence from the plan files (the trace keeps only re-planned routes)."""
    plan = {}
    for f in sorted((F5 / "plan").glob("*.jsonl")):
        for it in _jsonl(f):
            plan[(it["case_id"], it["robot"])] = it
    ev = {}
    for reg in ("WWEST", "GAPW1", "S"):
        for p in json.loads((F4 / "sample" / f"{reg}_pairs.json").read_text())["pairs"]:
            ev[p["pair_id"]] = {"dist_m": p["dist_m"], "len_ratio": p["evidence"]["len_ratio"]}
    for k, r in tr.items():
        it = plan.get(k)
        if it:
            r["_astar_uv"] = it["astar_route_uv"]
        r["_pair"] = ev.get(r["pair_id"]) or {}
    return tr


def pick_representatives(R, n, key):
    if len(R) <= n:
        return list(R)
    strata = collections.defaultdict(list)
    for r in R:
        strata[(r["region"], r["robot"])].append(r)
    alloc = {k: max(1, round(n * len(v) / len(R))) for k, v in strata.items()}
    while sum(alloc.values()) > n:
        k = max(alloc, key=lambda k: alloc[k])
        alloc[k] -= 1
    while sum(alloc.values()) < n:
        k = max(strata, key=lambda k: len(strata[k]) - alloc[k])
        alloc[k] += 1
    out = []
    for k, v in sorted(strata.items()):
        v = sorted(v, key=lambda r: (key(r) is None, key(r) or 0, r["case_id"]))
        m = alloc[k]
        idx = sorted({int(round(x)) for x in np.linspace(0, len(v) - 1, m)})
        out += [v[i] for i in idx]
    return out


def cmd_select(a):
    tr = attach_inputs(load_traces(a.trace_dir))
    facts = {(r["case_id"], r["robot"]): r for r in csv.DictReader(open(Path(a.out_dir) / "facts.csv"))}
    for r in facts.values():
        for k in ("c_locus_mm", "c_start_mm", "c_goal_mm", "c_route_lateral_mm", "a_clearance3d_mm"):
            r[k] = float(r[k]) if r[k] not in ("", "None") else None
        r["c_by_tolerance"] = {"True": True, "False": False}.get(r["c_by_tolerance"])
        r["b_reproduced"] = {"True": True, "False": False}.get(r["b_reproduced"], r["b_reproduced"])
    chosen, why = [], {}
    by_class = collections.defaultdict(list)
    for k, fr in facts.items():
        if fr["source"] != "F4" or fr["group"] == "genuine":
            chosen.append(k)
            why[k] = "genuine" if fr["group"] == "genuine" else "F3-targeted"
        else:
            by_class[(fr["mechanism"], fr["robot"])].append(fr)
    overviews = []
    for (m, rb), R in sorted(by_class.items()):
        reps = pick_representatives(R, 30, lambda r: r["c_locus_mm"])
        for fr in reps:
            k = (fr["case_id"], fr["robot"])
            chosen.append(k)
            why[k] = "all rows of class" if len(R) <= 30 else f"representative 1 of 30 (class n = {len(R)})"
        if len(R) > 30:
            pts, blk = [], collections.Counter()
            for fr in R:
                r = tr[(fr["case_id"], fr["robot"])]
                c = case_dict(r, fr)
                pts.append({"region": fr["region"], "uv": c["locus_uv"], "value": fr["c_locus_mm"]})
                for b in c["blockers"] or []:
                    blk[(fr["region"], b["scene_id"], tuple(np.round(b["mean_route"][:2], 3)))] += 1
            overviews.append({"name": f"{m}_{rb}", "robot_map": rb, "points": pts,
                              "value_label": "failure-locus clearance (mm)",
                              "title": f"class {m}, {rb}: all {len(R)} rows' failure loci (colour = locus clearance; "
                                       f"x = endpoint blockers named by >= 5 rows)",
                              "blockers": [{"region": g, "scene_id": s, "uv": list(uv), "n": n}
                                           for (g, s, uv), n in blk.most_common() if n >= 5]})
    cases = []
    for k in chosen:
        r, fr = tr[k], facts[k]
        c = case_dict(r, fr)
        c["why"] = why[k]
        cases.append(c)
    _dump(F5 / "fig_selection.json", {"boxes": BOXES, "cases": cases, "overviews": overviews})
    print(len(cases), "figures;", collections.Counter((c["mechanism"], c["robot"], c["why"][:14]) for c in cases))
    print("overviews", [o["name"] for o in overviews])


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    for n in ("facts", "select", "review"):
        s = sub.add_parser(n)
        s.add_argument("--trace-dir", default=str(F5 / "trace"))
        s.add_argument("--out-dir", default=str(F5))
    a = p.parse_args(argv)
    {"facts": cmd_facts, "select": cmd_select, "review": cmd_review}[a.cmd](a)


CAUSE = {
    "POST-TIMEOUT": "GMC finds and certifies a route in seconds, but the optional route shortening runs past G2's 120 s "
                    "limit, mostly in `shortcut` proving that long farthest-first candidate chords collide",
    "SIMPLIFY-ZERODIV": "the lifted polyline contains an a -> b -> a spike of a few nm; `simplify` divides by |ac|^2 = 0",
    "EP-SLACK": "the endpoint is > 2 mm clear by the oracle, yet GMC's refined envelope gap is <= the 1 mm buffer",
    "CUT-WRONG": "certified UNREACHABLE contradicted by a demonstrated route (soundness)",
    "OTHER": "not explained by the trace",
    "EP-FLOOR": "the endpoint is only 1-2 mm from a Gaussian below the chassis (809/832: a sub-floor splat); GMC needs "
                "margin + buffer = 2 mm to grow a certified endpoint cell (by design)",
    "EP-SIDE": "the endpoint is within margin + buffer of an obstacle beside the body",
    "REPLAY-AABB": "the shared replay's known-space test checks the world AABB of each swept 0.20 m export segment; near "
                   "a box face in the 64-degree-rotated frame that AABB leaves the prism although the body does not",
    "REPLAY-RATE": "float round-off of a us-scale in-place turn or translation time exceeds the replay's 1e-9 rate "
                   "tolerance; geometry passes and 1 ms floors clear it",
}
ORDER = ["CUT-WRONG", "OTHER", "POST-TIMEOUT", "SIMPLIFY-ZERODIV", "EP-SLACK", "EP-FLOOR", "EP-SIDE", "REPLAY-AABB",
         "REPLAY-RATE"]


def cmd_review(a):
    sel = json.loads((F5 / "fig_selection.json").read_text())
    facts = {(r["case_id"], r["robot"]): r for r in csv.DictReader(open(Path(a.out_dir) / "facts.csv"))}
    figs = {(c["case_id"], c["robot"]): c for c in sel["cases"]}
    allrows = list(facts.values())
    L = ["# F5 review index: every GMC failure case on confirmed-reachable pairs, for human approval",
         "",
         "Generated by `gmc/experiments/aerial3dg_fail5_analyze.py review` from `gmc/results/aerial3dg/f5/facts.csv` "
         "(one row per failure, all 1567 F4 rows + 35 F3 targeted rows) and the figures in "
         "`gmc/results/aerial3dg/f5/fig/`. Report: [`aerial3dg_failures_f5.md`](aerial3dg_failures_f5.md).",
         "",
         "**How to review.** Open each figure, check that (a) the blue A* route really goes from start to goal around "
         "the obstacles, (b) GMC's failure is what the title says, and (c) the zoom / side profile show the cause. Then "
         "fill `human_verdict` in `gmc/results/aerial3dg/f5/review_verdicts.csv` with `algorithm_failure`, "
         "`not_algorithm_failure` or `unsure` (+ `human_note`). Classes with more than 30 rows are shown by 30 "
         "stratified representatives plus an overview map; the verdicts CSV has one class-level row for each, which "
         "is your verdict for the rest of that class.",
         "",
         "Columns: (a) = the A* route re-replayed with the compile's own gs3d oracle on the real body, margin 1 mm "
         "(PASS = a real route); (b) = GMC's failure reproduced on the same persisted compile; (c) = clearance at the "
         "failure locus in mm (endpoint rows: the endpoint's oracle clearance; route rows: the A* route's lateral "
         "clearance), `tol` when <= margin + buffer (2 mm) + 0.1 mm (F3/F4's cut for GMC's slack). For export classes (c) is context only: their mechanism "
         "is not a clearance.",
         "",
         "## Classes", "",
         "| class | group | F4 cylinder | F4 sweeper | F3 targeted | figures | cause | code |",
         "|---|---|---|---|---|---|---|---|"]
    cnt = collections.Counter((r["mechanism"], r["robot"] if r["source"] == "F4" else "F3") for r in allrows)
    for m in ORDER:
        n = [cnt[(m, "cylinder")], cnt[(m, "sweeper")], cnt[(m, "F3")]]
        if not sum(n):
            continue
        nf = sum(1 for k in figs if facts[k]["mechanism"] == m)
        L.append(f"| {m} | {group(m)} | {n[0]} | {n[1]} | {n[2]} | {nf} | {CAUSE[m]} | `{CODE[m]}` |")
    L += ["", "Soundness: " + ("**no** certified UNREACHABLE on any confirmed-reachable pair (0 of 10,000 F4 "
                               "queries); nothing to trace." if not cnt[("CUT-WRONG", "cylinder")] +
                               cnt[("CUT-WRONG", "sweeper")] else "see CUT-WRONG below."), ""]
    vrows = []
    n_case = 0
    for src, head in (("F4", "F4 benchmark (5000 confirmed-reachable pairs)"),
                      ("F3-targeted", "F3 targeted discovery (verified candidates; kept separate)")):
        L += [f"## {head}", ""]
        for m in ORDER:
            R = [r for k, r in sorted(facts.items()) if r["source"] == src and r["mechanism"] == m and k in figs]
            if not R:
                continue
            allm = [r for r in allrows if r["source"] == src and r["mechanism"] == m]
            L += [f"### {m} ({group(m)}): {len(R)} figure(s) of {len(allm)} row(s)", "", f"Cause: {CAUSE[m]}. Code: `{CODE[m]}`.", ""]
            for rb in ("cylinder", "sweeper"):
                ov = F5 / "fig" / "overview" / f"{m}_{rb}.png"
                if src == "F4" and ov.exists():
                    L += [f"Overview of all {sum(1 for r in allm if r['robot'] == rb)} {rb} rows: "
                          f"[`{ov.name}`](../gmc/{ov})", ""]
                    vrows.append({"review_id": f"class:{m}/{rb}", "case_id": f"(all {m} {rb} rows not shown)",
                                  "robot": rb, "source": src, "mechanism": m, "group": group(m),
                                  "n_rows": sum(1 for r in allm if r["robot"] == rb),
                                  "figure": f"gmc/{ov}", "cause": CAUSE[m]})
            L += ["| # | case | robot | GMC | cause (this row) | (a) A* | (b) repro | (c) mm | figure | why shown |",
                  "|---|---|---|---|---|---|---|---|---|---|"]
            for r in R:
                n_case += 1
                k = (r["case_id"], r["robot"])
                f = f"gmc/results/aerial3dg/f5/fig/{r['case_id']}_{r['robot']}.png"
                c = r["c_locus_mm"]
                tol = " tol" if r["c_by_tolerance"] == "True" else ""
                cause = r["cause"].replace("|", "/")
                if r["under_buffer0"]:
                    cause += f" [buffer 0: {r['probe_buffer0']}]"
                L.append(f"| {n_case} | {r['case_id']} | {r['robot']} | {r['gmc_orig'][:45]} | {cause} | "
                         f"{r['a_astar_replay']} | {r['b_reproduced']} | {c}{tol} | [png](../{f}) | {figs[k]['why']} |")
                vrows.append({"review_id": f"{r['case_id']}/{r['robot']}", "case_id": r["case_id"], "robot": r["robot"],
                              "source": src, "mechanism": m, "group": group(m), "n_rows": 1, "figure": f,
                              "cause": r["cause"]})
            L.append("")
    (DOCS / "aerial3dg_f5_review.md").write_text("\n".join(L) + "\n")
    with open(F5 / "review_verdicts.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["review_id", "case_id", "robot", "source", "mechanism", "group", "n_rows",
                                           "figure", "cause", "human_verdict", "human_note"])
        w.writeheader()
        for v in vrows:
            w.writerow({**v, "human_verdict": "", "human_note": ""})
    print("review index:", n_case, "cases;", len(vrows), "verdict rows")


if __name__ == "__main__":
    main()
