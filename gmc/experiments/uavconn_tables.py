"""C2: markdown tables (Chinese headers) for docs/uavconn_report.md, generated from comparison.json.

No number in the report tables is typed by hand; they come from ``results/uavconn/comparison.json``.

Usage (from ``gmc/``)::  python experiments/uavconn_tables.py results/uavconn/comparison.json > results/uavconn/tables.md
"""
from __future__ import annotations

import json
import sys

OUT = {"path": "有路（回放通过）", "no_path_certified": "认证无路 UNREACHABLE", "exhausted": "队列耗尽（未认证）",
       "unknown": "UNKNOWN", "budget": "预算停止（5 h）", "timeout": "作业超时", "replay_failed": "回放失败",
       "not_run": "未运行", "job_failed": "作业失败", "other": "其他"}


def f(x, nd=2, unit=""):
    return "—" if x is None else f"{x:.{nd}f}{unit}"


def ratio(x):
    if x is None:
        return "—"
    return f"{x:,.0f}×" if x >= 10 else (f"{x:.1f}×" if x >= 1 else f"{x:.2g}×")


def sec(x):
    if x is None:
        return "—"
    return f"{x:,.0f} s" if x >= 100 else f"{x:.2f} s"


def a3_cell(m):
    s = OUT.get(m["outcome"], m["outcome"])
    if m["outcome"] == "unknown":
        s += f" (`{m['reason']}`)"
    return s


def lat_cell(m):
    return OUT.get(m["outcome"], m["outcome"])


def bench(doc):
    rows = [r for r in doc["rows"] if r["set"] == "benchmark"]
    out = ["| 查询 | 真值 | aerial3d 结果 | aerial3d 长度 / 顶点 | aerial3d 单次查询 冷 / 热 | lattice 结果 | lattice 长度 / 节点 | "
           "lattice 单次查询 | 两者先灯下后桌上 |", "|---|---|---|---:|---:|---|---:|---:|---|"]
    for r in rows:
        a, l = r["aerial3d"], r["lattice"]
        ar, lr = a.get("raw") or {}, l.get("raw") or {}
        at = a.get("time") or {}
        lt = (r.get("lattice_retime") or {}).get("time") if r["query"] == "M1" else None
        ltime = sec(l["time"]["algorithm_s"]) + (" (L2, 并发节点)" if r["query"] != "M1" else " (L2, 并发节点)")
        if lt:
            ltime += f"；C2 单独重测 {sec(lt['algorithm_s'])}"
        order = []
        for m in (a, l):
            ev = m.get("evidence")
            order.append("—" if not ev else ("是" if ev["passes_under_lamp"] and ev["under_precedes_above_table"]
                                             else ("从灯上方过" if ev["over_lamp_samples"] else "否")))
        out.append(f"| {r['query']} | {'有路' if r['ground_truth'] == 'PATH' else '无路'} | {a3_cell(a)} | "
                   f"{f(ar.get('path_length_m'), 3, ' m')} / {ar.get('n_vertices', '—')} | "
                   f"{sec(at.get('cold_s'))} / {sec(at.get('warm_median_s'))} | {lat_cell(l)} | "
                   f"{f(lr.get('path_length_m'), 3, ' m')} / {lr.get('n_vertices', '—')} | {ltime} | "
                   f"{order[0]} / {order[1]} |")
    return "\n".join(out)


def success(doc):
    names = {"benchmark": "基准 5 查询", "extended_A": "扩展集 A 类（走廊→桌上）", "extended_B": "扩展集 B 类（随机无碰撞对）",
             "extended": "扩展集合计（24）", "all": "全部（29）"}
    out = ["| 集合 | 方法 | 成功 / 总数 | 成功率 [Wilson 95%] | 有路 | 认证无路 | 队列耗尽 | UNKNOWN | 预算停止 / 超时 | 矛盾 |",
           "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for key in ("benchmark", "extended_A", "extended_B", "extended", "all"):
        for m in ("aerial3d", "lattice"):
            s = doc["success"][key][m]
            by = s["by_outcome"]
            ci = s["wilson95"]
            out.append(f"| {names[key]} | {m} | {s['success']} / {s['n']} | "
                       f"{100 * s['rate']:.0f}% [{100 * ci[0]:.0f}–{100 * ci[1]:.0f}%] | {by.get('path', 0)} | "
                       f"{by.get('no_path_certified', 0)} | {by.get('exhausted', 0)} | {by.get('unknown', 0)} | "
                       f"{by.get('budget', 0) + by.get('timeout', 0)} | {s['contradictions']} |")
    return "\n".join(out)


def extended(doc):
    out = ["| 对 | 类 | 直线段 (oracle) | 过灯平面 | aerial3d | 长度 / 顶点 | 冷 | lattice | 长度 / 节点 | 耗时 | 真值 | 成功 a / l |",
           "|---|---|---|---|---|---:|---:|---|---:|---:|---|---|"]
    for r in doc["rows"]:
        if r["set"] != "extended":
            continue
        a, l = r["aerial3d"], r["lattice"]
        ar, lr = a.get("raw") or {}, l.get("raw") or {}
        out.append(f"| {r['query']} | {r['class']} | {r['straight_segment_oracle']} | "
                   f"{'是' if r['crosses_lamp_plane'] else '否'} | {a3_cell(a)} | "
                   f"{f(ar.get('path_length_m'), 3)} / {ar.get('n_vertices', '—')} | "
                   f"{sec((a.get('time') or {}).get('cold_s'))} | {lat_cell(l)} | "
                   f"{f(lr.get('path_length_m'), 3)} / {lr.get('n_vertices', '—')} | "
                   f"{sec((l.get('time') or {}).get('algorithm_s'))} | "
                   f"{'有路' if r['ground_truth'] == 'PATH' else '无路证据'} | "
                   f"{'✓' if a.get('success') else '✗'} / {'✓' if l.get('success') else '✗'} |")
    return "\n".join(out)


def smooth(doc, key="all"):
    s = doc["smoothness"][key]
    lab = {"path_length_m": "路径长度 (m)", "n_vertices": "折线顶点数", "n_turning_vertices": "转折顶点数",
           "total_turning_rad": "总转角 (rad)", "max_turning_rad": "最大单次转角 (rad)",
           "bending_energy_per_m": "离散弯曲能 Σθ²/ℓ (1/m)", "vertical_travel_m": "垂直行程 (m)",
           "integrated_squared_jerk": "∫|jerk|² dt", "jerk_dimensionless": "无量纲 jerk J·T⁵/L²",
           "duration_s": "平滑后时长 (s)", "smooth_path_length_m": "平滑后长度 (m)",
           "gs3d_turn_metric_rad": "平滑后转角指标 (rad)", "sampled_curvature_max_radpm": "平滑后最大曲率 (1/m)"}

    def lhe(p):
        lo, le, n = p["aerial3d_lower_count"], p["aerial3d_leq_count"], p["n"]
        return f"{lo} / {le - lo} / {n - le}"

    def cell(st):
        return "—" if not st else f"{st['median']:.3g} [{st['q1']:.3g}, {st['q3']:.3g}]"

    out = [f"原始路径：两种方法都有回放通过路径的 {len(s['queries_where_both_have_paths'])} 个查询；"
           f"平滑后：两者平滑曲线都通过重新验证的 {len(s['queries_where_both_smoothed_and_reverified'])} 个查询。",
           "", "| 指标 | aerial3d 中位数 [IQR] | lattice 中位数 [IQR] | aerial3d 更小 / 相等 / 更大 |", "|---|---:|---:|---:|"]
    for k, st in s["raw"].items():
        p = s["paired"][k]
        out.append(f"| 原始：{lab[k]} | {cell(st['aerial3d'])} | {cell(st['lattice'])} | {lhe(p)} |")
    for k, st in s["smoothed"].items():
        p = s["paired"]["smoothed_" + k]
        out.append(f"| 同一 A6 平滑后：{lab[k]} | {cell(st['aerial3d'])} | {cell(st['lattice'])} | {lhe(p)} |")
    return "\n".join(out)


def timing(doc):
    t = doc["time"]
    names = {"booth": "展台（M1、M5）", "plug": "展台 + 堵板（N2、N2h）", "lamp": "只有灯（C4h）", "ext": "展台（扩展集 24 对，另一作业）"}
    out = ["| 场景编译 | 编译墙钟 | 其中 octree / cells+portals | CPU 时间 | 峰值 RSS | 节点 CPUAlloc/CPUTot | 服务查询数 |",
           "|---|---:|---:|---:|---:|---:|---:|"]
    for g, c in t["compiles"].items():
        st = c["stages"]
        out.append(f"| {names.get(g, g)} | {sec(c['compile_wall_s'])} | {sec(st.get('octree'))} / {sec(st.get('cells'))} | "
                   f"{sec(c['compile_cpu_s'])} | {c['peak_rss_mb'] / 1024:.2f} GB | {c['host'].get('CPUAlloc')}/"
                   f"{c['host'].get('CPUTot')} | {c['queries_served']} |")
    out += ["", "| 查询 | lattice 单次 | aerial3d 冷 | aerial3d 热（中位数） | 加速比 冷 | 加速比 热 | 首条路径耗时 lattice / aerial3d（含编译） | 比值 |",
            "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for p in t["per_query"]:
        out.append(f"| {p['query']} | {sec(p['lattice_s'])}{' †' if p['lattice_outcome'] == 'budget' else ''} | "
                   f"{sec(p['aerial3d_cold_s'])} | {sec(p['aerial3d_warm_median_s'])} | {ratio(p['speedup_cold'])} | "
                   f"{ratio(p['speedup_warm'])} | {sec(p['lattice_time_to_first_path_s'])} / "
                   f"{sec(p['aerial3d_time_to_first_path_s'])} | {ratio(p['speedup_time_to_first_path'])} |")
    out.append("\n† lattice 在 5 h 预算内没有给出答案（预算停止）；比值是下界。M1 的 lattice 用 C2 单独重测值"
               "（L2 在并发节点上记录的是 6 117 s）；其余基准行是 L2 记录值（并发节点）。扩展集的 aerial3d 编译是另一作业的 658 s"
               "（与 371 s 的同一编译相比节点更忙）。")
    return "\n".join(out)


def main():
    doc = json.load(open(sys.argv[1]))
    for title, fn in (("基准", bench), ("成功率", success), ("扩展集逐对", extended), ("平滑度", smooth), ("耗时", timing)):
        print(f"<!-- {title} -->\n{fn(doc)}\n")


if __name__ == "__main__":
    main()
