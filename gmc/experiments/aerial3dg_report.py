"""G1 report artefacts that need only the run JSONs (no archive): timing chart, the filled copy of
``measurement_template.csv`` (one column per robot) and the 5000-pair time estimate.

Usage (from ``gmc/``)::

    python experiments/aerial3dg_report.py --demo DIR --screen DIR --sacct SACCT.json --out results/aerial3dg
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROBOTS = ("sweeper", "cylinder")
ROBOT_C = {"sweeper": "#2a78d6", "cylinder": "#eb6834"}
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["DejaVu Sans"], "font.size": 9,
                     "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
                     "text.color": INK, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
                     "axes.edgecolor": GRID, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "axes.axisbelow": True})
COMPILE_STAGES = ["scene_prepare", "pair_candidates", "envelopes", "octree", "cells", "possible_graph", "audit"]
QUERY_GROUPS = {  # template-aligned grouping of the query's own stage records
    "locate endpoints + portal A* (or cut certificate)": ["locate", "locate_cells", "cut_certificate", "graph_search"],
    "path extraction (lift + shortcut + tighten + merge)": ["lifting", "shortcut", "tighten", "merge_corners"],
    "own continuous verifier": ["own_verification"],
    "shared gs3d replay": ["shared_verification"],
}


def view(top, pair) -> dict:
    """One (robot, pair) column: the robot's compile record merged with that pair's answers."""
    pd = top["pairs"][pair]
    d = {k: v for k, v in top.items() if k != "pairs"}
    d.update(result=pd["result"], calls=pd["calls"], replay=pd["replay"], straight_line=pd["straight_line"],
             pair={k: pd[k] for k in ("name", "role", "start_uv", "goal_uv", "dist_m")},
             identical=pd["identical_all_calls"], attribution=pd.get("certificate_attribution"))
    return d


def pct(a, b):
    return 100. * a / b if b else float("nan")


def q(xs, p):
    return float(np.percentile(np.asarray(xs, float), p)) if len(xs) else float("nan")


# ----------------------------------------------------------------------------- chart
def fig_timing(demos, screens, est, dst):
    fig, axes = plt.subplots(1, 3, figsize=(17.5, 5.6), gridspec_kw={"width_ratios": [1.05, 1.15, 1]})
    # (a) compile sub-stages, seconds, robots side by side
    ax = axes[0]
    y = np.arange(len(COMPILE_STAGES))
    for k, r in enumerate(ROBOTS):
        st = {s["stage"]: s["seconds"] for s in demos[r]["compile"]["stages"]}
        vals = [st.get(s, 0.) for s in COMPILE_STAGES]
        ax.barh(y + (k - .5) * .38, vals, height=.34, color=ROBOT_C[r],
                label=f"{r}: {demos[r]['compile']['compile_wall_s']:.1f} s total, "
                      f"{demos[r]['compile']['pairs']['candidate_pairs']:,} candidate pairs")
        for yy, v in zip(y + (k - .5) * .38, vals):
            ax.text(v, yy, f" {v:.2f}", va="center", fontsize=7.5, color=INK2)
    ax.set_yticks(y, COMPILE_STAGES)
    ax.invert_yaxis()
    ax.set_xlabel("seconds (paid once per scene + robot + box)")
    ax.set_title("(a) compile, once: where the time goes", loc="left", fontsize=10)
    ax.legend(loc="lower right", fontsize=7.5)
    # (b) query calls on the demo pair: cold, warm x3, reloaded — stacked by stage group
    ax = axes[1]
    rows, labels = [], []
    for r in ROBOTS:
        for pname, pd in demos[r]["pairs"].items():
            for c in pd["calls"]:
                rows.append((r, c))
                labels.append(f"{r} {c['call_id']} ({pd['status'][:5]})")
    greys = ["#eda100", "#008300", "#e87ba4", "#4a3aa7"]   # plane_timing_report GROUP_COLOR hues (validated)
    for gi, (g, names) in enumerate(QUERY_GROUPS.items()):
        left = np.array([sum(c["stages"].get(n, 0.) for gg in list(QUERY_GROUPS)[:gi] for n in QUERY_GROUPS[gg])
                         for _, c in rows])
        w = np.array([sum(c["stages"].get(n, 0.) for n in names) for _, c in rows])
        ax.barh(np.arange(len(rows)), w, left=left, height=.62, color=greys[gi], edgecolor=SURFACE, lw=1, label=g)
    for i, (r, c) in enumerate(rows):
        ax.barh(i, .0, color=ROBOT_C[r])
        ax.text(c["algorithm_wall_s"], i, f" {c['algorithm_wall_s']:.3f} s", va="center", fontsize=7.5, color=INK2)
        ax.scatter([-0.02 * max(cc['algorithm_wall_s'] for _, cc in rows)], [i], color=ROBOT_C[r], s=30, clip_on=False)
    ax.set_yticks(np.arange(len(rows)), labels)
    ax.invert_yaxis()
    ax.set_xlabel("seconds per query call (algorithm wall, incl. own verifier + shared gs3d replay)")
    ax.set_title("(b) both demo pairs answered from ONE compile per robot: cold, warm, reloaded", loc="left", fontsize=10)
    ax.legend(loc="upper center", bbox_to_anchor=(.45, -.13), ncol=2, fontsize=7, frameon=False)
    # (c) amortisation: compile once vs compile per pair, N pairs
    ax = axes[2]
    n = np.logspace(0, np.log10(5000), 60)
    for r in ROBOTS:
        e = est["robots"][r]
        once = e["compile_s"] + e["load_compiled_s"] + n * e["query_reachable_mean_s"]
        each = n * (e["archive_load_and_crop_s"] + e["compile_s"] + e["query_reachable_mean_s"])
        ax.plot(n, once / 3600, color=ROBOT_C[r], lw=2, label=f"{r}: compile once, reuse")
        ax.plot(n, each / 3600, color=ROBOT_C[r], lw=1.4, ls=(0, (4, 3)), label=f"{r}: load + compile per pair")
        ax.text(n[-1] * 1.05, once[-1] / 3600, f"{once[-1] / 3600:.2f} h", fontsize=7.5, color=INK2, va="center")
        ax.text(n[-1] * 1.05, each[-1] / 3600, f"{each[-1] / 3600:.1f} h", fontsize=7.5, color=INK2, va="center")
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlim(1, 5000 * 2.2)
    ax.set_xlabel("number of start/goal pairs on one scene + robot")
    ax.set_ylabel("single-core hours")
    ax.set_title("(c) N pairs: compile once vs per pair\n(every query at the REACHABLE mean)", loc="left", fontsize=10)
    ax.legend(loc="upper left", fontsize=7.5)
    fig.suptitle("aerial3d ground bodies on the real archive — compile once, query many (sweeper vs cylinder, the same two demo pairs)",
                 x=.01, ha="left", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, .95))
    fig.savefig(dst, dpi=100)
    plt.close(fig)


# ----------------------------------------------------------------------------- estimate
def estimate(demos, screens, sacct) -> dict:
    out = {"robots": {}, "assumptions": []}
    for r in ROBOTS:
        d, s = demos[r], screens[r]
        qs = [x["algorithm_wall_s"] for x in s["results"]]
        status, by_status = {}, {}
        for x in s["results"]:
            status[x["status"]] = status.get(x["status"], 0) + 1
            by_status.setdefault(x["status"], []).append(x["algorithm_wall_s"])
        calls = [c for pd in d["pairs"].values() for c in pd["calls"]]
        warm = [c["algorithm_wall_s"] for c in calls if c["mode"] == "warm"]
        e = {"compile_s": d["compile"]["compile_wall_s"], "compile_cpu_s": d["compile"]["compile_cpu_s"],
             "archive_load_and_crop_s": d["archive_load_and_hash_s"] + d["scene_build_and_crop_s"],
             "load_compiled_s": d["persist"]["load_wall_s"], "persisted_bytes": d["persist"]["bytes"],
             "screen_queries": len(qs), "screen_status": status,
             "query_mean_s": float(np.mean(qs)), "query_median_s": q(qs, 50), "query_p95_s": q(qs, 95),
             "query_max_s": float(np.max(qs)),
             "query_by_status": {k: {"n": len(v), "mean_s": float(np.mean(v)), "p95_s": q(v, 95), "max_s": float(np.max(v))}
                                 for k, v in by_status.items()},
             "query_reachable_mean_s": float(np.mean(by_status.get("REACHABLE", qs))),
             "demo_cold_s": {n: pd["calls"][0]["algorithm_wall_s"] for n, pd in d["pairs"].items()},
             "demo_warm_median_s": q(warm, 50), "peak_rss_gb": sacct.get(r, {}).get("maxrss_gb")}
        e["cpu_h_5000_compile_once"] = (e["archive_load_and_crop_s"] + e["compile_s"] + 5000 * e["query_mean_s"]) / 3600
        e["cpu_h_5000_all_reachable_mean"] = (e["archive_load_and_crop_s"] + e["compile_s"]
                                              + 5000 * e["query_reachable_mean_s"]) / 3600
        e["cpu_h_5000_p95_every_pair"] = (e["archive_load_and_crop_s"] + e["compile_s"] + 5000 * e["query_p95_s"]) / 3600
        e["cpu_h_5000_compile_per_pair"] = 5000 * (e["archive_load_and_crop_s"] + e["compile_s"] + e["query_mean_s"]) / 3600
        for k in (1, 4, 8):
            per_task = 5000 / k * e["query_mean_s"]
            e[f"wall_h_{k}_tasks"] = (e["archive_load_and_crop_s"] + e["compile_s"] + e["load_compiled_s"] + per_task) / 3600
        out["robots"][r] = e
    both = sum(out["robots"][r]["cpu_h_5000_compile_once"] for r in ROBOTS)
    out["total_cpu_h_both_robots_compile_once"] = both
    out["total_cpu_h_both_robots_all_reachable_mean"] = sum(out["robots"][r]["cpu_h_5000_all_reachable_mean"] for r in ROBOTS)
    out["total_cpu_h_both_robots_p95_bound"] = sum(out["robots"][r]["cpu_h_5000_p95_every_pair"] for r in ROBOTS)
    out["total_cpu_h_both_robots_compile_per_pair"] = sum(out["robots"][r]["cpu_h_5000_compile_per_pair"] for r in ROBOTS)
    out["assumptions"] = [
        "one compile per (scene, robot, box): the extended corridor box u[-9,3.7] v[-0.35,2.75] (route m); 5000 pairs "
        "per robot drawn inside it, so the compile is paid once per robot, persisted, and every array task loads it",
        "per-query cost = measured algorithm wall of the screen queries (first call per pair on the one compile; "
        "includes own verifier and the in-query shared gs3d replay), pooled over the booth-box screen (40 pairs) and "
        "the extended-corridor screen (80 pairs). Central = pooled mean; pessimistic = every pair costs the "
        "REACHABLE mean (REACHABLE answers are the expensive ones: shortcut + tighten); bound = every pair at p95",
        "the pooled screens are not uniform samples (bucketed toward lamp/thread pairs); in the extended corridor "
        "the cylinder answers UNKNOWN cheaply on pairs across the u~-5.6 structure, which lowers its central mean -- "
        "hence the pessimistic column",
        "UNREACHABLE answers (certified cut) cost ~0.01-0.2 s and UNKNOWN endpoint answers ~0.01 s: the tail is the "
        "REACHABLE pairs whose path post-processing (shortcut + tighten) runs; p95 bound = every pair at the p95",
        "no refinement / retry loop exists in aerial3d's query: an UNKNOWN is returned, not retried, so there is no "
        "unbounded tail; compile is deterministic and query cost does not grow with the number of pairs",
        "wall excludes Slurm queueing (cpu_short QOS 120 GB / 32 CPU per user shared with ground5k); archive load + "
        "crop is paid once per task that needs a fresh PreparedScene -- here the compile pickle carries it",
        "single-core: the query path is single-threaded numpy/scipy; tasks scale ~linearly while the node is not "
        "oversubscribed (uavconn saw 1.8x compile variance from node contention)",
    ]
    return out


# ----------------------------------------------------------------------------- template
def fill_template(src, dst, tops, screens, est, sacct, extra):
    rows = list(csv.reader(open(src, encoding="utf-8-sig")))
    head, body = rows[0], rows[1:]
    pairs = list(tops["sweeper"]["pairs"])
    cols = [(r, pn) for pn in pairs for r in ROBOTS]
    val = {c: {} for c in cols}
    for (r, pn) in cols:
        demos = {rr: view(tops[rr], pn) for rr in ROBOTS}
        for rr in ROBOTS:
            demos[rr]["route_kind"] = extra.get("route_kind", {}).get(pn, {}).get(rr, "见报告")
        demos.update({k: v.get(pn, v) if isinstance(v, dict) else v for k, v in extra.items() if k.startswith("_")})
        d, s = demos[r], screens[r]
        res, comp = d["result"], d["compile"]
        cold = d["calls"][0]
        warm = [c for c in d["calls"] if c["mode"] == "warm"]
        st = {x["stage"]: x["seconds"] for x in comp["stages"]}
        qs = cold["stages"]
        total = comp["compile_wall_s"] + cold["algorithm_wall_s"]
        def t(x):
            return f"{x:.3f} s；占总时间 {pct(x, total):.1f}%"
        grp = {g: sum(qs.get(n, 0.) for n in names) for g, names in QUERY_GROUPS.items()}
        stages_sum = sum(st.values()) + sum(qs.values())
        others = sum(qs.get(n, 0.) for n in ("locate", "locate_cells", "cut_certificate"))
        rest = total - stages_sum
        big = max([(k, v) for k, v in st.items()] + [(k, v) for k, v in qs.items()], key=lambda kv: kv[1])
        b, host = d["body"], d["host"]
        own = res["verification"]["own"] or {"segments": []}
        pairs_checked = sum(sg.get("pairs_checked") or 0 for sg in own["segments"])
        rep = d.get("replay") or {}
        n_edges = len((rep.get("geometry") or {}).get("reports", []) or []) or None
        e = est["robots"][r]
        pr = comp["pairs"]
        oct_, cel = comp["octree"], comp["cells"]
        sl = d["straight_line"]
        other = demos["cylinder" if r == "sweeper" else "sweeper"]
        v = val[(r, pn)]
        v["实验设置｜代码版本 / Git commit"] = f"版本：aerial3d ground (G1)；commit：{host.get('git_commit', '')[:10]}"
        v["实验设置｜运行日期"] = host["utc"][:10]
        v["实验设置｜硬件与运行环境"] = (f"CPU：{host['node']}（Slurm cpu_short，{host['cpus_per_task']} 核分配，查询单线程）；GPU：无；"
                                  f"RAM：作业申请 {sacct.get(r, {}).get('req_mem', '?')}；OS：Linux (RHEL) / Python {host['python']}")
        v["实验设置｜每个配置的 warm-up 次数与正式重复次数"] = (f"Warm-up：0 次（首次调用即 cold）；正式运行：1 cold + {len(warm)} warm "
                                                    f"（同一内存编译） + 1 次从磁盘重载后的查询")
        v["实验设置｜场景名称 / 场景编号"] = (f"uavlamp_gallery_booth_v2（展厅 3DGS + 灯/隔断），route 盒 u[{d['box_route']['lower'][0]},"
                                   f"{d['box_route']['upper'][0]}] v[{d['box_route']['lower'][1]},{d['box_route']['upper'][1]}] m")
        v["实验设置｜场景 Gaussian 数量"] = (f"N_G = {d['crop']['input_supports']:,}（全档案）；裁剪到盒内 {d['crop']['selected_supports']:,}；"
                                     f"本机器人候选 pair {pr['candidate_pairs']:,}")
        p = d["pair"]
        v["实验设置｜Start 与 End"] = (f"Start：route {p['start_uv']}（z_c {b['ground_clearance_m'] + b['half_height_m']:.3f}）；End：route "
                                 f"{p['goal_uv']}；直线距离：{p['dist_m']:.3f} m")
        v["实验设置｜机器人名称与几何参数"] = (f"机器人：{r}；宽：{2 * b['radius_m']:.3f} m；长：{2 * b['radius_m']:.3f} m；高："
                                    f"{2 * b['half_height_m']:.3f} m；其他参数：圆柱体 r={b['radius_m']}，离地 {b['ground_clearance_m']} m，"
                                    f"margin 0.001 m，ground_unicycle")
        v["时间｜单次完整规划总时间（cold start）"] = (f"Median：{total:.3f} s；P95：N/A（n=1；= 编译 {comp['compile_wall_s']:.2f} s + 冷查询 "
                                          f"{cold['algorithm_wall_s']:.3f} s，不含档案加载/裁剪 {e['archive_load_and_crop_s']:.1f} s）")
        wv = [c["algorithm_wall_s"] for c in warm]
        v["时间｜单次完整规划总时间（已有场景结构后的 warm query）"] = f"Median：{q(wv, 50):.3f} s；P95：{q(wv, 95):.3f} s（n={len(wv)}）"
        v["时间｜场景层级结构 / BVH 构建耗时"] = t(st.get("scene_prepare", 0.)) + "（gs3d PreparedScene：协方差校验 + BVH）"
        v["时间｜scene–robot 候选 Gaussian pair 生成耗时"] = t(st.get("pair_candidates", 0.))
        v["时间｜scene–robot 精确几何交互 / 碰撞计算耗时"] = t(st.get("envelopes", 0.) + st.get("audit", 0.)) + "（支撑函数包络表 + 夹逼审计）"
        v["时间｜自由 / 碰撞状态或 configuration-space domain 构建耗时"] = t(st.get("octree", 0.) + st.get("cells", 0.)) + \
            f"（octree 标注 {st.get('octree', 0.):.3f} s + 凸自由胞/portal {st.get('cells', 0.):.3f} s）"
        v["时间｜自适应 refinement 耗时"] = "N/A（无独立 refinement 轮；octree 节点内的方向自适应细化已计入上一行）"
        v["时间｜规划结构 / graph / operator 组装耗时"] = t(st.get("possible_graph", 0.)) + "（possible 连通图；portal 在 cells 阶段内）"
        v["时间｜factorization / preconditioner setup 耗时"] = "N/A（无线性求解）"
        v["时间｜全局求解 / 图搜索 / 数值求解耗时"] = t(qs.get("graph_search", 0.)) + "（portal 图 A*）"
        v["时间｜路径提取耗时"] = t(grp["path extraction (lift + shortcut + tighten + merge)"])
        v["时间｜最终连续碰撞检查与路径验证耗时"] = t(grp["own continuous verifier"] + grp["shared gs3d replay"]) + \
            f"（自有验证 {grp['own continuous verifier']:.3f} s + 共享 gs3d 重放 {grp['shared gs3d replay']:.3f} s）"
        v["时间｜其他未归类耗时"] = t(others) + "（端点定位；不可达时含割证书）"
        v["时间｜各阶段耗时之和与总时间的差值"] = f"{rest:.3f} s；差值占总时间 {pct(rest, total):.1f}%"
        v["时间｜当前最耗时阶段"] = f"阶段：{big[0]}；耗时：{big[1]:.3f} s；占总时间：{pct(big[1], total):.1f}%"
        v["计算量｜单次规划 collision / contact 查询次数"] = (f"自有验证 pair 检查 {pairs_checked:,} 次；共享重放 oracle edge 检查 "
                                                 f"{n_edges if n_edges is not None else 'n/a'} 次")
        v["计算量｜scene–robot 候选 Gaussian pair 数量"] = f"Candidate pairs：{pr['candidate_pairs']:,}"
        ap = comp["active_pairs"]["union"]
        v["计算量｜scene–robot 实际有效 Gaussian pair 数量"] = f"Active pairs：{ap:,}（作为自由胞支撑面或 BLOCKED 叶证书的 pair）"
        v["计算量｜有效 pair 比例"] = f"Active / Candidate：{pct(ap, pr['candidate_pairs']):.1f}%"
        v["计算量｜broad-phase 剪枝比例"] = (f"{pct(pr['pruned_pairs'], pr['opacity_selected']):.1f}%（盒内 opacity>tau 的 "
                                       f"{pr['opacity_selected']:,} 个中被 z 窗/域剪掉 {pr['pruned_pairs']:,}）")
        rw = qs.get("shared_verification")
        v["计算量｜单次 collision / contact query 平均耗时"] = (f"Mean：{1000 * rw / n_edges:.2f} ms；P95：N/A（查询内共享 gs3d 重放耗时 / 边数）"
                                                   if rw and n_edges else "N/A（无路径，无需重放）")
        v["计算量｜重复或近重复 collision / contact query 比例"] = "N/A（未统计）"
        lv = oct_["leaves_by_status"]
        v["计算量｜单次规划使用的 configuration states / 节点 / 网格数量"] = (f"octree 叶 {oct_['leaves']:,}（SAFE {lv['SAFE']:,} / BLOCKED "
                                                            f"{lv['BLOCKED']:,} / UNKNOWN {lv['UNKNOWN']:,}）；凸自由胞 {cel['cells']:,}")
        v["计算量｜orientation bins 数量"] = "N/A（轴对称圆柱，yaw 不影响碰撞）"
        v["计算量｜单次规划使用的边数"] = f"portal {cel['portals']:,}（图边）"
        v["计算量｜稀疏 operator 非零元素数量"] = "N/A"
        v["计算量｜refinement 轮数"] = "N/A"
        v["计算量｜solver 迭代次数"] = "N/A（A* 图搜索，非迭代求解）"
        v["计算量｜峰值 CPU 内存"] = (f"{sacct.get(r, {}).get('maxrss_gb', float('nan')):.2f} GB（作业 MaxRSS，含档案加载）；进程内编译后 "
                                f"{comp['peak_rss_mb'] / 1024:.2f} GB")
        v["计算量｜峰值 GPU 显存"] = "N/A"
        v["复用性｜同一场景、同一机器人，仅更换 start/end 后的总规划时间"] = (f"Median：{e['query_median_s']:.3f} s；P95：{e['query_p95_s']:.3f} s"
                                                         f"（筛选 {e['screen_queries']} 对，同一编译）")
        v["复用性｜仅更换 start/end 时必须重新计算的阶段"] = "端点定位、portal A*、路径提取、自有验证、共享 gs3d 重放（query）"
        v["复用性｜仅更换 start/end 时可以直接复用的阶段"] = "scene_prepare(BVH)、pair 生成、包络表、octree、凸胞+portal、possible 图、审计（compile 全部）"
        v["复用性｜仅更换 start/end 时可复用计算占原 cold-start 时间比例"] = f"{pct(comp['compile_wall_s'], total):.1f}%"
        v["复用性｜相同 start/end 重复查询时的缓存命中率"] = ("编译产物 100% 复用（warm/重载调用中无任何编译阶段）；查询结果本身不缓存 → 0%；"
                                             f"cold/warm/重载结果完全一致：{d['identical']}")
        for k, name in ((1, "sweeper"), (2, "cylinder")):
            bb, dd = demos[name]["body"], demos[name]
            v[f"机器人对比｜机器人 {k} 参数"] = (f"名称：{name}；宽：{2 * bb['radius_m']:.3f} m；长：{2 * bb['radius_m']:.3f} m；高："
                                         f"{2 * bb['half_height_m']:.3f} m；其他：离地 {bb['ground_clearance_m']} m，体带 "
                                         f"{bb['ground_clearance_m']:.2f}–{bb['ground_clearance_m'] + 2 * bb['half_height_m']:.2f} m")
            rr = dd["result"]
            v[f"机器人对比｜机器人 {k} 规划结果"] = (
                f"成功 / 无路径 / 超时：{rr['status']}；路线类型：{dd.get('route_kind', '见报告')}；路径长度："
                f"{(rr.get('metrics') or {}).get('path_length_m', float('nan')):.3f} m；规划时间：{dd['calls'][0]['algorithm_wall_s']:.3f} s"
                f"（冷查询）；最小 clearance：{rr.get('clearance_lower_m') if rr.get('clearance_lower_m') is not None else float('nan'):.4f} m")
        v["机器人对比｜机器人 3 参数"] = "名称：uav；宽：0.50 m；长：0.50 m；高：0.20 m；其他：本阶段未运行（UAV 见 uavconn 分支）"
        v["机器人对比｜机器人 3 规划结果"] = "本阶段未运行"
        tt = {n: demos[n]["compile"]["compile_wall_s"] + demos[n]["calls"][0]["algorithm_wall_s"] for n in ROBOTS}
        v["机器人对比｜同一场景、同一 start/end，更换机器人后的总规划时间"] = (f"机器人 1：{tt['sweeper']:.2f} s；机器人 2：{tt['cylinder']:.2f} s；"
                                                          "机器人 3：未运行（编译 + 冷查询）")
        v["机器人对比｜更换机器人后重新构建 scene–robot planning representation 的时间"] = (
            f"机器人 1→2：{demos['cylinder']['compile']['compile_wall_s']:.2f} s；机器人 2→3：未运行；机器人 1→3：未运行")
        v["机器人对比｜更换机器人时必须从头重新计算的阶段"] = "pair 生成、包络表、octree、凸胞+portal、possible 图、审计（依赖 r、半高、z 窗）"
        v["机器人对比｜更换机器人时可直接复用的阶段"] = "档案加载+裁剪、scene_prepare（PreparedScene/BVH，传入 prepared= 即可复用）"
        v["机器人对比｜更换机器人时可复用计算占原总计算量比例"] = f"{pct(st.get('scene_prepare', 0.), total):.1f}%（仅 scene_prepare；不含档案加载）"
        v["机器人对比｜连续完成三种机器人规划所需总时间"] = f"{tt['sweeper'] + tt['cylinder']:.2f} s（两种；UAV 未运行）"
        v["机器人对比｜三种机器人是否得到不同路径"] = f"是 / 否：{demos.get('_differ', '见报告')}；差异描述：{demos.get('_differ_text', '见 docs/aerial3dg_g1.md')}"
        for key in ("机器人对比｜三种机器人得到完全相同路径的比例（多组 start/end）", "机器人对比｜更换机器人后可达 / 不可达结论发生变化的比例",
                    "机器人对比｜更换机器人后主要路线发生变化的比例", "机器人对比｜小机器人选择窄路捷径的比例",
                    "机器人对比｜大机器人避开无法通过窄路的比例", "机器人对比｜高机器人避开低矮通道的比例"):
            v[key] = demos.get("_multi", {}).get(key, "待 G2/G3（5000 对）")
    out = [[head[0]] + [f"{r} · {pn}（本次测量）" for r, pn in cols]]
    for row in body:
        key = row[0]
        out.append([key] + [val[c].get(key, row[1] if len(row) > 1 else "") for c in cols])
    with open(dst, "w", encoding="utf-8-sig", newline="") as f:
        csv.writer(f).writerows(out)
    return out


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--demo", type=Path, required=True)
    p.add_argument("--screen", type=Path, nargs="+", required=True, help="screen dirs (pooled for query costs)")
    p.add_argument("--sacct", type=Path, required=True, help="json {robot: {maxrss_gb, req_mem}}")
    p.add_argument("--extra", type=Path, help="json with _differ/_differ_text/_multi and route kinds")
    p.add_argument("--template", type=Path, default=Path("/scratch/wg2381/splathjb/measurement_template.csv"))
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    demos = {r: json.loads((a.demo / f"demo_{r}.json").read_text()) for r in ROBOTS}
    screens = {}
    for r in ROBOTS:
        docs = [json.loads((sd / f"screen_{r}.json").read_text()) for sd in a.screen]
        screens[r] = {**docs[-1], "results": [x for dd in docs for x in dd["results"]],
                      "sources": [str(sd) for sd in a.screen]}
    sacct = json.loads(a.sacct.read_text())
    extra = json.loads(a.extra.read_text()) if a.extra else {}
    est = estimate(demos, screens, sacct)
    (a.out / "estimate_5000.json").write_text(json.dumps(est, indent=1, ensure_ascii=False) + "\n")
    fig_timing(demos, screens, est, a.out / "timing.png")
    fill_template(a.template, a.out / "measurement_g1.csv", demos, screens, est, sacct, extra)
    print(json.dumps({r: {k: round(v, 3) if isinstance(v, float) else v for k, v in est["robots"][r].items()}
                      for r in ROBOTS}, ensure_ascii=False, indent=1))
    print("total cpu h (compile once, both robots):", round(est["total_cpu_h_both_robots_compile_once"], 3))


if __name__ == "__main__":
    main()
