# 任务：把 Neural Green's Functions (NeurIPS 2025, arXiv:2511.01924) 做成我们 3DGS mobility 论文里可以公平比较的 baseline，并跑出对比结果

## 目标

在 `/scratch/wg2381/splathjb` 增加一个 B10 家族的 "Green / harmonic navigation field" baseline，让它在我们自己的 SplatC-Bench 上，与 GMC（certified mobility compiler + robot-conditioned coreset）在同一套 scene / robot / query / checker / 预算下跑出可发表的对比，并写清楚它答得了什么、答不了什么。**结论允许是 baseline 赢——那就照实写。**

## 语境

- 论文本身不是 3DGS 论文。NGF 学的是线性 PDE（可特征分解算子）的 solution operator：从表示 domain 的 volumetric point cloud 抽 per-point feature → 预测 solution operator 的分解 → 数值积分得解；卖点是跨几何、跨 source/boundary function 泛化，比需要 meshing 的数值 solver 快到 350×。所以"转成 3DGS baseline"是语义转写，不是复现：3DGS 场景的 free space 就是它要的 volumetric point-cloud domain，goal 当 source、障碍当 Dirichlet 边界，解出的 harmonic / screened-Poisson field 下降就是 navigation field。
- 它正面打我们两条 claim：(1) Green operator 天生就是 compile-once/query-many——换 goal 只换 source function，不重建表示；这正是 Claim B 的领土。(2) 如果一个直接在原始 splats 上算/学的 field 就够用，那"robot-conditioned coreset + certified compiler"就没有必要。**所以这个 baseline 必须做强，不许做弱。**
- 我们这边的权威文档：`06_GMC_DESIGN_MANUAL_v1.md`（theory-first 设计，三值认证 M_safe ⊂ M_true ⊂ M_possible）、`00_MASTER_EXECUTION_PLAN.md`（claim ladder 与已判负边界）、`02_BASELINES_AND_PUBLIC_DATASETS.md`（B10 槽位 + §4 公平协议）、`03_SPLATC_BENCH_SPEC.md`（G1–G10 场景族、R1–R6 机器人、E1–E8 episode、oracle 协议、§9 schema）、`gmc/GOAL.md`（当前证据边界，别把未闭合的当已闭合）、`docs/design/SPLATC_v2_Mobility_Coreset_Design_Revision.md`（现行 v2 主线）。代码在 `gmc/src/gmc`，CLI 是 `python -m gmc.cli {compile,query,verify,benchmark}`。
- 已成立的硬边界，不要推翻重来：opacity / render alpha 不定义物理碰撞；一切安全与拓扑结论最终由 independent hard-support 连续 checker 背书；P3.8 已预注册判负——不许回到"等预算下更快找到细门路径"的 claim。我们的护城河在输出物：closure threshold、gate 角度区间、route class 枚举、三值 REACHABLE/UNREACHABLE/UNKNOWN 证书、compile-once/query-many 摊销。

## 必须守住的比较纪律

- 同一 scene/robot/query 集合；所有方法的最终路径由同一个 hard checker 复验；统一 path cost（02 §4.4）；分开计时（预处理 / build / per-goal query / path extraction / certification / morphology update）；三条预算曲线（02 §4.3）；随机方法报 seed、median、IQR。
- baseline 不许读 oracle 的 free mask 或 path；blind split 按 03 §8。
- baseline 至少两档强度：**full-resolution exact-domain Green**（能力上限；这一档赢不了我们也认）和 **budgeted 版**（coarse grid / 谱截断 / low-rank，或真训一个 NGF-style operator）。
- 禁止的话术：不能声称 scalar field 在信息论上无法表示 topology（06 §14.1）；不能声称连续精确 Green 理论上穿不过可达窄门（02 B10）。允许的 claim 只有：有限 representation budget 下 thin gate 何时被抹平，以及输出物差异（证书 / 角度区间 / route class）。
- 朝向是我们的核心，NGF 是纯几何域算子。你自己决定怎么诚实地 lift（per-yaw C-space slice 域、robot-conditioned 域，或别的），把选择理由和它对 baseline 强弱的影响写进报告；如果某个 lift 让 baseline 不公平地变弱，指出来并给出更强版本。
- **跑之前先预注册判决规则**：比哪些指标、什么阈值算谁赢、哪些格子标 N/A，写进报告，跑完不改。

## 环境

NYU HPC。登录节点只改代码、查状态、提交作业。重活一律 `sbatch`（模板见 `gmc/hpc/baseline_suite_wg2381.sbatch`：account `torch_pr_527_general`，python `/scratch/wg2381/.conda/envs/gmc-venv/bin/python`，`PYTHONPATH=src`，日志 `gmc/logs/%x-%j.out`）。不要 `srun --pty`，不要 `--wait`，提交后异步轮询 `squeue -j <jobid>`。要装依赖或建新 env，先说明装什么、装到哪，再动手。

## 交付

1. baseline 实现（放 `gmc/src/gmc/baselines/`），带测试，和现有全量测试套一起过。
2. 可复现的 sbatch + 结果 artifact（JSON，对齐 03 §9.6 MethodRunRecord）。
3. 对比报告 `docs/baselines/ngf_green_vs_gmc.md`：预注册规则、方法转写与 lift 选择的理由、公平性核对、表与图、baseline 赢在哪、我们赢在哪、以及这个 baseline 结构上答不了的问题清单。
4. 一句话结论：这个 baseline 是否改变 claim ladder；如果改变，改哪一条。

自己判断顺序和范围，不确定的地方在报告里写清假设，不要停下来等确认。
