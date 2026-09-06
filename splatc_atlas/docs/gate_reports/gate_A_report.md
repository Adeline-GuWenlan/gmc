# Gate A report — Sprint A（2026-08-11）

**总判定：`PASS`（当日两轮审计 + de-alignment 补强后升级）** — 六条判据全部 PASS。
blind split 已在 method tuning 之前密封（判据 5，见下）。**授权进入 Sprint B。**

> 历史：初版为 CONDITIONAL PASS（blind 未密封）。当日先经用户质询触发两轮审计
>（保守证书、fine 补跑、V1–V8 独立校验组、ε_t 条款），再完成 de-alignment 参数、
> 三层 manifest 密封与 de-alignment 演示实验，升级为 PASS。

评测环境：本地 macOS（darwin 24.6.0）、Python 3.11 / numpy 1.26 / scipy 1.10。
复现命令见 `splatc_atlas/README.md`；全部数字由脚本生成
（`results/tables/*.json`、`results/figures/*.png`）。

---

## Gate 0 判据（问题合同）

| 判据 | 状态 | 证据 |
|---|---|---|
| 两个独立 collision 实现一致 | **PASS** | tests #5/#14：PW vs 点–椭圆距离，4000 随机 config + 900 随机 pose，符号全一致 |
| rigid transform equivariance | **PASS** | test #7，误差 < 1e-9 |
| theta periodicity（含中心对称）| **PASS** | test #8 |
| schema 可回放 | **PARTIAL** | `configs/g1_pilot.json` + `evaluation/records.py` 就绪；oracle 全量 record 落盘待 Sprint B 接线 |
| 公平 budget 写入配置 | **PENDING** | 依计划：uniform baseline 首跑后定标（Sprint B）|
| 10+ analytic cases | **PASS** | 15 cases，`tests/test_analytic.py`，全部通过（7.7s）|

## Gate A 判据

| # | 判据 | 状态 | 证据 |
|---|---|---|---|
| 1 | medium/fine oracle 结论一致 | **PASS** | gate sweep 10 个 fine 档门宽：reachable / components 与 medium 全一致（唯一例外是 knife-edge w=w*，见发现 4）|
| 2 | 三种机器人结果正确 | **PASS** | 三 morphology 在 coarse/medium/**fine** 全部实跑：小圆 1 分量可达；长椭圆 1 分量可达（侧身）；大圆 2 分量不可达。（初版报告曾在只跑过长椭圆 fine 的情况下写"三档一致"，属过度声明，审计后补跑坐实——见审计节）|
| 3 | thin-gate 曲线正确 | **PASS** | 14 个门宽，oracle 中面区间 vs 解析 Θ(w) 误差 ≤ 0.25°（= θ 测量分辨率）；闭合点恰在 w*=2b；`gate_sweep.png` |
| 4 | oracle path 连续安全 | **PASS（审计后升级）** | 初版只是逐边 4 点**密集采样**（在最贴 gate 处不构成保守证书：采样间隔内体点位移 ~0.02 m > 米制间隙 ~0.01 m）。审计后实现真正的 swept 证书（free-bubble：`m(q1)+m(q2) > |Δp|+a·|Δθ|`，不闭合则二分）：medium 路径 certified，min 米制余量 9.95 mm，二分深度 2，211 次 margin 检查；fine 路径 certified，min 余量 2.59 mm，深度 3 |
| 5 | blind cases 密封 | **PASS** | 先补 de-alignment 参数（tests #16/#17 验证倾斜/偏移几何），再生成 dev 24 / validation 18 / blind 21 episodes 并 SHA-256 密封（`results/manifests/split_seals.json`：blind `408d47ef…`）。blind 含临界区随机 de-alignment、just-unreachable、morphology holdout（R_blind_ellipse，仅 blind 引用）、极端倾角、multi-goal batch；不带标签 |
| 6 | 记录可复现 | **PASS** | 配置冻结于 `configs/g1_pilot.json`；实验无随机性（tests 固定 seed）；图表由脚本重生成 |

## 审计附录（用户质询后的 challenge-清单复核，2026-08-11 晚）

按顶层 README 规则 6，对每条已验证项写"能排除什么 / 不能排除什么"，并专攻后者。
复核发现并修复了两个真实问题：

| 验证项 | 能排除 | 不能排除（→ 处置）|
|---|---|---|
| 15 个解析 tests | 两 checker 公式性错误、等变/周期性 bug | 近相切区（\|m\|≤1e-5 被显式排除）两 checker 的数值分歧 → 已在报告注明该排除域 |
| 300×3 随机 pose 双 checker 一致 | pose 级符号系统错误 | 大网格路径上的 alive-filter / scatter-min 组合 bug → 靠 knife-edge 观测（ρ≈9e-5 与波纹解析预算吻合）交叉佐证 |
| "path certified" | 采样点上的碰撞 | **采样间隔内的碰撞（贴 gate 处不闭合）→ 已实现保守 swept 证书并重认证，PASS** |
| "三档分辨率一致" | 长椭圆的分辨率敏感性 | **两个圆形机器人从未跑过 fine → 已补跑，结论坐实** |
| 解析 gate 公式 | corridor 门的必要/充分性（完全浸没论证，T≥2a）| thin-wall / 倾斜门变体（→ spec §5 已分开；倾斜门是 de-alignment 待办的一部分）|

结论：Gate A 判定维持 `CONDITIONAL PASS`（阻塞项不变：blind split）；判据 4 的证据等级
从"采样检查"升级为"保守证书"。

## 独立校验组（V1–V8，第二轮质询后新增，最终 ALL PASS）

`experiments/oracle/validate_sprint_a.py` — 原则：每个结论都经一条与生产管线尽量不共码的
独立路径重推。运行 188s，退出码即结论。

| 检查 | 独立于什么 | 结果 |
|---|---|---|
| V1 场景镜像精确性 | 生成器实现（集合级坐标断言）| PASS，390 primitives |
| V2 蛮力 oracle（checker#2 + 三角不等式界，全 72 切片，无 PW/KD-tree/镜像）| PW、broad phase、对称优化 | 3 morphology 硬不一致 = 0 |
| V3 MC footprint 第三检查器（1200 采样点/pose，800 poses 含边界壳层）| 两套解析 checker 的全部数学 | 矛盾 = 0 |
| V4 free-mask 对称性（y/θ 与 x/θ 镜像，coarse+medium 全数组）| 数值实现细节 | diff = 0（首轮抓到 120 个相切噪声 cell → 引出 ε_t 条款）|
| V5 独立 BFS 分量 + θ-roll 不变性 | scipy.ndimage + wrap 合并 | 计数与 sizes 全等 |
| V6 手写 heapq Dijkstra | scipy.sparse.csgraph | 代价一致到 1e-9 |
| V7 路径纪律（步进合法性、代价重加和、端点、保守证书）| 图构建代码 | PASS，min 余量 9.95 mm |
| V8 蛮力中面 gate 复算（720 θ，无对称捷径）| 生产测量函数 | 集合逐位一致，与解析式吻合 |

**校验组产生的合同修正：** FREE 判定加 ε_t = 1e-9 保守 tie-break（problem_spec §2）——
G1 整数几何 × 整数网格产生测度零精确相切 pose，标签不能留给 ±1 ulp 浮点噪声决定。
该修正只影响数学上恰好相切的 cell，此前发布的 sweep / rotate-door 数字全部不变
（knife-edge 信号 ρ≈9e-5 高于 ε_t 五个量级，仍在）。

## 决定性产出

1. **`rotate_door_path_medium.png`** — 计划 10.2 节要求的"第一张决定性结果"：长椭圆从
   (−2,0,90°) 出发，提前旋转，以 ~25° 侧身穿过 w=0.7 corridor 门，到达 PointGoal；
   全程无 seed、连续认证。cost=4.267。
2. **`gate_sweep.png`** — M3 曲线：解析 Θ(w) 与 oracle 测量全程吻合，物理闭合点精确。
3. `slices_*.png` — 三 morphology 的具身 free-space 结构可视化。

## Sprint A 发现（已回写 worklog / 03 号文档）

1. **corridor vs thin-wall 公式**：03 的投影公式只对 corridor 门精确；thin-wall 是弦条件。
   benchmark 默认 corridor（T=1.2），两公式已入 problem_spec §5。
2. **对齐网格作弊问题（对 Claim C 关键）**：门中心/门角与网格相位对齐时，uniform 网格在
   任意薄 gate 上不失败（实测 coarse 在 ±3.73° gate 仍正确）。G1 生成器必须加
   `door_center_offset` / `door_tilt` 并随机化相位，否则 thin-gate 对比不可测。
   已补进 03 号 G1 规格；**这是 blind split 前的必做项**。
3. **最优路径贴 gate 边缘**：J-最优解只转到 gate 边界内侧（25° vs ±26.7°），min ρ 小是
   cost-optimality 的结果，不是安全缺陷；clearance 与 optimality 指标必须分开报告。
4. **w=w* 是 knife-edge**：恰好临界宽度下 disc 包络波纹（±1e-4）产生单-cell 孤岛
   （ρ≈9e-5，且位置精确落在 disc 圆心中点——与解析波纹预算吻合）；该参数点标
   `ORACLE_UNRESOLVED`，不进入主评估。

## 性能基点（后续 budget 定标参考）

- G1 场景 390 primitives；oracle 单 morphology：coarse 0.6s / medium 5.2s / fine ~40s（长椭圆，本机）。
- medium Dijkstra（~0.9M free nodes）0.9s。
- 全 14 宽度 sweep（含 10 个 fine）约 8 分钟。规模化到 SplatC-Procedural 时迁 HPC sbatch。

## De-alignment 演示实验（Claim C 现象的首次实测）

`dealign_demo.py`：w ∈ {0.502…0.54} × {aligned, offset 13mm, tilt 7°, both} ×
coarse/medium/fine，全部案例解析上可达（w > 2b）。结果（`dealign_demo.png`）：

- **aligned 全部正确可达**（网格相位红利，之前发现的"作弊"）；
- **任何 de-alignment 下，w ≤ 0.52 连 fine（dx=25mm, dθ=1.25°）都 false-unreachable**；
- offset 在 w=0.54 恢复（可行 y 窗 ±20mm 覆盖 25mm 网格线；数值与解析窗宽完全对账）；
- tilt 案例分量数 4→12 随分辨率增长：走廊内存在孤立 free 格但 6-连通链被斜向混叠打断
  ——false-unreachable 的两种机制（整窗错过 / 链断裂）都被捕获。

**对 Stage 1 协议的推论（重要）**：临界区 de-aligned 案例超出了 fine 均匀 oracle 的能力，
此类案例的真值必须来自解析标签（G1 由旋转等变性给出）或自适应细化；网格收敛协议对它们
必须输出 `ORACLE_UNRESOLVED` 而不是硬标。这同时是论文论点的正面证据：**均匀离散在
thin gate 处从根上低效——连 oracle 都需要解析/自适应帮助**。

## 移交 Sprint B 的待办

1. ~~oracle 全量 OracleRecord 落盘接线~~（已完成:HPC 阵列 17183942,42 份记录,
   schema 校验强制化）;
2. baseline/evaluator 接口骨架接第一个 uniform baseline + budget 定标;
3. uniform / generic-adaptive / contact-cues 三条 Pareto 曲线（Sprint B 决定性早期实验），
   数据源直接用本报告的 de-aligned 临界区案例;
4. **G5 remote-closure 场景族 + dev 侧 multi-goal records**（Stage 1 交付清单遗留项,
   设计符合性审计補记）;
5. smooth overlap `c_ij` 函数族（value/gradient,compiler 的 cue 输入,Sprint B 必做）;
6. 多 primitive 机器人与各向异性场景 primitive 的评估器扩展（R3–R6 / G8 / G9 前置）;
7. SE(2) NavMesh feasibility audit + 数据集 license audit + figure skeleton
   （后台线程,已到触发时间）。
