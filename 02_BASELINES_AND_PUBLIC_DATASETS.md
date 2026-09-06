# SplatC-Atlas — Baselines 与公开数据集执行规范

**检索状态：** 按公开论文、项目页和代码仓库核查至 2026-08-11。  
**原则：** baseline 的说服力不取决于“年份越新越好”，而取决于是否直接攻击我们的 claim、是否足够强、是否可公平复现。

---

# 1. 我们的 claim 决定 baseline

当前主 claim 是：

\[
\{h_{ij}(\cdot),c_{ij}(\cdot)\}
\xrightarrow{\mathcal C}
\mathfrak R_{E,R}
\]

能够在不保存 converged dense C-space 的条件下，保留 morphology-conditioned connectivity 与 thin orientation gates，并支持 reusable multi-goal query。

因此 baseline 必须覆盖五种替代解释：

1. 普通均匀离散已经足够；
2. 普通 adaptive refinement 已经足够；
3. 现有 yaw-aware global representation 已经足够；
4. 现有 ellipsoid/Gaussian/CDF geometry 加普通 planner 已经足够；
5. sampling planner 无需任何 compiler 已经足够。

---

# 2. Baseline 分级

## Tier 0 — Ground truth，不是被击败对象

### B0. Dense Exact `SE(2)` Oracle

**作用**

- free/collision truth；
- connected components；
- gate angular interval；
- reachable/unreachable；
- reference path cost；
- convergence audit。

**实现要求**

- coarse / medium / fine，多分辨率收敛；
- independent collision implementation；
- theta periodicity；
- continuous edge validation；
- graph/min-plus shortest path；
- optional harmonic/Green solution 作为第二种 reference。

**禁止**

- candidate 读取 oracle free mask 或 path；
-把某一个未收敛 resolution 当 ground truth。

---

## Tier 1 — 必须进入主实验的 representation baselines

### B1. Uniform `SE(2)` Grid / Fixed Yaw Layers

**攻击的 claim**：我们的优势是否只是采了更多状态？

**公平比较**

- 相同 state budget；
- 相同 collision-query budget；
- 相同 theta channels；
-相同 query backend。

**输出**

- topology accuracy；
- gate recall；
- false-unreachable；
- representation size。

---

### B2. Generic Adaptive C-space

**定义**

同样自适应，但只能使用：

- free/collision label disagreement；
- scalar clearance；
- boundary proximity；
- local occupancy entropy。

不允许使用：

- active pair identity；
- bilateral contact balance；
- morphology continuation；
- gate-specific angular interval detector。

**攻击的 claim**：contact structure 是否真的带来额外价值，而不是“任何 adaptive grid 都可以”。

---

### B3. SE(2) Navigation Mesh / Faithful SE(2)-NavMesh-style Baseline

**论文**：*SE(2) Navigation Mesh*, 2026, arXiv:2607.01454。  
**关键能力**：

- non-circular robot；
- yaw-dependent traversability；
- yaw-specific layers；
- explicit translation/rotation connectivity；
- reusable global representation；
- ASA pathfinding；
- HM3D 和真实机器人实验。

**为什么是最近邻**

它已经覆盖“机器人 footprint + yaw-aware global representation + narrow passages + multiple queries”。因此我们不能再只 claim：

> 为非圆机器人构造 `SE(2)` global map。

我们的 residual 必须落在：

- continuous Gaussian contacts；
- 不依赖固定 yaw channels；
-更薄 gate 的 matched-budget preservation；
- morphology-parametric update；
- active-contact topology；
- Gaussian primitive decomposition / updates。

**代码状态**（2026-08-12 审计确认：未发布——占位无链接，org 仅网站 repo。
复现触发条件已生效；复现范围与风险评估见
`splatc_atlas/docs/baseline_audit_2026-08-12.md`。对齐参数 N_psi=40。）

项目页标记 `Code (soon)`；论文说明将开源。因此执行策略：

1. 持续监控官方 release；
2. 在官方代码不可用时实现 faithful 2D subset：footprint masks + fixed yaw channels + translational/rotational graph；
3. 文中明确标注 `our reimplementation`，不能伪称官方实现；
4. 使用其公开论文参数与 HM3D subset 尽量对齐。

**主比较曲线**

\[
\text{minimum preserved gate width}
\quad\text{vs}\quad
N_{\theta},\text{states},\text{memory}.
\]

---

### B4. Highway RoadMap (HRM)

**论文**：Ruan et al., *Efficient Path Planning in Narrow Passages for Robots With Ellipsoidal Components*, IEEE T-RO 2023（DOI 10.1109/TRO.2022.3187818）。  
**代码**：`ChirikjianLab/hrm`。（2026-08-12 审计：可用但 2023 年后休眠；**GPL-3.0，
只能进程外调用**；场景/机器人为椭球 CSV，与我们格式接近；依赖 OMPL/FCL/CGAL 需在
overlay 内构建。）

**关键能力**

- robot represented by ellipsoidal components；
- closed-form Minkowski operations / C-space boundary parameterization；
- `SE(2)` 和 `SE(3)`；
- narrow passage planning；
- roadmap reuse；
-与 PRM/RRT 等公开 benchmark。

**为什么危险**

它比普通 PRM 更接近“ellipsoid geometry -> compact free-space representation”。

**适配方式**

- 使用相同 scene/robot ellipsoid supports；
-先跑 HRM 官方 2D planner；
- 相同 start-goal 和 hard checker；
- 记录 build time、roadmap size、collision checks、success、clearance；
- 单独比较 morphology change 是否需要全量重建。

---

### B5. CDF / GCDF + Common Global Query Backend

**基础 CDF**：Li et al., *Configuration Space Distance Fields for Manipulation Planning*, RSS 2024, arXiv:2406.01137。  
**更新近邻 GCDF**：Li et al., *Fast and Safe Trajectory Optimization for Mobile Manipulators With Neural Configuration Space Distance Field*, 2026, arXiv:2601.18548。

**关键能力**

- continuous configuration-space distance；
- gradients；
- neural compact representation；
- GCDF 支持 translational + rotational DoFs 和 unbounded workspace。

**公平 baseline 不能只是 CDF 单独使用**

应比较：

\[
\text{CDF/GCDF representation}
+
\text{与我们相同的 min-plus / graph / Eikonal query backend}.
\]

**攻击的 claim**

- continuous C-space field 是否已经足够；
- scalar distance 是否能恢复 active-contact topology；
- neural CDF 是否在 thin gate 上产生 oversmoothing；
- morphology change 是否需要重新采样/训练。

**推荐版本**

- 主文：analytic/sample-based CDF + same query；
- 若 GCDF 代码/模型可获得：加入 GCDF；
-否则将 GCDF 放入 related work + method discussion，不虚构结果。

---

## Tier 2 — 必须进入规划与 Gaussian 近邻实验

### B6. FOCI

**论文**：*FOCI: Trajectory Optimization on Gaussian Splats*, IROS 2025 Oral, arXiv:2505.08510。  
**代码**：官方项目页公开 GitHub。

**关键能力**

- Gaussian scene + Gaussian robot；
- analytic overlap collision term；
- orientation-aware trajectory optimization；
- tight spaces；
-大规模 Gaussian scenes。

**它攻击什么**

> 为什么不直接用 Gaussian overlap 做 trajectory optimization？

**必须比较**

- straight-line initialization；
- heuristic initialization；
- oracle route initialization（上限，不作为公平主结果）；
-每个 goal 重优化成本；
- unreachable detection；
- remote closure；
- seed sensitivity；
- multi-goal cumulative time。

“能旋转穿门”不能作为我们独立 novelty，因为 FOCI 已经做到 orientation-aware tight-space optimization。

---

### B7. Selective Densification

**论文**：Huang et al., *Selective Densification for Rapid Motion Planning in High Dimensions with Narrow Passages*, 2025, arXiv:2507.15710。

**关键能力**

- multi-resolution sampling；
- narrow passages；
- `SE(2)`, `SE(3)`, `R^14`；
-无需训练的在线 densification。

**攻击的 claim**

> 我们的 contact-guided refinement 是否真的优于先进的 generic multi-resolution narrow-passage planning？

**执行策略**

- 优先使用作者代码；
-若无稳定公开代码，实现其核心多分辨率采样思想或选一个可复现的 multilevel planner；
- 主文中严格区分官方结果与 reimplementation。

---

### B8. LazyPRM* / PRM*

**实现**：OMPL。（2026-08-12 审计：PyPI `ompl` v2.0.1 的 Python 绑定含 PRM/PRM*/
RRT-Connect，**不含 LazyPRM\***——主 baseline 用 PRM\*，除非后续自行补绑定。）

**为什么必须有**

PRM 本身就是可预处理、支持多 start-goal queries 的 reusable roadmap。它直接攻击我们的 multi-goal claim。

**比较合同**

\[
T_{\mathrm{build}} + K\,T_{\mathrm{query}}
\]

与：

\[
T_{\mathrm{compile}} + K\,T_{\mathrm{query}}^{\mathrm{ours}}
\]

共同报告。

**采样器**

- uniform；
- Gaussian/obstacle-based；
- bridge-test 或 narrow-passage sampler（若 OMPL 支持/可实现）。

---

### B9. RRT-Connect

**实现**：OMPL。

**作用**

攻击单查询场景：

> 只有一个 goal 时，为什么不直接在线规划？

**要求**

- 多随机种子；
-相同 checker；
- equal wall-clock / collision-query budget；
- success distribution；
- path cost / clearance；
-不与 representation topology 指标强行比较，标为 `N/A`。

---

## Tier 3 — 条件性 baseline / ablation

### B10. Harmonic / Green / Laplacian

只有当正文讨论 diffusion/operator semantics 时进入主表。

推荐两个版本：

- full-resolution exact-domain Green：能力上限；
- budgeted Green：coarse grid / spectral truncation / low rank。

公平 claim 只能是：

> 在有限 representation budget 下 thin gate 是否更早被弱化。

不能声称连续精确 Green 理论上无法穿过可达窄门。

### B11. CDFlow

**论文**：*CDFlow: Generative Gradient Flows for Configuration Space Distance Fields via Neural ODEs*, 2025, arXiv:2509.13771。

适用于：

- 若论文加入 neural CDF comparison；
-若 thin-boundary sharpness 成为主要 claim；
- 若代码可用且适配成本可控。

否则放 related work/appendix。

### B12. Riemannian Distance Fields / Geodesic Flows

**论文**：Li & Qiu & Calinon, IJRR 2026。

只有当模块 4 使用 Riemannian/Finsler Eikonal 或 geodesic flow 时进入 query baseline。

### B13. Splat-Nav / SPLANNING

- Splat-Nav：Gaussian map 上安全走廊 + Bézier path；
- SPLANNING：normalized GS 上 risk-aware trajectory optimization。

更适合作为 Gaussian planning related work 或 public 3DGS qualitative baseline；对 embodied `SE(2)` topology 不如 FOCI 直接。

---

# 3. 最小主表 baseline 集合

在正文空间有限时，主表至少保留：

1. Dense Oracle；
2. Uniform C-space；
3. Generic Adaptive C-space；
4. SE(2) NavMesh-style；
5. HRM；
6. CDF/GCDF + same query；
7. FOCI；
8. LazyPRM*；
9. RRT-Connect；
10. Ours。

Selective Densification 优先进入窄通道专表；Green/CDFlow 依据最终 claim 决定主文或附录。

---

# 4. 公平比较协议

## 4.1 统一 hard geometry truth

所有方法最终路径由同一 independent hard-support checker 重新认证。

## 4.2 分开计时

必须分别报告：

- scene/robot preprocessing；
- representation build/compile；
- per-goal query；
- path extraction；
- final certification；
- morphology update。

## 4.3 三种预算

每个 representation method 至少画：

1. topology accuracy vs collision queries；
2. gate recall vs memory/states；
3. success/path cost vs wall-clock。

## 4.4 相同 path cost

建议统一：

\[
J(\gamma)=\int
\sqrt{\dot x^2+\dot y^2+\ell_R^2\dot\theta^2}
\,dt
+
\lambda_{\mathrm{clr}}J_{\mathrm{clearance}}.
\]

不同方法可使用自己的内部 objective，但最终按统一 metric 重新评分。

## 4.5 随机方法

报告：

- seeds；
- median；
- interquartile range；
- success rate；
- timeout / collision-query cap。

## 4.6 不可运行 baseline

代码未公开或无法稳定适配时：

- 记录版本、commit、失败原因；
-做 faithful subset reimplementation；
-明确标注；
-不把论文中的数字复制到我们的表中冒充同条件结果。

---

# 5. 公开数据集：有什么，缺什么

## D1. BARN

**来源**：Perille et al., *Benchmarking Metric Ground Navigation*, 2020。  
**规模**：300 个二维拥挤环境；提供 difficulty ordering 与生成不同 robot footprints 的工具。

**可用部分**

- external 2D clutter layouts；
- public, standardized navigation tasks；
-可生成不同 footprint 的场景；
-适合 OMPL / NavMesh / Atlas 对比。

**缺口**

- 标准任务并不标注连续 orientation gate；
-通常是 occupancy-grid ground truth；
-无 Gaussian scene/robot contact functions；
-没有 morphology bifurcation labels。

**我们的使用方式**

- 只用 layouts；
-重新定义 non-circular robots；
-重新生成 `SE(2)` starts/goals；
-构造自己的 dense oracle 和 gate labels；
-不直接使用原 challenge 的 Jackal scores 作为核心结论。

---

## D2. Habitat-GS

**来源**：Habitat-GS 官方仓库与数据发布，ECCV 2026。  
**规模**：129 个 3DGS scenes：110 train / 19 val。

公开资产包含：

- 65 个 self-reconstructed scenes；
-64 个 InteriorGS scenes；
- PointNav/ImageNav/ObjectNav/VLN episodes；
- self-reconstructed scenes 中包含 3DGS、collision mesh 和 NavMesh；
- InteriorGS 部分通常包含 3DGS + NavMesh。

**优点**

-直接提供 public Gaussian scenes；
-与 Habitat navigation tooling兼容；
- self-reconstructed subset 有 collision mesh，可做外部 reference；
-适合 qualitative/realism validation。

**缺口**

-标准 episodes 是 yaw-action navigation，不是 morphology-conditioned `SE(2)` topology benchmark；
- NavMesh 通常假定特定 agent footprint；
- reconstructed GS 与 collision mesh 可能不完全一致；
-无 orientation-gate truth。

**推荐使用**

优先选择 self-reconstructed subset：

```text
3DGS -> candidate input / visualization
collision mesh -> external hard reference
our robot library -> morphology episodes
our oracle -> reachability/gate labels
```

但核心 paper 必须明确：这些场景违反“perfect Gaussian geometry”的理想假设程度如何，并把 mismatch 单独统计。

---

## D3. InteriorGS

**来源**：官方 GitHub / Hugging Face，2025。  
**规模**：1,000 个 indoor 3DGS scenes，80+ environment types，提供：

- compressed 3DGS；
- object boxes/semantics；
- occupancy maps；
- `structure.json` floorplans、walls、doors、instances。

**优点**

-场景规模大；
-直接 Gaussian；
-有 floorplan/door structure，便于自动筛选窄门；
- occupancy 可用于 proposal generation。

**缺口**

- occupancy/nav data 不是 non-circular `SE(2)` truth；
-硬碰撞 support 仍需我们定义/验证；
- dataset license 必须逐条检查派生发布权限；
-无官方 planner benchmark 与 gate labels。

**推荐使用**

- 从 `structure.json` 筛选 door/corridor scenes；
-使用少量代表性 subset；
-自建 robot morphology episodes 和 oracle；
-不要把 occupancy map 当最终 hard truth。

---

## D4. HM3D

**来源**：Ramakrishnan et al., 2021。  
**规模**：1,000 个 building-scale textured mesh reconstructions。

**优点**

- large-scale、完整、Habitat-compatible；
- SE(2) NavMesh 已使用 HM3D，便于直接对齐；
-适合多房间、多楼层和真实布局。

**缺口**

-不是 Gaussian scene；
- access/license 有 academic restrictions；
-需要 mesh-to-Gaussian/ellipsoid conversion；
-没有我们的 morphology/gate labels。

**推荐使用**

- 选择与 SE(2) NavMesh 展示相近的 6-10 个 scenes；
-统一 mesh reference；
-对 ours 和 SE(2) NavMesh-style 使用相同几何；
-转换误差单独报告。

---

## D5. Replica

**来源**：Straub et al., 2019。  
**规模**：18 个高质量 indoor scenes，dense meshes、textures 和 semantics。

**优点**

-小而精，容易做逐场景 QA；
- mesh quality 高；
-Habitat-compatible；
-适合初期 public adapter debugging。

**缺口**

-数量少；
-不是 Gaussian；
-无 morphology-conditioned labels。

**推荐使用**

在 Habitat-GS/HM3D adapter 不稳定时作为 public-scene fallback。

---

## D6. 3D-FRONT

**来源**：Fu et al., 2020。  
**规模**：18,968 furnished rooms，13,151 textured furniture models（论文版本统计）。

**优点**

- synthetic exact-ish geometry；
-大量 doors、furniture gaps 和 room layouts；
-容易程序化生成 robot episodes；
-比真实扫描更少 reconstruction ambiguity。

**缺口**

-不是 Gaussian；
- access/license 与下载流程需确认；
-room-centric，不一定有 building-scale connectivity；
-需 deterministic conversion。

**推荐使用**

仅当 procedural benchmark 需要更多真实家具布局时加入，不作为第一优先。

---

## D7. MotionBenchMaker

**来源**：Chamzas et al., RA-L 2022。  
**内容**：40 个 prefabricated manipulation datasets（5 robots x 8 environments），每个 100 problems，并提供生成工具。

**价值**

-借鉴 benchmark generator、manifest、统一 planner runner 和可复现协议。

**不适合作为主数据**

-主要是 manipulation；
-机器人和任务不匹配移动 `SE(2)` PointGoal；
-不提供我们的 orientation-gate labels。

---

# 6. 推荐的公开数据组合

## 核心论文组合

### Layer A — 我们的解析 benchmark

`SplatC-Gates`：所有核心 topology/thin-gate claim 的 ground truth。

### Layer B — BARN

公共二维外部压力测试：证明不是只对手工门有效。

### Layer C — Habitat-GS / InteriorGS

公共 Gaussian scenes：证明方法能接入真实规模的 Gaussian assets。

### Layer D — HM3D subset

用于与 SE(2) NavMesh 对齐；如果实现/访问成本太高，可以降为附录或后续。

## 不建议本轮同时全部使用

BARN + Habitat-GS/InteriorGS 已覆盖二维外部布局和 Gaussian realism。Replica/3D-FRONT 只在 adapter 或 scene diversity 有明显缺口时加入。

---

# 7. 公开数据集不能替代自建 benchmark 的原因

没有一个现成数据集同时提供：

- scene Gaussian hard supports；
- robot Gaussian hard supports；
-同一 scene 的多 morphology；
- continuous `SE(2)` collision truth；
- orientation-gate interval；
- critical morphology threshold；
- topology birth/death；
- remote closure；
- reusable multi-goal query labels。

因此我们不需要重新采集原始视觉数据，但必须开发新的 **benchmark layer**：

```text
public raw scene assets
+
our robot morphologies
+
our task generator
+
our converged oracle annotations
+
our unified evaluator
```

这项工作应被称为 controlled benchmark suite，而不是夸大为全新的大型视觉数据集。

---

# 8. Baseline 实施优先级

若单负责人资源有限，按以下顺序：

1. Dense Oracle；
2. Uniform；
3. Generic Adaptive；
4. SE(2) NavMesh-style；
5. HRM；
6. CDF/GCDF + common query；
7. FOCI；
8. LazyPRM*/RRT-Connect；
9. Selective Densification；
10. optional Green/CDFlow/Riemannian。

这不是按论文年份排序，而是按“对主 claim 的威胁程度 / 适配成本”排序。

---

# 9. 主要参考链接

## Closest representations / planners

- SE(2) Navigation Mesh: https://arxiv.org/abs/2607.01454
- SE(2) NavMesh project: https://se2-navmesh.github.io/
- HRM paper/project: https://chirikjianlab.github.io/hrm-planning-page/
- HRM code: https://github.com/ChirikjianLab/hrm
- CDF: https://arxiv.org/abs/2406.01137
- GCDF: https://arxiv.org/abs/2601.18548
- FOCI: https://arxiv.org/abs/2505.08510
- FOCI project: https://rffr.leggedrobotics.com/works/foci/
- Selective Densification: https://arxiv.org/abs/2507.15710
- CDFlow: https://arxiv.org/abs/2509.13771
- Splat-Nav: https://arxiv.org/abs/2403.02751
- OMPL: https://ompl.kavrakilab.org/

## Public datasets

- BARN: https://arxiv.org/abs/2008.13315
- Habitat-GS: https://github.com/zju3dv/habitat-gs
- Habitat-GS dataset: https://huggingface.co/datasets/RukawaY/gs_scenes
- InteriorGS: https://github.com/manycore-research/InteriorGS
- HM3D: https://arxiv.org/abs/2109.08238
- Replica: https://arxiv.org/abs/1906.05797
- 3D-FRONT: https://arxiv.org/abs/2011.09127
- MotionBenchMaker: https://www.kavrakilab.org/publications/chamzas2022-motion-bench-maker.html
