# SPLATC-GMC — 统一项目总控计划 v3.2

**状态：** 当前唯一总控文件（取代 v2 Atlas 总控，v2 全套已移至 `archive/plans_v2_atlas/`）
**权威设计文档：** `06_GMC_DESIGN_MANUAL_v1.md`（Gaussian Mobility Compiler
Theory-first Manual，2026-08-25 用户修订版）——**theory-first、三值认证逻辑
（M_safe⊂M_true⊂M_possible）、支撑函数 oracle 为唯一 GS-native 原语、
overlap 语义降级为 surrogate**。`05_GMC_RESEARCH_DESIGN.md`（08-21 版）保留
为历史/claim-ladder 语境，与 06 冲突处以 06 为准。
**建立日期：** 2026-08-24（v3.2 修订 2026-09-01）
**执行模式：** 单负责人、Gate 驱动、risk-first + **Theory Gate 纪律**
（06 §9：任何 Gate 未通过不得以规模实验替代理论缺口）

---

## 0. 一句话主线

把 scene–robot Gaussian pairs 在位姿域诱导的解析 convex forbidden primitives 编译成 orientation-fibered mobility complex（固定 θ 的 arrangement/free-dual + orientation critical-event gluing），输出采样规划器结构上无法产出的对象：**认证的 closure threshold、gate 角度区间、route-class 枚举、morphology bifurcation、compile-once/query-many 摊销**。

---

## 1. 已确立的事实边界（不得重新争论）

1. **P3.8 kill（2026-08-22，预注册判负）**：单 query thin-gate 效率上，RRT-Connect
   在 thin 0.505 达到 10/10、pair_ops 0.73×、墙钟 0.91×。任何 v3 claim 不得回到
   "equal budget 下更快找到细门路径"的领土。护城河只在**输出物**：closure
   threshold / gate interval / route classes / 多 query 摊销 / morphology 复用
   ——这些是采样法结构上答不出的问题（它只能回答"这次找没找到"）。
2. **Gate B 机制、P3.7 体积族 8–11× 分离、P4a.2/P4a.3 结论**维持有效（见
   `splatc_atlas/docs/HANDOFF.md`），相关代码与数据是 v3 的复用资产。
3. **Hard-truth 纪律**不变：opacity/render alpha 不定义物理碰撞；一切安全与拓扑
   结论最终由独立 hard-support 连续 checker 背书。

## 2. Claim ladder（v3，输出物优先）

- **Claim A（第一篇）**：固定朝向 + orientation slicing 下，pair-resolved contact
  complex 能以认证方式输出 closure threshold、gate angular interval 与 route-class
  集合，且与 converged dense oracle 一致；scalar aggregation 与均匀 θ 采样在同预算
  下丢失或误报这些结构。预算效率是**次要轴**，不是主 claim。
- **Claim B（中期）**：orientation-fibered mobility complex 支持任意 start–goal
  多 query，存在明确 break-even query count；morphology sweep 只做局部 event 更新。
- **Claim C（长期）**：hierarchy + event continuation + local kinodynamic
  certification 扩展到大场景与多运动模型（nonholonomic overlay 属于此层，
  **不进第一篇**——沿用 v2 §7.3 禁令）。

## 3. 资产继承表

| v2 资产 | v3 角色 |
|---|---|
| dense SE(2) oracle + SplatC-Gates bench + blind split | 原样复用为 ground truth 与评测（设计稿 Table 26 的 family 是其超集） |
| hard-support 连续 checker + budget protocol | 原样复用；并升级为 v3 认证策略的核心（见 §5.1） |
| `src/gaussian_geometry`（overlap、梯度、broad phase、active pairs） | M1/M2 的直接基础 |
| 自适应 chart compiler | **不删除**：作为 certified subdivision fallback（θ 区间无法证明无事件时的慢路径） |
| P3.x/P4a 报告与送审件 | provenance，只读 |
| `02_BASELINES_AND_PUBLIC_DATASETS.md`、`03_SPLATC_BENCH_SPEC.md` | 继续有效，按设计稿 §10 扩充（新增 PNO-like、FOCI-style、mesh/SDF analogue） |

## 4. 模块与阶段

模块定义 M0–M7 以设计稿 §7（Table：模块|输入|核心职责|输出|验证方式）为准。阶段与 Gate：

### Gate G-H2（先于一切方法工程）— event 稀疏性判定
H2 是整个升级的**单点赌注**：topology-changing orientation events 稀疏则 fast path
成立；爆炸则退化为 certified subdivision（≈v2 compiler）。
- 实验：`splatc_atlas/experiments/gmc/h2_event_density.py`——θ 细网格下统计三类
  签名变化：S1 nerve 边集（churn）、S2 障碍拓扑、S3 自由空间拓扑（#components,
  #holes）；扫 N 与 seed，画 events-vs-N 曲线。
- **通过标准**：S3 事件数随 N 增长显著慢于 S1 churn（目标：近线性或以下，且
  S3/S1 比值随 N 下降）；否则触发设计稿 §13.3 no-go，项目重构为
  "GS-native narrow-passage analysis"。
- 后续需在结构化场景（墙体、走廊——非均匀随机 splats）复测后才算 Gate 通过。

### Stage G1 — 固定 θ slice（对应设计稿 Phase 0–1）
闭式 forbidden primitive（含双语义：overlap threshold / conservative support）、
union、complement free cells、gate 几何、与 dense oracle 的 connectivity 一致性。
Gate：所有 bench family 上 free-component 与 gate 的 precision/recall = 1 vs oracle。

### Stage G2 — orientation event continuation（Phase 2）
gap functions、event bracketing、pseudo-arclength continuation、certified
subdivision fallback、退化处理。Gate：等 pair-query 预算下 closure-threshold error
显著低于均匀 θ 采样，且无漏事件（对照高分辨率 sweep ground truth）。

### Stage G3 — mobility complex + 多 query（Phase 3）
free-cell lineage、跨切片 gluing、node localization、graph search、path lifting +
连续认证、break-even 曲线。Gate：reachability 与 oracle 一致；多 goal 不重建
compiler；break-even query count 实测存在。此阶段吸收 v2 的 P4b 两门多目标硬合同
作为第一个可判定实验。

### Stage G4+ — hierarchy / 大场景 / nonholonomic overlay / learning proposal
按设计稿 Phase 4–5。进入条件：G3 通过且 deterministic 管线成为真实瓶颈。

## 5. 关键工程与写作决定（v3 新增）

1. **认证策略：witness + hard checker，不追求 exact conic arrangement。**
   free cell / gate 的正确性由 witness pose 经独立 hard-support checker 认证背书；
   几何层用双侧保守多边形化（内接认证 free、外接认证 obstacle）+ 区间算术。
   CGAL exact predicates 只留作退化 fallback。理由：conic arrangement 工程成本
   高一个量级且不增加 claim 强度。
2. **相关工作必须补经典 CG 谱系**：Schwartz–Sharir piano movers、Lozano-Pérez
   C-obstacles、Halperin–Sharir free-space arrangement、critical-curve exact cell
   decomposition、KDS（Basch et al. 已有）。定位话术：固定 θ 层就是 exact cell
   decomposition 在 GS pair primitives 上的实例化；novelty =
   GS-native 闭式带 identity 的 factorization + output-sensitive event
   continuation + 认证管线 + 摊销/形态复用实验学。
3. **薄片 splat 反制引理（写进论文）**：forbidden primitive 形状矩阵为
   Σ_i + RΛ_jR^T，被机器人 part 协方差下界垫肥，aspect ratio 有界 → union 复杂度
   的 fat-object 论证可用。需要一页推导 + 实验验证。
4. **per-pair threshold 的 false-safe 缺口**：多个低密度 splat 累积成实心而每个
   pair 单独不过阈值。Phase 0 校准曲线必须显式覆盖该 case；保守 support 模式兜底。
5. 设计稿正文需增补 §1 中的 P3.8 边界陈述（设计稿 v1.0 日期早于 P3.8 落锤一天）。

## 6. Stop / reframe 条件

沿用设计稿 §13.3 全部条目 + v2 §10.2 仍适用的条目。最高优先的三条：
1. S3 事件不稀疏（G-H2 失败）→ 重构为 narrow-passage analysis；
2. 固定 θ free dual 构造成本 ≈ dense grid → 同上；
3. scalar adaptive field + 标准 planner 在 equal-budget 上达到相同 thin-gate
   recall → claim 降级。

## 7. 当前执行状态

- [x] v2 计划文档归档至 `archive/plans_v2_atlas/`
- [x] 设计稿导入 `05_GMC_RESEARCH_DESIGN.md`
- [x] G-H2 pilot 首轮（uniform random 族，2026-08-24）：churn 二次（2.06）vs
      S3 近线性（1.12），S3/S1 占比 0.80→0.24 单调降——**H2 首轮支持**。
      结果 `splatc_atlas/results/gmc_h2/`，worklog `docs/worklog/gmc_H2.md`
- [x] G-H2 真实场景复测（K2 门洞/桌群窗口，2026-08-24）：closure 阶梯单调、
      S3 稀疏聚簇（≤0.10 step 占比）、gate 角度区间可测——**G-H2 实质通过**；
      附带架构结论：contact complex 建在合并 primitive 上（M1 加聚类）
- [x] 整层 K2 sweep（2026-08-25，本地 8 进程 pilot）：34 瓦片，S3 中位 36/瓦
      （步占比 0.05），事件空间局域于接触结构——**Gate G-H2：PASS**
      （canonical HPC 重跑排入论文判定链待办）
- [x] Stage G1 开工：M1 保守聚类 v3 定型（切线多边形 ⊕ 机器人椭圆精确
      Minkowski；leak=0、churn 940×降、gate 一致率 0.889 全保守向）；
      两层认证层级架构定型（详 `splatc_atlas/docs/worklog/gmc_G1.md`）
- [x] G1 细化层：两层协议一致率 1.000（L24）/0.997/0.989（L32×2），
      false_open 全零；残差解法 = patch 自适应生长（待实现）
- [x] G1 桶参数扫描：一致率对参数不敏感，默认 (0.5, 8)，整层可用 (0.8, 8)
- [x] G1 代码提升：`src/splatc/gmc/` 三模块 + 12 项测试（套件 91/91，零回归）；
      patch 自适应生长落地，K2 回归 door_B/L32 一致率 1.0/0/0
- [x] **Gate G1 PASS（2026-08-25）**：slice 层（hard-support Minkowski 语义、
      双侧认证、witness 经冻结 checker）对 oracle 验收全绿——冻结 bench
      864/864 + 新构造三族 0 fail，gate 三明治 4 门宽零违规（对解析真值差
      ≤0.003），witness 零失败；测试 94/94。过程修复 workspace body-containment
      语义错位（keyhole θ=90 暴露，已固化回归测试）
- [x] **G2 第一腿 PASS（2026-08-26）**：认证 gate 区间算法（Hausdorff 速率
      + Minkowski 缓冲扰动认证 + 双侧真值夹层）落地 `src/splatc/gmc/events.py`；
      bench 解析事件 9/9 配置全部落在认证夹层内，double_door 以 1/3 预算
      无漏事件对照密集参考；测试 97/97
- [x] **设计手册 v1 导入（2026-08-26）**：`06_GMC_DESIGN_MANUAL_v1.md` 成为
      权威设计文档；模块重编号 M0–M9、阶段 P0–P5、Theory Gate（Lemma 1–4 /
      Theorem 1–3 / Prop 1）、强制反例测试（9.3 + 附录 B）。已建资产映射：
      slice 双侧 = C⁺/C⁻（P1 门槛已过）；切线多边形 = O⁺（缺 O⁻ 与自适应
      方向加密）；events.py 扰动认证 = Theorem 1 构造性充分条件候选；
      K2 = P4 前哨证据。**keyhole θ=90 的发现已被手册收编为
      workspace-boundary event 类型 + 强制反例 T-Event-02。**
- [!] **独立 guide reference implementation（`gmc/`）纠偏重开
      （2026-09-01）**：代码与构造性 `REACHABLE` 路径可运行，但复核确认
      P3 interval/global cover、§10 query-refine、I7 full artifacts 与 `M_dyn`
      均未完成，不能称 P0–P4 关门或全部 M0–M9。HPC toy 17461675 只绑定
      08-26 历史快照。本轮已修大坐标 margin/BVH 假安全、workspace clearance、
      budget、provenance、precision、M9 corridor、endpoint/C0、affine interpolation
      与一次性 oracle 等反例。当前冻结代码本地 226/226；新 schema-v3 toy
      为 REACHABLE + 独立 verify（LB 0.02171590806152206），但 manifest 仍正确
      声明 I7 False。真实状态见 `gmc/GOAL.md` 与 `gmc/CONFORMANCE.md`。
- [~] **P5 工程组件已实现、验收待重跑（2026-09-01）**：保守 BVH、
      pair/sandwich/union 增量更新、原子分币种 ledger、equal-budget/scaling/
      cProfile runner 均存在；为保证确定性，当前相邻 θ seed 不参与几何结果，
      不再宣称 continuation 已生效。两份旧 P5 JSON 早于本轮代码且无 code
      binding，只能作历史数据。冻结 Atlas 仍只作外部 oracle/bench。
- [!] **P5 科学结果 No-Go/reframe**：G1 的 pair-resolved 与 strong exact-min
      scalar 臂在 `[64,128,256,512,1024,2048]` 同预算曲线逐点打平。
      撤下 pair-resolved query-complexity 优势；工程 PASS 不等于 full GMC/P2
      论文结果。
- [ ] **当前工程/研究队列**：先补 P3 interval certificate、query-refine 与
      I7，再对 full compiler 独有输出物（π0/route classes/morphology reuse/
      compile-once-many-query）做外部 truth，覆盖 non-gate events 和 N_R
      scaling；不再继续调 G1 两臂以寻找优势。
- [ ] P4 预注册决定：K2 无 mesh 真值——κ/ρ 标定用 Habitat-GS collision-mesh
      子集补，或 K2 降级 qualitative demo（进 P4 前定）

工作纪律沿用根 `README.md`（meta-rules、先 visualize 再 metric、HPC 使用规范、
worklog 边做边写）。
