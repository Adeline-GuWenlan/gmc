GAUSSIAN MOBILITY COMPILER

基于 3D Gaussian 场景几何的可认证 SE(2) 全局可达性编译

Theory-first Design & Implementation Manual  ·  Version 1.0

封面图：从 3DGS 原生 primitive 几何到双图夹逼、事件驱动分解和可认证查询。

> GS-native 使用 mean、covariance、primitive identity 与解析 support function。
> Certified 输出 reachable / unreachable / refine，并保存连续路径 witness。
> Output-sensitive 计算围绕近邻 pair、交事件与歧义区域展开，而非全域等精度离散。

适用范围：perfect static 3DGS · 有限个刚性 Gaussian body primitives · SE(2) · holonomic / reversible kinematic core

日期：2026-08-25

# 执行摘要

> ★  一句话科学问题 能否利用 3DGS 场景与机器人身体之间逐 primitive 的凸配置障碍及其临界组合变化，在不稠密离散完整位姿空间的情况下，认证地恢复机器人特定的全局可达结构？

本手册定义一套确定性、几何优先的 SE(2) 方法。它不训练神经网络去回归 occupancy、costmap 或 value field；它将每个场景 Gaussian 与每个机器人 body Gaussian 转化为一个可查询支持函数的凸 configuration obstacle，并维护障碍的内外包络、固定朝向的碰撞 nerve、自由空间 arrangement 以及跨朝向的安全/可能 mobility graphs。

系统对每个规划查询给出三值结果：若安全图连通，则返回带连续碰撞证书的路径；若可能图断开，则证明不可达；其余情况只细化导致歧义的 pair、support directions 或 orientation slabs。

> (3DGS, robot) → pair support oracles → O⁻ ⊂ O ⊂ O⁺ → {contact nerve, free-space arrangement} → certified orientation slabs → M_safe ⊂ M_true ⊂ M_possible → query / refine / path lift

## 本版本的边界

| 包含在 v1 核心 | 不属于 v1 核心 |
|---|---|
| SE(2) 刚体几何、固定/自适应朝向切片 | 任意 SE(3) articulated robot |
| 解析 support function、保守内外包络 | 把 Gaussian overlap 直接等同物理碰撞 |
| 碰撞 nerve + 自由空间 arrangement | 从 obstacle nerve 自动生成 free-space dual |
| 安全/可能双 mobility graph 与三值判定 | 无证书的单图近似结论 |
| holonomic 或可倒车的受限运动模型 | forward-only、drift、强动力学约束的全局完备性 |
| 确定性几何编译与 query refinement | operator learning、diffusion、RL 或 end-to-end policy |

# 文档地图

| 章节 | 主题 | 用途 |
|---|---|---|
| 1 | 科学命题与范围 | 明确研究对象、假设、主张与非目标。 |
| 2 | 形式化几何模型 | 定义 3DGS 支持、机器人模型、位姿域与碰撞语义。 |
| 3 | Pairwise configuration obstacles | 给出 Minkowski-sum 等价性和 GS-native support oracle。 |
| 4 | 认证内外包络 | 建立 safe / true / possible 的集合夹逼与三值规划逻辑。 |
| 5 | 固定朝向的拓扑对象 | 严格区分 collision nerve 与 free-space arrangement。 |
| 6 | 朝向事件与 slab 分解 | 定义事件、区间证书与自适应求精。 |
| 7 | Mobility graph 与路径证书 | 构造双图、查询、细化和 path lifting。 |
| 8 | 运动能力扩展 | 给出 reversible 模型与一般非完整 edge certifier。 |
| 9 | Theory Gate | 列出必须证明的 Lemma / Theorem、反例与 No-Go。 |
| 10 | 系统设计与模块 | 模块接口、数据结构、算法与数值约定。 |
| 11 | 实现路线与实验 | 阶段性交付、baseline、metric、ablation 与规模测试。 |
| 12 | 风险登记 | 列出数学、几何、数值、系统与科学 claim 风险。 |

> →  阅读建议 若目标是立刻实现，请先读第 2–7 章，再按第 9 章 Theory Gate 和第 10 章模块接口执行；任何 Gate 未通过，均不得以大规模实验替代理论缺口。

# 1. 科学命题与研究范围

## 1.1 科学问题

> Core question 机器人特定的全局 pose-space connectivity，能否由逐 Gaussian 场景—身体 configuration obstacles 的凸组合结构与有限临界事件恢复，而无需对整个 SE(2) 域进行稠密黑盒采样？

这里的“全局”不是指在所有位姿上执行同等精度计算，而是指最终输出对整个工作空间和全部朝向具有连通性意义。全局覆盖由所有 pairwise configuration obstacles 的显式组合提供；局部精度由 support refinement、intersection certification 和 critical-event isolation 提供。

## 1.2 核心假设

3DGS 的 primitive identity 与 anisotropic covariance 可以提供稳定、可解析的 scene–body pair geometry。

在 SE(2) v1 setting 下，固定朝向的 pairwise configuration obstacles 为紧凸集，因此可以用 support functions、convex covers 与 intersection predicates 进行认证计算。

全局可达性所需的高精度计算集中在近邻 pair、集合交和朝向临界事件附近；普通区域可以被保守 bounds 快速排除。

当内外几何近似导致的 safe graph 与 possible graph 对查询给出相同答案时，该答案可以被证明；不一致时算法应继续 refine 或 abstain。

## 1.3 主要科学主张

> Target claim Gaussian pair factorization 不比完整 scalar collision field 拥有更强的理论表达能力；它的优势在于显式暴露 convex primitives、intersection combinatorics 与 event equations，从而支持 output-sensitive、可认证的全局 connectivity compilation。

## 1.4 Assumption registry

| ID | 假设 | 必要性 | 若不满足 |
|---|---|---|---|
| A1 | 3DGS 已几何校准，可转换为有限实体 supports | 给出物理碰撞语义 | 需要表面重建或保守厚化 |
| A2 | 场景静态，workspace 紧致 | 保证有限问题与离线编译 | 需要增量维护与时间维度 |
| A3 | 机器人由有限个刚性 Gaussian supports 表示 | 支持 pairwise C-obstacle 分解 | 需 articulated / deformable 扩展 |
| A4 | 核心位姿域为 W × S¹ | 允许 orientation slab 分解 | SE(3) 需多个 rotation charts |
| A5 | v1 为 holonomic 或可倒车受限模型 | 几何 connectivity 可直接使用或易认证 | 一般动力学只保留 directed certified edges |
| A6 | 数值算法允许返回 unknown | 避免错误拓扑结论 | 若必须总给答案，将失去证书 |

# 2. 形式化几何模型

## 2.1 Scene supports

场景包含 N_E 个 3D Gaussian primitives。对平面 v1，可使用其地面投影或预先提取的 2D 切片；每个 primitive 被实体化为有限椭圆支持：

> E_i = { x ∈ R² : (x − μ_i)^T Σ_i^{-1} (x − μ_i) ≤ κ_i² }

μ_i 为中心，Σ_i ≻ 0 为各向异性 covariance，κ_i 决定物理等值面。κ_i 必须由几何标定、置信水平或保守安全 margin 冻结；opacity 不直接等于实体厚度。

## 2.2 Robot supports

> R_j = { x ∈ R² : (x − ν_j)^T Λ_j^{-1} (x − ν_j) ≤ ρ_j² },    j = 1,…,N_R

机器人形态由多个 body supports 的并集表示。ν_j 是相对机器人参考坐标系的位置，Λ_j 与 ρ_j 描述部件形状。机器人位姿为 q=(t,θ)，其中 t∈W⊂R²，θ∈S¹。

## 2.3 Collision set 与 free pose space

> B(q) = ⋃_j [ t + R_θ R_j ] C = { (t,θ) : B(t,θ) ∩ (⋃_i E_i) ≠ ∅ } F = (W × S¹) \ C

## 2.4 Notation

| 符号 | 定义 |
|---|---|
| O_ij^θ | 固定朝向 θ 时，pair (i,j) 的平移 configuration obstacle |
| C_θ | 固定 θ 的总碰撞集合：⋃_{i,j} O_ij^θ |
| F_θ | 固定 θ 的自由平移空间：W \ C_θ |
| K_θ | pair obstacle cover 的 contact nerve |
| D_θ | F_θ 的 connected-cell / arrangement 表示 |
| I_k | 经认证无遗漏事件的 orientation slab |
| M_safe | 由外障碍包络构成的保守安全 mobility graph |
| M_possible | 由内障碍包络构成的可能 mobility graph |
| witness | 证明某条图边或整条路径存在的连续几何轨迹 |

图 1. 每个 scene–body Gaussian pair 在固定朝向下诱导一个紧凸 configuration obstacle；GS covariance 通过解析 support function 被直接利用。

# 3. Pairwise Configuration Obstacles

## 3.1 精确集合定义

令中心化支持 E_i^0 与 R_j^0 表示移至原点后的实体。固定 θ，pair (i,j) 的所有碰撞平移为：

> O_ij^θ = μ_i − R_θ ν_j + ( E_i^0 ⊕ (−R_θ R_j^0) )

这一表达式来自 configuration-space obstacle 的标准 Minkowski-sum 构造：t∈O_ij^θ 当且仅当 E_i 与 t+R_θR_j 存在交点。由于两个 supports 均为紧凸集，O_ij^θ 也是紧凸集。

> !  关键限制 两个椭圆的精确 Minkowski sum 通常不是椭圆。任何把 O_ij^θ 直接写成单一椭圆的实现，都必须明确它是内近似、外近似或 Gaussian-overlap surrogate，而不是精确几何。

## 3.2 解析 support function

> s_ij^θ(u) = u^T(μ_i − R_θν_j)           + κ_i √(u^TΣ_i u)           + ρ_j √(u^TR_θΛ_jR_θ^T u),     \|\|u\|\|=1

support function 具有三个用途：构造内外凸包、进行 pair separation / intersection bounds、以及在 orientation interval 上建立保守变化界。它是本方法最基础的 GS-native oracle。

## 3.3 Pair pruning

全量 pair 数为 N_E N_R。实际系统必须在 support oracle 之前使用层级 bounds：

对 scene Gaussians 建 BVH / covariance-aware cluster tree；机器人 primitives 建小型层级。

对 orientation interval I 计算 cluster-pair 的最小可能距离与最大支持半径；确定远离者整组剪枝。

只保留可能进入 workspace 或可能参与交事件的 pair；所有剪枝必须是 conservative。

复杂度报告必须同时给出总 splat 数、保留 pair 数、intersection 候选数与 event 数。

## 3.4 数值接口

建议接口（概念签名）

> class PairObstacleOracle:     pair_id: tuple[int, int]     support(theta: float, u: Vec2) -> float     support_point(theta: float, u: Vec2) -> Vec2     interval_support(theta_interval: Interval, u: Vec2) -> Interval     coarse_aabb(theta_interval: Interval) -> AABB

# 4. 认证内外包络与三值逻辑

## 4.1 Pair-level convex sandwich

选择方向集 U_m={u_k}。利用 support points 的凸包构造内包络，利用 supporting halfspaces 构造外包络：

> O_ij^{−,θ} = conv{ x_ij^θ(u_k) } O_ij^{+,θ} = ⋂_k { x : u_k^T x ≤ s_ij^θ(u_k) } O_ij^{−,θ} ⊂ O_ij^θ ⊂ O_ij^{+,θ}

方向集自适应加密，直至 pair-level support gap 或 Hausdorff upper bound 不超过 ε_pair。近 passage 区域可使用更密方向，其余区域保持粗表示。

## 4.2 Scene-level sandwich

> C_θ^- = ⋃ O_ij^{−,θ}  ⊂  C_θ  ⊂  C_θ^+ = ⋃ O_ij^{+,θ} F_θ^safe = W \ C_θ^+  ⊂  F_θ  ⊂  W \ C_θ^- = F_θ^possible

## 4.3 查询判定

| 判定条件 | 结论 | 证书 |
|---|---|---|
| s,g 在 M_safe 中连通 | certified reachable | 图路径 + 每条边的连续 witness + 全轨迹碰撞验证 |
| s,g 在 M_possible 中不连通 | certified unreachable | possible graph 的 component / cut certificate |
| 其余 | unknown / refine | 返回歧义 pair、交谓词或 orientation slabs |

图 2. 通过 pairwise 凸内外包络形成自由空间的安全下界与可能上界；算法只在二者对查询不一致时继续细化。

## 4.4 Margin guarantee

> Proposition 若所有 pair 外包络相对真实障碍的 Hausdorff 误差不超过 ε，则任何对真实碰撞集 clearance 大于 ε 的轨迹仍包含在 F_safe 中；safe graph 返回的路径不存在 false-safe。

# 5. 固定朝向的拓扑对象

## 5.1 Contact nerve：障碍侧

> K_θ = Nerve({ O_ij^θ })

一个 simplex σ 存在，当且仅当对应 configuration obstacles 的公共交非空。因 cover elements 为凸集，所有非空有限交均可缩，故 nerve 与碰撞并集 C_θ 同伦等价。

> !  实现约束 Helly 性质可以降低公共交判定的维度，但不能把完整 nerve 简单截断为 pair graph 或 2-skeleton。若多个集合具有共同交，必须保存其 maximal simplex，否则会人为制造错误同调。

## 5.2 Free-space arrangement：规划侧

> D_θ = ConnectedCells( W \ C_θ )

Nerve 不能自动给出 complement 的路径图。固定朝向的规划对象必须由 obstacle boundaries 与 workspace boundary 的 arrangement / planar subdivision 获得。对 polygonal inner/outer approximations，可直接使用计算几何库求 union 与 complement components。

## 5.3 每个 free component 必须保存的对象

component ID、orientation slice / slab ID；

多边形边界或可查询 membership oracle；

代表点与对 workspace 边界的关系；

内部路径规划 backend（triangulation、visibility graph 或 exact cell adjacency）；

与相邻 orientation slab 的 continuation witness 候选；

clearance lower bound 与导致不确定性的 pair IDs。

图 3. Contact nerve 认证障碍并集的组合拓扑；自由空间 arrangement 才负责 connected components、passages 与路径 witness。

## 5.4 v1 输出语义

> Reachability first v1 的正式输出是 path-component connectivity 与至少一条 certified path；“枚举所有 homotopy route classes”属于后续扩展，不能由 component graph 自动得到。

# 6. 朝向事件与 Certified Slabs

## 6.1 为什么存在有限分解

ellipsoidal supports、旋转变量 (cosθ,sinθ) 与碰撞存在量词共同定义半代数集合。对投影 π:F→S¹，半代数局部平凡性保证存在有限 orientation partition，使每个 regular interval 上的 free space 与某一代表 fiber 构成乘积同胚。

> 理论含义 “只需处理有限 critical orientations”具有存在性依据；但该定理不自动提供低成本算法。实现仍必须给出不会漏事件的 interval certificate 或在预算不足时返回 unknown。

## 6.2 事件类型

| 类别 | 几何条件 | 可能影响 |
|---|---|---|
| Pair tangency | 两个 pair obstacles 首次接触或分离 | nerve edge 出生/死亡、free passage 关闭/打开 |
| Higher-order intersection | 三重及以上公共交出现/消失 | maximal simplex 与 union topology 改变 |
| Workspace-boundary event | configuration obstacle 与 W 边界相切 | free component 出生/消失、封闭 |
| Arrangement vertex event | 边界交点交换顺序或退化 | cell adjacency 改变 |
| Component split / merge | free fiber 的 component lineage 改变 | 全局 mobility graph 分支改变 |
| Degenerate simultaneous event | 多个谓词同角度为零 | 需要 symbolic perturbation 或区间隔离 |

## 6.3 Slab 认证策略

为 orientation interval I 计算所有保留 pair 的 interval support / AABB bounds。

对 pair intersection、higher-order intersection、boundary event 与 arrangement ordering 建立符号稳定性测试。

若所有相关谓词在 I 内远离 0，则认证该 interval 为 regular slab，只保存代表 slice。

若不能认证，二分 I；接近根时使用 root isolation 或 pseudo-arclength continuation 精确定位 θ*。

若达到最小角分辨率仍无法证明，则标记 uncertain slab，不得强行连接 mobility nodes。

图 4. 朝向域被分解为 regular slabs 与 critical events；全局覆盖保留，但高精度只用于组合结构可能改变的位置。

# 7. Mobility Graph 编译、查询与 Path Lifting

## 7.1 节点与边

每个 mobility node 表示一个 orientation slab 内连续存在的 free-space component tube。边分为两类：

Intra-slab edge：在同一 component tube 内改变平移与朝向；

Event edge：在 critical slab 附近连接 split、merge、birth 或 death 前后的 components。

一条边只有在保存了连续 pose path witness，并且 witness 通过所有 pair collision bounds 时，才可进入 M_safe。M_possible 可以包含尚未排除的候选边，但必须保留其不确定性来源。

## 7.2 双图夹逼

> M_safe ⊂ M_true ⊂ M_possible

## 7.3 Query-and-refine

Algorithm A：三值规划查询

> function QUERY(start s, goal g):     locate nodes ns_safe, ng_safe, ns_pos, ng_pos     if connected(M_safe, ns_safe, ng_safe):         P <- shortest certified graph path         tau <- lift stored witnesses(P)         return REACHABLE if continuous_verify(tau)     if not connected(M_possible, ns_pos, ng_pos):         return UNREACHABLE     ambiguity <- minimum cut / path disagreement region     refine(ambiguity)     repeat

## 7.4 Path lifting

图路径必须被转换为连续 q(t)=(t(t),θ(t))。每条 graph edge 保存局部 witness；拼接后执行以下验证：

位置与朝向连续，θ 处理周期边界；

整条轨迹位于 workspace；

对所有 pair 或经 conservative pruning 后的候选 pair，连续 collision lower bound 非负；

若使用多边形外包络，最终可调用精确 support-based pair checker 消除近似假阳性；

任何验证失败的 edge 从 M_safe 删除并触发局部 refine。

图 5. M_safe 与 M_possible 对同一查询形成可证明的下界和上界；不一致时只细化歧义区域。

# 8. 机器人运动能力与非完整约束

## 8.1 v1 几何核心

核心 mobility graph 首先描述有限尺寸刚体在 pose space 中的连续无碰撞连通性。对 holonomic SE(2)，该连通性可直接用于规划。

## 8.2 可倒车 differential drive

对于无 drift、控制 v 与 ω 可取正负的 unicycle / differential-drive 模型，其控制向量场及 Lie bracket 在 SE(2) 上张成完整切空间。若 free component 为开且连通，在标准 bracket-generating 条件下可得到 kinematic accessibility；因此此受限模型可以作为 v1 的可选验证对象。

> !  边界 上述结论不覆盖 forward-only Dubins car、强最小转弯半径、速度/加速度约束、drift 系统或动态稳定性。

## 8.3 一般机器人：Directed edge certification

> e ∈ M_dyn  ⇔  ∃ u(t) : q̇=f_r(q,u),  q(t) stays inside certified corridor of e

| 机器人/约束 | 建议 backend | 证书内容 |
|---|---|---|
| Differential drive / Reeds–Shepp | motion primitives 或 analytic steering | 端点、曲率与 corridor containment |
| Ackermann / forward-only | state lattice 或 short-horizon collocation | 有向可达、转弯半径、clearance |
| Drone / second-order | local trajectory optimization 或 reachable tube | 动力学、速度/加速度、碰撞 |
| 复杂 articulated robot | sampling / optimization backend | 仅对少量候选 mobility edges 认证 |

几何图与动力学图必须分开：M_geom 提供候选全局连通结构，M_dyn 通过局部动态认证删除不可执行的边。

# 9. Theory Gate：实现前的证明闭环

图 6. Theory Gate 依赖图：局部集合等价与夹逼是基础，事件完备性与 path lifting 是当前最关键的开放环节。

## 9.1 必须正式写出的结果

| 状态 | 对象 | 结论 | 实现门槛 |
|---|---|---|---|
| Formal | Lemma 1：pair obstacle equivalence | t∈O_ij^θ 当且仅当对应实体 supports 碰撞。 | 解析推导 + 随机数值对照 |
| Formal | Lemma 2：support sandwich | O^-⊂O⊂O^+，并推导 free-space 夹逼。 | 凸几何构造 |
| Formal | Lemma 3：nerve correctness | K_θ 与 C_θ 同伦等价；高阶 simplex 不可遗漏。 | closed-convex nerve theorem |
| Conditional | Lemma 4：arrangement components | 固定 θ 的 complement decomposition 正确给出 π₀(F_θ)。 | 可靠几何库 + exact predicates |
| Open | Theorem 1：event-free slab | 若 interval certificate 通过，则 component lineage 不变。 | 完整 critical predicates |
| Conditional | Theorem 2：mobility graph | 在精确 slab partition 与 witness edges 下，图连通等价于 F 中 path connectivity。 | cover/gluing proof |
| Formal | Proposition 1：ε-margin guarantee | 外包络不产生 false-safe，并保留 clearance>ε 的路径。 | Hausdorff bound |
| Deferred | Theorem 3：general dynamics | 一般非完整系统的全局完备性。 | v1 不承诺 |

## 9.2 Thin-gate complexity hypothesis

对未知宽度 δ 的合法 orientation interval，纯均匀 point sampling 为保证不遗漏需要 Ω(1/δ) 级样本；若 pair event 可写成带 bracket 的标量根 g(θ)=0，则二分定位到精度 ε 只需 O(log(Δθ/ε)) 次 oracle 调用。

> 可发表的第一阶段 claim 在同等几何 oracle 预算下，event-driven pair continuation 比全域黑盒 orientation sampling 更接近物理闭合阈值地追踪趋零姿态通道，同时保持保守安全证书。

## 9.3 强制反例测试

四个凸集具有共同交：验证 maximal simplex 不被 2-skeleton 截断。

pair graph 不变但 workspace-boundary 事件改变 free connectivity。

两个局部完全相同、远处 branch 不同的场景：验证局部 continuation 不能替代 global cover。

近退化 triple event：验证事件隔离与 symbolic perturbation。

外包络关闭真实窄门：验证 unknown / refine 而不是错误 unreachable。

内包络创造可能假通道：验证其只进入 M_possible。

# 10. 核心系统设计

图 7. 模块化数据流。M1–M8 构成确定性几何核心；M9 为独立动力学扩展。

## 10.1 模块接口

| 模块 | 输入 | 输出 | 失败模式 |
|---|---|---|---|
| M0 Input calibrator | 3DGS、robot model、workspace | 实体 supports、尺度与置信参数 | 几何未校准、异常 splats |
| M1 Support oracle | pair、θ、u | support value / point / interval | 数值病态、Σ 非正定 |
| M2 Pair hierarchy | scene tree、robot tree、θ interval | 保守候选 pairs | 误剪枝将破坏完备性 |
| M3 Convex approximator | support oracle、ε_pair | O^-、O^+、误差证书 | 方向不足、曲率尖锐 |
| M4 Slice topology | pair approximations、W | K_θ、D_θ、components | 高阶交遗漏、几何退化 |
| M5 Event manager | orientation interval、slice states | regular slabs / critical brackets | 漏事件、无限细分 |
| M6 Mobility compiler | slab components、lineage | M_safe、M_possible | 错误 glue、周期边界 |
| M7 Query engine | s,g、双图 | answer / ambiguity set | 定位失败、图不一致 |
| M8 Path verifier | graph path、witnesses | 连续 certified trajectory | 拼接不连续、近碰撞 |
| M9 Dynamics certifier | candidate edge、f_r,U_r | directed valid edge / reject | 局部 planner 不完备 |

## 10.2 关键数据结构

核心数据结构（概念）

> GaussianSupport:     mean: Vec2     covariance: Mat2SPD     level: float     primitive_id: int  ConvexApprox:     inner_polygon: Polygon     outer_polygon: Polygon     hausdorff_upper: float     source_pair: PairID     theta_domain: Interval  SlabNode:     slab_id: int     component_id: int     safe_geometry: Region     possible_geometry: Region     witnesses: list[PathWitness]     uncertainty_sources: set[PredicateID]  MobilityEdge:     src, dst: NodeID     status: SAFE \| POSSIBLE     witness: PoseCurve \| None     clearance_lb: float

## 10.3 可靠性不变量

| Invariant | 要求 |
|---|---|
| I1 Pair nesting | 任意 pair / θ：O^-⊂O⊂O^+。 |
| I2 Graph nesting | M_safe 的任意节点/边均为真实可行；真实结构不超出 M_possible。 |
| I3 Witness ownership | M_safe 的每条边必须绑定可复验的连续 witness。 |
| I4 No silent fallback | 证书失败只能进入 POSSIBLE/UNKNOWN，不能默认为 SAFE。 |
| I5 Reproducibility | 所有容差、方向集、剪枝 bounds 与 event brackets 写入日志。 |
| I6 Periodicity | θ=0 与 θ=2π 的 cells 和 lineage 必须一致 glue。 |

# 11. 算法设计

## 11.1 固定朝向编译

Algorithm B：固定 θ 的认证 slice

> function BUILD_SLICE(theta, eps_pair):     pairs <- BVH_CANDIDATES(theta)     for each pair a in pairs:         oracle[a] <- BUILD_SUPPORT_ORACLE(a)         (Ominus[a], Oplus[a], cert[a]) <- APPROXIMATE_CONVEX(oracle[a], eps_pair)     Kminus, Kplus <- BUILD_NERVES(Ominus, Oplus, exact_predicates=True)     Dsafe <- COMPLEMENT_COMPONENTS(W, union(Oplus))     Dpossible <- COMPLEMENT_COMPONENTS(W, union(Ominus))     return CertifiedSlice(theta, Kminus, Kplus, Dsafe, Dpossible, cert)

## 11.2 Orientation interval refinement

Algorithm C：事件驱动 slab 分解

> function REFINE_INTERVAL(I, slice_left, slice_right):     predicates <- COLLECT_RELEVANT_EVENTS(I)     if CERTIFY_SAME_SIGN(predicates, I) and COMPONENT_MATCH(slice_left, slice_right):         return RegularSlab(I, lineage_certificate)     if width(I) <= theta_min:         return UncertainSlab(I, predicates)     I1, I2 <- bisect(I)     mid <- BUILD_SLICE(midpoint(I), local_eps(I))     return REFINE_INTERVAL(I1, slice_left, mid) +            REFINE_INTERVAL(I2, mid, slice_right)

## 11.3 Mobility compilation

Algorithm D：双 mobility graph

> function COMPILE_MOBILITY(slabs):     initialize M_safe, M_possible     for each slab I:         add safe/possible component nodes     for adjacent slabs (Ia, Ib):         candidate_links <- MATCH_COMPONENTS(Ia, Ib)         for link in candidate_links:             if CERTIFY_CONTINUOUS_WITNESS(link):                 add link to M_safe and M_possible             elif NOT_EXCLUDED(link):                 add link to M_possible     glue theta=0 and theta=2pi     return M_safe, M_possible

## 11.4 Path extraction

M_safe 上的图搜索可以使用 Dijkstra/A*，edge weight 由 witness 长度、最小 clearance 或后续 dynamics cost 决定。图级最短并不自动等于连续空间全局最短；v1 只承诺可行性与可解释的近似 cost。

# 12. 实现方案

## 12.1 推荐技术栈

| 层 | 建议工具 | 原因 |
|---|---|---|
| 原型语言 | Python + NumPy/SciPy | 快速验证 support、interval 与测试生成 |
| 二维几何 | Shapely/GEOS 或 CGAL Python binding | polygon union、arrangement、exact-ish predicates |
| 高可靠后端 | C++17 + CGAL | exact predicates、arrangements、AABB tree |
| 区间算术 | MPFI/Boost.Interval/interval arithmetic library | orientation predicates 与 support bounds |
| 图算法 | NetworkX（原型）/ Boost Graph（系统） | component、cut、shortest path |
| 可视化 | Matplotlib + custom viewer | slice、events、双图与 witness debug |
| 3DGS I/O | PLY/torch tensors | 读取 mean、scale、rotation、opacity |
| 加速 | CUDA/Triton（后续） | 大规模 pair bound 与 support batch |

## 12.2 数值约定

| 参数 | 意义 | 默认策略 |
|---|---|---|
| κ_i, ρ_j | 实体等值面尺度 | 由 ground-truth geometry 标定；v1 不从 opacity 猜测 |
| ε_pair | pair convex approximation 误差 | 先 1e-3 × scene scale，再做收敛实验 |
| ε_event | critical angle 定位误差 | 与 passage closure metric 联动 |
| θ_min | 最小 slab 宽度 | 达到后仍不确定则 abstain |
| ε_clear | 最终路径安全 margin | 必须大于累计几何与数值误差 |
| predicate tol | 交/切事件判定 | exact predicates 优先，浮点容差必须分层 |

## 12.3 开发路线

图 8. 阶段性交付与 Gate。任何阶段未通过，不得用后续规模实验掩盖。

## 12.4 Phase deliverables

| 阶段 | 必须交付 | 通过条件 |
|---|---|---|
| P0 Oracle | pair collision、support value/point、inner/outer nesting 单测 | 10^5 随机查询零 nesting violation；外包络 false-negative=0 |
| P1 Slice | 固定 θ 的 union、nerve、free components 与路径 | 与高分辨率/精确几何 ground truth 的 π₀ 完全一致 |
| P2 Slabs | 自适应 orientation partition 与 event brackets | 已知解析场景 event recall=100%；无法认证时正确 abstain |
| P3 Mobility | M_safe/M_possible、query、lift、verify | safe path 100% 连续无碰撞；possible 不漏 ground-truth path |
| P4 3DGS | 真实 splat ingestion、pair hierarchy | 几何校准误差可量化；规模提升不改变证书语义 |
| P5 Dynamics | directed edge certification | 指定机器人模型下所有保留 edge 可执行 |

# 13. 实验与验证设计

图 9. 实验顺序：先检验证书语义，再测试 thin-gate 效率，最后测试大规模可扩展性。

## 13.1 Synthetic theorem tests

单门连续闭合：解析 ground-truth orientation interval 与 closure angle。

双门与两条 homotopy 路线：检验 component connectivity，但 v1 不要求完整 route-class enumeration。

Keyhole：位置宽、姿态窄，验证 orientation slabs。

三障碍/四障碍公共交：检验 higher-order nerve 与 event detection。

Workspace boundary pinch：检验 pair graph 不变但 complement connectivity 改变的情况。

退化 simultaneous events：检验 interval isolation、deterministic perturbation 与 abstention。

## 13.2 Baselines

| Baseline | 公平性要求 | 比较对象 |
|---|---|---|
| Dense SE(2) occupancy + A* | 使用同一实体 supports 与 collision checker | 内存、query 数、thin-gate recall、path validity |
| Adaptive grid / interval map | 允许自适应与全局 refinement | 与我们的 output-sensitive refinement 直接比较 |
| PRM / RRT* | 相同碰撞查询预算、多随机种子 | success、漏门率、运行时间 |
| Scalar clearance oracle | 使用 min pair clearance，允许最强 adaptive sampling | 验证优势来自显式组合访问而非弱 baseline |
| Trajectory optimizer | 给定 start-goal 与相同 initial budget | query-specific 解法与可复用 global compilation 的差异 |
| Ablated pair graph | 去掉 higher-order simplex / arrangement | 证明这些模块的必要性 |

## 13.3 Metrics

| 维度 | 指标 |
|---|---|
| Correctness | false-safe rate、false-unreachable rate、safe/possible sandwich violation |
| Topology | π₀ accuracy、event recall/precision、component lineage accuracy |
| Thin gate | 合法 orientation interval IoU、closure-angle error、minimum recovered width |
| Planning | success、path length、clearance、continuous verification pass rate |
| Efficiency | support calls、pair candidates、intersection tests、slabs、runtime、peak memory |
| Scalability | 随 N_E、N_R、near-pair 数和 event 数的增长曲线 |
| Abstention | unknown rate、refinement depth、未决区域体积 |

## 13.4 No-Go criteria

> No-Go 1 若 strong adaptive scalar baseline 在相同 oracle 预算下获得相同 thin-gate recall、closure error 与证书质量，则“pair combinatorics 带来 query-complexity 优势”的核心 claim 不成立。

> No-Go 2 若 arrangement / event 数在中型场景中接近 dense state-space 大小，且 hierarchy 无法显著削减，则 global compiler 的工程价值不足。

> No-Go 3 若 3DGS→实体 supports 的保守校准关闭大量真实通道或产生不可接受假空间，则必须先解决几何表示，而不是继续规划系统。

# 14. 风险登记与缓解

| ID | 风险 | 后果 | 缓解 | 等级 |
|---|---|---|---|---|
| R1 | Gaussian 无限支撑 | 碰撞语义不明确 | 固定 iso-support；区分 physical support 与 overlap surrogate | High |
| R2 | 3DGS 几何非实体表面 | 假障碍/漏障碍 | 仅在 perfect geometry setting 立论；真实场景需校准与保守厚化 | High |
| R3 | Minkowski sum 被错误椭圆化 | false-safe 或过度保守 | 使用 exact support oracle + certified inner/outer convex approximation | High |
| R4 | 只构建 pair graph | 高阶 topology 错误 | 记录 maximal simplices；使用 Helly tests 仅作判定加速 | High |
| R5 | nerve 被误当 free graph | 无法 lift path | 独立构建 complement arrangement 与 path witnesses | High |
| R6 | orientation event 漏检 | 错误 glue 全局组件 | interval certificates；失败则 uncertain slab | High |
| R7 | 组合复杂度爆炸 | 运行时间/内存失控 | BVH、interval pair pruning、局部增量 update、query-driven refinement | Medium |
| R8 | 浮点退化 | split/merge 不稳定 | exact predicates、symbolic perturbation、尺度归一化 | High |
| R9 | θ 周期边界错误 | 0/2π 产生假断裂 | 显式 periodic IDs 与 wrap-around tests | Medium |
| R10 | path witness 拼接失败 | 图连通但无连续路径 | 每条 safe edge 必须持有可复验 witness；拼接后全轨迹验证 | High |
| R11 | 非完整约束不匹配 | 几何路径不可执行 | 分离 M_geom / M_dyn；只保留 directed certified edges | High |
| R12 | claim 过度扩张 | 理论与实验不匹配 | v1 仅 claim connectivity + one path；route classes / SE(3) 延后 | High |

## 14.1 科学表述禁区

不得声称 scalar field 在信息论上无法表示 topology；差异是访问结构、query complexity 与证书。

不得声称 contact nerve 本身就是 planner；free-space arrangement 与 path lifting 是不可省略的。

不得声称 v1 恢复所有 homotopy classes；它首先恢复 path components 与一条 certified path。

不得将 Gaussian overlap threshold 无条件解释为物理碰撞。

不得用均匀 grid 作为唯一 baseline；必须包含强 adaptive scalar 方法。

不得在 event certificate 失败时输出确定连通结论。

# 15. 项目决策与近期执行

## 15.1 Go / Conditional Go / Deferred

| 状态 | 对象 | 结论 | 实现门槛 |
|---|---|---|---|
| GO | Pair support oracle | 数学定义清楚、实现直接、可独立单测。 | 立即开始 |
| GO | 固定朝向的 certified slice | 可用凸包络、union 与 arrangement 建闭环。 | P0 后开始 |
| Conditional | Event-complete orientation slabs | 存在理论基础，但算法证书仍是关键。 | 先做解析 toy scenes |
| Conditional | 全局双 mobility graph | 依赖 slab lineage 与 witness correctness。 | P2 通过后 |
| GO | Thin-gate mechanism paper | 可在受限 SE(2) family 中形成硬 claim。 | 优先实验方向 |
| Deferred | 一般 SE(3) / articulated | rotation space 与组合复杂度显著增加。 | 非首篇 |
| Deferred | Learning / operator module | 当前不是必要组件，可能削弱证书链。 | 几何核心后再评估 |

## 15.2 接下来两周的任务

冻结 Gaussian support calibration 与单位尺度；建立 2D synthetic scene / robot generator。

实现 exact pair support value 与 support point；对 10^5 随机配置做碰撞等价测试。

实现 O^- / O^+ 自适应方向加密，并验证 nesting 与误差单调收敛。

接入二维 polygon union / complement components；完成固定 θ 的 D_safe 与 D_possible。

实现 contact nerve 的 maximal simplex 构造，并加入四集合共同交反例。

构造 single-door、keyhole、triple-event 三个解析 benchmark。

实现最小 orientation bisection 与 event bracket；先不追求一般 completeness。

建立自动报告：pair 数、support calls、components、events、sandwich disagreement、path validity。

## 15.3 成功标准

> ✓  Milestone v1 在 20–100 个 scene supports、1–5 个 robot supports 的 SE(2) 合成场景中，系统能够：固定朝向正确恢复 free components；自适应定位所有预定义 critical events；对查询给出无 false-safe 的 reachable/unreachable/refine 结果；并将 safe graph path lift 为连续无碰撞轨迹。

## 15.4 论文级最小主张

> Minimal publishable claim A Gaussian-native, certificate-carrying SE(2) mobility compiler can exploit pairwise convex configuration obstacles and event-driven orientation refinement to recover near-closure passage connectivity with fewer black-box pose queries than dense or adaptive scalar discretization, while never returning a false-safe path.

# 附录 A：证明草案索引

## A.1 Lemma 1 — Pair obstacle equivalence

证明由碰撞点存在性直接重排得到 Minkowski-sum 形式。需要明确所有 supports 为闭、非空、紧凸集。

## A.2 Lemma 2 — Convex sandwich

内凸包由真实 boundary support points 生成，故包含于真实凸体；外多面体由真实 supporting halfspaces 交生成，故包含真实凸体。并集与补集运算给出 scene-level nesting。

## A.3 Lemma 3 — Nerve correctness

有限凸 cover 的任意非空交仍凸且可缩。应用 nerve theorem 得到 |K_θ|≃C_θ。若使用闭集版本，需在正式论文中引用相应定理或使用任意小 open thickening 并证明 homotopy 稳定。

## A.4 Theorem 1 — Event-free slab

目标：定义一组完整 critical predicates；若它们在 compact interval I 上均不取 0，并且边界保持横截，则 arrangement combinatorics 与 free-component lineage 在 I 上不变。可从半代数 triviality 给出存在性，再为 v1 的有限谓词族给出构造性充分条件。

## A.5 Theorem 2 — Mobility graph correctness

假设 slabs 覆盖 F，每个 node 对应 path-connected component tube，相邻 node 的 edge 当且仅当集合非空交且保存 path witness。则图路径可拼接为 F 中路径；反向由任意连续路径的紧致像选取有限 cover 子序列得到图路径。

## A.6 Proposition — ε-margin guarantee

若 C⊂C^+⊂C⊕B_ε，则 F_safe=W\C^+⊂F。任意距 C 大于 ε 的轨迹不进入 C⊕B_ε，因此保留在 F_safe。

# 附录 B：最小测试清单

| Test ID | 构型 | 通过条件 |
|---|---|---|
| T-Pair-01 | 随机椭圆 pair + 固定 θ | support/Minkowski obstacle 与 direct collision 一致 |
| T-Pair-02 | 极端 aspect ratio | 数值稳定，无 nesting violation |
| T-Nerve-01 | 两两交但无三重交 | nerve 只有边，无 triangle |
| T-Nerve-02 | 四集合共同交 | 保存 3-simplex，不产生假 S² |
| T-Free-01 | 障碍不接触 workspace | free component 数正确 |
| T-Free-02 | 障碍连接左右边界 | free component split 正确 |
| T-Event-01 | 单门闭合 | θ* 定位误差≤ε_event |
| T-Event-02 | pair graph 不变的 boundary event | 仍能检测 free connectivity 变化 |
| T-Event-03 | triple simultaneous event | 正确 isolate 或 abstain |
| T-Graph-01 | safe path | lift 后全轨迹连续无碰撞 |
| T-Graph-02 | possible-only edge | 不得进入 safe graph |
| T-Periodic-01 | θ=0/2π | node lineage 正确闭合 |
| T-Baseline-01 | thin gate δ sweep | 记录样本量/δ 与 event method scaling |

# 参考文献

[R1] T. Lozano-Pérez. Spatial Planning: A Configuration Space Approach. IEEE Transactions on Computers, C-32(2):108–120, 1983.

[R2] J.-C. Latombe. Robot Motion Planning. Kluwer Academic Publishers, 1991.

[R3] S. M. LaValle. Planning Algorithms. Cambridge University Press, 2006.

[R4] R. M. Hardt. Semi-Algebraic Local-Triviality in Semi-Algebraic Mappings. American Journal of Mathematics, 102(2):291–302, 1980.

[R5] S. Basu, R. Pollack, M.-F. Roy. Algorithms in Real Algebraic Geometry. Springer, 2nd ed., 2006.

[R6] H. Edelsbrunner and J. Harer. Computational Topology: An Introduction. AMS, 2010.

[R7] R. Schneider. Convex Bodies: The Brunn–Minkowski Theory. Cambridge University Press, expanded ed., 2014.

[R8] K. Crane, C. Weischedel, M. Wardetzky. Geodesics in Heat. ACM Transactions on Graphics, 2013.

[R9] H. Zhou, Z. Lähner. Laplace-Beltrami Operator for Gaussian Splatting. arXiv:2502.17531v2, 2026.

[R10] S. Matada et al. Generalizable Motion Planning via Operator Learning. ICLR, 2025.

[R11] S. Yoo et al. Neural Green’s Functions. NeurIPS, 2025.

[R12] S. Wang, T. Guo, M. Guo. Customize Harmonic Potential Fields via Hybrid Optimization over Homotopic Paths. arXiv:2507.09858, 2025.

[R13] J. Zuo et al. From Representation to Action: A Unified Laplacian Framework for Spatial Representation and Path Planning. ICML, 2026.

[R14] D. Shehmar et al. Laplacian Representations for Decision-Time Planning. ICML, 2026.

[R15] J. A. Reeds and L. A. Shepp. Optimal Paths for a Car that Goes Both Forwards and Backwards. Pacific Journal of Mathematics, 145(2), 1990.

> 版本声明 本手册以“可认证的 Gaussian-native SE(2) mobility compilation”为唯一核心。任何 learning、value field、spectral basis、diffusion 或 route-generation 模块，只有在不破坏证书链且能通过独立 ablation 证明必要性时，才允许作为后续扩展。
