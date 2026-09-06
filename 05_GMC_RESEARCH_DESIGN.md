SPLATC

Gaussian Contact Complexes for Robot-Specific Global Mobility

从逐 Gaussian 接触分解到机器人特定的全局可移动结构

概念总览：3DGS 与机器人 → pair-resolved contacts → 接触复形 → mobility complex → 全局查询  视觉概念草图由 Imagine2 生成，并在本文中重新标注。

> 核心科学问题 机器人特定的全局可移动结构，能否由逐 Gaussian 场景—身体禁行区域的交叠组合及其临界演化恢复，而不需要稠密离散完整状态空间？

Research Design Specification  |  Version 1.0  |  2026-08-21

状态：核心假设与方法设计已形成；完整性、复杂度与大场景有效性仍需验证。

# 0. Executive Summary

> 一句话 idea 3DGS 不仅是一张可渲染场，也是一组具有显式 identity、各向异性 covariance 和闭式 pair interaction 的 primitives；SPLATC 利用这些 primitives 在机器人位姿域中诱导的禁行集合及其交叠组合，直接编译一个目标无关的全局 mobility complex，再对任意 start–goal query 做路径查询与动力学认证。

本项目不以“学习一张 costmap”“预测某个目标的 value field”或“优化一条给定初值的轨迹”为核心。它研究的是一个更前置的表示与计算问题：3DGS 的 primitive-wise contact factorization 是否能够提供标量碰撞聚合所丢失的全局连通信息，并能否以 output-sensitive 的方式恢复薄姿态通道、分支关系和形态变化导致的通道闭合。

推荐的方法对象是 Gaussian Mobility Complex（GMC）。在固定朝向下，每个 scene Gaussian 与 robot Gaussian pair 对应一个解析或保守的 convex forbidden primitive；这些 primitives 的交叠组合形成 Gaussian Contact Complex，而自由空间 complement 的 arrangement/dual 描述可通行区域与 gate。随着朝向变化，只有少数临界接触事件会改变组合结构；通过 event continuation 将各个朝向切片 glue 起来，得到覆盖整个 SE(2) 位姿域的稀疏 mobility complex。

Learning 不应承担最终拓扑判断。最合理的角色是候选 pair/event proposal、优先级排序、warm start 或跨场景摊销；最终 connectivity 和 safety 由解析几何、区间界、连续追踪与局部 kinodynamic certification 给出。

| 项目维度 | 当前结论 | 必须证明的证据 |
|---|---|---|
| GS-native | 通过：使用 pair identity、anisotropic covariance、闭式 overlap/禁行 primitive，而非只用中心或转 mesh。 | 与 centers-only、scalar aggregation、mesh/SDF analogue 做机制消融。 |
| 与 costmap 区分 | 通过：不是 dense field regression；全局结构由 forbidden-set combinatorics 与 critical events 推导。 | 显示计算随 active pairs / intersections / events 增长，而非随全位姿网格体积增长。 |
| 方法美感 | 有条件通过：固定朝向的 convex cover + nerve/free dual + orientation event gluing 形成统一链条。 | 严格区分 obstacle nerve 与 free-space complement；给出事件定义与更新规则。 |
| Feasibility | SE(2)、rigid Gaussian robot、perfect static GS 可行；任意 SE(3)/articulated robot 是长期目标。 | 先完成固定朝向、orientation slicing、gate closure 和多 query 复用。 |
| Technical soundness | 局部与固定朝向部分有清晰数学基础；完整 global recovery 与 nonholonomic completeness 尚未成立。 | target theorem、resolution-complete fallback、path lifting 与 local dynamics certificates。 |

## 0.1 最小可发表 claim

> MVP Claim 在 perfect 3DGS、rigid Gaussian robot 与 SE(2) setting 下，pair-resolved Gaussian forbidden-set complex 与 critical-event continuation 能在相同几何查询预算下，比 scalar collision aggregation 或均匀位姿采样更准确地保留趋零的 orientation passage，并更接近真实物理闭合阈值。

## 0.2 最终 vision

从 3DGS scene、机器人形态和运动能力直接编译一个目标无关、可复用、可查询、可认证的 global mobility complex；其规模由 planning-critical contacts 与 topology events 控制，而不是由完整状态空间离散分辨率控制。

# 目录

1. Motivation：为什么不是 costmap、value field 或单轨迹优化

2. Scientific Question、假设与 claim ladder

3. Formal Definitions：scene、robot、pair interaction 与禁行集合

4. Core Principle：Gaussian Contact Complex 与 free-space dual

5. Orientation-Fibered Mobility Complex 与 critical events

6. Robot Motion Capability：从几何连通到有向可执行连通

7. Method Architecture：SPLATC 核心模块

8. Algorithms 与数据结构

9. Implementation Plan：从 toy proof 到大场景系统

10. Experiments、baselines、metrics 与 ablations

11. Pitfalls、proof obligations 与风险缓解

12. Related Work Positioning

13. Decision Log、Go/No-Go Criteria 与下一步

14. References

# 1. Motivation：为什么不是 costmap、value field 或单轨迹优化

## 1.1 3DGS 的真正机会不是“可微碰撞”，而是“显式分解”

3D Gaussian Splatting 将场景表示为具有显式 identity、mean、covariance、opacity 与 appearance 的各向异性 Gaussian primitives。已有 GS-native geometry processing 工作指出，只保留 Gaussian centers 会丢失 covariance 中的局部方向信息，而先转成 mesh 又可能引入额外计算和几何误差 [3]。因此，对 planning 而言，最值得利用的并不只是“可以算一个连续碰撞值”，而是每个 scene primitive 与每个 body primitive 之间存在可追踪、可微、可组合的 pair interaction。

FOCI 已经证明，用 Gaussian–Gaussian overlap 可以构造 orientation-aware 的碰撞目标并优化一条轨迹 [8]。因此，本项目不能把“Gaussian overlap”“full-body orientation-aware planning”或“直接在 GS 上轨迹优化”作为核心 novelty。真正的 residual 是：不把 pair interactions 立即求和成一个标量，而是保留 pair identity 与其在位姿域中诱导的 forbidden regions，用这些集合的组合关系恢复可复用的全局 mobility structure。

## 1.2 为什么 scalar collision map 不够

标量碰撞场把全部 pair contact 压缩为 C(q)=Agg({c_ij(q)})。这种聚合适合做局部优化或安全过滤，但它通常只保留“当前姿态有多危险”。当左右或多侧 contact gradients 互相抵消时，标量梯度可能接近零；然而 pair-resolved 结构仍然明确显示哪些独立约束在作用、哪些方向仍然自由、active set 如何交换。后者正是 corridor ridge、orientation gate 与 closure event 的候选信号。

图 2. 标量聚合与逐 pair 接触分解的不同信息量。标量值可用于碰撞评分，但无法反演完整的独立约束子空间和 active-set transition。

## 1.3 为什么不是直接预测 V_g

PNO 将 planning 写成 Eikonal solution operator learning：输入 cost function 与 goal，输出 goal-conditioned value field，并进一步作为梯度场或 A* heuristic [5]。这是合理且强的 baseline，但它回答的是“给定一个已定义的 planning domain，怎样快速求某个目标的解”。SPLATC 研究的是更前置的问题：3DGS 与机器人如何共同诱导出哪些 pose 连接真实存在，以及这些连接随 orientation、morphology 或 motion model 如何变化。V_g 可以从最终 mobility complex 上求出，但不是网络直接学习的核心对象。

## 1.4 为什么不是“完全局部”

单个局部 snapshot 不可能推出远处是否存在另一条 disconnected branch。纯 continuation 从一个 seed 出发，只能追踪已知分支。因此，完整 global planning 必须保留 global coverage；但不需要在整个位姿域上做同等精度的 dense resolution。SPLATC 的原则是：全局结构来自所有 Gaussian pair forbidden primitives 的组合关系，局部精度来自对少量临界 contact events 的连续追踪。

> 设计原则 We retain global coverage but avoid global resolution.  全局覆盖由 Gaussian 组合结构提供；高精度计算只集中在 topology-changing events 与薄通道附近。

# 2. Scientific Question、假设与 claim ladder

> Primary Scientific Question Can robot-specific global mobility be recovered from the combinatorics and critical evolution of pairwise Gaussian forbidden regions, rather than from dense state-space discretization?

中文：机器人特定的全局可移动结构，能否由逐 Gaussian 场景—身体禁行区域的交叠组合及其临界演化恢复，而不需要稠密离散完整状态空间？

## 2.1 事实、假设、目标定理必须分开

| 层级 | 陈述 | 状态 |
|---|---|---|
| Fact F1 | 在 Gaussian overlap threshold 模型下，固定机器人朝向时，单个 scene-body pair 在平移空间诱导一个椭球 superlevel set。 | 可直接推导。 |
| Fact F2 | 固定朝向下的 forbidden primitives 若为凸集，则其任意有限非空交集仍为凸集，形成 good cover；其 nerve 与障碍并集同伦等价。 | 标准 Nerve Theorem。 |
| Fact F3 | Nerve 只描述 obstacle union；planning 还必须构造 complement arrangement / free-space dual。 | 方法设计硬约束。 |
| Hypothesis H1 | pair identity 与交叠组合保留 scalar aggregation 丢失的 thin-gate 与 closure 信息。 | 需 equal-budget 机制实验。 |
| Hypothesis H2 | 典型 3DGS 场景中的 topology-changing orientation events 相对稀疏，因此 event-driven tracking 比 uniform pose discretization 更高效。 | 需复杂度与统计实验。 |
| Target T1 | 对一类 generic SE(2) 场景，完整 event tracking 与 free-cell gluing 可恢复真实几何 connectivity。 | 尚未证明。 |
| Target T2 | 加入 robot motion model 后，通过局部 kinodynamic certification 可得到有向、物理可执行的 mobility graph。 | 模块化可实现；完整性取决于 steering/certifier。 |

## 2.2 Claim ladder

Claim A（第一篇）固定朝向与 orientation slicing 下，pair-resolved contact complex 更准确地保留临界 gate，并能预测 passage closure。

Claim B（中期）orientation-fibered mobility complex 支持任意 start–goal query，并相对 dense C-space 在多 query 场景中获得更低摊销成本。

Claim C（长期）Gaussian hierarchy、critical event continuation 与 local dynamics certification 可扩展到大场景与多类机器人。

## 2.3 Setting 与明确不 claim 的内容

| 项目 | 第一阶段设置 | 不在第一阶段 claim |
|---|---|---|
| Scene | perfect、static、geometry-calibrated 3DGS；已去除 free-space outliers。 | 动态 GS、严重 photometric artifacts、未知实体语义。 |
| Robot | rigid Gaussian body；SE(2) pose；先做 fully actuated，再做 differential-drive/Ackermann。 | 任意 articulated robot、完整 SE(3) 通用性。 |
| Collision semantics | 固定 overlap threshold 或 calibrated iso-density support；提供保守模式。 | 把 opacity 直接当物理实体。 |
| Guarantee | 固定 θ 的解析 forbidden sets；orientation 上做 certified subdivision / event bracketing。 | 一次性宣称任意场景 complete。 |
| Learning | 仅作 proposal、ranking、warm start。 | 让黑盒网络决定最终 connectivity 或 safety。 |

# 3. Formal Definitions：scene、robot、pair interaction 与禁行集合

## 3.1 3DGS scene 与 Gaussian robot

环境由 N_E 个 Gaussian primitives 构成，机器人由 N_R 个固定在 body frame 中的 Gaussian primitives 构成。

> G_E = { G_i^E = (mu_i, Sigma_i, alpha_i, id_i) }_(i=1..N_E) G_R = { G_j^R = (nu_j, Lambda_j, w_j, part_j) }_(j=1..N_R) q = (t, theta) in SE(2),    T(q) = [R(theta), t]. 符号约定

其中 id_i 与 part_j 必须保留，因为项目的核心不是一个匿名密度场，而是 scene primitive 与 body part 之间可追踪的 contact factorization。

## 3.2 Pairwise overlap / contact function

对 scene Gaussian i 与 robot Gaussian j，定义 pair overlap：

> omega_ij(q) = integral  g_i^E(x) * g_j^R(T(q)^(-1)x)  dx             = Normal(mu_i ; t + R(theta)nu_j, Sigma_i + R(theta)Lambda_jR(theta)^T). Gaussian product integral 的闭式形式

根据用途可定义软碰撞代价 c_ij(q)=psi(omega_ij(q))，但 SPLATC 的核心对象不是其总和，而是每个 pair 对应的 forbidden set、交叠关系与事件。

## 3.3 固定朝向的 pairwise forbidden primitive

固定 θ 后，translation t 仅出现在一个正定二次型中。因此 overlap superlevel set：

> F_ij^theta = { t in R^2 : omega_ij(t, theta) >= tau_ij }            = { t : (t - c_ij(theta))^T A_ij(theta) (t - c_ij(theta)) <= rho_ij(theta)^2 }. 在 probabilistic threshold 模型下为椭圆

若采用实体 iso-density ellipsoid 与 Minkowski sum 语义，pair forbidden set 仍为 convex，但不一定是精确椭圆；可使用精确凸集表示或保守外接椭圆。核心拓扑构造只要求 convex cover，而不是强制椭圆。

## 3.4 Contact dictionary、active set 与局部约束几何

所有 pair contact functions 构成 contact dictionary H(q)={h_ij(q)}。在 pose q 附近，active set A(q) 只包含接近阈值或决定局部边界的 pairs。其 gradients 形成 contact Jacobian J_A(q)。

> A(q) = { (i,j) : \|h_ij(q)\| <= epsilon } J_A(q) = [ grad_q h_i1j1(q)^T ; ... ; grad_q h_ikjk(q)^T ] T_contact(q) = { v : J_A(q) v >= 0 }. 局部非穿透 tangent cone

“contact basis”可作为直觉，但正式数学对象应是 row space、normal cone、tangent cone、projector 与 singular values，因为 basis 本身不唯一。

# 4. Core Principle：Gaussian Contact Complex 与 free-space dual

## 4.1 固定 θ：从 forbidden primitives 得到 obstacle combinatorics

给定 orientation θ，全部 forbidden primitives 的并集为 O^θ=∪_{i,j}F_ij^θ。因为每个 primitive 为凸集，非空有限交集也是凸集；因此可构造 nerve：

> K_theta = Nerve( { F_ij^theta }_(i,j) ) simplex sigma exists  iff  intersection_(a in sigma) F_a^theta is non-empty. Gaussian Contact Complex at fixed orientation

在 good-cover 条件下，K_θ 与 O^θ 具有相同 homotopy type。这给出一个完全由 Gaussian pair identities 与交叠组合构成的 global obstacle representation，无需先生成 dense costmap。

## 4.2 为什么 nerve 不等于 planner

Nerve 描述的是 forbidden union，而机器人在其 complement 中移动。仅知道障碍并集有几个洞，并不能自动给出自由空间中的 region adjacency、gate width 或可 lift 的路径。因此必须显式构造：

> D_theta = DualFree( R^2 \ O^theta ) D_theta = (free cells, adjacency, gates, geometric certificates). Free-space arrangement / dual

在 SE(2) MVP 中，最稳妥实现是对 fixed-θ ellipse/convex arrangement 求 union boundary 和 complement cells，再以 shared boundary arc 或 narrow separator 作为 adjacency/gate。Nerve 可用于快速维护 obstacle combinatorics；free dual 才是 planning 对象。

图 3. 固定朝向下的三层对象：pair forbidden regions、障碍并集的 contact complex，以及真正用于规划的 free-space dual。

## 4.3 为什么这不是“学一张 map”

SPLATC 的输入不是一个固定分辨率张量，输出也不是每个 pose 的 scalar label。它从解析 forbidden primitives 的交叠关系构造一个组合复形，并通过局部 event solver 只在结构改变时更新。因此其理想复杂度与 active pairs、primitive intersections、free cells 和 critical events 相关；dense map 的复杂度则与 N_x×N_y×N_θ 全域分辨率相关。

> 复杂度目标（不是当前已证明结论） T_dense ≈ O(N_x N_y N_theta P);   T_SPLATC ≈ O(P_near log N_E + I + E + C_local)，其中 P_near 为实际邻近 pair 数，I 为交叠关系数，E 为 orientation critical events，C_local 为局部认证成本。最坏情况仍可能出现 arrangement explosion，必须在实验中测量。

# 5. Orientation-Fibered Mobility Complex 与 critical events

## 5.1 从一族 2D free-space dual 到完整 SE(2) connectivity

随着 θ 变化，每个 F_ij^θ 的中心、形状和方向连续变化。因此得到一族 D_θ。绝大多数 θ 区间内，free-cell combinatorics 不变；只有当 forbidden boundaries 发生切触、交叠出现/消失、free gap 归零或高阶交点改变时，拓扑才发生事件。

> M_G,R = Glue_theta { D_theta } node  = free cell x orientation interval edge  = within-slice adjacency or cross-slice continuation across a certified event. Orientation-fibered Gaussian Mobility Complex

图 4. orientation slicing、passage closure event 与跨切片 gluing。右侧稀疏图是完整位姿域 free-space connectivity 的组合表示。

## 5.2 Critical event 的候选类型

| Event 类型 | 几何定义 | 对 mobility complex 的影响 |
|---|---|---|
| Pair tangency | 两个 forbidden boundaries 的最小距离降为 0，法向共线。 | nerve edge/simplex 出生或消失；free gate 可能关闭/打开。 |
| Obstacle–boundary tangency | forbidden primitive 与 workspace boundary 接触。 | 自由区域分裂、合并或消失。 |
| Higher-order intersection | 三个或更多 primitives 出现共同交点。 | free-cell adjacency 发生组合切换。 |
| Active-set exchange | 决定局部 boundary/gate 的 pair identity 改变。 | continuation 分支切换，但拓扑可能不变。 |
| Rank / singular event | contact Jacobian 的最小奇异值趋零或 feasible cone 维数改变。 | 候选 branch split/merge 或 orientation gate closure。 |
| Morphology event | 机器人尺寸/形状参数变化使上述事件发生。 | robot-specific topology bifurcation。 |

## 5.3 Event continuation 与 certified fallback

事件定位不应只依赖 uniform θ sampling。建议维护候选 pair/tuple 的 gap functions，并使用 bracketed root finding、pseudo-arclength continuation 和 interval bounds。为避免漏掉事件，保留一个 certified angular subdivision fallback：若某个 θ interval 的组合结构无法由 bounds 证明不变，则继续二分。

Fast path由上一切片的 active pairs 和 BVH 邻近关系预测下一事件。

Safe path对未认证区间做 interval subdivision，直到事件被 bracket 或区间被证明无事件。

Degeneracy handling对近同时事件使用 symbolic perturbation / event clustering，并记录 involved pair IDs。

Output-sensitive goal大多数常规区间不重建整个 arrangement，只更新受事件影响的局部 simplices 与 free cells。

# 6. Robot Motion Capability：从几何连通到有向可执行连通

## 6.1 身体形态与运动能力是两个独立条件

Gaussian robot 决定 body occupancy 与几何 forbidden sets；motion model 决定机器人能否执行某个局部连接。相同 body morphology 下，全向机器人、差速车与 Ackermann 车辆的几何 free space 相同，但有向可达关系不同。

> q_dot = f_r(q,u),    u in U_r R_Delta(q) = { q' : exists u(.) satisfying dynamics and all pairwise safety constraints over [0,Delta] }. 有限时域 local reachable relation

因此 M6 输出的是 candidate geometric mobility complex；M7 再将边转成 directed, executable edges。不能仅以 null-space 维数或瞬时 feasible direction 判断 nonholonomic reachability。

## 6.2 局部 edge certification 后端

| 机器人类型 | 推荐 certifier | 输出证据 |
|---|---|---|
| 全向 SE(2) | straight/curvature-bounded local path + continuous pair collision check | 无碰撞 path segment 与 clearance bound。 |
| Differential drive | motion primitives、state lattice 或短时域 direct collocation | entry/exit pose、方向、可执行控制序列。 |
| Ackermann / car-like | Dubins / Reeds–Shepp 候选 + constrained optimization | 最小转弯半径与有向 edge。 |
| UAV / general dynamics | 短时域 trajectory optimization、reachable tube 或局部 HJ | 有限时域 reachable certificate；计算只在少量 gate patch 上。 |
| Articulated robot | 局部 sampling-based planner + exact Gaussian collision checker | 概率性或经验 edge validity；不在 MVP 完整性 claim 内。 |

## 6.3 Query 与 path lifting

给定 start s 与 goal g，先定位其 mobility nodes，在 directed graph 上运行 A*/Dijkstra 得到 node/gate sequence，再由各 edge 的 local certificate lift 成连续 trajectory。最后使用 pairwise overlap / support constraints 做连续碰撞验证；若失败，只 refine 对应 edge 的 orientation interval 或 local arrangement。

> P* = GraphSearch( M_G,R, node(s), node(g) ) tau_sg = LiftAndCertify( P*, local steering, pairwise collision oracle ). Query-time planning

# 7. Method Architecture：SPLATC 核心模块

图 5. SPLATC 模块架构。核心计算是解析/组合/认证；learning 只可作为候选生成与加速层。

| 模块 | 输入 | 核心职责 | 输出 | 验证方式 |
|---|---|---|---|---|
| M0 几何校准 | raw/perfect 3DGS、robot splats | 确定 physical support、阈值、workspace boundary、去重与 outlier policy。 | GeometryModel | 与 mesh/ground-truth collision 做 calibration curve。 |
| M1 Pair Pruner | GeometryModel、θ interval | BVH / spatial hashing / covariance bounds，找可能交互的 scene-body pairs。 | CandidatePairSet | recall=100% 的保守剪枝；记录 false positive。 |
| M2 Forbidden Primitive | candidate pairs、θ | 构造 ellipse/convex set、gradient、gap function 与 bounding certificate。 | PairPrimitive | 数值与闭式 overlap、自动微分交叉验证。 |
| M3 Contact Complex | all primitives at fixed θ | 维护 nerve/simplex 与 obstacle union combinatorics。 | K_θ | 与 exact polygonized union 的 Betti numbers 对比。 |
| M4 Free-space Dual | obstacle union / arrangement | 构造 complement cells、adjacency、gate geometry。 | D_θ | 与 dense exact C-space connectivity 对比。 |
| M5 Event Tracker | D_θ、pair functions | 定位 tangency/intersection/active-set/morphology events 并局部更新。 | event stream | 与高分辨率 θ sweep 的事件 ground truth 对比。 |
| M6 Mobility Compiler | {D_θ}+events | 跨 orientation interval glue，构造目标无关 global complex。 | M_geom | connectivity precision/recall、thin-gate recall。 |
| M7 Dynamics Certifier | M_geom、motion model | 验证 edge 的有限时域可执行性，生成 directed edges 和 local controllers。 | M_mob + certificates | simulation/hardware execution、continuous safety check。 |

# 8. Algorithms 与数据结构

## 8.1 Algorithm A：Build fixed-orientation slice

> Algorithm BuildSlice(theta):   1. Pairs <- PairPruner.query(scene_BVH, robot_BVH, theta)   2. Primitives <- { BuildForbiddenSet(pair, theta) for pair in Pairs }   3. K_theta <- BuildOrUpdateNerve(Primitives)   4. U_theta <- BuildConvexUnionArrangement(Primitives)   5. D_theta <- BuildFreeSpaceDual(workspace \ U_theta)   6. Attach gate geometry, involved pair IDs, clearance certificates   7. return SliceState(theta, K_theta, D_theta, Primitives)

## 8.2 Algorithm B：Track orientation events

> Algorithm TrackEvents(theta0, theta1, SliceState0):   1. CandidateEvents <- PredictFromActivePairsAndBVH(SliceState0)   2. For each candidate event e:        bracket theta* using conservative gap bounds        refine theta* by root finding / pseudo-arclength continuation   3. Let theta_next be the earliest certified event   4. Advance primitives to theta_next; update only affected simplices/cells   5. If interval [theta0, theta_next] cannot be certified event-free:        subdivide interval and rebuild local slice   6. emit Event(theta_next, type, involved_pairs, before/after certificates)

## 8.3 Algorithm C：Compile and query the mobility complex

> Algorithm CompileMobility(scene, robot, motion_model):   1. S0 <- BuildSlice(theta = 0)   2. Sweep theta around S^1 using TrackEvents   3. For each event-free interval, create nodes = free cells x theta interval   4. Glue nodes across intervals using cell continuation and event updates   5. For each candidate edge, run local kinodynamic certification   6. Store directed edge, local path/controller, and safety certificate   7. return MobilityComplex  Algorithm Query(MobilityComplex M, start s, goal g):   1. locate ns <- LocateNode(s), ng <- LocateNode(g)   2. route <- AStar(M, ns, ng)   3. tau <- concatenate certified local edge motions   4. continuously validate pairwise safety; refine only failing edge   5. return tau

## 8.4 建议数据结构

| 数据对象 | 关键字段 |
|---|---|
| GaussianRecord | id, mean, covariance, opacity, geometry confidence, hierarchy node |
| RobotGaussian | part_id, body-frame mean/covariance, physical support parameters |
| PairPrimitive | scene_id, robot_part_id, theta interval, quadric/convex representation, bounds, derivatives |
| SimplexRecord | member pair IDs, intersection witness, validity theta interval |
| FreeCell | cell ID, boundary arcs, neighboring gates, representative pose, theta interval |
| CriticalEvent | theta*, type, involved pairs, before/after combinatorics, numerical certificate |
| MobilityNode | free-cell lineage, orientation interval, admissible entry/exit sets |
| MobilityEdge | direction, cost, local path/controller, safety/dynamics certificate |

# 9. Implementation Plan：从 toy proof 到大场景系统

## 9.1 Phase 0：验证单 pair 与物理语义

实现 Gaussian overlap 闭式函数及对 translation/yaw 的解析或自动微分梯度。

验证固定 θ superlevel set 的 ellipse 参数与 brute-force samples 一致。

建立两种 collision semantics：probabilistic overlap threshold 与 conservative iso-density support。

用 exact mesh/primitive collision 标定 tau 或 kappa，并绘制 false-safe / false-blocked 曲线。

## 9.2 Phase 1：固定朝向的 contact complex 与 free dual

从 2D synthetic scene 开始：圆/椭圆 scene splats + 椭圆 robot splats。

使用 Shapely/CGAL 构造 ellipse polygon approximation、union 与 complement cells。

独立实现 nerve/simplex 检测，并验证其 Betti numbers 与 obstacle union 一致。

构造 free-cell adjacency graph，验证 U-shape、双门、多障碍和狭缝的 connectivity。

## 9.3 Phase 2：orientation event continuation

均匀粗采样 θ 作为 reference，但不作为最终算法。

对关键 ellipse pairs 建 gap function，检测 intersection/tangency roots。

实现 event bracketing、root solve、pseudo-arclength continuation 和 interval subdivision fallback。

在 door-width / robot-aspect-ratio sweep 中测 gate interval 与 closure threshold。

## 9.4 Phase 3：mobility complex、query 与 path lifting

建立 free-cell lineage：在 event-free θ interval 内追踪同一 free component。

将 free cells × θ intervals glue 成 SE(2) combinatorial complex。

对 start/goal 做 node localization、graph search、local geometric lifting 与 continuous collision validation。

评估同一 scene 上大量 s–g queries 的摊销成本。

## 9.5 Phase 4：nonholonomic overlay

先加入 differential-drive 或 Ackermann motion primitives。

将 geometric edges 变成 directed candidate edges，并记录合法 entry/exit orientation sets。

在 gate patch 上做 local collocation / lattice search，成功后缓存控制序列。

测试同一 geometry 在不同 robot motion models 下产生不同 mobility graph。

## 9.6 Phase 5：大场景与可选 learning

Hierarchyscene Gaussian BVH、robot part hierarchy、orientation interval bounds。

Incremental updatesevent 只更新受影响的 primitives、simplices 和 free cells。

Learned proposal预测高概率 active pairs、event type 或下一 event angle；必须保留 conservative fallback。

Cross-scene amortization学习 reusable proposal model，但不改变 analytic/certified backend。

## 9.7 实现路线与 claim ladder

图 6. 从解析单 pair 到大场景系统的分阶段路线。第一篇应停在 Phase 2 或 Phase 3，而不是一次性承诺任意机器人与完整全局拓扑。

## 9.8 推荐软件栈

| 阶段 | 推荐工具 | 理由 |
|---|---|---|
| 快速研究原型 | Python, NumPy/JAX or PyTorch, SciPy, Shapely, NetworkX | 便于自动微分、root solve、2D arrangement 与快速可视化。 |
| 精确/保守几何 | CGAL（arrangement/union）、interval arithmetic library | 用于 exact predicates、degeneracy handling 与 certified subdivision。 |
| GPU pair evaluation | CUDA / PyTorch custom kernels | 批量 overlap、bounds、gradient 与 candidate ranking。 |
| 局部轨迹认证 | CasADi / Ceres / OMPL / custom state lattice | 不同 robot motion model 的局部 backend。 |
| 3DGS I/O | PLY/3DGS parser, Open3D only for visualization | 核心计算不应依赖 mesh reconstruction。 |

# 10. Experiments、baselines、metrics 与 ablations

## 10.1 Synthetic benchmark families

| Family | 变化参数 | 验证问题 |
|---|---|---|
| Single Door | 门宽、倾角、robot width/length | orientation interval、physical closure threshold、pair vs scalar。 |
| Double Door / multi-homotopy | 两门宽度、相对位置 | 是否保留所有 route classes；free dual 是否漏 branch。 |
| U-shape / cul-de-sac | 凹形障碍、出口宽度 | 局部 contact 与全局 connectivity 的一致性。 |
| Keyhole / rotating slot | 入口与内部空间、机器人 aspect ratio | 需要旋转进入的 thin pose passage。 |
| Branch split/merge | 障碍间距连续变化 | critical event 定位与 cell lineage。 |
| Morphology sweep | 机器人长度、宽度、body-part covariance | topology bifurcation 与 closure prediction。 |
| Nonholonomic gate | turn radius、reverse allowed | geometry edge 与 executable edge 的差异。 |
| Large corridor network | room/corridor 数、splat 密度 | 输出规模、multi-query amortization 与 hierarchy。 |

## 10.2 Baselines

Dense pose occupancy + A*在 (x,y,θ) 网格上精确碰撞；作为 connectivity ground truth 与计算基线。

Scalar Gaussian costmap同一 pair oracle，但先 sum/max 聚合，再用 A*/FMM/RRT；回答“是不是只要一张 map”。

Uniform orientation slicing相同 fixed-θ geometry backend，但不用 event continuation。

PRM / RRT*使用 exact Gaussian collision checker；比较 narrow-passage sample efficiency。

FOCI-style trajectory optimization使用 aggregated overlap objective 和同等初值预算；比较 single-query success 与多-query reuse。

Direct value predictor / PNO-like输入 scene/robot/goal，预测 V_g；比较 topology errors、query latency 和训练依赖。

Mesh/SDF analogue将相同 scene 转为 mesh/SDF 或 feature pairs，测试 GS-native residual 而非 GS-exclusive claim。

## 10.3 Core metrics

| 类别 | 指标 |
|---|---|
| Topology | free-component precision/recall、gate precision/recall、Betti-number error、route-class recall。 |
| Thin passage | 最小可检测 orientation interval、closure-threshold error、near-closure success rate。 |
| Planning | success、collision rate、path length/cost、minimum clearance、dynamics feasibility。 |
| Efficiency | build time、event count、exact pair queries、memory、query latency、break-even query count。 |
| Robustness | splat density change、duplicate/outlier splats、threshold perturbation、near-degenerate events。 |
| Generalization | unseen scenes、robot morphology sweep、motion-model changes（若加入 learning）。 |

## 10.4 必做 ablations

去掉 pair identity，只保留 scalar overlap / clearance。

将 full anisotropic covariance 替换为 isotropic 或 centers-only。

去掉 event continuation，改用相同预算的 uniform θ samples。

只用 obstacle nerve，不构造 free-space dual。

去掉 hierarchy，测 pair explosion。

learning proposal 开/关，同时保持最终 certifier 不变。

只做 geometry connectivity 与加入 kinodynamic certification 的差异。

# 11. Pitfalls、proof obligations 与风险缓解

| 风险 | 为什么严重 | 缓解/验证 |
|---|---|---|
| Gaussian 无限支撑 | overlap 永远非零；“接触/碰撞”若无阈值就没有物理意义。 | 明确定义 τ/κ；提供 probabilistic 与 conservative support 两种模式。 |
| 3DGS 不等于实体几何 | photometric splats 可能在 free space、内部或表面外。 | MVP 假设 geometry-calibrated perfect GS；后续单独研究 outlier/uncertainty。 |
| Nerve 与 free space 混淆 | obstacle union 同伦不自动给出 complement path graph。 | 必须实现 arrangement/complement dual；单独验证 free connectivity。 |
| Arrangement explosion | 凸集交叠在最坏情况下可二次或更高增长。 | BVH、local updates、complex simplification；报告 output size，不隐瞒 worst case。 |
| 事件遗漏 | uniform θ 或局部 continuation 可能跳过极窄事件。 | interval bounds、certified subdivision、event bracketing、high-res ground truth。 |
| 高阶/同时事件 | 多个 tangency 同时发生会使 branch labeling 不稳定。 | symbolic perturbation、event clustering、exact predicates。 |
| Path lifting 失败 | 组合边存在不代表能找到连续、无碰撞路径。 | 每条边存 witness/portal；continuous collision check；失败则局部 refine。 |
| Nonholonomic 误判 | 瞬时 tangent cone 不等价于长期可达。 | 有限时域 local reachability certification；mobility graph 使用有向边。 |
| GS-native 不等于 GS-exclusive | mesh/SDF feature pairs 也可能构造类似 contact structure。 | 做 analogue baseline；claim 3DGS 提供原生解析 factorization 与效率 residual。 |
| Learning 破坏保证 | proposal 模型会漏 pair/事件。 | 保守 fallback；learning 只能排序，不能删除未经界证明的候选。 |
| 动态场景更新 | 复杂结构可能频繁失效。 | 不在第一篇；以后做局部 complex update 与 event invalidation。 |

## 11.1 关键 proof obligations

固定 θ forbidden primitive 的解析/保守正确性。

contact complex 与 obstacle union 的 good-cover 条件。

free-space dual 对 complement connectivity 的正确性。

event-free orientation interval 内组合结构保持不变的 certificate。

event update 后 free-cell lineage 与 gluing 的一致性。

对所声明 setting 的 resolution completeness 或 probabilistic completeness。

local kinodynamic edge certificate 与连续碰撞安全。

## 11.2 最危险的失败模式

> Stop condition 若在固定 θ 上构造 free-space dual 的成本已与 dense grid 相当，或 event 数量随 splat 数爆炸，或 pair-resolved 方法在 equal-budget thin-gate benchmark 上无法稳定优于 scalar aggregation，则需要将项目重构为“GS-native narrow-passage analysis”而非 general global planner。

# 12. Related Work Positioning

以下比较用于明确科学层级，而不是声称其他方法不能完成 planning。

| 方法 | 核心对象 | 如何 planning | 与 SPLATC 的边界 |
|---|---|---|---|
| Heat Method [1] | 固定 domain 上的 geodesic distance solver | 短时热扩散 + 归一化梯度 + Poisson；可预分解重复查询。 | domain/connectivity 已给定；不从 GS pair structure 构造 robot-specific free-space topology。 |
| Neural Green’s Functions [2] | linear PDE 的 domain-conditioned solution operator | 预测 Green kernel/eigenfeatures，处理任意 source/boundary。 | 学习已知 PDE 的 solution operator；不恢复未知 mobility topology。 |
| LBO for GS [3] | GS surface 上的 Laplace–Beltrami operator | Mahalanobis neighborhood + covariance-aware LBO，支持 heat geodesic。 | 利用 covariance 恢复 surface connectivity；本项目利用 scene-body pair forbidden sets 恢复 robot pose mobility。 |
| Harmonic customization [4] | 可定制 homotopy 的 harmonic potential | 优化 tree structure/weights，再沿负梯度。 | planning domain 与 harmonic family 已给定；本项目先恢复 robot-specific route existence。 |
| PNO [5] | cost function → goal-conditioned value function | 学习 Eikonal solution operator；梯度或 A* heuristic。 | 直接预测 V_g；SPLATC 输出 goal-independent mobility complex。 |
| Representation-to-Action [6] | Laplacian basis + Green potential | 用 eigenbasis 合成 goal potential，沿梯度行动。 | 已有 spatial domain/Laplacian；本项目研究 domain connectivity 如何由 GS-body interaction产生。 |
| ALPS [7] | transition-derived Laplacian latent space | cluster graph 上 Dijkstra，高层 subgoals + CEM。 | 从离线 transitions 学 reachability；SPLATC 从显式 GS geometry 与 robot body/dynamics 编译。 |
| FOCI [8] | 聚合 Gaussian overlap trajectory objective | 给定 coarse initial route，优化一条 orientation-aware trajectory。 | 用 contacts score/optimize one path；SPLATC 用 pair forbidden-set combinatorics infer which route classes exist。 |

## 12.1 最清楚的一句话区分 FOCI

> Positioning FOCI uses Gaussian contacts to score and optimize a path. SPLATC uses the combinatorics and critical evolution of Gaussian pair contacts to infer which paths exist and to compile a reusable global mobility structure.

# 13. Decision Log、Go/No-Go Criteria 与下一步

## 13.1 已锁定的设计决定

| Decision | 理由 |
|---|---|
| 不把“operator learning”作为主叙事 | 任何 scene-to-field mapping 都可叫 operator；术语本身不构成 scientific contribution。 |
| 核心输出是 mobility complex，不是 V_g | 支持全部 goals/starts；显式 route topology、gate events 与 morphology changes。 |
| global layer 必须来自 GS primitives | 避免外接 coarse grid；固定 θ 的 forbidden-set complex 提供原生 global combinatorics。 |
| local continuation 只负责 critical precision | 它不负责凭空发现 disconnected branches；candidate coverage 来自 global complex。 |
| 正式对象用 cone/subspace/complex，不用任意 basis | basis 不唯一；数学上使用 contact Jacobian、normal/tangent cone 与 singular events。 |
| nonholonomic 采用 directed local certification | 瞬时可行方向不足以判断长期可达。 |
| learning 是可选加速层 | 防止黑盒拓扑错误并保留可解释/可认证 claim。 |

## 13.2 Go criteria

固定 θ contact complex + free dual 在所有 synthetic families 上与 exact dense ground truth 的 connectivity 一致。

event tracker 在相同 pair-query budget 下显著降低 closure-threshold error，并避免漏掉 thin orientation gates。

mobility complex 的 node/edge 数显著小于 dense pose grid，并在多 query 时出现明确 break-even point。

pair identity、anisotropic covariance、event continuation 三个 ablation 均产生可解释的性能下降。

FOCI-style trajectory optimizer 在缺少好初值或多 route classes 时不能替代全局 structure；SPLATC 能提供可靠 seed/route class。

## 13.3 No-Go / reframe criteria

contact complex 构建和维护的复杂度在真实场景中持续接近或超过 dense C-space。

scalar adaptive field + standard planner 在 equal-budget benchmark 上达到相同 thin-gate recall。

critical events 不稀疏，orientation sweep 近似退化为全域离散。

free-space dual 无法从 GS forbidden primitives 稳定构造，或 path lifting 频繁失败。

GS-native analogue 对 mesh/SDF feature methods 没有任何效率或精度 residual。

## 13.4 立即执行的 8 个任务

冻结 MVP setting：perfect 3DGS、rigid Gaussian robot、SE(2)、静态 workspace。

实现并单元测试 Gaussian overlap、gradient、fixed-θ ellipse extraction。

实现 fixed-θ union/arrangement 与 free-cell dual；不要先做 learning。

构建 exact dense C-space ground-truth generator。

完成 single-door、double-door、U-shape、keyhole 四个 benchmark family。

实现 scalar aggregation baseline 与 equal-budget protocol。

实现 orientation event bracketing + interval subdivision；先不追求最优复杂度。

在数据支持后再决定论文主 claim 是“thin-gate mechanism”还是“完整 mobility complex”。

# 14. References

[1] Crane, K., Weischedel, C., and Wardetzky, M. Geodesics in Heat. ACM Transactions on Graphics, 2013. arXiv:1204.6216.

[2] Yoo, S., Yeo, K., Hwang, J., and Sung, M. Neural Green’s Functions. NeurIPS 2025. arXiv:2511.01924.

[3] Zhou, H. and Lähner, Z. Laplace–Beltrami Operator for Gaussian Splatting. arXiv:2502.17531v2, 2026.

[4] Wang, S., Guo, T., and Guo, M. Customize Harmonic Potential Fields via Hybrid Optimization over Homotopic Paths. arXiv:2507.09858, 2025.

[5] Matada, S., Bhan, L., Shi, Y., and Atanasov, N. Generalizable Motion Planning via Operator Learning. ICLR 2025.

[6] Zuo, J., He, Y., Zhang, W.-H., Fang, F., and Wu, S. From Representation to Action: A Unified Laplacian Framework for Spatial Representation and Path Planning. ICML 2026.

[7] Shehmar, D., Schlegel, M., Taylor, M. E., and Machado, M. C. Laplacian Representations for Decision-Time Planning. ICML 2026. arXiv:2602.05031v2.

[8] Gomez Andreu, M., Wilder-Smith, M., Klemm, V., Patil, V., Tordesillas, J., and Hutter, M. FOCI: Trajectory Optimization on Gaussian Splats. IROS 2025. arXiv:2505.08510.

[9] Edelsbrunner, H. and Harer, J. Computational Topology: An Introduction. American Mathematical Society, 2010.

[10] Basch, J., Guibas, L. J., and Hershberger, J. Data Structures for Mobile Data. Journal of Algorithms, 1999.

[11] Allgower, E. L. and Georg, K. Introduction to Numerical Continuation Methods. SIAM, 2003 edition.

[12] LaValle, S. M. Planning Algorithms. Cambridge University Press, 2006.

> 最终定义 SPLATC 是一个 GS-native geometry/topology compiler：它把 scene-body Gaussian pairs 在 pose domain 中诱导的 forbidden primitives 编译为 orientation-fibered mobility complex，并通过局部 dynamics certification 将几何连接转成机器人真实可执行的有向移动结构。
