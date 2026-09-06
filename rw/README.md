# SplatHJB related-work pack

更新日期：2026-08-05

这里收集的是与当前 SplatHJB 方法最直接相关的论文：以 Gaussian splat 所表达的连续物理 measure/field 为输入，学习 full-state UAV HJB value/policy solution operator，并要求结果不依赖具体的 splat 拆分方式。

当前目录包含 **25 篇已验真的 PDF**。每一篇都通过了 PDF 类型、非零页数、前两页可抽取文本和 SHA-256 校验。另有 1 篇开放获取论文的出版社 PDF 下载被 HTTP 403 拦截，已在 `manifest.csv` 和 `allinone.md` 中如实记录，没有用非正规镜像替代。

## 目前最重要的检索结论

截至本次检索，**没有发现直接完成 `Gaussian measure → arbitrary-goal, full-state UAV HJB solution operator` 的论文**。

但我们需要面对一个很强的“组合式反事实基线”：

```text
Gaussian measure
    ↓ 解析或采样成连续 hazard/cost field
PNO / HJRNO / 其他 HJ operator
    ↓
value field + feedback policy
```

也就是说，论文的新颖性不能只写成“GS 上学一个 value field”或“用 NN 加速 HJB”。这些部件分别已有先例。真正需要守住的是它们的**联合结构**：

1. 输入定义在 Gaussian 所表达的 measure/连续物理场上，而不是固定长度 splat 列表或栅格；
2. measure-preserving split/merge/refinement 后，value 与 policy 应保持一致；
3. operator 同时条件化场景、任意 goal 和完整 UAV state/dynamics；
4. 输出是可直接闭环执行的 HJB feedback，而不是先画粗路径、再局部优化或只做安全过滤；
5. 对 measure perturbation → value error → policy/closed-loop error 给出可检验的稳定性链条。

## 建议先读的 10 篇

| 顺序 | 论文 | 先看什么 | 它对我们的压力 |
|---:|---|---|---|
| 1 | [Splat-Nav](splatnav_2403_02751.pdf) | GS 如何变成 safe polytopes 与 Bézier 路径 | 证明 native GS planning 已经存在；我们的差异必须是直接 value feedback、非 corridor/seed route |
| 2 | [FOCI](foci_2505_08510.pdf) | anisotropic Gaussian overlap 与 full-body trajectory optimization | 最强 native-GS 连续碰撞/轨迹优化对照 |
| 3 | [SPLANNING](splanning_2409_16915.pdf) | normalized GS 与 collision-risk upper bound | 风险建模与 GS 归一化不能被误写成我们的独占贡献 |
| 4 | [PNO](planning_neural_operator_2410_17547.pdf) | cost-function → Eikonal-value operator | 最危险的替代方案；阻断“场景到 value operator 本身是新的” |
| 5 | [HJRNO](hjrno_2504_19989.pdf) | obstacle/dynamics → HJ reachability operator | 阻断“neural HJ operator + feedback 是新的” |
| 6 | [Mean-field neural networks](mean_field_wasserstein_operator_2210_15179.pdf) | Wasserstein measure → function operator | 最直接挑战“measure-valued input/operator 是新的” |
| 7 | [ReNO](reno_2305_19913.pdf) 与 [DI-Nets](dinet_2206_01178.pdf) | representation equivalence、aliasing、finite discretization error | 我们必须把 split/merge 不变性讲得比普通 discretization invariance 更具体 |
| 8 | [Adaptive Deep HJB](adaptive_deep_hjb_1907_05317.pdf) | 高维 HJB value/gradient 与实时反馈 | 阻断“高维 neural HJB feedback 是新的” |
| 9 | [Neural Navigation Functions](neural_navigation_functions_2606_03756.pdf) | zero-shot、全局一致、目标唯一极小 | 强迫我们正面测 global planning quality，而不只测速度 |
| 10 | [FastBridge](fastbridge_2607_01200.pdf) | full quadrotor realization 与 GS safety filter | 强迫我们用真实动力学、执行误差和闭环安全做验证 |

## 按研究问题分组

### A. 原生 GS 规划、轨迹优化与安全控制

- [Splat-Nav](splatnav_2403_02751.pdf)：safe polytope + Bézier global planning。
- [FOCI](foci_2505_08510.pdf)：直接利用 Gaussian overlap 做连续、姿态相关的轨迹优化。
- [SPLANNING](splanning_2409_16915.pdf)：normalized GS 上的 collision-risk upper bound 与轨迹优化。
- [SAFER-Splat](safer_splat_2409_09868.pdf)：在线 GS map 上的 CBF action filter。
- [Analytic CCBF](analytic_ccbf_2509_14421.pdf)：3DGS 上解析 collision-cone barrier。
- [Conflict-Aware CBF](conflict_aware_2605_20566.pdf)：GS field 中感知与控制约束冲突。
- [FastBridge](fastbridge_2607_01200.pdf)：高速四旋翼、执行器与高相对阶 safety filter。
- [GAVIS](gavis_2605_30342.pdf)：active mapping/visibility 邻接工作；静态规划不是其主任务。

### B. 直接 value/cost-to-go field 与无种子梯度规划

- [c2g-HOF](c2g_hof_highdim_2012_06023.pdf)：workspace → cost-to-go network，沿梯度生成路径。
- [Continuous Environment Fields](continuous_environment_fields_2111_13997.pdf)：连续 reaching-distance field。
- [NTFields](ntfields_2210_00120.pdf)：physics-informed Eikonal time field。
- [H-NTFields](h_ntfields_2604_13204.pdf)：稀疏 roadmap 提供全局拓扑锚点，PDE loss 保持局部几何。
- [PC-Planner](pc_planner_2410_12805.pdf)：shape-aware distance + physics-constrained value learning。
- [Neural Navigation Functions](neural_navigation_functions_2606_03756.pdf)：zero-shot、全局结构化的 navigation function。

### C. HJB/HJI、neural PDE 与 solution operator

- [DeepReach](deepreach_2011_02082.pdf)：高维 neural HJI reachability 与 safety feedback。
- [HJRNO](hjrno_2504_19989.pdf)：HJ reachability neural operator。
- [PNO](planning_neural_operator_2410_17547.pdf)：跨 cost field 的 Eikonal solution operator。
- [HJ policy iteration + DeepONet](hj_policy_iteration_deeponet_2406_10920.pdf)：跨 terminal function 的 HJB/operator inference。
- [Adaptive Deep HJB](adaptive_deep_hjb_1907_05317.pdf)：半全局高维 HJB value/gradient 与实时 feedback。

### D. Measure、表示等价与离散化稳定性

- [Mean-field neural networks](mean_field_wasserstein_operator_2210_15179.pdf)：Wasserstein space 上的 measure-to-function learning。
- [ReNO](reno_2305_19913.pdf)：operator aliasing 与 representation-equivalent neural operators。
- [DI-Nets](dinet_2206_01178.pdf)：有限采样下的 discretization-invariance bounds。
- [Gaussian Particle Operator](gaussian_particle_operator_2602_21551.pdf)：Gaussian latent basis 的 PDE operator；是邻接启发，不是 native-3DGS 直接 collision。

### E. 完整 UAV dynamics 与强执行基线

- [Time-Optimal Quadrotor Waypoint Flight](time_optimal_quadrotor_2108_04537.pdf)：完整执行器能力与真正 time allocation。
- [Perception-Aware Time-Optimal Planning](perception_aware_time_optimal_quadrotor_2603_04305.pdf)：full nonlinear dynamics、rotor limits、aerodynamics、几何与视觉约束。
- *Optimal control for quadrotors UAV based on deep neural network approximations of stable manifold of HJB equation*：12D quadrotor neural-HJB feedback 的直接先例；出版社页面可读且标注 open access，但本次 PDF 下载被 HTTP 403 拦截，详见 `manifest.csv`。

## 读完后要产出的对比表

下一步不要继续泛读，而应从每篇抽取同一组字段：

```text
输入表示 / 是否依赖固定离散化
goal 是否可变
state 与 dynamics 维度
求的是 path、trajectory、value、policy 还是 safety filter
是否需要 seed route / roadmap / corridor
是否给 global optimality / safety / stability 保证
训练教师与标签来源
推理时间与地图规模
真实 UAV 闭环验证
split / merge / refinement 反事实是否测试
```

`manifest.csv` 是机器可读清单；`allinone.md` 保存检索范围、错误与未下载的次级候选。
