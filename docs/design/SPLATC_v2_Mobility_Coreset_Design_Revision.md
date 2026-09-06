# SPLATC v2.0 设计修订：从“完整 3DGS 的确定性编译”到“学习规划充分几何 + 认证编译”

> **文档目的**：明确说明相对于上一版 geometry-only 设计，为什么要改、改了什么、哪些内容保留、哪些内容删除，以及新版应如何实现和验证。
>
> **一句话变化**：上一版让确定性 Gaussian mobility compiler 直接处理完整 3DGS；新版只学习一个机器人条件化的 **mobility coreset**，把原始 3DGS 压缩为对该机器人规划充分的最小 Gaussian 几何，再由原有的认证几何编译器恢复全局可达结构。

---

## 0. 版本定位

### 上一版（v1）的主线

上一版的核心流程是：

\[
\mathcal G
\xrightarrow{\text{full deterministic compiler}}
\bigl(\mathcal M_r^{\mathrm{safe}},\mathcal M_r^{\mathrm{possible}}\bigr)
\xrightarrow{(s,g)}
\tau\ \text{or REFINE}.
\]

其中完整 3DGS \(\mathcal G\) 直接进入：

1. Gaussian finite support；
2. pairwise configuration obstacles；
3. inner/outer convex envelopes；
4. obstacle nerve 与 free-space arrangement；
5. certified orientation slabs；
6. safe/possible mobility graphs；
7. path lifting 与连续验证。

这条路线数学上比早期版本可靠，但仍有一个明显问题：**完整 3DGS 是为渲染构建的过完备表示，直接将全部 splats 送入组合几何编译器，会造成巨大的 pair、intersection 和 event 复杂度。**

### 新版（v2）的主线

新版在认证编译器之前增加且只增加一个学习对象：

\[
\boxed{
(\mathcal G,r)
\xrightarrow{\mathcal B_\theta}
\Pi_r
\xrightarrow{\text{certified macro geometry}}
(\widetilde{\mathcal G}_r^{-},\widetilde{\mathcal G}_r^{+})
\xrightarrow{\operatorname{CertifiedCompile}}
(\mathcal M_r^{\mathrm{safe}},\mathcal M_r^{\mathrm{possible}})
\xrightarrow{(s,g)}
\tau\ \text{or REFINE}
}
\]

其中：

- \(\mathcal B_\theta\) 是唯一的学习模块；
- \(\Pi_r\) 是对 Gaussian hierarchy 的机器人条件化 cut / partition；
- \(\widetilde{\mathcal G}_r^{-}\) 与 \(\widetilde{\mathcal G}_r^{+}\) 是由确定性几何构造的内外压缩表示；
- 认证编译器仍负责拓扑、安全和路径证书；
- 如果压缩导致结论不确定，系统只展开相关 hierarchy nodes，而不是整场景回退。

新版的核心原则是：

> **Learning discovers the minimal planning-sufficient geometry; certified geometry decides connectivity and safety.**

---

## 1. 为什么必须修改上一版

### 1.1 上一版最强的部分不是问题

以下内容仍然正确，应完整保留：

- Gaussian support 的有限实体化；
- 固定朝向下的 pairwise configuration obstacle；
- 解析 support function；
- inner/outer obstacle sandwich；
- obstacle nerve 与 free-space arrangement 的分离；
- event-driven orientation subdivision；
- safe/possible 双 mobility graph；
- `REACHABLE / UNREACHABLE / UNKNOWN` 三值输出；
- safe edge 必须绑定连续 witness；
- 路径必须在原始 supports 上连续复验。

这些模块构成正确性的来源，不能被网络替代。

### 1.2 上一版真正的瓶颈

假设场景有 \(N_E\) 个 splats，机器人有 \(N_R\) 个 body primitives。即使只看 pairwise interaction，也存在：

\[
O(N_E N_R)
\]

个潜在 scene-body pairs。随后还可能出现：

- pair obstacles 的二重交；
- 高阶公共交；
- orientation interval 上的临界事件；
- free-space arrangement 的组合更新。

原始 3DGS 中，大量 splats 对一个具体机器人是规划冗余的。例如：

- 同一面墙上的密集 splats 产生近乎重复的约束；
- 离机器人可活动高度很远的 splats 对地面机器人无关；
- 宽阔区域的高频渲染细节不会改变 connectivity；
- 只有接近门框、窄通道、转角和闭合阈值的几何需要高分辨率保留。

因此上一版虽然正确方向清楚，但计算目标仍然是：

\[
\text{compile all rendering primitives exactly}.
\]

这是不必要的。

### 1.3 为什么不采用“五个 learned priors”

曾考虑的五个学习对象包括：

1. robot embodiment prior；
2. local connection prior；
3. physical geometry prior；
4. topology prior；
5. mobility-preserving Gaussian basis。

如果全部独立学习，会造成：

- 模块堆叠；
- 训练标签不一致；
- 错误逐层传播；
- 很难通过 ablation 判断贡献来源；
- 数学主干被黑盒模块淹没；
- reviewer 可以合理质疑“为什么需要这么多网络”。

新版将其收缩为：

\[
\boxed{\text{one learned representation} + \text{one certified compiler}.}
\]

---

## 2. 新版科学问题

### 2.1 核心问题

> **一个面向渲染、通常高度过完备的 3DGS，能否被学习性地压缩为一个机器人条件化的最小 Gaussian 表示，在显著减少 primitive 数量的同时，保持该机器人的稳健可达性、关键窄通道与通道闭合事件？**

英文：

> **Can an overcomplete rendering-oriented 3DGS be distilled into a minimal robot-conditioned Gaussian representation that preserves robust mobility connectivity and topology-critical passages?**

### 2.2 核心假设

\[
\boxed{
\text{Rendering-complete geometry}
\supsetneq
\text{Mobility-sufficient geometry for robot }r.
}
\]

即：完整重建与渲染需要保留的几何信息，严格多于某个具体机器人完成全部规划查询所需的信息。

对于不同机器人：

\[
\widetilde{\mathcal G}_{r_1}
\neq
\widetilde{\mathcal G}_{r_2}.
\]

例如：

- 小型机器人可以忽略不会形成其瓶颈的大尺度细节；
- 宽机器人必须细致保留门框和 orientation gate；
- 地面机器人可忽略高空结构；
- 无人机不能忽略天花板、悬挂物和垂直 passage。

### 2.3 新版主要 claim

新版不声称“学习替代几何证明”，而是声称：

> **学习可以摊销地发现对某个机器人规划充分的最小 Gaussian 分辨率分布；认证编译器则保证压缩不会静默创造假通道，并在不确定区域自动恢复原始分辨率。**

最短表达：

> **We learn the smallest robot-conditioned Gaussian geometry that preserves robust mobility connectivity.**

---

## 3. 相对于上一版的精确修改表

| 设计项 | 上一版 v1 | 新版 v2 | 修改原因 |
|---|---|---|---|
| 完整 3DGS 的角色 | 全部 splats 直接进入 compiler | 先形成 Gaussian hierarchy，再由网络选择合法 cut | 避免渲染冗余进入组合几何 |
| Learning | 不使用，或仅作为未来 overlay | 只学习 mobility coreset | 给 learning 一个必要且统一的职责 |
| 网络输出 | 无 | hierarchy cut / partition \(\Pi_r\) | 不允许网络自由生成障碍几何 |
| Robot embodiment | 显式几何与运动参数 | 仍为显式条件输入 | v2 不额外训练 embodiment encoder |
| Physical geometry prior | 无，假设 perfect GS | 仍不加入 | 避免同时解决感知几何校准 |
| Topology prior | 认证 compiler 的结果 | 变为 coreset 的训练约束与验收条件 | 不是第二个预测网络 |
| Local motion prior | 独立 dynamics certifier | 仍使用解析 steering / local planner | 暂不加入 diffusion/flow prior |
| Pairwise compiler | 处理全部 splats | 处理 macro-primitives，必要时局部展开 | 复杂度随规划关键结构增长 |
| Refinement | 收紧几何 envelope / orientation interval | 额外增加 hierarchy-node split | 将不确定性反向定位到表示分辨率 |
| 输出 | safe/possible mobility graph | 不变 | 保留理论与安全语义 |
| 主实验 | topology correctness、thin gate、runtime | 新增 compression-topology-runtime Pareto | 直接验证学习表示是否必要 |
| 主贡献 | GS-native certified mobility compilation | learned mobility-sufficient geometry + certified compilation | 从纯 computational geometry 升级为 learning + geometry |

---

## 4. 新版唯一学习对象：Robot-Conditioned Gaussian Mobility Coreset

### 4.1 为什么不用 “basis” 作为正式名称

“Basis”通常暗示：

- 线性独立；
- 张成某个线性空间；
- 通过线性组合重建对象。

当前学习对象本质上是：

- 选择；
- 分组；
- 合并；
- 多分辨率保留；
- 在关键区域递归细分。

因此正式名称使用：

> **Robot-Conditioned Gaussian Mobility Coreset**

或更理论化地称：

> **Robot-Conditioned Gaussian Mobility Quotient**

### 4.2 原始 Gaussian hierarchy

给定：

\[
\mathcal G=\{G_i\}_{i=1}^{N},
\qquad
G_i=(\mu_i,\Sigma_i,\alpha_i,\ldots).
\]

首先确定性构建层级结构：

\[
\mathcal H(\mathcal G).
\]

每个 hierarchy node \(v\) 表示一组叶子 splats：

\[
I(v)\subseteq\{1,\ldots,N\}.
\]

父节点是若干子节点的并。每个 node 存储：

- spatial bounding volume；
- conservative support envelope；
- covariance eigenvalue statistics；
- surface normal / anisotropy statistics；
- opacity 与 density 汇总；
- descendant count；
- 与机器人尺度相关的 normalized features。

### 4.3 网络输出是合法 hierarchy cut

学习模块：

\[
\boxed{
\Pi_r=\mathcal B_\theta(\mathcal H(\mathcal G),r)
}
\]

输出：

\[
\Pi_r=\{v_1,\ldots,v_K\}
\]

并满足：

1. 每个原始 splat 被且只被一个选中 node 覆盖；
2. 一个 node 被选中后，其祖先和后代不能同时被选中；
3. cut 可以局部细化：用 children 替换某个 node；
4. 最细 cut 等于全部叶子 splats，因此存在确定性 fallback。

网络不直接输出任意的：

\[
\widetilde \mu_k,\widetilde \Sigma_k,\widetilde \alpha_k.
\]

这样避免网络缩小障碍、移动门框或制造 false-safe passage。

### 4.4 Coreset 的理想优化定义

令 \(\mathcal M(\mathcal G,r)\) 表示完整场景对机器人 \(r\) 的 mobility structure。理想 coreset 为：

\[
\boxed{
\Pi_r^\star
=
\arg\min_{\Pi\in\operatorname{Cuts}(\mathcal H(\mathcal G))}
|\Pi|
\quad
\text{s.t.}
\quad
d_{\mathrm{mob}}
\bigl(
\mathcal M(\mathcal G,r),
\mathcal M(\Pi,r)
\bigr)
\le \varepsilon.
}
\]

神经网络的作用是摊销地近似这一昂贵组合优化：

\[
\mathcal B_\theta(\mathcal G,r)\approx\Pi_r^\star.
\]

---

## 5. Certified Macro-Primitive：learning 决定分组，geometry 决定形状

对于 cut 中的 node \(v_k\)，其原始 obstacle union 为：

\[
E(v_k)=\bigcup_{i\in I(v_k)}E_i.
\]

确定性模块构造：

\[
\boxed{
E_k^{-}
\subseteq
E(v_k)
\subseteq
E_k^{+}.
}
\]

其中：

- \(E_k^{-}\)：障碍物内包络；
- \(E_k^{+}\)：障碍物外包络；
- 二者间 gap 表示由压缩引入的几何不确定性。

压缩表示为：

\[
\widetilde{\mathcal G}_r^{-}=\{E_k^{-}\}_{k=1}^{K},
\qquad
\widetilde{\mathcal G}_r^{+}=\{E_k^{+}\}_{k=1}^{K}.
\]

对机器人 body supports 做 Minkowski construction 后，单调性给出：

\[
O_{kj}^{-,\theta}
\subseteq
O_{kj}^{\theta}
\subseteq
O_{kj}^{+,\theta}.
\]

因此整体 collision/free-space 继续满足：

\[
C^{-}\subseteq C^\star\subseteq C^{+},
\]

\[
F^{\mathrm{safe}}
=
Q\setminus C^{+}
\subseteq
F^\star
\subseteq
Q\setminus C^{-}
=
F^{\mathrm{possible}}.
\]

这一步保证：**learned coarsening 只能增加 UNKNOWN，不能未经认证地产生 SAFE。**

---

## 6. 认证编译器：上一版中保留不变的数学主干

新版不替换以下链条：

\[
\boxed{
\text{macro supports}
\rightarrow
\text{pairwise support functions}
\rightarrow
\text{inner/outer C-obstacles}
\rightarrow
\text{contact nerve + free-space arrangement}
\rightarrow
\text{certified orientation slabs}
\rightarrow
\mathcal M^{\mathrm{safe}},\mathcal M^{\mathrm{possible}}
}
\]

### 6.1 Obstacle nerve 与 free-space arrangement 仍严格分开

固定朝向 \(\theta\)：

\[
K_\theta
=
\operatorname{Nerve}\bigl(\{O_a^\theta\}\bigr)
\]

用于表示 obstacle union 的组合拓扑。

规划对象仍由：

\[
D_\theta
=
\operatorname{ConnectedCells}\left(
W\setminus\bigcup_aO_a^\theta
\right)
\]

显式构造。

不能重新退回“nerve 自动给出 free-space dual”的旧错误。

### 6.2 Safe / possible mobility graphs

分别对外障碍和内障碍编译：

\[
\boxed{
\mathcal M_r^{\mathrm{safe}}
\subseteq
\mathcal M_r^\star
\subseteq
\mathcal M_r^{\mathrm{possible}}.
}
\]

查询逻辑：

\[
\operatorname{status}(s,g)=
\begin{cases}
\texttt{REACHABLE},
& s\leftrightarrow g\ \text{in }\mathcal M_r^{\mathrm{safe}},\\
\texttt{UNREACHABLE},
& s\not\leftrightarrow g\ \text{in }\mathcal M_r^{\mathrm{possible}},\\
\texttt{REFINE},
& \text{otherwise}.
\end{cases}
\]

### 6.3 Path lifting 与最终复验

只有 safe graph 中的边可以用于最终路径：

\[
P_{\mathcal M}
\rightarrow
\tau_{s\to g}
\rightarrow
\operatorname{VerifyOnOriginalGS}(\tau).
\]

最终验证使用原始 leaf-level supports，而不是只使用 macro-primitives。

---

## 7. 新增的关键机制：Ambiguity-Driven Coreset Refinement

上一版 refine 的对象主要是：

- support directions；
- convex envelope accuracy；
- orientation interval；
- event root bracket。

新版新增一种更高层 refine：**表示细化**。

假设当前 cut 为 \(\Pi_r^{(k)}\)，safe/possible graphs 对查询不一致。系统提取导致不确定性的 macro nodes：

\[
\mathcal U^{(k)}
=
\operatorname{TraceAmbiguity}
\left(
\mathcal M^{\mathrm{safe}},
\mathcal M^{\mathrm{possible}},
\text{certificates}
\right).
\]

然后只展开这些 nodes：

\[
\boxed{
\Pi_r^{(k+1)}
=
\operatorname{Split}
\left(
\Pi_r^{(k)},
\mathcal U^{(k)}
\right).
}
\]

重新编译受影响的 local pairs、arrangements 与 slabs，而不是重建全局。

因为 hierarchy 最底层是全部原始 splats：

\[
\Pi_r^{(k)}\rightarrow\Pi_{\mathrm{leaf}}
\]

最终可回退到上一版完整确定性 compiler。因此学习模块的失败模式应是：

> **压缩率下降或 refinement 次数增加，而不是输出错误安全路径。**

---

## 8. Learning Architecture：只预测“保留什么分辨率”

### 8.1 输入

#### Scene hierarchy node features

每个 hierarchy node \(v\) 的特征可包括：

\[
x_v=
[
\text{center},
\text{extent},
\text{covariance spectrum},
\text{anisotropy},
\text{normal dispersion},
\text{opacity statistics},
\text{child count},
\text{depth}
].
\]

#### Robot descriptor

v2 不单独学习 embodiment prior。使用显式 descriptor：

\[
r=
[
\text{body supports},
\text{footprint scale},
\text{height},
\text{orientation DOF},
\text{motion-model tag}
].
\]

v0 若先做 holonomic SE(2)，motion-model tag 可以固定。

### 8.2 网络结构

推荐第一版采用：

- tree GNN / hierarchical Transformer；
- robot token 对 node features 做 FiLM 或 cross-attention；
- 每个 node 输出：
  \[
  p_v^{\mathrm{keep}},
  \quad
  p_v^{\mathrm{split}};
  \]
- top-down decoding 保证输出合法 cut。

### 8.3 不建议的输出形式

以下设计不进入 v2：

- 直接生成新的自由 Gaussian parameters；
- 直接预测 mobility graph；
- 直接预测 topology event；
- 直接生成 \(V_g\)；
- 直接生成 trajectory；
- 用 diffusion 生成全局路线。

这些都会破坏“one learned representation + one certified compiler”的统一性。

---

## 9. 训练目标：Topology prior 是约束，不是第二个网络

### 9.1 总目标

第一版建议：

\[
\boxed{
\mathcal L
=
\lambda_{\mathrm{split}}\mathcal L_{\mathrm{split}}
+
\lambda_{\mathrm{gap}}\mathcal L_{\mathrm{gap}}
+
\lambda_{\mathrm{mob}}\mathcal L_{\mathrm{mob}}
+
\lambda_{\mathrm{budget}}\mathcal L_{\mathrm{budget}}.
}
\]

### 9.2 Teacher split supervision

离线使用完整 compiler 或局部 exact tests，为 hierarchy node 生成标签：

- `MERGE_SAFE`：合并后不改变训练尺度上的 robust mobility；
- `MUST_SPLIT`：合并会删除/创造关键 passage，或造成过大 certificate gap；
- `UNCERTAIN`：继续 exact evaluation。

\[
\mathcal L_{\mathrm{split}}
=
\operatorname{BCE}(p_v^{\mathrm{split}},y_v).
\]

### 9.3 Certificate-gap loss

鼓励网络选择能产生窄 inner/outer gap 的 cut：

\[
\mathcal L_{\mathrm{gap}}
=
\sum_{v\in\Pi_r}
\operatorname{Gap}(E_v^{-},E_v^{+}).
\]

也可以在 pose samples 上定义：

\[
\mathcal L_{\mathrm{gap}}
=
\mathbb E_q
\left[
 d_r^{+}(q)-d_r^{-}(q)
\right].
\]

### 9.4 Mobility consistency loss

对完整场景与压缩表示比较：

- connected-component labels；
- passage existence；
- critical orientation intervals；
- closure thresholds；
- robust persistence features。

形式上：

\[
\mathcal L_{\mathrm{mob}}
=
 d_{\mathrm{mob}}
 \left(
 \mathcal M(\mathcal G,r),
 \mathcal M(\Pi_r,r)
 \right).
\]

第一版无需强求全流程可微。可以使用：

- teacher labels；
- contrastive ranking；
- node-level criticality classification；
- straight-through hierarchy decisions。

### 9.5 Compression budget

\[
\mathcal L_{\mathrm{budget}}
=
\frac{|\Pi_r|}{N_E}.
\]

目标不是最少 splats，而是 mobility fidelity 与复杂度的 Pareto 最优。

---

## 10. 数据生成方式

### 10.1 不需要 trajectory demonstrations

训练样本为：

\[
\left(
\mathcal G,
 r,
 \Pi_r^\star\ \text{or node labels},
 \mathcal M^\star,
 \Theta_{\mathrm{critical}}
\right).
\]

标签来自完整、未压缩的解析 compiler，而不是 RL 或 human demonstrations。

### 10.2 推荐数据课程

1. **程序化 2D/SE(2) toy scenes**：单门、双门、U-shape、keyhole；
2. **连续 morphology sweeps**：robot width、aspect ratio；
3. **Gaussian densification variants**：同一物理场景的不同 splat 数量；
4. **不同 robot scales**：验证 robot-conditioned cut；
5. **中型 indoor layouts**：走廊、房间、窄门网络。

### 10.3 最重要的训练对照

同一个物理场景，生成多种 3DGS parameterization：

\[
\mathcal G^{(1)},\mathcal G^{(2)},\ldots
\]

要求网络输出的 mobility coreset 在规划意义上稳定，而不是依赖具体 densification pattern。

---

## 11. 新版模块设计

### M0 — Gaussian Support & Hierarchy Builder

**输入**：原始 3DGS。  
**输出**：finite supports 与 hierarchy \(\mathcal H(\mathcal G)\)。  
**性质**：确定性、可缓存、与 goal 无关。

### M1 — Robot-Conditioned Mobility Coarsener

**输入**：hierarchy 与显式 robot descriptor。  
**输出**：合法 hierarchy cut \(\Pi_r\)。  
**性质**：唯一 learning module。

### M2 — Certified Macro-Primitive Constructor

**输入**：\(\Pi_r\)。  
**输出**：\(\widetilde{\mathcal G}_r^-\)、\(\widetilde{\mathcal G}_r^+\) 及 gap certificates。  
**性质**：确定性；必须满足集合嵌套。

### M3 — Certified Mobility Compiler

内部仍包含：

- pair support oracle；
- pair pruning；
- convex sandwich；
- obstacle nerve；
- free-space arrangement；
- orientation slab certification；
- safe/possible mobility graph gluing。

### M4 — Ambiguity Tracer & Coreset Refiner

**输入**：双图不一致证据。  
**输出**：应 split 的 hierarchy nodes。  
**性质**：可以完全解析实现；不需要第二个网络。

### M5 — Query, Path Lifting & Verification

**输入**：\((s,g)\) 与 safe mobility graph。  
**输出**：连续轨迹或 `UNKNOWN`。  
**性质**：最终在原始 leaf supports 上验证。

### M6 — Optional Dynamics Certifier（后置）

v2 主线先保持 holonomic SE(2)。一般 nonholonomic dynamics 作为后置 directed-edge certifier，不进入 coreset 学习核心。

---

## 12. 更新后的端到端算法

```text
ALGORITHM: LEARNED_CERTIFIED_MOBILITY_COMPILE(G, robot r, query s,g)

1. H <- BUILD_GAUSSIAN_HIERARCHY(G)
2. Pi <- COARSENER_theta(H, r)

3. repeat
4.     (Gminus, Gplus, cert_repr) <- BUILD_CERTIFIED_MACROS(H, Pi)
5.     Msafe     <- COMPILE_MOBILITY(Gplus, r)   # outer obstacles -> safe free space
6.     Mpossible <- COMPILE_MOBILITY(Gminus, r)  # inner obstacles -> possible free space

7.     if CONNECTED(Msafe, s, g):
8.         P <- GRAPH_SEARCH(Msafe, s, g)
9.         tau <- LIFT_WITH_WITNESSES(P)
10.        if VERIFY_ON_ORIGINAL_GS(tau, G, r):
11.            return REACHABLE, tau, certificates
12.        else:
13.            U <- TRACE_FAILED_WITNESS_TO_HIERARCHY(tau, cert_repr)

14.    else if NOT CONNECTED(Mpossible, s, g):
15.        return UNREACHABLE, separation_certificate

16.    else:
17.        U <- TRACE_GRAPH_AMBIGUITY_TO_HIERARCHY(Msafe, Mpossible, cert_repr)

18.    if U is empty or Pi is leaf cut:
19.        return UNKNOWN, unresolved_certificate

20.    Pi <- SPLIT_NODES(Pi, U)
```

关键性质：

- learning 只产生初始 cut；
- 所有 SAFE 答案来自外障碍构造的 safe graph；
- 所有路径最终在完整 3DGS 上复验；
- ambiguity 会推动局部 cut 展开；
- 最坏情况退化为上一版完整 compiler。

---

## 13. Theory Gate 如何从上一版升级

上一版的 compiler gates T0-T4 保留；新版前置增加 compression gates C0-C3。

### C0 — Hierarchy Cut Validity

证明/测试：

- cut 完整覆盖所有 leaves；
- 无 ancestor-descendant 冲突；
- local split 后仍是合法 cut。

### C1 — Macro Geometry Sandwich

对每个 node：

\[
E_v^{-}
\subseteq
\bigcup_{i\in I(v)}E_i
\subseteq
E_v^{+}.
\]

必须有自动 property tests。

### C2 — Mobility Sandwich Monotonicity

证明：

\[
\mathcal M^{\mathrm{safe}}(\Pi)
\subseteq
\mathcal M^\star
\subseteq
\mathcal M^{\mathrm{possible}}(\Pi).
\]

这里的图嵌套应理解为可达结论的上下界，而不是要求图节点一一同构。

### C3 — Refinement Convergence

如果每次 split 都减少 macro uncertainty，且最终允许到 leaf cut，则：

\[
\mathcal M^{\mathrm{safe}}_k
\uparrow
\mathcal M^\star,
\qquad
\mathcal M^{\mathrm{possible}}_k
\downarrow
\mathcal M^\star
\]

至少在有限离散 hierarchy 与 exact leaf compiler setting 下成立。

### 保留的 T0-T4

- T0：pair obstacle correctness；
- T1：inner/outer pair sandwich；
- T2：fixed-slice nerve + arrangement correctness；
- T3：event-free orientation slab certification；
- T4：mobility gluing、path lifting 与 witness correctness。

---

## 14. 实验设计必须怎样改变

### 14.1 新主图：Compression–Topology–Runtime Pareto

横轴：

\[
\text{compression ratio}=|\Pi_r|/N_E.
\]

纵轴至少包括：

- mobility connectivity accuracy；
- gate recall；
- closure-threshold error；
- certified query rate；
- compiler build time；
- pair/intersection/event counts。

### 14.2 新 baselines

除上一版的规划 baselines 外，必须加入 coreset baselines：

- random pruning；
- opacity pruning；
- uniform spatial clustering；
- covariance-agnostic clustering；
- rendering-oriented GS compression；
- geometry-only coarsening；
- non-robot-conditioned learned coreset；
- full uncompressed compiler。

### 14.3 Robot-conditioning test

对同一场景改变：

- robot width；
- aspect ratio；
- height；
- orientation freedom。

检查：

1. cut 是否变化；
2. critical regions 是否被不同程度保留；
3. topology fidelity 是否提高；
4. non-conditioned baseline 是否失败。

### 14.4 Thin-gate stress test

令合法 orientation interval：

\[
\delta\rightarrow0^+.
\]

测量：

- 在何种 \(\delta\) 下粗 coreset 直接认证；
- 在何种 \(\delta\) 下进入 REFINE；
- refinement 后是否恢复真实结果；
- 是否出现 false-safe；
- 为保留 passage 最终需要多少 leaves。

### 14.5 Learning 的成功标准

学习模块必须带来：

\[
\text{same certified connectivity}
\quad\text{with}\quad
\text{fewer primitives / predicates / events}.
\]

不能只展示：

- 网络 loss 更低；
- 视觉 reconstruction 更好；
- splat 数变少。

---

## 15. 与现有路线的新版区分

### Costmap / occupancy

\[
(\mathcal G,r)\rightarrow c(q)\rightarrow\text{dense/search planning}.
\]

它学习或构造一个 pose-wise scalar field。

### PNO / direct value learning

\[
(c,g)\rightarrow V_g.
\]

它学习目标条件化的 planning solution。

### Flow / diffusion trajectory

\[
(\mathcal G,r,s,g)\rightarrow\tau\ \text{distribution}.
\]

它生成一次 query 的动作或轨迹。

### HRM-like analytic roadmap

\[
\text{analytic obstacles}\rightarrow\text{fixed/random slices}\rightarrow\text{roadmap}.
\]

它使用人工或解析几何直接构造 roadmap。

### 新版 SPLATC

\[
\boxed{
(\mathcal G,r)
\rightarrow
\text{minimal mobility-sufficient Gaussian geometry}
\rightarrow
\text{certified mobility structure}
\rightarrow
\text{all }(s,g)\text{ queries}.
}
\]

最清楚的区别：

> **其他 learning 方法学习规划答案；我们学习在不改变稳健可达性的前提下，规划到底需要保留多少场景几何。**

---

## 16. 新版风险与 No-Go 条件

### 风险 R1：不存在显著可压缩性

若保持 topology 需要保留几乎全部 splats：

\[
|\Pi_r|/N_E\approx1,
\]

则 learning coreset 没有计算价值。

**No-Go**：在主要场景族上，learned coreset 相比确定性 geometry-only coarsening 无显著压缩或加速。

### 风险 R2：Macro outer envelope 过度保守

大量 groups 的 \(E^+\) 过大，导致 safe graph 长期断开。

**缓解**：

- orientation-aware macro supports；
- multi-convex macro representation；
- ambiguity-driven splitting；
- 不要求单个 group 只能对应一个椭圆。

### 风险 R3：训练标签成本过高

完整 compiler 生成 teacher labels 可能很贵。

**缓解**：

- 从 toy / medium scenes 开始；
- 只标注 hierarchy merge 是否破坏局部 robust connectivity；
- 使用 active learning 优先标注模型不确定 nodes；
- 缓存跨机器人和 morphology sweep 的 compiler 结果。

### 风险 R4：网络只学到简单尺寸阈值

如果 learned model 与 hand-crafted size/clearance rule 等价，学习贡献不足。

**检验**：与 geometry-only adaptive coarsening 正面对比，尤其测试：

- anisotropic passage；
- orientation-dependent closure；
- 多 body components；
- 同一局部尺寸但全局 topology 不同的场景。

### 风险 R5：Mobility consistency 不可微

这不是致命问题。v2 不要求 compiler 端到端可微。可以采用：

- offline teacher supervision；
- discrete cut policy；
- imitation of optimal/near-optimal cuts；
- reinforcement over compiler cost，但不以安全为 reward；
- straight-through 或 policy-gradient 仅用于 cut selection。

### 风险 R6：Learning 没有提升 novelty

若最终方法只相当于“先做普通 GS compression，再跑旧 compiler”，则贡献不足。

必须证明：

\[
\text{robot-conditioned mobility supervision}
>
\text{rendering/geometry-only compression}
\]

在 topology fidelity 与 compiler acceleration 上均有残余优势。

---

## 17. 推荐实现路线

### Phase 0 — 保留并冻结上一版 compiler

先确保上一版的 T0-T4 deterministic core 有可运行 reference implementation。它同时承担：

- teacher generator；
- correctness oracle；
- leaf-level fallback；
- final verifier。

### Phase 1 — Deterministic hierarchy 与 macro sandwich

不训练网络，先完成：

- hierarchy；
- node inner/outer supports；
- local split；
- coreset cut 的 deterministic API；
- dual mobility graphs 的增量更新。

### Phase 2 — Hand-crafted coarsening baseline

实现：

- spatial threshold；
- covariance similarity；
- opacity/size pruning；
- robot-scale heuristic；
- ambiguity-driven split。

这一阶段决定是否真的存在 mobility compression signal。

### Phase 3 — Teacher cut generation

对小中型场景搜索近似最优 cut：

\[
\min |\Pi|
\quad\text{s.t. robust mobility preserved}.
\]

可以使用 greedy merge / split、beam search 或 dynamic programming on tree。

### Phase 4 — Learned coarsener

训练 \(\mathcal B_\theta\) 模仿 teacher cut，并加入 certificate-gap 与 budget loss。

### Phase 5 — Query-driven refinement

将 compiler ambiguity 反向映射到 hierarchy nodes，实现局部 split 与局部重编译。

### Phase 6 — Scaling and generalization

最后才进入：

- 大型 indoor 3DGS；
- 不同 densification；
- unseen robot sizes；
- GPU hierarchy encoding；
- incremental scene update。

---

## 18. 新版最小可发表结果

第一篇不必同时解决真实 noisy 3DGS、任意机器人和复杂 dynamics。最小可发表 setting 可以是：

- perfect/static 3DGS-derived 2D supports；
- rigid robot；
- holonomic SE(2)；
- learned robot-conditioned hierarchy cut；
- certified inner/outer macro geometry；
- event-driven mobility compiler；
- ambiguity-driven fallback；
- thin-gate、morphology sweep 和 large-layout benchmarks。

最小论文主张：

> **A robot-conditioned learned Gaussian coreset can remove most rendering-oriented primitives while preserving all mobility features above a prescribed robustness scale; a certified compiler detects ambiguity and locally restores resolution, preventing silent false-safe connectivity.**

中文：

> **机器人条件化的 learned Gaussian coreset 能删除大部分与规划无关的渲染 primitives，同时保持给定稳健尺度以上的移动连通结构；认证编译器会检测压缩造成的不确定性并局部恢复分辨率，从而避免静默产生虚假安全通道。**

---

## 19. 最终决策记录

### 保留

- pairwise configuration obstacle；
- support-function geometry；
- inner/outer certification；
- nerve + arrangement；
- critical orientation slabs；
- safe/possible mobility graph；
- three-way query；
- path witness 与原始 GS verification。

### 删除

- 五个独立 learned priors；
- direct topology predictor；
- learned event scheduler 作为核心贡献；
- direct value-field generation；
- global diffusion/flow planning；
- 网络自由生成 macro Gaussian geometry。

### 新增

- deterministic Gaussian hierarchy；
- robot-conditioned mobility coarsener；
- certified macro inner/outer supports；
- representation-level ambiguity tracing；
- hierarchy-node refinement；
- compression-topology-runtime Pareto evaluation。

### 延后

- physical geometry prior；
- learned embodiment encoder；
- diffusion/flow local maneuver prior；
- general nonholonomic robots；
- real noisy 3DGS calibration；
- SE(3) articulated planning。

---

## 20. 最终统一表达

### 一句话方法

> **我们学习一个机器人条件化的 Gaussian mobility coreset，仅保留维持稳健可达性和关键通道所需的最小场景几何，再通过双侧认证的解析编译器构造可复用的全局 mobility structure。**

### 公式

\[
\boxed{
(\mathcal G,r)
\xrightarrow{\text{learned mobility coarsening}}
\Pi_r
\xrightarrow{\text{certified macro geometry}}
(\widetilde{\mathcal G}_r^{-},\widetilde{\mathcal G}_r^{+})
\xrightarrow{\text{certified mobility compilation}}
(\mathcal M_r^{\mathrm{safe}},\mathcal M_r^{\mathrm{possible}})
\xrightarrow{(s,g)}
\tau\ \text{or REFINE}.
}
\]

### 与上一版最本质的差异

上一版：

\[
\text{完整 3DGS}
\rightarrow
\text{认证编译器}.
\]

新版：

\[
\boxed{
\text{完整 3DGS}
\rightarrow
\text{学习“规划需要保留什么”}
\rightarrow
\text{认证编译器}.
}
\]

### 设计哲学

> **Learning determines representation resolution; geometry determines truth.**

