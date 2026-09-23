# GMC 严格 3DGS-native UAV 高空目标规划实现方案（核验修订版）

> **版本**：Verified v2  
> **目标仓库**：`https://github.com/Adeline-GuWenlan/gmc`  
> **目标**：在不把场景投影到二维、不把 3DGS 转成 mesh / voxel occupancy / ESDF 的前提下，直接利用完整三维 Gaussian primitives，为 UAV 规划能够改变高度并到达高空目标的路径。  
> **工程原则**：保留当前 GMC 的三值认证、pair identity、inner/outer sandwich、query-refine 与独立连续验证；冻结现有二维实现为参考后端，新增真正的三维 Gaussian C-obstacle 与 aerial mobility backend。

---

# 0. 核验结论

## 0.1 对上一版方案的判断

上一版方案**不是完全错误**，但只能称为：

> **使用 3D Gaussian collision oracle 的 adaptive-octree UAV planner。**

它的局部几何部分是 GS-native 的：

- scene 与 UAV 都由三维 Gaussian supports 表示；
- pair collision 直接由 mean、full anisotropic covariance 和 Gaussian identity 定义；
- 未先提取 scene mesh；
- 未构造 ESDF；
- 最终路径计划由三维 Gaussian pair oracle 验证。

但它仍有四个关键缺口，因此不能直接称为严格的 **3D GMC / SPLATC planning**：

| 检查项 | 上一版状态 | 问题 | 本版修正 |
|---|---|---|---|
| 场景是否完整保留为 3DGS | 通过 | 无二维投影 | 保留 |
| collision 是否直接来自 Gaussian pairs | 通过 | hard-support 语义未与当前代码明确锁定 | 锁定为当前 GMC 的 calibrated iso-density hard supports |
| global mobility 是否由 Gaussian combinatorics 产生 | **未通过** | 主要由 generic octree cell adjacency 产生，弱化了 Gaussian Contact Complex 主线 | 主后端改为 Gaussian pair C-obstacle arrangement / free-volume dual；octree 仅作 fallback 与 reference |
| free-space 是否只由 3DGS 决定 | 条件通过 | `W_safe` 若来自 camera-ray carving，就是额外输入，不是纯 GS core | 分成 `geometry-complete strict mode` 与 `deployment safety augmentation` 两个合同 |
| yaw 维度是否严格连续 | 部分通过 | “多个 3D 图 + yaw gluing”未形成完整 product-space certificate | 改为 interval-wide swept C-obstacle 与 `free volume × yaw interval` product cells |
| 最终证书是否回到原始 Gaussians | 部分通过 | 文档写了 continuous verification，但与 octree/polytope 的边界还不够严格 | 明确所有 `REACHABLE` 必须对原始 3D Gaussian supports 逐 pair 连续验证 |

因此，本版的正式结论是：

\[
\boxed{
\text{上一版可以作为 3D 实现 bootstrap，不能作为最终严格 GMC 后端。}
}
\]

最终正式方法必须是：

\[
\boxed{
\text{full 3DGS}
\rightarrow
\text{pairwise 3D Gaussian C-obstacles}
\rightarrow
\text{Gaussian-induced free-volume complex}
\rightarrow
\text{3D/4D mobility graph}
\rightarrow
\text{continuous Gaussian verification}.
}
\]

---

# 1. “严格通过 3DGS 做 planning”的操作性定义

本项目中，只有同时满足以下条件，才能称为 **strictly GS-native planning**。

## 1.1 输入层不降维

场景必须保留：

\[
\mathcal G_E
=
\{G_i^E=(\mu_i,\Sigma_i,\alpha_i,\kappa_i,id_i)\}_{i=1}^{N_E},
\]

其中：

- \(\mu_i\in\mathbb R^3\)；
- \(\Sigma_i\in\mathbb S_{++}^{3}\) 是完整三维 covariance；
- `id_i` 必须保留；
- 不允许只保留 Gaussian center；
- 不允许将 \(\Sigma_i\) 压成二维 covariance；
- 不允许将场景先变成 BEV、height map、mesh、point occupancy、voxel map 或 ESDF 后再规划。

## 1.2 碰撞定义必须来自 Gaussian primitive

每个 scene Gaussian 与 robot Gaussian 直接诱导一个配置障碍：

\[
( G_i^E,G_j^R )
\longrightarrow
\mathcal O_{ij}(q).
\]

全局障碍不是由 voxel label 构造，而是：

\[
\mathcal O(q)=\bigcup_{i,j}\mathcal O_{ij}(q).
\]

## 1.3 全局结构必须可追溯到 Gaussian pair

最终 mobility node、gate、portal、free-volume boundary 或不确定区域必须能够追溯到：

```text
scene Gaussian id
robot body Gaussian id
yaw interval
support directions / support planes
inner/outer approximation error
```

若一个 graph edge 只知道“经过了 voxel 1234”，却无法解释由哪些 Gaussian pairs 形成边界，那么它不能作为 GMC 的正式 global representation。

## 1.4 中间多面体允许存在，但不能是 scene mesh conversion

允许构造：

- pair C-obstacle 的 certified inner/outer convex polytopes；
- Gaussian-induced arrangement；
- free-space complement 的 tetrahedral/cellular decomposition；
- BVH；
- route graph。

因为这些是**配置空间证书**，由 Gaussian support function 直接生成，不是把场景重建为 mesh。

不允许把 surface mesh / ESDF 当作碰撞真值或规划 substrate。

## 1.5 最终安全必须回到原始 Gaussian supports

即使搜索发生在 free-volume complex 或辅助 routing graph 上，最终每段连续轨迹都必须重新调用：

\[
\operatorname{Verify}_{\mathcal G_E,\mathcal G_R}(\tau),
\]

而不是只检查：

- octree cell 是否标成 free；
- polytope 是否相交；
- graph edge 是否预先缓存为 safe。

## 1.6 辅助 octree 的合法边界

Octree 可以用于：

- dense/reference ground truth；
- unresolved arrangement 的 fallback；
- query-local witness routing；
- debug visualization；
- performance baseline。

但最终论文若声称的是 GMC / Gaussian mobility complex，octree 不能取代 Gaussian-induced global complex 成为核心方法。

---

# 2. 当前代码真实状态

当前仓库的可执行对象是二维的：

```text
GaussianSupport2D
Pose2(x, y, theta)
RobotModel2D
SceneModel2D
PairOracle with directions in R2
Shapely fixed-theta polygon union/complement
SE(2) mobility graph
```

它已经具备应当保留的认证资产：

- hard-support Gaussian semantics；
- exact pair support values / support points；
- independent Perram–Wertheim overlap oracle；
- certified inner/outer convex sandwich；
- `F_safe ⊆ F_true ⊆ F_possible`；
- orientation interval subdivision；
- `REACHABLE / UNREACHABLE / UNKNOWN`；
- SAFE witness 与 independent continuous replay；
- budget、provenance、failure-as-data。

但当前代码明确不是：

- raw/full 3DGS loader；
- 三维 scene/robot geometry；
- 三维 free-volume arrangement；
- \(\mathbb R^3\times S^1\) UAV planner；
- UAV dynamics planner。

因此升级策略不是把所有 `(2,)` 改成 `(3,)`，而是：

```text
shared certification core
├── existing ground_2d backend（冻结）
└── new aerial_3d backend
    ├── 3D Gaussian pair geometry
    ├── 3D free-volume compiler
    ├── yaw-interval product complex
    └── 3D continuous verifier
```

---

# 3. Collision semantics：必须先锁定

3DGS 的 Gaussian 具有无限数学支撑。如果不定义阈值，任意两个 Gaussian 的 overlap 都非零，整个空间都无法获得“严格无碰撞”语义。

因此，严格 GS-native 不意味着“完全不做物理语义定义”，而意味着：

> **物理碰撞语义直接定义在每个 Gaussian primitive 上，而不是先转成另一张地图。**

本实现应与当前 GMC 代码保持一致，第一版只采用一种正式语义：

## 3.1 Calibrated iso-density hard support

对 scene Gaussian：

\[
E_i
=
\left\{
X:
(X-\mu_i)^\top\Sigma_i^{-1}(X-\mu_i)
\le \kappa_i^2
\right\}.
\]

对 body Gaussian：

\[
B_j
=
\left\{
X:
(X-\nu_j)^\top\Lambda_j^{-1}(X-\nu_j)
\le \rho_j^2
\right\}.
\]

其中 `kappa_i` 与 `rho_j` 是明确冻结的 collision levels。

### 重要边界

- opacity \(\alpha_i\) 不直接等于 solidity；
- spherical harmonics 不参与 collision；
- mean 与 full covariance 原样保留；
- multi-view consistency 可作为 scene-Gaussian 接受条件，但不能在 planning 内偷偷删除困难 pair；
- 若使用“平面地面 Gaussian proxy”，它必须作为显式 preprocessing 记录，并对所有 baseline 共用。

## 3.2 不在第一版混用 overlap-threshold collision

Gaussian overlap superlevel 也可以产生解析 forbidden ellipsoid，但它与 hard-support Minkowski-sum 是不同物理语义。

第一版不要一部分模块使用：

```text
iso-density hard ellipsoid intersection
```

另一部分模块使用：

```text
Gaussian product overlap threshold
```

否则 safe/possible sandwich 与 independent verifier 不再验证同一对象。

可在后续单独实现 `OverlapThresholdPairOracle3D` 作为另一种完整 backend，但不能混在同一次 certified run 中。

---

# 4. 状态空间：UAV 到达高空 goal 最小需要什么

## 4.1 Phase U0：轴对称 UAV

第一版使用球形或绕竖直轴对称的保守 UAV body：

\[
q=p=(x,y,z)\in\mathbb R^3.
\]

这一版已经能严格验证：

- 垂直起飞；
- 到达高空目标；
- 飞越低墙；
- 从桥上/桥下选择不同高度通路；
- 避开天花板与悬挂物；
- 在竖井中上升。

它是最重要的第一步，因为它首先证明系统不是二维 projection。

## 4.2 Phase U1：非轴对称 UAV

加入 yaw：

\[
q=(p,\psi)=(x,y,z,\psi)
\in\mathbb R^3\times S^1.
\]

用于：

- 长条 UAV 通过三维狭槽；
- 同一位置只有部分 yaw 安全；
- 起点和目标 yaw 不同；
- 平移与转向组合。

## 4.3 Phase U2：完整姿态与动力学

只有当 roll/pitch 会决定通道通过性时，才扩展：

\[
q=(p,R)\in SE(3).
\]

考虑动力学时：

\[
x=(p,v,R,\omega,\ldots).
\]

第一版不应直接在 full kinodynamic state 上构造全局 dense complex。

## 4.4 高空 goal

高空目标必须显式包含高度：

\[
g=(x_g,y_g,z_g)
\]

或：

\[
\mathcal Q_g
=
\{(p,\psi):\|p-p_g\|\le r_g,\ \psi\in I_g\}.
\]

若 goal 只有 `(x_g,y_g)`，就没有定义“高空”。

---

# 5. 三维 Gaussian pair C-obstacle

## 5.1 UAV body transform

MVP 中：

\[
T(q)X=R_z(\psi)X+p.
\]

body primitive `j` 的 world center 与 covariance：

\[
c_j(q)=p+R_z(\psi)\nu_j,
\]

\[
\Lambda_j(q)=R_z(\psi)\Lambda_jR_z(\psi)^\top.
\]

## 5.2 固定 yaw 的精确 translation C-obstacle

固定 \(\psi\) 后，translation \(p\) 导致 pair \((i,j)\) 碰撞的集合是：

\[
\boxed{
\mathcal O_{ij}^{\psi}
=
\mu_i-R_z(\psi)\nu_j
+
E_i^0
\oplus
\left(-R_z(\psi)B_j^0\right).
}
\]

这里：

\[
E_i^0=\{x:x^\top\Sigma_i^{-1}x\le\kappa_i^2\},
\]

\[
B_j^0=\{x:x^\top\Lambda_j^{-1}x\le\rho_j^2\}.
\]

这是三维凸体，通常不是一个椭球，但它有闭式 support oracle。

## 5.3 精确 support value

对任意三维单位方向 \(u\)：

\[
\boxed{
h_{ij}^{\psi}(u)
=
 u^\top(\mu_i-R_z(\psi)\nu_j)
+
\kappa_i\sqrt{u^\top\Sigma_i u}
+
\rho_j\sqrt{
 u^\top R_z(\psi)\Lambda_jR_z(\psi)^\top u
}.
}
\]

## 5.4 精确 support point

\[
\boxed{
x_{ij}^{\psi}(u)
=
 c_{ij}(\psi)
+
\kappa_i\frac{\Sigma_i u}{\sqrt{u^\top\Sigma_i u}}
+
\rho_j
\frac{R_z\Lambda_jR_z^\top u}
{\sqrt{u^\top R_z\Lambda_jR_z^\top u}}.
}
\]

其中旋转矩阵均取 \(R_z(\psi)\)。

这些公式是当前二维 `PairOracle` 的维度一致扩展，不需要 scene mesh、SDF 或 voxel occupancy。

---

# 6. Certified 3D pair envelopes

对每个 pair C-obstacle，不直接离散整个 workspace，而是从 support oracle 构造 certified convex sandwich。

## 6.1 球面方向集

使用逐级细化的 icosphere 方向：

\[
U=\{u_k\}_{k=1}^{K}\subset S^2.
\]

必须保存最终被 oracle 实际消费的 normalized binary64 direction rows，不能只保存角度后重新计算。

## 6.2 Outer polytope

\[
\boxed{
P_{ij}^{+,\psi}
=
\bigcap_{k}
\{x:u_k^\top x\le h_{ij}^{\psi}(u_k)+\epsilon_k^{fp}\}.
}
\]

每一个 supporting halfspace 都包含真实 convex body，因此：

\[
\mathcal O_{ij}^{\psi}
\subseteq
P_{ij}^{+,\psi}.
\]

`epsilon_fp` 只负责 directed outward numerical rounding，不负责修补错误几何。

## 6.3 Inner polytope

\[
\boxed{
P_{ij}^{-,\psi}
=
\operatorname{conv}
\{x_{ij}^{\psi}(u_k)\}_{k=1}^{K},
}
\]

因此：

\[
P_{ij}^{-,\psi}
\subseteq
\mathcal O_{ij}^{\psi}.
\]

## 6.4 必须保留的 sandwich

\[
\boxed{
P_{ij}^{-,\psi}
\subseteq
\mathcal O_{ij}^{\psi}
\subseteq
P_{ij}^{+,\psi}.
}
\]

需要有三维版的：

- nesting audit；
- exact/dyadic predicate fallback；
- scale-aware floating-point error budget；
- adaptive spherical-direction refinement；
- pair provenance。

---

# 7. 正式 global backend：Gaussian-induced free-volume complex

这是本次最重要的修正。

## 7.1 不再把 octree 作为正式 global representation

上一版：

```text
Gaussian pairs → cell classifier → octree graph
```

虽然 collision query 仍来自 Gaussian，但 global connectivity 主要由一个与 Gaussian combinatorics 无关的规则网格产生。

本版正式主线改成：

```text
Gaussian pairs
→ certified pair C-obstacle polytopes
→ obstacle union arrangement
→ workspace complement
→ free-volume components / portals
→ mobility complex
```

## 7.2 固定 yaw 下的 safe / possible obstacle union

\[
\mathcal O^-_{\psi}
=
\bigcup_{i,j}P_{ij}^{-,\psi},
\]

\[
\mathcal O^+_{\psi}
=
\bigcup_{i,j}P_{ij}^{+,\psi}.
\]

由于 pair-wise nesting：

\[
\mathcal O^-_{\psi}
\subseteq
\mathcal O_{\psi}
\subseteq
\mathcal O^+_{\psi}.
\]

在 bounded workspace \(W\) 内：

\[
\boxed{
\mathcal F^{safe}_{\psi}
=
W\setminus\mathcal O^+_{\psi},
}
\]

\[
\boxed{
\mathcal F^{possible}_{\psi}
=
W\setminus\mathcal O^-_{\psi}.
}
\]

因此：

\[
\boxed{
\mathcal F^{safe}_{\psi}
\subseteq
\mathcal F^{true}_{\psi}
\subseteq
\mathcal F^{possible}_{\psi}.
}
\]

## 7.3 三维 arrangement / complement dual

需要从 \(\mathcal F^{safe}\) 与 \(\mathcal F^{possible}\) 提取：

- connected free volumes；
- shared free facets；
- narrow portals；
- Gaussian-pair-defined boundary patches；
- component lineage。

推荐实现：

```text
certified convex pair polytopes
→ robust 3D Boolean union
→ bounded workspace difference
→ constrained tetrahedral/cellular decomposition
→ component and facet adjacency
```

关键说明：

> 这里出现的 polyhedron / tetrahedron 是配置空间证书与 routing cells，不是将 3DGS scene 转成 mesh。

每个 boundary facet 必须保存产生它的：

```text
pair_id
support direction
yaw / yaw interval
inner or outer side
numeric slack
```

## 7.4 推荐的实现后端

Python 的 Shapely 没有等价的可靠 3D Boolean backend。正式版建议增加 C++/pybind11 模块，例如：

```text
gmc_ext/arrangement3d/
    convex_polytope.cpp
    directed_rounding.cpp
    boolean_union.cpp
    workspace_complement.cpp
    free_cell_decomposition.cpp
    portal_extraction.cpp
```

几何库应支持：

- exact predicates；
- rational/dyadic coordinates 或 directed rounding；
- closed polyhedral Boolean operations；
- connected-component extraction；
- tetrahedral/cellular decomposition。

若使用非 exact backend，任何 Boolean ambiguity 必须返回 `UNKNOWN`，不能返回 free。

## 7.5 复杂度控制

不能对全部 \(N_E N_R\) pairs 全量求 union。

需要：

1. 3D scene BVH；
2. robot rotational bounding volume；
3. workspace/yaw-interval conservative candidate pruning；
4. local pair cluster；
5. incremental union tree；
6. output-sensitive update；
7. query-refine 只更新受影响区域。

但是所有 pruning 都必须有保守 bound；learning 不能直接删除未经解析界证明的 pair。

---

# 8. Axisymmetric UAV：最先实现的严格 3D 版本

对于 Phase U0：

\[
q=p\in\mathbb R^3.
\]

流程：

```text
full 3DGS scene
+ axisymmetric 3D Gaussian UAV body
→ 3D pair C-obstacles
→ certified pair polytopes
→ safe/possible obstacle unions
→ 3D free-volume dual
→ locate start and high-altitude goal
→ graph search through free-volume portals
→ continuous Gaussian verification
```

这一阶段不需要任何 orientation slicing。

它必须先通过：

- low wall overflight；
- full-height wall rejection；
- bridge above/below；
- ceiling；
- vertical shaft；
- high-altitude goal。

这是证明“完整三维 GS planning”最直接、最少混淆的一步。

---

# 9. 非轴对称 UAV：严格的 \(\mathbb R^3\times S^1\) product complex

## 9.1 不能只做若干独立 yaw slices

如果只在若干 yaw midpoint 建三维图，再用启发式连接，可能漏掉：

- 极窄 yaw gate；
- slab 内突然出现的碰撞；
- translation 与 yaw 联合变化；
- interval 边界上的错误 gluing。

因此每个 yaw interval 都必须拥有 interval-wide 几何证书。

## 9.2 Pair obstacle 的 yaw Hausdorff bound

设 interval：

\[
I=[\psi_0-\delta,\psi_0+\delta].
\]

对 body primitive `j`，定义相对 UAV yaw 轴的最大水平旋转半径：

\[
r_j^{rot}
\ge
\max_{x\in B_j}
\|P_{xy}x\|.
\]

一个保守闭式上界是：

\[
r_j^{rot}
=
\|P_{xy}\nu_j\|
+
\rho_j
\sqrt{
\lambda_{max}(P_{xy}\Lambda_jP_{xy}^\top)
}.
\]

则：

\[
d_H(
\mathcal O_{ij}^{\psi},
\mathcal O_{ij}^{\psi_0}
)
\le
2r_j^{rot}\sin\frac{|\psi-\psi_0|}{2}
\le
r_j^{rot}|\psi-\psi_0|.
\]

记：

\[
\Delta_{ij}(I)=r_j^{rot}\delta.
\]

## 9.3 Interval outer obstacle

\[
\boxed{
P_{ij,I}^{+}
=
P_{ij}^{+,\psi_0}
\oplus
B(\Delta_{ij}(I)).
}
\]

于是：

\[
\bigcup_{\psi\in I}
\mathcal O_{ij}^{\psi}
\subseteq
P_{ij,I}^{+}.
\]

## 9.4 Interval inner obstacle

\[
\boxed{
P_{ij,I}^{-}
=
P_{ij}^{-,\psi_0}
\ominus
B(\Delta_{ij}(I)).
}
\]

若 erosion 非空，则：

\[
P_{ij,I}^{-}
\subseteq
\bigcap_{\psi\in I}
\mathcal O_{ij}^{\psi}.
\]

如果无法认证 erosion，必须返回空 inner set 或继续二分 yaw interval，不能猜测。

## 9.5 Interval free volumes

\[
\mathcal F_I^{safe}
=
W\setminus
\bigcup_{ij}P_{ij,I}^{+},
\]

\[
\mathcal F_I^{possible}
=
W\setminus
\bigcup_{ij}P_{ij,I}^{-}.
\]

`safe` volume 中任意位置对 interval 内所有 yaw 都安全。

## 9.6 Product cells

对 interval `I_l` 中的 free-volume component `V_kl`，构造：

\[
\boxed{
C_{k\ell}=V_{k\ell}\times I_\ell
\subset
\mathbb R^3\times S^1.
}
\]

这才是正式 4D mobility complex 的 node。

### Spatial adjacency

同一 interval 内，两个 free volumes 通过 certified shared portal 连接。

### Yaw adjacency

相邻 intervals 只有在存在共同 spatial portal，并且固定位置 yaw sweep 经过 direct Gaussian verification 时才连接。

### Local steering

同时平移与旋转的 edge 可以加入，但必须存储实际连续 witness，并由 3D pair oracle + yaw interval bound 验证。

---

# 10. Path lifting：不能只连接 cell centers

## 10.1 Portal-based witness

free-volume graph 路径：

```text
V0 → portal01 → V1 → portal12 → ... → Vn
```

应 lift 为：

1. start 到第一 portal；
2. 每个 convex/certified free cell 内的 segment；
3. portal 间 path；
4. 最后 portal 到 goal；
5. 必要 yaw transition。

不能默认相邻 octree/tetrahedral cell centers 的直线一定留在两 cell union 内。

## 10.2 Query-local routing

在一个已认证 free component 内，可以使用：

- visibility graph；
- tetrahedral dual A*；
- local PRM；
- adaptive grid；
- funnel-like portal routing。

这些只是 witness generator。其安全性必须由 free-volume certificate与最终 direct verification共同给出。

## 10.3 Smoothing

可以对 polyline 做：

- shortcut；
- B-spline；
- minimum-snap polynomial；
- corridor optimization。

每次修改轨迹后必须重新验证，不能继承旧路径的安全标志。

---

# 11. 独立连续 3D Gaussian verification

## 11.1 Pose-level independent oracle

最终 verifier 不能只问 outer polytope 或 arrangement。

它应直接检查：

\[
T(q)B_j\cap E_i=\emptyset
\qquad
\forall i,j.
\]

推荐两套独立算法之一：

- 三维 Perram–Wertheim contact function；
- support-mapping GJK distance + conservative numerical fallback。

数值失败必须返回 unresolved / `UNKNOWN`，不能返回 collision-free。

## 11.2 Translation segment

对固定 yaw 的 segment \(p(t)\)，使用：

- pair clearance lower bound；
- clearance 对 translation 的 1-Lipschitz 性；
- conservative advancement；
- adaptive interval subdivision。

## 11.3 Yaw segment

对固定位置的 yaw transition：

\[
q(t)=(p,\psi(t)),
\]

使用：

- pair yaw Hausdorff/Lipschitz bound；
- interval midpoint clearance；
- recursive angular subdivision。

## 11.4 General local segment

对同时平移和 yaw：

\[
q(t)=(p(t),\psi(t)),
\]

一个保守 variation bound 为：

\[
\Delta_{pair}
\le
\|p(t_1)-p(t_0)\|
+
 r_j^{rot}|\psi(t_1)-\psi(t_0)|.
\]

以此递归细分，直到：

- clearance lower bound严格为正；
- 发现碰撞；
- 达到预算并返回 UNKNOWN。

## 11.5 `REACHABLE` 的硬条件

\[
\boxed{
\operatorname{GraphPathFound}
\land
\operatorname{LiftedCurveExists}
\land
\operatorname{DirectGaussianContinuousVerify}=\texttt{CERTIFIED}.
}
\]

缺一不可。

---

# 12. 真实 3DGS 中“没有 splat 是否等于 free”的处理

这是输入完备性问题，不是 planner 可以凭算法解决的问题。

## 12.1 Strict geometry-complete mode

论文的第一版正式 core 建议采用：

```text
bounded workspace W
perfect / geometry-calibrated / sufficiently complete 3DGS
all physical obstacles inside W represented by accepted Gaussian supports
```

在这一假设下：

\[
\mathcal F
=
W\setminus\bigcup_{ij}\mathcal O_{ij}
\]

可以直接定义 free configuration space。

这是真正纯 GS 输入的 core setting。

## 12.2 Real deployment augmentation

真实重建中，未观测区域不能自动视为 free。可以额外加入：

- camera-ray free-space evidence；
- depth/LiDAR；
- known flight envelope；
- online local sensing。

但必须标注为：

> **deployment safety augmentation，不是 core Gaussian mobility compiler 的输入表示。**

正式实验应分别报告：

1. GS-only geometry-complete benchmark；
2. GS + observation mask deployment benchmark。

不能将外部 free-space carving 的收益写成 GMC 自身恢复出的结果。

---

# 13. Octree 在修订方案中的正确位置

## 13.1 可以保留的用途

`PairDrivenOctree3D` 可用于：

- 小场景高分辨率 connectivity reference；
- strict arrangement 的交叉验证；
- arrangement Boolean 失败时的 query-local fallback；
- unresolved region refinement；
- visualization；
- runtime baseline。

## 13.2 不能承担的 claim

若最终 pipeline 是：

```text
3DGS → octree occupancy labels → A*
```

那么它属于：

> direct-GS collision + generic 3D grid planning，

不是 Gaussian Contact Complex / Gaussian Mobility Complex 的完整主张。

## 13.3 即便使用 octree，也必须保持的条件

- scene 不预先 voxelize；
- cell label 每次可由 Gaussian pair oracles 重放；
- 每个 cell 保存 relevant pair IDs；
- `SAFE` 要证明整个 cell free，而不是只测 center；
- `BLOCKED` 要证明整个 cell 被 obstacle 覆盖，而不是仅与 obstacle 相交；
- unresolved cell 必须保持 UNKNOWN；
- final path 必须 direct Gaussian continuous verify。

---

# 14. 3DGS loader：不得发生隐式 projection

## 14.1 输入字段

典型 3DGS 文件应读取：

```text
xyz / mean
scale parameters
rotation quaternion
opacity
primitive index
optional confidence / multi-view consistency
```

若 decoded principal scales 为 \(a_x,a_y,a_z\)，rotation 为 \(R_i\)：

\[
\boxed{
\Sigma_i
=
R_i
\operatorname{diag}(a_x^2,a_y^2,a_z^2)
R_i^\top.
}
\]

必须根据具体 PLY/checkpoint 格式正确解码 log-scale、quaternion ordering 与 normalization。

## 14.2 Round-trip audit

loader 必须输出可在真正 3D viewer 中检查的 artifact：

- Gaussian means；
- principal axes；
- support ellipsoids；
- primitive IDs；
- world coordinate frame；
- start、goal 和 UAV body。

禁止只用 XY matplotlib projection 作为 I/O 验证。

## 14.3 强制单元测试

- covariance symmetry；
- SPD；
- quaternion normalization；
- rotation determinant；
- scale units；
- world-frame transform；
- save/load round-trip；
- no dropped `z`；
- no dropped covariance off-diagonal terms。

---

# 15. 文件级实现方案

## 15.1 保留 shared core

```text
gmc/src/gmc/core/
    status.py
    result.py
    budget.py
    provenance.py
    ledger.py
```

保留：

- `CertStatus`；
- `PlanStatus`；
-三值结果；
- budget；
- failure-as-data；
- artifact bindings。

## 15.2 冻结现有二维 backend

```text
gmc/src/gmc/backends/ground_2d/
    pair_oracle_2d.py
    envelopes_2d.py
    slice_compiler_2d.py
    arrangement_2d.py
    orientation_2d.py
    mobility_2d.py
    witness_2d.py
```

现有测试和 artifacts 不应因 UAV 开发而失效。

## 15.3 新增三维 models 与 I/O

```text
gmc/src/gmc/models/
    gaussian_3d.py
    scene_3d.py
    robot_3d.py
    pose_xyz.py
    pose_xyz_yaw.py

gmc/src/gmc/io/
    gs3d_ply.py
    gs3d_npz.py
    robot3d_io.py
    workspace3d_io.py
```

核心类型：

```python
@dataclass(frozen=True)
class GaussianSupport3D:
    mean: np.ndarray          # (3,)
    covariance: np.ndarray    # (3,3), SPD
    level: float
    primitive_id: int
    opacity: float | None = None

@dataclass(frozen=True)
class PoseXYZ:
    xyz: np.ndarray           # (3,)

@dataclass(frozen=True)
class PoseXYZYaw:
    xyz: np.ndarray           # (3,)
    yaw: float

@dataclass(frozen=True)
class SceneModel3D:
    supports: tuple[GaussianSupport3D, ...]
    workspace: object

@dataclass(frozen=True)
class RobotModel3D:
    supports: tuple[GaussianSupport3D, ...]
```

## 15.4 新增 3D pair geometry

```text
gmc/src/gmc/geometry3d/
    support.py
    collision_pw.py
    collision_gjk.py
    bvh.py
    spherical_directions.py
    envelopes.py
    yaw_bounds.py
```

协议：

```python
class PairOracle3D(Protocol):
    def support_values(self, yaw: float, U: np.ndarray) -> np.ndarray: ...
    def support_points(self, yaw: float, U: np.ndarray) -> np.ndarray: ...
    def center(self, yaw: float) -> np.ndarray: ...
    def radius_bound(self) -> float: ...
    def yaw_hausdorff_bound(self, half_width: float) -> float: ...
```

## 15.5 新增严格 3D arrangement backend

```text
gmc/src/gmc/backends/aerial_3d/
    pair_candidates.py
    fixed_yaw_compiler.py
    interval_yaw_compiler.py
    obstacle_union.py
    free_volume_decomposition.py
    portal_graph.py
    product_complex.py
    query.py
    witness.py
    refine.py
```

底层 C++ extension：

```text
gmc_ext/arrangement3d/
    halfspace_polytope.cpp
    convex_hull.cpp
    exact_boolean.cpp
    workspace_difference.cpp
    connected_volumes.cpp
    tetrahedralize.cpp
    portals.cpp
```

## 15.6 新增独立 verifier

```text
gmc/src/gmc/verification/
    pose_collision_3d.py
    translation_3d.py
    yaw_rotation_3d.py
    curve_xyz_yaw.py
    uav_retiming.py
```

---

# 16. 分阶段实现路线

## Phase 0：冻结当前二维证据

- tag 当前 main；
- 保存当前 353-test 结果；
- 现有 2D backend 行为不变；
- 抽象 `PairOracle / SpatialBackend / ContinuousVerifier` 接口。

**Go**：现有全部 tests 与 artifacts 保持一致。

## Phase 1：真正的 raw/full 3DGS I/O

- 读取 mean、scale、quaternion、opacity；
- 重建完整 covariance；
- 3D viewer artifact；
- `GaussianSupport3D` round-trip；
- 3D robot Gaussian body。

**Go**：不存在 XY projection；真实 3D viewer 中 principal axes 与 source 一致。

## Phase 2：3D pair kernel

- `PairOracle3D`；
- support values；
- support points；
- 3D Perram–Wertheim；
- independent GJK cross-check；
- 3D BVH；
- spherical inner/outer envelope。

**Go**：

\[
P^-\subseteq\mathcal O\subseteq P^+
\]

在 analytic、random 和 adversarial pair tests 中成立。

## Phase 3：轴对称 UAV 的严格 R3 compiler

- fixed orientation；
- pair outer/inner union；
- workspace complement；
- free-volume components；
- portal graph；
- high-altitude goal query；
- continuous direct Gaussian verification。

**Go**：low wall、full-height wall、bridge/underpass、vertical shaft 全部通过。

## Phase 4：octree reference backend

- direct pair-cell tests；
- high-resolution connectivity reference；
- arrangement vs octree cross-check；
- 不作为正式 GMC 输出结构。

**Go**：小场景 connectivity 与 strict arrangement 一致；所有差异产生 failure artifact。

## Phase 5：R3 × S1 product complex

- yaw interval outer/inner obstacles；
- interval free volumes；
- product cells；
- cross-yaw portals；
- rotate / local-steering witness；
- continuous 4D verification。

**Go**：orientation-dependent 3D slot 与 dense 4D reference 一致。

## Phase 6：UAV dynamics

- velocity、vertical speed、yaw rate；
- acceleration、jerk、tilt、thrust；
- retiming；
- local trajectory optimizer；
- edge interface state。

**Go**：几何可行但动力学不可行的路径不再被接受。

## Phase 7：real incomplete 3DGS safety augmentation

- camera/depth observation coverage；
- online local sensing；
- receding-horizon verification；
- unknown-space abstention。

**Go**：未观测高空区域不返回 false REACHABLE。

---

# 17. 必须通过的 strictness tests

| 测试 | 失败意味着什么 |
|---|---|
| Loader 输出完整 `(3,)` means 与 `(3,3)` covariance | 若失败，仍是伪 3D / projection |
| Bridge 与 underpass 在相同 XY 下形成两个不同 z route | 若失败，global planner 仍在做二维规划 |
| Low wall 可从上方通过，full-height wall 不可穿越 | 若失败，z 维没有进入 topology |
| 高 UAV 与小 UAV 对悬空窄口得到不同 route | 若失败，3D embodiment 未进入 C-obstacle |
| 所有 graph boundary 能追溯到 Gaussian pair IDs | 若失败，global representation 退化为匿名 grid |
| 禁止 planner 读取 scene mesh / ESDF / voxel occupancy | 若失败，不是 strict GS-native core |
| Pair outer/inner nesting 可重放 | 若失败，safe/possible 不可信 |
| Graph path 每段均 direct Gaussian continuous verify | 若失败，routing surrogate 被误当成安全证书 |
| Octree resolution 改变不应改变已 certified REACHABLE 的安全性 | 若失败，安全依赖离散采样 |
| yaw interval 收缩时 UNKNOWN 单调减少 | 若失败，orientation certificate 不一致 |

---

# 18. Benchmark 与验收场景

| 场景 | 严格验证点 | 预期 |
|---|---|---|
| Open vertical ascent | `z` 真正是自由度 | 直接爬升至高空 goal |
| Low wall | 非 XY projection | 从墙上方飞越 |
| Full-height wall | 三维障碍闭合 | 绕行、UNREACHABLE 或 UNKNOWN |
| Overhang / ceiling | 悬空 obstacle | 不穿透顶部结构 |
| Bridge / underpass | 同 XY 多高度 route | 可选桥上或桥下 |
| Vertical shaft | 三维 narrow passage | 沿 shaft 上升 |
| Suspended ring | 三维洞与 yaw | 穿环或绕行 |
| Yaw-dependent slot | R3 × S1 product complex | 调整 yaw 后通过 |
| Two UAV sizes | embodiment-specific topology | 小 UAV 走捷径，大 UAV 绕行 |
| Incomplete high sky | observation boundary | deployment mode 返回 UNKNOWN |

每个结果必须保存：

```text
raw 3DGS identifiers
robot Gaussian model
pair candidate list
pair envelopes
outer/inner union
safe/possible free volumes
portal graph
product cells if yaw is enabled
lifted path
continuous verification trace
minimum clearance
budget / UNKNOWN reasons
```

---

# 19. 量化指标

## 19.1 正确性

- free-volume component precision / recall；
- route-class recall；
- false `REACHABLE`；
- false `UNREACHABLE`；
- `UNKNOWN` rate；
- continuous-verification pass rate；
- minimum direct-Gaussian clearance；
- yaw gate closure error。

## 19.2 GS-native 证据

- pair IDs involved per gate；
- anisotropic covariance ablation；
- centers-only ablation；
- isotropic covariance ablation；
- mesh/ESDF analogue baseline；
- octree-only baseline；
- fraction of graph facets traceable to Gaussian support planes；
- no-projection test coverage。

## 19.3 高空目标

- terminal altitude error；
- high-altitude goal success；
- vertical travel distance；
- wall-overflight success；
- ceiling violation count；
- bridge/underpass route correctness。

## 19.4 效率

- scene Gaussian count；
- robot Gaussian count；
- candidate pair count；
- pruned pair count；
- support evaluations；
- pair polytope facets；
- Boolean union time；
- free-volume decomposition time；
- yaw interval count；
- product-cell count；
- query time；
- verification time；
- peak memory。

---

# 20. Learning 只能在严格解析 3D baseline 后决定

完成以上 pipeline 后，再用 profiling 判断 learning 是否必要。

| 解析瓶颈 | 合理 learning 位置 | 不可被 learning 取代的部分 |
|---|---|---|
| 3D candidate pairs 太多 | pair proposal / ranking | conservative BVH fallback 与最终 pair checks |
| spherical directions/refinement 太多 | direction/refinement priority | inner/outer nesting audit |
| 3D Boolean arrangement 重复编译慢 | local arrangement warm-start / incremental prediction | final free-volume topology verification |
| yaw intervals 过多 | event proposal / slab priority | certified interval subdivision |
| local UAV trajectory optimization 慢 | trajectory warm start | optimizer convergence与continuous verifier |
| 同一 scene/UAV 大量 goals | graph heuristic / preconditioner | mobility complex 与安全证书 |

不能使用：

- direct 3DGS-to-path 代替 global topology；
- learned free-space label 直接产生 certified edge；
- 网络删除未经解析 bound 排除的 pair；
- 网络输出替代 direct Gaussian verifier。

---

# 21. 最终系统合同

## 21.1 Strict GS-native core 输入

```text
- full 3D Gaussian scene: mean + full covariance + id + fixed support level
- 3D Gaussian UAV body
- bounded workspace W
- scene-completeness assumption within W
- start: (x,y,z) or (x,y,z,yaw)
- high-altitude goal: 3D goal region
- geometric tolerances and budgets
```

## 21.2 Strict GS-native core 禁止输入

```text
- scene mesh used as planning geometry
- scene voxel occupancy used as planning geometry
- ESDF/SDF used as planning geometry
- BEV / height-map replacement of the full scene
- 2D projected Gaussian covariance
```

## 21.3 输出

```text
REACHABLE
    - product/free-volume graph path
    - continuous 3D/4D UAV trajectory
    - direct Gaussian pair clearance certificate
    - pair provenance
    - workspace certificate

UNREACHABLE
    - only with a complete possible-space cut under the declared setting

UNKNOWN
    - unresolved 3D Boolean geometry
    - insufficient direction/yaw refinement
    - exhausted budget
    - incomplete observation in deployment mode
    - continuous verifier unresolved
    - dynamics retiming unresolved
```

## 21.4 最终不变量

\[
\boxed{
\text{场景从输入到最终验证始终保留为完整 3D Gaussian primitives。}
}
\]

\[
\boxed{
\text{global mobility 的边界由 Gaussian pair C-obstacles 诱导，}
\text{而不是由匿名 voxel occupancy 决定。}
}
\]

\[
\boxed{
\text{任何 REACHABLE 都必须由原始三维 Gaussian supports 连续重放验证。}
}
\]

---

# 22. 最终工程决策

## 不推翻整个仓库

保留：

- 三值认证纪律；
- pair support abstraction；
- inner/outer sandwich；
- orientation interval思想；
- query-refine；
- path lifting + independent replay；
- budgets/provenance/artifacts。

## 但也不是简单 extension 一个维度

必须新增：

- full 3DGS loader；
- 3D robot body；
- 3D pair C-obstacle；
- spherical support envelopes；
- robust 3D obstacle union/complement；
- free-volume dual；
- R3 × S1 product complex；
- direct continuous 3D verifier；
- UAV dynamics layer。

## 最准确的一句话

\[
\boxed{
\textbf{现有 GMC 提供认证架构；}
\quad
\textbf{新后端必须从完整 3D Gaussian pair forbidden sets 直接编译三维 aerial mobility。}
}
\]

该方案支持 UAV 到达高空 goal，同时保持真正的 GS-native 主线，而不是把 3DGS 只当作生成 voxel/ESDF 的前处理输入。

---

# 参考依据

- Current repository: https://github.com/Adeline-GuWenlan/gmc
- Current 2D types: https://raw.githubusercontent.com/Adeline-GuWenlan/gmc/main/gmc/src/gmc/types.py
- Current 2D scene I/O boundary: https://raw.githubusercontent.com/Adeline-GuWenlan/gmc/main/gmc/src/gmc/io/gs_io.py
- Current pair support oracle: https://raw.githubusercontent.com/Adeline-GuWenlan/gmc/main/gmc/src/gmc/geometry/support.py
- Current independent collision oracle: https://raw.githubusercontent.com/Adeline-GuWenlan/gmc/main/gmc/src/gmc/geometry/c_obstacle.py
- Current fixed-orientation compiler: https://raw.githubusercontent.com/Adeline-GuWenlan/gmc/main/gmc/src/gmc/spatial/slice_compiler.py
- Current continuous verifier: https://raw.githubusercontent.com/Adeline-GuWenlan/gmc/main/gmc/src/gmc/verification/continuous.py
- Current conformance boundary: https://raw.githubusercontent.com/Adeline-GuWenlan/gmc/main/gmc/CONFORMANCE.md
- GMC research design: https://github.com/Adeline-GuWenlan/gmc/blob/main/05_GMC_RESEARCH_DESIGN.md
- 3D Gaussian Splatting: https://arxiv.org/abs/2308.04079
- FOCI: Trajectory Optimization on Gaussian Splats: https://arxiv.org/abs/2505.08510
