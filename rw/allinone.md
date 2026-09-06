# SplatHJB literature-search record

检索日期：2026-08-05

## 目标与边界

目标不是罗列所有包含 “Gaussian” 或 “planning” 的论文，而是寻找会支持、替代或击穿以下方法主张的 primary papers：

> 学习一个只依赖 Gaussian 所表达的连续物理 measure/field、而不依赖具体 splat 分解，并能直接求解 arbitrary-goal full-state UAV HJB feedback control 的 solution operator。

纳入四条主线：native 3DGS planning/control；value/Eikonal/HJB planning；measure/discretization-invariant operator learning；full-dynamics UAV planning/control。动态 GS 重建、纯渲染和不涉及规划的通用 3DGS 工作不作为主线。

## 检索来源

- 自动学术检索：arXiv、Semantic Scholar、Crossref、OpenReview、OpenAlex、DBLP。
- primary-source 复核：arXiv 摘要/PDF、OpenReview、Taylor & Francis 文章页。
- 开放获取状态复核：DOAJ 与出版社页面。
- 本地既有 OA 全文：`/Users/mmvc/local_work/IDEAS/UAV/papers`；复制前后重新验真。

## 主要检索式

```text
goal conditioned neural value field HJB solution operator robot motion planning
discretization invariant measure neural operator Gaussian basis PDE
3D Gaussian Splatting UAV planning control trajectory optimization
site:arxiv.org H-NTFields neural time fields hierarchical motion planning
Hamilton-Jacobi Based Policy-Iteration via Deep Operator Learning
full-state quadrotor neural HJB feedback control
3D Gaussian Splatting safety filter quadrotor
measure to function operator Wasserstein neural network
```

## 检索异常与证据边界

- DBLP 在三组自动组合检索中均返回 HTTP 500。
- OpenAlex 在 3DGS/UAV planning 组合检索中返回 HTTP 504。
- 第二、第三组组合检索出现连接器 rate-limit 等待。
- Taylor & Francis 页面明确标注 Yuhuan Yue 2025 论文为 open access，DOAJ 也收录；文章 HTML 可读，但 PDF 直链在脚本和浏览器下载尝试中均被 HTTP 403/reader wrapper 拦截。因此仅记录书目信息，没有用 ResearchGate、Sci-Hub 或其他非正式镜像替代。
- 这是一份针对 SplatHJB claim 的高召回核心集，不声称覆盖所有机器人规划或所有 neural-operator 论文。

## 最强 novelty collisions

1. **PNO (`2410.17547`)**：已实现 cost-function → Eikonal-value solution operator，并跨地图、目标和分辨率泛化。
2. **HJRNO (`2504.19989`)**：已实现 obstacle/dynamics-conditioned HJ reachability neural operator，并从 value 导出反馈。
3. **c2g-HOF (`2012.06023`)**：已实现 workspace → continuous cost-to-go，并沿梯度无种子生成路径。
4. **Mean-field neural networks (`2210.15179`)**：已学习 Wasserstein measure → function operator，阻断宽泛的“measure input 是新概念”主张。
5. **ReNO / DI-Nets (`2305.19913`, `2206.01178`)**：已系统讨论 representation equivalence、operator aliasing 与有限离散化误差。
6. **Adaptive Deep HJB / DeepReach / PI-DeepONet**：高维 neural HJB/HJI、feedback 和 operator inference 均已有祖先。
7. **Splat-Nav / FOCI / SPLANNING / SAFER-Splat / FastBridge**：native GS planning、risk optimization、safety filtering 和真实 quadrotor realization 都已有强基线。

## 统一后的未占据位置

没有检索到一篇同时具备以下全部属性的直接先例：

```text
variable-cardinality Gaussian measure input
+ exact/controlled split-merge-refinement quotient
+ arbitrary goal
+ full UAV state and nonlinear dynamics
+ HJB value/policy solution operator
+ direct closed-loop planning without seed route/corridor/router
+ measure perturbation → value → policy stability statement
```

因此当前最可信的研究问题不是“别人没学过 value field”，而是：

> 能否把 3DGS 的非唯一 primitive 分解 quotient 掉，使不同但物理等价的 splat 表达严格或近似映射到同一个 full-state HJB value/policy，并证明这种表示稳定性真正改善跨重建、跨 densification 和跨地图的闭环规划质量？

## 已下载的 25 篇

完整标题、分类、页数、来源和 SHA-256 见 `manifest.csv`。所有 `status=open_access_downloaded_verified` 的条目都已放在本目录。

## 已核实但未获得 PDF 的 1 篇

- Yuhuan Yue, *Optimal control for quadrotors UAV based on deep neural network approximations of stable manifold of HJB equation*, Automatika 66(2), 2025, DOI `10.1080/00051144.2025.2461827`。开放获取页面已核实；PDF 获取状态为 `open_access_pdf_download_blocked_http_403`。

## 次级候选，本批未下载

- *Physics-informed Neural Motion Planning via Domain Decomposition in Large Environments* (`2506.12742`)：关注 large-environment scalability；与 H-NTFields 问题高度重叠，但对 measure quotient 较弱。
- *Physics-informed Neural Mapping and Motion Planning in Unknown Environments* (`2410.09883`)：Active NTFields，偏未知环境在线映射，不属于当前静态规划主线。
- *Point Cloud Neural Operator for Parametric PDEs on Complex and Variable Geometries* (`2501.14475`)：实现层邻接，但对 Gaussian measure/split-merge claim 弱于 Wasserstein operator 与 ReNO/DI-Nets。
- *Riemannian Eikonal Equation* (`2412.05197`)：几何 value-field 邻接工作，但不是 native GS/full UAV HJB operator。

## 本批未下载 SI

用户未要求 supporting information；本批只收 primary paper PDF。
