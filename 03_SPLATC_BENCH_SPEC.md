# SplatC-Bench — 自建 Benchmark Layer 规格 v1.0

**定位：** 不是重新采集视觉数据，而是在精确解析场景和公开场景资产之上，增加 robot morphologies、PointGoal episodes、converged `SE(2)` oracle、orientation-gate labels 与统一评估协议。

---

# 1. 为什么必须自建 benchmark layer

现有公开数据集没有同时提供：

- exact Gaussian / ellipsoidal hard supports；
-同一 scene 下多个 robot morphologies；
- continuous `SE(2)` collision truth；
- orientation-gate angular interval；
- gate 随 morphology 出生/收缩/消失的临界点；
- reachable/unreachable ground truth；
- remote closure；
- reusable multi-goal query；
- compiler/representation efficiency labels。

因此 SplatC-Bench 的本质是：

\[
\boxed{
\text{scene assets}
+
\text{robot library}
+
\text{episode generator}
+
\text{oracle annotations}
+
\text{evaluation harness}
}
\]

---

# 2. 数据集层次

```text
SplatC-Bench/
  SplatC-Gates/          # 解析可控核心 benchmark
  SplatC-Procedural/     # 随机多房间/多障碍 topology benchmark
  SplatC-Public/         # BARN / Habitat-GS / InteriorGS / HM3D adapters
```

## 2.1 SplatC-Gates

核心因果数据。所有 geometry、gate 和 critical thresholds 可解析或高精度验证。

## 2.2 SplatC-Procedural

验证方法不是单门专用；保留精确生成参数和 dense oracle，但不要求每个 topology 事件都有闭式解析式。

## 2.3 SplatC-Public

派生自公开资产。用于外部有效性，不承担主要 thin-gate 定理或精确临界点 claim。

---

# 3. 场景族设计

## G1 — Single Orientation Gate

### 几何

两个房间由一扇门连接，门宽 `w` 可连续变化。

### 机器人

长椭圆半轴 `a>b`，或由多个 Gaussian/ellipsoid primitives 组成的等效长条 body。

### 解析标签

横向投影半径：

\[
r(\theta)=\sqrt{a^2\sin^2\theta+b^2\cos^2\theta}.
\]

可通行条件：

\[
2r(\theta)<w.
\]

合法角度集合：

\[
\Theta(w)=\{\theta:2r(\theta)<w\}.
\]

保存：

- `gate_angle_intervals`；
- `critical_width = 2b`；
- `reachable`；
- `minimum_clearance`；
- oracle path。

### 目的

测量 fixed yaw layers、adaptive sampling 和 contact compiler 在：

\[
|\Theta(w)|\rightarrow 0
\]

时何时提前丢失 connectivity。

### 必需参数：de-alignment（Sprint A pilot 发现，2026-08-11）

G1 若门中心 `y=0`、gate 中心角 `theta=0` 都落在均匀网格相位上，uniform baseline 在任意薄的
gate 上都不会失败（pilot 实测：coarse 网格在 gate ±3.73° 时仍正确可达）。因此 G1 生成器必须
包含并在 sweep 中随机化：

- `door_center_offset`：门中心相对网格的连续偏移；
- `door_tilt`：门/走廊倾角（gate 中心角偏离 0）；
- 记录相位参数，保证可复现。

另外两点 pilot 结论：G1 的解析公式假设 corridor 式门（壁厚 `T >= 2a`）；thin-wall 变体的正确
标签是弦条件（见 `splatc_atlas/docs/problem_spec.md` §5）。恰好临界 `w = 2b` 是 knife-edge
参数点，按 §7.3 标 `ORACLE_UNRESOLVED`，不进入主评估。

---

## G2 — Sequential Incompatible Gates

两扇连续门要求不同姿态：

- gate A 只允许接近 `theta_A`；
- gate B 只允许接近 `theta_B`；
- `theta_A` 与 `theta_B` 相差较大。

测试：

-是否只发现单个 bottleneck；
-能否表达完整 orientation sequence；
- chart/gate transition 是否正确；
-局部 planner 是否在中间区域恢复/改变姿态。

---

## G3 — L-Corner / S-Corridor

机器人必须在拐角中联合改变 `x,y,theta`。

参数：

- corridor width；
- corner radius；
- segment length；
- robot aspect ratio。

目的：排除“只检测平行门宽”的特例方法。

---

## G4 — Short-Thin vs Long-Wide Dual Routes

两条 homotopy routes：

- Route A：短但 orientation gate 很薄；
- Route B：长但 clearance 大。

标签：

- 两条 route 的 existence；
- oracle costs；
- gate severity；
- optimal route under different `ell_R` / clearance penalty。

目的：

- topology completeness；
- min-plus cost comparison；
- compiler 是否过度偏向宽通道而删除真实短路线。

---

## G5 — Remote Closure

起点附近保持完全相同，只在远端关闭 corridor 或 gate。

保存两种 paired scenes：

```text
scene_open
scene_closed
```

要求：

- start-local contact values相同或数值近似相同；
- global reachability/goal query 改变；
- compiler update 能定位受影响区域。

目的：证明方法不是局部 overlap gradient。

---

## G6 — Morphology Bifurcation

固定 scene，连续改变 robot parameter：

\[
R(\alpha),\quad \alpha\in[\alpha_{\min},\alpha_{\max}].
\]

参数可以是：

- width；
- length；
- aspect ratio；
- Gaussian covariance；
- primitive relative placement。

保存：

- `critical_alpha`；
- gate interval vs alpha；
- component count vs alpha；
- representation update size/time。

目的：测试 embodiment topology discriminant 与 morphology update。

---

## G7 — Symmetric Alternatives

两条完全或近似对称路线。

允许输出：

- deterministic tie-break；
- explicit ambiguity flag；
- top-2 routes（secondary）。

禁止：

-用随机噪声制造不可复现选择；
-把一条路线丢失当成“等价选择”。

---

## G8 — Multi-Contact Combinatorics

多个 scene primitives 和 robot primitives 同时接近。

测试：

- scalar clearance `rho(q)=min h_ij(q)` 是否丢失 pair identity；
- active contact signature；
- balanced bilateral/multilateral contacts；
- primitive permutation invariance；
- duplicate split invariance。

---

## G9 — Decomposition Invariance

相同 hard body 用不同 Gaussian/ellipsoid decomposition 表示：

- 1 primitive；
- 2-4 overlapping primitives；
- dense decomposition。

场景障碍也做相同测试。

目的：

- representation 不应仅因 primitive 数量改变 topology；
-测量 compiler 对 decomposition 的稳定性与 scaling。

---

## G10 — Topology Stress Layouts

组合：

- multiple rooms；
- dead ends；
- loops；
- narrow gates；
- wide open spaces；
- repeated structures。

用于 SplatC-Procedural。

---

# 4. Robot Morphology Library

每个 robot 都由 hard support 与 Gaussian parameters 同时定义。

## R1 — Circle / Point-limit family

- 多个半径；
-用于 point-agent limit 和 obstacle inflation baseline。

## R2 — Single Ellipse

- aspect ratio sweep；
-最容易产生解析 orientation gate。

## R3 — Rounded Rectangle / Multi-Gaussian Bar

- 2-5 Gaussian primitives；
-更接近非圆移动机器人 footprint。

## R4 — Asymmetric Body

-前后/左右不对称；
-测试 `theta` 与 `theta+pi` 不再等价。

## R5 — Two-Lobe / Concave Approximation

- 多 primitive 非凸 body；
-测试 pair combinatorics 和 HRM/FOCI 适配。

## R6 — Same Hard Support, Different Gaussian Decomposition

-用于 representation invariance。

每个 robot metadata：

```yaml
robot_id:
family:
hard_support_primitives:
gaussian_primitives:
reference_frame:
characteristic_length_ell_R:
footprint_area:
circumradius:
inradius:
aspect_ratio:
decomposition_id:
```

---

# 5. Episode 类型

每个 scene-robot pair 生成以下 episodes：

## E1 — Clearly Reachable

宽通道，所有合理方法应成功。防止 compiler 过度保守。

## E2 — Critical Reachable

gate 非空但很薄。核心测试 false-unreachable。

## E3 — Just Unreachable

刚越过 physical critical threshold。核心测试 false-reachable。

## E4 — Globally Unreachable

不同 connected components，无任何窄门歧义。

## E5 — Multi-Route

至少两条 route，代价或 clearance 不同。

## E6 — Remote Closure Pair

open/closed scene pair。

## E7 — Multi-Goal Batch

固定 representation，包含一组：

- reachable goals；
- unreachable goals；
- different components；
- different route classes。

## E8 — Morphology Batch

固定 scene/start/goals，改变 robot morphology。

---

# 6. Gate Severity 指标

不要只用门的 workspace width。定义 configuration-space severity。

## 6.1 Angular measure

\[
s_{\theta}=\frac{\operatorname{measure}(\Theta_{\mathrm{gate}})}{2\pi}.
\]

越小越难。

## 6.2 Normalized clearance

\[
s_{\mathrm{clr}}=\frac{\delta_{\min}}{r_{\mathrm{char}}}.
\]

## 6.3 State-space neck volume proxy

\[
s_{\mathrm{neck}}=\frac{\operatorname{Vol}(\Gamma_{\mathrm{gate}})}{\operatorname{Vol}(Q_{\mathrm{local}})}.
\]

## 6.4 Critical distance to bifurcation

\[
s_{\alpha}=\frac{|\alpha-\alpha^*|}{\alpha^*}.
\]

报告时按 `easy / medium / hard / critical` 分层，但原始连续数值必须保留。

---

# 7. Oracle 生成协议

## 7.1 两类 truth

### Analytic truth

适用于 G1 等简单几何：

- exact gate interval；
- critical morphology；
- physical open/closed label。

### Converged numerical truth

适用于复杂场景：

- coarse / medium / fine / extra-fine；
- connectivity stability；
- path-cost convergence；
- gate interval convergence；
- independent collision checker。

## 7.2 Oracle record 必须保存

```yaml
oracle_method:
collision_checker_id:
resolution_xyztheta:
theta_periodic: true
connected_components:
reachable:
reference_cost:
reference_path:
minimum_clearance:
gate_intervals:
gate_ids:
critical_morphology:
convergence_status:
uncertainty_flags:
```

## 7.3 Oracle 不确定性

若 medium/fine 不一致：

-标记 `ORACLE_UNRESOLVED`；
-不进入主评估；
-继续 refine 或使用 analytic/interval methods；
-不能强行指定标签。

---

# 8. 数据 split

## 8.1 原则

- split 按 scene template、morphology family、gate severity 和 random seed 划分；
-不能只把同一模板的相邻参数随机分到 train/test；
- blind split 在 method tuning 前密封。

## 8.2 建议结构

```text
dev/
  known scene families
  non-critical + some hard gates
validation/
  held-out parameter ranges
  held-out morphology sizes
blind/
  held-out scene compositions
  critical gates
  morphology family holdouts
public/
  BARN/Habitat-GS/InteriorGS/HM3D derived episodes
```

## 8.3 必须有的 holdouts

- held-out aspect ratios；
- held-out gate widths；
- held-out primitive decompositions；
- held-out route topology；
- held-out public scenes。

若没有 learning，仍需保持 split，防止手工阈值对 benchmark 过拟合。

---

# 9. 文件 schema

## 9.1 SceneSpec

```yaml
scene_id:
source: analytic | procedural | BARN | HabitatGS | InteriorGS | HM3D
coordinate_frame:
units:
workspace_bounds:
hard_support_primitives:
gaussian_primitives:
source_asset_refs:
generation_parameters:
license_metadata:
```

## 9.2 RobotSpec

见第 4 节。

## 9.3 PlanningQuery

```yaml
query_id:
scene_id:
robot_id:
start_pose: [x, y, theta]
goal_position: [x, y]
goal_radius:
final_orientation: free
cost_config_id:
```

## 9.4 GateRecord

```yaml
gate_id:
scene_id:
robot_id:
local_region:
position_domain:
orientation_intervals:
active_contact_pairs:
minimum_clearance:
critical_parameters:
connected_chart_ids:
truth_type: analytic | converged_numeric
```

## 9.5 OracleRecord

见第 7 节。

## 9.6 MethodRunRecord

```yaml
method_id:
method_version:
commit_hash:
config_hash:
scene_id:
robot_id:
query_ids:
compile_time:
query_times:
path_times:
certification_times:
collision_queries:
retained_states_or_charts:
representation_bytes:
update_time:
paths:
failures:
random_seed:
hardware:
```

---

# 10. 评价指标

## 10.1 Topology

- connected-component accuracy；
- pairwise connectivity precision/recall；
- reachable/unreachable accuracy；
- false-reachable rate；
- false-unreachable rate。

## 10.2 Gate

- gate existence recall；
- gate position error；
- orientation interval IoU；
- angular-width error；
- minimum preserved gate severity；
- morphology critical-threshold error。

## 10.3 Path

- certified success；
- normalized cost suboptimality；
- minimum hard clearance；
- orientation smoothness；
- path length；
- number of refinements/rejections。

## 10.4 Representation

- retained states/cells/charts；
- gate count；
- memory bytes；
- collision-query count；
- compile time；
- per-goal query time；
- cumulative `K`-goal time；
- morphology update time；
- scene/robot primitive scaling。

## 10.5 Robustness

- theta resolution；
- position resolution；
- primitive decomposition；
- `ell_R` sensitivity；
- contact threshold；
- chart merge threshold；
- random seeds；
- public-scene representation mismatch。

---

# 11. 主要 benchmark plots

必须自动生成：

1. topology accuracy vs collision queries；
2. gate recall vs retained states/memory；
3. false-unreachable vs `s_theta`；
4. minimum preserved gate width vs yaw channels；
5. cumulative time vs number of goals；
6. update cost vs morphology change；
7. path cost vs representation budget；
8. component/gate count vs robot parameter `alpha`；
9. public-scene success vs scene complexity；
10. failure taxonomy stacked chart。

---

# 12. Public Adapter 规范

## 12.1 BARN Adapter

```text
occupancy grid
-> polygon/obstacle representation
-> Gaussian/ellipsoid decomposition
-> robot morphology injection
-> SE(2) episode generation
-> dense oracle labels
```

保存原 BARN environment ID 和 difficulty score。

## 12.2 Habitat-GS Adapter

优先 self-reconstructed scenes：

```text
GS asset -> method input / visualization
collision mesh -> hard reference
NavMesh -> proposal only, not truth
our robots -> morphology episodes
```

必须统计 GS hard-support 与 mesh collision 的 mismatch。

## 12.3 InteriorGS Adapter

```text
structure.json -> door/corridor candidates
3DGS -> method input
occupancy -> proposal
our hard-support conversion -> checker
our oracle -> labels
```

## 12.4 HM3D Adapter

```text
mesh subset
-> common geometry for SE(2) NavMesh-style and ours
-> deterministic Gaussian/ellipsoid conversion
-> same robot and query manifests
```

派生数据发布需遵守原 dataset license，不复制不能再分发的原始资产。

---

# 13. 数据质量 Gate

## Data Gate 0 — Schema

- all files validate；
- coordinate frames/units明确；
-每条 episode 可回放。

## Data Gate 1 — Geometry

- analytic cases一致；
- broad phase no false negative；
- duplicate/permutation invariance tests。

## Data Gate 2 — Oracle

- resolution convergence；
- unresolved cases隔离；
- path certification。

## Data Gate 3 — Split integrity

- no near-duplicate leakage；
- blind manifest hash 固定；
- public test scenes未用于调参。

## Data Gate 4 — Benchmark reproducibility

- one-command generation；
- fixed seeds；
- auto tables/plots；
- raw + aggregate results；
- failure logs。

---

# 14. 数据集规模建议

不追求巨大规模，追求覆盖主张。

## Core controlled set

每个核心 scene family：

- 多个 parameter sweeps；
-至少 3 morphology families；
- reachable / critical / unreachable 都覆盖；
-每个 scene-robot pair 多 goals。

## Procedural set

以数百个 scene-robot pairs 为目标，而不是数十万 episodes。每个 pair 可生成多个 goals，因此 multi-goal evaluation 自然扩展。

## Public set

- BARN：选择覆盖不同 difficulty 的 representative subset，再做完整扩展；
- Habitat-GS/InteriorGS：先 5-10 scenes 做 QA，再扩大；
- HM3D：与 SE(2) NavMesh 对齐的少量场景优先。

规模只在 evaluator 和 truth 稳定后扩大。

---

# 15. 发布内容

若作为论文 artifact，发布：

- generator code；
- robot library；
- configs/seeds；
- analytic formulas；
- derived manifests；
- oracle labels；
- baseline runner；
- evaluator；
- table/figure scripts；
- small redistributable examples；
- public dataset download/adaptation scripts。

不发布：

- 无权再分发的 HM3D/3D-FRONT/其他原始资产；
-用户私有扫描；
-未解决 oracle 标签。

---

# 16. Benchmark contribution 的正确表述

可以说：

> We introduce a controlled benchmark layer for morphology-conditioned configuration-space topology, with analytic and converged labels for vanishing orientation gates, morphology bifurcations, remote closures, and multi-goal reuse.

不应说：

> We introduce the first large-scale Gaussian navigation dataset.

除非实际完成大规模、公开、许可完整、标准 split 和广泛场景覆盖；当前计划的首要贡献仍是 functional compiler。
