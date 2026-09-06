# Worklog — GMC Gate G-H2：orientation event 稀疏性 pilot

目标：判定 Hypothesis H2（topology-changing orientation events 稀疏，event-driven
tracking 才有存在价值）。这是 v3 总控 §4 的第一个 Gate，先于一切方法工程。

## 2026-08-24 首轮（uniform random 场景族，本地）

- 环境：本地 miniconda base + pip shapely 2.0.7（pilot 量级，几分钟；结构化场景
  复测按 README 规范上 HPC）。
- 实现：`experiments/gmc/h2_event_density.py`。θ 均匀细网格（1440 = 0.25°）作为
  reference 计数器（**不是最终算法**），统计三类签名的相邻步变化：
  - S1 = nerve 1-skeleton（哪些 forbidden primitives 两两相交）→ 组合 churn；
  - S2 = 障碍并集拓扑 (#comp, #holes)（限 workspace 内）；
  - S3 = 自由空间拓扑 (#comp, #holes) → mobility complex 真正要重新 glue 的事件。
- 语义：overlap superlevel 闭式椭圆，固定 Mahalanobis 支撑半径 ρ=2（保守
  support 味道）；椭圆 32 边形多边形化（pilot 级，非认证几何）。
- 机器人：两 part、刻意不对称（避免 π 对称把事件数砍半造成假象），支撑意义下
  长宽比约 3:1。

### 遇到的问题与解决

1. 本地无 shapely：所有 conda env 均无 → pip 装进 base（记录版本 2.0.7）。
   结构化复测时用 HPC overlay env，勿依赖本地环境复现。
2. θ 网格计数是真事件数的**下界**（两格点之间的成对事件会相消）。对策：JSON 里
   记录 step_change_fraction，饱和（→1）即标注该签名被低估——首轮 N=160 的 S1
   预计饱和，这本身就是"churn 密集"的证据，但 S1 绝对数不可引用。
3. 机器人对称性陷阱：对称 body 会让事件以 π 为周期重复计数/隐藏，首版特意用
   不对称 parts 并全 2π 扫描。

### 结果（2026-08-24 首轮，1440 θ × N∈{10..160} × 3 seeds，exit 0）

| N | S1 中位 | S3 中位 | S3/S1 | S1 饱和度 | S3 饱和度 | churn | free 面积占比 |
|---|---|---|---|---|---|---|---|
| 10 | 20 | 16 | 0.80 | 0.01 | 0.01 | 20 | 0.88 |
| 20 | 78 | 55 | 0.71 | 0.05 | 0.04 | 78 | 0.78 |
| 40 | 317 | 141 | 0.44 | 0.22 | 0.10 | 365 | 0.65 |
| 80 | 947 | 282 | 0.30 | 0.66 | 0.20 | 1511 | 0.48 |
| 160 | 1406 | 342 | 0.24 | **0.98** | 0.24 | 5726 | 0.28 |

log-log 斜率：churn（S1 的非饱和代理）= **2.06**（二次）；S3 全程 = **1.12**
（近线性），top-3 N = 0.64。S1 计数在 N=160 完全饱和（0.98），绝对值不可引用，
churn 代理替代。S3 饱和度 0.24 → 存在轻度低估（约 10–15% 量级），不改变量级结论。

**首轮判读：H2 在 uniform random 族上成立方向明确**——组合 churn 二次增长而
自由空间拓扑事件近线性，S3/S1 从 0.80 单调降至 0.24；barcode 显示 S3 有清晰的
event-free 区间与事件聚簇（event continuation 的理想目标形态），S1 处处密集
（必须局部消化，不能全局重建）。S2 与 S3 几乎重合，符合"gate 闭合多伴随障碍
并集合并"的预期。

图：`results/gmc_h2/figs/{slices_N40,barcode_N40,events_vs_N}.png`。
slices 图人工复核过：θ=0 全连通 vs θ=90 分裂为 4 个分量——朝向驱动的连通性
变化真实出现。渲染瑕疵：被围住的 free 腔显示为白色（union 洞的填充盖在彩色
free cell 上层），只影响显示不影响计数；G1 出正式可视化时修复。

### Gate G-H2 状态

首轮（uniform random 族）**通过方向**，但 Gate 未关闭：需结构化场景族
（墙体/走廊/房间，HPC）复测 + N=160 处 S3 用更细网格或 bracketing 验证低估幅度。

## 2026-08-24 K2 真实场景窗口轮（v1 → v2）

实验：`experiments/gmc/h2_k2_windows.py`，三窗口（door_A (22.3,−15.2)、
door_B (15.2,−11.0)、tables (36,−19)，各 6×6 u），720 θ ∈ [0,π)（单 part
机器人 π 对称），gate 判定 = 门两侧探针点同分量。

### v1 轮结果与三个发现（k2_windows_v1.json）

1. **door_A/door_B：0 个 S3 事件、全朝向可通**。诊断：机器人支撑长 1.6 <
   门宽 ~2.0，平行于墙也能过——语义正确（"该形态任意朝向可过此门"），但
   造不出 orientation gate。对策：v2 加长机器人（支撑 2.4 / 3.2 > 门宽），
   顺势升级为 morphology 扫描 = Claim A 原型（同一真实门，不同机器人长度
   → 不同 gate 角度区间 → closure）。
2. **tables：68 个 S3 事件**（真信号：桌椅群随 θ 产生分量分裂/走廊开合，
   切片图确认），但探针放置逻辑假定有墙，落进禁行区 → gate 指标对无墙
   窗口无效。v2 中该窗口只测 S3。
3. **churn ≈ 1.0–1.6 × 10⁷ 相交对/切片**（12–17k primitives，平均度
   ~1500）：真实 3DGS 的 splat 在墙面上高度堆叠，机器人卷积加粗后近邻
   全连。**架构结论：contact complex 绝不能建在原始 splat 上，M1 必须
   包含 primitive 合并/聚类（墙段级），nerve/churn 只在合并后对象上
   维护。** 这也是 v1 运行时间爆炸的原因（churn 采样占 4/5 时长），v2
   每配置只采一次存档。

### v2 轮：morphology closure 阶梯（k2_windows_v2.json）

机器人支撑长 {1.6, 2.4, 3.2} × 宽 0.6：

| 配置 | S3 事件 | gate 开放占比 | 判读 |
|---|---|---|---|
| door_A/L16 | 0 | 1.00 | 门宽>机器人任何投影，恒开（正确） |
| door_A/L24 | 55 | 0.44 | **真 orientation gate**：开合带清晰，事件聚在开合角附近 |
| door_A/L32 | 73 | 0.00* | *探针伪象（见下） |
| door_B/L16 | 0 | 1.00 | 恒开 |
| door_B/L24 | 64 | 0.58 | 真 gate，6 个开合角 |
| door_B/L32 | 46 | 0.00* | *探针伪象 |
| tables/L16→L24 | 68→42 | n/a | 机器人变大 → 更多区域封死 → 事件减少 |

**closure 阶梯（全开→部分开→?）在两扇真实门上复现**——Claim A 的
embodiment-specificity 原型。L24 的 S3 事件在 barcode 上聚簇于 gate 开合角
附近，且存在闪烁段（如 door_A 132.8–135.2° 连续 switch）= 近切触区，
正是 event bracketing 要精确定位的对象。

### v2→v3：点探针被吞 bug

L32 "全关" 经切片图复核为伪象：θ=120 门廊自由通道明显存在，但点探针
（±1.3u）被加粗墙体吞掉 → gate 恒 False。**教训：gate 判定不能依赖单点
成员资格**（长机器人把大片位姿空间变禁行）。v3 改为门两侧径向线段探针
（offset 1.0–2.4u），任一自由分量同时截到两条线段即为开——对局部吞没
鲁棒。只重跑 door×{L24,L32}，其余从 v2 合并。这个 bug 模式对 G1 的
witness 设计也适用：**witness 应是集合/线段，不是单点**。

### v3 终版结果（k2_windows.json，线段探针，2026-08-24）

| 配置 | S3 事件/π | gate 开放占比 | gate 开合角（deg） |
|---|---|---|---|
| door_A/L16 | 0 | 1.00 | 恒开 |
| door_A/L24 | 55 | 0.64 | 2.0, 29.0, 29.5, 38.5, 91.2, 119.3 |
| door_A/L32 | 73 | 0.27 | 45.5, 82.0, 158.5, 169.8 |
| door_B/L16 | 0 | 1.00 | 恒开 |
| door_B/L24 | 64 | 0.72 | 3.2, 28.8, 93.5, 118.7 |
| door_B/L32 | 46 | 0.34 | 37.8, 83.0, 132.8, 143.2, 157.8, 163.5 |

- **closure 阶梯单调成立**（两门 1.00→0.64/0.72→0.27/0.34），主开放带包住
  门法向（60.75°±墙向），L32 出现非显然的第二窄开放带（斜穿姿态的独立
  route class in orientation）——采样规划器无法枚举的输出。
- **S3 事件稀疏且结构化**：720 采样中 42–73 个事件（step 占比 0.06–0.10），
  聚簇于 gate 开合角附近，稳定带内大段 event-free——event continuation 的
  理想目标形态；churn 固定 ~10⁷ 对/切片（饱和，架构结论见 v1 节）。
- 汇总图 `figs/k2_closure_ladder.png`；逐配置图 `figs/k2_<win>_<robot>.png`。
- 过程 bug 记录：③ 结尾一次性写 JSON 被中途 kill 后全丢 → 改为逐配置落盘
  + 断点续传（脚本 v3.1）。

### 整层 K2 sweep（2026-08-25，k2_fullfloor.json）

L24 机器人、34 个活跃 6×6u 瓦片（stride 5）× 720θ，本地 8 进程 pilot：
- S3 事件/瓦片：min 0、**中位 36**、p90 75、max 100；中位 θ 步占比 **0.050**；
- 全楼合计 1202 个事件（0.25° 分辨率下）——mobility complex 只需这 ~1200 次
  局部更新，对照 dense 均匀离散需全域 720 层重建；
- **空间局域性**（`figs/k2_event_heatmap.png`）：高事件瓦片精确贴合墙角/
  门洞/长墙/桌群，开阔区near零——事件住在接触结构上，output-sensitive
  假设的直接图证。

### Gate G-H2 判定（2026-08-24 初判，2026-08-25 关门）

**三证据齐：① uniform random 合成族（churn 二次 vs S3 近线性）；② K2 真实
门洞窗口（S3 稀疏聚簇、gate 区间可测、closure 阶梯单调）；③ K2 整层
（中位步占比 0.05、事件空间局域于接触结构）。**

**Gate G-H2：PASS**（2026-08-25，基于本地 pilot 证据；canonical HPC 重跑
排入论文判定链待办）。三个预注册退出条件均未触发。主线全力进 Stage G1。

### 本轮明确不能回答的

- 结构化场景（墙/走廊/房间）下 S3 是否仍近线性——uniform random 是最软的
  场景族，Gate G-H2 需在结构化族复测通过才算过；
- 真事件数（需 bracketing/continuation 而非网格计数）；
- gate/adjacency 级事件（本轮只有分量/洞计数，同分量内 gate 开合未计入 S3，
  会低估 planning-relevant 事件——G1 做出 free dual 后补 adjacency 签名）。
