# Worklog — GMC Stage G1：固定 θ slice 正式实现

## 2026-08-25 开工：M1 保守聚类 pilot（v1→v2→v3 三轮收敛）

背景：H2 实测 churn ~10⁷ 对/切片（`gmc_H2.md` v1 发现 3）→ contact complex
必须建在合并 primitive 上。本轮在 door_A/L24（有 720θ 未聚类 gate 参照）上
迭代三版构造，标准三条：保守（leak=0、false_open=0）、便宜、保真。

### 三版对比（`results/gmc_h2/m1_cluster_pilot*.json`）

| 指标 | v1 椭圆拟合+协方差相加 | v2 椭圆拟合+Minkowski | **v3 切线多边形+Minkowski** |
|---|---|---|---|
| max leak (u²) | 0.82 ❌ | 0.0 | **0.0** |
| false_open | 30 ❌ | 0 | **0** |
| 过覆盖 (u²) | 1.19 | 6.94 ❌ | **1.48** |
| gate 一致率 | 0.879 | 0.44 ❌ | **0.889** |
| open frac (参照 0.644) | 0.607 | 0.082 | **0.533** |
| primitive 数 | 423 | 423 | 423（12793 原始，30×） |
| churn 对数 | 1.1e4 | 1.3e4 | 1.2e4（**940× 降**） |
| 切片耗时 | 23ms | 24ms | 25ms（原始 560ms，22×） |

### 数学结论（G1 的正式构造，写进论文 M1 节）

1. **场景层支撑包含经机器人卷积不保序**（v1 的坑，即总控 §5.3 预留的
   margin 引理缺口的实测版）：√(a+λ)−√(b+λ) < √a−√b（凹性收缩），
   所以"合并椭圆盖住成员支撑 ⇒ 协方差相加禁行椭圆也盖住"**不成立**。
2. **成立的保守链**（v2/v3）：
   F_i = E(Σᵢ+Λθ) ⊆ E(Σᵢ)⊕E(Λθ)（支撑函数 √(a+b)≤√a+√b）
   ⊆ E_m⊕E(Λθ)（E_m ⊇ E(Σᵢ)，θ 无关！）⊆ P_m⊕R_θ（外切多边形化）。
   凸多边形 Minkowski 和用边合并精确计算（角度需归一化到 [0,2π)，
   否则从最低点出发的边序被负角度边插队产生自交——v2 首跑的 bug）。
3. **合并形状用支撑函数切线多边形，不用椭圆拟合**（v3 的关键改进）：
   对桶内成员并集按 24 个固定方向取 h(u_k)=maxᵢ[cᵢ·u_k+ρ√(u_kᵀΣᵢu_k)]，
   相邻切线求交得顶点。逐方向精确、零拟合损耗，构造 0.02s。

### 架构结论：两层认证层级

v3 残余 11% extra-closed（桶粒度在墙端的加粗）表明合并层是**广相/全局
结构层**，不是 gate oracle。G1 采用两层：
- **合并切线多边形层**：全局 arrangement/free dual/nerve 维护（churn 1e4、
  25ms/slice 可行）；其 free 判定是认证的（外覆盖 ⇒ 说 free 必真 free）；
- **原始成员细化层**：只在合并层报 closed 且答案关键的 gate 邻域局部展开
  （细化只会"打开"，单调，认证格干净）。
细化触发与 event tracker 的"只更新受影响区域"天然同构。

## 2026-08-25（续）：细化层 + 参数扫描

### 细化层 pilot（m1_refine_pilot.json）

两层协议：coarse 开 ⇒ 认证开（直接采纳）；coarse 关 ⇒ 门廊 patch（探针
hull ⊕ 机器人半长 + 0.3）内的 hot 桶换回原始成员、cold 区保持合并，复算。

| 配置 | 一致率 | false_open | false_closed | 细化步数 | 提速 vs 全原始 |
|---|---|---|---|---|---|
| door_A/L24 | **1.000** | 0 | 0 | 336/720 | 3.9× |
| door_A/L32 | 0.997 | 0 | 2 | 560/720 | 2.1× |
| door_B/L32 | 0.989 | 0 | 8 | 532/720 | 2.1× |

- **false_open 全零**：混合集仍覆盖真禁行集，"开"方向双层皆认证。
- L32 残差 false_closed（2/8 步）＝预言的第三情形：近临界姿态下真实通路
  绕出固定 patch。**生产设计：refined-closed 触发 patch 自适应生长至
  不动点**（最坏退化为全原始）——一致率按构造为 1，成本自适应。
- 6×6 小窗口里 hot 桶占 38–50%，2–4× 提速是**下界**；整层部署 cold 区
  为全楼，coarse 全局 sweep + 门邻域细化的收益远大于此。

### 桶参数扫描（m1_param_sweep.json，door_A/L24）

| cell | bins | n_merged | 一致率 | extra_closed | 过覆盖 | churn | 切片 ms |
|---|---|---|---|---|---|---|---|
| **0.5** | **8** | **423** | **0.889** | 80 | 1.47 | 12.2k | 23 |
| 0.5 | 12 | 603 | 0.885 | 83 | 1.47 | 24.7k | 32 |
| 0.3 | 8 | 690 | 0.868 | 95 | 1.25 | 31.7k | 36 |
| 0.3 | 12 | 982 | 0.876 | 89 | 1.29 | 63.8k | 47 |
| 0.8 | 8 | 265 | 0.858 | 102 | 2.06 | 4.7k | 14 |

- false_open 全配置恒 0（构造性质，断言验证）；
- gate 一致率对参数**不敏感**（0.86–0.89 平台）——细化层兜底后 coarse
  参数是纯成本旋钮。**默认 (0.5, 8)**；整层用 (0.8, 8) 可再省 2.5×。

## 2026-08-25（三）：代码提升进 src + patch 自适应生长

**`src/splatc/gmc/`** 三模块落地（`geometry.py` 凸核、`cluster.py` M1 聚类、
`hierarchy.py` 两层认证查询含 `gate_query` 自适应生长），API 经
`splatc.gmc` 导出；**`tests/test_gmc.py` 12 项测试**（Minkowski 对暴力凸包
参照、外切/内接包含、禁行椭圆对 Mahalanobis 采样、聚类保守性全 θ、合成门
两层=全原始逐 θ 一致、认证方向永不假开、退化薄协方差）。全套件 **91/91
通过**，对 atlas 既有代码零回归。

### 测试首跑抓住的两个真 bug（都已修，教训入注释）

1. **margin 回归**（重构引入）：`build_merged` 把 margin 乘在整个支撑值
   h = c·u + ρ√(uᵀSu) 上——c·u<0 时 h 反被乘小、切线内移、产生泄漏
   （pilot3 原版只乘 √ 项）。修：margin 只进 ρ。
2. **Minkowski 0/2π wrap**：角度 ≈0 的边因浮点噪声（atan2 → −1e-13）
   mod 2π 后≈2π 被排到末位——这是合法的循环旋转（形状不变、面积逐位
   一致），但锚定起点的链条被平移了整条边长（实测质心偏 0.115）。
   修：任意切点重建链条后整体平移对齐 bottom(P)+bottom(Q)（Minkowski
   不变量），对 wrap 与角度并列鲁棒。debug 路径：泄漏区→单成员薄
   sliver 桶→"面积相同但对称差 0.147"→平移诊断。

### patch 自适应生长（gate_query）

coarse 关 → patch 从探针 hull 起步，每轮扩 robot 半长；refined-开 →
认证开；hot 覆盖全部桶 → 全原始、判定精确。**代价不对称是设计使然**：
真关的 θ 步会升级到全原始切片（小窗口里≈放弃提速），固定 patch
（0.989 一致率）与自适应（构造性 1.0）是暴露给调用方的策略选择。
K2 回归（door_B/L32，src API，m1_src_regression.json）：**一致率 1.0、
false_open 0、false_closed 0**；证书构成 coarse-open 221 / refined-open 25 /
exact-closed 474。墙钟 486s > 全原始 403s——小窗口里真关步全部升级全原始
（窗口即全域，无 cold 区），实证了"两层收益只在域≫patch 时兑现"；
整层部署 coarse 全局 + 门邻域细化才是收益端。

## 2026-08-25（四）：G1 主体验收 —— Gate G1 关门

**`src/splatc/gmc/slice.py`**：hard-support（Minkowski）语义的固定 θ slice，
直接建在冻结 bench primitives 上（disc 场景 ⊕ 椭圆机器人，逐 disc-group
一次 Minkowski 原型 + 平移广播，~25ms/slice）。双侧多边形化 = 总控 §5.1
的两侧认证：外切 → free 欠估 → 连通/开 判定认证；内接 → free 过估 →
断开/关 判定认证；open(outer) ⊆ truth ⊆ open(inner)。每个 free 分量带
witness pose，由冻结独立 checker（eval_points）认证，永不由多边形管线
自证。

### 验收（g1_slice_validation.json，dx=0.05/144θ oracle + 720θ gate 扫）

| 族 | 组合 | 切片判定 |
|---|---|---|
| single_door（冻结 bench） | 4 门宽 × 3 机器人 | **864/864 ok** |
| double_door（新构造） | ×R_long_ellipse | 64 ok + 6 certified_conn + 2 sliver，0 fail |
| u_shape / keyhole（新构造） | ×R_long_ellipse | **72/72 + 72/72 ok** |

- **总计 1080 切片：0 fail、witness 失败 0、gate 三明治违规 0**；
- gate 区间（4 门宽 × 720θ）：open fraction 与解析真值差 ≤0.003
  （= 0.25° 采样 + 多边形化间隙，方向正确 outer ≤ truth）；
- 6 个 certified_conn = GMC-outer 连通（构造性证书）而粗网格 oracle 碎裂
  亚网格临界走廊；dx/2 复核多数向 GMC 靠拢，两例更碎（走廊 <0.025）——
  无论网格怎么说，outer free ⊆ true free ⇒ 连通即证明；
- 2 个 sliver = ≤8 cells 亚分辨率碎片，记录在案。

**Gate G1 判据（bench 各族 free-component 与 gate 对 oracle P/R=1）达成：
Gate G1 PASS（2026-08-25）。** 测试套件 94/94（新增 TestSlice 三项回归）。

### 验收过程抓住的真发现：workspace 语义错位

keyhole θ=90° 一度出现"GMC 合并了 oracle 分开的区域"且 dx/2 复核不消。
诊断：**oracle 的 free 语义包含机器人整体在 workspace 内**（eval_points
按 support_half_widths 腐蚀边界），而 slice 初版只约束中心 → 绕障碍外侧
出现零测度"通道"。冻结 bench 满分恰因它把墙封出 workspace 边界，掩蔽了
错位；新构造族不封边，暴露之。修复：slice 按 (px,py) 腐蚀 workspace
（outer 侧加 ε 保持严格子集）。固化为回归测试
`test_workspace_body_containment_semantics`。
教训：**自造验收族的价值恰在于打破冻结族的隐式约定**。

### 待办

- [ ] margin 引理 + 双侧认证的正式写法（proof obligation 清单入总控 §5）
- [ ] arrangement 级 gate 几何提取（gate 在哪段边界、宽度、涉及 pair）——
      归 G2 event tracker 的输入
- [ ] K2 pilot 脚本迁移到 src API（experiments 版冻结为记录）
- [ ] Stage G2 开工：gap functions + event bracketing + certified subdivision
