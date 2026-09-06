# SplatC-Atlas problem_spec v0.1 (Stage 0 contract, G1 pilot)

**状态：** Sprint A 冻结稿。改动必须记入 `docs/decisions.md` 并 bump 版本。

## 1. 坐标、单位与状态

- 单位：米、弧度。世界系右手系，`theta` 为机器人 body +x 轴相对世界 +x 轴的逆时针角。
- 配置 `q = (x, y, theta) ∈ SE(2)`，`theta ∈ [0, 2π)` 周期。对称机器人满足 `rho(θ) = rho(θ+π)`，实现可利用该对称，但语义上仍是全圆。
- 位置–旋转特征长度：`ell_R = 0.5 m`（全局固定，跨 morphology 不变）。
- 路径代价（所有方法统一重评分用）：
  `J = Σ sqrt(Δx² + Δy² + ell_R²·Δθ²)`，`Δθ` wrap 到 `(−π, π]`。

## 2. Hard support 与接触函数语义

- Scene primitive：`E_i = {x : (x−μ_i)ᵀ Σ_i⁻¹ (x−μ_i) ≤ 1}`（本 pilot 中全部为圆盘，即 `Σ = R²·I`）。
- Robot primitive：body 系椭圆，pose `q` 下刚体变换到世界系。
- Pairwise 接触函数取 **Perram–Wertheim 接触值**：
  `h_ij(q) = sqrt(F_PW) − 1`，其中 `F_PW = max_{λ∈(0,1)} S(λ)`。
  `h > 0` 分离，`h = 0` 相切，`h < 0` 相交。**注意 `h` 是无量纲缩放余量，不是米制距离**；米制 clearance 需要时另行报告（checker #2 输出米制 `dist − R`）。
- 全机器人 clearance：`rho(q) = min_ij h_ij(q)`（仅在 broad-phase 近邻壳层内精确计算；远离所有障碍处 `rho` 以 cap 值截断，不影响 free 判定与 gate 度量）。
- FREE 判定：`rho(q) > ε_t` 且 workspace 四条 support 余量均 `> ε_t`，**ε_t = 1e-9**
  （保守 tie-break：数学上恰好相切的 pose 按定义不 free；由此 ±1 ulp 浮点噪声不可能翻转
  标签——G1 的整数几何 + 整数网格会产生大量测度零精确相切 pose，V4 对称性审计实测抓到
  120 个此类噪声翻转 cell，是本条款的来源）。ε_t 比最薄关心信号（knife-edge ρ≈9e-5）低
  约 5 个量级，不影响真值。
- 路径认证时报告沿途 `min rho`（PW 单位）与保守 swept 证书的米制最小余量。

## 3. 两套独立碰撞实现（Gate 0 要求）

- Checker #1（主）：Perram–Wertheim，`λ` 上对 `dS/dλ` 做向量化二分（40 iter），ellipse–disc 情形闭式对角化。
- Checker #2（独立）：disc 圆心到机器人椭圆的点–椭圆距离（标准 `F(t)` 二分），碰撞 ⟺ `dist < R` 或圆心在椭圆内；圆形机器人则为纯圆–圆闭式。
- 两者在解析 case 与随机 pose 上必须符号一致（tests 强制）。

## 4. G1 场景族（pilot 冻结几何）

- Workspace：`[−3.5, 3.5] × [−2.3, 2.3]`。
- 分隔墙：两段矩形区域 `x ∈ [−T/2, T/2]`，`w/2 ≤ |y| ≤ 2.3`，**T = 1.2**（= 长条机器人 2a），即 corridor 式门。
- 墙由 disc primitives 组成（契约要求 ellipsoid primitives）：
  - corridor 边缘行：r = 0.10，间距 0.02，包络线波纹 ≤ 5e-5 m；
  - 墙面列：r = 0.10，间距 0.04（波纹 ≤ 2e-3 m）；
  - 内部填充：r = 0.15，网格 0.2（≤ r√2，无空洞 ⟹ 无墙内幻影连通分量）；
  - 门楣四角为 r = 0.10 圆角（记录在案；不影响 mid-corridor gate 真值）。
- 门宽 `w` 为族参数。

## 5. 解析 gate 真值（G1）

θ 定义为机器人长轴与门法向（+x）的夹角；`r(θ) = sqrt(a² sin²θ + b² cos²θ)` 是**门宽方向投影半径**。

- **Corridor 门（T ≥ 2a，本 benchmark 默认）**：可通行姿态集
  `Θ(w) = {θ : 2 r(θ) < w}`，临界宽度 `w* = 2b`。
  正确性论证：T ≥ 2·max proj_x = 2a ⟹ 穿越必经过完全浸没时刻 ⟹ 必要性；固定 θ∈Θ(w) 沿 y=0 直线平移 ⟹ 充分性。
- **Thin-wall 门（t → 0，非默认，需显式标注）**：正确条件是**弦条件**
  `Θ_thin(w) = {θ : 2ab / sqrt(a² cos²θ + b² sin²θ) < w}`，
  严格宽于 corridor 条件（同 `w*=2b`）。
  ⚠️ 03_SPLATC_BENCH_SPEC §3 G1 的投影公式默认指 corridor 情形；如做 thin-wall 变体，标签必须换弦公式。
- Oracle 端 gate interval 测量协议：门中面 `x = 0`，`Θ_num = {θ : ∃y, free(0, y, θ)}`。
  Corridor 情形下解析上 `free(0,y,θ) ⟺ 2r(θ)<w 且 |y| < w/2 − r(θ)`。

## 6. Robot morphology library（pilot 三件）

| id | 形状 | 参数 | w=0.7 时预期 |
|---|---|---|---|
| `R_small_circle` | 圆盘 | r = 0.25 | 任意朝向直接过门 |
| `R_long_ellipse` | 椭圆 | a = 0.6, b = 0.25 | 必须侧身（2b=0.5 < 0.7 < 2a=1.2）|
| `R_big_circle` | 圆盘 | r = 0.40 | 不可达（0.8 > 0.7）|

`R_small_circle ⊂ R_long_ellipse`（同心）⟹ body-inclusion monotonicity 可测。

## 7. 任务与 PointGoal

- start：`q_start = (−2.0, 0, π/2)`（侧向朝向，强迫演示 pre-rotation）。
- PointGoal：`g = (2.0, 0)`，`ε_g = 0.30`，最终朝向自由。
- unreachable 语义：start 分量与 goal 集合所有 free pose 分量不相交。

## 8. Dense Oracle 分辨率

| 档 | dx=dy | dθ | 规模（G1）|
|---|---|---|---|
| coarse | 0.10 | 5.0° (72) | ~0.5M poses |
| medium | 0.05 | 2.5° (144) | ~1.9M poses |
| fine | 0.025 | 1.25° (288) | ~15M poses |

- 连通分量：6-连通 + θ-wrap 合并；路径：Dijkstra（10-邻域，统一 `J` 度量）。
- **邻接合同**：可达性标签 = 6-连通（保守，物理正确：对角跳跃可能扫过被堵角落）；
  路径图 = **起点分量内**的 10-邻域——xy 对角只允许在分量内部用于路径质量，绝不作为
  跨分量桥。因此"reachable 标签"与"路径存在性"永不矛盾，且每条边仍需保守 swept 认证。
- 统一代价 `J` 的 clearance 权重 **λ_clr = 0**（pilot 冻结值；若启用须重新定标并记决策日志）。
- 收敛判据：medium 与 fine 在 components / reachable / gate interval 上结论一致。
- Oracle 路径每条边做连续认证：边内插值 4 个 pose 逐一 hard check。

## 9. Endpoint 预注册（同步 00 号文件）

Primary endpoint = Claim C thin-gate preservation；其余 residual 维度为 secondary。

## 10. 本 spec 尚未覆盖（Sprint A 内待补）

- blind split manifest（generator 参数族确定后密封）；
- baseline budget 协议数值（collision-query / state / wall-clock 三种 matched 预算的具体额度，待 uniform baseline 首跑后定标）。
