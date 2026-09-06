# Worklog — GMC Stage G2：orientation event 层

## 2026-08-25 深夜：认证 gate 区间算法落地 + 第一腿验收

### 核心构造（`src/splatc/gmc/events.py`，进论文方法节）

1. **刚性旋转系统**：机器人多边形改为 θ=0 基多边形的刚性旋转
   （`slice.robot_support_poly`）。旋转与外切/内接可交换 ⇒ G1 三明治不受
   影响；系统对 θ 严格 Hausdorff-Lipschitz，速率 **L = max 顶点半径**（精确）。
2. **免费 Minkowski 缓冲**：F_i = disc_i ⊕ E_robot ⇒ 膨胀 ε = disc 半径
   +ε（精确等式）；逐 primitive 腐蚀 = disc 半径 −ε（合法子集，
   cap 于最小 disc 半径）。workspace 腐蚀量同步 ±ε。`build_slice(perturb=)`。
3. **扰动认证**：verdict 在 perturb=+ε 下不变 ⇒ |dθ| ≤ ε/L 内 verdict 恒定
   （+ε 收 free、−ε 放 free，分别证 open/closed 恒定）。
4. **覆盖算法** `certified_gate_intervals`：每个探测点产出一段带证书的判定
   恒定区间，贪心覆盖 [0,π)；缝隙用定制 ε=L·宽/2 微扰一击封掉（该修复把
   unresolved 从 38 → 0–2）；事件被挤进宽 ≤ tol 的 bracket。
5. **双侧真值夹层**：outer 系统事件保守偏早、inner 偏晚，真值 ∈
   [outer bracket, inner bracket] 并集——**认证包含真值的事件区间**，均匀
   扫描在任何预算下给不出。

### 第一腿验收（g2_event_validation.json）

- **single_door（解析真值）**：3 门宽 × 3 tol（0.2/0.1/0.05°）共 9 配置，
  **解析事件全部落在认证夹层内**；中点误差 0.001–0.06°。
- **double_door（密集参考）**：认证 1194 切片 vs 参考均匀 3600 切片
  （0.05° 步）：2 个事件全中、无假 bracket、unresolved 6 个微缝——
  **1/3 预算 + 无漏事件证书**。
- 单元测试 TestEvents 3 项（夹层包含解析事件、覆盖完备、认证区间内随机
  抽查恒定）；全套件 **97/97**。

### 诚实注记：夹层宽度的地板

tol 收紧时夹层宽度趋于平台（~0.18–0.32°）——由 outer↔inner 多边形化间隙
决定（ndir=48 的系统间距），不是算法极限；ndir 提高按 ~1/ndir² 收缩。
JSON 里 `uniform_worstcase_err_same_budget` 只计采样误差，**低估了均匀
扫描的真误差**：均匀扫描定位的同样是多边形系统的事件，其对真值误差
= 采样/2 + 同样的多边形偏移（≥ 地板），且无任何证书。同预算下认证方法
不劣于均匀且带证书；预算标度上事件 bracket 只随 log(1/tol) 增长。

### Gate G2 状态

**第一腿 PASS（bench 解析 + double_door 参考）。** 完整关门还差：
- [ ] K2 真实场景窗口的事件认证（同机制接 2D splat slice——需要 K2 侧
      的 perturb 语义：椭圆 splat 的腐蚀 cap 处理）
- [ ] closure-threshold（形态参数 w / 机器人长度）continuation：同一
      扰动认证机制沿 w 轴运行（Hausdorff 速率对 w 为 1/2）
- [ ] 预算深扫：eps ladder 自适应化（当前 ~400 切片/侧有 2–3× 余地）
- [ ] 高阶事件类型（分量分裂/合并，非 gate 型）的覆盖验证
