# Sprint A worklog

## 2026-08-11（深夜：de-alignment 与密封）

- **[实现] G1 生成器加 `door_offset` / `door_tilt`**：整墙绕门中心刚体旋转 + 平移，墙长随
  倾角自动延长保持密封。解析标签由旋转等变性直接迁移：可通行集为
  `|θ − tilt| (mod π) < Θ(w)`。新增 tests #16（倾斜门中线 vs 旋转解析式）与
  #17（pose 级旋转等变性,精确到 1e-9），17/17 通过。
- **[修正] make_splits 的 sanity check 起初把 multi-goal batch 中的同房间 goal 误判为
  错误**——E7 语义本来就允许同房间 goal。改为：单 goal 必须对侧,多 goal 至少一个对侧。
- **[密封] SplatC-Gates 三层 manifest**：dev 24 / validation 18 / blind 21 episodes。
  blind 含临界区随机 de-alignment、just-unreachable、morphology holdout
  （R_blind_ellipse a=0.5,b=0.3,只在 blind 引用）、极端倾角与 multi-goal batch;
  **不带任何标签**。SHA-256 记录于 `results/manifests/split_seals.json`,密封于任何
  method tuning 之前。
- **[结果] de-alignment 演示——Claim C 现象首次实测。** 全部解析可达的案例中:aligned
  全分辨率正确;offset 13mm 或 tilt 7° 后,w≤0.52 连 fine 都 false-unreachable。两种失效
  机制都被捕获:offset=整个可行 y 窗落在网格线之间(2 分量,窗宽数值与解析对账);
  tilt=走廊内有孤立 free 格但 6-连通链被斜向混叠打断(4→12 分量随分辨率增长)。
  **协议推论:临界区 de-aligned 案例的真值必须走解析标签或自适应细化,均匀网格 oracle
  在这里自己也会失败——这本身就是论文主论点的正面证据。** Gate A 升级 PASS,授权 Sprint B。

按顶层 README 纪律：每步记录**遇到的问题**与**如何解决**，边做边写。

## 2026-08-11

- **[发现] 03 号文档 G1 解析公式隐含 corridor 假设。** 推导 gate 真值时发现：投影条件
  `2r(θ)<w` 只对 thick/corridor 门精确（需 T ≥ 2a 的完全浸没论证）；thin-wall 门的正确条件是
  弦条件 `2ab/sqrt(a²cos²θ+b²sin²θ) < w`，同临界宽度 2b 但角度区间严格更宽
  （例 a=0.6,b=0.25,w=0.7：corridor ±26.7° vs thin ±50.3°）。
  **解决：** benchmark 默认 corridor 门（T=1.2），两公式均写入 problem_spec §5；thin-wall 变体需换标签公式。
- **[决定] h 的量纲。** Perram–Wertheim 接触值是缩放余量（无量纲），不是米制距离。签名与
  相切位置和 hard-set 语义完全一致，采用 `h = sqrt(F_PW) − 1`；米制 clearance 由 checker #2 提供。
  两者符号必须一致（进 tests）。
- **[决定] 墙的 primitive 组成。** 契约要求 scene 为 ellipsoid primitives，不能用解析矩形。
  用 disc 行/列/填充构成墙：corridor 边缘行 r=0.10 s=0.02（包络波纹 5e-5，低于最细网格一个量级）；
  内部填充 r=0.15 网格 0.2 < r√2 ⟹ 墙内无空洞 ⟹ 无幻影连通分量。门楣四角 0.1 圆角，只放松
  角部约束，不影响 mid-corridor gate 真值（必要性论证只用完全浸没时刻）。
- **[决定] rho 的 broad-phase 截断。** rho 只在障碍近邻壳层内精确（quick-accept: d > a+R ⟹ 该 pair
  自由且不参与 min；quick-collide: d < b+R）。远处 rho 取 cap 值。free 符号严格精确；
  gate/clearance 度量都发生在壳层内，不受影响。
- **[发现] 文档生成器把 `\frac` 的 `\f` 写成了换页符（0x0C）**（03 §6.2）。已修复并重建 v2 合订本；
  v2 现在由 `tools/rebuild_v2.sh` 从 canonical 编号文件拼接生成。
- **[结果] 15 个解析测试一次通过**；coarse oracle 三 morphology 行为全部符合预期
  （小圆 1 分量可达 / 长椭圆 1 分量可达 / 大圆 2 分量不可达）。
- **[发现] 最优路径"贴 gate 边缘"现象。** medium oracle 的 rotate-door 最优路径把朝向从 90° 只
  转到 ~25°（解析 gate ±26.7°），穿门时 min ρ 仅 0.023——在 J 度量下少旋转更省代价，最优解
  天然贴住 gate 边界。这是正确行为，但对后续有两个含义：(1) path clearance 指标必须和
  cost-optimality 分开报告，否则"贴边"会被误读成不安全；(2) 若要演示"侧身到 0°"的教科书
  画面，需要 clearance-加权 cost（spec §1 的 λ_clr 项），不要改 hard 语义。
- **[观察] 接触边界的网格混叠。** free-slice 图中墙面相切列出现单像素锯齿（面列波纹 2e-3 m
  vs 网格 0.05 m），属于预期的边界混叠，不影响分量/可达结论；fine 网格下更细。
- **[bug→修复] gate 中面测量的 y 网格没含 y=0。** w=0.505 时解析 gate ±3.73° 但测量为 0：
  临界附近可行 y 集合收缩到 {0}，而 `arange` 起点不对称恰好跳过 0。改为对称奇数点 `linspace`。
  教训：**临界测量网格必须显式包含对称中心**。修复后 14 个门宽全部与解析式吻合（≤0.25°，
  即 θ 测量分辨率）。
- **[重要发现] 对齐网格让 uniform baseline 在薄门上"作弊"。** G1 的门中心 y=0、gate 中心
  θ=0 都恰好是网格点，于是 coarse（dθ=5°、dx=0.1）在 gate 仅 ±3.73° 时依然正确可达——
  均匀网格在这个对齐特例上永不失败（gate 只要非空且包含在网格相位上）。**Claim C 的
  thin-gate 对比要可测，G1 必须加 de-alignment 参数**：门中心偏移、门倾斜（gate 中心角 ≠ 0）、
  或随机相位。已在 03 号文档 G1 处补充参数建议。这正是 pilot 应产出的 benchmark 设计修正。
- **[审计] 用户质询"怎么过的审计"→ challenge-清单复核，抓到两个真实问题并修复：**
  (1) `certify_path` 是密集采样不是保守证书——贴 gate 处采样间隔内体点位移 ~0.02 m 大于米制
  间隙 ~0.01 m，理论上采样间隔内塞得下碰撞。已实现 `certify_path_conservative`
  （free-bubble + 自适应二分），medium/fine 路径均真正闭合（min 余量 9.95 mm / 2.59 mm）。
  (2) 报告写"三 morphology 三档一致"但 fine 只跑过长椭圆——过度声明。已补跑两个圆形
  机器人 fine 档，结论坐实。教训：**报告里的每个"一致"都必须指向一次真实运行的记录文件**。
- **[校验组] 用户要求独立校验 → 建 V1–V8 八项独立复核**（蛮力 checker2 全切片 oracle、
  MC footprint 第三检查器、精确对称性、独立 BFS 分量、手写 Dijkstra、蛮力 gate 复算、
  路径纪律、保守证书）。首轮 12/13 PASS，V4 medium x-镜像抓到 120 个不一致 cell。
- **[发现→修复] 精确相切 pose 的标签由 1e-16 噪声决定。** V4 的 120 个 cell 全部是数学上
  恰好相切的 pose（G1 整数几何 × 整数网格的共谋，如 x=±1.2, θ=0 时鼻尖精确碰墙面；
  median |ρ|=3.3e-16），镜像侧浮点路径不同（sin(π)=1.2e-16、linspace ulp 不对称）导致标签
  翻转不一致。**修复是合同级的**：FREE 判定改为 `ρ > ε_t`（ε_t=1e-9，恰好相切按定义不
  free），写入 problem_spec §2。顺带修正：生成器坐标已改为 k·step 精确对称构造（V1 依赖）。
  教训：**round-number benchmark 几何 + round-number 网格必然制造测度零相切点,标签语义
  必须显式处理它们,不能留给浮点噪声**。这同时是 de-alignment 待办的又一条理由。
- **[发现] 恰好临界宽度 w=2b 是 knife-edge。** medium/fine 在 w=0.500 测到 20 个单-cell
  "free 孤岛"（ρ≈9e-5），位置恰在两个墙 disc 圆心之间——这是 disc 组合墙包络波纹
  （设计值每侧 5e-5，双侧 ≈1e-4）的真实几何，不是 bug；可达性结论不受影响（三档均 U）。
  处理：w=w* 参数点按 03 §7.3 标 `ORACLE_UNRESOLVED`，不进入主评估；同时这验证了包络
  波纹的解析预算。
