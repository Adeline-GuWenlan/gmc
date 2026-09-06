# Ridge Continuation 第三轮技术报告(dump → A/B/C → 去 oracle → 单位分离 → Γ(δ))

> **2026-08-13 修正附录(round-3 外部评审后)**——评审全文另存对话记录;逐项处置:
>
> 1. **[状态判定,采纳]** 机制验证 PASS;可进入 compiler 集成;**正式 Gate B 未通过;
>    Gate C/Method Freeze 未通过;论文主方法 claim 不冻结**。当前代码冻结为
>    **候选 gate-discovery kernel**,非完整 method。方法定名改为
>    **Contact-Ridge-Guided Continuation with Certified Path Output**
>    (弃 "Certified Contact-Ridge Continuation"——被认证的是最终路径,
>    不是 ridge 唯一性/gate 完整覆盖)。
> 2. **[表述收缩,采纳] "metadata-free" 改为
>    "door-geometry-metadata-free under oracle side grouping"**:主管道确实不读
>    door pose/width/wall extent,但 h± 的对侧分组仍是预给 oracle;最终须由接触
>    法向 + 相邻性 + 时序 signature 自动聚类(P1)。
> 3. **[口径错误确认并改正] Γ 范围**:本报告 §0 原文"Γ 有界(26–41)"混用了口径——
>    26.3–40.8 是 **Γ_amortized**(扣除一次性 seed;其合法性依赖尚未证明的
>    multi-goal reuse);**Γ_total = 39.6–104.3**;另补 **Γ_self**(分母=自身输出
>    路径的认证成本)= 105→21。三个口径此后并报,不再单挑有利者。
> 4. **[拟合审计,当场复算一致]** 评审重算与我方复核逐位吻合:窄三点局部指数
>    c_track **0.819**(R²=1.000)、cert-only **0.806**(R²=1.000)、total 0.363
>    (R²=0.991);total 六点 α=0.17 受固定 seed 与 c_densify 递减(3520→1320)
>    抵消污染,R²=0.776,**不作主张统计量**。改用的可靠表述:
>    *最窄三档中 ridge tracking 与 witness certification 局部标度相近(均 ~0.8),
>    c_track/C_cert-only 稳定 ≈22*;结论级别 =
>    **finite-range evidence consistent with bounded discovery premium on this
>    instance family**,不 claim δ→0 渐近。
> 5. **[hybrid α≈2 撤回]** 三点精确拟合实为 **2.50**,且 128k 点是预算截断的
>    lower bound——图注改 "CENSORED lower bound (last pt at budget cap)",
>    不再给 censored 数据标指数;正式 baseline 待 P3 同代码同 checker 重跑。
> 6. **[关门措辞修正]** w=0.49 = **safe abstention**(证书兜住启发式发现,拒绝
>    输出),**不是 closure certification**;"physically closed, correctly not
>    certified" 的图题已改。Gate C 要求与 oracle 一致的 REACH/UNREACH 分类,
>    UNKNOWN 不够。
> 7. **[cold-start 定位修正]** 改称 **oracle-positioned stationwise lower-bound
>    ablation**,只进机制消融,不进 end-to-end 主排名(未组装路径、未同证书)。
> 8. **[P0 处置:实验包版本错位确认并修复]** 评审指出的图/JSON 不一致属实:
>    上传诊断图为 v2.3 版(w=0.505: n=51/2.21mm),JSON 为 v3.1 版
>    (132 站/2.34mm)。已建 `prov.py`:所有结果 JSON 带 provenance 块
>    (script+src 树 sha256、UTC 时间、arm、checker/计费版本;无 git,以内容
>    hash 代 commit),所有图由所读 JSON 自动携带脚注戳;全链统一重跑。
>    本报告正文数字以重跑后带戳文件为准(确定性管道,数值不变)。
> 9. **[五问答复,采纳]** ①截面 corrector 保持 heuristic,认证责任在最终路径
>    证书;要 claim gate 完整性时认证对象是截面 free components + 相邻连通,
>    非 argmax。②m<m_esc/2 修剪应替换为"回退到最后健康截面 component →
>    枚举替代 component → 显式构造并认证 connector → gate graph 加
>    branch-switch 边;失败保留 UNKNOWN"(列入 P1)。③Γ 双分母+amortized+
>    compile+M·query 并报(已实施)。④cold-start 按 ablation 保留(见 7)。
>    ⑤优先级 P0(本附录)→P1(去 side-tag oracle + branch-aware gate chart)
>    →P2(冻结超参,offset/tilt×width×morphology 泛化;防 dev overfitting)
>    →P3(matched-budget uniform/generic,Gate B 决定性实验)→P4(两门
>    多目标 Atlas 演示)→P5(路径质量 polish,降级不急做)。
> 10. **[Claim A–E 判定,采纳]** A 未测试 / B 部分成立 / C 有力迹象未正式成立 /
>     D 未测试 / E 未成立。统一 claim 口径采用评审末段文本(见 HANDOFF)。
>
> 附:evaluation 复核脚本输出(本地复算评审数字)见 worklog 08-13 末条。

日期:2026-08-13(晚)。状态:**round-3 评审已处置(见上附录);等待 P1–P4 推进**。代码为 `splatc_atlas` 当前磁盘版本;
v1 tracker 归档于 `experiments/compiler/archive/ridge_continuation_v1_20260813.py`,
v1 运行记录另存 `results/tables/ridge_{continuation,stations}_v1.json`。
复现入口(全部本地、确定性、无 RNG):

```
PYTHONPATH=src python experiments/compiler/ridge_continuation.py   # v3.1 六宽度 sweep + 站点 dump
PYTHONPATH=src python experiments/compiler/ridge_abc.py            # 实验 A/B/C
PYTHONPATH=src python experiments/compiler/gamma_delta.py          # Γ(δ) 三臂
PYTHONPATH=src python experiments/compiler/{ridge_diag_plots,ridge_abc_plots,gamma_delta_plots}.py
```

## 0. 一句话结论

上一轮评审两次点名的站点 dump 做完之后,一晚上连破五层实现/机制问题;现在
**metadata-free 的 Certified Contact-Ridge Continuation 在 G1 de-aligned
(offset 13mm, tilt 7°, R_long_ellipse)上,δ=40→1.25mm 六档全部输出保守认证
路径**,总查询 11.0k–20.6k;Γ(δ)=C_discover+cert/C_cert-only 有界(26–41,
随 δ 变小不增),对照体积法(hybrid floor)α≈2 且 δ≤2.5mm 时 ≥128k 失败。
关门 w=0.49 如实拒绝;零 false-reachable 全程保持(证书兜底)。
问题文档的成功判据("continuation 的 Γ 常数或 polylog,体积法发散")在本实例族
上达成。**尚不是 Gate B2 通过**:单实例、单机器人,side tags 仍是声明的
oracle ablation,路径质量有已定位欠账(§7)。

## 1. 站点 dump(评审处方第 1 条)——v1 两类失败定位

插桩零额外计费 probe;复现逐字节一致(−17.93/−0.52mm、9 站、c_seed=936)。
诊断剖面 = checker #2 米制 margin、5mm 采样、独立计 c_diag。
图:`results/figures/ridge_diag_w*.png`(当前为 v3.1 版;v1 版机制见 worklog)。

- **w=0.54/0.58 CERT_FAIL**:seed 网格赢家在门中心,`t_dir[0]≥0` 强制只向 goal
  行进 ⟹ 走廊左半段从未追踪;证书 polyline 用 start→seed 直线补,直线在门口偏
  倾斜轴 ~78mm > 裕度(40/20mm),剖面 min −27.5/−48.0mm,差 20.5mm ≈ 裕度差
  (机制自洽)。插站只在站间加密,碰不到该段 ⟹ 余量逐字节不变(上轮之谜解)。
- **w≤0.52 station-1 死**:粗网格(500mm/0.45m/22.5°)bilateral 候选全部
  quick-collide 平台(m=−0.5),argmax 容碰撞候选 + tie-break 扫描序,选中
  jamb 肩部深碰撞位姿;balance 双端同号静默返回原点;θ 三分在 −0.5 平台上盲转。
  疑点"seed 落门右侧"排除。

## 2. 实验 A/B/C(评审处方第 2 条;B/C 的 oracle 轴位姿均已声明)

数据:`results/tables/ridge_abc.json`;图:`ridge_exp{A,B,C}.png`。

- **A seed 审计**:500mm 网格 free bilateral 候选数 w≤0.54 全为 0(w=0.58 仅 1);
  250mm(6.8k q)1–9 个;125mm(51.7k q)~50 个。
- **B corrector 恢复域**:oracle 轴位姿扰动 ±5..100mm/±1..15°(含联合),单轮
  balance 二分 + θ 三分恢复 92–96/96(**含 w=0.505**);迭代 5 轮 94–96/96;
  我实现的 2×2 KKT Newton 反而差(27–69/96)。⟹ corrector 不是瓶颈。
- **C oracle-seed 双向 tracker**:漏斗口 oracle seed + 现役机器双向行进 →
  w=0.505..0.58 全部 CERTIFIED(3.3–3.8k q),余量贴解析裕度;w=0.49 拒绝。
  ⟹ 瓶颈裁决:**纯 seeding + 单向行进**,机器本体健康。

## 3. 去 oracle 化(v2→v2.3),每步由 dump 驱动

- v2(双向 + 250mm 网格只取 free):w≤0.52 唯一 free 候选是**墙面姿态**
  (bilateral 滤波假阳性:两 tag 均入 shell ≠ 夹缝),polish 后双侧读 cap,
  march 终止条件 station 0 即真。
- v2.1(候选按 m 降序逐试,容浅碰撞/弃 −0.5 平台 + polish 后验证 m>0 且
  max(h±)≤0.22):seed 正确,朝门 march 死于 mouth——θ 三分在窄尖峰(±3.7°)
  塌错 lobe;reject 后 retry 平移当前站不验证,站漂入碰撞;step 下限过大。
- v2.2(θ 15 点粗扫定界 + 三分精化;retry 只缩步不动站;5mm step 下限):
  暴露更深一层——**顺序 balance→θ 坐标下降在 mouth 分叉处有伪不动点**
  (被堵 lobe 的漏斗 medial axis,(−0.69,−0.022,169°),m→0),可通行分支在
  (y≈轴,θ≈187°) 隔 lobe 相望。w=0.54 能过只因 gate 窗 ±10.8° 两 lobe 早合并。
- v2.3(reject 时升级**截面 corrector**:θ 扫 × 每 θ balance,取 (n,θ) 截面
  m-argmax——问题文档 ridge 定义的字面实现;跳枝后跳过一次 secant):
  六宽度全绿,total ~11k。

## 4. 单位分离(v3/v3.1)

- 终止条件改 **active-set 语义**:单侧 h≥RHO_CAP−ε ⟹ 该侧 shell 内无墙
  (弃 0.22 PW 代理;0.3>cap 的不可触发 bug 同类根除)。
- 步长律转米制:m_metric = m_PW·(b+R_EDGE)(sound in-shell 因子 0.35),
  step = clip(4·max(m_m,1mm), 5mm, 120mm) ⟹ 站数 ∝ L/δ(δ=1.25mm 用 237 站)。
- balance no-bracket 显式失败(None),二分深 11(0.15mm 分辨率)。
- **v3 首跑回退与修复**:米制小步长让 march 贴近死端 pinch(m=0.0001)后才跳枝,
  densify 在 pinch 站与跳枝目标间直线插值横穿 lobe 封锁区(w=0.505 −1.32mm,
  profile seg 23)——v2.3 大步长 reject 早,属侥幸。修复:跳枝时**修剪死端尾站**
  (m < m_esc/2 的尾巴弹出),跳枝段从健康后撤点出发。修复后六宽度全绿。
- 循环帽 800(200 帽曾把 δ=1.25mm 的 march 在 x=0.407 静默截断为
  "loop_limit"——与 0.3 阈值同类的常量 bug,dump 验证后改)。

## 5. Γ(δ) 三臂实验(评审处方第 4 条)

数据:`results/tables/gamma_delta.json`;图:`results/figures/gamma_delta.png`。
δ=(w−2b)/2 ∈ {40,20,10,5,2.5,1.25}mm + 关门 w=0.49。

| δ (mm) | continuation total (c_seed/c_track/c_densify/c_cert) | min 余量 mm | cert-only | cold-start | Γ_total | Γ−seed |
|---|---|---|---|---|---|---|
| 40 | 11260 (6849/784/3520/107) | 17.18 | 108 | 910 | 104.3 | 40.8 |
| 20 | 11231 (6849/1336/2904/142) | 1.37 | 108 | 910 | 104.0 | 40.6 |
| 10 | 11108 (6898/2190/1848/172) | 9.85 | 158 | 910 | 70.3 | 26.7 |
| 5 | 12449 (6898/3662/1584/305) | 4.68 | 170 | 7336 | 73.2 | 32.7 |
| 2.5 | 15348 (6898/6376/1540/534) | 2.34 | 292 | 18368 | 52.6 | 28.9 |
| 1.25 | 20584 (6898/11390/1320/976) | 1.14 | 520 | 19740 | 39.6 | 26.3 |
| 关门 | TRACK_LOST(拒绝) | — | — | 4/65 站 | — | — |

指数拟合(log-log 最小二乘,C=A·δ^−α):
**continuation total α=0.17;减 seed α=0.33;c_track α=0.77;cert-only α=0.45;
cold-start α=1.09**。体积参照(hybrid floor,不重跑,引自
`floor_probe_hybrid.json`):4k/16k/128k @ 40/20/10mm,α≈2,δ≤2.5mm ≥128k 失败。

读法:(i) Γ 有界且不随 δ→0 增长——发现溢价是常数因子,非发散;(ii) c_track
α≈0.8 ≈ cert-only 的站数标度,即"发现不比验证贵"的实测形态;(iii) cold-start
臂(同一 balance/θ-扫原语、无站间信息)α=1.09,窄档绝对成本反超 continuation
减 seed——**θ/分支连续性的贡献被隔离出来了**;(iv) cert-only 见证 min 余量
逐档恰为解析裕度(40.0/20.0/10.0/5.0/2.5/1.25mm)——分母可信。

## 6. 声明边界(oracle/units/计费清单)

- **不读 door metadata**:v3.1 主管道 seed/方向/终止全部由 h± probe 导出。
- **仍是 oracle 的**:side tags(h± 分侧)= 声明的 oracle-side-grouping
  ablation(全线旧债,最终须接触法向自动聚类);Γ 实验的 cold-start 臂
  (站位 x 网格用墙 extent)与 cert-only 臂(解析轴线见证)是声明的诊断/分母臂。
- **计费**:C_seed/C_track/C_densify/C_cert 全为 pose 查询(side_rho 与
  eval_points 同价记账);诊断剖面(checker #2)独立计 c_diag,不入方法计费。
- **单位**:步长/终止已米制/active-set 化;h± 仍以 PW 报告(界的 sound 换算
  因子 b+R 已对抗验证 50 万构型 0 违例,见 round-1 修正附录 §3)。

## 7. 已知欠账(自报)

1. **mouth 切内角**(路径质量,非可靠性):w=0.54/0.58 未触发截面升级,顺序
   corrector 渐进旋转使 min 余量只有 1.37/17.18mm(解析 20/40);候选修法:
   每站无条件截面 corrector(+~180 q/站)或终点抛光 pass。
2. 单实例、单机器人、单 (offset,tilt);未跑 dev/validation 42 案例族与其他
   G 族;blind 未触碰。
3. cold-start 臂只输出逐站姿态,未组装/认证路径——其成本是 cold 管道的下界,
   对比对 continuation 有利的方向是保守的(cold 全管道只会更贵)。
4. PW 界形式证明、C₂ 商、跨层邻接、NavMesh 复现等旧债不变。
5. seed 网格 250mm 是 exp A 实测的"该实例族最粗可用",尚无跨族自适应策略
   (θ 分辨率主导;候选逐试上限 8 未触及)。

## 8. 供评审的问题

1. 截面升级 corrector(θ扫×balance 的 argmax)是否需要认证性表述(如"截面
   上 m 的下界证书"),还是作为启发式 predictor-corrector 的一部分即可
   (最终可靠性仍由路径证书兜底)?
2. 死端修剪的 m<m_esc/2 阈值是临时的;更原则的做法是否应当在跳枝时对
   "后撤-绕行-前进"段显式插入截面站?
3. Γ 的分母用 oracle 轴线见证是否恰当,还是应改用"给定 continuation 输出
   路径的重认证成本"(两者都已测:后者=c_cert 列)?
4. cold-start 臂的公平性:是否需要给它加"路径组装+认证"以构成完整对照管道?
5. 下一步优先级:路径质量修复 vs 跨实例泛化(offset/tilt 扫描)vs 接触法向
   自动聚类(去掉最后一个 oracle)?

## 9. 文件清单

- 代码:`experiments/compiler/{ridge_continuation,ridge_abc,gamma_delta}.py`
  (+ 三个 `*_plots.py`;v1 归档 `archive/`)
- 数据:`results/tables/{ridge_continuation,ridge_stations,ridge_abc,gamma_delta}.json`
  (+ `*_v1.json`)
- 图:`results/figures/{ridge_diag_w*,ridge_expA,ridge_expB,ridge_expC,gamma_delta}.png`
- 过程记录:`docs/worklog/sprint_B.md` 2026-08-13 各条
