# 开放问题:薄通道的发现成本 vs 验证成本(w=0.505 案例)

日期:2026-08-13。状态:Round-2 floor 实验剥离出的最小开放问题。

## 设定(可完全复现)

G1 de-aligned:门宽 w=0.505,offset 13mm,tilt 7°;机器人椭圆 a=0.6, b=0.25。
通道几何:最优朝向 θ*=7° 时横向可行半宽 δ=(w−2b)/2=**2.5mm**,角度窗 ±3.73°;
联合可行集是 (y,θ) 中的 2D 区域:|y−轴线| < w/2 − r(θ−7°),沿走廊轴 x 平移不变
(长约 1.2m + 两端漏斗)。真值:构造性 witness 已过保守证书(min 余量恰 2.5mm)。

查询模型:一次 pose 查询返回 free/collide、PW 余量 h、逐侧 h₊/h₋(给定对向分组),
解析梯度 ∇c(已实现)与 ∇h(可实现)可用但未用。

## 测量到的现象(问题本体)

- **验证是 δ⁻¹ 的**:witness 路径 bubble 证书 = 284 次 margin 检查(δ=2.5mm)。
- **发现是 δ⁻² 的**:全知管道内的细分式发现,hybrid floor = 4k/16k/~128k
  (δ=40/20/10mm,≈每减半 ×4);δ=2.5mm 时 128k 内失败。
  瓶颈:细分要求某个 cell **中心**命中 y×θ ≈ 2.5mm×1° 的窗,盲目二分命中成本 ∝ δ⁻²。

**问题:设计查询策略,使"输出一条已认证路径"的总成本从 O(δ⁻²) 降到 Õ(L/δ) 甚至
L/δ + polylog——即发现不比验证贵。**

## 有依据的攻击方向

1. **平衡流形延拓(最有希望,即计划 08 §8.2 的 balanced-contact continuation)**:
   通道中轴局部就是 {h₊(q)=h₋(q)} 的极大 clearance 分支(=走廊的 medial axis)。
   关键观察:**该流形连续延伸到门外漏斗区,那里余量是 O(10cm) 级**——从宽区起步做
   predictor-corrector(沿 x 站位走,固定 x 在 (y,θ) 上牛顿修正 max min(h₊,h₋)),
   逐站追进窄区,绕开 δ-依赖的首次命中问题。预期成本 O(L/步长 × 牛顿轮数),近似 δ-free。
2. **站位角度区间检测**:固定门轴 x,对 min(h₊,h₋)>0 做认证式 1D 区间二分(θ_diagnostic
   图显示该信号就是可行窗)。
3. 文献接口:medial-axis 采样规划(MAPRM 系)、窄通道 homotopy continuation、
   min-type Morse continuation——我们的新料是逐侧 h± 让 medial-axis 追踪变得便宜、
   且有保守证书兜底(false-reachable 结构性为零)。

## 复现入口

- 数据:`results/tables/floor_probe{,_aniso,_hybrid}.json`
- 机器:`src/splatc/baselines/aniso_tree.py`(+ hybrid answer 见 worklog 2026-08-13)
- 诊断图:`results/figures/theta_diagnostic.png`
- witness 真值:`docs/reports/probe_round1_review_reply.md` §D17

## 修订(2026-08-13,第二轮评审后;以下取代正文中的相应表述)

1. **对象改名与修正**:追踪对象不是 {h₊=h₋}(那只给门轴 n=0,无角度约束——round-1
   失败的根源),而是 **contact-clearance ridge**:B=h₊−h₋=0 且公共余量
   M=(h₊+h₋)/2 在 (n,θ) 截面上局部极大(KKT:∇M=λ∇B, B=0);h± 为 PW 界时不得
   称 Euclidean medial axis。方法定名 **Certified Contact-Ridge Continuation**,
   是模块 3 的 gate-discovery primitive,非新 planner。
2. **标度声明降级为条件性**:角度窗 ∝ √δ(已验证:√(2bδ/(a²−b²)) 给 3.78° vs 实测
   3.73°),可行截面 O(δ^{3/2});实测 ~δ⁻²(4k→16k→128k,倍率 4×/8×,非稳定)是
   isotropic center-hit 细分模型的性质。"一切维度耦合表示必败"撤回,改为:
   **依赖 vanishing-measure 管内首次盲目命中的表示与搜索必败**。δ⁻¹ 认证标度需多 δ 复测。
3. **核心指标**:Γ(δ) = C_discover+cert / C_cert-only;成功 = continuation 的 Γ 常数或
   polylog,体积法发散。成本拆 C_seed + C_track + C_cert。
4. **实验矩阵**:δ∈{40,20,10,5,2.5,1.25}mm + 关闭门;六方法(uniform / generic /
   bilateral-v4 / oracle-tube floor / cold-start 站位优化 / continuation),后者与
   cold-start 对照隔离"延拓消除首次命中"的贡献;拟合 C=Aδ^{−α} 而非三点目测;
   诊断量含 Newton 轮数、KKT Jacobian 最小奇异值、active-pair 切换、bubble 半径。
5. **Gate B 拆分**:B1 = w∈[0.51,0.58] 的 residual(不阻塞,照常推进);
   B2 = w→0.50⁺ 的 discovery-gap 收敛(vanishing-measure claim 的核心,非可选)。
   只过 B1 → 只能 claim "contact-guided adaptive representation";过 B2 才能 claim
   "vanishing-measure gate preservation"。
6. **未消失的旧债**:PW 界形式证明、side tags 自动对向聚类(当前实验标注为
   oracle-side-grouping ablation)、C₂ 对称商、跨层邻接(Atlas 全局连接仍需)。
7. **论文主张的目标形态**:已知局部 contact oracle 时,薄通道验证便宜、体积首次命中
   发现昂贵;SplatC-Atlas 以认证式接触脊延拓把发现成本压至接近认证成本。

## 追记(2026-08-13 晚,实验 A/B/C 后):攻击方向 1 的第一个端到端证据

oracle-seed(漏斗口)+ 双向 predictor-corrector + 2cm densify + bubble 证书,在
w=0.505/0.51/0.52/0.54/0.58 全部 CERTIFIED_REACHABLE,总查询 3.3–3.8k,
**在 δ=40→2.5mm 上近乎平坦**(对照 hybrid floor 4k→128k+ 的 ~δ⁻²);cert min 余量
2.21/4.73/9.65/19.73/39.73mm,逐宽度贴解析裕度 (w−0.5)/2。w=0.49(关门)如实拒绝。
corrector 恢复域实测 ±100mm/±15°(单轮 92–96/96,含 w=0.505)——正文"绕开 δ-依赖
首次命中"的机制成立。**同日晚已去 oracle(v2.3)**:250mm 网格候选逐试+polish 验证
seeding、双向行进、reject 时升级 (n,θ) 截面 corrector(mouth 分叉的 lobe 切换,
顺序坐标下降有伪不动点——KKT 观点的实测必要性),六宽度全部无 metadata 认证,
total ~11k 平坦(c_seed=6.9k 一次性占主导)。
**Γ(δ) 已测(同晚,v3.1 单位分离后;`gamma_delta.json`/`.png`,报告
`ridge_round3_report.md` 含 round-3 评审修正附录)**:δ=40→1.25mm 六档全部
认证输出(δ=1.25mm:237 站、20.6k、min 余量 1.14mm),关门 safe abstention。
**口径(评审修正)**:Γ_total 39.6–104.3 / Γ_amortized(扣一次性 seed,
合法性依赖未证明的 reuse)26.3–40.8 / Γ_self 105→21;可靠统计量是窄三档
c_track 与 cert-only 的局部指数(0.819/0.806,R²≈1)与稳定比值 ≈22;
total α=0.17 受 seed 固定项与 c_densify 递减抵消污染,不作主张。cold-start
stationwise ablation(oracle 站位、无站间信息)α=1.09 且窄档绝对成本反超;
hybrid floor 为 censored 参照(精确拟合 2.5,末点预算截断,待同码重跑)。
**结论级别:finite-range evidence consistent with bounded discovery premium
on this instance family**——"发现不比验证贵"在本实例族有限范围内成立,
δ→0 渐近 claim 不做。评审并要求后续以 |Θ_gate|∝√δ 为主横轴复报
(cost / success vs 角度窗测度,直指 Claim C)。
仍开放:跨实例族泛化(P2)、side tags 去 oracle 与 branch-aware gate chart
(P1)、matched-budget uniform/generic(P3=Gate B)、multi-goal reuse(P4)。
数据:`results/tables/ridge_abc.json`;图:`results/figures/ridge_exp{A,B,C}.png`;
站点级失败定位(为何旧管道 CERT_FAIL/station-1 死):`ridge_stations.json` +
`ridge_diag_w*.png` + worklog 08-13。
