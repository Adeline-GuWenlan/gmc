# Round-4 增量报告:P1(去 oracle + gate chart)→ P2(泛化)→ P3(Gate B matched-budget)

> **2026-08-14 P0.5/P3.5 处置附录(round-4 评审后)**——判定采纳:
> Gate B-G1 = PASS-CONDITIONAL,kernel 冻结,不称 Atlas/certified topology,
> 1000× 表述撤回,"budgeted false negative" 术语采纳,P2 改称
> frozen-hyperparameter within-family validation,长椭圆表述改
> "每宽度至少一例认证,合计 12/14 open 配置"。条件项处置:
>
> 1. **[P0.5 合同清理,完成]** gamma provenance arm v3.1→v4.1;
>    `floor_probe.json` 补 provenance;p2 truth schema 防护(unresolved →
>    reachable=None);P3 标签用 actual 查询数(131,240);`_answer` 增
>    goal_resolved / 双连通性(FREE-only 保守 + FREE+AMBIG 乐观)/
>    failure taxonomy(start_unresolved / goal_unresolved / ambiguous_cut /
>    optimistically_disconnected),并注明 reachable=false 语义为
>    budgeted false negative。
> 2. **[正控制,通过]** 同一 evaluator 在 w=1.10 对齐档成功:小圆
>    uniform@16k / generic@32k / contact@8k;长椭圆 uniform@32k /
>    generic@32k(contact-v4 NEVER——round-1 已知标量线索病理,一致)。
>    ⟹ 五个 de-aligned 薄门的全灭是实例难度,非 evaluator 失效。
>    附注:w=0.70 小圆在 32k 正控制帽内未过(该控制预算偏紧,如实记录)。
> 3. **[三层计费,完成]** pair_ops/bp_hits 计数器(纯测量)进 primitives;
>    continuation w=0.505:pose 15.8k / pair_ops 2.22M / bp 30.5k;
>    基线同预算 pair_ops:uniform 2.62M / generic 4.53M / contact-v4
>    19.7M。⟹ 以 pair_ops 计,continuation 仍优于或相当于 uniform
>    (2.2M vs 2.6M)且远优于 generic/contact-v4;"每 pose 查询的信息量
>    差异"不再隐藏(≈140 pairs/查询 vs uniform ≈20)。
> 4. **[gate-band 访问统计(诊断)]** 基线在 131k 内进入门带的 cell 数:
>    uniform ~2.2k(带内含 θ 窗 80–248)/ generic ~1.9k(314–576)/
>    contact-v4 1.5–1.6k(**0–6**)——基线确实到过门带,但 θ 维分辨不足;
>    contact-v4 的 θ 命中近零与其 round-1 病理一致。
> 5. **[same-architecture scalar-clearance 消融(评审 §2.3 关键项),
>    完成——0/14 认证 vs pair-contact 6/6]** 同骨架同冻结常量,仅把
>    (h₊,h₋) 换成标量 rho:K=8(冻结档)六开门宽度全 NO_SEED(标量
>    in-shell 滤波无选择性,墙面候选淹没 m-降序头部);K=64(scalar
>    宽限档)窄档可 seed 进入追踪但 march 全部死于 mouth
>    (`ridge_dead`,x≤−0.58 从未跨墙),宽档仍 NO_SEED。
>    ⟹ pair identity 的两层贡献被分离:seed 选择性 + mouth 处脊条件化。
>    Claim E(residual)首个直接证据;限定:信息替换消融,非最优标量方法,
>    且 vs SDF/CDF/凸体 oracle 的完整 P5 audit 仍待做。
> 6. **[witness 升级协议,完成]** 6 个 unresolved 案例:v4.1 见证 +
>    checker#1(PW 符号)/checker#2(点-椭圆米制)双重 2mm 密集重放
>    (每案 2.7–4.6k 采样)全部零违例,min 余量 24.9/7.4/9.95/9.93/9.95/
>    9.93mm;amendment 写入 `outputs/oracle_records_amendments/`
>    (原记录未动,标记排除出独立准确率统计)。
> 7. 数据:`p3_matched_budget.json`(v2:controls+taxonomy+band+计费)、
>    `ridge_scalar_ablation.json`、`gamma_delta.json`(arm 修正重跑)、
>    `floor_probe.json`(带 prov 重跑)。
>
> 尚余(按评审规划):P4a handshake 数据结构(六类对象)→ P4b 两门多目标
> → P5 Gaussian residual audit(vs SDF/CDF)。

日期:2026-08-13(深夜)。状态:**供外部评审(round-4)**。本报告只覆盖
round-3 评审之后按其 P0–P5 处方完成的工作;round-3 的修正附录与全部处置见
`ridge_round3_report.md` 顶部。全部结果 JSON/图带 provenance 戳(P0 机制),
图由所读 JSON 自动生成。

## 0. 一句话状态

P0(实验包一致性)、P1(side-tag 去 oracle + branch-aware gate chart)、
P2(冻结超参泛化,42 案例)、P3(matched-budget Gate B 实验)完成;
**零 false-reachable 纪律在全部新实验中保持**。方法名沿用评审定名:
Contact-Ridge-Guided Continuation with Certified Path Output(v4.1)。

## 1. P1a:side-tag oracle 移除(fully tag-free discovery loop)

- 新 API `SceneGeometry.contact_pairs`(shell 内逐 primitive (h, 接触方向),
  计费=1 次碰撞查询;active pair identity 属方法合法信息——round-3 §一确认)
  + `bilateral_rho_tagfree`:冷启动按接触方向排序角的两个最大 circular gap
  切分(GAP_MIN=45° 设计期冻结),march 沿线用上一评估的参考方向做时序
  signature 连续性;缺侧读 RHO_CAP;碰撞区返回精确 PW(side_rho 是 −0.5
  quick-collide 标记——tag-free 严格更多信息)。
- **验证**(`tagfree_validation.py`):free pose 上与 oracle side_rho 逐位一致
  (309/309 沿脊站点、时序链;2925/2925 墙域网格、冷);分歧仅在碰撞区语义。
- **端到端**:v4 sweep 与 v3.1 五个认证宽度逐位一致(全部成本/余量/站数)。
  oracle 路径保留为声明消融开关(TAGFREE=False)。
- round-3 表述"door-geometry-metadata-free under oracle side grouping"的
  限定语解除;cold-start 诊断臂与 cert-only 分母臂仍 oracle(声明不变)。

## 2. P1b:branch-aware gate chart + 认证跳枝(v4.1)

- 截面 `section_eval` 输出**全部 sampled-free components**(K=15 冻结扫;
  组件=连续 feasible θ-run,含 θ 区间/m_best/m_min/q_best),按触发器
  seed / multimodal(粗扫免费多峰提示)/ reject / backtrack 记入 chart;
  事件流:lobe_death / branch_switch / connector_cert_fail / section_blocked
  / march_death。
- **图景修正(供评审注意)**:w=0.505 mouth 截面实测是**单 component 经
  薄颈连通**(163°..189°,m_min=0.0017≪m_best=0.070),不是分离双 lobe;
  round-2 的"双 lobe"是坐标下降视角。分离与薄颈两种拓扑 chart 均如实记录。
- **协议**(round-3 五问 #2 的落实,m_esc/2 修剪删除):corrector lobe-local
  化(θ 粗扫只在含来向 θ 的 feasible run ±2 步内取 argmax;跨 lobe 无静默
  旁路);凡截面导出跳变,connector 先按 witness 密度 2cm 抛光
  (`polish_segment`,与主 densify 共用原语)再 certify_path_conservative
  **当场认证**(计费 c_connector);锚点按记录站 margin **降序**尝试
  ("返回最后健康 component"的操作化,无新阈值);全部失败保留 UNKNOWN。
- 迭代过程(三轮 dump 驱动,含"采样连通 component 的直线弦穿出薄颈"与
  "从濒死站起跳贴颈 0.05mm"两个中间失败)全程记录于 worklog 08-13。
- **结果**:六宽度全对,min 余量 2.34/4.68/9.85/1.37/17.18mm 与 v3.1
  逐位一致(质量零损失);窄三档各 1 次认证 branch_switch(connector 余量
  ~10mm);关门 w=0.49 chart 结构化记录死亡(x=−0.62 截面塌缩至单点 m≈0 →
  三次 BLOCKED → march_death)——closure certificate(Gate C)的原料。
  Γ(δ) 全链重测:Γ_total 40.5–106.2 / Γ_amortized 27.2–42.8 / Γ_self
  107→21.6(三分母并报);narrow-3 α:track 0.747 / cert 0.806。
  chart+connector 溢价 ≈ +2(Γ 单位)。
- 未竟(如实):截面间显式 adjacency/birth-split-merge 推导器、component
  contact-signature 字段(数据已含所需信息);多 gate 图属 P4。

## 3. P2:冻结超参泛化(dev+val 42 案例;cn263 直跑)

超参全冻结,run_case 仅参数化 robot_id。数据
`p2_generalization.json`;HPC phase_002。

| 判定 | 数 |
|---|---|
| FALSE_REACHABLE | **0 / 42** |
| certified_true_positive | 13 |
| correct_abstention(全部物理关门) | 10 |
| oracle_unresolved → 全部构造性认证 REACHABLE | 6 |
| abstain_on_reachable | 13 |

三个结构性发现:
1. **"薄"是机器人相对的**:长椭圆 0.55–1.10 全宽度认证(体长维持双侧接触);
   大圆在 w=0.90(其 δ=50mm)认证;小圆仅薄门。kernel 恰在各机器人自己的
   thin regime 工作。
2. **6 个 ORACLE_UNRESOLVED_GRID 临界 de-aligned 案例全部解决**:fine 网格
   (288×185×281)不能定论的案例,tracker 以 ~10k 查询给出保守认证见证,
   余量 7.4–24.9mm 逐案贴解析 (w−2b)/2。见证回填 oracle records 列入待办。
3. **域外区 = 机器人相对宽门**:12 例 NO_SEED(250mm 网格上 bilateral 候选
   为空——宽走廊单侧 shell 看不到对面墙)。这是 Atlas 缺 open-space chart
   的定量证据,非 tracker 缺陷。
   认证案例成本 9.9k–14.4k,跨 regime 平坦。

## 4. val_016 诊断(只读;按无调优纪律未修)

唯一质量失败:椭圆 w=0.80、dy=−0.021、tilt=+12°(val 最大倾角)。dump:
seed(第 6 个候选)落在 jamb 肩部双侧结构(θ≈101°),march 沿 jamb 爬升
(101→132°,两次局部 connector 认证通过、一次 −28mm 正确拒绝),全程未进入
门的朝向带(±34.9°@12°);密化后爬升段一节 −4.72mm,最终证书如实拒绝
(报告值 −1.21 为证书 early-abort 部分观察)。定性:**域外 seed 的脱靶追踪,
可靠性链条全程在岗**;修向与宽门 NO_SEED 同根(gap 双侧 vs jamb 肩部双侧的
seed 区分 / open-space chart),推迟到相应工作项。

## 5. P3:matched-budget Gate B 实验(cn263;同代码同 checker 同计费)

冻结的 round-1 ProbeTree 评估器(uniform / generic-adaptive / contact-v4
标量线索探针),预算阶梯延至 131072(去 censored);对照 = 当前 v4.1
continuation 的认证 total(同代码树,provenance 关联)。五个 de-aligned
临界宽度。数据 `p3_matched_budget.json`;hybrid floor(oracle-tube 下界
诊断)同码重跑 `floor_probe*.json`。

**结果(cn263,`outputs/p3_run.log`;图 `p3_matched_budget.png`)**:

| w | continuation v4.1(认证路径) | uniform | generic | contact-v4 探针 |
|---|---|---|---|---|
| 0.505 | **15,804** | >131,072 | >131,072 | >131,072 |
| 0.51 | **12,903** | >131,072 | >131,072 | >131,072 |
| 0.52 | **11,614** | >131,072 | >131,072 | >131,072 |
| 0.54 | **11,441** | >131,072 | >131,072 | >131,072 |
| 0.58 | **11,470** | >131,072 | >131,072 | >131,072 |

- 三基线在五个临界宽度上**全部 NEVER**(≤131k 无正确 REACHABLE),含最宽
  δ=40mm 档;差距 ≥8–11×,且基线侧被预算截断(真实差距下界)。
- **对比方向对基线有利仍全灭**:基线只需连通性判定(无需认证路径),
  continuation 输出的是保守认证路径。
- 表示规模:基线终态 114,870 叶 vs continuation ~10² 站点+截面(≈10³×)。
- false-unreachable:基线在全部 (w, budget≤131k) 组合上持续 false-unreachable
  (实例族全部解析可达);continuation 零 false-unreachable(五宽全过)。
- **iso 机器 oracle-tube floor 同码重跑逐位复现 round-2**(tube 0.2:
  24k/96k/>128k @ 0.58/0.54/≤0.52)——完美调度下该表示机器在最容易档的
  下界(24k)已超过 continuation 全管道(11.5k)。hybrid floor(aniso 中心
  标签机)未在本轮重跑,round-2 引用维持 censored 标注。
- 判定建议:Gate B 的冻结判据("contact-guided 在至少一个临界 gate 上优于
  matched-budget uniform + generic adaptive,同评估器")在**全部五个临界
  gate** 上满足,差距为下界意义上的 ≥8×。是否正式翻转 Gate B 状态由评审
  判定(见 §7 问题 3)。

## 6. 文件清单(全部带 provenance)

- 代码:`experiments/compiler/{ridge_continuation(v4.1), tagfree_validation,
  p2_generalization, p3_matched_budget, gamma_delta}.py` + plots;
  `src/splatc/gaussian_geometry/primitives.py`(contact_pairs /
  bilateral_rho_tagfree);archive/ 各版本
- 数据:`ridge_continuation.json`(v4.1 canonical)、`ridge_stations.json`
  (chart/events)、`gamma_delta.json`、`p2_generalization.json`、
  `p3_matched_budget.json`、`floor_probe*.json`(重跑)
- 图:`ridge_diag_w*.png`(含 chart 截面条)、`gamma_delta.png`、
  `p3_matched_budget.png`
- HPC:phase_002(P2)/ p3_run.log;cn263 直跑(队列拥挤,用户指示),
  规范照旧(rsync 镜像、overlay env、bash -lc、STATUS/phase_logs)

## 7. 供评审的问题(round-4)

1. P2 的域边界(机器人相对宽门 NO_SEED)是否如我们判断应由 open-space
   chart 处理(P4 的一部分),还是 seed 判据本身应扩展?
2. 6 个 oracle-unresolved 案例的构造性解决,可否作为 oracle 记录的正式
   补全(见证回填 + 标签升级为 REACHABLE-by-witness)?
3. P3 结果(见 §5)是否满足 Gate B 的正式判据;若满足,Gate B 状态可否
   翻转为 PASS(限 G1 de-aligned 家族)?
4. val_016 的修向归类(seed 区分 vs open-space chart)哪个优先?
5. P4(两门多目标 Atlas 演示)开工前,chart 数据结构还需要什么最小补全
   (adjacency 推导器?signature 字段?)?
