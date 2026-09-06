# Gate B 证据报告 — Contact-Ridge-Guided Continuation with Certified Path Output

日期:2026-08-18(round-8 处置)。文档地位:round-8 评审指令将原单一
"终版"报告拆分为两份——本文档**只承载已通过并关账的 Gate B 证据**
(kernel、P2、Γ(δ)、Gate B-G1 + P3.7、witness amendments、复现链);
chart 层(P4a)的全部结果——包括负结果——在
`sprint_c_p4a_report.md`,与本文档一起随包发布。两份文档均非"终版":
完整 SplatC-Atlas method 尚未形成(见 sprint-C 文档 §6)。

round-8 评审维持的判定:**Gate B–G1 PASS;v4.1 gate-tracking kernel
安全性 PASS 且保持冻结;matched conservative volumetric-family
separation PASS(严格限于所测实现族);论文级 residual HOLD**。所有
数字来自包内带 provenance 的 JSON;两层发布门(§7)全部 exit 0 方可
发布。

## 0. 统一主张(round-7 评审批准口径,round-8 维持)

在 G1 de-aligned single-door family 上,tag-free、**G1-frame-conditioned**
的 contact-ridge continuation 能从局部 pair-contact probes 生成经连续
认证的 thin-gate path(半裕度低至 1.25mm),并在物理关闭实例上安全
abstain(非 closure 证书)。在相同 ROI、相同 [0,π) quotient 与相同预算
下,它显著优于所测试的 isotropic / anisotropic conservative volumetric
cell-tree implementations(P3 历史臂 + P3.7 匹配臂,见 §4)。该结果
通过了 **Gate B 的 mechanism criterion(round-7 正式 PASS)**;尚未
建立:goal-independent Atlas、frame-equivariant compilation、closure
certification、非体积最近邻 residual(sampling/roadmap)、跨 family
泛化与 Gaussian-specific residual。kernel(v4.1)冻结;它是
gate-tracking kernel,**不是**完整 Contact Atlas。

**成本口径限定(round-8 §9,round-10 收紧)**:§4 的 8.3–11.5× 是
**gate-tracking kernel vs 全局体积连通树**的对比,**不是**完整
SplatC-Atlas method 的编译成本优势。完整方法的编译成本目前**没有**可
引用的数字:sprint-C 文档 §6 记录的是 G1 单门 chart+connector 原型
工作量(其构成与未计入项在彼处如实列明),完整 compiler cost 待
goal-free gate discovery 与多门场景实现后才可结算。

## 1. Kernel(v4.1,冻结)

- **P1a tag-free 分组**:`contact_pairs`(逐 primitive (h, 接触方向),
  1 查询计费)+ 最大 circular-gap 冷聚类 + march 时序参考连续性。
  与 oracle 分组在全部 free pose 逐位一致(309/309 沿脊站点 +
  2925/2925 墙域网格);端到端 sweep 零变化。限定:**tag-free within
  the tested G1 primitive organization and contact regime**;真实 3DGS
  decomposition 的鲁棒性测试(置换/旋转/密度/重分块/三面角)未做。
- **G1-frame conditioning(round-6 更正,旧限定语撤回)**:kernel 运行时
  不读 door width / tilt / side tags(零 door-metadata 查询),但源码含
  G1-frame 几何先验常量:出口判定 |x|>0.7、跨墙 span 判定
  min(x)<−0.65 ∧ max(x)>0.65、seed 盒 [−3,3]×[−1.8,1.8]×[0,π)、
  start/goal 左右布局假设。正确定名限定为 **tag-free,
  G1-frame-conditioned**;此前英文限定语中的几何无关含义撤回。整场景
  刚体变换下 kernel 本体不可运行(run_case 内建 G1-frame 场景),该
  限制在案;chart 层的 frame/phase 不变性测试与 gate 完成语义的
  chart-attachment 化(替代全部 x 阈值)见 sprint-C 文档 §3–§4。
- **P1b branch-aware 追踪**:截面 sampled-free components(K=15 冻结扫)
  + lobe-local corrector + **certified local transitions**(跳枝 connector
  按 witness 密度抛光后当场认证,锚点按站 margin 降序)+ 最终路径保守
  证书。正式名:*branch-aware contact-ridge continuation with certified
  local transitions and certified final path output*。**不是** certified
  gate topology:components 是采样对象,birth/split/merge 完整性未认证。
- 六宽度 canonical(`tables/ridge_continuation.json`,含三层计费):
  w=0.505/0.51/0.52/0.54/0.58 全部 CERTIFIED,min 余量
  2.34/4.68/9.85/1.37/17.18mm;w=0.49 安全 abstain。已知路径质量非单调
  (w=0.54 的 1.37mm < w=0.52 的 9.85mm):branch/path selection 未优化
  bottleneck clearance,修法定为 gate-tube 内终局 readout(P5 路线),
  不做每站加密。

## 2. P2:frozen-hyperparameter within-family validation(42 案例)

不是跨场景泛化;dev+validation 全部来自 single-door G1 geometry family。
val 集已因 val_016 分析失去未触碰地位;method freeze 前将启用新 sealed
blind。数据 `tables/p2_generalization.json`(unresolved 真值
reachable=null,schema 防护后重跑)。

- 26 个独立确认可达:13 认证(50%),13 abstain;
- 计入 6 个 witness-resolved:19/32 open cases 认证(59.4%);
- **条件于 tracker 进入追踪:19/20 = 95%**;
- 10 个物理关闭:全部安全 abstain;**false-reachable = 0/42**;
- 终态分解:19 CERTIFIED / 19 NO_SEED / 3 TRACK_LOST / 1 CERT_FAIL。

结论:tracker 本体在进入有效 bilateral regime 后可靠;coverage 瓶颈在
gate applicability detection、开阔区 handoff 与 semantic seed
selection——正是 chart 层(sprint-C)要解决的对象。长椭圆:每测试宽度
至少一例认证,合计 12/14 open 配置(dev_013 NO_SEED,val_016 CERT_FAIL
= 证书如实拒绝域外 jamb 追踪)。kernel 的工作域 = **稳定双侧接触可观测
regime**(与 robot-relative thinness 相关但不等价)。

## 3. Γ(δ)(`tables/gamma_delta.json`)

三分母并报:Γ_total 40.5–106.2 / Γ_amortized 27.2–42.8(扣一次性 seed,
合法性依赖未证明的 reuse)/ Γ_self 107.2→21.6。可靠统计量 = 窄三档局部
指数:c_track 0.747、cert-only 0.806(R²≈1),比值稳定 ≈22;total
α=0.17 受固定 seed 与 c_densify 递减抵消污染,不作主张。结论级别:
**finite-range evidence consistent with bounded discovery premium on
this instance family**;无 δ→0 渐近主张。oracle-tube floor
(`tables/floor_probe.json`)为 **tested-budget bracket**(w=0.58 在所测
24k 档首次成功,更窄宽度至测试帽未成功),非理论下界。

## 4. Gate B-G1:matched-budget(`tables/p3_matched_budget.json` + `p36_fairness.json`)

五个 de-aligned 临界宽度,同代码树、同 checker、同计费,预算阶梯至
名义 131,072(实际 131,240,batch overshoot 如实记录)。

| w | continuation(认证路径) | uniform | generic | contact-v4 | pair-aware |
|---|---|---|---|---|---|
| 0.505 | **15,804** | >131,240 | >131,240 | >131,240 | >131,240 |
| 0.51 | **12,903** | >131,240 | >131,240 | >131,240 | >131,240 |
| 0.52 | **11,614** | >131,240 | >131,240 | >131,240 | >131,240 |
| 0.54 | **11,441** | >131,240 | >131,240 | >131,240 | >131,240 |
| 0.58 | **11,470** | >131,240 | >131,240 | >131,240 | >131,240 |

基线的 `reachable=false` 是 **budgeted false negative**(预算内未建立
保守 FREE-only 连通),非 unreachable 证书;终态全部
start/goal resolved、FREE+AMBIG 乐观连通、failure_reason=ambiguous_cut。

**kernel 级对比口径(round-5 §2 表述;完整方法口径见 §0 限定)**:
相对 uniform,continuation 用 8.3–11.5× 更少的 configuration queries、
1.2–1.9× 更少的 primitive-pair evaluations、2.4–3.6× 更少的
instrumented wall time、8.6–11.6× 更少的 broad-phase hits(G1 家族
实测;相对冻结 isotropic ProbeTree 家族,见下方域口径注记;pose-query
倍数是下界)。w=0.505 绝对值:continuation 15,804 poses / 2.22M pairs;
uniform 131,240 / 2.62M;generic 4.53M;contact-v4 19.69M。表示规模
对比限定为 **single-query working set**:成功的 continuation 保留
~10² 站/截面记录,失败的 ProbeTree 保留 114,870 叶——非 Atlas 级
内存对比。

**域口径注记(round-6)**:ProbeTree 基线在 (7.0×4.6)m × 2π 全域上运行,
continuation 的 seed 盒为 (6.0×3.6)m × π(θ 取 [0,π) 商)——域体积比
≈2.98×,θ 商仅 continuation 使用。因此本表建立的是"相对冻结 isotropic
体积基线家族的优势";symmetry/ROI/θ-商匹配判定由 P3.7 完成(见下)。

**计费边界注记(round-6,拆分已落地)**:continuation 的 pair_ops 与
instrumented wall time 含终局诊断 margin_profile,pose 查询数不含——
边界不一致方向不利于 continuation。core/cert/diag 三段拆分现于
`ridge_continuation.json`(pair_ops_core / pair_ops_cert 字段):诊断占
pair_ops 的 30–42%;干净口径(core+certificate)w=0.505 为 1.55M。
历史含诊断值如实保留并同图展示。

**P3.7 matched conservative volumetric-family separation(round-7 定名;
round-8 维持 PASS、限实现族;`tables/p37_matched.json`,图
`figures/p37_matched.png`)**:全部体积臂获得与 continuation 相同的搜索
域——ROI = seed 盒 (−3,3)×(−1.8,1.8) m、θ∈[0,π) 商(中心对称机器人;
π-周期性经抛弃场景直接重放断言)。四臂:iso uniform / aniso uniform /
aniso generic(|ρ|)/ aniso pair-aware(方法同款 tag-free 线索);
AnisoTree 单维二分(Lipschitz 贡献最大维),深度惩罚取 generation
口径,各臂共享粗层开阔区认证分支;同预算阶梯至 131,072。结果:**四臂
× 五宽度 = 20 主跑全部 NEVER(实际 131,240 / 131,132);kill condition
(任一匹配臂首胜 ≤2× continuation)0/5 触发;pose 口径下界
≥8.3–11.5×**。正控制健康:匹配域宽门 w=1.10 全 2×2 格
({R_long_ellipse, R_small_circle} × {aligned, de-aligned})各臂全部
成功,且普遍优于 P3 未匹配臂(iso uniform 长椭圆 4k vs 旧 32k)——
域匹配确实生效,all-NEVER 非 evaluator 假象。**邻接语义(round-7
公平性修正)**:全部树连通性(_answer / cut 定位 / chart 构造)自
round-7 树起使用 exact face-overlap 邻接——旧 face-center 单探针可能
漏掉 hanging-face 邻居,漏失方向恰是 budgeted-false-negative 结论所
依赖的方向;精确邻接经几何暴力 oracle 单元测试逐叶一致后全链重生成,
本节数字即 exact 邻接口径。cut 定位:失败臂 frontier 门带内仍为 0/0
——匹配域内体积保守认证依旧死于逼近走廊,未及门口;aniso pair-aware
的门带访问随宽度上升(w=0.58:22,706 cells 在带)但 frontier 不进带。
三币种 @cap:匹配臂 pair_ops 3.41M(iso uniform)/ 4.22M / 5.77M /
20.4M(pair-aware)vs continuation core+cert 1.55M(w=0.505)——
pair-op 口径 ≥2.2×(对 uniform,下界)至 ≥13×(对 pair-aware 匹配臂)。
结论(round-7 裁决口径,round-8 维持):在 G1 family、同 checker、同
预算、同域同商下,**separation 相对已测 conservative volumetric
cell-tree 实现族成立(含 anisotropic 与 pair-aware 线索臂);kill
condition 未触发**。这**不是**论文级 residual:candidate 与体积基线解
的任务仍不完全相同(路线模板辅助的 path witness 构造 vs 全局保守连通
性构造,cut 诊断支持此读法);非体积近邻(sampling/roadmap)与
chart-factored connector 基准(F_L/F_R 同给、无门轴/θ 窗)是下一步
生死实验(P3.8),论文级 residual 在此之前保持 HOLD。

**正控制**(evaluator 无罪):对齐 w=1.10——uniform@16k(小圆)/32k
(椭圆)、generic@32k、contact@8k(小圆)成功;w=0.70 小圆在 32k 控制帽
内未过(帽偏紧,如实)。**2×2 de-aligned 宽门矩阵**(P3.6):
uniform 与 generic 在**全部四个宽门格**(w∈{0.90,1.10} ×
{aligned, de-aligned 13mm/7°})均于 32k 成功——de-alignment 本身不阻碍
体积基线,**thinness 被单独隔离为致命因子**。contact-v4 与 pair-aware
在宽门格全部 NEVER(两种 contact-cue 排序各有宽门病理:bilateral 优先
排序系统性压制解连通所需的开阔区细分——如实记录,亦说明 contact 线索
在体积表示中即使在宽门也非普适优势)。

**ambiguous_cut 定位**(P3.6 诊断;exact 邻接口径):失败 run 的
start-FREE-component ambiguous frontier 定位:thin 主跑
(uniform/generic)frontier ~10.4–12.1k cells,**门带内 0 cells**——
131k 预算下保守 FREE 连通甚至尚未认证到 mouth 的逼近段;门带内已评估的
2k+ cells(含 θ 窗内 164–576)全部停留在 AMBIG 且未接入 start
component。cut 不是门口的薄缝,而是整个逼近走廊的体积认证欠账;
pair-aware thin 主跑 frontier 1.7–5.4k cells、门带内 0/0(排序病理,
见下);contact-v4 宽门失败的 frontier 门带占比 22/22–42/42(到了门带
但 θ 分辨不足)。

**信息消融(scalar-clearance,同架构)**:K=8(冻结档)0/6 open widths
认证(seed 选择性丧失:标量 in-shell 滤波头部被墙面候选淹没);K=64
(scalar 宽限/敏感性档,非独立样本)0/6(窄三档能 seed 但 march 全部死
于 mouth);关闭实例两档均安全 abstain。pair-contact 对照 6/6。结论
(限定表述):**within the current continuation architecture, replacing
pair-disaggregated contacts with scalar clearance destroys both seed
selectivity and mouth-crossing reliability**——机制级证据,非
"优于最优 scalar planner"或 Gaussian-specific residual(P5 audit 待做:
vs SDF/SDF+gradient/CDF/凸体 oracle/nearest-feature pairs)。

**pair-aware volumetric 基线**(P3.6,隔离 pair 信息 vs pair 信息+
continuation):方法同款 tag-free pair 信息 + 体积表示、无站间
continuation。结果:五个 de-aligned 临界宽度全部 NEVER(≤131,240),
且 cut 定位显示其 ambiguous frontier **完全不在门带内**(band 0/0;
frontier 1.7–5.4k cells,exact 邻接口径)——pair 排序把预算烧在门带
之外的双侧结构上。结论:**方法同款 pair 信息 + 体积表示 + 无
continuation = 全灭**;与 scalar 消融合并读:在当前两个已测实现中,
仅有 pair 信息(无 continuation)与仅有 continuation 架构(scalar
线索)均未在测试预算内成功——实现级证据,限于所测两实现,不构成一般
性的相互必要性证明。pair-aware ProbeTree 是**诊断臂**(隔离变量用),
不是最强近邻基线;最强基线属 P3.7(symmetry-matched anisotropic)与
chart-factored Gate B 基准(P3.8)。补充公平图
`figures/p36_fairness.png` 含 pair-aware 臂与 2×2 宽门控制矩阵。

## 5. Witness amendments(`oracle_amendments/`)

6 个 ORACLE_UNRESOLVED_GRID 案例:**dual-formulation replay**(checker#1
PW 符号 + checker#2 点-椭圆米制,共享 source tree——非独立实现;真正独立
的 GJK/interval 后端列为后续;amendment JSON 键名同为
`dual_formulation_replay`,round-6 前的旧键名已废)2mm 采样全部零违例,
min 余量 24.93/7.36/9.95/9.93/9.95/9.93mm。原记录未动;amendment 标记
`excluded_from_independent_accuracy_stats`。endpoint contract 字段
(start_match/goal_distance)补入 amendment schema 属后续项。

## 6. 边界与非主张(Gate B 范围)

1. kernel ≠ Atlas:无 multi-goal reuse、无 goal-independent 编译合同的
   实现;chart 层的当前状态(含负结果)与 **Atlas schema 重新打开**
   (round-8 撤销"已冻结"说法)见 sprint-C 文档。
2. sampled components ≠ certified topology;关门 = safe abstention ≠
   closure certificate(Gate C 未通过)。
3. Gaussian-specific residual 未建立;当前定位:**contact-topology
   compiler kernel implemented on Gaussian/ellipsoidal primitives**。
4. 跨 scene-family Gate B 未通过;本报告全部结论限于 G1 family。
5. 禁令(round-5)在案:无 val_016 专属阈值;无 GAP_MIN 调参;seed
   tries 未扩(scalar K=64 为标注的敏感性档,仅在消融臂)。
6. P3.7 = matched conservative volumetric-family separation(PASS,限
   该实现族);**论文级 residual HOLD**——非体积近邻
   (RRT-Connect / HRM / SE(2) NavMesh)与 chart-factored connector
   基准(P3.8)是 headline 前的生死对比。
7. G1-frame 先验在案(§1 注记);kernel 级刚体等变性不可测试直至 P4b
   注入式 compile(scene, robot) 存在;chart 层 frame/phase 不变性测试
   的现状(round-7 FAIL → P4a.1 重测)在 sprint-C 文档。

## 7. 复现与验证

两层发布门(round-6 P0-R2):**第一层** `validate_package.py`(静态:
校验和/JSON/schema/图-JSON provenance/单一 source tree(从快照重算)/
生成脚本哈希重算/输入依赖 preflight/禁用措辞/报告数字对照/负结果
一致性(round-8:records 内冻结 FAIL 记录与正文披露互检,P4a.1 机器
判定必须原样引用于正文))。**第二层**净室复现:在全新环境解包,于
`src_snapshot/` 内 `bash reproduce.sh` 全链重跑,
`compare_reproduction.py` 将复现产物与包内 canonical 表及 amendments
逐数对比(status/verdict/全部查询计数/pair_ops/bp_hits/站数/事件数/
summary 计数精确相等;浮点余量与拟合系数容差 1e-6;host 相关计时字段
除外)。两层全部 exit 0 为发布判据。复现输入随包:dev+validation
episode manifests、42 份 oracle records、split seals。环境:**核心
数值依赖按范围固定**(python 3.11.15 / numpy 1.26.4 / scipy 1.10.1,
Linux x86_64——canonical 数值链由这三者决定);绘图与测试依赖为最低
版本约束(matplotlib≥3.9 / pytest≥8,不影响数值);overlay 实测完整
`environment_freeze.txt` 随包为证据。这不是 conda-lock 级别的全依赖
锁。无 VCS 工作树:代码身份 = `src_tree_sha256`(src/*.py)+
**`code_bundle_sha256`**(round-7:src/ 与 experiments/compiler/ 全部
*.py 按相对路径参与哈希,覆盖实验脚本的传递 import),两者均由校验器
从快照**重算**,不信任 JSON 自报字符串。禁语扫描覆盖 .md 与全部机器
provenance(表 / amendments / 图 sidecars)。第二层通过后,
`cleanroom_reproduction_receipt.json` 随最终包提供(记录被验证内容的
MANIFEST 哈希、bundle 哈希、`verified_content_zip_sha256`(round-8
更名:回执记录的是验收时的 content zip,最终发布 zip 只多回执一个
文件,其 sha256 以外置 `.zip.sha256` 文件随 zip 提供,不在 zip 内)、
host、环境 freeze 哈希、起止时间、三段 exit code 与逐表 sha256;回执
自身是验证后追加的唯一文件,MANIFEST 闭环由校验器复核)。
