# Sprint-C P4a 报告 — OpenChart 层:两轮负结果披露与 P4a.2 返工

日期:2026-08-20(round-10 处置)。文档地位:承载 chart 层(P4a)的
全部结果——**首先是负结果**:round-7 的 chart-count kill FAIL,与
round-10 对 round-9 机器判定的**撤回**。Gate B 证据(已 PASS、关账)
在 `gate_b_evidence_report.md`。两份文档均非"终版":完整 SplatC-Atlas
method 尚未形成(§7)。

**Atlas schema 状态:OPEN**(round-8 撤销冻结,round-10 维持)。

## 1. 负结果记录一:round-7 chart-count kill(树 c9cd4676375b)

**P4a chart-layer rigid kill (15k fixed budget): FAIL** —— wide door
w=1.10 在固定 15k 预算、整场景刚体变换下,chart 连通性随坐标系翻转
(五旋转 major charts 2/1/1+debris/2/2)。冻结记录:
`records/p4a_rigid_charts_round7_15k_FAIL.json`。该轮同时暴露报告
"待做"掩盖已跑负结果的流程失职;validator 检查 7b 自 round-8 起把此
类失职变为机器拦截。

## 2. 负结果记录二:round-9 机器判定撤回(树 10d00fb69494)

**round-9 P4a.1 machine PASS: WITHDRAWN (FAIL-CORRECTABLE)**(round-10
裁决)。冻结记录:
`records/p4a_semantic_invariance_round9_WITHDRAWN.json`。撤回理由,
全部经复核证实(逐位复现评审计数):

1. **域证书不 sound**:旋转场景的 workspace 被替换为 AABB,checker 的
   box margin 对 AABB 计算;domain 判定只约束**机器人中心** footprint
   在旋转矩形内——无人证明**椭圆本体**在真正旋转后的 workspace 内。
   实测(wide R30 60k):11,116 个成员 cell 中 **4,672** 个中心位姿的
   椭圆本体越出旋转矩形,**5,860** 个 cell 存在越界采样点。
2. **portal 编译 goal-conditioned**:round-9 harness 先定位当前
   start/goal 所在 chart、只对这一对尝试 portal——违背
   compile-once/goal-independent 合同;其 164 checks 也只是该对的
   筛选成本。
3. **attachment 测试没测 attachment**:所检查的 witness 首末点即
   start/goal(本就是 chart test 前提);真正的 ridge 端点站
   (x≈−1.103/+1.146)locate 均为 None,witness 中段 170 个路点全部
   不在任何 certified chart;且 witness 由冻结 kernel 生成,上游因果
   依赖 x 阈值——round-9 报告的"无 x 阈值参与"表述过度,撤回。
4. **计费累计**:identity/相位变换共享 scene 对象,pair_ops 跨
   run/跨预算累计(R0 三档 226,856/948,992/1,671,128 为累计值);
   另 round-9 报告的假 portal 统计分母有误(把全部 81 个 build 当了
   分母;实际 thin 0/27 builds、0/216 attempts;thin+sealed 0/54、
   0/432——wide 建立过 12 个 connector,更不该并入)。
5. **连接器非分类器**:直线 connector 的宽度扫描非单调(de-aligned
   长椭圆、level-2 锚点:w=0.58 认证成功 margin 7.27mm,w=0.70/0.80
   失败,w=0.90/1.10 成功),且旧候选枚举锚定单侧导致
   portal(A,B)≠portal(B,A)(w=0.62 实测)——"OpenPortal=宽接口 /
   拒绝=contact gate"的读法是锚点启发式伪象,撤回。

## 3. P4a.2 返工(逐条对应 §2;全部有回归测试)

1. **Orientation-aware 域 support 证书**(`domains.py` +
   `chart_builder.py`):在 domain 局部系内按朝向相关 support
   half-widths 计算四边余量 m_dom;m_dom 对认证运动度量
   D=|dxy|+a_max|dθ| 1-Lipschitz,故成员资格增加第三个条件
   m_dom(center) > r_cell(同一 Lipschitz 半径);不足的 FREE cell
   继续细化。region 序列化双 slack(障碍 + 域)。修复后 wide R30:
   成员 11,116→1,464 个 sound cell,越界 **0/0**;identity 链数字
   逐位不变(Gate B 零扰动)。portal/attachment 在带 domain 的问题上
   叠加**域 bubble 证书**(同度量、独立闭合)。
2. **Goal-free 对称 all-pairs 连接器枚举**(`assemble.py`
   `enumerate_portals` + `portals.py`):编译调用链**无任何位姿参数**;
   全部 major chart 无序对逐一尝试;候选按无序对 canonical 化 +
   双侧 θ-列并集,(A,B) 与 (B,A) 按构造执行同一计算(w=0.62 回归
   测试);portal 证书携带 `certified_tube_radius_m`(1-Lipschitz
   论证:折线周围半径 m*/2 的 tube 内余量 ≥ m*/2 > 0)——单条正
   margin 路径不再自称 "broad portal",类别名改为
   **generic_certified_connector**(证明来源,非几何分类;contact-
   structured connector 是另一证明来源,两者不互斥)。
3. **真 attachment–branch–attachment 链**(`assemble.py`
   `build_gate_chain`):从 kernel witness 显式拆出并**全新认证**:
   chart 内成员 cell 中心 → ridge 端点站的两条 attachment 折线
   (保守 bubble 证书)、端点站间 dense ridge 折线的独立重认证
   (branch witness,margin 2.336mm)、端点站实测 bilateral 接触
   (h± 与接触方向)构成 ContactSignature/SectionComponent。装配为
   AtlasEdge(source_attachment/witness/target_attachment)。**如实
   限定**:witness 本身仍由冻结 v4.1 kernel 在 G1 帧、固定任务、
   x 阈值条件下生成——本轮建立的是 attachment 语义与验证链,
   goal-free gate **discovery** 是 P4b 合同,此处不主张。
4. **Atlas 实例 + 语义 validate()**(`atlas_types.py`):新增
   validate_region(slack 数量/正性、双 slack、邻接索引合法且真正
   face-adjacent——纯网格数学复核)、validate_portal(端点几何上
   位于两 chart region 内、tube 半径、checker 绑定)、
   validate_attachment(chart 侧端点在 region 内、component 侧端点
   匹配锚位姿)、validate_edge_chain(attachment 锚定 branch
   **端点** component、witness 连接同一对端点)。Atlas 增加
   scene_hash/robot_hash/domain_hash/checker_id 身份绑定,证书
   checker 不匹配即违规。round-10 的三个对抗变异(乱几何
   attachment、伪造 portal、负 slack+越界邻接 region)现全部被抓,
   连同假邻接边、未绑定证书、断链共六类变异测试。
5. **序列化断链合同**:serialize → reload → validate → query 全程
   无 live tree(tests/test_assemble.py);θ-wrap chart_connect
   专项测试。
6. **Per-run 计费**:每 build 全新 scene 对象,pair_ops/bp_hits 为
   本 run 增量。
7. **预算阶梯至收敛**:150k 顶档在 max_level=3 下**保证**收敛
   (最坏 level-3 波 ≈ 8×全部 level-2 cells ≈ 88k;实际花费即收敛
   成本,典型 45–60k)——判定基于 converged 停机(构建确定性 ⇒
   预算无关终值)或顶两档一致;域修复使 level-3 波变大后,R60 类
   变换在旧 60k 顶档下差 ~100 查询无法进 level 3,这正是 round-9
   单预算判定不可靠的又一例证,如实记录。

## 4. P4a.2 实验(`tables/p4a2_semantic.json`)

命名按 round-10 §8.5 收缩:所测为 **single-query chart-layer
reachability consistency under the tested transforms and budget
ladder**——不是 graph isomorphism,不是完整方法验收(均 HOLD)。
3 门型 × 9 变换 × 阶梯 {15,30,60,150}k,编译段零位姿输入。

机器判定(validator 7b 互检):
**P4a.2 chart-layer reachability consistency: PASS**

- thin:全部变换 2 major cores、0 generic connector、收敛判定分离
  一致;**identity Atlas 原型**:goal-free 编译(charts + all-pairs
  connector 枚举)+ 真 gate 链装配,语义 validate() 零违规,
  序列化查询经 `g0_e` 路由,**10 组随机 query 位姿对下 atlas hash
  与 compile 计数零漂移**(pose-independence 回归)。
- wide:全部变换在收敛判定下 reachability-connected;9/9 逆变换
  query 在恒等帧对原场景复证通过。
- sealed:全部变换 2 cores、0 connector。

## 5. P4a.3 sealed 验证集(`tables/p4a3_validation.json`)

冻结种子(20260820)、开发期从未见过的轴:宽度阶梯 × 3 形态
(+plug 控制)、**30 个随机整场景刚体变换**(thin+wide)、10 个随机
网格原点相位、max_level {2,3,4} 敏感性、offset/tilt 网格。全部
连通性判定档位保证收敛(150k;lvl-4 500k)。每个 build 附**独立
soundness 审计**(builder 证书路径之外,逐成员 cell 采样 4 角 × 3θ
对域 support 余量)。

**机器判定(validator 7b 互检):P4a.3 sealed validation: PASS**

判定输入(包内 JSON `verdict_inputs`):soundness 违例总数 **0**、
plug 门 connector 总数 **0**、thin 意外连通 **0**、wide 意外不连通
**0**。中间宽度的 connector 出现性为 characterization 数据
(`connector_ladder_by_robot`),**不作为验收判据、不据此分类接口**。
补充观测(如实记录,非主张):长椭圆在**收敛档(level-3 分区)**的
阶梯规整——w≤0.58 无 connector、w=0.62–0.80 双核 + connector、
w≥0.90 门廊自体积认证并入单 chart;round-10 抓到的非单调现象
(w=0.58 在 level-2 锚点下认证成功而 0.70/0.80 失败)在收敛档消失,
确证其为锚点分辨率伪象(§2.5)——这不改变"connector 是证明来源、
非宽度分类器"的定位。

## 6. 工作量口径(round-10 §8.2 收缩)

不存在可引用的"完整方法编译成本"。可引用的是 **G1 单门
chart+connector 原型工作量**(identity thin,收敛档,per-run 计费):
charts ~17.5k 查询 + all-pairs connector 筛选(拒绝亦入账)+
gate 链构造与认证 ~0.9k checks,合计见表内
`atlas_prototype_identity_60k.compile_query_count`。**未计入**(因此
不构成 compiler cost):goal-free gate discovery、多门场景全部
branch、query backend、多目标摊销。kernel witness 的 15,804 属
kernel 级数字(Gate B 文档),其生成依赖固定任务与 x 阈值。

## 7. 边界、红线与下一步

1. 本文档建立的是 chart 层 reachability 一致性 + 序列化/验证合同
   (G1、所测变换族、收敛档),不是 kernel 等变性(P4b 前不可测)、
   不是跨 family 结论;**论文级 residual 维持 HOLD**。
2. 方法红线(round-10 确认):compile 不接收 start/goal、不读
   x slab 阈值;单条直线路径不等同 broad topology(tube 证书 +
   证明来源语义);不得靠调常数令个案通过(收敛档规则为此设);
   两门演示禁做直至 attachment 与 compiler 合同完整(P4b)。
3. 下一步:**P3.8 chart-factored connector 基准**(F_L/F_R 同给;
   contact/scalar continuation、generic connector、RRT-Connect、
   local PRM/bridge、HRM 或 SE(2) NavMesh 至少其一;kill 2×)→
   **P4b** 注入式 compile(scene, robot) 硬合同(无任务编译、两门两
   route、hash 不变、全部 transition 预存在、双路径认证)。
