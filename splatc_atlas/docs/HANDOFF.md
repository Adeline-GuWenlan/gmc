# HANDOFF — GMC current closure（2026-09-04）

- **权威入口**：当前完整状态见 `gmc/HANDOFF.md`，逐模块吻合边界见
  `gmc/CONFORMANCE.md`。完整 Implementation Guide GOAL 仍未关闭；冻结、
  non-blind Atlas G1 fixed-translation scoped claim 已由正式 HPC `17653106`
  关闭。
- **当前证据**：本地 353/353；HPC `17653106` 亦为 353/353，并通过 CLI
  toy。schema-v4 local run
  `gmc/outputs/door_run_local_final_budget_20260903` 为 safe 72n/74e、possible
  128n/138e；query `(-2,0,0.3)→(2,0,0.3)` 为 `REACHABLE`，独立 verifier
  `certified=True`，clearance LB `0.02171590806152206`。query proof 含 3 个
  独立复验 segment，query artifact `missing=[]`；compile manifest 仍诚实写
  `i7_complete=false`。
- **已补能力**：theorem pair/interval/full-S1 cover、fixed-translation
  connectivity sandwich、theorem-only global `UNREACHABLE`、query-relevant
  atomic refine、hard budgets、failure artifacts 与 invariant
  `INTERNAL_ERROR` 均已实现。P3 不再是纯 L/M/R sampling，P4 也不再是
  query-only abstain。
- **仍不能主张**：generic arrangement/event-topology completeness、一般
  free-final-orientation SE(2)、route classes、query-many/break-even、real
  3DGS、full I7、完整 `M_dyn`/local-steering M9。
- **I7 边界**：pruning/fixed-direction 有语义 replay，interval tree、components、
  lineage、witness、query proofs 已序列化；但 4 个 full-run independent replay
  roles 尚缺且显式 fail-closed，所以不能写 I7 complete。
- **P5 当前证据**：HPC `17580448` 完成；BVH N=8192 为 1024 retained/7168
  pruned 且与 flat 一致；incremental N=128 为 180 support calls、14 union，
  full 为 local 23040/HPC 23092、254 union，geometry 语义一致。equal-budget
  两臂均 6 cases、12/12 events、0 spurious、788 calls，结论为 **TIE** 且
  `formal_closure_evidence=false`，不支持效率优势。
- **最终 bug tranche**：除 x86 outer undercoverage、Accelerate GEMM、I7 role
  alias、formal-profile bypass、query/connectivity budget races 外，又修正了
  evaluator 的 leaf-resolution/budget-completion 混淆，并把 strict-convex
  containment 换成 exact fan search；不安全的 fixed-precision bulk-union
  shortcut 已完整回滚。新攻击均进入 353 项回归。
- **正式闭环**：`17653106` 为 `COMPLETED 0:0`，日志以 completion marker
  结束；正式 JSON 为
  `gmc/artifacts/atlas_g1_full_gate_intervals_hpc_17653106.json`。top gate 与
  42→16 两阶段均 PASS；初始 42/42 containment，精炼 16/16 acceptance；14
  个 partial 为 56/56 events 与 56/56 brackets，最大宽度
  `4.921875000000009° < 5°`，0 spurious、0 false-open/false-closed、0 budget
  exceed。`17585021`/`17620029` 是保留的有效预算拒绝；`17652860` 仅为
  `dev_016` diagnostic。
- **Atlas 关系不变**：`splatc_atlas` source/data/oracle/bench 保持冻结，只作
  外部 input/truth；不得调用 legacy `splatc.gmc` 生成 GMC 方法输出或真值。

---

# HANDOFF — GMC v0（2026-08-26 历史快照：41/41，HPC 后由 17461675 关闭）

**用户设定 GOAL**:按 Implementation Guide v0.1 docx 全部实现(不作防御
编程)→ 数学/模块吻合检查 → HPC toy 测试跑通。

- **新独立仓 `gmc/`**(guide §2.1 骨架原样):src 2506 行 + tests 717 行,
  conda env `gmc`(py3.12.13/shapely2.1.2/scipy1.18.1/networkx3.6.1)。
  M0–M9 全模块 + 三值 query(REACHABLE/UNREACHABLE/UNKNOWN)+ 双图
  M_safe/M_possible + witness 认证 + 独立 verifier + CLI 六命令 + artifact
  manifest。**测试 41/41**(unit18/theorem10/integration13,含四集合
  maximal simplex、boundary pinch、单门解析事件隔离、sealed door
  UNREACHABLE cut 证书、独立 SLSQP oracle 等价 200 构型)。
- **CLI demo(guide §16.3)通过**:synth→compile(safe 72n/74e)→query
  (θ=1.57 难例,REACHABLE,clearance 0.0172)→独立 verify certified。
- **吻合检查 `gmc/CONFORMANCE.md`**:公式逐条对照全过;8 处偏差
  (D1–D8)全部有 guide 内依据或实测反例支撑;未实现项均属 guide 自身
  的 P5/延后范围。
- **实现过程锤出来的要点**(详见 CONFORMANCE D1–D8):nerve 团枚举在
  稠密场景爆炸(默认关);GJK 定向迭代负界循环 → 批量角度扫描;
  safe/possible 图命名空间必须分离(索引碰撞曾把 sealed door 左右房
  连通);删/罚共享事件桥毒死宽角路线 → Yen 枚举;lift 锚点 DP
  (腐蚀公共区各连通片);verify 与 lift 的 floor 判据必须一致
  (两个合法下界事后比较会拒真)。
- **HPC**:代码经 base64-inline(login 节点 exec/sftp 通道被拒,
  仅 PTY `-tt` 可用——新增 workaround 记录)传至
  /scratch/sy2366/Project/gmc;toy 测试 sbatch **17445937**
  (compute+cair,overlay+容器内 miniconda 幂等 setup + pytest 子集 +
  CLI demo)运行中。
- **GOAL 关闭(08-26)**:HPC toy 测试通过——作业 17461675:pytest 子集
  28/28 + CLI demo(REACHABLE/clearance 0.0809/独立 verify certified)。
  HPC 侧踩坑记录:singularity 3.6.4 无 overlay create → 用
  /share/apps/NYUAD5/singularity/4.2.0 绝对路径;sbatch 用 -l shebang 会
  被 profile 吞(0 秒空完成);conda 26 named env 会落到绑挂载 $HOME →
  必须 -p 显式前缀且显式装 pip;login 节点 exec/sftp 通道长期不稳,
  仅 PTY -tt 可用,文件传输用 base64-inline。overlay:
  /scratch/sy2366/Project/gmc/gmc_env.ext3(env 已缓存,后续作业直接用)。
- 当时下一步（P5 现已完成，见本文顶部）:① P5(BVH/增量/benchmark runner,guide §15-16);② 与
  splatc_atlas 资产关系待用户定(gmc 为 guide 独立参考实现;atlas 的
  oracle/bench 可作 P2 外部 ground truth);③ flaky 测试防线:其余共享
  RNG 的测试可逐步改确定性构造。

---

# HANDOFF — SPLATC-GMC(2026-08-26:设计手册 v1 接管,theory-first 转向)

**用户修订版设计手册导入为权威文档 `06_GMC_DESIGN_MANUAL_v1.md`**,总控
升 v3.1。要点:三值认证逻辑(M_safe⊂M_true⊂M_possible 双图夹逼)、
支撑函数为唯一 GS-native oracle(overlap 语义降级 surrogate,与 M1
v1→v3 的实测教训一致)、Theory Gate 纪律、模块 M0–M9/阶段 P0–P5、
强制反例测试。我们的 keyhole θ=90 发现被收编为 workspace-boundary
event + T-Event-02;经典 CG 文献缺口补齐;真实 3DGS 推至 P4(K2 现有
结果 = 前哨证据,P4 重跑归档)。

**资产映射**:slice 双侧=C⁺/C⁻(P1 已过);切线多边形=O⁺;
events.py 扰动认证=Theorem 1 构造性充分条件候选(可反哺手册);
K2 事件认证的 perturb 语义由支撑函数路线直接解决(支撑值+ε 即缓冲)。

**队列重排(theory-first)**:P0 oracle 接口+10⁵ 等价测试 → O⁻+方向
加密 → nerve maximal simplex+反例 → Lemma 1–3/Prop 1 正式化 →
triple-event bench → M6 双图(P3)。注意 P4 预注册决定:K2 无 mesh
真值,κ/ρ 标定需 Habitat-GS 补或 K2 降级 demo。

---

# HANDOFF — SPLATC-GMC(2026-08-26:G2 事件层落地,第一腿 PASS)

- **`src/splatc/gmc/events.py`:认证 gate 区间算法**——刚性旋转系统精确
  Hausdorff 速率 L=max 顶点半径 + "disc 半径即 Minkowski 缓冲"免费扰动
  (`build_slice(perturb=)`)⇒ 每探测点一段带证书的判定恒定区间,事件挤进
  ≤tol bracket;outer/inner 双侧运行给出**认证包含真值的事件夹层**。
- **第一腿验收 PASS**(`g2_event_validation.json`):single_door 3 门宽×
  3 tol 解析事件全部落在夹层内(中点误差 0.001–0.06°);double_door
  认证 1194 切片 vs 均匀参考 3600:全事件命中、零假 bracket——1/3 预算
  +无漏事件证书。测试 97/97(TestEvents 3 项新增)。
- 诚实注记:夹层宽度地板由 ndir=48 多边形化间隙决定(~0.2°,随 ndir²
  收缩);均匀扫描同样背负该偏移且无证书(worklog gmc_G2.md 详述)。
- 下一步(G2 完整关门):① K2 真实场景事件认证(splat slice 的 perturb
  语义);② closure-threshold 沿形态参数 continuation;③ eps ladder
  自适应(预算再降 2–3×);④ 非 gate 型事件覆盖验证。

---

# HANDOFF — SPLATC-GMC(2026-08-25 深夜:Gate G1 PASS——slice 层对冻结 oracle 全绿)

- **`src/splatc/gmc/slice.py`**:hard-support Minkowski 语义固定 θ slice
  (disc-group 原型⊕平移广播,25ms/slice;双侧多边形化两侧认证;witness
  由冻结 checker 认证)。
- **Gate G1 验收全绿**(`g1_slice_validation.json`):冻结 bench 4 门宽×
  3 机器人 864/864 ok;新构造 double_door/u_shape/keyhole 共 216 切片
  0 fail(6 certified_conn=GMC 构造性证书 vs oracle 网格碎裂,2 sliver);
  witness 失败 0;gate 三明治 4 门宽×720θ 零违规,open fraction 对解析
  真值差 ≤0.003。**判据 P/R=1 达成,Gate G1 PASS。** 测试 94/94。
- **真发现:workspace 语义错位**——oracle free 含"机器人整体在
  workspace 内"(support_half_widths 腐蚀),slice 初版只约束中心,被
  keyhole θ=90 暴露(冻结 bench 因封边掩蔽)。已修+回归测试。教训:
  自造验收族的价值恰在打破冻结族的隐式约定。
- 下一步:**Stage G2 开工**(gap functions、event bracketing、certified
  subdivision;arrangement 级 gate 几何提取作为 event tracker 输入);
  margin 引理正式化;K2 pilot 迁移 src API。

---

# HANDOFF — SPLATC-GMC(2026-08-25:Stage G1 开工,M1 保守聚类三轮收敛;整层 sweep 进行中)

- **M1 聚类 pilot 收敛于 v3 构造**(door_A/L24,对照 720θ 未聚类参照):
  桶内成员并集的 24 方向支撑切线多边形 ⊕ 机器人椭圆(凸多边形精确
  Minkowski)。**leak=0、false_open=0(构造性保守)**,gate 一致率 0.889
  (分歧全为保守向),primitive 30×降、churn 940×降(10⁷→1.2e4)、
  切片 22× 提速。v1 证实"场景层包含经卷积不保序"(总控 §5.3 margin
  引理缺口的实测),v2 修数学但椭圆拟合过肥,v3 弃椭圆用切线多边形。
  过程 bug:Minkowski 边合并角度未归一化 [0,2π) 产生自交多边形。
  详 `docs/worklog/gmc_G1.md` + `results/gmc_h2/m1_cluster_pilot*.json`。
- **架构定型:两层认证层级**——合并层做全局结构(说 free 必真),
  原始成员层只在 gate 邻域按需细化(细化单调打开)。
- **整层 K2 sweep 完成 → Gate G-H2:PASS**(34 活跃瓦片×720θ:S3 中位
  36/瓦、步占比 0.05、min 0/max 100;事件密度热图
  `figs/k2_event_heatmap.png` 显示事件精确局域于墙角/门洞/桌群,开阔区
  near零——output-sensitivity 直接图证;全楼合计 1202 事件 vs dense 720
  层全域重建)。三个预注册退出条件均未触发。canonical HPC 重跑排入
  论文判定链待办。
- **细化层 + 参数扫描完成(08-25 续)**:两层协议(coarse 开=认证开;
  coarse 关→门廊 patch 内换原始成员复算)达 **door_A/L24 一致率 1.000**,
  L32 两窗 0.997/0.989(残差=真路绕出固定 patch,生产设计:patch 自适应
  生长至不动点);false_open 三配置全零;窗口级提速 2.1–3.9×(下界)。
  参数扫描:一致率对桶参数不敏感(0.86–0.89),false_open 恒 0,
  默认 (cell 0.5, bins 8)。`m1_refine_pilot.json`/`m1_param_sweep.json`。
- **代码提升完成(08-25 三):`src/splatc/gmc/`**(geometry 凸核/cluster
  M1/hierarchy 两层查询含 gate_query 自适应生长),`tests/test_gmc.py`
  12 项,**全套件 91/91**,atlas 零回归。测试首跑抓两个真 bug(margin
  乘错整支撑值;Minkowski 0/2π wrap 平移——修为任意切点重建+底点对齐)。
  **K2 回归 door_B/L32:一致率 1.0/0/0**(coarse-open 221、refined-open
  25、exact-closed 474);小窗口下自适应精确慢于全原始(真关步全升级)
  ——两层收益只在域≫patch 时兑现,属预期。
- 下一步:① G1 主体:合并层 free dual + gate 几何 + witness 认证,
  bench 四族对 oracle 验收;② margin 引理正式化;③ K2 pilot 脚本
  迁移 src API(experiments 版冻结为记录)。

---

# HANDOFF — SPLATC-GMC(2026-08-24:方向切换落地;v2 计划归档;G-H2 pilot 首轮完成)

**主线正式切换至 GMC(Gaussian Mobility Complex)**,承接 P3.8 kill 后的
compile-once/query-many + 认证 gate 表征方向。

- **文档变更**:v2 总控四件(00/01/04/SplatC_Atlas_Complete_Project_Plan_v2)
  移至 `archive/plans_v2_atlas/`(附归档 README);新总控
  `00_MASTER_EXECUTION_PLAN.md` v3.0;设计稿 docx(2026-08-21 v1.0)导入为
  `05_GMC_RESEARCH_DESIGN.md`。02/03(baselines/bench)继续有效。
  `splatc_atlas/` 代码、判定链、送审件全部不动(复用资产 + provenance;
  chart compiler 保留为 certified subdivision fallback)。
- **v3 claim 纪律**(总控 §1-2):P3.8 边界为已确立事实,claim 一律以输出物
  为主(closure threshold/gate interval/route classes/摊销/morphology),
  预算效率降为次要轴;nonholonomic 不进第一篇。
- **Gate G-H2 pilot 首轮完成**(uniform random 族,本地,
  `experiments/gmc/h2_event_density.py`,1440θ×N∈{10..160}×3 seeds):
  组合 churn 二次增长(log-log 斜率 2.06,N=160 网格饱和 0.98)而
  **自由空间拓扑事件 S3 近线性(1.12)**,S3/S1 占比 0.80→0.24 单调下降;
  barcode 显示 S3 有清晰 event-free 区间。**H2 首轮判读:成立方向明确**。
  详见 `docs/worklog/gmc_H2.md` + `results/gmc_h2/`。
- 已知限制:θ 网格计数是下界(S3 饱和 0.24,轻度低估);gate/adjacency 级
  事件未计入(待 G1 free dual);仅 uniform random 族。
- **K2 真实 GS 场景接入完成(08-24 补记)**:用户提供的 397 万 splat 室内
  楼层扫描,角色 = 统计源/H2 测量基底/demo(非裁决基底)。
  `data/gs_scenes/k2/`:raw + sha256、`processed_full.npz`、`slab_2d.npz`
  (65.6 万 SE(2) 条件化 2D splats)、meta.json。地板 −1.09/层高 5.16u,
  尺度假设 1u≈0.5m(层高+门宽双证据,UNVERIFIED)。已知:墙缝审计未做
  (玻璃洞 false-free)。详见 `docs/worklog/gmc_M0_k2.md`。
- **K2 窗口 H2 实验完成(08-24 补记,v1→v3 三轮)**:两真实门洞 ×
  机器人支撑长{1.6,2.4,3.2} + 桌群窗口。**closure 阶梯单调成立**
  (开放占比 1.00→0.64/0.72→0.27/0.34),gate 开合角可测,L32 出现
  第二窄开放带;S3 事件稀疏聚簇(42–73/π,step 占比≤0.10);churn ~10⁷
  对/切片 → **架构结论:contact complex 必须建在合并 primitive 上
  (M1 加聚类)**。过程修复两个测量 bug(点探针被吞→线段探针;结尾
  一次写 JSON→逐配置落盘)。结果 `results/gmc_h2/k2_windows.json` +
  `figs/k2_closure_ladder.png`,详 `docs/worklog/gmc_H2.md`。
- **Gate G-H2 判定:实质通过**(合成族+真实窗口双证据);正式关门只差
  整层 K2 sweep(HPC)。
- 下一步:① **Stage G1 固定 θ slice 立即开工**(复用
  src/gaussian_geometry;witness+hard-checker 认证,witness 用集合/线段
  不用单点;M1 含 splat 聚类,见总控 §5.1 与 worklog 架构结论);
  ② 整层 K2 sweep(HPC)关门 G-H2;③ 墙缝审计 pass。

---

# HANDOFF — SplatC-Atlas(2026-08-22:P3.8 完成,round-11 送审件就绪;kill HIT,主线转向)

**P3.8(rounds 7/8/10 预注册生死实验)执行完毕,kill 条件触发。**

- **任务**:同给两个 converged goal-free chart 的序列化认证 region
  (F_L/F_R),零门先验(守卫 dict 机械强制不读 scene.meta),六臂
  (contact/scalar continuation、generic connector、RRT-Connect、
  lazy PRM/bridge、HRM 风格 SE(2))统一计费,认证连接件才算成功,
  独立场景复检,budget 262,144,采样臂 10 种子。
- **canonical 结果(链 17359268,13h02m)**:thin 0.505 上
  **rrt_connect 10/10,pair_ops 0.73×、墙钟 0.91×**(nq 2.87×,
  continuation 16,462/1,865,643/2.34mm)→ **P3.8 kill condition:
  HIT**(任一货币 ≤2× 预注册规则;三货币齐进读法同样触发;contact
  臂的 G1-frame 先验利我方,HIT a fortiori)。PRM 0/10、HRM 0/1;
  mid 0.62 基线全通(公平性证据)+ generic 157q/5.81mm 互补;
  **负控制 plug 全臂拒绝、假声明 0**。
- **主张变更(预注册协议执行)**:单查询 thin-gate 效率撤下论文主
  residual(此前两报告的"论文级 residual HOLD"就此关闭——判负);
  主线转 **compile-once/query-many + 认证 gate 表征(2.34mm vs 基线
  polish 后 0.37–1.91mm)+ morphology/topology 复用**,P4b 为载体。
  Gate B 机制、P3.7 体积族 8–11× 分离、P4a.2/P4a.3 结论不变。
- **发布**:验收 17366850(**/scratch 净室——用户 08-21 指令弃用
  tmpfs**;12h58m exit 0):净室 79/79 tests + **18/18 artifacts
  逐位 MATCH** + 回执三段门 0/0/0;防篡改自检 4/4(含报告单方面
  软化 kill 判定被 7c 抓)。新增 validator 7c(kill/负控制/参考数/
  WITHIN-2x 引用互检)。
- **送审件:`outputs/splatc_atlas_round11_tree_e934bb006541.zip`
  sha256 `ae0267d44145257848f0043840349393ef75f4949e3267c9e4a777a9ac7ff734`**
  (外置 .sha256;回执内 verified_content_zip_sha256 = 31bfc966accd…
  为验收时内容 zip,按设计与终包不同)。树 e934bb006541 / bundle
  9b8540838116。交付物已同步镜像并远端校验 OK。
- 流程注记:链再生的 oracle_amendments 在
  outputs/oracle_records_amendments/,拉表时须一并拉回(本轮先漏后
  补,层-1 曾以树戳不匹配正确拦截)。
- 下一步:**P4b 两门多目标硬合同**(compile 无 start/goal、atlas
  hash 不变、goal→route 选择、全程认证)——新主线第一个可判定实验;
  两门 demo 在合同完成前禁做。等指令再动。

---

# HANDOFF — SplatC-Atlas(2026-08-21:P0-R9+P4a.2+P4a.3 完成,round-10 送审件就绪)

**round-10 裁决(08-20)**:round-9 "P4a.1 semantic invariance: PASS"
**撤回 → FAIL-CORRECTABLE**(域 support 无证书、goal 条件化 portal、
attachment 测试造假、validate 结构性、计费累计、宽度扫描非单调五类
缺陷全部逐位核验属实);Gate B PASS 不动;两门 demo 在 P4b 合同完成
前禁做。

**P0-R9 + P4a.2 + P4a.3 完成(08-20~21,单次 tree bump):round-10
送审件就绪。**
- **撤回落地**:round-9 判定冻结 `records/..._round9_WITHDRAWN.json`
  (validator 7b 互检:记录须保留被撤回的 PASS + 正文携带撤回声明);
  禁语增 "P4a.1 semantic invariance: PASS" 等三条。
- **三 blocker 修复**:① `domains.support_margin`(朝向相关四边余量,
  1-Lipschitz 同度量)进成员资格 + 双 slack(R30 越界 4,672→0);
  ② goal-free 编译:`enumerate_portals` 无位姿参数、无序对 canonical
  化 + 双侧 θ-列并集 ⇒ (A,B)≡(B,A) 构造性对称,类别改
  generic_certified_connector(证明来源,非宽度分类器,tube 半径
  证书);③ 真 gate 链:`build_gate_chain` 从 witness 拆
  attachment–branch–attachment 全新认证(branch 重认证 2.336mm),
  `compile_atlas` 身份绑定(scene/robot/domain/checker hash),
  `query_reachable` 纯序列化;10 随机 query 对 hash/计数零漂移。
- **validate() 语义化**:region/portal/attachment/edge-chain 几何
  校验;评审三变异 + 假邻接 + 未绑定 + 断链六类全被抓。测试 62→73。
- **预算阶梯至收敛**:顶档 150k 保证 max_level=3 收敛(第一轮本地
  FAIL 因 R60 类 60k 差 ~100 查询卡 level-2——按收敛档原则解决,
  未调 connector)。
- **canonical 双 PASS**(链 sbatch 17347507,9h20m;第一发 17347389
  死于 conda env 被配额清理删除,sbatch_rebuild_env.sh 按 pin 3 分钟
  重建,spec 存 overlays/splatc_env_explicit.txt):P4A2 27 变换全
  converged、atlas 原型 0 违规、compile_q 18,355、逆变换 9/9 复证
  0.151–0.212m;P4A3 134 builds sound_viol/plug/thin_bad/wide_bad
  全 0,收敛档阶梯 {≤0.58:0, 0.62–0.80:1, ≥0.90:0(单 chart)}——
  round-10 非单调确证为 level-2 锚点伪象并披露。
- **验收 17355868(cn024,tmpfs 净室)**:10h41m exit 0,
  CLEANROOM_ALL_PASS——净室 validate PASS + 73/73 tests + **17/17
  artifacts 逐位 MATCH(float maxdev 0.0)**;回执三段门 0/0/0。
  (第一发 17355854 死于 test fixture 读链产物;fixture 改经 run_case
  自包含重生成,tests 不进 hash、链未重跑。)防篡改自检 3/3 被抓。
- **送审件:`outputs/splatc_atlas_round10_tree_9bb7e1df3252.zip`
  sha256 `f5b10ff2f0870373235e5737bf76a05cda4e64901ce30f244fdb38ddf928a87d`**
  (外置 .sha256;回执内 verified_content_zip_sha256 = 07da7e1e… 为
  验收时内容 zip,按设计与终包不同)。树 9bb7e1df3252 / bundle
  92c8d4f6455c。交付物已 rsync 镜像并远端 sha 校验 OK。
- **/scratch 配额仍超限(其它项目占用),Lustre 间歇性拒写风险仍在**
  (本次 rsync 亦触发一次自动重传)。
- 下一步(评审既定顺序):**P3.8 chart-factored connector 基准**
  (F_L/F_R 同给全部方法;contact/scalar continuation、generic
  connector、RRT-Connect、local PRM/bridge、HRM 或 SE(2) NavMesh;
  kill 2×)→ **P4b 两门硬合同**。等指令再动。

---

# HANDOFF — SplatC-Atlas(2026-08-19:P0.8+P4a.1 完成,round-9 送审件就绪)

**round-8 裁决(08-18)**:复现/provenance PASS、kernel 安全 PASS、
Gate B–G1 PASS、P3.7 PASS(限实现族)——维持;**P4a chart 层 FAIL
必须返工**(wide-door 15k 固定预算下语义连通性随坐标系翻转,且当时
报告写"待做"、负结果只在 JSON——流程失职);Atlas schema 撤销冻结;
report 不得按"终版"外发。全部指控逐条核验属实(两个动态诊断本地逐位
复现:30k 宽门恢复连通、mask 角点越界正好 114 cells)。

**P0.8+P4a.1 完成(08-18~19,单次 tree bump):round-9 送审件就绪。**
- 报告拆分:`gate_b_evidence_report.md`(Gate B 证据,关账)+
  `sprint_c_p4a_report.md`(chart 层,round-7 FAIL 原样披露 + P4a.1);
  round-7 FAIL 数据冻结于 `records/`;validator 7b = 负结果一致性
  机器互检;"精确环境锁"/"P4a kill test 待做"入禁语。
- P4a.1:wave-complete 建造器(波开销开波前已知)+ cell-domain 三态
  判定(SAT 精确;成员=FREE∧IN)+ region 序列化 certified cell cover
  (chart_contains/chart_connect 零查询、折线在认证并内)+
  **OpenPortal≠ContactGate**(通用直线+保守证书,零门先验;宽门认证、
  thin/sealed 全拒)+ validate() 九类 witness-chain 校验。测试 42→62。
- **p4a_semantic.py**(canonical):3 门型 × 9 变换(5 旋转 + 2 固定
  网格平移 + 2 origin 相位)× 预算阶梯 {15,30,60}k;稳定判据 =
  converged(max_level/no_frontier 停机 ⇒ 更大预算构建恒等)或
  band(30k=60k)。**OVERALL PASS**:wide 9/9 语义连通 + 9/9 逆变换
  query 恒等帧复证(0.041–0.229m);thin 9/9 双核 + 0/81 假 portal +
  attachment 判据成立(witness 端点 C000≠C001,无 x 阈值);sealed
  9/9 双核零 transition。完整编译账:thin ≈33.4k ⇒ 对 131k ≈3.9×
  (8–11× 明确限定 kernel 级)。
- 发布:全链 sbatch 17286328(11h00m,exit 0,P3.7 kill 0/5 维持);
  验收第一发 17304460 在数值链完成后的画图段死于 /scratch 账号配额
  (其它项目占用;文件数 144% 超限)——净室搬 node-local tmpfs 重发
  17320798(cn222,10h19m,exit 0):**16/16 artifacts 逐位 MATCH
  (float maxdev 0.0,含新表)、CLEANROOM_ALL_PASS**;回执三段门
  0/0/0、MANIFEST 闭环过;验证器对抗自测 7/7 篡改包全被抓(含 3 个
  7b 新检查 + 回执掉包)。终包 159 文件。
- **送审件:`outputs/splatc_atlas_round9_tree_10d00fb69494.zip`
  sha256 `7956ed67d8f216d8b029fff33ec89f026f90f6044786da587a8e0fce966b613e`
  (外置 .sha256 同名文件;回执内 verified_content_zip_sha256 =
  3ffdb86e… 是验收时的内容 zip,按设计与终包不同)。**
  树 10d00fb69494 / bundle ca4d0f29acaa。
- 注意:验收 sbatch 的 tmpfs 版脚本在被验证快照之外(两趟合同要求
  终包=被验证内容+回执),存于 mirror 旁 ops_scripts/;下轮 tree bump
  前纳入。**/scratch 配额仍超限(其它项目),后续作业有随机失败风险。**
- 下一步(评审既定):**P3.8 chart-factored connector 基准**
  (F_L/F_R 同给、无门轴/θ 窗;RRT-Connect + local PRM/bridge + HRM 或
  SE(2) NavMesh 至少其一;kill 2×)→ P4a 收尾(∇h/PW 接触法向、
  robustness)→ P4b 两门硬合同。

---

# HANDOFF — SplatC-Atlas(2026-08-17:P0-R3 完成,round-8 送审件就绪)

**round-7 裁决(08-17)**:**Gate B–G1 正式 PASS**(立项以来第一个正式
主门);P3.7 改名 matched conservative volumetric-family separation
(PASS,限该实现族);**论文级 residual HOLD**(非体积近邻 + chart-
factored 基准未跑);旧包因 provenance method_arm 残留已撤回限定语判
外发 FAIL。评审逐位复现了我们两个动态样本(数字分毫不差)。

**P0-R3 完成(08-17):round-8 送审件就绪。**
验收 v3(cn225 自有分配)单发 10h11m exit 0:16/16 artifacts 逐位
MATCH;回执(cleanroom_reproduction_receipt.json)三段门 0/0/0 并通过
MANIFEST 闭环校验;终包 149 文件 =
`outputs/splatc_atlas_round8_tree_c9cd4676375b.zip`(sha256 5a7286e6…)。
树 c9cd4676375b / bundle 05b5fa061966。本轮内容:arm 字符串修正;code_bundle_sha256(传递 import 覆盖);
validator v4(禁语扫 provenance、bundle 重算、回执闭环校验);**exact
face-overlap 邻接**(金标准 O(n²) oracle 测试逐叶一致;两棵树 + cut +
chart 全切换);p37 正控制补 2×2;P4a validate() 补 opposing_pair/
self-transition;环境 freeze 实测入包;两趟发布 + 净室回执机制;报告改
评审批准的唯一 claim 口径。全链(17261306,10h12m)后 exact 邻接判定不变(P3.7 四臂仍全 NEVER,
kill 0/5——评审保留条件满足);frontier 诊断量按 exact 口径更新入报告。
之后按评审
指令 #6:P3.8 chart-factored connector + RRT-Connect(+ local
PRM/bridge)= 下一个生死实验;P4a 删 x 阈值走 chart-incidence;P4b 两门
合同不变。

---

(以下为历史记录,倒序)

# HANDOFF — SplatC-Atlas(2026-08-13 会话末;深夜追加 P1 完成)

**P1 后状态补记**:v4.1 = fully tag-free discovery loop + branch-aware gate
chart + 认证跳枝协议;六宽度余量与 v3.1 逐位一致;Γ(δ) 全链重测
(Γ_total 40.5–106.2 / Γ_amortized 27.2–42.8 / Γ_self 107→21.6,chart+connector
溢价 ≈ +2;narrow-3 α:track 0.747 / cert 0.806)。

**P2 已完成(cn263 直跑,用户指示绕过拥挤队列)**:42 案例(dev+val,超参
全冻结)——**FALSE_REACHABLE 0/42**;13 认证真阳(长椭圆全宽度 0.55–1.10;
"薄"是机器人相对的);10 正确 abstain(全部物理关门);**6 个
ORACLE_UNRESOLVED_GRID 临界案例全部被构造性认证 REACHABLE**(余量贴解析,
fine 网格解不了的案例 tracker ~10k 查询解决——待回填 oracle records);
13 域外 abstain(机器人相对宽门 NO_SEED = Atlas 缺 open-space chart 的证据;
val_016 椭圆 w=0.80/tilt=12° CERT_FAIL −1.21mm 为唯一质量失败,证书如实拒绝,
按纪律未修,dump 留诊)。数据 `p2_generalization.json`;HPC 侧 phase_002。

**round-6 评审已回(08-14):旧包判 FAIL——快照缺 data manifests +
oracle records,复现必崩(验证器只查了内部一致性、信任了 JSON 自报哈希);
G1-frame 几何先验被点名(|x|>0.7 / ±0.65 / seed 盒 = 先验,非 metadata
字段读取;旧英文限定语撤回,改 tag-free, G1-frame-conditioned);P3 域
不匹配 2.98×(基线无 θ 商/ROI 匹配)→ Gate B-G1 改记 机制 PASS /
strong-residual HOLD。四项可查主张全部本地验证属实。**

**P0-R2 完成(08-15):两层发布门全绿,包可外发。** build_package 补
manifests+42 records+split seals 进快照;validate_package v3(重算树哈希
与脚本哈希、输入依赖 preflight、blind 防泄漏、新禁语;对抗自测 6 FAIL
全抓);compare_reproduction.py(第二层逐数对比);environment.yml 精确
锁;报告五处句修 + 域口径/计费边界注记;amendments 键改
dual_formulation_replay 并以 HPC 锁定环境版为 canonical(mac 版有 libm
采样数漂移,余量逐位一致);gamma 裁剪修复;新增 p36_fairness.png。
**正式验收:sbatch 17247388(自有分配)单发不间断 6h28m ExitCode 0——
解包→validate→reproduce 全链(25 tests + 8 表 + 6 amendments + 图)→
compare 14/14 artifacts 逐位 MATCH(float maxdev 0.0)→
CLEANROOM_ALL_PASS**;log=outputs/p0r2_cleanroom_sbatch.log。教训入册:
直连 ssh 的多小时链会随借宿 SLURM cgroup 一起被清(首发净室 6h 处静默
死),长链一律 sbatch。发布物 = outputs/round4_final_package.zip。

**P3.7 完成(08-15):residual 生死实验通过。** 新树 5353fc1c6d97 全链
重生成(8 表旧字段 vs c45ed **零差异**——树漂移未改冻结行为)+
p37_matched:ROI=seed 盒、[0,π) 商(π-周期性重放断言)、四匹配臂
iso/aniso uniform、aniso generic、aniso pair-aware——**20 主跑全部
NEVER(131,240/131,132),kill condition 0/5 触发,pose 下界
≥8.3–11.5×**;正控制 2k–16k 全过且优于未匹配臂(域匹配生效);cut
frontier 门带 0/0 依旧。计费拆分落地:诊断占 pair_ops 30–42%,干净口径
pair-op 优势 ≥2.2×(iso uniform)至 ≥13×(pair-aware)。报告已更新
(matched-volumetric residual PASS;非体积近邻基线 RRT-Connect/HRM/
NavMesh 为 headline 前必做)。**新树包验收 v2 已过(sbatch 17252937,
单发 8h57m exit 0,15/15 artifacts 逐位 MATCH maxdev 0.0,
CLEANROOM_ALL_PASS)——`outputs/round4_final_package.zip`(树
5353fc1c6d97,137 文件,含 p37 表+图与更新后报告)= round-7 送审候选。**
c45ed 验收记录存 outputs/p0r2_cleanroom_sbatch_c45ed_PASS.log。

**P4a 增量 1 完成(08-15/16,结构层)**:Atlas.add 重复 ID 硬错误 +
atlas.validate()(10 类违规,测试逐个触发);goal-free OpenChart 构造器
(chart_builder,成员 cell 全域认证的保守语义 + domain mask);disc 场景
精确刚体变换(datasets/transforms);chart 层刚体 kill test
(p4a_rigid_charts.json):细门五角度全不变,宽门 30° 暴露粗分辨率假象
(宽门真拓扑本就连通)= gate 需要 branch/transition 层的结构性论据;
kernel 级等变换在 P4b compile(scene,robot) 前不可跑(如实记录)。测试
37/37。注意:P4a 的 src 改动 = 又一次树漂移,**下次打包前需再跑一次全链
+ 验收**(p4a_rigid_kill 已入 reproduce.sh 与 build TABLES)。
剩余:∇h 接触签名、chart-branch incidence、多假设 seed(依赖 P4b
compile 包装)→ P4b 两门多目标。
终报告:`docs/reports/round4_final_report.md`(唯一口径)。

**P3 已完成(cn263)——三基线五宽度全灭(budgeted false negative ≤131,240
actual),continuation 11.4–15.8k 认证输出,差距 ≥8–11×(下界)**。

**round-4 评审已回并处置完毕(08-14 凌晨;处置附录在 round4 报告顶部)**:
判定 = **Gate B-G1 PASS-CONDITIONAL → 条件项已全部执行**——
① 正控制通过(w=1.10 对齐:uniform@16k/32k、generic@32k、contact@8k
⟹ evaluator 无罪);② goal_resolved/failure taxonomy/双连通性字段;
③ 三层计费(continuation pair_ops 2.22M ≈140/查询 vs uniform 2.62M
≈20/查询——pair_ops 口径下 continuation 仍 ≤ uniform);④ gate-band
访问统计(基线到过门带但 θ 窗命中 0–576——θ 分辨是死因);
⑤ **scalar-clearance 同架构消融 0/14 vs pair-contact 6/6**(seed 选择性
+ mouth 脊条件化;Claim E 首证);⑥ witness 升级协议 6/6(双 checker
零违例,amendment 制,原记录未动);⑦ provenance 三漏洞修复 + schema
防护。**Gate B-G1 具备正式 PASS 材料,待评审确认**。
下一步:**P4a**(handshake 六类对象,branch IDs + adjacency 先行,
compile 无 goal-specific pruning)→ P4b 两门多目标 → P4c 多假设 seed +
region incidence → P5 Gaussian residual audit(vs SDF/CDF/凸体 oracle)。
纪律注记:val 集已因 val_016 分析失去"未触碰"地位,method freeze 前
必须用新的 sealed blind(评审 §三.2)。

新 session 首先读:本文件 → `docs/worklog/sprint_B.md`(倒序)→
`docs/reports/problem_thin_gate_discovery.md`(含修订节)。
计划 canonical:上层目录 `00–04_*.md`。HPC:`/scratch/sy2366/Project/splathjb`
(cair QOS,`bash -lc` 提交;env overlay 就绪;见记忆文件 hpc-jubail-verified-facts)。

## 状态一句话

Gate A PASS;Sprint B round-3 外部评审已回并处置完毕(附录见
`docs/reports/ridge_round3_report.md` 顶部):**机制验证 PASS,当前代码冻结为
候选 gate-discovery kernel(定名 Contact-Ridge-Guided Continuation with
Certified Path Output;tag-free, G1-frame-conditioned——round-6 撤回旧
"door-geometry-…-free" 限定语,G1-frame 先验在案);正式 Gate B / Gate C
未通过,论文主方法 claim 不冻结**。
实测:δ=40→1.25mm 六档全部认证输出(1.25mm:237 站/20.6k/余量 1.14mm),
关门 safe abstention;Γ_total 39.6–104.3 / Γ_amortized 26.3–40.8 / Γ_self
105→21;可靠统计量 = 窄三档 c_track 与 cert-only 局部 α 均 ~0.8、比值 ≈22
(finite-range evidence,无渐近 claim);cold-start ablation α=1.09;
hybrid floor 为 censored 参照(精确拟合 2.5,末点预算截断)。
统一 claim 口径(评审定稿):*在 de-aligned single-door family 上,oracle
opposing-side grouping 下的 contact-ridge-guided predictor–corrector 能从局部
连续接触 probe 发现并连续认证半裕度低至 1.25mm 的穿门路径,关门实例 safe
abstain;站间 continuation 在最窄段与 witness certification 局部标度相近,
明显优于逐站 cold start。验证了 gate-tracking kernel,尚未建立 side-tag-free
discovery、goal-independent Atlas、unreachable certification、multi-goal
reuse、matched-budget residual。*

## 下一步(评审定序 P0–P5)

- **P0(已完成 08-13 晚)**:实验包一致性——`prov.py` 给全部 JSON/图打
  provenance 戳(script+src sha256/UTC/arm/checker/计费;无 git 以内容 hash
  代),图由 JSON 自动生成,全链重跑。评审抓到的版本错位(诊断图 v2.3 vs
  JSON v3.1)已根除。
- **P1(已完成 08-13 深夜,v4.1)**:
  **P1a** tag-free 对侧分组(`bilateral_rho_tagfree`:接触方向最大角隙聚类 +
  时序参考连续性;free pose 与 oracle 分组逐位一致 309/309+2925/2925;
  sweep 端到端零行为变化)——发现回路 fully tag-free。
  **P1b** branch-aware gate chart(`section_eval` 全组件截面 + 事件流)+
  认证跳枝协议(lobe-local corrector;凡截面跳变先 2cm 抛光再当场认证,
  锚点按站 margin 降序 = "最后健康 component" 操作化;m_esc/2 修剪删除)。
  六宽度余量与 v3.1 逐位一致;关门的 chart 结构化记录截面塌缩→march_death。
  **重要图景修正:mouth 处是单 component 薄颈,非分离双 lobe**(v2.2 表述
  为坐标下降视角)。未竟小项:截面间 adjacency 边/birth-split-merge 推导器、
  component signature 字段(数据已备,推导未写);多 gate 图属 P4。
- **P2:冻结全部超参**(250mm 网格、K=8 候选、±0.35 θ 窗、15 点扫、5mm 步长
  下限、m_esc/2、800 帽),跑 offset/tilt × width × morphology 泛化
  (dev 42 案例族起步)。一晚多修数条 = dev overfitting 风险已被评审点名,
  在当前 G1 个例上不再调优。
- **P3:matched-budget Uniform SE(2) + Generic Adaptive**(Gate B 决定性实验;
  同代码同 checker 同预算;报告 gate recall / false-unreachable / topology
  accuracy / representation size;hybrid floor 重跑取代 censored 引用)。
- **P4:两门多分支多目标场景**:compile 一次、存两 gate、不同 goal 触发不同
  min-plus 路由——第一次证明"是 Atlas 不是单路径 tracker"(Claim D)。
- **P5(降级):路径质量 polish**——不做每站 +180q 的无条件截面;待 certified
  gate band 建好后在 band 内做一次 smoothing/max-margin readout。
- 计费扩展(随 P3):unique queries / pair ops / wall-clock / memory /
  caching 后重复查询 / preprocessing-vs-per-goal 分开报告。
- HPC:P2/P3 跑批时上(cair QOS,bash -lc);单例实验留本地。

## 08-13 晚已裁决的事实(运行记录见 ridge_stations.json / ridge_abc.json + worklog)

- v1 CERT_FAIL(w=0.54/0.58)= start→seed 直线在门口偏轴 ~78mm > 裕度 20/40mm;
  v1 station-1 死(w≤0.52)= 粗网格全 −0.5 平台候选 + argmax 容碰撞 bug。
- corrector 恢复域 ±100mm/±15°(exp B,单轮 92–96/96,含 w=0.505);KKT Newton 版更差,弃。
- v2.x 迭代裁决(全部 dump 驱动,见 worklog 末条):墙面姿态是 bilateral 滤波的
  假阳性(polish 后验证解决);θ 三分在窄尖峰塌错 lobe(粗扫描定界解决);
  retry 平移站点致其漂入碰撞(只缩步不动站解决);顺序 balance→θ 坐标下降在
  mouth 分叉有**伪不动点**(被堵 lobe 的 medial axis)——升级 (n,θ) 截面 corrector
  解决,这是评审 KKT/active-pair 切换预言的实测形态。
- v2.3 六宽度全绿;w≤0.52 路径薄点贴解析裕度(<0.3mm 差),w=0.54/0.58 mouth
  切内角(v3.1 后 1.37/17.18mm)为路径质量欠账、非可靠性问题。
- v3 单位分离曾回退 w=0.505(pinch 站 + 跳枝插值横穿 lobe 封锁区,−1.32mm)——
  死端修剪修复;200 循环帽曾静默截断 δ=1.25mm march——改 800。两条均 dump 验证。
- Γ(δ) 终版见 worklog 末条与 round3 报告 §5;cert-only 见证余量逐档恰为解析
  裕度(分母可信);cold-start 关门"解出"的 4/65 站全在墙外漏斗侧,非路径主张。

## 纪律红线(全程有效)

blind 密封未标注;零 false-reachable(证书兜底,一直在正确工作);side tags=oracle
ablation(最终须接触法向自动聚类);不读 door metadata;每步先看图后修码;
所有"一致/成功"须指向运行记录文件。旧债:PW 界形式证明、C₂ 商、跨层邻接、
NavMesh 复现(审计 memo 在 docs/baseline_audit_2026-08-12.md)。

## 关键文件

- 方法机器:`src/splatc/baselines/{probe_methods,aniso_tree}.py`(v4 冻结含注记)
- continuation:`experiments/compiler/ridge_continuation.py`(0.22 阈值+插站+dump 版);
  实验 A/B/C:`ridge_abc.py`(B/C 用 oracle 轴位姿,已声明);画图:
  `ridge_diag_plots.py` / `ridge_abc_plots.py`
- floor 数据:`results/tables/floor_probe{,_aniso,_hybrid}.json`;
  continuation:`ridge_continuation.json` + 站点 dump `ridge_stations.json` +
  A/B/C `ridge_abc.json`;图:`ridge_diag_w*.png`、`ridge_exp{A,B,C}.png`、
  `theta_diagnostic.png`
- 评审往来:`docs/reports/probe_round1_{full_report,review_reply}.md` + response 文件
- 真值:witness 证书(D17,2.5–40mm)、42 份 OracleRecord、密封 splits(sha 在
  results/manifests/split_seals.json)
