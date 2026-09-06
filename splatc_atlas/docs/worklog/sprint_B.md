# Sprint B worklog

## 2026-08-12（HPC onboarding）

- **[现状] HPC 侧是 8/5 的 SplatHJB 空骨架**（concept/preflight 阶段，无环境无代码）。
  按顶层 README 纪律 onboard：代码 `rsync` 到 `/scratch/sy2366/Project/splathjb/splatc_atlas/`，
  计划文档进 `docs/plan/`，STATUS.md 与 phase_logs/phase_001 记录 pivot。
- **[事实核对] QOS/分区**：`cair` QOS 挂在 compute / condo / nvidia 分区下
  （`sacctmgr show assoc`）。本项目当前全是 CPU numpy 工作负载 → 用 `-p compute -q cair`，
  不占 GPU 节点；README 的 a100/h100 建议留给以后真正的 GPU 阶段。
- **[实现] conda-in-overlay 环境**：`hpc/build_env.sbatch` 在计算节点
  `singularity overlay create` 10G ext3 → miniforge → numpy 1.26.4（与本地一致）/scipy/
  matplotlib/pytest → `/ext3/env.sh`；构建完在只读 overlay 下跑完整 17-test 冒烟。
- **[纪律] blind 不标注**：`label_manifests.py` 只装载 dev+validation;
  labeling 作业阵列 42 tasks（`hpc/label_manifests.sbatch`）。convergence 协议实现了
  `ORACLE_UNRESOLVED_GRID`：fine 网格结论低于解析真值（de-aligned 临界案例的已知现象）
  时打 flag,不硬标。
- **[坑] 非 login shell 没有 sbatch**：`ssh hpc bash -s` 环境缺 SLURM 路径,提交要用
  `bash -lc`。
- **[坑→修复] env 首建失败（17183255）**：最新 miniforge base 是 Python 3.13,
  numpy 1.26.4 无 3.13 wheel → pip 源码构建炸。改为 mamba 建 `splatc` 环境,锁
  python=3.11 / numpy=1.26.4 / scipy=1.10.*,与本地逐版本一致（数值可复现要求）。
  副产品:确认计算节点有外网(下载都成功了)。重提为 17183652。
- **[结果] dev+validation 真值标注完成（HPC 阵列 17183942，42/42 COMPLETED）**：
  36 converged / 6 `ORACLE_UNRESOLVED_GRID`——6 个全是薄门 de-aligned 案例
  （dev w=0.55 与 val w=0.52 的偏移+倾斜组合，小圆和长椭圆都中招），unresolved 协议
  按设计工作。false-unreachable 阶梯:coarse 7 → medium 6 → fine 4（零 false-reachable，
  离散化误差单侧性符合预期）。gate 区间测量 vs 解析 max 0.185°。26/26 参考路径保守
  认证,最小米制余量 0.45mm（临界 de-aligned 案例贴边穿门,值得在 Pareto 实验里单列）。
  汇总在 `results/tables/oracle_truth_summary.txt`。
- **[坑] 监听命令查错目录**：驱动把记录写 `splatc_atlas/outputs/`（相对脚本路径），
  监听查的是项目根 `outputs/` → 首查 0 条虚惊。教训:作业脚本的输出路径要在提交前
  echo 出绝对路径。
- **[设计符合性审计]（用户质询触发，第三轮）**：逐条对照 00 §1–7 / 03 规格 / problem_spec。
  结论:无合同违背;抓到并修复 2 项:
  (A) **可达性标签(6-连通)与 Dijkstra(10-邻域)连通性不一致**——对角步理论上可跨分量成
  "桥",出现标签 U 但搜索找到路的矛盾。已发布结果未中招(路径仅在分量可达时提取+全部
  保守认证),但属设计漏洞。修:路径图限制在起点 6-连通分量内;邻接合同写入 spec §8;
  rotate-door 代价回归不变(4.267232),校验组 ALL PASS。
  (B) **标注驱动绕过 records.py schema**——加 `validate_oracle_record` 落盘前强制校验,
  records.py 为唯一 canon;42 份已有记录全部通过校验(零漂移)。
  另 λ_clr=0 从隐式改为显式冻结条款。
  范围性欠账(非违背,已排期):G5 remote-closure 场景族、dev 侧 multi-goal records、
  smooth overlap c_ij 族(Sprint B 主项)、多 primitive 机器人与各向异性场景 primitives
  (R3–R6/G8/G9 需要,当前评估器 disc-scene/单 primitive 已在 spec 注明)、
  NavMesh feasibility audit 等后台线程(已到我自己修订的触发时间,落后)。
- **[决定性实验,第一轮结论:未分胜负,但基础设施已可信]** 探针实验迭代记录:
  (1) 首版"cell 中心采样打标签"结构性失败(全策略全预算全 U)——census 显示预算烧在墙内
  兔子洞,walls 无界细分。**教训:跳过了计划 Stage 3 第 2 步的三态认证,这是设计偷工。**
  (2) 重写为 FREE/COLL/AMBIG 认证树:可靠米制界(远处三角不等式 d₁−(a+R),壳内 PW 缩放
  h·(b+R));102k 采样 pose 经验验证 0 违例;连通性只走 FREE_CERT ⟹ false-reachable
  结构性为零。起终点 cell 强制种子细分(各策略一致)。
  (3) 逐层修复:goal 判定改 box-disc 相交;开阔区粗骨架认证分支(level≤2,各策略一致);
  balance(|h₊−h₋|)排序实现计划的 balanced-contact 线索。
  (4) **当前状态:w=0.58 de-aligned 下 64k 查询,三策略均未打通。** uniform/generic 失败
  符合理论预期(体积/边界诅咒)。contact 的 bilateral+balance 线索能找到并认证通道本体
  (一度 1468 个 level 7-8 走廊 FREE cell),但预算被"门轴线 × 整个 θ 圆环"的 balanced
  区域稀释(θ 无选择性:balance 只识别位置在门轴,不识别姿态可通行,浪费 ~12×)。
  **这不是 bug,是信号不足**——恰好落在计划预设的下一级线索上:bilateral/balance 只是
  cue 清单前两项,"gate-specific angular interval detector"(02 中明确 banned for B2、
  保留给主方法的信号)还没实现。下一步:θ-选择性 gate 检测器
  (bilateral cell 按 |min(h₊,h₋)|+balance 排序,或对检测到的门轴做显式 1D 角度扫描,
  即 08 §8.2 的 adaptive critical-contact discovery)。
- **[外部评审→当场验证三件事]**（评审文件 probe_round1_full_report_response.md,
  处置全文见报告修正附录):
  ① PW→米制界对抗验证:50 万构型 vs 独立 checker,0 违例,max ratio=1.000000(紧点在
  短轴接触)——界可靠性从"cell 内随机采样"升级为"全构型空间对抗验证";
  ② θ-环检查:房间定点整环 180/180 认证 FREE 且连成 1 分量——wrap 无 bug,镜像大分量
  实为门区细层壳簇的未完成认证孤岛(此前"正常现象"表述已撤回);
  ③ 构造式真值:5 个门宽显式路径全部过保守证书,min 余量与解析 (w−0.5)/2 精确吻合
  (2.5→40mm)。**副产物:见证只需 28–284 次 margin 检查,信息下限 ≪ 64k——
  表示层认证 vs 路径层认证的溢价成为下一轮核心测量。**
  已确认的自身错误:角度因子 12×→5.76×(漏算 θ+π 窗);"Morse"命名撤回(是标量等距
  启发式);balance 分数奖励双侧碰撞态(b<0 排最前,兔子洞共因)。
  Round 2 冻结矩阵:v4 / v4+双队列 / v4+各向异性 θ-split / v4+认证角度区间检测器 /
  oracle-guided floor(仅诊断);宽度加 {0.49,0.495,0.50};三态输出;三层计费;
  精确面邻接;REACHABLE 回溯 witness 闭环。
- **[Round 2 首实验:oracle-guided floor —— 否证"纯排序瓶颈"假设(评审 §七 怀疑成立)]**
  全知管道调度下:w=0.58 floor=24k;w=0.54 floor=96k(**高于 round-1 的 64k 预算**);
  w≤0.52 在 128k 内不可达。经验标度 floor ∝ margin⁻²(24k→96k 对应 40mm→20mm)。
  tube 0.2→0.4 只差 ~2×,排序空间总增益就这个量级。结论:round-1 在 w≤0.54 的全灭
  是**机器容量决定的**,任何排序信号都救不了。
  机制(层级直方图佐证:w=0.58 全知仍需 L6+L7 共 2.6 万 FREE cell):各向同性 2×2×2
  分裂 + 方向盲 Lipschitz 半径把 x/y/θ 耦合——走廊沿轴向清晰度几乎不变,却被迫全维度
  细分。**这是项目主论点的实测新实例:维度耦合表示在薄门处必有标度病,gate 需要
  各向异性/沿轴参数化的专属 chart(Atlas gate-chart 概念的定量依据)。**
  Round-2 序修订:先修机器(①各向异性分裂 2× 代 8×;②方向感知认证——bubble 证书
  推广到 cell,允许沿走廊轴的长薄 cell),预测 floor 降至千级;然后才做排序信号消融
  (角度区间检测器等),否则信号差异不可测。
- 问题与解决随做随记：（下接）

## Round-2 机器修复(2026-08-13)

- **各向异性分裂单独无效**(floor 32k/128k,略差于 iso):argmax-维度分裂仍受
  r_cell 全向耦合约束,走廊轴不变性未被利用——"aniso 必然大赢"的预测错误,已实测否证。
- **混合机器有效(6× 改善)**:发现用中心标签树,REACHABLE 可靠性由路径 bubble 证书
  提供(模块 5 语义,体积认证是探针期的过度工程)。hybrid floor:
  w=0.58: 4k(原 24k);w=0.54: 16k(原 96k);w=0.51/0.52: 128k(原 >128k);
  w=0.505 仍 FAIL——发现层的"中心命中 y×θ 窗"仍有 margin 标度,下一杠杆是梯度引导
  放置(消融矩阵臂)。路径证书仅 +168~551 次检查。可靠性:false-reachable 仍结构性
  为零(证书兜底)。数据:floor_probe{_aniso,_hybrid}.json。
- **[第二轮评审确认]** 新问题与 round-1 同根("balance 只找到门轴"的下一层)。关键数学
  修正被采纳并验证:角度窗 ∝ √δ(公式 vs 实测 3.78°/3.73°),截面 O(δ^{3/2});δ⁻² 是
  center-hit 模型的病。方法定名 Certified Contact-Ridge Continuation(ridge = B=0 且
  M 极大,KKT 延拓 + bubble tube 认证);Gate B 拆 B1/B2。详见问题文档修订节。
- **[ridge continuation 最小版首跑:全宽度 TRACK_LOST]**(ridge_continuation.json):
  w≤0.52 在 station 1 即失(3 次 predictor 步全撞);w=0.54/0.58 追了 51 站但从未过门
  (疑似在房间中线 ridge 上徘徊/秒差方向不稳)。已定位的两个实现缺陷:
  ① corrector(balance 二分 + θ 三分)无可行性守卫——全撞区间照样"收敛"到垃圾点;
  ② 秒差 predictor 方向会被 corrector 回拉翻转,缺少前进方向单调性约束。
  好消息:单案例只要 0.6–3.6 秒、~1–3k 查询——调试环路极快,机制成本量级符合预期。
  下一步:corrector 加"m 必须改善否则拒绝"守卫 + 方向锁(与上一步方向点积>0)。
- **[评审 cap bug 确认+修复]** side_rho 继承 RHO_CAP=0.25,终止阈值 0.3 数学上不可触发
  (读码+实测双确认)。阈值降 0.22 后:w=0.58/0.54 tracker 以 9 站、总查询 ~1.28k
  穿过门抵达远侧(首跑的 TRACK_LOST 标签系错误);证书如实拒绝了粗折线
  (w=0.54 仅差 0.52mm)——零 false-reachable 保持。w≤0.52 仍死于 station 1
  (seed/corrector basin,评审实验 A/B 待做)。下一步:门区加密站点(证书密度由
  clearance 控制,属 C_cert),单位分离(PW score vs 米制 margin),实验 A/B/C 拆分。
- **[插站修复无效→失败点定位在前缀]** 门区 2cm 插站(c_densify=1400)后 CERT_FAIL 的
  min 余量逐字节不变(−17.93/−0.52mm),c_cert 仅 4–10 步即失 ⟹ 证书死在路径前缀
  (start→seed 直线段或 seed 站自身),与走廊无关。两次跳过评审的"先 dump 站点"
  指令导致两次盲修——下一步无条件先做:全站点 (x,y,θ,h±,B,m) 落盘 + 三张轨迹图
  (xy / x-θ / margin),然后按实验 A(seed audit)/B(corrector 恢复)/C(oracle seed)拆。
  疑点清单:seed 全局 argmax 可能落在右侧/错误分支;balance 无 bracket 时静默返回原点;
  start→seed 直线在 seed θ 下可能切墙。
- **[站点 dump + 三张图落地:两类失败全部定位]**(dump:`ridge_stations.json`;
  图:`results/figures/ridge_diag_w*.png`;插桩零额外计费 probe,复现逐字节一致
  −17.93/−0.52mm、9 站、c_seed=936)。诊断剖面为 checker #2 米制 margin,5mm 采样,
  单独计 c_diag,不入三层计费。
  ① **w=0.54/0.58 CERT_FAIL = 疑点 3 确认**:seed 网格赢家在门中心 (0,0,0°),tracker
  被 `t_dir[0]≥0` 强制只向 goal 侧行进 ⟹ 走廊左半段(start 漏斗→门心)从未被追踪;
  证书 polyline 用 start→seed 直线补这段,直线在门口 (x≈−0.67) 偏离倾斜走廊轴 ~78mm,
  超过 θ=7° 的横向裕度(w=0.58:40mm;w=0.54:20mm)⟹ 剖面实测 min −27.5/−48.0mm
  (两者之差 20.5mm ≈ 裕度差 20mm,机制自洽;证书报的 −17.93/−0.52 是 early-abort
  的部分观察值)。插站为何无效:densify 只在站与站之间跑,seg 1(start→seed)不在
  站列 ⟹ 逐字节不变,与 08-13 观察吻合。**追踪过的右半段完全健康**(站上
  m=0.056–0.25 PW,B≈0,θ 全程在解析 gate 带内)。
  ② **w≤0.52 station-1 死 = 疑点 1+2 确认(更强形式)**:粗 seed 网格(500mm/0.45m/22.5°)
  bilateral 候选全部 quick-collide(m=−0.5 平台,零 free 候选),argmax 退化为扫描序
  tie-break ⟹ 选中下左 jamb 肩部 (−0.5,−0.45,67.5°) 的深碰撞位姿;seed polish 的
  balance 双端同号 no-bracket 静默返回(dump 有事件记录),theta_opt 在 −0.5 平台上
  盲转到 47.6°;predictor 三连拒 → ridge_dead,c_track=114。剖面:该"路径"穿墙
  −150mm。疑点"seed 落门右侧"排除(x=−0.5,分支正确,是 θ/深度错)。
- **[实验 A/B/C(评审 §14 处方)——瓶颈三选一裁决:纯 seeding + 单向行进]**
  (`experiments/compiler/ridge_abc.py` → `results/tables/ridge_abc.json`;B/C 用
  door metadata 构造 oracle 轴位姿,全部按纪律标注 oracle 诊断,不是方法本体):
  **A seed 审计**:粗网格 (500mm) free bilateral 候选数:w≤0.54 全部为 0(w=0.58 仅 1)
  ——station-1 死的充分解释;加密一级 (250mm,6.8k q) 即 1–9 个,再密 (125mm,51.7k q)
  ~50 个。⟹ 网格 seeding 有 δ 依赖但一次性成本可控(6.8k),或改用漏斗延拓免网格。
  **B corrector 恢复域**:oracle 轴位姿扰动 ±5..100mm/±1..15°(含联合),现役单轮
  corrector(balance 二分+θ 三分)恢复率 92–96/96,**w=0.505 也 92/96**;迭代 5 轮
  94–96/96;我实现的 2×2 KKT Newton 反而差(27–69/96,有限差分步长在窄缝内出界)。
  ⟹ **corrector 不是瓶颈,恢复域大到 ±100mm/±15° 量级**;"单轮 vs 迭代"差异可忽略。
  **C oracle-seed 双向 tracker**:seed 放漏斗口 (t=−0.65,oracle),现役 tracker 机器
  不改、只加反向行进(±d 两次跑),组装 start→bwd 站…seed…fwd 站→goal + 2cm densify
  + bubble 证书:**w=0.505/0.51/0.52/0.54/0.58 全部 CERTIFIED_REACHABLE**,
  总查询 3.3–3.8k,cert min 余量 2.21/4.73/9.65/19.73/39.73mm ≈ 解析裕度
  (w−0.5)/2=2.5/5/10/20/40mm(贴解析上限,tracker 停在脊上的直接证据);
  w=0.49(物理关门)TRACK_LOST,证书如实拒绝——零 false-reachable 纪律保持。
  **⟹ 发现成本在 δ=40→2.5mm 上近乎平坦(~3.5k),对照 hybrid floor 的 4k→128k+
  (δ⁻²):这就是 Gate B2 要的"延拓消除首次命中"的第一个端到端认证证据,
  差的只是 metadata-free seeding(A 给了两条路)+ cold-start 对照。**
  图:`ridge_expA/B/C.png`。已知欠账不变:0.22 终止阈值仍是 PW 单位(HANDOFF §3
  单位分离未做);side tags 仍 oracle ablation;C 的 seed/方向为 oracle(下一步
  就是把它换成 250mm 网格 seed + 双向行进,进 cold-start 对照矩阵)。
- **[去 oracle 化收口(v2.3):六宽度全部无 metadata 认证,c_total ~11k 平坦]**
  (`ridge_continuation.py` v2.3;v1 归档 `archive/ridge_continuation_v1_20260813.py`,
  v1 运行记录另存 `ridge_continuation_v1.json`/`ridge_stations_v1.json`)。
  四轮 dump 驱动迭代,每轮失败机制都落了盘(`ridge_stations.json` 逐版覆盖,
  末版为 v2.3;中间版机制记录见本条):
  v2 = 双向行进 + 250mm 网格只取 free bilateral → w≤0.52 唯一 free 候选是**墙面
  姿态**(两 tag 均入 shell 但非"夹缝"),polish 转到双侧 cap,march 终止条件
  station 0 即真。
  v2.1 = 候选按 m 降序逐试(容浅碰撞、弃 −0.5 平台)+ polish 后验证
  (m>0 且 max(h±)≤0.22)→ seed 落漏斗口正确,但朝门 march 死于 mouth:
  dump 三机制——θ 三分在窄尖峰(±3.7°)塌错 lobe(θ=148.9°,hp=−0.5);
  reject 后 retry 平移当前站不验证,把站漂进碰撞(m=−0.000);step 下限 0.02
  超出 corrector 能力。
  v2.2 = θ 粗扫描(15 点)定界+三分精化、retry 只缩步不动站、step 下限 5mm →
  更深一层:march 收敛到**顺序坐标下降的伪不动点**(被堵 lobe 的漏斗 medial
  axis,(−0.69,−0.022,169°),m→0),可通行分支在 (y≈轴,θ≈187°) 隔 lobe 相望
  ——这就是评审 KKT 框架的 active-pair/分支切换问题实测形态;w=0.54 侥幸通过
  只因 gate 窗 ±10.8° 两 lobe 早合并。
  v2.3 = reject 时升级**截面 corrector**(θ 扫 × 每 θ balance,取 (n,θ) 截面
  m-argmax;问题文档 ridge 定义的字面实现,顺序版是其坐标下降近似)+ 跳分支后
  跳过一次 secant → **w=0.505/0.51/0.52/0.54/0.58 全部 CERTIFIED_REACHABLE,
  w=0.49 关门 TRACK_LOST 如实拒绝**。
  数字(`ridge_continuation.json`):total 11018–11391(c_seed=6.9k 占主导,
  c_track 0.7–2.4k,c_densify 1.6–3.4k,c_cert 105–416);min 余量
  2.21/4.72/9.72/0.28/18.99mm。w≤0.52 的薄点全在走廊内、贴解析裕度
  ((w−0.5)/2,偏差<0.3mm)——升级 corrector 把路径跳到了居中分支;
  **w=0.54/0.58 薄点在 mouth(0.28/18.99mm)**:顺序 corrector 渐进旋转
  "切内角",未触发升级(无 reject),证书保守通过、可靠性无虞,但属路径质量
  欠账——候选修法:每站无条件截面 corrector(+~180 q/站)或终点抛光 pass,
  留给 Γ(δ) 实验的质量指标一并处理。零 false-reachable 全程保持。
  与 oracle-seed 对照(exp C 3.3–3.8k):无 metadata 溢价 ≈ +7.5k,全部在
  C_seed(6.9k 网格,一次性、δ-无关);C_track 端 δ=40→2.5mm 仍近乎平坦。
- **[v3 单位分离(HANDOFF ②)+ 死端修剪]** v3:终止条件改 active-set 语义
  (单侧 h≥0.249=RHO_CAP−ε ⟹ 该侧 shell 内无墙,弃 0.22 PW 代理);步长律转米制
  (m_metric=m_PW·(b+R_EDGE)=0.35 因子,step=clip(4·max(m_m,1mm),5mm,120mm));
  balance no-bracket 显式返回失败(不再静默);二分深 11(0.15mm 分辨率,
  δ=1.25mm 档需要)。**首跑回退**:w=0.505 CERT_FAIL −1.32mm——米制小步长让
  march 贴近死端 pinch(st3 m=0.0001)后才跳枝,densify 在 pinch 站与跳枝目标
  间直线插值,横穿两 lobe 封锁区(profile seg 23);v2.3 大步长下 reject 早、
  未贴 pinch,属侥幸。修复(v3.1):跳枝时修剪死端尾站(m < m_esc/2 的尾巴
  全部弹出),跳枝段从健康后撤点出发。六宽度全绿:min 余量
  2.34/4.68/9.85/1.37/17.18mm(w≤0.52 贴解析裕度;w=0.54/0.58 mouth 切内角
  欠账仍在),total 11.1–15.3k;米制步长的正确副产品:站数 ∝ L/δ
  (w=0.505 用 127 站),c_track 成为 δ⁻¹ 标度。数据:`ridge_continuation.json`
  + `ridge_stations.json`(v3.1 版)。
- **[Γ(δ) 实验(HANDOFF ③)首轮 + 循环帽 bug]**(`gamma_delta.py` →
  `gamma_delta.json`):三臂 = continuation v3.1(无 metadata)/ cold-start
  站位优化(2cm 站,K-加倍 θ 扫 × y-balance,无站间信息,诊断臂已声明)/
  cert-only(oracle 轴线见证,分母)。δ={40,20,10,5,2.5}mm 全过;cert-only
  见证 min 余量逐档恰为解析裕度(40.0/20.0/10.0/5.0/2.5/1.25),checks
  108→520(α≈0.45);δ=1.25mm 首跑 continuation TRACK_LOST——dump 验证
  `terminated=loop_limit`:5mm 步长走全程需 ~450+ 迭代,200 帽把 march 在
  x=0.407 截断(与 0.3 阈值同类的"常量不够用"bug,已改 800 重跑)。
  关门对照:continuation 拒绝;cold-start 仅"解出"4/65 站(全在墙外漏斗侧,
  不构成路径主张)。
- **[Γ(δ) 终版(800 帽重跑):δ=1.25mm 认证通过,Γ 有界——问题文档成功判据
  在本实例族达成]** δ=1.25mm:CERTIFIED_REACHABLE,total 20584
  (c_track 11390,237 站),min 余量 1.14mm(解析 1.25)。六档拟合:
  **continuation total α=0.17、减 seed α=0.33、c_track α=0.77、cert-only
  α=0.45、cold-start α=1.09**(体积参照 hybrid floor α≈2 且 δ≤2.5 失败);
  Γ_total 104→40、Γ−seed 41→26,随 δ→0 不增。cold-start 臂窄档绝对成本
  反超 continuation 减 seed(19.7k vs 13.7k)——θ/分支连续性贡献被隔离。
  数据:`gamma_delta.json`;图:`gamma_delta.png`。评审材料汇总:
  `docs/reports/ridge_round3_report.md`(含 oracle 声明清单、自报欠账、
  评审问题五条)。
- **[round-3 外部评审回执+处置]**(全文见对话记录;逐项处置在报告顶部修正附录):
  判定 = 机制验证 PASS / kernel 候选冻结 / **Gate B、Gate C 未过 / 主方法
  claim 不冻结**;定名改 Contact-Ridge-Guided Continuation with Certified
  Path Output;"metadata-free"收缩为 door-geometry-metadata-free under
  oracle side grouping;关门=safe abstention。**当场复算评审的拟合审计,
  逐位吻合**:窄三点 α c_track 0.819(R²=1.000)/cert-only 0.806(R²=1.000)/
  total 0.363;c_track/cert-only 窄段 21.5/21.8/21.9;Γ_total 39.6–104.3、
  Γ_amortized 26.3–40.8(报告曾把后者误标为总口径,已改);Γ_self 105→21;
  hybrid 三点精确拟合 2.50 且末点截断(α≈2 标注撤回)。**评审抓到实验包
  版本错位(诊断图 v2.3 版 vs JSON v3.1 版)——P0 修复**:新增 `prov.py`,
  全部 JSON 带 provenance 块(script/src sha256+UTC+arm+checker+计费版本,
  无 git 以内容 hash 代 commit),图从所读 JSON 自动携带脚注戳,全链重跑;
  gamma 增列 Γ_self 与 fits_narrow3。新优先级 P0(done)→P1(去 side-tag
  oracle + branch-aware gate chart,含修剪逻辑替换为 connector 认证)→
  P2(冻结超参跑 offset/tilt×width×morphology,防 dev overfitting)→
  P3(matched-budget uniform/generic = Gate B 决定性实验)→P4(两门多目标
  Atlas 演示)→P5(路径 polish 降级)。Claim A–E:未测/部分/迹象/未测/未成立。
- **[P1a 完成:side-tag oracle 移除,零行为变化]** 新 API
  `SceneGeometry.contact_pairs`(shell 内逐 primitive (h, 接触方向),计费
  = 1 次碰撞查询,active pair identity 属方法合法信息)+
  `bilateral_rho_tagfree`(对侧分组自动化:冷启动按接触方向排序角的两个最大
  circular gap 切分,GAP_MIN=45° 设计期冻结;沿 march 用上一次评估的参考方向
  做时序 signature 连续性匹配;缺侧读 RHO_CAP;碰撞区返回精确 PW 值而非
  side_rho 的 −0.5 quick-collide 标记——严格更多信息)。
  **验证(`tagfree_validation.py`)**:与 oracle side_rho 在全部 free pose 上
  逐位一致——309/309 v3.1 沿脊站点(时序链)+ 2925/2925 墙域网格 pose(冷);
  bilateral 检测分歧仅在碰撞区(~4%,语义差异所在)。
  **端到端(v4 tracker,TAGFREE=True,oracle 路径保留为声明消融开关)**:
  五个认证宽度与 v3.1 结果逐位一致(全部成本/余量/站数不变);w=0.49
  abstention 不变(c_seed 差 43 查询,碰撞候选语义差异,预期内)。
  "door-geometry-metadata-free under oracle side grouping" 的限定语正式解除
  → **fully tag-free discovery loop**(cold-start 诊断臂与 cert-only 分母臂
  仍 oracle,声明不变)。v3.1 归档 archive/ + `*_v31.json`。
- **[P1b 完成:branch-aware gate chart + 认证跳枝协议(v4.1,三轮 dump 迭代)]**
  (v4.0 归档 archive/;数据 `ridge_stations.json` 的 chart/events 字段)
  结构:`section_eval` 输出截面**全部 sampled-free components**(K=15 冻结扫,
  组件=连续 feasible θ-run,记 th 区间/m_best/m_min/q_best),按触发器
  seed / multimodal(theta_opt 粗扫免费多峰提示)/ reject / backtrack 记入
  diag["chart"];事件 lobe_death / branch_switch / connector_cert_fail /
  section_blocked / march_death 记入 diag["events"]。
  **修正 v2.2 图景**:w=0.505 mouth 截面实测不是分离双 lobe,而是**单 component
  经薄颈连通**(163°..189°,m_min=0.0017≪m_best=0.070)——"两 lobe"是
  坐标下降视角;分离与薄颈两种拓扑都被 chart 如实记录。
  三轮 dump 驱动迭代:(1) 直接移植 v3.1 逻辑 → 跳枝被两条旁路绕过
  (theta_opt 全局 argmax 静默跨 lobe;濒死 lobe 以 m≈1e-4 阻止 death 触发),
  w=0.505 回退 CERT_FAIL −1.08;(2) **corrector lobe-local 化**(theta_opt
  只在含来向 θ 的 feasible run ±2 扫描步内取 argmax,找不到即失败走协议)
  +"合并 component"回退条件过严改为"证书裁判"(固定 (x,y) 截面未合并不代表
  (y,θ) 联合 connector 不自由)→ 仍失败:采样连通 component 的**直线弦可穿出
  薄颈**,未认证跳变 −1.08;(3) **凡截面导出跳变必须当场认证**:connector 先
  按 witness 密度 2cm 抛光(`polish_segment`,与主 densify 共用原语)再
  certify_path_conservative,计费 c_connector;从濒死站起跳虽可认证但贴颈
  (min 余量 0.05mm)→ **锚点按记录站 margin 降序尝试**(评审"返回最后一个
  健康 component"的操作化,无新阈值),splice 丢弃死亡爬行尾。
  m_esc/2 修剪逻辑整体删除(评审五问 #2 落实)。
  **终版结果**:六宽度全对,min 余量 2.34/4.68/9.85/1.37/17.18mm 与 v3.1
  逐位一致(路径质量零损失),窄三档各 1 次认证 branch_switch(connector
  余量 10.69/9.4/9.9mm 量级,12–13 checks),total 11.4–15.8k(chart+connector
  溢价 ~0.3–0.5k);**关门 w=0.49 的 chart 结构化记录死亡**:x=−0.62 截面塌缩
  至单点 m≈0 → 三次 BLOCKED → march_death → abstention——closure certificate
  (Gate C)的直接原料。未做(如实):截面间显式 adjacency 边/birth-split-merge
  事件推导(chart 数据已含所需信息,推导器未写);component 的
  contact-signature 字段;goal-independent 多 gate 图(P4)。
- **[P2 泛化跑批(cn263 直跑,队列拥挤,用户指示)——红线保持,域边界清晰,
  外加一个超预期发现]**(`p2_generalization.json` + `outputs/p2_run.log`;
  HPC 侧 STATUS.md/phase_002 已记;42 案例 673s;v4.1 超参全冻结,
  run_case 仅参数化 robot_id)。判定构成:
  **FALSE_REACHABLE = 0/42**(红线);certified_true_positive 13;
  correct_abstention 10(全部是物理关门的 R_big_circle,含 NO_SEED 与
  TRACK_LOST 两种诚实路径);abstain_on_reachable 13;oracle_unresolved 6。
  三个结构性发现:
  ① **"薄"是机器人相对的,kernel 恰好在每个机器人自己的 thin regime 工作**:
  长椭圆 0.55–1.10 全宽度认证(其体长使宽走廊仍是双侧接触);大圆在 w=0.90
  (对它 δ=50mm)认证;小圆只在薄门认证。
  ② **6 个 ORACLE_UNRESOLVED_GRID 案例全部被构造性认证 REACHABLE**
  (余量 7.4–24.9mm,逐案贴各自解析裕度 (w−2b)/2)——fine 网格
  (288×185×281)解不了的临界 de-aligned 案例,tracker 用 ~10k 查询给出了
  保守见证。这是"表示层 vs 路径层"论点的又一实测:也是基准数据的实质补全
  (待把见证路径回填 oracle records,列入待办)。
  ③ **域外区 = 机器人相对的宽门**:12 例 NO_SEED(250mm 网格上 bilateral
  候选为空——宽走廊里单侧 shell 看不到对面墙;这不是 tracker 缺陷,是
  Atlas 还缺 open-space chart 的证据);1 例 val_016(椭圆 w=0.80,
  tilt=+12° 最大倾角)CERT_FAIL −1.21mm——tracker 穿了但证书如实拒绝,
  是唯一的质量失败,**按 P2 纪律不修不调**,dump 留给下轮诊断。
  认证案例成本 9.9k–14.4k,跨 regime 平坦。
- **[val_016 只读诊断]** seed(第 6 候选)落在宽门 regime 的 jamb 肩部双侧
  结构(θ≈101°),march 沿 jamb 爬升(101→132°;两次局部 connector 认证
  通过、一次 −28mm 正确拒绝),全程未进门的朝向带(±34.9°@12°);密化后
  爬升段一节 −4.72mm,最终证书如实拒绝(row 报 −1.21 为 early-abort 部分
  观察)。定性:域外 seed 脱靶追踪,可靠性链条在岗;修向与宽门 NO_SEED
  同根(gap 双侧 vs jamb 肩部双侧的 seed 区分 / open-space chart),推迟。
- **[P3 matched-budget = Gate B 决定性实验(cn263 直跑)——三基线五宽度
  全灭,差距 ≥8–11×(下界)]**(`p3_matched_budget.json` +
  `p3_matched_budget.png`;floor 同码重跑 `floor_probe.json`)。
  冻结 ProbeTree 评估器(uniform/generic/contact-v4),阶梯延至 131072,
  同代码同 checker 同计费;对照 = v4.1 continuation 认证 total(同码,
  provenance 关联)。结果:**三基线在全部五个 de-aligned 临界宽度上
  ≤131k 无正确 REACHABLE(含 δ=40mm)**,持续 false-unreachable;
  continuation 11.4–15.8k 输出认证路径(基线任务更容易——只需连通性判定
  ——仍全灭,对比方向保守)。表示规模 114,870 叶 vs ~10² 站点。
  iso oracle-tube floor 重跑逐位复现 round-2(24k/96k/>128k @ tube 0.2)
  ——完美调度下界(24k)已超 continuation 全管道。hybrid floor 未重跑,
  round-2 引用维持 censored 标注。**Gate B 冻结判据在全部五个临界 gate
  满足;状态翻转交评审判定**。round-4 送审材料:
  `docs/reports/ridge_round4_report.md`(P1+P2+val_016 诊断+P3,评审问题
  五条)。
- **[round-4 评审回执+P0.5/P3.5 处置(08-14 凌晨)]** 判定采纳:
  **Gate B-G1 = PASS-CONDITIONAL**,kernel 冻结,撤 1000× 表述,
  P2 改称 within-family validation,"budgeted false negative" 术语采纳,
  长椭圆改"12/14 open 配置"。评审的 P2 重算(50%/59.4%/95% 条件分解)
  当场复核逐位吻合;评审抓到的三个 provenance 漏洞(gamma arm v3.1 stale、
  floor 无 prov、131240 actual)全部属实并修复。条件项执行(cn263,
  `outputs/p35_run.log`):
  ① **正控制通过**:w=1.10 对齐档 uniform@16k/32k、generic@32k、
  contact@8k 成功 ⟹ 薄门全灭是实例难度非 evaluator 失效(w=0.70 小圆
  32k 帽内未过,控制预算偏紧,如实记录);
  ② `_answer` 增 goal_resolved/双连通性/failure taxonomy(纯测量);
  ③ **三层计费**:pair_ops/bp_hits 计数器;continuation w=0.505 =
  2.22M pair_ops(≈140/查询)vs uniform 2.62M(≈20/查询)/ generic
  4.53M / contact-v4 19.7M ⟹ 以 pair_ops 计 continuation 仍 ≤ uniform;
  ④ gate-band 访问:基线到过门带(1.5–2.3k cells)但 θ 窗命中极少
  (uniform 80–248、generic 314–576、contact-v4 0–6)——θ 分辨是死因;
  ⑤ **scalar-clearance 同架构消融:0/14 认证 vs pair-contact 6/6**
  (K=8 全 NO_SEED=seed 选择性;K=64 宽限档窄档能 seed 但 march 全死
  mouth=脊条件化)——Claim E 首个直接证据(限定:信息替换消融);
  ⑥ **witness 升级协议**:6 案 amendment(双 checker 2mm 重放零违例,
  原记录未动,排除出独立准确率统计)`outputs/oracle_records_amendments/`;
  ⑦ p2 truth schema 防护(unresolved→reachable=None)。
  数据:`p3_matched_budget.json`(v2)、`ridge_scalar_ablation.json`、
  `gamma_delta.json`(arm 修正)、`floor_probe.json`(带 prov)。
  处置附录置于 round4 报告顶部。
- **[round-5 评审(领导实际解包核验)→ P0-R + P3.6 + P4a 全部执行完毕,
  终包机器验证 PASS(08-14)]** round-5 判定:Gate B-G1 内部 PASS、
  评审包 FAIL(三处"报告说已修、文件未同步"全部属实,本地逐一复核确认)。
  执行:
  **P0-R**:单一冻结源树(c45ed2b394a6)一条链重生成全部 canonical
  (sweep/p2/gamma/p3/p36/scalar/witness/floor;cn009 起跑、负载飙到 14 后
  尾链迁 cn263,跨节点逐位复现;OpenBLAS 单线程钉死修掉 60× cgroup 争用
  减速;setsid 修掉双层 ssh 的 nohup SIGHUP 竞态);干净版
  `round4_final_report.md`(无勘误分层,全部收缩口径内嵌);图 provenance
  sidecar 机制;源码快照 + `reproduce.sh` + **`validate_package.py`
  七类机器检查作为发布门**(占位符/禁用措辞/hash/schema/图-JSON 一致性);
  `outputs/round4_final_package.zip` **PACKAGE PASS**(77 文件)。
  **P3.6**:2×2 宽门矩阵——uniform/generic 在全部四个
  aligned/de-aligned 宽格 32k 成功 ⟹ **thinness 单独隔离为致命因子**;
  pair-aware ProbeTree(方法同款 tag-free pair 信息、体积表示、无
  continuation)五个 thin 主跑全 NEVER 且 frontier 不在门带 ⟹
  **continuation 架构承重,非 oracle 更富**;全部失败 run 的
  ambiguous-cut 定位(thin 主跑:start-FREE frontier ~10k cells、
  门带内 0——保守认证未及 mouth 逼近段)。
  **P4a**:`src/splatc/compiler/{atlas_types,api}.py` 六类对象
  (sampled/certified 分离、adjacency 仅经 CertifiedTransition、
  多假设共存、atlas_hash 为 goal-independence 见证)+ 冻结 API
  (compile 收 goal 类键即 ValueError);合同测试 5/5,全套件 25/25。
  witness 重放跨平台采样数 ±10%(libm 1ulp→ceil),margins/违例数
  逐位一致。三条禁令执行中(未动 GAP_MIN/val_016/seed tries)。下一步:P4a(handshake 六类对象:
  SectionComponent/ContactSignature/GateBranch/CertifiedTransition/
  OpenChart/AtlasEdge;branch IDs 与 adjacency 必须先有)→ P4b 两门
  多目标(compile 一次、无 goal、两 goal 各自 min-plus 选门、不重编译、
  统一 checker)→ P4c(多假设 seed + region incidence 自然排除 jamb)
  → P5(Gaussian residual audit vs SDF/CDF)。

## 2026-08-15 P0-R2(round-6 救命顺序第一项):两层发布门

round-6 判旧包 FAIL 的四项可查主张全部本地验证属实(快照缺 manifests +
oracle records→复现必崩;域体积比 2.981;margin_profile 在 pair_ops 读取
前运行;amendment 键 independent_replay 残留)。处置(树 c45ed 未动,
canonical 表全部保持有效):

- build_package v2:dev+validation manifests、42 份 oracle records、
  split_seals 进快照(blind 永不进包,validator 强制);
  compare_reproduction.py 随包;environment.yml 精确锁(overlay 实测
  py3.11.15 / numpy1.26.4 / scipy1.10.1 / mpl3.11.1)。
- validate_package v3(第一层,静态):快照 src 树哈希**重算**并核对全部
  provenance 主张(8 表+6 amendments+图 sidecars);生成脚本哈希重算
  (stale 表即 FAIL);reproduce.sh 输入依赖 preflight(逐 episode 查
  record);MANIFEST 全覆盖(未列文件即 FAIL);新禁语。对抗自测:删
  record + 改 src + 塞禁语 → 6 FAIL 全抓。
- compare_reproduction.py(第二层,动态):status/verdict/查询计数/
  pair_ops/bp_hits/站数/事件数/summary 精确相等;浮点容差 1e-6;计时字段
  跳过。正向(包 vs 本地 canonical)14 artifacts MATCH;负向(±1
  pair_op、+1e-3mm)全抓。
- 报告五处句修:机制 PASS / residual HOLD 拆分;G1-frame conditioning
  注记(|x|>0.7、±0.65、seed 盒为几何先验,旧英文限定语撤回);validator
  claim 限定范围;相互必要性降级为实现级表述;dual_formulation_replay
  键名。另加域口径注记(2.98×,residual HOLD 待 P3.7)与计费边界注记
  (margin_profile 污染,方向不利于 continuation,拆分随 P3.7)。
- 图:gamma 右标题两行化(裁剪修复);新增 p36_fairness.png(pair-aware
  臂 + 2×2 控制矩阵,never 标记逐臂错行防遮挡,双 provenance sidecar)。
- 净室验收(第二层)cn009 启动:解包→validate→reproduce 全链→compare,
  log=outputs/p0r2_cleanroom.log。amendments 有跨平台 libm 采样数 ±10%
  既知风险(本 worklog 08-14 条),canonical 改由 HPC(锁定环境)生成
  ——结果见追记。

追记(amendments canonical 定版):HPC 锁定环境重生成 6/6——samples
4098/2829/4207/2682/3860/3883,viol 全 0/0,min 余量与旧 canonical 逐位
一致(24.933/7.356/9.953/9.928/9.95/9.927mm)。mac 版三例采样数漂移
(dev_003 4622、val_000 4110、val_003 4005)证实既知 libm 跨平台效应;
canonical 采 HPC 版,包重建(130 文件,layer-1 PASS + 对抗自测 6 FAIL
全抓),净室从新 zip 重启(cn009 pid 3352300, 2026-08-14T20:29:58Z,
log=outputs/p0r2_cleanroom.log)。P3.7 侦察:ProbeTree 域来自 scene
bounds + 硬编码 TWO_PI(probe_methods.py:61),ROI/θ-商匹配需动 src/ →
树漂移 → 全链重跑(reproduce.sh 已自动化,成本已知)。

追记 2(净室结果,08-15):首发净室(cn009 直连)于 ~6h 处被借宿 SLURM
cgroup 清场静默杀死(sacct:宿主任务 COMPLETED;OOM 排除,内存空闲
437G;日志无 Traceback = SIGKILL 签名)——**直连多小时链 = 宿主墙钟
抽签,教训入册**。死点仅剩 floor_probe+画图+对比;外科收尾后
**compare_reproduction 14/14 artifacts 全 MATCH,float maxdev 0.00e+00**
(含 ridge_stations 50,500 leaves、6 amendments)——锁定环境下逐位复现
成立。正式单发验收改走 sbatch 自有分配(job 17247388,compute/cair,
12h 墙钟),日志 outputs/p0r2_cleanroom_sbatch.log;通过后方改
HANDOFF 发布状态。

## 2026-08-15 P3.7 实现(域匹配基线,评审救命顺序第二项)

- src 改动(树漂移,defaults A/B 验证四 policy 逐位不变):ProbeTree/
  AnisoTree 加 roi/theta_span 参数(θ 商仅限中心对称机器人,脚本内
  π-周期性重放断言);AnisoTree 加 pair_info 钩子(同款 tag-free
  bilateral)与 P3.5 诊断 _answer(保守语义不变);两树 _answer 在
  checkpoint 处快照 pair_ops/bp_hits(三币种)。AnisoTree 此前零使用者。
- ridge_continuation.py 计费拆分(round-6):core(seed+track+densify,
  含 inline connector 认证)/ cert(+终证书)/ 总(+diag margin_profile)
  三个时点快照,旧字段含义不变。
- p37_matched.py:ROI=seed 盒 (−3,3)×(−1.8,1.8)、θ∈[0,π) 商,四臂
  iso_uniform_q / aniso_{uniform,generic,pair}_q,同预算阶梯至 131072,
  正控制(w=1.10 aligned×2 robots + de-aligned),树无关 cut 定位,
  kill_condition_hit 字段(任一匹配臂 ≤2× continuation)。烟测:商域
  断言过;匹配域下控制门 8k 即解(iso/aniso-uniform/pair);细门小预算
  尚 NEVER(正常,真跑到 131k)。测试 25/25。
- p37 入 reproduce.sh 与 build_package TABLES;新树全链(全部 canonical
  重生成 + p37)以 sbatch 17247454 提交(borrowed-cgroup 教训,不再直连
  跑长链)。与 P0-R2 验收(17247388,冻结 zip,互不相干)并行。

追记 3(P0-R2 正式关账,08-15):sbatch 17247388(自有分配)单发不间断
6h28m,ExitCode 0:0——解包→layer-1 validate→reproduce.sh 全链→layer-2
compare **14/14 artifacts MATCH,float maxdev 0.00e+00**,
CLEANROOM_ALL_PASS。round-6 P0-R2 验收判据(净室全 exit 0)满足;
发布物 outputs/round4_final_package.zip。

追记 4(P3.7 判定,08-15):sbatch 17247454 完成(新树 5353fc1c6d97 全链
+ p37)。**全尺度不变性:8 张 canonical 表旧字段 vs c45ed 零差异**(树
漂移未改任何冻结行为)。P3.7 结果:**四匹配臂 × 五宽度 = 20 主跑全部
NEVER(131,240/131,132 actual),kill condition 0/5 触发**;正控制
2k–16k 全过且普遍优于未匹配臂(域匹配生效,all-NEVER 非 evaluator
假象);失败臂 cut frontier 门带 0/0 依旧(匹配域内仍死于逼近走廊);
计费拆分:诊断占 continuation pair_ops 30–42%,干净口径(core+cert)
pair-op 优势 ≥2.2×(对 iso uniform 匹配臂,下界)至 ≥13×(对
pair-aware 匹配臂)。报告更新:matched-volumetric residual PASS,
非体积近邻基线(RRT-Connect/HRM/NavMesh)列为 headline 前必做;
p37_matched.png(锁定环境生成,双 provenance sidecar)。

## 2026-08-15 P4a 增量 1(结构层,评审第三项;树再漂移,下次打包前再跑链)

- Atlas.add 重复 ID 改硬错误(round-6 点名的静默覆盖);atlas.validate()
  落地:悬挂引用/AMBIGUOUS 带证书/同 chart 伪 gate/未认证 chart 上的
  edge/无或悬挂 witness/非正证书/goal 化 provenance(递归 key 扫 + 良性
  短语剔除后的文本扫)。测试 10 类违规逐个触发,合法 mini-atlas 零违规。
- chart_builder.build_open_charts:goal-free 粗认证 FREE 骨架→连通分量→
  OpenChart(证书语义:成员 cell 全域认证,面邻接并集保守连通);可选
  domain mask(处理斜角 AABB 无法表达的旋转矩形域)。locate_chart 零
  查询。datasets/transforms.py:disc 场景精确刚体变换。
- p4a_rigid_kill(chart 层,φ∈{0,30,60,90,137}°+平移):首跑暴露
  harness 泄漏(斜角 AABB 含未密封外部,绕墙端连通)→ mask 修正后:
  **细门 0.505 五角度全不变**(2 charts、分离、raw count);**宽门 1.10
  不变性破**(30° 并为 1 chart)——机制解读:宽门真拓扑本就连通,0° 的
  "2 charts" 是粗分辨率假象,旋转暴露之;粗体积分辨率在可通过门附近
  frame-sensitive,正是 gate 需要 branch/transition 层的结构性论据。
  kernel 级等变换测试在 P4b compile(scene,robot) 前不可跑(如实记录于
  表内 kernel_level 字段)。测试 37/37。表 p4a_rigid_charts.json 入
  reproduce.sh 与 build TABLES(随下次树 bump 打包)。

追记 5(验收 v2 通过,08-16):sbatch 17252937 单发不间断 8h57m,
ExitCode 0:0——树 5353fc1c6d97 重算 21 处 provenance 全匹,15/15
artifacts(9 表含 p37_matched 3,728 leaves + 6 amendments)逐位 MATCH
maxdev 0.0,CLEANROOM_ALL_PASS。round4_final_package.zip(P0-R2 双层门
+ P3.7 判定 + 更新后报告)= round-7 送审候选。

## 2026-08-17 P0-R3(round-7 处置;树 bump,全链 17261306 跑批中)

round-7 裁决:Gate B–G1 正式 PASS;P3.7 改名 matched conservative
volumetric-family separation(PASS 限实现族);论文级 residual HOLD;
旧包因 provenance method_arm 残留已撤回限定语判 FAIL。五项可查主张全部
本地核验属实(含评审逐位复现 w=0.49 core 557868/815628)。处置:

- arm 字符串改 "continuation-v4.1 (tag-free G1 contact grouping;
  G1-frame-conditioned exit/span logic)";另清 gamma_delta.py docstring
  里评审未点名的同类残留("v3.1 metadata-free")。
- prov.py 增 code_bundle_sha256(src+experiments/compiler 全 *.py,相对
  posix 路径参与哈希,排除 archive;覆盖传递 import——gamma→ridge、
  p37→p3→p36)。validator v4:禁语扫全部 provenance(表/amendments/
  sidecars);bundle 从快照重算比对;回执若在须字段全、三 exit 0、且
  MANIFEST 剔除回执行后的哈希 == 回执记录的内容包 MANIFEST(闭环)。
- exact face-overlap 邻接(round-7 公平性修正):ProbeTree(split_set
  + 祖先行走/劈分下降)与 AnisoTree(区域下降 + 区间重叠,θ 环绕)
  全部连通性(_answer/cut/chart_builder)切换;**金标准单元测试:小树
  上 O(n²) 几何暴力 oracle 逐叶一致,旧 face-center 探针为真子集且
  hanging faces 确有出现(3 测试)**。本地预览:w=1.10 正控制
  first_success 不变(32000/32000)。
- p37 正控制补全 2×2 格;P4a validate() 增 opposing_pair 索引与
  self-transition 检查,test fixture 修正评审点名的两处语义瑕疵
  (单 cluster 配 [0,1]、c0→c0)。测试 42/42。
- 环境:overlay 实测 pip freeze(cn222)入包;报告措辞收缩为"核心数值
  依赖精确固定 + 最低版本约束 + freeze 证据"。
- 回执机制:write_receipt.py(验收作业末尾在锁定环境内落
  cleanroom_reproduction_receipt.json:内容 MANIFEST 哈希/zip 哈希/
  bundle/host/freeze 哈希/起止/三 exit/逐表 sha256);两趟发布
  (内容包→验收→--with-receipt 终包,防旧回执污染)。
- 报告:采用评审批准的唯一 claim 口径;P3.7 改名;residual HOLD;
  正控制 2×2;邻接语义注记;§7 两层门 + bundle + 回执描述。

追记(P0-R3 关账,08-17):验收 v3 = sbatch 17264045(cn225,自有分配)
单发 10h11m ExitCode 0——validate(树 c9cd4676375b + bundle 05b5fa061966
双重算 22 处全匹)→ reproduce 全链(42 测试 + 10 表 + 6 amendments)→
compare **16/16 artifacts 逐位 MATCH** → 回执落地(三段门 0/0/0,内容
MANIFEST c2c63d19733c…,freeze 哈希在案)→ CLEANROOM_ALL_PASS。终包
= 内容 + 回执(149 文件,layer-1 回执闭环校验 PASS)。送审件:
outputs/splatc_atlas_round8_tree_c9cd4676375b.zip
(sha256 5a7286e6…6351c85e)。round-7 全部 blocker 清零:provenance
残留、bundle 身份、环境措辞、第四正控制、exact 邻接(判定不变)、回执。
下一步(评审指令 #6):P3.8 chart-factored connector + RRT-Connect +
local PRM/bridge,kill condition 2×。

## 2026-08-18:round-8 处置——P0.8 + P4a.1(单次 tree bump,进行中)

round-8 裁决全部指控逐条核验属实(含两个动态诊断本地逐位复现:30k
预算宽门五旋转恢复连通;mask 角点越界 cell 正好 114 个)。执行:

- **P4a FAIL 披露**:round-7 15k 固定预算 chart-count kill 的 FAIL
  数据冻结为 records/p4a_rigid_charts_round7_15k_FAIL.json(provenance
  身份 c9cd4676375b 在文件内;validator 7b 内容互检 + 豁免现树等价);
  流程失职(数据入包、正文写"待做")在 sprint_c 报告 §1 如实声明。
- **报告拆分**:round4_final_report.md 退役入 docs/reports/archive/;
  新 gate_b_evidence_report.md(Gate B 证据,关账)+
  sprint_c_p4a_report.md(chart 层,含负结果与 P4a.1);"精确环境锁"
  与"P4a kill test 待做"入禁语表。
- **chart_builder v2**:wave-complete refinement(波开销开波前已知,
  放不下不开工;分区只依赖 scene/domain/budget band);cell-domain
  三态判定(domains.py,IN/OUT/CROSS,SAT 精确;成员=FREE∧IN;
  114-cell 泄漏类回归测试);region 序列化 certified cell cover
  (grid+cells+邻接+逐cell证书余量+domain)+ chart_contains/
  chart_connect(纯 region、零查询、折线在认证并内+oracle 复核)。
- **OpenPortal ≠ ContactGate**(portals.py):通用直线+保守证书连接
  认证,θ-列多样性候选,零门先验;判别实测:宽门认证、thin/sealed
  全拒且逐尝试入账。atlas_types:OpenPortal/ChartAttachment 新类 +
  validate() 九类 witness-chain 校验(评审 §8 清单);schema 冻结
  说法撤销。
- **p4a_semantic.py**(替代退役的 p4a_rigid_kill,v1 入 archive/):
  3 门型 × 9 变换(5 旋转 + 2 固定网格平移 + 2 origin 相位)×
  预算阶梯 {15,30,60}k;稳定判据 = converged(max_level/no_frontier
  停机 ⇒ 更大预算构建恒等,可证明)或 band(30k=60k);本地冒烟:
  **OVERALL PASS**(thin 9/9 双核+attachment C000≠C001;wide 9/9
  连通+9/9 逆变换 query 认证 0.041–0.229m;sealed 9/9 双核零
  transition;wide R0/R90 的 15k↔30k 翻转如实入 low_band_agree)。
- **发布门升级**:validator 7b(负结果一致性机器互检)、records/
  豁免、双报告、receipt 字段改名 verified_content_zip_sha256、外置
  .zip.sha256、build_package --with-receipt 提示修复、environment.txt
  措辞收缩。测试 42→62 项全绿。
- **HPC**:树同步镜像;全链 sbatch 17286328(compute/cair,14h 帽)
  已提交,PENDING(Priority)排队;完成后:拉回 canonical 表→核对
  报告引用数字→内容包→验收 sbatch(净室+回执)→--with-receipt 终包
  →round9 送审件。

追记(08-19,验收第一发失败与处置):全链 sbatch 17286328 COMPLETED
11h00m ExitCode 0——P4A1 OVERALL PASS(canonical)、P3.7 kill 0/5 维持、
continuation 逐位复现;内容包 layer-1 PASS(树 10d00fb69494 / bundle
ca4d0f29acaa,22 处双重算全匹);验证器对抗自测 6/6 篡改包全被抓
(其中 3 个由新增 7b 负结果互检拦截)。验收第一发 sbatch 17304460
FAILED @10h23m:V=0、R=1——42 测试 + 全部表复现完成后,画图段写 PNG
报 "Disk quota exceeded"(/scratch 账号级配额:文件数 735k/512k=144%、
字节 92%,由本账号其它项目占用;我方净室目录仅 ~6MB)。处置:清掉
我方全部旧净室目录;验收脚本 v4.1 把净室工作区改到 node-local tmpfs
(工作集 <1GB,一次性;--mem 提至 32G 覆盖 cgroup 计费;回执与日志仍
落 /scratch),重发为 sbatch 17320798。科学结论不受影响(失败发生在
数值链完成之后的图渲染)。

追记(08-19,round-9 关账):验收 v4.1(tmpfs 净室,sbatch 17320798,
cn222)10h19m exit 0——16/16 artifacts 逐位 MATCH(float maxdev 0.0,
含 p4a_semantic_invariance 2157 leaves)、CLEANROOM_ALL_PASS、回执
0/0/0 + MANIFEST 闭环。两趟合同执行中发现验收脚本是打内容包之后新建
的文件 ⇒ 从树移出(ops_scripts/,树外),终包 = 被验证内容 + 回执,
159 文件,layer-1 含回执闭环 PASS。回执掉包自测(T7)被闭环检查抓住
(累计 7/7)。送审件 splatc_atlas_round9_tree_10d00fb69494.zip,
sha256 7956ed67d8f216d8b029fff33ec89f026f90f6044786da587a8e0fce966b613e。

## 2026-08-20:round-10 处置——P0-R9 + P4a.2 + P4a.3(单次 tree bump,进行中)

round-10 全部指控核验属实(四个动态发现逐位复现:R30 越界 4,672/5,860;
witness 首末=start/goal、ridge 端点 locate=None;validate 三变异静默;
R0 计费累计;portal 宽度扫描非单调+w=0.62 非对称)。执行:

- **round-9 判定撤回落地**:p4a_semantic_invariance.json 冻结为
  records/..._round9_WITHDRAWN.json;validator 7b 增撤回互检(记录内
  必须留着被撤回的 PASS + 正文原样携带撤回声明);禁语增补三条。
- **域 support 证书**(blocker 1):domains.support_margin(朝向相关
  四边余量,1-Lipschitz 同度量)进成员资格第三条件 + 不足 FREE cell
  继续细化 + region 双 slack;R30 成员 11,116→1,464、越界 0/0;
  identity 构建逐位不变。portal/attachment 叠加域 bubble 证书。
- **goal-free 编译**(blocker 2):enumerate_portals 无位姿参数、全
  major 无序对、canonical 化+双侧 θ-列并集保证 (A,B)≡(B,A)(w=0.62
  回归);tube 半径证书;类别改 generic_certified_connector(证明
  来源,非分类器)。
- **真 gate 链**(blocker 3):build_gate_chain 从 witness 拆
  attachment–branch–attachment 并全新认证(branch 重认证 2.336mm,
  端点站实测 bilateral 构造 signature);compile_atlas 装配 + 身份
  绑定(scene/robot/domain/checker hash);query_reachable 纯序列化
  查询;10 随机 query 对 hash/计数零漂移。
- **validate() 语义化**:region/portal/attachment/edge-chain 四个
  几何校验;评审三变异 + 假邻接 + 未绑定 + 断链共六类全被抓。
- **计费**(§7):fresh scene per run。
- **阶梯至收敛**:150k 顶档保证 max_level=3 收敛;第一轮本地 FAIL
  (R60 类在 60k 差 ~100 查询进不了 level-3,level-2 相位下直线
  connector 不可证)由此暴露并以收敛档原则性解决(非调 connector)。
- **P4a.3 sealed 验证集**(134 builds,种子 20260820):S1 宽度阶梯
  ×3 形态+plug、S2 30 随机刚体变换、S3 10 随机网格相位、S4 层帽
  {2,3,4}、S5 offset/tilt;独立 soundness 审计逐 build。
- **本地 canonical 候选双 PASS**:P4A2(27 变换全 converged 判定;
  atlas 原型 0 违规、compile_q 18,355、逆变换 9/9 复证 0.151–0.212m)
  + P4A3(sound_viol=0 / plug=0 / thin_bad=0 / wide_bad=0;收敛档
  阶梯规整,round-10 非单调确证为 level-2 锚点伪象)。
- 报告:sprint-C 重写(双负结果记录 + 撤回声明 + 收缩命名);Gate B
  仅改成本引用句;四标记完好;禁语零命中;测试 62→73。7b 定向对抗
  自测 4/4 CAUGHT。链 sbatch 17347389(16h 帽)已提交。

## 2026-08-21 追记:round-10 发布闭环(送审件就绪)

- **链第一发 17347389 9s 即死**:"EnvironmentNameNotFound: splatc"——
  conda env 在 08-20 配额清理中被删(env 在 overlay 外
  /scratch/.../conda_envs/splatc,经 ~/.condarc)。sbatch_rebuild_env.sh
  按精确 pin 3 分钟重建(版本断言 + 73/73 smoke + freeze 重测回拉 +
  explicit spec 存 overlays/splatc_env_explicit.txt)。
- **canonical 链 17347507**:9h20m exit 0,P4A2 PASS + P4A3 PASS,
  全部报告引用数字与本地逐位一致(compile_q 18,355、branch 2.336mm、
  逆变换 9/9 0.151–0.212m、134 builds 判定输入全零)。
- **验收第一发 17355854 75s 即死**:test_assemble.py fixture 读
  results/tables/ridge_stations.json,净室里 pytest 先于链跑、表不存在
  ——fixture 改为经 run_case 现场重生成 witness(自包含;tests 不进
  tree/bundle hash,链无需重跑);本地把 results/tables 挪走复证 73/73。
  内容包重建:zip sha 07da7e1e…(树 9bb7e1df3252 / bundle 92c8d4f6455c
  不变)。
- **验收 17355868(cn024,tmpfs 净室)**:10h41m exit 0,
  **CLEANROOM_ALL_PASS**:净室 validate PASS + 73/73 tests +
  **17/17 artifacts 逐位 MATCH(float maxdev 全 0.00e+00)**;回执三段
  门 0/0/0,verified_content_zip_sha256 = 07da7e1e… 与内容包一致。
- **终包**:回执拉回后 `build_package.py --with-receipt`(树零改动,
  两趟合同;layer-1 含回执闭合 PASS)。防篡改自检 3/3:回执 sha 翻位
  被抓、表篡改被抓两处(sha + 判定引用互检)、复原后 PASS。
- **送审件:`outputs/splatc_atlas_round10_tree_9bb7e1df3252.zip`
  sha256 `f5b10ff2f0870373235e5737bf76a05cda4e64901ce30f244fdb38ddf928a87d`**
  (外置 .sha256;= round4_final_package.zip 同字节)。交付物 + 回执已
  rsync 镜像并远端 sha 校验 OK(传输中出现一次 Lustre 间歇写入告警,
  rsync 自动重传后校验通过——配额风险仍在)。

## 2026-08-21:P3.8 chart-factored connector 基准(round-11,进行中)

评审 rounds 7/8/10 预注册的生死实验。同给 F_L/F_R(converged 150k
goal-free 构建的两最大 chart 序列化 region),零门先验(守卫 dict
机械强制不读 scene.meta),六臂统一计费(BilledChecker + 场景
pair_ops/bp_hits + 墙钟),认证连接件才算成功,独立场景复检零假声明,
budget 262,144,采样臂 10 种子。新码:baselines/connector_arms.py
(RRT-Connect 双树 π-商采样 + lazy PRM/bridge 懒边验证 + HRM 风格
θ-层扫描;嵌入 KDTree 近邻;共享 retraction polish)、
p38_connector_bench.py、tests/test_p38_arms.py(6 tests,含守卫与
同种子 bit-重放)、validator 7c(kill/负控制/参考数/WITHIN-2x 四类
引用互检)。kernel run_case/run_case_scalar 仅加纯参数化透传
(door_plug / return_geometry),逻辑与计费零改动。

**本地候选结果(canonical 待链 17359268)**:
- thin 0.505:contact 16,462q 全链认证 2.34mm;**rrt_connect 10/10,
  median pair_ops 0.73×、墙钟 0.84×(nq 2.87×)→ 预注册任一货币
  ≤2× 规则触发,P3.8 kill condition: HIT**(即便按三货币齐进读法也
  触发);PRM 0/10、HRM 0/1、scalar NO_SEED、generic 全拒。
- mid 0.62:generic 157q/5.81mm;基线全通(公平性证据);contact
  双侧接触丢失如实入表(互补制式)。
- plug 0.505:全臂拒绝,false claims 0 → 负控制 PASS。
- 判定后主张变更按预注册协议执行:单查询过门效率撤下主 residual,
  主线转 compile-once/query-many(P4b 载体);P3.7 的 8–11× 体积族
  分离与 Gate B 机制结论不受影响。报告
  docs/reports/p38_connector_report.md 全部标记/数字程序化对齐表。
- 净室按用户指令(08-21)改回 /scratch,不再用 tmpfs。

## 2026-08-22 追记:round-11 发布闭环(P3.8 送审件就绪)

- **canonical 链 17359268**:13h02m exit 0;P38 overall 与本地一致
  (kill HIT / 负控制 PASS / false claims 0)。contact 三货币逐位同
  本地;RRT 有平台级小漂移(median nq 47,235.5→47,204.5,墙钟比
  0.84×→0.91×),报告全部改用 canonical 值并程序化校验 20 个引用。
- **打包坑**:链再生的 oracle_amendments 在
  outputs/oracle_records_amendments/(非 results/),首次打包因树戳
  不匹配被层-1 正确拦截;补拉后 PASS。round-11 树 e934bb006541 /
  bundle 9b8540838116,内容 zip 31bfc966accd…。
- **验收 17366850(/scratch 净室,遵用户 08-21 指令弃 tmpfs)**:
  12h58m exit 0,CLEANROOM_ALL_PASS:净室 validate PASS + 79/79
  tests + 18/18 artifacts 逐位 MATCH;回执三段门 0/0/0,
  verified_content_zip_sha256 与内容包一致;/scratch 全程无配额事故。
- **终包**:回执后 --with-receipt(树零改动),防篡改自检 4/4:回执
  sha 翻位、表内 kill 判定翻转、报告单方面软化 kill 判定(7c 双重
  抓)、复原 PASS。
- **送审件:`outputs/splatc_atlas_round11_tree_e934bb006541.zip`
  sha256 `ae0267d44145257848f0043840349393ef75f4949e3267c9e4a777a9ac7ff734`**
  (外置 .sha256;= round4_final_package.zip 同字节)。交付物 + 回执
  已同步镜像并远端 sha 校验 OK。
