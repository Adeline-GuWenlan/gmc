# P3.8 Chart-Factored Connector Benchmark(round-11 送审)

**一句话结论:预注册的生死实验按规则触发了 kill——在 chart-factored
设定下,RRT-Connect 以 10/10 成功率进入 continuation 的 2× 以内
(pair-ops 0.73×、墙钟 0.91×;pose queries 2.87×),因此
single-query thin-gate 查询效率不再作为论文主 residual;主线按
round-7/8/10 预先接受的协议转向 compile-once / query-many、morphology
update、topology reuse、local recompilation(P4b 两门硬合同即其载体)。
负控制与零假可达红线全部成立。**

本报告与 `tables/p38_connector.json` 一一对应;所有判定为机器产出,
validator(7c)强制正文逐字引用。canonical 数字由锁定 Linux 环境的
全链再生(receipt 三段门),本地候选与 canonical 若有平台差异以
canonical 为准。

---

## 1. 任务与授予物(rounds 7/8/10 的 P3.8 指令)

每个 arm 得到完全相同的输入:

- **F_L / F_R**:该案例 converged(150k 预算、max_level=3、goal-free)
  chart 构建的两个最大 OpenChart 的**序列化认证 region**(certified
  cell cover + slack + 邻接);
- 对 scene 的**计费查询访问**(pose queries 经 BilledChecker 计数;
  pair_ops / bp_hits 由 scene 自身计数器累计;墙钟由 harness 计时);
- 一个固定随机种子(采样类 arm)与统一预算 **262,144 pose queries**。

**不提供**:门轴、合法 θ 窗、oracle centerline;任何 arm 不读
`scene.meta`(测试 `tests/test_p38_arms.py` 用守卫 dict 机械强制——
读即 fail)。

**成功判据**(对所有 arm 相同):输出一条 waypoint 折线,首点
`chart_contains` 于 F_L、末点于 F_R,整条通过
`certify_path_conservative`(保守 bubble 证书)。harness 另在**全新
场景上独立复检**每条成功路径(端点归属 + 全程再认证,不计费)——
任何未通过复检的"成功"计为 false claim,红线要求全局为 0。

共享后处理:所有采样类 arm 在认证前可用同一个 retraction polish
(对内部 waypoint 做计费的 margin 爬升)——continuation 内部本就带
polish,不给基线等价物才是不公平。

## 2. Arms

| arm | 类别 | 说明 |
|---|---|---|
| contact_continuation | 我方 | v4.1 kernel witness → `build_gate_chain` 全新认证 attachment–branch–attachment 链 |
| scalar_continuation | 我方消融 | 标量 clearance 连续化(声明消融)+ 同款 attachment |
| generic_connector | 我方 | P4a.2 对称认证直线连接件(证明来源) |
| rrt_connect | 基线 | 双树 RRT-Connect,根 = 两 region 最大-slack cell 中心,[0,π) 商采样(π 周期性 replay 断言),嵌入 KDTree 近邻 |
| lazy_prm_bridge | 基线 | lazy PRM + bridge-test 窄通道采样,懒边验证,逐边认证 |
| hrm_se2 | 基线 | HRM 风格 θ-层扫描分解(SE(2) NavMesh 族代表),分辨率倍增阶梯,确定性 |

**披露的不对称(利我方)**:contact arm 的 kernel 出口/跨墙判据是
G1-frame 条件化的(|x| 阈值——v4.1 已记录限制,P4b 合同项),即它
携带基线没有的门框先验。因此下表比值是对一个**占优 continuation**
测得的;基线仍进入 2× ⇒ kill 结论 a fortiori 成立。

采样类 arm 跑 10 个固定种子(plug 案例 3 个——拒绝由认证强制,与
种子无关);确定性 arm 跑一次。同种子重放 bit-一致(测试强制)。

## 3. 结果(canonical:`tables/p38_connector.json`)

### 3.1 thin_0.505(kill 案例;de-aligned dy=0.013, tilt=7°)

共享编译输入:73,368 queries 构出两 chart(6,762/7,064 cells),
不计入任何 arm。墙钟均为 canonical Linux 主机(锁定环境)测得。

| arm | 成功率 | median nq | median pair_ops | median 墙钟 | 认证余量 |
|---|---|---|---|---|---|
| contact_continuation | 1/1 | **16,462** | 1,865,643 | 70.6s | 2.34mm |
| scalar_continuation | 0/1(NO_SEED) | 7,280 | — | — | — |
| generic_connector | 0/1(全拒,266 checks)| — | — | — | — |
| rrt_connect | **10/10** | 47,204.5 | 1,367,644.5 | 64.45s | 0.37–1.91mm |
| lazy_prm_bridge | 0/10(预算耗尽)| 262,144 | ~2.82M | ~419s | — |
| hrm_se2 | 0/1 | 169,680 | — | 8.5s | — |

比值(基线/continuation,median,成功种子):

- P3.8 rrt_connect: WITHIN 2x —— nq 2.87×、**pair_ops 0.73×**、
  **墙钟 0.91×**(后两项基线反而更便宜;预注册规则为任一货币 ≤2×
  即触发,对基线从宽;即便按"三货币齐进"读法本案仍触发)。
- lazy_prm_bridge、hrm_se2:未在预算内产出认证连接件,不触发。

**P3.8 kill condition: HIT**

诚实注记:(i) continuation 在 pose queries 上仍保有 2.87× 优势——
其单次查询(双侧接触聚类)比基线的 check_pose 更贵,这正是三货币
报告存在的原因;(ii) RRT 的认证余量 0.37–1.91mm 系在共享 polish 后
才可认证(polish 前的原始路径普遍擦墙),而 continuation 的 witness
天然居中(2.34mm);(iii) chart-factored 设定天然利于双向采样器:
两根均为已认证的墙两侧 chart 内位姿。这些都不改变 kill 判定,只
界定它的含义:被杀死的是"单查询过门效率"这一主张,不是 kernel 的
Gate B 机制(round-10 裁决:Gate B PASS 不动)。

### 3.2 mid_0.62(generic-connector 制式对照)

| arm | 成功率 | median nq | 认证余量 |
|---|---|---|---|
| generic_connector | 1/1 | **157** | 5.81mm |
| contact_continuation | 0/1(双侧接触在更宽走廊丢失)| 10,991 | — |
| rrt_connect | 10/10 | 26,259.5 | 9.25–35.76mm |
| lazy_prm_bridge | 10/10 | 8,874 | 12.17–50.06mm |
| hrm_se2 | 1/1(r1 分辨率)| 170,128 | 56.97mm |

对照结论:任务在该宽度对所有方法可解(harness 公平性证据);我方
机制互补性按 round-10 设计成立——generic 证明源以 157 queries 覆盖
0.62,contact 源覆盖 0.505(0.62 的 contact 失败如实入表)。

### 3.3 plug_0.505(负控制)

corridor 远端封堵(G5 制式)。**全部 arm 拒绝**:contact
UNKNOWN_TRACK_LOST(11,883q)、scalar NO_SEED、generic 全拒(72
checks)、rrt/prm 3 种子全预算耗尽无认证输出、hrm 无跨墙 interval。
独立复检下 false claims = 0。

**P3.8 negative control (plugged, all arms refuse): PASS**

## 4. 判定后的主张变更(预注册协议的执行)

据 round-7/8/10 预先接受的 kill 协议,自本轮起:

1. **撤下**:"thin-gate 单查询效率是论文主 residual"——不再出现于
   任何对外材料(P3.7 的 8–11× 分离结论**不受影响**,它严格限于
   symmetry/ROI-matched conservative volumetric cell-tree 实现族,
   而 P3.8 表明非体积采样基线不在该分离之内)。
2. **主线转向**(P4b 两门硬合同为载体):
   - compile-once / query-many:atlas 一次编译,任意 goal 对纯序列化
     查询零新增接触编译(round-10 已建立 pose-independence 回归);
   - 认证 gate 表征:attachment–branch–attachment 对象携带可复用的
     bilateral signature 与认证余量(2.34mm vs 采样基线 polish 后
     0.37–1.91mm),这是"一条裸路径"不具备的复用资产;
   - morphology update / topology reuse / local recompilation。
3. Gate B(机制)与 P4a.2/P4a.3(chart 层语义一致性)结论不变。

## 5. 边界与已知限制

- contact arm 的 G1-frame 条件化(§2)——利我方先验,已披露;其
  goal-free 化是 P4b 合同项。
- 基线为按文献常规默认参数的忠实实现,未做对抗性调参;更强的基线
  只会强化(不会推翻)HIT 判定。
- HRM 的分辨率阶梯在 262k 预算内到 r1;其失败是预算判定,非能力
  上限声明。
- 采样类结果依赖平台浮点;canonical 数字以锁定 Linux 环境全链再生
  为准(receipt 三段门 + 逐位 compare)。

## 6. 下一步

P4b 两门多目标硬合同(compile 无 start/goal、atlas hash 不变、
goal→route 选择、全程认证)——即 §4 新主线的第一个可判定实验。
两门 demo 在 P4b 合同完成前禁做(round-10 指令维持)。
