# 对评审的逐项答复(材料清单 A–D)

日期:2026-08-13。对应评审:`probe_round1_full_report_response.md`。
修正附录见主报告头部;本文件只回答"我需要哪些材料"部分。

## A. 安全正确性材料

**A1. `pw_ellipse_disc` 代码与 h 的精确定义。**
完整代码:`src/splatc/gaussian_geometry/contact.py`(48 轮二分,对 dS/dλ 符号)。
精确定义:`h = sqrt(F_PW) − 1`,其中
`F_PW = max_{λ∈(0,1)} λ(1−λ)[u₁²/D₁ + u₂²/D₂]`,`D₁=(1−λ)a²+λR²`,`D₂=(1−λ)b²+λR²`
(disc 情形的闭式对角化;一般椭圆-椭圆版 `pw_ellipse_ellipse` 同文件)。
即 μ=√F 是两 hard support 同时绕各自中心缩放至相切的公共因子,h=μ−1。

**A2/A5. 独立 oracle 与验证脚本。**
独立参照 = checker#2:`point_ellipse_distance`(标准 F(t) 二分求点-椭圆最近距离,
90 轮)+ `checker2_signed`,与 PW 无共享数学路径。`cell_radius` 与
`eval_points_bounds` 完整实现见 `probe_methods.py` / `primitives.py`(报告 §2 已引核心段,
文件为准)。

**A3. 102,120-pose 验证的诚实澄清(评审的怀疑部分成立)。**
该测试的 pose 标签用的是 `check_pose`,即 **checker#1(PW)自身**——它验证的是
"Lipschitz cell 逻辑相对 pose 标签"的一致性,**不构成界对精确距离的独立验证**。
完整证据链现在是三段:
① pose 标签符号正确性:tests #5/#14,4,900 组构型 PW vs checker#2 符号全一致;
② **界 ≤ 精确距离(评审要求的对抗验证,已补做)**:50 万随机构型
(u₁,u₂∈[−2,2], R∈[0.02,0.6]),FREE 侧 445,792 例 `h(b+R) ≤ d_exact` 零违例,
max ratio = 1.000000(短轴接触取等);穿透侧(圆心在外,MTD = R − dist)39,362 例
`−h(b+R) ≤ depth` 零违例,max ratio = 1.000000;
③ cell 级 Lipschitz 逻辑:原 102k 测试。
形式证明(含圆心在椭圆内的穿透分支)列入论文数学附录待办。

## B. 信息泄漏边界

**B6/B7. `side_rho` 与 `group_tags` 生成。**
代码:`primitives.py::side_rho / by_tag`;tags 在 `g1_gate.py` 生成器的 sign 循环中
按墙段赋 ±1(门楣塞子 tag=0)。**方法运行时读取的 generator metadata 仅有
`group_tags`;`door_offset/door_tilt/door 轴`等从未被策略读取**(gate_interval 的
oracle 侧测量除外,那是 reference 路径)。
**承认:tags 是特权探针信息**——结论已限定为"给定对向分组时,双侧标量信号仍不够"。
primitive ID 乱序不变性:未测,列入 round-2 前置(预期不变:tags 按值分组,与顺序无关,
但要实测)。自动分组方案:接触法向对向聚类(round-2 混杂修复第 2 项)。

**B9. 角度检测器如何不用门轴 metadata。**
新诊断图(`theta_diagnostic.png`)直接给出答案:可通行 θ 窗 = `min(h₊,h₋) > 0` 的
区间,其中 h± 来自自动聚类的对向接触组——检测器输入只需 contact 几何,无需任何
generator 语义。

## C. 失败机制材料

**C10/C11. checkpoint CSV 与 heap-pop 统计:部分缺失,如实报告。**
v1 机器的全量 checkpoint 在 `results/tables/probe_pareto.json`;v2–v4 的迭代是
inline 运行,只有终态 census(报告 §3 表),**没有留 heap-pop 日志——round-2 harness
把 (level × 区域 × θ-bin × status) pop 统计做成标配**。
**C12. 门轴 θ 诊断图:已产出**(`theta_diagnostic.png`,s∈{−0.45, 0, +0.45})。
三个结论:balance 全 θ ≈0(零角度选择性,实锤);score_v4 最低点在深碰撞 θ 区
(奖励平衡碰撞,评审 §九 实锤);可通行窗 = h₊、h₋ 同正区间(检测器信号现成)。
**C13. oracle-guided floor:round-2 第一项**(最便宜,决定其余实验的预算设置)。
**C14. v3/v4 可视化**:`probe_debug_slice.png`(v3 期 θ=7° 切片);round-2 每 arm 标配。

## D. Ground truth 材料

**D15/D16. 场景图与门宽定义。** disc 布局见 `rotate_door_path_medium.png`(灰色即
disc 并集);门宽定义:两侧 corridor 边缘行 disc 的 support 包络间距恰 = w
(圆心在 ±(w/2+0.1),r=0.1),包络波纹 ≤5e-5 m。
**D17. 独立可达真值(评审 §五 要求,已做)**:5 个门宽全部构造显式路径
(旋转→沿倾斜轴穿门→到 goal)并通过保守 swept 证书,min 余量
2.50/5.00/10.00/20.00/40.00 mm,与解析 (w−0.5)/2 逐个吻合。附带发现:见证仅需
28–284 次 margin 检查,信息下限 ≪ 64k(表示层溢价成为 round-2 核心测量)。
注:dense 网格 oracle 在 de-aligned 临界实例上 fine 档即失败(dealign_demo 已记录),
故这些实例的真值以构造性证书 + 解析等变性为准。
**D18. θ 定义域**:[0,2π) 全环;中心对称仅在 reference oracle 的切片镜像里利用,
probe tree 未利用 ⟹ ~2× 冗余,三策略同担(内部公平),表示效率对比时须处理;
round-2 采用评审方案 1(pilot 用 θ∈[0,π))或显式 C₂ 商并单独报告冗余成本。
**D19. `_answer` / `locate` / 面邻接代码**:`probe_methods.py`(报告 §2.4);
面中心探针的欠连接问题承认,round-2 换精确 face-overlap 邻接。

## 战略答复

Round-2 按评审设计执行(冻结 v4 为消融臂;修三个混杂;加双队列/各向异性 θ-split/
认证角度区间检测器;oracle-guided floor 先行;宽度加 {0.49,0.495,0.50};三态输出;
三层计费;REACHABLE 回溯 witness 闭环)。原"1D 均匀角度扫描"方案撤回
(会被正确地读成局部 fixed yaw channels);检测器信号采用 `min(h₊,h₋)>0` 区间的
认证式 interval subdivision,门轴与对向组由接触几何自动推导。
