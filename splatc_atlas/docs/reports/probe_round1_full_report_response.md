## 总判断

**做得对，但要准确定位：这是一次成功的“失败机制探针”，不是一次证明主方法有效的正实验。**

我会给它这样的状态：

| 项目                                | 判断                 |
| --------------------------------- | ------------------ |
| 实验记录与可复现性                         | **通过**             |
| 三态认证树的基本方向                        | **通过，但证明需补强**      |
| Uniform / Generic / Contact 的诊断价值 | **通过**             |
| 当前 contact scoring 是否成功           | **失败，而且失败得很有价值**   |
| 是否证明了 Contact Atlas 优于 baseline   | **尚未证明**           |
| 是否已经通过 Gate B                     | **没有**             |
| 是否值得继续下一轮                         | **值得，而且下一步已经相当明确** |

你的实验与原计划中“先建立可靠认证、逐层找出偏差来源、方法不成立就停止调参并记录失败”的纪律是吻合的。
报告也没有掩盖失败：你完整记录了四版 scoring、wall-clock 不公平、完整 Pareto 尚未重跑、邻接只能造成 false-unreachable，以及停止继续在 dev 上手调。这一点非常正确。

---

# 一、你真正做对了什么

## 1. 你没有用“中心点自由”冒充“整个 cell 自由”

这一点非常重要。

你现在的三态逻辑是：

[
m_{\mathrm{free}}>r_{\mathrm{cell}}
\Rightarrow
\text{整个 cell 都是 FREE},
]

[
m_{\mathrm{pen}}>r_{\mathrm{cell}}
\Rightarrow
\text{整个 cell 都是 COLL},
]

否则才是：

[
\text{AMBIG}.
]

这比普通 octree 中“检查中心点，然后假设整个 cell 一样”可靠得多。你还用 102,120 个内部随机 pose 做了 sanity check，未发现 certificate violation。

从实验工程角度看，这一层是合理的。

不过要区分：

* 随机采样零违例是很强的测试；
* 它不是数学证明；
* 最终论文仍需要把 (m_{\mathrm{free}})、(m_{\mathrm{pen}}) 和 (r_{\mathrm{cell}}) 的可靠性正式写清楚。

对当前单椭圆机器人，你使用：

[
r_{\mathrm{cell}}
=================

\sqrt{(s_x/2)^2+(s_y/2)^2}
+
a_{\max}(s_\theta/2)
]

作为 pose cell 内机器人最大位移上界，思路是正确的。以后换成多 primitive robot 时，(a_{\max}) 必须是：

> 从机器人旋转中心到所有 hard-support 点的最大距离，

而不能只取某个 primitive 的长半轴。

---

## 2. 你设置了一个真正会暴露离散问题的场景

你没有用完全对齐的门，而是加入：

* `door_offset = 0.013 m`；
* `door_tilt = 7°`；
* 接近临界的门宽；
* 多档 query budget；
* 连续姿态空间的 adaptive cells。

这能减少“刚好落在网格中心”“刚好与角度 channel 对齐”造成的虚假成功。

这是正确的 probe 设计。

---

## 3. 你没有只说“失败了”，而是定位了预算消耗在哪里

报告已经区分了三个不同机制：

[
\text{Uniform}
\rightarrow
\text{在整个 }SE(2)\text{ 体积上浪费},
]

[
\text{Generic }|\rho|
\rightarrow
\text{在全部碰撞边界表面积上浪费},
]

[
\text{Bilateral + scalar balance}
\rightarrow
\text{能够找到门轴，但在整个 }\theta\text{ 环上浪费}.
]

这是这轮实验最有价值的结果。

尤其是 v3 中已经在走廊内认证出 1,468 个 FREE cells，说明 contact 信息确实曾把细化预算送到了正确的空间位置；但 v4 加入房间骨架后，预算又被“开阔空间 + 门轴全角度区域”分走，最终走廊链消失。

所以当前数据支持一个较窄但很重要的结论：

[
\boxed{
\text{空间上的 gate localization 已有信号，}
\quad
\text{姿态上的 angular selectivity 尚未解决。}
}
]

这个结论是成立的。

---

## 4. 你及时停止了对单一 dev 场景继续手调

你明确记录了四轮 scoring 变化，并承认继续调整系数会构成对单场景过拟合，然后停止触碰 blind。

这是正确的研究纪律。

很多项目会在这里继续修改：

```text
0.15l → 0.13l
0.5b → 0.43b
threshold 0.2 → 0.17
```

直到一张门图成功，但这种结果无法支撑论文。你停下来的决定是对的。

---

# 二、当前报告中最需要纠正的地方

## 1. 现在的 `balance` 还不是真正的 Morse balanced-contact 条件

你当前定义：

[
\text{balance}
==============

|h_+-h_-|.
]

它只回答：

> 机器人到两侧障碍的标量距离是否相近？

这不是完整的 balanced-contact condition。

真正的 min-type / Clarke 临界条件应该类似：

[
\boxed{
0\in
\operatorname{conv}
\left{
\nabla_qh_k(q):k\in A(q)
\right}.
}
]

或者定义一个 residual：

[
\beta(q)
========

\min_{\substack{\lambda_k\ge0\\sum_k\lambda_k=1}}
\left|
\sum_k\lambda_k\nabla_qh_k(q)
\right|_{G^{-1}}.
]

当：

[
\beta(q)\approx0
]

时，才表示多个 contact gradient 在完整的：

[
q=(x,y,\theta)
]

空间中近似平衡。

这一区别恰好解释了你现在的失败。

在门的中轴上，对称两侧通常满足：

[
h_+\approx h_-
]

对几乎所有 (\theta) 都成立，所以：

[
|h_+-h_-|\approx0
]

没有任何角度选择性。

但两侧 contact gradient 可以写成类似：

[
\nabla h_+
==========

(\cdots,+n,\partial_\theta h),
]

[
\nabla h_-
==========

(\cdots,-n,\partial_\theta h).
]

两者的平移法向分量可以抵消，但角度分量一般不会抵消。只有在机器人横向投影对角度达到极值的位置，例如正确侧身方向附近，才可能同时满足完整的 gradient balance。

因此，这轮实验并没有证明：

> Morse/contact criticality 没有角度选择性。

它证明的是：

[
\boxed{
\text{“两侧等距离”这个标量启发式，不等于真正的 contact criticality。}
}
]

建议报告里把：

> “Morse balanced-contact 线索”

改成：

> “scalar bilateral equal-clearance heuristic”。

否则容易让审稿人认为我们错误实现了 Morse 条件后，再据此评价 Morse 方法。

---

## 2. `group_tags = ±1` 是一个带有特权信息的 probe

你现在把障碍 discs 预先标成：

* `+1` 上段；
* `-1` 下段。

然后分别计算：

[
h_+,\qquad h_-.
]

作为机制探针，这没有问题，因为你是在问：

> 假如我知道哪两组障碍构成门的两侧，bilateral signal 是否足够？

答案已经得到：不够。

但是在最终方法里，这些 tags 不能直接存在。因为原始输入合同只有 Gaussian / ellipsoid geometry，没有：

```text
这个 primitive 属于门上侧；
那个 primitive 属于门下侧；
这两组应该组成一对 gate。
```

需要自动从 geometry 中得到，例如：

* active-pair contact normals clustering；
* obstacle connected-component identity；
* 局部空间聚类；
* 相反法向匹配；
* contact graph 中的 opposing constraints；
* full-gradient balance。

还有一个重要区别：

> pair identity 可以是一次几何评估的副产品；

但：

> 把数百个 pair 自动组织成“两个相对的门侧”，并不是零成本副产品。

所以你报告中的：

> “逐侧分解视为同一次配对评估的副产品，计 0”

只能作为理想化的 pose-query 计费模型，不能直接用于最终计算效率 claim。

---

## 3. (w=0.58) 时的“约 12 倍角度浪费”算错了一倍

你给出的：

[
a=0.6,\qquad b=0.25,\qquad w=0.58.
]

允许通过的角度满足：

[
\sin^2\theta
<
\frac{(w/2)^2-b^2}{a^2-b^2}.
]

代入得到：

[
|\theta|<15.63^\circ.
]

但这个单椭圆机器人具有：

[
\theta\equiv\theta+\pi
]

的几何对称性。

所以在 (0^\circ) 到 (360^\circ) 内有两个合法区间：

[
[-15.63^\circ,15.63^\circ]
]

和：

[
[164.37^\circ,195.63^\circ].
]

总合法角度宽度约为：

[
62.53^\circ.
]

占整个角度圆环：

[
\frac{62.53}{360}
\approx17.37%.
]

对应的均匀浪费因子约为：

[
\boxed{5.76\times}
]

而不是 (12\times)。

你的定性结论仍然成立，但数字要修正。

有趣的是，在其他门宽下，因子分别大致是：

|   (w) |          单侧半角 | 总合法角度占比 |       角度稀疏因子 |
| ----: | ------------: | ------: | -----------: |
| 0.505 |  (3.73^\circ) |   4.14% | (24.2\times) |
| 0.510 |  (5.29^\circ) |   5.87% | (17.0\times) |
| 0.520 |  (7.52^\circ) |   8.36% | (12.0\times) |
| 0.540 | (10.78^\circ) |  11.97% | (8.35\times) |
| 0.580 | (15.63^\circ) |  17.37% | (5.76\times) |

所以“约 12 倍”实际上更接近 (w=0.52)，不是 (w=0.58)。

---

## 4. “(\theta) 与 (\theta+\pi) 形成镜像分量是正常现象”这句话要谨慎

正确的是：

> 椭圆在 (\theta) 和 (\theta+\pi) 下有相同的碰撞几何。

但不一定正确的是：

> 它们应该成为两个不同的 connected components。

如果房间足够宽，机器人可以在房间内连续旋转：

[
\theta
\rightarrow
\theta+\pi.
]

那么这两个姿态副本应该通过中间角度连在同一个自由空间分量中。

只有两种情况下，分开才合理：

1. 你明确把姿态空间取成了商空间 (S^1/\pi)，并以某种方式复制显示；
2. 房间真的窄到机器人无法完成连续半圈旋转。

你的 workspace 看起来足够大，所以建议马上做一个最小检查：

```text
固定左房间内部一个远离墙壁的 (x,y)
连续扫描 theta ∈ [0,2π)
确认整个角度环是否 FREE
确认 union-find 是否把角度环连成一个分量
```

如果整个角度环都 free，但 union-find 仍产生两个镜像分量，那么问题在：

* 跨层级邻接；
* (\theta) 周期处理；
* 未认证的角度 cell；
* 或 refinement coverage。

因此，报告里的这句话目前不能直接保留为“正常现象”。

---

## 5. 面中心邻接可能是一个真实混杂因素

你当前跨层级邻接只使用：

> 6 个面中心探针点。

你也正确承认了，这可能漏掉阶梯状的 fine/coarse 邻接，因此只会增加 false-unreachable。

但因为当前实验的失败结果本身就是 `UNKNOWN / false-unreachable`，这个偏差方向不能被忽略。

特别是 v3 中：

* 走廊内已经有 1,468 个 FREE cells；
* 但出现 14 个分量；
* start 分量只有 8 个 cells。

这里不能只归因于“房间没有认证成连续骨架”。跨层级邻接漏连也可能贡献很大。

正式 Pareto 扫描前应改为：

* 基于 cell face AABB overlap 的精确邻接；
* 或对每张 face 枚举所有相交叶 cell；
* 或至少使用多个 face samples，而不只是中心一个点。

更好的做法是：一旦 union-find 判断 reachable，显式恢复一条 cell chain，并通过：

[
\text{cell overlap witness}
\rightarrow
\text{continuous pose path}
\rightarrow
\text{exact hard collision validation}
]

生成真正的可达 witness。

这样“不会产生 false positive”就不只是一句程序逻辑判断，而有一条可验证路径作为证据。

---

## 6. 强制细化 start cell 会让当前 probe 带有 start dependency

你的报告说：

> 起点 cell 有强制种子细分，全策略一致。

这对“从这个 start 出发是否能在预算内找到通路”的在线查询实验是合理的。

但它不再是完全的：

[
\text{scene–robot pair}
\rightarrow
\text{start/goal-independent representation}.
]

需要把两种成本分开：

### Offline compile

[
(\mathcal G_E,\mathcal G_R)
\rightarrow
\mathfrak R_{E,R}
]

不允许使用 start 或 goal。

### Online query refinement

[
(\mathfrak R_{E,R},q_s,g)
\rightarrow
\text{query-specific refinement}.
]

可以从 start/goal 附近开始细化，但必须单独计费。

当前实验最好命名为：

> **start-seeded certified reachability probe**

而不是直接命名为完整的：

> **goal-independent atlas compiler evaluation**。

这不是说实验无效，而是 claim 必须与实际接口一致。

---

## 7. 当前计费只能证明 sample efficiency，不能证明计算效率

报告中同一 (w=0.58,64k) 下：

* uniform：9 秒；
* generic：12 秒；
* contact：70 秒。

你解释当前 `side_rho` 是重复子场景计算，未来可以融合。

这可以接受，但最终需要同时报告三种预算：

### Pose-query budget

[
\text{评估了多少个 }q=(x,y,\theta)?
]

这衡量 contact information 的采样效率。

### Primitive-pair operation budget

[
\text{实际评估了多少 scene–robot primitive pairs?}
]

这衡量 richer contact information 是否真的免费。

### Wall-clock / hardware budget

包括：

* KD-tree 查询；
* active-pair aggregation；
* gradient；
* side clustering；
* heap 和 tree 操作。

所以当前公平性可以表述为：

> 在理想化的一次 pose evaluation 返回完整 pair information 的模型下，三者使用相同 pose-query budget。

不能暂时表述为：

> 三种方法计算成本相同。

---

## 8. 当前 ground truth 还需要独立确认

你使用：

[
w>2b=0.5
]

判断所有门宽解析可达。

对理想的平行无限走廊，这个判断成立。

但当前真实测试场景还包含：

* 1.2 m 的有限厚度；
* 7° tilt；
* 0.013 m offset；
* 离散 disc-union 墙；
* 有限 workspace；
* 起始姿态和门前旋转过程。

所以“理想横截面可通过”不完全等于“完整 start-to-goal pose path 可达”。

正式 ground truth 应至少有一个独立来源：

1. 收敛的 dense (SE(2)) Oracle；
2. 显式构造的连续安全路径；
3. 独立的高精度 planner 加 continuous collision certification；
4. 解析中心线加门前旋转区域的分段证明。

尤其是 (w=0.505) 时，理论半宽裕量只有约 2.5 mm，必须排除 disc approximation 或数值 tie-break 改变真实临界宽度。

---

## 9. 当前只测了“全部可达”的实例

你的门宽集合全部满足：

[
w>0.5.
]

这适合研究 false-unreachable，但还不足以验证完整决策语义。

下一轮至少加入：

[
w\in
{0.49,0.495,0.5,0.505,0.51,\ldots}.
]

算法输出最好明确分成：

```text
CERTIFIED_REACHABLE
CERTIFIED_UNREACHABLE
UNKNOWN_AT_BUDGET
```

当前方法依靠 certified FREE chain 可以可靠证明 REACHABLE。

但“预算内没有找到 chain”只能是：

[
\text{UNKNOWN}
]

不能自动变成：

[
\text{UNREACHABLE}.
]

要认证 UNREACHABLE，需要：

* 完成足够范围的空间覆盖；
* 或构造 certified collision cut；
* 或证明 start 和 goal 所在 component 被碰撞边界隔开。

---

# 三、你对失败机制的核心判断是否正确？

## 基本正确

你说：

> pair identity 和 bilateral/balance 找到了“门在哪里”，但没有找到“哪个角度可以通过”。

这个结论与当前数据一致。

但要修正成更严格的说法：

[
\boxed{
\begin{aligned}
&\text{primitive-group bilateral proximity}\
&+\text{scalar equal-clearance}\
&\text{可以提升空间 gate localization，}\
&\text{但无法区分完整 }SE(2)\text{ 中的可行姿态窗口。}
\end{aligned}
}
]

原因不是 contact geometry 没用，而是目前 scoring 丢掉了：

[
\nabla_q h_{ij}
]

尤其是：

[
\partial_\theta h_{ij}.
]

---

# 四、下一步不应该直接做“普通 1D 角度扫描”

你的方向是对的：

> 检测门轴后，进一步识别可通行角度区间。

但不能简单变成：

```text
在门轴上均匀扫描 360 个 theta；
找到 free channels；
只在那里细化。
```

否则审稿人会说：

> 你只是把全局 fixed yaw channels 换成了门口的局部 fixed yaw channels。

更合适的下一步分成两层。

## 第一级：完整 contact-gradient criticality

对候选 active contacts 计算：

[
\beta(q)
========

\min_{\lambda\in\Delta}
\left|
\sum_k\lambda_k\nabla_q h_k(q)
\right|_{G^{-1}}.
]

用：

* bilateral proximity；
* full-gradient balance residual；
* hard FREE/COLL certificate；

共同找到候选姿态中心。

这会将“门轴 × 全部 (\theta)”缩减为少量 angular critical branches。

---

## 第二级：认证的 angular interval isolation

围绕候选角度，不做均匀扫描，而是在：

[
\theta\in[\theta_0,\theta_1]
]

上做 interval subdivision。

对每个角度区间认证：

```text
整个 interval 都 FREE
整个 interval 都 COLL
否则 AMBIG，继续二分
```

最终得到：

[
\Theta_{\mathrm{free}}
======================

\bigcup_r
[\theta_r^-,\theta_r^+].
]

复杂度应主要取决于：

* free/collision 边界数量；
* 所需角度精度；
* 临界点附近的间隙；

而不是固定的 (N_\theta)。

---

## 第三级：不要只检测一个门中心点

因为门有：

* 有限厚度；
* tilt；
* offset；
* 以后还会有 L/S corridor；
* 机器人可能需要一边横移一边旋转。

真正要构造的不是单点：

[
\Theta(q_{\mathrm{door\ axis}})
]

而是局部 gate tube：

[
\boxed{
\Gamma
======

\left{
(s,n,\theta):
h_{ij}(s,n,\theta)>0
\right}
}
]

其中：

* (s)：沿通道方向；
* (n)：横向位置；
* (\theta)：姿态。

也就是追踪：

> 合法角度区间怎样随机器人穿过门的纵向位置而变化。

单点角度窗口只能解决当前对称门 probe；gate tube 才能进入通用方法。

---

# 五、下一轮应怎样做，才算真正通过

## A. 先修掉三个混杂因素

在比较 scoring 之前先完成：

1. 精确跨层级 face adjacency；
2. 自动 contact-side / opposing-normal clustering，去掉人工 `±1 tags`；
3. fused contact evaluation，并同时记录 pose queries、pair operations 和 wall-clock。

否则下一轮成功后，审稿人仍然可以说优势来自：

* 人工门标签；
* 邻接实现差异；
* 不公平成本模型。

---

## B. 冻结四个策略

建议下一轮同时比较：

### Uniform

纯 breadth-first。

### Generic scalar

只允许使用：

[
\rho,\quad m_{\mathrm{free}},\quad m_{\mathrm{pen}},\quad r_{\mathrm{cell}}.
]

但应该给 generic 自己完整的 validation tuning，而不是只保留一个未经优化的：

[
|\rho|+0.25l.
]

最好报告多个 generic policies 的 Pareto envelope。

### Scalar bilateral

保留你当前的：

[
|h_+-h_-|
]

作为 ablation。

### Full contact geometry

使用：

[
\text{active pair identity}
+
\nabla_qh_{ij}
+
\beta(q)
+
\text{certified angular intervals}.
]

这样可以清楚说明究竟是哪一级信息带来改进。

---

## C. 运行完整二维 Pareto

不是只看：

[
w=0.58,\quad B=64k.
]

而是完整运行：

[
w
\times
\text{budget}
\times
\text{random/de-alignment seed}.
]

主要图应为：

[
\text{certified reachability rate}
\quad\text{vs.}\quad
\text{pose-query budget}.
]

再补：

[
\text{gate angular-width error}
\quad\text{vs.}\quad
\text{budget},
]

[
\text{primitive-pair operations}
\quad\text{vs.}\quad
\text{success},
]

[
\text{wall-clock}
\quad\text{vs.}\quad
\text{success}.
]

---

## D. 必须恢复一条路径 witness

一旦 union-find 判断 REACHABLE，就输出：

[
q_0,q_1,\ldots,q_K
]

并完成：

* cell-chain reconstruction；
* cross-face witness；
* local interpolation；
* continuous hard collision check；
* minimum clearance。

否则实验只验证了一个离散连接标签，没有验证最终 planning contract。

---

# 六、我建议你怎样修改本轮报告的结论

现在的结论可以改成下面这样：

> **Round 1 falsifies a scalar contact-priority hypothesis.** Certified adaptive refinement based on primitive-side proximity and equal-clearance successfully concentrates samples near the spatial doorway, but it remains non-selective over orientation because (h_+\approx h_-) holds along the door axis for a broad range of (\theta). Consequently, refinement is diluted over the angular fiber and fails to form a certified start-to-goal chain under the tested budgets. This result motivates using full configuration-space contact gradients and certified local angular-interval continuation rather than further tuning scalar priority coefficients. No residual advantage over generic adaptive representations has yet been established.

中文：

> 第一轮否证了“只靠双侧接近度和等距离标量就足以发现姿态门”的假设。该信号能够把细化预算集中到门的空间位置，但由于它忽略完整位姿空间中的接触梯度，特别是角度导数，所以无法在 (\theta) 维度选择真正可通行的姿态窗口。下一轮应转向完整 contact-gradient balance 和认证的局部角度区间追踪，而不是继续调整标量 scoring 系数。本轮尚未证明方法相对 generic adaptive baseline 存在 residual advantage。

---

# 最终评价

[
\boxed{
\textbf{作为第一轮探针，你做对了。}
}
]

而且它不是“什么都没跑出来”的失败。它已经排除了一个看起来合理、但实际上不够的候选：

[
\text{bilateral proximity}
+
|h_+-h_-|.
]

这节省了继续调参的时间，也指出了真正缺失的数学信息：

[
\boxed{
\text{full }SE(2)\text{ contact gradients}
+
\text{certified angular gate intervals}.
}
]

但目前不能得出：

* 我们优于 generic adaptive；
* contact policy 已保留 thin gate；
* Gate B 已通过；
* 12 倍角度优势已经被证明；
* 人工 side decomposition 是零成本；
* 镜像 connected components 是正常拓扑。

最准确的阶段状态是：

[
\boxed{
\text{诊断实验通过；当前 scoring 假设失败；下一假设有充分依据；主方法 Gate 尚未通过。}
}
]


## 我的判断

**作为第一轮“失败探针实验”，你做得总体是对的，而且研究纪律很好。**
但目前能成立的结论比报告最后写的要窄一些：

[
\boxed{
\text{当前 v4 的 bilateral/balance 优先级，在 64k 中心查询预算下，没有构造出连通的认证自由通道。}
}
]

报告还相当有说服力地表明：

[
\boxed{
\text{pair identity + bilateral balance 能帮助定位门附近，}
\text{但不能充分区分真正可通过的姿态角窗口。}
}
]

这与报告中 v1-v4 的演化、走廊曾经出现大量认证 FREE cells、随后预算被房间骨架和整圈角度候选重新分配掉的现象是一致的。你也正确记录了手工调参历史、没有触碰 blind、承认 wall-clock 不公平、并明确说完整 Pareto 尚未重跑。这些做法是对的。

不过，当前实验**还不能证明**以下更强结论：

1. 失败纯粹是排序信号造成的；
2. gate-specific angular interval detector 是唯一或必然的解决方案；
3. 64k 查询一定足够构造完整连通链；
4. 所有 FREE/COLL cell certification 都已经数学可靠；
5. `REACHABLE` 在当前代码中绝不可能假阳；
6. Contact 方法已经在公平计算预算下优于 generic baseline。

所以我的状态判断是：

| 部分                         | 判断                         |
| -------------------------- | -------------------------- |
| 实验问题设置                     | **基本正确**                   |
| 负结果记录                      | **正确且可信**                  |
| one-sided certification 架构 | **方向正确，但关键数学界尚需审计**        |
| 三策略 pose-query 对照          | **适合作为信息消融，不足以代表最终计算效率**   |
| “门位置找到了、姿态窗没找到”            | **较强支持，但还不是严格证明**          |
| “纯排序瓶颈”                    | **目前只是合理假设**               |
| Gate B 主张                  | **尚未通过**                   |
| 下一步研究方向                    | **角度区间检测合理，但需要控制实验排除其他解释** |

---

# 一、你做得比较正确的地方

## 1. 你没有把失败包装成成功

报告明确写出：

* v1 无认证导致 rabbit hole；
* v2/v2′ 的排序问题；
* v3 走廊出现 FREE cells，但全局连通失败；
* v4 修复房间和 goal 后，预算又把走廊细化挤掉；
* 四版打分都只在 dev 场景上调；
* 继续手调会过拟合，因此停止。

这是非常正确的实验处理。它符合之前执行计划要求的“逐层定位偏差，不只看最终路径”，也没有偷偷使用 blind case 调参。 

## 2. FREE / COLL / AMBIG 三态结构是对的

你不是根据中心点标签直接宣布整个 cell 自由，而是使用：

[
m_{\mathrm{free}}(q_c)>r_{\mathrm{cell}}
]

认证整个 cell FREE，使用：

[
m_{\mathrm{pen}}(q_c)>r_{\mathrm{cell}}
]

认证整个 cell COLL，其余保持 AMBIG。

这个架构比“中心点 free 就把 cell 当 free”可靠得多。对于单个、以机器人坐标原点为中心的椭圆，

[
r_{\mathrm{cell}}
=================

\sqrt{(s_x/2)^2+(s_y/2)^2}
+
a,s_\theta/2
]

确实是一个合理的保守机器人点位移上界，因为旋转造成的最大位移满足：

[
2a\sin\frac{|\Delta\theta|}{2}
\le a|\Delta\theta|.
]

因此，只要中心处的米制 clearance lower bound 确实可靠，这个 cell certification 逻辑就是成立的。

## 3. 你把 uniform、generic 和 contact 放在相同中心查询预算下

这很适合回答一个特定问题：

> 在同样询问多少个 configuration center 的情况下，额外使用 contact identity 和 bilateral information 是否更容易找到 gate？

这是有效的信息消融实验。

你也没有把当前 contact 的 70 秒和 generic 的 12 秒隐藏掉，而是明确指出 `side_rho` 目前重复运行、wall-clock 还不公平。这一点处理正确。

## 4. De-alignment 是必要且正确的

`door_offset=0.013 m`、`door_tilt=7°` 能减少以下偶然因素：

* 门刚好落在 cell boundary；
* 机器人最优姿态刚好等于某个 yaw channel；
* 对称轴刚好和网格对齐。

这比只使用完美轴对齐门更可信。

## 5. 你发现了一个真实的表示问题

当前 contact score 使用：

[
\text{balance}=|h_+-h_-|
]

来找两侧接触平衡位置。门中轴上确实可能满足：

[
h_+\approx h_-
]

但这并没有自动区分：

* 横着卡在两侧；
* 斜着卡在两侧；
* 刚好能够侧身通过。

所以“balanced contacts 定位 neck，但不自动给出可行 angular fiber”是一个很有价值的发现。

更精确地说：

[
\boxed{
\text{当前 contact signal 找到的是候选 gate base location，}
\text{还没有恢复这个位置上可行的 } \theta \text{ fiber。}
}
]

这正好对应下一层需要研究的结构：

[
\Theta_{\mathrm{free}}(x,y)
===========================

{\theta:\rho(x,y,\theta)>0}.
]

---

# 二、报告里有一个明确的数值错误

你对 (w=0.58) 时角度窗口占比的计算不对。

理想平行门框中，椭圆半轴：

[
a=0.6,\qquad b=0.25
]

可通过条件是：

[
2\sqrt{a^2\sin^2\theta+b^2\cos^2\theta}<w.
]

所以：

[
\theta_{\max}
=============

\arcsin
\sqrt{
\frac{(w/2)^2-b^2}{a^2-b^2}
}.
]

代入 (w=0.58)：

[
\theta_{\max}\approx15.63^\circ.
]

在 ([0,2\pi)) 中，由于椭圆在 (\theta) 和 (\theta+\pi) 下几何相同，存在两个可行窗口，每个宽度为：

[
2\theta_{\max}\approx31.26^\circ.
]

总宽度为：

[
4\theta_{\max}\approx62.53^\circ.
]

因此占完整角度域的比例是：

[
\frac{62.53^\circ}{360^\circ}
\approx17.37%
\approx\frac1{5.76}.
]

不是报告中的约 (1/12)。

[
\boxed{
w=0.58\text{ 时，单纯由角度窗口比例产生的浪费约为 }5.8\times，
\text{而不是 }12\times。
}
]

(1/12) 更接近 (w=0.52) 时的比例。

这个错误不会推翻“需要角度选择性”的结论，但会削弱报告中“12 倍浪费压过 64k”的定量论证。实际浪费仍可能大于 5.8 倍，例如：

* 细分同时作用于 (x,y,\theta)；
* 大量碰撞姿态也被 balance score 优先；
* 门轴附近有多个相邻空间 cell；
* siblings 和 ancestors 也必须被评估。

但这些额外倍数必须从日志统计，而不能全部归因于 angular-window measure。

---

# 三、(\theta) 与 (\theta+\pi) 的处理需要重新检查

报告把成对 component 解释为：

> (\theta/\theta+\pi) 镜像成对，是对称体自由空间的正常现象。

从代码参数化角度看，它是预期现象；但从物理 configuration space 看，它实际上是**冗余双覆盖**。

对于没有“正面/背面”标签的纯椭圆机器人：

[
R(\theta)\mathcal B_R
=====================

R(\theta+\pi)\mathcal B_R.
]

物理 configuration 应当满足：

[
\theta\sim\theta+\pi.
]

因此更准确的状态空间是：

[
SE(2)/C_2,
]

或者至少在表示中显式识别：

[
(x,y,\theta)
\equiv
(x,y,\theta+\pi).
]

当前完整 (2\pi) 参数化会：

* 重复表示所有状态；
* 约增加两倍 angular budget；
* 产生重复 connected components；
* 使 component count 和 representation size 失真；
* 容易在角窗口占比计算中漏算第二个窗口。

这不破坏 uniform/generic/contact 三者之间的内部公平性，因为三者都受同一冗余影响。但在论文中比较 representation efficiency 时必须处理。

可接受的两种做法是：

1. 对这个椭圆 pilot 使用 (\theta\in[0,\pi))；
2. 保留 ([0,2\pi))，但在拓扑和邻接中加入 (C_2) symmetry identification，并把冗余成本单独报告。

---

# 四、最关键的未知项：PW 到米制 clearance 的界是否真的可靠

这是我目前**无法仅凭报告确认**的最大问题。

报告使用：

```python
g_free = max(g_free, h * (b + R))
g_pen  = max(g_pen, -h * (b + R))
```

并给出理由：

> PW 缩放至相切时，接触点沿中心射线移动；该距离在内切半径与外切半径之间，因此可乘 (b+R) 得到米制 lower bound。

这个论证是否成立，取决于 `pw_ellipse_disc` 返回的 (h) 的精确定义。

风险在于：

* PW contact function 通常描述 homothetic scaling；
* homothetic radial gap 不一定等于真正的最短欧氏 clearance；
* 对一般椭圆，最近接触点不一定与椭圆中心和圆心共线；
* “沿中心射线的距离”通常更容易成为欧氏最近距离的上界，而不是下界。

因此，以下结论目前只能条件成立：

[
m_{\mathrm{free}}>r_{\mathrm{cell}}
\Rightarrow
\text{整个 cell FREE}
]

以及：

[
m_{\mathrm{pen}}>r_{\mathrm{cell}}
\Rightarrow
\text{整个 cell COLL}.
]

你做的 102,120 个随机 pose、零违例是很有用的经验检查，但它还不能代替这个界的证明，尤其要确认验证 oracle 与 PW 实现完全独立。

### 我需要的额外信息

需要提供：

1. `pw_ellipse_disc` 的完整代码；
2. (h) 的精确定义，例如它到底是：
   [
   \lambda-1,\quad \sqrt{\lambda}-1,\quad F-1
   ]
   还是其他量；
3. PW 界的完整推导；
4. 102,120 个 pose 使用的“真实碰撞/距离 oracle”代码；
5. 该 oracle 是否调用了同一个 PW 函数；
6. 至少一组：
   [
   \frac{h(b+R)}{d_{\mathrm{exact}}}
   ]
   的 adversarial sweep，而不仅是随机 cell 内采样。

最有说服力的测试是：对随机 ellipse-disc pose 使用一个独立的精确最近距离求解器，检查：

[
0\le h(b+R)\le d_{\mathrm{exact}}
]

是否对所有 free pose 成立；碰撞状态同理检查 penetration lower bound。

---

# 五、“所有门宽都解析可达”还需要独立验证

条件：

[
w>2b
]

能说明椭圆可以放进一个**理想无限直条通道**。

但你的真实场景还包括：

* 墙厚 (T=1.2)；
* 门偏移；
* 门旋转 (7^\circ)；
* union-of-discs 墙体；
* 边缘圆半径 (0.10)；
* 填充圆半径 (0.15)；
* 有限 workspace boundary。

所以：

[
w>2b
]

是重要条件，但不自动证明真实 disc-union 场景中存在从 start 到 goal 的连续路径。

特别是 (w=0.505) 时，理想侧向总余量只有：

[
0.505-0.500=5\text{ mm},
]

每侧约 (2.5\text{ mm})。disc 离散、门角点和倾斜都可能改变真实有效宽度。

### 我需要的额外信息

需要提供：

* 场景的俯视图，显示所有 obstacle discs；
* door width 的精确定义，是 disc support 间距还是中心线间距；
* 每个 (w) 的 Dense Oracle 或独立 certified path；
* 每条 oracle path 的最小 hard clearance；
* 门轴上多个 (x) 位置的精确：
  [
  \Theta_{\mathrm{free}}(x)
  ]
  曲线。

在这些材料出现之前，应将：

> “全部解析可达”

改成：

> “对于理想平行门模型满足必要的几何可通过条件；真实 disc-union 实例的可达性由独立 Oracle 待确认或已经确认。”

---

# 六、`group_tags=+1/-1` 可能构成信息泄漏

这是方法泛化性上另一个必须说清楚的问题。

当前 contact signal 依赖：

```text
+1 = 上段墙
-1 = 下段墙
```

然后分别计算：

[
h_+,\qquad h_-.
]

在程序生成场景里，这些标签很容易提供。但真实 Gaussian 场景通常不会自带：

> “这是门的上边，那个是门的下边。”

所以这里有两种完全不同的情况。

## 情况 A：标签只是自动得到的 obstacle/contact cluster identity

例如由：

* primitive connectivity；
* 接触法向；
* 空间聚类；
* 对向 contact normal；
* active-pair graph；

自动推导出两个相对障碍组。

这种情况是合理的。

## 情况 B：标签由数据生成器直接告诉方法哪边是门框

那么 contact policy 获得了 benchmark-specific gate semantics，不能作为最终方法。

### 我需要的额外信息

需要提供：

* `side_rho` 完整代码；
* `meta.group_tags` 是怎样生成的；
* 方法在运行时是否读取了 `door_axis`、`door_center`、`upper/lower wall` 等 generator metadata；
* 随机打乱 primitive ID 后结果是否不变；
* 删除人工 side labels 后，是否能从 contact normals 自动恢复两个 opposing groups。

后续 angular interval detector 同样不能直接使用已知门轴。门轴必须由 contact geometry 推导，例如通过相对法向或局部 contact Jacobian 发现。

---

# 七、“瓶颈纯粹是排序信号”目前还没有被证明

你给出的论据是：

* level 6 足以认证通道；
* 理论链长约 115 cells；
* 总预算 64k；
* 所以预算够，问题只是队列没有选中正确 cells。

这个方向很合理，但尚不充分。

原因是 115 个最终 leaf cells 并不等于 115 次查询。构造这些 leaf 需要：

* 评估所有祖先；
* 每次 split 生成 8 个 children；
* 评估大量 siblings；
* 同时建立 start 到门、门到 goal 的房间连接；
* 处理跨层邻接；
* 认证 goal region；
* 处理 (\theta) 周期和重复分支。

因此，“115 < 64k”还不能证明 64k 必然足够。

### 最直接的判定实验

增加一个只用于诊断、不能作为最终方法的：

> **Oracle-guided scheduler**

使用 Dense Oracle path 或解析 gate interval，只优先细分与真实可行 tube 相交的 cells。

记录它首次构造出完整认证连接所需的：

* center queries；
* primitive-pair evaluations；
* leaf count；
* max level；
* start-to-goal certified chain。

若 oracle-guided scheduler 在远低于 64k 时成功，才能比较有力地说明：

[
\boxed{
\text{表示容量足够，主要失败在优先级信号。}
}
]

如果 oracle-guided 也超过 64k，问题就不只是排序，还可能来自：

* 八叉式各向同性 split 太浪费；
* cell certification 太保守；
* room/gate 双任务预算冲突；
* adjacency 漏连接；
* geometry bound 太弱。

---

# 八、当前实验还不能证明 angular detector 是唯一答案

报告证明的是：

[
\boxed{
\text{当前单一全局 heap score 不足。}
}
]

它尚未区分三种可能修复：

### 1. 增加角度区间信号

你提出的方案：

[
(x,y)\text{ gate candidate}
\rightarrow
\Theta_{\mathrm{free}}(x,y)
]

是很合理的。

### 2. 改成多队列或预算配额

v3 擅长细门，v4 擅长房间骨架。这可能意味着需要：

```text
open-space skeleton queue
+
gate-detail queue
```

而不是把二者强行压成一个 scalar score。

例如固定一部分预算用于：

* 全局 coarse connectivity；

另一部分用于：

* bilateral gate refinement。

### 3. 改为各向异性 split

在候选门轴上，主要不确定性可能来自 (\theta)，但当前每次 split 同时把：

[
x,\ y,\ \theta
]

全部二分，生成 8 个 children。

如果只需要细化角度，却同时把空间也细化四倍，会产生大量不必要状态。更合适的可能是：

[
\text{split only in }\theta
]

或根据 interval uncertainty 选择 split 维度。

因此下一轮应当至少比较：

1. v4 单队列；
2. v4 + 双队列配额；
3. v4 + 仅 (\theta) anisotropic refinement；
4. v4 + certified angular interval detector；
5. angular interval + anisotropic refinement。

否则无法判断成功究竟来自“第三级 contact signal”，还是普通的调度和细分结构改进。

---

# 九、当前 contact score 可能偏向碰撞姿态

根据报告：

```python
b = max(h_plus, h_minus)
score = balance + 0.5 * b + 0.15 * level
```

并且：

[
\rho>0
]

表示 free/open。

那么当一个姿态同时与两侧碰撞时：

[
h_+<0,\qquad h_-<0,
]

可能有：

[
b<0.
]

因为 score 越小优先级越高，负的 (b) 会让“双侧碰撞但平衡”的姿态比“双侧接近且可行”的姿态更优先。

这可能不是单纯的“没有角度选择性”，而是：

[
\boxed{
\text{当前 score 显式奖励了 balanced collision states。}
}
]

这有时有助于尽快把碰撞 cell 认证为 COLL，但也可能造成你观察到的“门轴 × 整个角度圆环”rabbit hole。

### 最需要的一张诊断图

在门轴上固定若干 (x)，画：

[
\theta
\mapsto
h_+(\theta),
\quad
h_-(\theta),
\quad
\rho(\theta),
\quad
b(\theta),
\quad
\text{balance}(\theta),
\quad
\text{score}(\theta).
]

同时标出真实 FREE angular interval。

这张图会直接告诉我们：

* balance 是否真的全角度接近零；
* bilateral 是否具有姿态选择性；
* score 是否优先了深碰撞；
* 阈值 (b<0.2) 实际覆盖了多少角度；
* angular detector 应该检测 sign transition、free interval，还是某种 contact Jacobian rank。

---

# 十、计费合同目前只适合“采样效率”，不适合“总效率”

你将每个 cell center 计作一次查询，并把 side decomposition 当作 pair evaluation 的副产品。这可以作为一种理论 pose-query contract。

但正式实验必须同时报告：

[
\text{pose queries}
]

和：

[
\text{primitive-pair evaluations}.
]

因为当前实现中：

* generic：12 秒；
* contact：70 秒。

即使将来合并实现，返回所有 active pairs、按障碍 group 聚合、计算 bilateral/balance 仍然有额外成本。

推荐正式报告四种预算：

1. pose-center queries；
2. primitive-pair evaluations；
3. wall-clock；
4. peak memory。

这样可以同时回答：

* contact information 是否减少 configuration samples；
* contact information 本身是否值得它的计算成本。

当前报告已经如实说明 wall-clock 不公平，因此这里不是实验错误，而是**尚未完成最终效率验证**。

---

# 十一、数据集目前全是 reachable，不足以验证完整判定器

当前门宽集合：

[
{0.505,0.51,0.52,0.54,0.58}
]

全部被设为 reachable。

这适合研究 false-unreachable，但不能验证：

* 方法会不会错误打开真实关闭的门；
* tangency semantics；
* collision certification；
* gate detector 是否总倾向于宣布存在通道。

至少应加入：

[
w\in{0.49,0.495,0.5}
]

以及：

* 一个明显不可达宽度；
* 一个临界相切实例；
* 一个只有另一条绕路可达的实例；
* 一个门口可放置但无法完成连续转向的实例。

需要分别报告：

[
\text{false reachable}
]

和：

[
\text{false unreachable}.
]

---

# 十二、当前只能回答 reachability，还没有验证完整 planning

这一轮成功判据是：

> 在 checkpoint 正确回答 REACHABLE。

作为 representation probe，这是合理的。

但它还没有验证：

* union-find 是否能输出一条实际 cell chain；
* cell chain 能否转成连续 pose path；
* path 是否通过 hard continuous validation；
* 路径是否包含正确旋转序列；
* minimum clearance；
* 多目标复用。

所以这轮不能写成：

> “Contact Atlas planning 失败/成功。”

更准确的标题是：

> **Certified topology probing under finite refinement budget.**

---

# 我现在还需要哪些材料？

按照优先级，最少需要以下内容。

## A. 判断安全正确性所必需

1. `pw_ellipse_disc` 完整代码与数学定义；
2. 独立 hard collision / exact distance oracle；
3. 102,120 pose 验证脚本；
4. `cell_radius` 完整代码；
5. `eval_points_bounds` 完整函数，而不只是核心片段。

## B. 判断是否存在信息泄漏所必需

6. `side_rho` 完整代码；
7. `meta.group_tags` 的生成过程；
8. 场景 primitive 和 tag 可视化；
9. angular detector 计划如何从原始 geometry 中获得门轴。

## C. 判断失败机制所必需

10. 每个策略的完整 checkpoint CSV；
11. heap pop 的统计：
    [
    \text{level}\times\text{空间区域}\times\theta\text{ bin}\times\text{status};
    ]
12. 门轴上的 `rho/h+/h-/balance/score` 对 (\theta) 曲线；
13. oracle-guided scheduler 的最小成功预算；
14. v3、v4 的 certified free-cell 可视化和 component 可视化。

## D. 判断 ground truth 所必需

15. disc-union 场景图；
16. 每个门宽的 Dense Oracle reachability；
17. Oracle path 与 minimum clearance；
18. (\theta) 的实际定义域及 (\theta+\pi) 处理方式；
19. `_answer`、跨层 `locate` 和面邻接代码。

提供整个 `splatc_atlas` 仓库快照当然最完整；若只提供关键文件，上述 19 项已经足以完成一次比较严格的审计。

---

# 下一轮最小实验应该怎样设计

不要继续改 score 系数。冻结 v4，然后增加三个诊断版本：

[
\boxed{
\text{v4}
}
]

[
\boxed{
\text{v4 + dual-queue / fixed budget allocation}
}
]

[
\boxed{
\text{v4 + certified angular interval + anisotropic }\theta\text{ split}
}
]

同时加入一个不能作为方法、只用于证明预算可行性的：

[
\boxed{
\text{oracle-guided refinement upper bound}
}
]

在以下门宽上统一运行：

[
w={0.49,0.50,0.505,0.51,0.52,0.54,0.58}.
]

结果画成：

[
\text{reachability correctness}
\quad\text{vs.}\quad
\text{pose queries}
]

以及：

[
\text{gate interval recall}
\quad\text{vs.}\quad
\text{primitive-pair evaluations}.
]

下一轮真正有力的成功条件不是“某个 score 在 (w=0.58) 终于通了”，而是：

> 在冻结参数和多个 de-aligned validation instances 上，contact-derived angular interval refinement 比 uniform、generic adaptive、contact-v4 和普通双队列，在相同预算下更晚丢失真实 gate；同时对 (w\le0.5) 不产生 false reachable。

---

# 最终结论

[
\boxed{
\textbf{你的实验作为 round-1 negative probe 是合格的。}
}
]

它已经可信地说明：

> 当前 bilateral/balance contact score 虽然能把注意力带到门附近，但还不能有效恢复门上的可行姿态 fiber。

但目前还不应写成：

> 64k 完全足够，失败纯粹来自角度排序，加入 angular detector 后 Gate B 必然成立。

在进一步结论成立前，最需要修正或确认的是：

1. (w=0.58) 的角度窗口比例计算；
2. (\theta\sim\theta+\pi) 的机器人对称性；
3. PW 到米制 clearance 的可靠下界；
4. side tags 是否泄漏门结构；
5. oracle-guided scheduler 是否证明预算确实足够；
6. 单队列失败是否也能由双队列或各向异性 split 修复。

所以当前最准确的项目记录应当是：

> **Round 1 passed as a failure-mechanism probe; the current contact priority is falsified. The proposed angular-fiber detector remains a justified next hypothesis, not yet a validated method.**
