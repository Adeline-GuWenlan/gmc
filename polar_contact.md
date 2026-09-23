# GMC 新学习主线：Deep-Unfolded Gaussian Polar Contact

> **版本**：Method Design v0.1  
> **目标仓库**：`https://github.com/Adeline-GuWenlan/gmc`  
> **适用范围**：完整三维 3DGS 场景、完整三维机器人 body；地面机器人可受限于 `SE(2)` manifold，UAV 可使用 `R^3`、`R^3 x S^1` 或完整 `SE(3)`。  
> **核心决策**：不学习 routing、certificate、gating、pair deletion、最终路径或碰撞标签。学习对象只来自 Gaussian pair 配置障碍的精确连续优化问题。

---

# 0. 执行摘要

当前严格解析方案从完整三维 Gaussian pairs 构造配置障碍，再通过 spherical support directions、inner/outer polytopes、三维 Boolean arrangement、free-volume decomposition 与 orientation product complex 得到全局 mobility。该路线具有明确的三值认证和全局拓扑优势，但在三维、yaw 以及完整 `SE(3)` 下会遭遇：

- 每个 pair 需要大量球面 support directions；
- 需要构造和更新大量 pair polytopes；
- 三维 obstacle union / workspace complement 昂贵；
- yaw interval 与未来 `SO(3)` cells 会导致 product-space 膨胀；
- 离散 inner/outer envelope 在 narrow passage 中可能过度保守；
- 离散 free-volume complex 不易直接与 UAV dynamics 和连续 trajectory optimization 联合。

此前考虑过学习：

- pair priority；
- direction priority；
- slab / cell refinement priority；
- certificate gain；
- solver warm start。

这些方案可以做工程加速，但不适合作为核心学习方法，因为它们学习的是某个 solver 的执行策略，而不是原始 Gaussian planning 问题中的稳定数学对象。

本方案选择：

\[
\boxed{
\textbf{Deep-Unfolded Gaussian Polar Contact}
}
\]

其核心是：对每个 scene Gaussian 与 robot-body Gaussian，在当前机器人 pose 下，先写出一个精确的 polar contact optimization；该问题的最优变量是具有明确几何意义的 contact covector / separating normal。网络只摊销这个反复求解的 latent variable，显式 trajectory optimizer 继续负责状态、动力学、控制和边界约束。

一句话方法：

\[
\boxed{
\textbf{Learn the Gaussian contact latent; optimize the trajectory explicitly.}
}
\]

中文：

> **学习 Gaussian pair 的连续接触隐变量；轨迹、动力学和安全约束仍由显式优化器处理。**

---

# 1. 为什么之前的 learning 方向不够优雅

## 1.1 学习的是实现行为，而不是问题变量

`pair priority`、`direction priority`、`slab priority`、`certificate gain` 等量依赖：

- 当前 refinement rule；
- 当前 Boolean backend；
- 当前 cache；
- 当前 tolerance；
- 当前并行策略；
- 当前数据结构。

一旦更换 solver，实现标签就会变化。

它们描述的是：

\[
\text{how this implementation happens to run},
\]

而不是：

\[
\text{what the Gaussian--robot geometry intrinsically is}.
\]

因此这些量缺乏稳定的数学身份。

## 1.2 多个异质输出没有统一 latent object

若模型同时预测：

\[
\text{pair score},\quad
\text{direction score},\quad
\text{cell score},\quad
\text{split score},\quad
\text{warm start},
\]

那么“统一模型”只是软件接口上的统一，不是理论上的统一。

网络究竟学了什么，很难用一个精确数学对象回答。

## 1.3 不具有 Gaussian 专属性

同样的 learned scheduling 可以用于：

- mesh refinement；
- voxel subdivision；
- branch-and-bound；
- SAT / MILP search；
- generic motion-planning roadmap。

因此它没有回答：

> 为什么这个 learning 是由 Gaussian C-obstacle 的特殊结构自然推出的？

## 1.4 标签不唯一

同一个最终 certificate 可能由多种顺序获得：

\[
a_1\rightarrow a_2\rightarrow a_3,
\]

或：

\[
a_2\rightarrow a_1\rightarrow a_3.
\]

“最佳下一步”还依赖预算、缓存和硬件，因此监督本身不稳定。

## 1.5 主要只改运行时间，不改变几何能力

一个更好的 scheduler 最多减少 support calls 或 refinement 次数，但不会根本消除：

- `S^2` direction discretization；
- 三维 Boolean arrangement；
- yaw / `SO(3)` orientation tessellation；
- inner/outer envelope 在窄通道中的离散保守性；
- 与 dynamics 联合优化的困难。

## 1.6 容易被简单解析规则替代

必须与以下 deterministic baselines 比较：

- largest-residual-first；
- exact sensitivity；
- incremental Boolean update；
- caching；
- priority queue；
- parallel batch refinement。

如果这些规则已接近最优，learning 很难形成独立科学贡献。

## 1.7 正式结论

\[
\boxed{
\text{routing / certificate / gating learning 可以保留为工程工具，}
\quad
\text{但不作为核心研究方法。}
}
\]

---

# 2. 三个候选方案

| 候选 | 学习对象 | 优势 | 根本问题 | 决策 |
|---|---|---|---|---|
| A. Neural Gaussian C-space field | 直接预测 scene--robot configuration distance、collision value 或 gradient | 便于接入 trajectory optimizer；连续 | 直接学习答案；scene-level 数据重；active-pair 切换非光滑；难以保留严格几何含义 | 不选主线 |
| B. Physics-distilled local reachability | 预测 embodiment-conditioned 局部可行裕度、代价和不确定性 | 可处理 noisy 3DGS 与真实物理不一致 | teacher 依赖完整 local planner / dynamics；改变了 collision semantics；是另一个科学问题 | 作为独立分支保留 |
| C. Deep-unfolded Gaussian polar contact | 预测 Gaussian pair 精确 polar problem 的最优 contact covector / normal | 局部自动标注；数学对象唯一；Gaussian-specific；同一模型支持受限 `SE(2)` 与完整 `SE(3)`；显式 optimizer 保留 | 必须证明 repeated inner solve 具有足够开销与摊销空间 | **选择** |

最终选择：

\[
\boxed{
\textbf{C. Deep-Unfolded Gaussian Polar Contact}
}
\]

---

# 3. 与当前严格解析 3DGS 方法的关系

当前三维解析主线是：

```text
full 3DGS
+ full 3D robot body
-> pairwise Gaussian C-obstacles
-> spherical inner/outer pair envelopes
-> 3D obstacle union / workspace complement
-> free-volume decomposition
-> yaw or orientation product complex
-> global path
-> direct continuous Gaussian verification
```

本方案不否定其中的 Gaussian collision semantics，也不学习已有的闭式 support function。

需要严格区分：

## 3.1 保留的解析对象

- calibrated iso-density hard supports；
- scene Gaussian mean 与完整 covariance；
- robot Gaussian mean 与完整 covariance；
- pair identity；
- exact pair support value；
- exact pair support point；
- conservative BVH pair pruning；
- robot motion / dynamics constraints；
- final direct continuous Gaussian verification。

## 3.2 被重新表达的部分

当前方法首先编译整个或局部配置空间障碍，再从 free-volume complex 中找路径。

本方案改为：

```text
candidate trajectory
-> repeated pairwise continuous contact problems
-> latent contact variables
-> explicit constrained trajectory optimization
-> direct continuous Gaussian verification
```

也就是从：

\[
\text{global C-space compilation},
\]

转为：

\[
\text{trajectory-local contact optimization}.
\]

## 3.3 Learning 所处位置

Learning 不替代 Gaussian support oracle，而是替代下面这个反复执行的内层求解：

\[
\boxed{
\text{给定 pair geometry 与当前 pose，求最优连续 contact / separating direction。}
}
\]

---

# 4. 三维 Gaussian pair C-obstacle

## 4.1 Scene 与 robot hard supports

对 scene Gaussian `i`：

\[
E_i
=
\left\{
X:
(X-\mu_i)^\top\Sigma_i^{-1}(X-\mu_i)
\le \kappa_i^2
\right\}.
\]

对 robot-body Gaussian `j`：

\[
B_j
=
\left\{
X:
(X-\nu_j)^\top\Lambda_j^{-1}(X-\nu_j)
\le \rho_j^2
\right\}.
\]

机器人 pose 为：

\[
q=(p,R),
\qquad
p\in\mathbb R^3,
\quad
R\in SO(3).
\]

对地面机器人，`q` 可以受限于：

\[
p=(x,y,z_0),
\qquad
R=R_z(\theta),
\]

但碰撞几何仍在完整三维中计算。

## 4.2 固定 rotation 时的 translation C-obstacle

pair `(i,j)` 的配置障碍为：

\[
\mathcal O_{ij}^{R}
=
\mu_i-R\nu_j
+
E_i^0
\oplus
\left(-RB_j^0\right).
\]

记其中心为：

\[
c_{ij}(R)=\mu_i-R\nu_j.
\]

记中心化凸体为：

\[
K_{ij}(R)
=
E_i^0\oplus(-RB_j^0).
\]

则：

\[
\mathcal O_{ij}^{R}=c_{ij}(R)+K_{ij}(R).
\]

## 4.3 闭式 support function

对任意方向 `u`：

\[
\boxed{
h_{ij,R}(u)
=
\kappa_i\sqrt{u^\top\Sigma_i u}
+
\rho_j\sqrt{u^\top R\Lambda_jR^\top u}.
}
\]

因此：

\[
\boxed{
\text{support value 和 support point 已经是闭式解析量，不应由网络学习。}
}
\]

---

# 5. 精确 Gaussian polar contact problem

## 5.1 相对 translation

定义机器人 translation 相对于 pair C-obstacle 中心的位移：

\[
\boxed{
d_{ij}(q)
=
p-c_{ij}(R)
=
p-\mu_i+R\nu_j.
}
\]

pair 碰撞等价于：

\[
d_{ij}(q)\in K_{ij}(R).
\]

## 5.2 Minkowski gauge

定义：

\[
\gamma_{ij}(q)
=
\inf\left\{
a\ge 0:
d_{ij}(q)\in aK_{ij}(R)
\right\}.
\]

其碰撞含义为：

\[
\boxed{
\begin{aligned}
\gamma_{ij}(q)&>1
&&\Longleftrightarrow
\text{pair separated},\\
\gamma_{ij}(q)&=1
&&\Longleftrightarrow
\text{pair touching},\\
\gamma_{ij}(q)&<1
&&\Longleftrightarrow
\text{pair overlapping}.
\end{aligned}
}
\]

## 5.3 Polar-dual expression

由凸体 gauge 与 polar set 的关系：

\[
\boxed{
\gamma_{ij}(q)
=
\max_{h_{ij,R}(u)\le1}
u^\top d_{ij}(q).
}
\]

上式中的 \(u\in\mathbb R^3\) 是 polar-space covector。可行域

\[
K_{ij}(R)^\circ
=
\{u:h_{ij,R}(u)\le1\}
\]

是中心化 pair C-obstacle 的 polar body。因此，这不是对离散方向表的搜索，而是一个连续三维凸优化问题。

这是本方法的精确内层优化问题。

## 5.4 最优 latent variable

记最优解：

\[
\boxed{
u_{ij}^\star(q)
=
\arg\max_{h_{ij,R}(u)\le1}u^\top d_{ij}(q).
}
\]

Gaussian latent contact feature 定义为：

\[
\boxed{
\operatorname{GPC}_{ij}(q)
=
\left(
u_{ij}^\star(q),\gamma_{ij}^\star(q)\right),
}
\]

其中：

- `u*` 是 polar contact covector / separating covector；
- `gamma*` 是 pair gauge；
- `gamma* > 1` 表示分离；
- `gamma* = 1` 表示接触；
- `gamma* < 1` 表示重叠。

## 5.5 KKT 条件与几何意义

拉格朗日条件为：

\[
d_{ij}(q)
=
\lambda_{ij}^\star
\nabla h_{ij,R}(u_{ij}^\star),
\]

\[
h_{ij,R}(u_{ij}^\star)=1,
\qquad
\lambda_{ij}^\star\ge0.
\]

由于 support function 一阶齐次：

\[
u^\top\nabla h(u)=h(u),
\]

所以：

\[
\boxed{
\lambda_{ij}^\star
=
{u_{ij}^\star}^\top d_{ij}(q)
=
\gamma_{ij}^\star(q).
}
\]

此外：

\[
\nabla h_{ij,R}(u)
=
\kappa_i
\frac{\Sigma_i u}{\sqrt{u^\top\Sigma_i u}}
+
\rho_j
\frac{R\Lambda_jR^\top u}
{\sqrt{u^\top R\Lambda_jR^\top u}}.
\]

因此：

\[
\boxed{
x_{ij}^\star
=
\nabla h_{ij,R}(u_{ij}^\star)
=
\frac{d_{ij}(q)}{\gamma_{ij}^\star(q)}
}
\]

是从 C-obstacle 中心沿当前相对位移射线到达的边界接触点。

这个 latent variable 不是任意神经 feature，而是精确 Gaussian C-obstacle polar problem 的最优解。

---

# 6. 等价的单位球 formulation

为了使网络输出方向而不是带尺度的 covector，令：

\[
n\in S^2,
\qquad
u(n)=\frac{n}{h_{ij,R}(n)}.
\]

则自动满足：

\[
h_{ij,R}(u(n))=1.
\]

于是：

\[
\boxed{
\gamma_{ij}(q)
=
\max_{\|n\|=1}
\frac{n^\top d_{ij}(q)}{h_{ij,R}(n)}.
}
\]

定义：

\[
f_{ij}(n;q)
=
\frac{n^\top d_{ij}(q)}{h_{ij,R}(n)}.
\]

网络只需要输出：

\[
\widehat n_{ij}(q)\in S^2.
\]

然后解析恢复：

\[
\widehat u_{ij}
=
\frac{\widehat n_{ij}}
{h_{ij,R}(\widehat n_{ij})},
\]

\[
\widehat\gamma_{ij}
=
\widehat u_{ij}^\top d_{ij}(q)
=
\frac{\widehat n_{ij}^\top d_{ij}(q)}
{h_{ij,R}(\widehat n_{ij})}.
\]

这带来两个重要性质：

1. 网络无论输出多差，`u_hat` 都满足 exact polar feasibility：

   \[
   h_{ij,R}(\widehat u_{ij})=1.
   \]

2. 因为 `gamma*` 是最大值：

   \[
   \boxed{
   \widehat\gamma_{ij}\le\gamma_{ij}^\star.
   }
   \]

因此网络预测天然给出 pair gauge 的下界，而不是不受约束的标量猜测。

---

# 7. Deep-unfolded contact operator

## 7.1 不使用普通 black-box direction regressor

第一版不建议直接使用一个大 Transformer：

```text
pair parameters -> arbitrary MLP -> contact direction
```

更合适的是从精确 spherical optimization 展开固定层数的迭代。

## 7.2 精确球面梯度

对：

\[
f(n)=\frac{n^\top d}{h(n)},
\]

欧氏梯度为：

\[
\nabla f(n)
=
\frac{
d\,h(n)
-
(n^\top d)\nabla h(n)
}
{h(n)^2}.
\]

球面切空间梯度为：

\[
\boxed{
g_{S^2}(n)
=
\left(I-nn^\top\right)\nabla f(n).
}
\]

## 7.3 展开层

初始化：

\[
n^{(0)}
=
\frac{d}{\|d\|},
\qquad
\|d\|>0.
\]

第 `l` 层：

\[
\widetilde n^{(l+1)}
=
 n^{(l)}
+
\eta_l
\left(I-n^{(l)}{n^{(l)}}^\top\right)
P_l\,g_{S^2}(n^{(l)}),
\]

\[
\boxed{
n^{(l+1)}
=
\frac{\widetilde n^{(l+1)}}
{\|\widetilde n^{(l+1)}\|}.
}
\]

可学习量限制为：

- step size `eta_l`；
- damping；
- 低维、正定且以等变方式构造的 preconditioner `P_l`；
- 可选的 covariance-conditioned initialization correction。

第一版应优先保持：

- rotation equivariance；
- permutation-independent pair processing；
- fixed-depth inference；
- 每一层都有明确优化意义。

## 7.4 输入

单个 pair / pose 的最小输入是：

\[
\boxed{
\left(
d_{ij}(q),
\Sigma_i,
R\Lambda_jR^\top,
\kappa_i,
\rho_j
\right).
}
\]

它已经包含：

- scene Gaussian shape；
- robot Gaussian shape；
- 当前 robot orientation；
- pair relative translation；
- collision levels。

不需要输入：

- start--goal global map；
-完整导航轨迹标签；
- graph node ID；
- slab ID；
- solver routing history。

## 7.5 输出

最小输出：

\[
\boxed{
\widehat n_{ij}(q)\in S^2.
}
\]

其他量全部解析得到：

\[
\widehat u_{ij},
\quad
\widehat\gamma_{ij},
\quad
\widehat x_{ij}^{contact},
\quad
\nabla_p\widehat\gamma_{ij},
\quad
\nabla_R\widehat\gamma_{ij}.
\]

网络不直接输出：

- collision / free label；
- final safety status；
- path；
- active pair set；
- graph edge；
- certificate；
- routing decision。

---

# 8. 训练数据与 exact teacher

## 8.1 不需要完整场景轨迹

训练样本可由局部 pair geometry 自动生成：

\[
\left(
\Sigma_i,
\Lambda_j,
R,
d,
\kappa_i,
\rho_j
\right).
\]

数据来源可混合：

1. 当前 3DGS scenes 与 robot models 中抽取的真实 covariance；
2. 合理谱范围内随机生成的 SPD matrices；
3. 随机 orientations；
4. near-contact 与 narrow-clearance oversampling；
5. 大小、长宽比和 anisotropy 的 counterfactual pairs。

## 8.2 Exact teacher

使用高精度 convex / Riemannian solver 求：

\[
n^\star
=
\arg\max_{\|n\|=1}
\frac{n^\top d}{h_R(n)}.
\]

每个标签必须通过以下检查：

- primal polar feasibility；
- KKT residual；
- multi-start agreement；
- objective upper/lower bound；
- 与 independent pair collision oracle 一致；
- near-boundary 高精度复算。

保存：

```text
scene covariance Sigma
robot covariance Lambda
relative rotation R
relative displacement d
support levels kappa, rho
optimal direction n_star
polar covector u_star
optimal gauge gamma_star
KKT residual
solver iterations
condition indicators
```

## 8.3 采样重点

不能只随机采 far-free pairs。应重点覆盖：

\[
\gamma^\star\in[0.8,1.2],
\]

以及：

- 高 anisotropy；
- 两个 Gaussian 主轴接近正交；
- 极扁或极长 body primitives；
- 接触方向快速变化区域；
- `d` 接近某条对称轴；
- contact normal 条件数差的情况。

## 8.4 损失函数

建议：

\[
\mathcal L
=
\lambda_n\mathcal L_{angle}
+
\lambda_r\mathcal L_{regret}
+
\lambda_k\mathcal L_{KKT}
+
\lambda_g\mathcal L_{gauge}.
\]

### Angular loss

\[
\mathcal L_{angle}
=
1-\widehat n^\top n^\star.
\]

### Objective regret

\[
\mathcal L_{regret}
=
\frac{
\gamma^\star-
\widehat\gamma
}
{\gamma^\star+\varepsilon}.
\]

理论上：

\[
\gamma^\star-\widehat\gamma\ge0.
\]

### KKT residual

\[
\mathcal L_{KKT}
=
\frac{
\left\|
 d-
\widehat\gamma\nabla h_R(\widehat n)
\right\|^2
}
{\|d\|^2+\varepsilon}.
\]

### Gauge regression

\[
\mathcal L_{gauge}
=
\operatorname{Huber}
\left(
\widehat\gamma,
\gamma^\star
\right).
\]

Near-contact samples应增加权重，因为它们直接决定规划边界。

---

# 9. 放回显式 trajectory optimizer

## 9.1 原始规划变量

令：

\[
\mathcal Q=\{q_h\}_{h=0}^{H},
\qquad
\mathcal U=\{a_h\}_{h=0}^{H-1}.
\]

显式优化问题负责：

- start / goal boundary；
- ground manifold 或 UAV pose；
- kinematics / dynamics；
- velocity、acceleration、yaw rate；
- thrust、tilt、jerk；
- control bounds；
- trajectory smoothness；
- task cost。

## 9.2 固定 contact latent 后的 exact sufficient constraint

对网络给出的单位方向 `n_bar`，使用 exact support inequality：

\[
\boxed{
\bar n_{ijh}^\top d_{ij}(q_h)
-
h_{ij,R_h}(\bar n_{ijh})
\ge
\delta_{ijh}.
}
\]

其中：

\[
\delta_{ijh}>0
\]

是沿该 separating plane normal 的安全裕度。

该约束具有关键性质：

\[
\bar n^\top d>h_R(\bar n)
\quad\Longrightarrow\quad
 d\notin K_R.
\]

因此，当该解析不等式被满足时，它是 pair separation 的**充分条件**，不是网络概率判断。

网络方向不准确时，主要后果是：

- 约束过于保守；
- 外层 optimizer 暂时找不到可行解；
- 需要重新更新 contact latent。

它不应直接导致把已碰撞状态判为 free。

## 9.3 交替闭环

完整迭代：

```text
current trajectory Q^k
-> evaluate pair-relative Gaussian geometry
-> unfolded contact operator predicts n^(k+1)
-> explicit constrained optimizer updates trajectory Q^(k+1)
-> repeat until convergence / budget
-> direct continuous Gaussian verification
```

数学形式：

\[
\boxed{
\mathcal Q^k
\xrightarrow{\text{contact operator}}
\mathcal N^{k+1}
\xrightarrow{\text{explicit optimizer}}
\mathcal Q^{k+1}.
}
\]

## 9.4 Pair coverage

不能由网络删除 pair。

候选 pairs 必须由 conservative analytic mechanism 给出，例如：

- scene BVH；
- robot swept-radius bound；
- trajectory tube overlap；
- conservative spatial hashing；
- exact fallback。

Learning 仅计算保留下来的每个 pair 的 latent contact variable。

## 9.5 Final verification

最终结果必须重新检查：

\[
T(q(t))B_j\cap E_i=\emptyset,
\qquad
\forall t, i, j.
\]

使用：

- direct Gaussian pair oracle；
- conservative advancement；
- adaptive interval subdivision；
- exact / high-accuracy fallback。

网络输出不能替代 continuous verification。

---

# 10. 如何统一 ground `SE(2)` 与 UAV `SE(3)`

统一不是把一个 `SE(2)` scheduler 与一个 `SE(3)` scheduler放在同一网络里，而是因为二者共享完全相同的 pair contact problem。

## 10.1 Ground robot

\[
q=(x,y,\theta)\in SE(2),
\]

嵌入完整三维 pose：

\[
p=(x,y,z_0),
\qquad
R=R_z(\theta).
\]

输入 contact operator：

\[
\left(
d,
\Sigma_i,
R_z(\theta)\Lambda_jR_z(\theta)^\top,
\kappa_i,
\rho_j
\right).
\]

## 10.2 Axisymmetric UAV

\[
q=p\in\mathbb R^3.
\]

若 body orientation 对碰撞不重要，则 `R` 可固定或从 body symmetry 消去。

## 10.3 UAV with yaw

\[
q=(p,\psi)\in\mathbb R^3\times S^1,
\qquad
R=R_z(\psi).
\]

## 10.4 Full UAV

\[
q=(p,R)\in SE(3).
\]

## 10.5 统一结论

对所有情况，contact operator 的输入形式相同：

\[
\boxed{
\left(
d,
\Sigma_i,
R\Lambda_jR^\top,
\kappa_i,
\rho_j
\right).
}
\]

不同之处只由显式 trajectory optimizer 处理：

- 哪些 poses 合法；
- 哪些 controls 合法；
- 哪些 dynamics 必须满足。

因此：

\[
\boxed{
\textbf{只需要一个 3D Gaussian contact learner，}
\quad
\textbf{不需要分别训练 SE(2) 与 SE(3) 模型。}
}
\]

---

# 11. 相对当前解析方法的优势

必须区分两类收益：

1. **解析重写本身**相对全局 C-space compiler 的收益；
2. **learning**相对同一重写下 exact polar solver 的额外摊销收益。

两者不能混写。

## 11.1 解析重写带来的优势

### 优势 1：从全空间编译改为 trajectory-local contact

当前解析方法为了支持任意 query，需要显式构造：

```text
pair obstacle envelopes
-> global obstacle union
-> workspace complement
-> free-volume components
-> portals / product cells
```

新 formulation 只沿候选 trajectory 评估：

\[
H\times N_{pair}^{local}
\]

个 contact problems。

这避免了在每个 query 中物化完整高分辨率 C-space arrangement。

### 优势 2：不需要 `S^2` 全局密集离散来发现当前 active direction

当前 pair envelope 使用：

\[
U=\{u_k\}_{k=1}^{K}\subset S^2.
\]

为了不知道当前接触方向，需要覆盖整颗球。

新 formulation 针对当前 pose 直接求：

\[
n_{ijh}^\star.
\]

从：

\[
\text{many directions over the whole pair obstacle},
\]

变为：

\[
\text{one continuous active direction at the queried state}.
\]

### 优势 3：减少三维 Boolean arrangement 压力

trajectory-local constraints不要求先构造完整：

- pair polytope unions；
- workspace complement；
- repeated yaw-slice unions；
- translation--orientation product cells。

这对大场景、少量 queries、动态 scene 和在线 replanning 尤其重要。

### 优势 4：连续处理 orientation

完整 `R` 直接进入：

\[
h_{ij,R}(n)
=
\kappa_i\sqrt{n^\top\Sigma_i n}
+
\rho_j\sqrt{n^\top R\Lambda_jR^\top n}.
\]

因此可以连续优化：

\[
R\in SO(3),
\]

而不是先构造 yaw intervals 或未来的 `SO(3)` cells。

### 优势 5：减少 narrow-passage 离散保守性

当前 outer polytope 的保守程度取决于 finite direction set。

若关键 narrow passage 的 active normal 没有被充分采样：

- outer obstacle 过大；
- safe free space 过小；
- 通道可能返回 `UNKNOWN`；
- 需要继续 refinement。

新 formulation 使用连续最优 contact direction，在候选轨迹附近可更紧地贴合真实 pair boundary。

### 优势 6：直接适配 dynamics-aware optimization

latent contact direction给出可微结构：

\[
\nabla_p\gamma_{ij}=u_{ij}^\star
\]

在最优解唯一的区域成立。

rotation dependence 也由闭式：

\[
R\nu_j,
\qquad
R\Lambda_jR^\top
\]

求导。

因此 collision geometry 可以直接与：

- minimum-snap；
- thrust / tilt；
- acceleration / jerk；
- nonholonomic ground dynamics；
- trajectory smoothness；

放入同一个连续优化程序。

## 11.2 Learning 相对 exact polar solver 的额外优势

即使采用 trajectory-local formulation，也可以每次精确求 polar problem。Learning 必须证明它比这个强 baseline 更有价值。

其潜在收益是：

### 优势 7：摊销 `pair x horizon x outer-iteration` 的重复内层求解

若：

- `N_pair_local` 个局部 pairs；
- `H` 个 horizon states；
- `K` 次外层 trajectory update；

则需要约：

\[
N_{pair}^{local}\times H\times K
\]

次 contact solves。

固定层数 unfolded inference 可以将不确定迭代次数变成固定计算图，并在 GPU 上批量执行。

### 优势 8：跨场景复用

contact operator只依赖局部 pair geometry，不依赖完整 scene layout。

训练一次后可以用于：

- 不同场景；
- 不同 start / goal；
- 不同局部 pair 组合；
- 同一 covariance 分布中的新 Gaussians。

### 优势 9：更好的 batched hardware utilization

精确 inner solver 通常包含：

- termination checks；
- variable iteration count；
- branch / line search；
- high-accuracy fallback。

展开模型使用固定层数，可以批量处理：

\[
\text{batch}\times H\times N_{pair}^{local}.
\]

### 优势 10：可作为 exact solver 的高质量初始化

即使不完全替代 exact solve，网络也可以输出 `n_hat`，再运行 1--2 次解析 Newton / Riemannian correction。

这保留 exact residual，同时减少 inner iterations。

---

# 12. 与当前解析方法的正面对照

| 维度 | 当前严格解析 compiler | Polar-contact trajectory formulation | Unfolded learning 的额外作用 |
|---|---|---|---|
| Gaussian pair support | 闭式 | 保留同一闭式公式 | 不学习 |
| Contact direction | 全局 `S^2` direction set 间接覆盖 | 每个 pose 连续求一个 active direction | 固定层数摊销求解 |
| 全局 geometry | 3D polytope union / complement | trajectory-local pair constraints | 不涉及 routing learning |
| Orientation | yaw intervals，未来 `SO(3)` cells | 连续 `R in SO(3)` | 同一 contact learner适用 |
| Narrow passage | 受 envelope resolution 影响 | 在候选轨迹附近使用连续 active normal | 减少 inner solve 开销 |
| Dynamics | 通常在路径后追加 | 与 collision constraints 联合优化 | 快速更新 contact latents |
| 多 query 静态场景 | 编译后复用强 | 每个 query 重新优化 | 当前方法可能更优 |
| 动态场景 | arrangement 更新复杂 | 局部 pairs 重新评估 | batched inference 有利 |
| `UNREACHABLE` 证明 | 强 | 一般较弱 | learning 不改善 |
| Global homotopy coverage | 强 | 依赖初始化 / 多启动 | 可由现有解析 coarse planner 提供 |

---

# 13. 新方法不能声称完全支配当前解析方法

当前解析 free-volume complex 在以下方面仍然更强：

- 显式全局 topology；
- 可证明 `UNREACHABLE`；
- 静态 scene 上的大量 start--goal queries；
- 多 homotopy class 系统枚举；
- 不依赖 trajectory initialization；
- 三值认证边界清楚。

新方法的主要弱点：

- outer trajectory optimization 非凸；
- 需要初始路径或多启动；
- 可能陷入局部最优；
- pair 数仍需 analytic pruning；
- full `SE(3)` dynamics 优化本身复杂；
- exact polar inner solve 可能已经足够便宜，从而削弱 learning 必要性。

因此推荐系统不是删除当前 analytic planner，而是：

```text
coarse analytic global structure / initial homotopy
-> unfolded Gaussian polar contacts
-> explicit SE(2) / SE(3) trajectory optimization
-> direct continuous Gaussian verification
```

其中：

- current analytic method 提供全局结构和初始路径；
- 新方法提供连续、低保守性的局部几何与 dynamics-aware refinement；
- learning 不做 global routing。

---

# 14. 安全与正确性边界

## 14.1 网络误差为何主要导致保守，而不是 false-free

对任意预测单位方向 `n_hat`：

\[
\widehat\gamma
=
\frac{\widehat n^\top d}{h_R(\widehat n)}
\le
\gamma^\star.
\]

若显式优化器满足：

\[
\widehat n^\top d-h_R(\widehat n)\ge\delta>0,
\]

则该 pair 必然分离。

方向预测差会让这个充分条件更难满足，但不会把一个不满足该解析不等式的状态自动标记为 safe。

## 14.2 仍需 continuous verification

离散 knot states 的 pair separation 不等价于整条连续轨迹安全。

最终必须检查：

- knot 间 translation；
- rotation sweep；
- body swept volume；
- numerical optimizer tolerance；
- 所有 conservative candidate pairs。

## 14.3 `UNKNOWN` 与 fallback

以下情况应进入 exact fallback 或 `UNKNOWN`：

- `d` 接近零，contact direction 不稳定；
- KKT residual 超阈值；
- unfolded objective regret 过大；
- optimizer 接近 contact boundary；
- final continuous verifier 无法解析；
- conservative pair coverage 不完整。

---

# 15. 最小实验：必须先证明 learning 有必要

实现大网络前，先完成三个层次的 baseline。

## 15.1 Baseline A：当前 global analytic compiler

记录：

- pair support evaluations；
- spherical directions；
- polytope facets；
- Boolean union time；
- free-volume decomposition time；
- yaw intervals / product cells；
- query time；
- final verification time；
- `UNKNOWN` rate；
- minimum direct-Gaussian clearance。

## 15.2 Baseline B：exact polar contact + trajectory optimization

不使用 learning，每个 pair / pose 使用高精度 inner solve。

该 baseline用于回答：

1. trajectory-local reformulation 是否真的减少 3D Boolean 和 orientation discretization；
2. 是否改善 narrow-passage 成功率；
3. 是否支持 dynamics-aware `SE(3)`；
4. polar inner solve 占总体 runtime 的比例是多少。

## 15.3 Method C：unfolded contact + trajectory optimization

比较：

- contact-direction angular error；
- gauge regret；
- KKT residual；
- pair classification near `gamma=1`；
- inner-solve wall time；
- outer optimizer iterations；
- final path success；
- final exact clearance；
- end-to-end planning latency。

## 15.4 必须满足的因果链

\[
\boxed{
\begin{aligned}
&\text{exact polar contact 在 pair x horizon x iteration 上反复出现}\\
&\rightarrow
\text{其累计成本是端到端显著部分}\\
&\rightarrow
\text{unfolded model 高精度预测同一 latent variable}\\
&\rightarrow
\text{显式 optimizer 仍满足解析 pair constraints}\\
&\rightarrow
\text{总体规划时间下降，路径质量或成功率不降低}.
\end{aligned}
}
\]

若 exact polar solve 占比很小，则：

\[
\boxed{
\text{保留 exact polar formulation，但不强行加入 learning。}
}
\]

---

# 16. 关键 benchmark

## 16.1 几何场景

- open vertical ascent；
- low-wall overflight；
- full-height wall；
- bridge / underpass；
- suspended obstacle；
- vertical shaft；
- anisotropic Gaussian corridor；
- narrow passage near envelope resolution limit；
- yaw-dependent slot；
- full-rotation asymmetric body passage。

## 16.2 Robot settings

- ground robot constrained to `SE(2)` but using full 3D body collision；
- axisymmetric UAV in `R^3`；
- non-axisymmetric UAV in `R^3 x S^1`；
- full-pose UAV in `SE(3)`；
- at least two body sizes and anisotropy patterns。

## 16.3 数据 split

- unseen scene layouts；
- unseen Gaussian covariance combinations；
- unseen relative orientations；
- unseen robot size interpolation；
- controlled extrapolation in anisotropy；
- near-contact holdout set。

## 16.4 Metrics

### Inner latent quality

- angle error of `n_hat`；
- normalized gauge regret；
- KKT residual；
- contact / free sign error near `gamma=1`；
- exact correction iterations from `n_hat`。

### Planning quality

- planning success；
- final direct-Gaussian collision rate；
- minimum clearance；
- path length / time / energy；
- dynamics feasibility；
- narrow-passage success；
- sensitivity to initialization。

### Efficiency

- inner contact time；
- total contact calls；
- trajectory optimizer time；
- Boolean compilation time avoided；
- end-to-end planning time；
- peak memory；
- GPU batch throughput。

---

# 17. Ablations

必须包含：

1. exact polar solver，无 learning；
2. direct scalar gauge regression；
3. direct collision classification；
4. black-box direction MLP；
5. unfolded model only learning step sizes；
6. unfolded model with learned preconditioner；
7. no covariance，仅 centers / radii；
8. isotropic covariance；
9. separate ground / UAV models；
10. one shared 3D contact model；
11. no exact correction；
12. 1--2 step exact correction after network；
13. current global analytic compiler；
14. hybrid coarse-global + learned-contact refinement。

最关键的比较不是“有网络 vs 无网络”，而是：

\[
\boxed{
\text{unfolded latent solver}
\quad\text{vs.}\quad
\text{高质量 exact inner solver}.
}
\]

---

# 18. Go / No-Go criteria

## GO

- exact polar teacher 在 adversarial pairs 上通过 KKT 与 independent collision cross-check；
-同一 contact model 在 ground-constrained poses 与 full `SE(3)` poses 上均有效；
- unfolded output 显著减少 exact correction iterations；
- near-contact gauge regret 足够小，不增加 final false-free；
- exact polar inner solves 在真实 trajectory workload 中是可测瓶颈；
- 相对 exact-polar baseline，end-to-end latency有实质下降；
- 相对当前 global compiler，在 orientation-heavy 或 dynamic-query tasks 上获得明显优势；
- final direct Gaussian verifier 的安全结果不降低。

## NO-GO / Reframe

- exact polar solve 本身已极便宜，累计占比很低；
- unfolded inference 开销接近或超过 exact solve；
-网络在 near-contact / high-anisotropy cases 上无法稳定；
- exact correction几乎每次都需要完整重解；
- outer trajectory optimizer 的局部最优问题主导失败，contact latent并非主要瓶颈；
- current analytic compiler 在目标 workload 上因多 query reuse 明显更优；
- full `SE(3)` 的主要困难来自 dynamics 或 initialization，而不是 pair contact。

---

# 19. 文件级实现建议

```text
gmc/src/gmc/geometry3d/
    pair_support.py
    polar_contact.py
    polar_teacher.py
    polar_kkt.py
    collision_pw.py
    collision_gjk.py


gmc/src/gmc/learning/contact/
    unfolded_contact.py
    invariant_features.py
    losses.py
    dataset.py
    exact_correction.py


gmc/src/gmc/optimization/
    trajectory_problem.py
    pair_constraints.py
    alternating_contact.py
    ground_se2.py
    uav_xyz.py
    uav_xyz_yaw.py
    uav_se3.py


gmc/src/gmc/verification/
    continuous_pair_3d.py
    continuous_curve_3d.py
    se3_curve.py


gmc/experiments/contact/
    validate_polar_teacher.py
    benchmark_exact_inner.py
    train_unfolded_contact.py
    evaluate_contact_generalization.py
    compare_global_vs_local.py
    benchmark_ground_uav_shared.py
```

---

# 20. 分阶段实现路线

## Phase 0：数学与数值核验

- 实现 `h_R(u)`、`grad h_R(u)`；
- 实现 gauge polar solver；
- 实现 KKT residual；
- 与 direct pair collision oracle cross-check；
- 对 random / adversarial SPD pairs 做高精度测试。

**Go**：`gamma > 1 / =1 / <1` 与 independent collision oracle 在数值容差内一致。

## Phase 1：exact trajectory-local baseline

- 不使用 learning；
- 将 exact polar contacts 接入 translation-only trajectory optimizer；
- 再接 ground yaw 和 UAV yaw；
- 最后接 full `SE(3)` pose variables；
- 与 current global compiler 比较。

**Go**：证明 trajectory-local formulation 本身具有目标优势。

## Phase 2：数据生成与 unfolded model

- 生成真实 covariance + synthetic SPD pairs；
- oversample near-contact；
- 训练 fixed-depth model；
- 加 exact correction fallback。

**Go**：在 unseen pairs 上显著减少 exact iterations，且 gauge lower-bound property保持。

## Phase 3：alternating trajectory optimization

- trajectory -> contacts -> trajectory 迭代；
- ground `SE(2)` 与 UAV `R^3 x S^1` 共用模型；
- 加入 trust region / proximal stabilization；
- final continuous verification。

**Go**：相对 exact-polar baseline 获得端到端收益。

## Phase 4：full dynamics

- nonholonomic ground constraints；
- UAV velocity / acceleration / thrust / tilt；
- full `SE(3)` rotation；
- dynamics-aware benchmarks。

**Go**：展示解析 global compiler 难以直接处理、而连续 contact formulation 能自然支持的任务。

## Phase 5：hybrid global-local system

- current analytic planner 提供 coarse route / homotopy；
- unfolded contact optimizer做连续 refinement；
- final direct Gaussian verifier；
- 保留 `UNKNOWN` 与 fallback。

---

# 21. 推荐论文表述

## 21.1 核心问题

> 如何在不离散完整三维或六维配置空间的前提下，直接从 Gaussian pair C-obstacles 获得可微、连续、可跨场景复用的机器人接触几何，并将其用于显式 constrained trajectory optimization？

## 21.2 方法主张

> 我们从 Gaussian pair 配置障碍的精确 polar problem 中推导 contact latent variables，将其迭代求解展开为固定深度的 equivariant contact operator，并将这些 latent variables 送入显式 `SE(2)` / `SE(3)` trajectory optimizer。Gaussian support、机器人运动约束和最终连续验证始终保持解析。

## 21.3 最短 claim

\[
\boxed{
\textbf{Amortize Gaussian contact; preserve explicit motion optimization.}
}
\]

中文：

\[
\boxed{
\textbf{摊销 Gaussian 接触求解，保留显式运动优化。}
}
\]

## 21.4 明确不主张

- 不学习 global routing；
- 不学习 certificate；
- 不学习 pair deletion / gating；
- 不直接预测整条路径；
- 不直接回归无约束 collision probability；
- 不替代 direct continuous Gaussian verification；
- 不声称完整替代当前 analytic mobility complex。

---

# 22. 最终系统图

```text
full 3DGS scene Gaussians
+ full 3D robot-body Gaussians
+ current candidate trajectory
                |
                v
conservative analytic pair coverage
                |
                v
Gaussian polar contact problems
                |
                v
fixed-depth unfolded contact operator
predicts n_star / u_star / gamma_star
                |
                v
explicit constrained trajectory optimizer
- ground manifold or SE(3)
- dynamics and control bounds
- start / goal
- smoothness and task cost
                |
                v
updated trajectory
                |
        repeat contact <-> motion
                |
                v
independent continuous Gaussian verification
                |
                v
certified trajectory or UNKNOWN / fallback
```

---

# 23. 最终决策

\[
\boxed{
\textbf{核心 learning 选择：Deep-Unfolded Gaussian Polar Contact。}
}
\]

它不是 learned routing、learned certificate 或 learned gating。

它学习的是：

\[
\boxed{
\text{Gaussian pair C-obstacle 精确 polar problem 的最优连续 contact latent。}
}
\]

相对当前解析方法，它最有价值的能力是：

\[
\boxed{
\begin{aligned}
&\text{从全局高维 C-space 编译转向 trajectory-local continuous contact；}\\
&\text{减少 spherical / orientation discretization；}\\
&\text{减少 3D Boolean 与 product-complex 压力；}\\
&\text{降低 narrow-passage 的 envelope 保守性；}\\
&\text{直接与 ground / UAV dynamics 联合；}\\
&\text{通过展开模型摊销大量重复 pair contact solves。}
\end{aligned}
}
\]

最终是否保留 learning，由 profiling 决定：

\[
\boxed{
\text{先证明 exact trajectory-local reformulation 有价值，}
\quad
\text{再证明 contact inner solve 值得被 learning 摊销。}
}
\]
