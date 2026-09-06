# 探针实验第一轮完整技术报告（code + settings + 数据 + 失败机制）

> **2026-08-13 修正附录(外部评审后)**——评审文件:`probe_round1_full_report_response.md`。
> 逐项处置:
> 1. **[错误确认] 角度浪费因子**:w=0.58 时应为 **5.76×**(两个镜像 θ 窗共 62.5°/360°),
>    我写 12× 时漏算了 θ+π 窗;12× 实际对应 w≈0.52。定性结论不变,定量修正。
> 2. **[命名修正] "balance" 不是 Morse 条件**:|h₊−h₋| 是 scalar equal-clearance 启发式;
>    真正的 min-type 临界条件 0∈conv{∇_q h_k} 含 ∂_θ h,可能本身就有角度选择性。
>    本轮否证的是标量代理,不是 Morse 方法。全文改称 "scalar bilateral equal-clearance"。
> 3. **[评审要求验证→已做,通过] PW→米制界对抗验证**:50 万随机构型 vs 独立
>    checker#2(点-椭圆距离),FREE 侧 445,792 例 + 穿透侧 39,362 例,**0 违例,
>    max ratio = 1.000000**(短轴接触取等 ⟹ b+R 是紧常数)。非形式证明,但已排除
>    循环验证质疑;形式证明列入论文待办。
> 4. **[评审质疑→已查,虚惊+新表述] θ/θ+π 镜像分量**:定点 (−2,−0.5) 整环 180/180 采样
>    角全部认证 FREE 且 union-find 连成 1 个分量 ⟹ θ wrap 与房间环无 bug。成对大分量是
>    **门区细层壳簇在镜像姿态下的未完成认证孤岛**,不是"正常拓扑",改为如实表述。
> 5. **[评审要求→已做] 真值独立见证**:5 个门宽全部构造显式连续路径并通过保守 swept
>    证书,min 余量 2.50/5.00/10.00/20.00/40.00 mm——与解析 (w−0.5)/2 精确吻合。
>    "全部解析可达"升级为"全部构造性认证可达"。
>    **附带重要发现:见证路径只需 28–284 次 margin 检查**——信息下限远小于 64k,
>    表示层认证相对路径层认证的溢价是下一轮的核心测量对象。
> 6. **[错误确认] balance 分数奖励双侧碰撞态**:b=max(h₊,h₋) 双负时分数可为负 → 深碰撞
>    平衡姿态排最前(§九),是 θ-环兔子洞的共因之一。下一轮修复。
> 7. **[接受] 其余全部方法论修正**:tags=±1 是特权探针信息(最终方法须由接触法向自动聚
>    类);计费分三层(pose queries / pair ops / wall-clock+内存);"start-seeded certified
>    reachability probe" 命名;UNKNOWN≠UNREACHABLE 三态输出;补不可达宽度;
>    oracle-guided scheduler 测预算下限;双队列/各向异性 θ-split/角度区间检测器消融矩阵;
>    跨层级精确面邻接;REACHABLE 须回溯 cell chain → 连续路径 → hard 认证闭环。
> 8. **[状态改写] 本轮正式结论**:Round 1 passed as a failure-mechanism probe; the scalar
>    contact-priority hypothesis is falsified. Gate B 未通过;angular-fiber/gradient 检测是
>    有依据的下一假设,尚非已验证方法。

日期:2026-08-12。代码状态:本报告引用的全部代码为 `splatc_atlas` 当前磁盘版本
（tests 20/20,已同步 HPC)。复现入口:
`PYTHONPATH=src python experiments/compiler/probe_pareto.py`。

---

## 1. 完整 Settings

### 1.1 场景(G1 de-aligned,`datasets/g1_gate.py`)

| 参数 | 值 |
|---|---|
| workspace | [−3.5, 3.5] × [−2.3, 2.3] m |
| 墙厚 T(corridor 门) | 1.2 m |
| 门宽 w 扫描 | {0.505, 0.51, 0.52, 0.54, 0.58} m(临界 2b=0.5,全部解析可达)|
| de-alignment | door_offset = 0.013 m,door_tilt = 7° |
| 墙 disc 组成 | 边缘行 r=0.10 @0.02;墙面列 r=0.10 @0.04;填充 r=0.15 @0.2 网格;共 ~390 primitives |
| 墙侧标签 | 每个 disc 带 tag ∈ {+1 上段, −1 下段}(meta.group_tags)|

### 1.2 机器人 / 任务

| 参数 | 值 |
|---|---|
| robot | R_long_ellipse:椭圆 a=0.6, b=0.25(单 primitive)|
| q_start | (−2.0, 0.0, π/2) |
| goal | (2.0, 0.0),半径 0.30,终点朝向自由 |
| 成功判据 | 在预算 checkpoint 处正确回答 REACHABLE |

### 1.3 几何评估常量(`gaussian_geometry/primitives.py`)

```python
PAD = 0.2       # broad-phase 壳层(m):rho 只在 d1 <= a+R+PAD 内精确
RHO_CAP = 0.25  # 壳层外 rho 封顶值(无量纲 PW 单位)
FREE_EPS = 1e-9 # 相切 tie-break:FREE 需 margin > eps
```

### 1.4 探针树常量(`baselines/probe_methods.py`)

```python
ROOT = (7, 5, 8)        # level-0 cell 数(x, y, theta);cell 尺寸 1.0 × 0.92 m × 45°
max_level = 10          # level l cell 尺寸 = ROOT尺寸 / 2^l
batch_splits = 48       # 每轮从堆中取出并细分的 cell 数(每轮产 384 个子 cell)
```

r_cell(Lipschitz 认证半径)按层级:

| level | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|
| r_cell (mm) | 57.2 | 28.6 | 14.3 | 7.1 | 3.6 |

### 1.5 预算

checkpoints = {500, 1k, 2k, 4k, 8k, 16k, 32k, 64k}(一次实验最多 256k)。
**计费合同:每个 cell 中心评估 = 1 次查询,三策略一致;contact 的逐侧分解视为同一次
配对评估的副产品计 0(见 §5.3 声明)。**

---

## 2. 机器代码(当前磁盘版本,verbatim)

### 2.1 可靠认证界(`primitives.eval_points_bounds`,核心段)

```python
# 每个 disc 组:
d1, _ = tree.query(pts, k=1)               # 最近圆心距离(KD-tree)
g_free = d1 - (robot.a + Rg)               # 三角不等式自由下界(处处可靠)
g_pen  = (robot.b + Rg) - d1               # 穿透下界(>0 时可靠)
# 壳层内(d1 <= a+Rg+PAD)的点补 PW 界:
h = pw_ellipse_disc(u1, u2, robot.a, robot.b, Rg)   # PW 缩放余量
scale = robot.b + Rg                        # 可靠 PW→米制系数(接触向延伸 >= b, R)
g_free[seen] = np.maximum(g_free[seen],  hmin[seen] * scale)
g_pen [seen] = np.maximum(g_pen [seen], -hmin[seen] * scale)
# 跨组合成:
m_free = min(box_margin, min_g g_free)      # 自由余量下界(米)
m_pen  = max(-box_margin, max_g g_pen)      # 穿透深度下界(米)
```

界的可靠性论证:PW 缩放至相切时接触点沿中心射线位移 = |h|·(中心到接触点距离),
该距离 ∈ [内切半径, 外切半径] ⟹ gap ≥ h·(b+R),depth ≥ |h|·(b+R)。
经验验证:随机 1601 个 FREE-cert + 1803 个 COLL-cert cell,每个内部采 30 个随机 pose,
共 102,120 个 pose,**违例 0**。

### 2.2 三态认证(`probe_methods._evaluate`)

```python
r_cell = self.cell_radius(k[0])   # |(sx/2, sy/2)| + a_max*(s_theta/2)
if   m_free[n] > r_cell: status = "FREE"   # 整 cell 认证自由,终结
elif m_pen [n] > r_cell: status = "COLL"   # 整 cell 认证碰撞,终结
else:                    status = "AMBIG"  # 唯一可细分状态
```

### 2.3 三策略打分(`probe_methods._push`,当前 v4 版)

```python
if self.policy == "uniform":
    score = float(l)                                  # 层级广度优先
elif self.policy == "generic":
    score = abs(rec["rho"]) + 0.25 * l                # 只准用标量 |rho|(B2 契约)
else:  # contact
    b = rec.get("bilateral", np.inf)                  # max(h+, h-):双侧接近度
    if b < 0.2:                                       # 双侧都真正进入壳层
        score = rec.get("balance", 1.0) + 0.5 * b + 0.15 * l
        #        ^ balance = |h+ - h-|:scalar equal-clearance 启发式(非 Morse 条件)
    else:
        score = 3.0 + abs(rec["rho"]) + 0.25 * l      # 非门区退化为 generic+惩罚
if rec["rho"] > 0.15 and l <= 2:                      # 开阔区粗骨架分支(全策略一致)
    score = min(score, 0.6 + 0.6 * l)
heapq.heappush(self._heap, (score, self._tick, key))
```

逐侧信号来源(`primitives.side_rho`):对 tag=+1 / −1 的子场景分别求 rho;
`bilateral = max(h+, h−)`,`balance = |h+ − h−|`。

### 2.4 答案提取(`probe_methods._answer`)

- 连通性:仅在认证 FREE cell 上做 union-find;邻接 = 6 个面中心探针点定位到的叶
  (θ 周期,跨层级)。⟹ **REACHABLE 不可能假阳(单侧误差)**。
- start:所在叶必须 FREE-cert(起点 cell 有强制种子细分,全策略一致);
- goal:FREE cell 的 xy 足迹与 goal 圆盘相交(v3 修复前误用"中心距离")。

---

## 3. 打分函数版本演化史(全部在 dev 场景上,blind 未触碰)

| 版 | contact 打分 | 结果(w=0.58, 64k)| 死因 |
|---|---|---|---|
| v1(无认证,中心标签)| `b + 0.25l`(b<0.5)| 296 free cell,走廊 0,层级直方图 {4:7730, 5:43882} | 墙内 cell 双侧 h 小 → 兔子洞无终结条件;**跳过了 Stage 3 第 2 步** |
| v2(认证树)| `b + 0.25l`(b<0.2)+ 种子细分 | U;free 16→10096 | 通道祖先排不过门前 free 壳层 |
| v2′ | `in_channel(0/1) + b + 0.15l`,预算至 256k | U;free 最多 148,524 | in_channel 反噬:通道祖先中心在碰撞态被罚 1.0,永远输给房间侧 free bilateral 壳层(自我繁殖)|
| v3(scalar equal-clearance)| `balance + 0.5b + 0.15l` | U;**但走廊内认证出 1468 个 FREE cell(L7:406, L8:1062)**;分量 14,start 分量仅 8 cell,goal free cell 0 | 房间没认证(群岛);goal 判定 bug |
| v4(+开阔分支+goal 修复)| v3 + `min(score, 0.6+0.6l)`(rho>0.15, l≤2)| U;free 14119(L6-8 有 13.5k),走廊 0;左房间成单一分量但左右不通 | 预算零和:房间骨架 + 门轴×全 θ 圆环的 balanced 区域,把走廊阶梯挤出 64k |

对照数据(v4,w=0.58,64k):uniform 9s / generic 12s / contact 70s(wall-clock);
free cell 层级分布 {1:372, 2:15, 4:20, 5:146, 6:1598, 7:5594, 8:6374};
分量 top sizes [4208, 4200, 1765, 1765](θ/θ+π 镜像成对——经定点整环检查为门区细层壳簇的未完成认证孤岛,非拓扑错误亦非'正常现象';房间 θ-环本身认证完整且连通)。

---

## 4. 定量失败机制

通道解析核算(w=0.58, tilt=7°):中轴可靠余量 m_free ∈ [40.0, 84.7] mm
⟹ **level 6(r_cell=14.3mm)即可认证通道链**,链长约 115 个 cell。注意(评审 §七):
115 个叶 ≠ 115 次查询(祖先/兄弟/8 子女/房间骨架/邻接全要算),"预算足够、瓶颈纯在排序"
目前是合理假设而非结论,由 oracle-guided floor 实验判定:

1. **uniform:体积诅咒。** 均匀铺到 level 6 需 280×8⁶ ≈ 7×10⁷ 查询;64k 只够 level ≈ 3。
2. **generic:边界面积诅咒。** |ρ|≈0 壳层 = 全部墙面 × 全部 θ;门占比 ≪ 1%。
3. **contact(bilateral+equal-clearance):θ 无选择性。** balance≈0 在门轴上**对所有 θ 成立**
   (两侧几何对称与朝向无关)⟹ 细分预算摊到"门轴 × 整个 θ 圆环";可通行 θ 窗
   (两个镜像窗共 62.5°)占圆环 17.4% ⟹ **5.76× 角度浪费**(修正:原 12× 漏算 θ+π 窗;
   另有 balance 分数奖励双侧碰撞态的共因,见修正附录 §6)。是否足以解释 64k 失败
   尚未证明,待 oracle-guided floor 实验。
   v3→v4 的走廊 free cell 从 1468 → 0 正是预算零和的直接证据。

## 5. 有效性声明(如实)

1. **超参调整史**:contact 打分经 4 版手调(系数 0.25l/0.15l/0.5b、阈值 0.2/0.15、
   开阔分支 0.6+0.6l 与 l≤2 封顶),全部只在 dev 场景;**继续手调 = 对单场景过拟合,
   已停止**。最终方法的打分必须冻结后过 validation。
2. **计费公平但 wall-clock 不公平**:side_rho 当前是子场景重复评估(contact 慢 ~7×);
   真实实现中 pair identity 是同一次配对评估的副产品。查询计费按合同一致;
   合并实现列入待办。
3. **baseline 失败的证据等级**:与理论一致,但完整 w × budget Pareto 扫描尚未在
   认证版机器上重跑;正式曲线待 gate 检测器完成后统一出。
4. **邻接保守性**:面中心探针可能漏掉跨层级阶梯邻居 → 只会加重 false-unreachable
   (单侧),不影响可靠性方向。

## 6. 结论与下一级信号

前两级 contact 线索(pair identity、bilateral/balance)解决了"门在哪"(位置定位成功,
走廊曾被认证),未解决"哪个姿态窗可通行"。02 号 B2 禁用清单中保留给主方法的第三级信号
**gate-specific angular interval detector**(= 08 §8.2 adaptive critical-contact
discovery)是下一步:对检测到的门轴做显式 1D 角度扫描测 Θ 窗,细分只在窗内展开。
若第三级在 matched budget 下打穿而 generic 仍失败 ⟹ Gate B 命题成立;
若仍不够 ⟹ 按 10.2 节纪律审视 claim。
