# Baseline / 数据集执行审计（2026-08-12,后台 agent 核查）

## SE(2) NavMesh（arXiv:2607.01454）——未放码,复现触发条件生效

- 项目页只有无链接的 "Code (soon)" 占位;org 下仅网站 repo。**01 号文档的触发条件生效:
  faithful reimplementation 立即启动。**
- 论文可用材料:Alg.1 体素分类 / Alg.2 体素 validity / Alg.3 NavMesh validity;
  连通性公式 (22)(23),代价 (27)(28),启发式 (29);完整超参表(N_ψ=40 yaw layers,
  0.1m 体素,ANYmal footprint 0.93×0.53×0.89,v_long 0.5 等);HM3D 六个命名场景;
  baseline RRT/RRT*/PRM。
- 复现三件套按难度:(a) yaw 分层 traversability(footprint-mask BFS,好写);
  (b) 连通图(watershed 分割 + ear-clipping 三角化 + 凸合并——**只有 prose,主要风险**);
  (c) ASA 寻路(A* + string-pulling + yaw 二次 A*,可写但调参敏感)。
- 我们的 2D subset 策略(02 B3):(a)+(c) 必做;(b) 可用网格内连通替代并标注
  `our reimplementation, 2D subset`。对齐参数:N_ψ=40。

## HRM——可用但 GPL-3.0,进程隔离

- 代码最后实质提交 2023-06;C++/CMake,依赖 OMPL≥1.5 / FCL / CGAL / KDL / Eigen / Boost。
- HRM2D 单椭圆体可独立跑;场景/机器人是 `/resources/` 下 CSV(椭球/超二次曲面)——
  **与我们的椭圆场景格式天然接近**。无 Python 绑定。
- **GPL-3.0:只能作为进程外 baseline 可执行文件调用(CSV 进出),绝不链接进我们的栈。**

## 数据集许可(派生 manifest 可否发布)

| 数据集 | 结论 |
|---|---|
| BARN | 无任何 LICENSE;manifest 是我们自产,可发,引用 Perille 2020 |
| Habitat-GS | 代码 MIT;HF 数据标 Apache-2.0 **但捆绑 64 个 InteriorGS 衍生场景,标签疑似越权**——按 02 建议只用 self-reconstructed subset |
| InteriorGS | 仅限非商用研究;ToU §2 禁止再分发下载数据。**ID+坐标 manifest 可发;几何与 occupancy PNG 一律不发** |
| HM3D | Matterport 学术 EULA;论文引用可,实质性再分发需 click-wrap;manifest 可发 |

## OMPL——pip 可装,一处计划修正

- PyPI `ompl` v2.0.1(BSD-3,nanobind,manylinux/macOS wheels,py3.10–3.13)✓。
- RRT-Connect 已绑定 ✓;**LazyPRM\* 未绑定**(只有 PRM/PRM\*)。
- **计划修正:02 号 B8 的 LazyPRM\* 改为 PRM\***(multi-query 语义等价于我们的比较目的),
  或后续自行补绑定。忽略过时的 `ompl-thin`。
