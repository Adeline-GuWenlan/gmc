# splatc_atlas

SplatC-Atlas 的执行仓库。计划文档(canonical)在上层目录:`00_MASTER_EXECUTION_PLAN.md` 起。

## 布局

```text
docs/problem_spec.md      Stage 0 冻结合同(单位、SE(2)、h 语义、G1 几何、解析 gate 公式)
docs/worklog/             按 sprint 工作日志(边做边写)
docs/gate_reports/        每个 Gate 的 PASS/FAIL 报告
configs/g1_pilot.json     冻结的 pilot 配置
src/splatc/
  common/se2.py           SE(2) 约定与统一路径代价 J
  gaussian_geometry/      Perram–Wertheim checker(#1)+ 独立点–椭圆 checker(#2)+ broad phase
  datasets/g1_gate.py     G1 场景生成器 + 三 morphology robot library + 解析 gate 真值
  reference/oracle.py     Dense SE(2) oracle(周期 θ 分量、可达性、Dijkstra、连续认证)
  evaluation/records.py   统一记录 schema(可回放)
tests/test_analytic.py    15 个解析/等变/交叉校验 case(Gate 0)
experiments/oracle/rotate_door.py      决定性首图:长椭圆侧身穿门(无 route seed)
experiments/thin_gate/gate_sweep.py    门宽 sweep:解析 vs oracle 区间 + 分辨率 false-unreachable
results/                  figures / tables / manifests(全部由脚本生成)
```

## 运行

```bash
PYTHONPATH=src python -m pytest tests -q
PYTHONPATH=src python experiments/oracle/rotate_door.py medium
PYTHONPATH=src python experiments/thin_gate/gate_sweep.py
```

本地 pilot 规模可直接跑(medium 档椭圆单次 oracle ~5s);SplatC-Procedural 与公开数据阶段
迁移到 HPC(见顶层 README 的 overlay/srun 纪律),oracle 生成写成 sbatch 批处理。

## 纪律红线(引自计划 00/05)

- candidate 不得读取 oracle mask / path / gate label;
- 所有最终路径由统一 hard checker 重新认证;
- compile 与 per-goal query 分开计时;报告 collision-query 数;
- Gate 未过不得进入下一 Stage。
