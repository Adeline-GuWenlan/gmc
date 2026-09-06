## 必读优先级最高

**Meta-rule**:最危险的状态不是"我没查",而是"**我把能查的都查了,所以一定是别人问题**"。绿勾数量 ≠ 距离 bug 的距离,verified list 本身就可能是没被审查的前提。

1. **Verified list 不是完整性证明**。N 项检查都过但结果错时,reflex 是"扩大清单",不是外部归因。当前清单上没有的环节才是嫌疑最大的。
   - 触发:已验证项 ✓ 但产出仍错。
   - 做法:写下"数据流里我从未看过中间产物的环节",从那里继续。

2. **Pipeline 先 visualize,再算 metric**。任何输出维度 > 标量的 pipeline(几何 / ML / parser / renderer),**第一件事**是出 human 能直接看的 artifact(PLY 进 meshlab、render 截图、AST 树),开真的 viewer 看 —— **再**算 scalar。Scalar 通过 ≠ pipeline 对。matplotlib 2D 投影是 lossy 的,不算数。
   - 触发:写完任何 high-dim → high-dim pipeline 后。
   - 做法:emit artifact → real viewer → 然后才 metric。

4. **用户"这不对"是 anchor signal,不是噪音**。verification 都过但用户 push back 时,**停止 retune 参数**,产出让用户能直接看的 artifact,让用户 anchor 到具体特征,我从 anchor 反推 bug。
   - 触发:用户反复说不对而我检查都过了。
   - 做法:产出 visualization → 让用户指出 → 反推。**用户的 anchor 比我穷举清单高一个量级效率**。

5. **外部归因恰好免除我责任时,警惕**。结论是"是别人的 bug / 等他们 release / spec 没写清"且这个结论恰好让我不用继续 debug —— 这是回避信号。回避常伪装成"理性资源分配"或"耐心等待"。
   - 触发:决定停止 debug 并归因外部。
   - 做法:先写"**如果只能靠自己解决,下一步会做什么?**"。非空就先做,再决定要不要等。

6. **压力下默认 challenge 清单,不是 defend 清单**。被 push 时 reflex 是 defend 已验证项(retune 未验证的),正确反过来:**challenge 已验证项**("这条 verification 真的覆盖它声称的语义吗?")。
   - 触发:用户压力 + 多次 retune 无效。
   - 做法:对每条 verification 写 (a) 它能排除的 bug class,(b) 它无法排除的。专攻 (b)。

## Environment

项目运行在 HPC 上,不在本地。

- **入口**:`ssh hpc:/scratch/sy2366/Project/xxx(项目名)`(全小写)
- HPC 存储配额有限,**conda 环境和数据集都通过 overlay 管理**,不要直接装到共享文件系统上。
- **查看 overlay 内容(只读 peek)用登录节点的 `mount-overlay` helper**,**不要为了 `ls` 一下就 srun**:
  ```
  mount-overlay -o xxx(项目名).ext3                # 单个 overlay
  mount-overlay -o xxx1(项目名).ext3 -o xxx2(项目名).ext3   # 多个叠加
  ```
  它内部跑 `singularity shell --overlay=... centos-8.2.2004.sif`,把 overlay 挂到根目录(挂载点跟 overlay 内部布局有关)。要非交互执行用 `singularity exec --overlay=foo.ext3:ro /share/apps/admin/singularity-images/centos-8.2.2004.sif bash -c "..."`(同一个 SIF)。
- **重活必须在计算节点执行**(`srun` 或 `sbatch`),登录节点上不要跑。包括:
  - 创建 / 修改 conda 环境
  - 下载、写入数据集
  - 任何训练 / 推理任务
  - 优先使用专属计算节点通过添加 -q cair
  - 优先使用a100/h100/h200。 例如：-gres=gpu:a100:n， -gres=gpu:h100:n，-gres=gpu:h200:n, n表示所需卡的数量，
  - -C 80/40g 可以用于选择a100-80g或者a100-40g

参考文档:
- conda + overlay:https://crc-docs.abudhabi.nyu.edu/hpc/software/conda_with_overlay.html
- singularity overlay:https://crc-docs.abudhabi.nyu.edu/hpc/software/singularity_overlays.html

**HPC 侧**(独立组织,跟 local 不一一对应):
- `/scratch/sy2366/Project/xxx(项目名)/scripts/` — 当前在用的 HPC 脚本
- `/scratch/.../outputs/` — 所有 推理 raw output
- `/scratch/.../figs/` — HPC 端 visualization staging
- `/scratch/.../xxx(数据名).ext3` — overlay数据集
- `/scratch/.../xxx(项目名)_env.ext3` — conda env overlay

## Working log

按 phase 维护工作日志:

- 每个 phase 一个独立的 markdown 文件
- 每做一步,把**遇到的问题**和**如何解决**追加进去
- 这是"我们试过什么、为什么"的真实记录 —— 边做边写,不要事后补