# Worklog — GMC M0-lite：K2 真实 GS 场景接入

角色（总控 v3 §双轨）：K2 是统计源 / H2 测量基底 / 常驻 demo，**不是** G1/G2
正确性裁决基底（裁决留在 synthetic + oracle）。

## 2026-08-24 接入与首轮处理

- 原始件：`data/gs_scenes/k2/raw/point_cloud.ply`（257 MB，396.6 万 splat，
  标准 3DGS 17-float 格式，SH 0 阶），sha256
  `66f45786beee86bc4778da8d9fa31a422d98a8156db6976dc5d4546ddf8dbb95`。
  来源：用户提供（Telegram，原名 `point_cloud_1 (3).ply`，header comment
  `source K2`）。
- 管线：`experiments/gmc/gs_k2_process.py` → `processed_full.npz`（清理后 3D）、
  `slab_2d.npz`（SE(2) 条件化 2D splats）、`meta.json`（全部参数与阶段计数）。

### 阶段计数

raw 3,966,460 → 垃圾过滤（op<0.05、尺度>1.0）3,878,752 → 裁剪到建筑主体
3,789,223 → floater 体素过滤 3,786,077 → **slab 2D splats 656,487**。

### 标定结论（记入 meta.json）

- 场景是大开间办公/实验室楼层，含隔断墙、家具岛、圆桌群；z 已重力对齐。
- 地板 z0=−1.09，天花板 4.08 → 层高 5.16 单位；墙体主方位 60.7°(mod 90)。
- **尺度假设 1 单位 ≈ 0.5 m，两个独立证据**：层高 5.16u→2.6 m；两个门洞开口
  实测 ≈2.0–2.1u→0.9–1.0 m（`figs/door_windows.png`）。仍标 UNVERIFIED，
  等真实参照一锤定音。
- 3D→SE(2)：机器人 slab [z0+0.1, z0+1.6]（≈0.05–0.8 m），每 splat 在 6 个高度
  取最大权重的条件化 2D Gaussian（μ、Σ 闭式条件分布），权重 = op·高度密度。

### 遇到的问题与解决

1. **地板检测 bug**：初版取"z<0 的直方图众数"，被 z≈−0.2 的家具/墙体质量抢峰
   （得 −0.17，视觉明明 −1.2）。修正为限制 z∈(−2,−0.5) 找峰 → −1.09，与侧视图
   一致。教训又是 README 第 2 条：先看图，才发现众数错了。
2. **floater 体素过滤几乎没削**（只删 ~3k）：0.5u 体素 + min 3 在 3.8M 密度下
   太宽松。开阔区稀疏噪点仍在，靠 slab 权重阈值压住了大半；正式版在 M2 语义里
   处理（或收紧到 min≥5 重跑对比）。
3. Slab 图（`figs/pipeline_stages.png` 右）验收：墙体清晰、门洞开口可见、
   家具岛/圆桌群保留——可直接当 H2 窗口基底。

### 已知问题（meta.json known_issues 同步）

- 墙上开口混有玻璃/扫描洞（false-free），墙缝审计 pass 未做——用此场景做
  planning 声明前必须完成；
- slab 语义是 pilot 级（K 高度取 max），认证版 slab 并集属于 M2；
- 低透明度累积实心问题（false-safe）留待 M0 校准曲线。

### 下一步

1. H2 sweep 接 `slab_2d.npz`：先在两个已确认门洞窗口（(22.3,−15.2)、
   (15.2,−11.0)，各 ~9k 2D splats）跑事件密度，再决定整层策略（HPC）；
2. 墙缝审计 pass（开口枚举 + 门楣质量分类）；
3. 机器人按门宽相对尺寸定型（细长 body，支撑意义宽 ≈0.6–0.7 门宽）。
