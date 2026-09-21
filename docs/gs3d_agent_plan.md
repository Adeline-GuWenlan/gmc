# GS3D: independent agent plan — 2026-09-21

## Authority and starting point

User request: build and launch independent agents (not subagents), save tokens where practical,
use parallel work only when logically independent, resume automatically after the rolling 5-hour
quota refresh, and push completed work to GitHub. User confirmed **start now**.

Repository: `git@github.com:Adeline-GuWenlan/gmc.git`.
Exact base: `9ff0d5c2a59c95a4444e45878ef878c8d9837108`, verified as the remote
`height-planefloor` tip on 2026-09-21. The interactive checkout is on
`k2-real-scene-test-a`; do not change it or the original `splathjb-plane` checkout.
All work goes into `codex/gs3d-20260921/*` branches and separate worktrees here.

This is a **major algorithm change**, not a visualization adjustment. Earlier height-band task
restrictions such as fixed UAV height, frozen implementation modules, no push, or no core edits
do not override this new explicit user request. Retain their useful evidence and safety semantics.

## User requirements / 用户要求

1. 在原始三维 Gaussian 上做规划、碰撞和 clearance；不得先将障碍压扁到二维。
2. 无人机代码、状态、轨迹、碰撞检测和可视化全部支持真实高度变化。
3. 扫地机器人 sweeper 和现有 cylinder（用户所指的注晶机器人）也使用三维场景、实体
   几何和碰撞验证。保留真实的地面支撑与运动限制，不把地面机器人变成会飞的点。
4. 在现有 GS 展示厅加入吊灯/吊顶等空中三维障碍；展示无人机先从吊灯下面穿过，再升高
   飞越桌面。先构建场景，再测试；保留原始场景和编辑记录。
5. cylinder 长距离失败需要诊断：终点 unknown 是假设；同时排查搜索预算、连通性与开销。
   允许在原终点附近的已知安全区域到达；明确容差，保留原目标，不允许穿过障碍或把
   unknown 直接当作 free。其他方法的原目标不能偷偷改动。
6. 从真正算法开始运行的阶段测量时间，同时记录各阶段开销、每次重规划的耗时及
   trajectory/control 的 dt。不得将渲染帧率、物理 dt 或排队时间当成算法耗时。
7. 先完整跑通三维 GS，再优化 sharp turns，输出平滑且重新验证安全的轨迹。
8. 研究相关工作，尤其 NeuPAN；形成可实施的设计。完整验收、证据、Git 历史和 GitHub 推送。

## Confirmed baseline facts

- `gmc/src/gmc/height/project.py:project_scene` makes `SceneModel2D` after height-band filtering.
- `height/run.py:compile_and_query`, `height/pathio.py` and core witnesses use `Pose2`.
- `height/prism.py:robot_table(z_c=1.20)` fixes UAV band at `z_c +/- 0.10 m`;
  `height/replay3d.py` replays that fixed band. It is not a variable-altitude planner.
- Existing sweepers and UAVs succeeded over 11.24 m; cylinder succeeded to 5.45 m, but cases
  with at least 73,178 supports exhausted the query budget. An unknown endpoint alone has not
  been established as the cause. See `gmc/results/height/plane/long/cyl_ladder/` and
  `docs/worklog/height_planefloor.md`.
- Base test result was 576 pass / 25 fail; prior logs attribute the 25 failures to unavailable
  sealed Atlas assets. Reproduce/classify rather than automatically exempting any new failure.
- Existing scene has manually replaced floor geometry. Results apply to that edited map, not
  a guarantee about the physical room. Keep opacity/support-level settings explicit (`tau=0.3`,
  `level=2.0` as baseline) and do not quietly remove obstacles to obtain a route.
- Scene loader is `gmc/experiments/showcase_scene.py`; decoded data is already available at
  `/scratch/wg2381/splathjb/splatc_atlas/data/gs_scenes/showcase/processed.npz` (782 MiB).
  The loader's `DATA` already points there. Original PLY and this decoded dataset are read-only.
  Worktree-local `splatc_atlas/data` is absent by design; avoid redundant dataset copies.
- Python: `/scratch/wg2381/.conda/envs/gmc-venv/bin/python`; from each worktree's `gmc/`,
  use `PYTHONPATH=src:experiments`, `MPLBACKEND=Agg`. Inspect asset paths before running.

## Agent roster and dependency graph

All eight agents are independent `codex exec` processes with independent persistent session IDs.
No agent may spawn subagents or launch other model conversations. Maximum two agents concurrently.
Models were verified with the installed Codex app-server `model/list`; effort is chosen per task.

| Agent | Model / effort | Depends on | Responsibility and handoff |
|---|---|---|---|
| A0 — architecture | GPT-6-Astra / high | exact base + this plan | Audit baseline, read NeuPAN/Splat-Nav, decide state/geometry/search design, freeze minimal interfaces and acceptance fixtures. |
| A1 — 3D core | GPT-6-Astra / xhigh | A0 | Direct 3D Gaussian collision/clearance, body volume and swept-edge verification, scalable search, synthetic 3D smoke evidence. |
| A2 — scene | GPT-5.6-Terra / high | A0 | Reproducible hanging light/ceiling + table edit of existing GS scene, visual inspection, manifests and known feasible under-then-over fixture. |
| A3 — robots/goals | GPT-5.6-Sol / high | A1 | Variable-z UAV; 3D swept bodies for ground robots; bounded goal-region search; cylinder diagnosis and long-distance cases. |
| A4 — timing | GPT-5.6-Terra / medium | A1 | Timing API/instrumentation and reproducible benchmark harness, separate cold/warm timings and physical dt. |
| A5 — 3D integration | GPT-5.6-Sol / high | A2, A3, A4 | Merge interfaces, demonstrate all robots in existing GS environment, numeric altitude and collision evidence, inspect rendered frames. |
| A6 — smooth turns | GPT-5.6-Sol / high | A5 | Constrained smooth trajectory optimization with body/kinematic constraints and complete post-smoothing verification. |
| A7 — final review | GPT-6-Astra / high | A6 | Independent requirements audit, fix omissions, reproduce final metrics/visuals, commit report; runner publishes branches only after acceptance. |

```
A0 -> A1 -> A3 ----+
 |      \-> A4 ---+|
 \-> A2 ---------+++-> A5 -> A6 -> A7 -> GitHub publication
```

A1/A2 own different files. A3/A4 own different files. A4 should expose instrumentation helpers
and a benchmark runner rather than editing A3's entrypoint concurrently; A5 integrates hooks.
Parallelism is a throughput choice, not a claim to save tokens. Reuse handoffs and pinned sessions;
avoid repeated broad searches, whole-repo dumps, and repeated tests without a new reason.

## Architectural boundaries A0 must resolve

- Keep full Gaussian means and 3x3 covariances in the planning/collision representation. A 3D
  spatial index, conservative bounding volumes, or search lattice can accelerate queries;
  neither 2D shadow geometry nor a point-only surrogate may become the collision authority.
- UAV may use translation (x,y,z) plus yaw if justified; do not demand 6-DOF roll/pitch planning
  unless needed. Ground bodies operate on their support manifold inside the same 3D environment.
  Document which dynamics/attitude assumptions the safety evidence covers.
- Define known/free/occupied/unknown explicitly. A Gaussian map alone need not establish observed
  free space everywhere. Distinguish search UNKNOWN/budget exhaustion from unobserved map space.
- Define robust body–ellipsoid clearance and swept-segment validation. Tests must catch between-
  waypoint collisions, rotated anisotropic Gaussians, grazing/tangent cases, vertical motion,
  ceiling/floor bounds, and non-finite/degenerate input. Never call sampled checks certificates.
- Freeze result/trajectory schema (xyz, yaw as applicable, timestamps or dt, original goal,
  attained goal, tolerance, safety status, diagnostics, seeds, timing, scene identity).
- Choose a bounded scalable search strategy from evidence; preserve the old 2D pipeline as an
  explicitly named comparison. Do not force the old SE(2) compiler into unbounded 3D enumeration.
- Goal-region default proposal: 0.25 m, sensitivity at 0 / 0.1 / 0.25 / 0.5 m; maximum 0.5 m
  without new user instructions. Validate finite nonnegative tolerances and yaw tolerance separately.
  Goal selection must be based on reachable safe candidates and include endpoint occupancy evidence.
- Freeze altitude swing and path/obstacle margins *before* the showcase run. A defensible initial
  demo target is >=0.5 m altitude range with an ordered low segment below the light then high
  segment above a table, using geometric body extents; select feasible dimensions first.
- A0 may adjust implementation choices and benchmark sizes with written rationale, not remove
  user requirements. Synthetic tests are a prerequisite; the existing GS scene demo remains required.

## Research seed references (read the primary sources)

- NeuPAN paper: https://arxiv.org/abs/2403.06828 ; authors' code: https://github.com/hanruihua/NeuPAN .
  It couples raw obstacle points to model-based control; assess its nonholonomic constraints,
  learned distance representation and applicability limits. It is not itself proof of direct 3D
  Gaussian UAV planning. No training project is required merely because the paper uses learning.
- Splat-Nav: https://arxiv.org/abs/2403.02751 ; authors' code: https://github.com/chengine/splatnav .
  It constructs Gaussian-aware collision corridors and smooth Bezier trajectories; assess the
  geometry and assumptions for this codebase. Record citations and licensing if adapting code.
- Codex automation: https://developers.openai.com/codex/noninteractive and
  https://developers.openai.com/codex/app-server . The scheduler uses `account/rateLimits/read`,
  `resetsAt`, JSONL `thread.started`, and explicit session-ID resume, not guessed fixed clock slots.

## Acceptance matrix (A7 must independently verify each item)

| ID | Pass requires |
|---|---|
| R1 | Production planning and validation use 3D Gaussian/body geometry; a test fails if projection is called. Two scenes with the same xy projection and distinct z geometry yield appropriately different routes. |
| R2 | UAV planned/exported/executed/replayed z actually changes; bounds, body geometry and vertical segment safety agree numerically and visually. |
| R3 | Sweeper and cylinder both run through the shared 3D scene/collision stack while satisfying ground support and their kinematic constraints; all three have successful reproducible runs. |
| R4 | Real existing GS scene plus deterministic airborne edits; ordered under-light then over-table UAV flight, frozen altitude/margin gates, same obstacles in planner and renderer; original scene untouched. |
| R5 | Original goal retained, exact/relaxed comparisons, tolerance obeyed, terminal pose safe; long-distance cylinder >=8 m start-to-original-goal attempt and successful safe route or further fixes. Report budget/map-unknown/connectivity separately. |
| R6 | Algorithm wall time from algorithm entry, separate stages, cold/warm prep, replan latency distribution, physical dt separately; metadata and seeds permit rerun, all three robots covered. |
| R7 | Smoothing runs after successful 3D integration; reduced turning/curvature metric, velocity/acceleration constraints as supported, continuous safety argument or explicitly limited validation, safe fallback to verified input. |
| R8 | Relevant regressions, classified full-suite results, rendered frames opened and reconciled with JSON, comprehensive evidence/limitations, clean commits, dependency history, verified GitHub branch refs. |

Do not mark the project complete when only a subset works. An infeasible scene is a reason to
repair the reproducible fixture within documented requirements, not to forge a success metric.
Diagnose genuine external blockers in the stage standup; leave a continuation or blocked state.

## Git and collaboration

- Runtime root: `/scratch/wg2381/codex_jobs/gs3d_20260921`.
- One stage, one worktree, one branch. Branch names: `codex/gs3d-20260921/A0` ... `/A7`.
- Each worktree starts from the accepted predecessor commit. Multi-parent stages merge the
  accepted predecessor commits using `git merge --no-ff`; resolve conflicts in that stage only.
  Never reset, force-push, rebase published history, or modify another stage's checkout.
- Commit cohesive changes with explicit file lists and messages `[gs3d A#] ...`; no fabricated
  model coauthor attribution. Commit stage reports into `docs/worklog/gs3d_A#.md`.
- State files and compact external standups carry handoffs; only the scheduler edits its own
  `state/scheduler.json`. Agents write `state/A#.progress.json`, `.continue`, `.done.json`.
- Artifacts, tests and acceptance evidence govern completion, not the process exit code.
- A7 receives all predecessor history. On accepted completion the runner pushes plan and agent
  branches plus `codex/gs3d-20260921/integration`, then verifies their remote object IDs. Main
  and the original height-planefloor branch are not moved. No PR merge is needed.
- Commit code/config/tests/docs and small evidence. Keep raw datasets, credentials, transcripts
  and large generated media out of Git; commit reproducible commands and artifact checksums.

## Scheduling, continuation and resource envelope

- Start immediately with A0. Successors are submitted only when prerequisites have accepted
  artifacts. The whole DAG is not prequeued. A small afterany recovery job accompanies active
  agents so even node failure/timeout can restart the dispatcher without spending model tokens.
- Each agent: 1 CPU, 4 GiB, 2 hours; the runner checkpoints at 100 minutes and resumes the same
  session. JSONL logs, session IDs and job IDs are persisted. No nested agents.
- Rate limits: read the live quota before an agent turn. At exhaustion/low headroom release the
  allocation and schedule `--begin` at the blocking quota reset + 30 seconds. Respect weekly
  limits too; do not consume earned reset credits, use reserve models, or buy credits.
- API unavailable: log the uncertainty and retry the metadata read in 15 minutes. A fallback
  interval is a retry delay, never a claimed reset time. There is no model call while gated.
- Failed/interrupted model turns resume the pinned thread, with bounded crash retries (12 per
  stage, 20 for A7). Quota waits and active compute do not spend crash retries. State explicitly
  records stalled/blocked situations. Total scheduling horizon: 14 days, then report incomplete.
- Heavy work uses `submit_compute.py`: max 2 compute jobs concurrently, 48 total; each <=4 CPU,
  <=32 GiB, <=4 hours on `cpu_short`, account `torch_pr_527_general`. Choose smaller requests
  when feasible. Default test job 2 CPU / 8 GiB / 1 hour. No GPU or package installs in this plan.
- Agents gate on their own and ancestors' outstanding compute ledgers before resuming. A waiting
  agent writes `.continue` with exact next steps and exits. No repeated model polling.
- Cancel only explicit IDs from this chain's ledgers. Existing job 18213611 belongs to the
  user's interactive allocation and is not part of this chain.

## Operational commands

`python3 scheduler.py status` shows stage, job and session IDs.
`bash sessions.sh` prints the per-stage interactive resume commands. Do not resume a live agent.
`python3 scheduler.py dispatch` safely reconciles and submits ready work.
Create `state/PAUSE` to prevent new agent/compute submissions (existing jobs keep running).
Remove it and run dispatch to continue. These commands are run from the runtime root.
The final summary will be `logs/A7_done.md`; publication proof is `state/publication.json`.
