#!/bin/bash
# Regenerate the concatenated v2 plan from the canonical numbered files.
cd "$(dirname "$0")/.." || exit 1
{ printf '# SplatC-Atlas 完整项目执行文档 v2.0\n\n> 本文件合并总控计划、相对时间表、baseline/公开数据、自建 benchmark 和旧计划冲突审计。\n>\n> **维护约定：** 编号文件 00–04 是 canonical 版本；本合订本由脚本拼接生成，不要直接编辑。重新生成：`bash tools/rebuild_v2.sh`。\n\n\n---\n\n'
  cat 00_MASTER_EXECUTION_PLAN.md; printf '\n\n---\n\n'
  cat 01_RELATIVE_TIMELINE_AND_GATES.md; printf '\n\n---\n\n'
  cat 02_BASELINES_AND_PUBLIC_DATASETS.md; printf '\n\n---\n\n'
  cat 03_SPLATC_BENCH_SPEC.md; printf '\n\n---\n\n'
  cat 04_OLD_PLAN_CONFLICT_AUDIT.md
} > SplatC_Atlas_Complete_Project_Plan_v2.md
echo "rebuilt SplatC_Atlas_Complete_Project_Plan_v2.md"
