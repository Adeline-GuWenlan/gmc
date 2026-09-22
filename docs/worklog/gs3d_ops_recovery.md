# GS3D operations recovery (2026-09-22)

The scene stage A2 produced its derivative, render evidence and clean commit
70ab219baa4cdfccf2c5c8606c778ce80fb2ec19. The scheduler interpreted runtime-relative
`worktrees/A2/...` and `logs/...` evidence as worktree-relative paths, rejected
valid files without reporting why, and exhausted 12 retries. The recovery adds
explicit path roots and actionable acceptance diagnostics while retaining clean
HEAD, branch, predecessor ancestry, tracked worklog and final R1–R8 checks.
A2 acceptance is revalidated from the existing artifacts; no planner result is
inferred from its hand-authored feasibility witness. A5 must still demonstrate
an actual computed path on this scene before smoothing and final review.

Earlier app-server startup errors reported failure to initialize SQLite state
under the shared Codex home. Quota reads now use temporary node-local SQLite
state. Each model stage uses separate node-local state, backed up through SQLite's
online backup API every minute and at normal shutdown, and restored on resume.
Backups include committed WAL pages and are atomically replaced. Auth and rollout
locations are unchanged. A hard node loss can lose up to one checkpoint interval
of SQLite metadata; rollout files remain in their usual shared location.

The operations branch starts at the original plan commit. A5 merges it alongside
accepted A2/A3/A4 history, and downstream acceptance requires its ancestry. Final
publication includes the operations branch. Existing accepted branches are not
rewritten. Long shared-filesystem checkouts get a bounded ten-minute timeout.

Validation: 23 offline unit tests passed, including runtime/worktree evidence
resolution, missing-file diagnostics, full acceptance with mixed path roots,
quota refresh waiting and active-WAL snapshot restore to a new local directory.
A live isolated app-server metadata read succeeded (ordinary usage allowed).
A0/A1/A2 acceptance was also rechecked against real artifacts and Git history.
The recovery installs only tested scripts and records original scheduler state
and the scheduler-generated A2 continuation marker before resuming dispatch.

Official configuration reference for sqlite_home:
https://developers.openai.com/codex/config-reference
Quota metadata API:
https://developers.openai.com/codex/app-server
