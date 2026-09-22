#!/usr/bin/env python3
"""Manual continuation using the same isolated state as the scheduled agent."""
import fcntl
import json
import subprocess
import sys
from scheduler import STATE, AGENTS, worktree
from runtime_state import SessionDatabase

stage = sys.argv[1]
if stage not in AGENTS:
    raise SystemExit('Unknown stage')
with (STATE / (stage + '.lock')).open('a') as guard:
    try:
        fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Stage is running; wait for its scheduled job to finish.')
    session = (STATE / (stage + '.session')).read_text().strip()
    persistent = STATE / 'sqlite' / stage
    database = SessionDatabase(persistent) if persistent.exists() else None
    args = ['codex', '-C', str(worktree(stage))]
    if database:
        args += ['-c', 'sqlite_home=' + json.dumps(str(database.path))]
    try:
        result = subprocess.run(args + ['resume', session])
    finally:
        if database:
            database.close()
    raise SystemExit(result.returncode)
