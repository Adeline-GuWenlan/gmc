#!/bin/bash
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
import shlex
root = Path('/scratch/wg2381/codex_jobs/gs3d_20260921')
for path in sorted((root / 'state').glob('A*.session')):
    stage = path.stem
    print(stage, '->', path.read_text().strip())
    print('  cd ' + shlex.quote(str(root / 'worktrees' / stage)) +
          ' && codex resume ' + shlex.quote(path.read_text().strip()))
print('Do not resume a stage while its scheduled job is live.')
PY
