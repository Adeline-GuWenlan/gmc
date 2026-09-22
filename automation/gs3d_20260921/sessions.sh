#!/bin/bash
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
import shlex
root = Path('/scratch/wg2381/codex_jobs/gs3d_20260921')
for path in sorted((root / 'state').glob('A*.session')):
    stage = path.stem
    print(stage, '->', path.read_text().strip())
    print('  python3 ' + shlex.quote(str(root / 'resume_agent.py')) + ' ' + stage)
print('Do not resume a stage while its scheduled job is live.')
PY
