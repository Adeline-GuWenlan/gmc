"""Node-local SQLite state with atomic, resumable snapshots on shared storage."""
import os
from contextlib import closing
from pathlib import Path
import sqlite3
import tempfile
import time
import uuid


def snapshot_database(source, destination):
    """SQLite backup includes committed WAL data; never copy a live database file."""
    destination = Path(destination)
    temporary = destination.with_name(destination.name + '.' + uuid.uuid4().hex + '.tmp')
    deadline = time.monotonic() + 20

    def progress(*_):
        if time.monotonic() > deadline:
            raise TimeoutError('SQLite snapshot timed out: ' + str(source))

    try:
        with closing(sqlite3.connect(Path(source).resolve().as_uri() + '?mode=ro', uri=True)) as src:
            with closing(sqlite3.connect(temporary)) as dst:
                src.backup(dst, pages=256, progress=progress)
                dst.execute('PRAGMA journal_mode=DELETE')
        temporary.chmod(0o600)
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()


class SessionDatabase:
    def __init__(self, persistent):
        self.persistent = Path(persistent)
        self.persistent.mkdir(parents=True, exist_ok=True, mode=0o700)
        local_root = os.environ.get('SLURM_TMPDIR', '/tmp')
        if not Path(local_root).is_dir():
            local_root = '/tmp'
        self.temporary = tempfile.TemporaryDirectory(prefix='gs3d-sqlite-', dir=local_root)
        self.path = Path(self.temporary.name)
        for source in self.persistent.glob('*.sqlite'):
            snapshot_database(source, self.path / source.name)

    def checkpoint(self):
        for source in self.path.glob('*.sqlite'):
            snapshot_database(source, self.persistent / source.name)

    def close(self):
        self.checkpoint()
        self.temporary.cleanup()
