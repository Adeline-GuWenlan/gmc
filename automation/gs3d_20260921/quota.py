"""Read quota metadata without model inference, secrets, or earned-reset consumption."""
import json
import os
import selectors
import shutil
import subprocess
import time


def read_quota(timeout=60):
    proc = subprocess.Popen([shutil.which('codex') or 'codex', 'app-server', '--stdio'],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, start_new_session=True)
    selector = selectors.DefaultSelector()
    selector.register(proc.stdout, selectors.EVENT_READ)

    def send(msg):
        proc.stdin.write((json.dumps(msg) + '\n').encode())
        proc.stdin.flush()

    try:
        send({'id': 1, 'method': 'initialize', 'params': {
            'clientInfo': {'name': 'gs3d_quota', 'version': '1.0'},
            'capabilities': {'experimentalApi': True}}})
        pending = b''
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not selector.select(1):
                if proc.poll() is not None:
                    break
                continue
            chunk = os.read(proc.stdout.fileno(), 65536)
            if not chunk:
                break
            pending += chunk
            while b'\n' in pending:
                line, pending = pending.split(b'\n', 1)
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue
                if msg.get('id') == 1:
                    if 'error' in msg:
                        raise RuntimeError('Codex app-server initialization failed')
                    send({'method': 'initialized'})
                    send({'id': 2, 'method': 'account/rateLimits/read'})
                elif msg.get('id') == 2:
                    if 'error' in msg:
                        raise RuntimeError('Codex quota read failed: ' +
                                           str(msg['error'].get('message', 'unknown'))[:300])
                    result = msg['result']
                    bucket = result.get('rateLimitsByLimitId', {}).get('codex')
                    bucket = bucket or result.get('rateLimits') or {}
                    # Deliberately omit account IDs, auth data and reset-credit identifiers.
                    return {'observed_at': time.time(),
                            'ordinary_usage_allowed': result.get('ordinaryUsageAllowed'),
                            'primary': bucket.get('primary'), 'secondary': bucket.get('secondary'),
                            'limit_reached': bucket.get('rateLimitReachedType'),
                            'spend_control_reached': bucket.get('spendControlReached')}
        raise RuntimeError('Codex quota metadata unavailable (timeout or app-server exit)')
    finally:
        selector.close()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


def next_allowed(snapshot, now=None):
    """Respect exhausted five-hour AND weekly windows; no fixed refresh-hour assumptions."""
    now = time.time() if now is None else now
    waits = []
    for key in ('primary', 'secondary'):
        window = snapshot.get(key) or {}
        if window.get('usedPercent', 0) >= 98:
            reset = window.get('resetsAt')
            waits.append(reset + 30 if reset and reset > now else now + 900)
    if (snapshot.get('ordinary_usage_allowed') is False or snapshot.get('limit_reached')
            or snapshot.get('spend_control_reached')) and not waits:
        waits.append(now + 900)
    # Missing both windows is uncertainty, not proof that quota is available.
    if not snapshot.get('primary') and not snapshot.get('secondary'):
        waits.append(now + 900)
    return max(waits, default=now)


if __name__ == '__main__':
    data = read_quota()
    data['next_allowed_epoch'] = next_allowed(data)
    print(json.dumps(data, indent=2))
