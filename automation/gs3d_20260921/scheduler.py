#!/usr/bin/env python3
"""Event-driven Slurm DAG for independent, resumable Codex sessions (stdlib only)."""
import argparse
from contextlib import contextmanager
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import re
import selectors
import signal
import subprocess
import sys
import time
import uuid

from quota import next_allowed, read_quota
from runtime_state import SessionDatabase

ROOT = Path(os.environ.get('GS3D_ROOT', Path(__file__).resolve().parent))
CONFIG = json.loads((ROOT / 'agents.json').read_text())
AGENTS = CONFIG['agents']
REPO = Path(CONFIG['repo'])
STATE = ROOT / 'state'
LOGS = ROOT / 'logs'


def load(path, default=None):
    return json.loads(path.read_text()) if path.exists() else default


def atomic(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    tmp.write_text(json.dumps(data, indent=2) + '\n')
    os.replace(tmp, path)


@contextmanager
def lock(name):
    STATE.mkdir(parents=True, exist_ok=True)
    with (STATE / (name + '.lock')).open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        yield


def cmd(args, cwd=None, check=True, timeout=90):
    result = subprocess.run([str(x) for x in args], cwd=cwd, capture_output=True,
                            text=True, timeout=timeout)
    if check and result.returncode:
        raise RuntimeError(f'{args[0]} rc={result.returncode}: {result.stderr[-1200:]}')
    return result


def git(*args, cwd=REPO, check=True):
    # A checkout on shared cluster storage can take several minutes.
    return cmd(['git', *args], cwd=cwd, check=check,
               timeout=600 if args and args[0] in ('worktree', 'merge') else 90)


def ledger(job, kind, stage=None):
    with (ROOT / 'jobids' / 'all.jsonl').open('a') as handle:
        handle.write(json.dumps({'job': job, 'kind': kind, 'stage': stage,
                                 'submitted_at': time.time()}) + '\n')


def submit(script, *options, args=(), kind='agent', stage=None):
    output = cmd(['sbatch', '--parsable', *options, script, *args]).stdout.strip()
    job = output.split(';')[0]
    if not job.isdigit():
        raise RuntimeError('Invalid sbatch job id: ' + output)
    ledger(job, kind, stage)
    return job


def queue():
    output = cmd(['squeue', '--me', '--noheader', '--format=%i|%j|%T'], timeout=30).stdout
    return {line.split('|')[0].strip(): line.strip().split('|')[1:]
            for line in output.splitlines() if line.count('|') == 2}


def machine():
    old = load(STATE / 'scheduler.json')
    if old:
        return old
    bootstrap = load(STATE / 'bootstrap.json')
    if not bootstrap:
        raise RuntimeError('Missing bootstrap.json: commit the plan before launch')
    return {'created_at': time.time(), 'plan_commit': bootstrap['commit'],
            'stages': {name: {'status': 'pending', 'crashes': 0, 'not_before': 0}
                       for name in AGENTS}, 'wakeups': []}


def save(data):
    atomic(STATE / 'scheduler.json', data)


def worktree(stage):
    return ROOT / 'worktrees' / stage


def ancestors(stage):
    found = {stage}
    for dep in AGENTS[stage]['deps']:
        found.update(ancestors(dep))
    return found


def compute_jobs(stage, live):
    rows = load(STATE / 'compute.json', [])
    own = [row['job'] for row in rows
           if row['stage'] in ancestors(stage) and row['job'] in live]
    capacity_wait = load(STATE / (stage + '.wait_compute.json'), [])
    return sorted(set(own + [job for job in capacity_wait if job in live]))


def evidence_path(stage, filename):
    path = Path(filename)
    if path.is_absolute():
        return path
    if '..' in path.parts:
        raise ValueError('Evidence paths must not contain parent traversal')
    root = ROOT if path.parts and path.parts[0] in ('worktrees', 'logs', 'state') else worktree(stage)
    return root / path


def accepted(stage, data, errors=None):
    """A zero process status is insufficient. Require pinned commits and evidence files."""
    wt = worktree(stage)
    def reject(reason):
        if errors is not None:
            errors.append(reason)
        return None
    path = STATE / (stage + '.done.json')
    if not path.exists() or (STATE / (stage + '.continue')).exists():
        return reject('Missing done manifest or an unfinished .continue marker exists')
    try:
        item = load(path)
        if item.get('accepted') is not True or not item.get('checks') or not item.get('artifacts'):
            return reject('Manifest needs accepted=true, nonempty checks and artifacts')
        if any(c.get('passed') is not True or not c.get('evidence') for c in item['checks']):
            return reject('Every check needs passed=true and an evidence file')
        files = item['artifacts'] + [c['evidence'] for c in item['checks']]
        for filename in files:
            fp = evidence_path(stage, filename)
            if not fp.is_file():
                return reject('Evidence file missing: ' + str(fp))
        report = LOGS / (stage + '_done.md')
        if not report.is_file() or not report.stat().st_size:
            return reject('Missing or empty external stage handoff report')
        head = git('rev-parse', 'HEAD', cwd=wt).stdout.strip()
        if item.get('commit') != head or git('status', '--porcelain', cwd=wt).stdout.strip():
            return reject('Manifest commit differs from HEAD or worktree is dirty')
        if git('branch', '--show-current', cwd=wt).stdout.strip() != CONFIG['branch_prefix'] + '/' + stage:
            return reject('Worktree is on the wrong branch')
        if git('ls-files', '--error-unmatch', 'docs/worklog/gs3d_' + stage + '.md',
               cwd=wt, check=False).returncode:
            return reject('Stage worklog is not tracked by Git')
        required = [CONFIG['base_commit'], data['plan_commit']]
        required += [data['stages'][d]['commit'] for d in AGENTS[stage]['deps']]
        if stage in ('A5', 'A6', 'A7') and data.get('ops_commit'):
            required.append(data['ops_commit'])
        if any(git('merge-base', '--is-ancestor', rev, head, cwd=wt, check=False).returncode
               for rev in required):
            return reject('Missing required base, plan, predecessor or operations ancestry')
        if stage == 'A7' and any(item.get('requirements', {}).get(f'R{i}') is not True
                                 for i in range(1, 9)):
            return reject('Final review must accept requirements R1 through R8')
        return item
    except (OSError, ValueError, TypeError, KeyError, RuntimeError) as exc:
        return reject(str(exc))


def wake(data, *, at=None, dependencies=()):
    signature = json.dumps([int(at or 0), sorted(dependencies)])
    if any(w['signature'] == signature for w in data['wakeups']):
        return
    options = []
    if at:
        options.append('--begin=' + datetime.fromtimestamp(at).strftime('%Y-%m-%dT%H:%M:%S'))
    if dependencies:
        options.append('--dependency=afterany:' + ':'.join(sorted(set(dependencies))))
    job = submit(ROOT / 'dispatch.slurm', *options, kind='dispatcher')
    data['wakeups'].append({'job': job, 'signature': signature})


def dispatch():
    with lock('scheduler'):
        data = machine()
        if (STATE / 'PAUSE').exists():
            print('Paused by state/PAUSE; no submissions.')
            return
        if time.time() - data['created_at'] > CONFIG['horizon_days'] * 86400:
            data['stopped_reason'] = '14-day scheduling horizon reached; incomplete'
            save(data)
            return
        live = queue()  # Fail closed on scheduler/network errors, never infer no live jobs.
        current = os.environ.get('SLURM_JOB_ID')
        live.pop(current, None)
        data['wakeups'] = [w for w in data['wakeups'] if w['job'] in live]
        for stage, st in data['stages'].items():
            if st['status'] in ('done', 'blocked'):
                continue
            active = [j for j, (name, _) in live.items() if name == 'gs3d_' + stage]
            if active:
                st.update(status='queued', job=active[0])
                continue
            if st['status'] in ('queued', 'running'):
                st.update(status='pending', job=None)
                st['crashes'] += 1
                st['last_reason'] = 'Agent job ended without normal handoff; recovering artifacts'
            if compute_jobs(stage, live):
                continue
            proof = accepted(stage, data)
            if proof:
                st.update(status='done', commit=proof['commit'], completed_at=time.time())
            elif st['crashes'] >= (20 if stage == 'A7' else 12):
                st.update(status='blocked', last_reason='Bounded crash retry limit reached')

        slots = CONFIG['max_agents'] - sum(st['status'] in ('queued', 'running')
                                          for st in data['stages'].values())
        waiting_times = []
        waiting_compute = []
        for stage, spec in AGENTS.items():
            st = data['stages'][stage]
            if st['status'] != 'pending':
                continue
            if not all(data['stages'][d]['status'] == 'done' for d in spec['deps']):
                continue
            remaining = compute_jobs(stage, live)
            if remaining:
                waiting_compute.extend(remaining)
                continue
            not_before = max(st.get('not_before', 0), data.get('quota_not_before', 0))
            if not_before > time.time():
                waiting_times.append(not_before)
                continue
            if slots <= 0:
                continue
            job = submit(ROOT / (stage + '.slurm'), stage=stage)
            st.update(status='queued', job=job)
            save(data)  # Recovery can rediscover submitted jobs by unique stage job name.
            recovery = submit(ROOT / 'dispatch.slurm', '--dependency=afterany:' + job,
                              kind='recovery', stage=stage)
            st['recovery_job'] = recovery
            slots -= 1
            print(stage, 'submitted', job, 'recovery', recovery, flush=True)
        if waiting_times:
            wake(data, at=min(waiting_times))
        if waiting_compute:
            # One afterany callback per compute job wakes whichever stage becomes ready first.
            for job in sorted(set(waiting_compute)):
                wake(data, dependencies=[job])
        save(data)
        complete = all(st['status'] == 'done' for st in data['stages'].values())
    if complete:
        try:
            publish()
        except Exception as exc:
            with lock('scheduler'):
                data = machine()
                data['publication_errors'] = data.get('publication_errors', 0) + 1
                data['publication_error'] = str(exc)[-1500:]
                if data['publication_errors'] <= 12:
                    wake(data, at=time.time() + 900)
                else:
                    data['stopped_reason'] = 'Publication failed repeatedly; branch work is accepted, push incomplete'
                save(data)
            print('Publication retry needed:', exc, file=sys.stderr, flush=True)


def prepare(stage, data):
    wt = worktree(stage)
    deps = AGENTS[stage]['deps']
    initial = data['stages'][deps[0]]['commit'] if deps else data['plan_commit']
    branch = CONFIG['branch_prefix'] + '/' + stage
    with lock('git'):
        if not wt.exists():
            git('worktree', 'add', '-b', branch, str(wt), initial)
        merges = [(dep, data['stages'][dep]['commit']) for dep in deps]
        if stage in ('A5', 'A6', 'A7') and data.get('ops_commit'):
            merges.append(('operations-repair', data['ops_commit']))
        for dep, sha in merges:
            if git('merge-base', '--is-ancestor', sha, 'HEAD', cwd=wt, check=False).returncode == 0:
                continue
            merge_head = git('rev-parse', '--git-path', 'MERGE_HEAD', cwd=wt).stdout.strip()
            if (wt / merge_head).exists():
                break
            result = git('merge', '--no-ff', '-m', f'[gs3d {stage}] Merge accepted {dep}',
                         sha, cwd=wt, check=False)
            if result.returncode:
                (LOGS / (stage + '_merge.txt')).write_text(result.stdout + result.stderr)
                break  # The stage agent resolves it; acceptance requires every parent ancestor.
    return wt


def set_stage(stage, **values):
    with lock('scheduler'):
        data = machine()
        data['stages'][stage].update(values)
        save(data)


def defer_quota(stage, snapshot=None, reason=None):
    now = time.time()
    until = next_allowed(snapshot, now) if snapshot else now + 900
    # If a model explicitly hit quota but the metadata has not caught up, back off, not spin.
    if until <= now:
        until = now + 900
    with lock('scheduler'):
        data = machine()
        data['quota_not_before'] = max(until, data.get('quota_not_before', 0))
        data['stages'][stage].update(status='pending', not_before=until, job=None,
                                     last_reason=reason or 'Quota refresh wait')
        if snapshot:
            atomic(STATE / 'quota.json', snapshot)
        save(data)
    print(stage, 'deferred until', datetime.fromtimestamp(until).isoformat(), reason, flush=True)


def fresh_quota(force=False):
    with lock('quota'):
        cached = load(STATE / 'quota.json')
        if not force and cached and time.time() - cached['observed_at'] < 60:
            return cached
        result = read_quota()
        atomic(STATE / 'quota.json', result)
        return result


def model_turn(stage, wt):
    spec = AGENTS[stage]
    database = SessionDatabase(STATE / 'sqlite' / stage)
    session_file = STATE / (stage + '.session')
    session = session_file.read_text().strip() if session_file.exists() else None
    job = os.environ.get('SLURM_JOB_ID', 'manual')
    log_path = LOGS / f'{stage}_{job}.jsonl'
    common_git = git('rev-parse', '--git-common-dir', cwd=wt).stdout.strip()
    args = ['codex', '--search', '--approve-for-me', '-C', str(wt),
            '--add-dir', str(ROOT), '--add-dir', str((wt / common_git).resolve()),
            '-m', spec['model'], '-c', 'model_reasoning_effort=' + json.dumps(spec['effort']),
            '-c', 'sandbox_workspace_write.network_access=true',
            '-c', 'sqlite_home=' + json.dumps(str(database.path)),
            '-c', 'features.multi_agent=false', 'exec']
    if session:
        args += ['resume', session]
    args += ['--json', '-o', str(LOGS / f'{stage}_last_message.md'), '-']
    if session:
        prompt = (ROOT / 'prompts/_resume.md').read_text()
    else:
        prompt = (ROOT / 'prompts/_rules.md').read_text() + '\n' + (ROOT / 'prompts' / (stage + '.md')).read_text()
    prompt += (f'\nRuntime root: {ROOT}\nYour stage: {stage}\nYour worktree: {wt}\n'
               f'Authoritative plan: {ROOT / "PLAN.md"}\n'
               'The user requested start now, automatically resume at actual quota reset, and '
               'push completed work to GitHub. You are authorized to perform this stage.\n')
    rejection = load(STATE / (stage + '.acceptance_rejection.json'))
    if rejection:
        prompt += '\nSCHEDULER ACCEPTANCE REJECTION: ' + json.dumps(rejection) + '\nFix these concrete errors; a success message alone cannot pass acceptance.\n'
    env = os.environ.copy()
    env.update(GS3D_ROOT=str(ROOT), GS3D_STAGE=stage, GS3D_WORKTREE=str(wt),
               MPLBACKEND='Agg', PYTHONUNBUFFERED='1')
    interrupted = [False]
    previous = {}
    for signum in (signal.SIGTERM, signal.SIGINT, signal.SIGUSR1):
        previous[signum] = signal.signal(signum, lambda *_: interrupted.__setitem__(0, True))
    tail = []
    started = time.monotonic()
    checkpoint_at = started + 60
    proc = subprocess.Popen(args, cwd=wt, env=env, stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            start_new_session=True)
    proc.stdin.write(prompt.encode())
    proc.stdin.close()
    selector = selectors.DefaultSelector()
    selector.register(proc.stdout, selectors.EVENT_READ)
    buffer = b''
    stop_time = None

    def consume(line, stream):
        nonlocal session
        stream.write(line + b'\n')
        stream.flush()
        text = line.decode(errors='replace')
        tail.append(text)
        del tail[:-80]
        try:
            event = json.loads(text)
        except ValueError:
            return
        if event.get('type') == 'thread.started' and event.get('thread_id'):
            sid = event['thread_id']
            if session and sid != session:
                raise RuntimeError('Codex resumed unexpected thread; stop to protect continuity')
            session = sid
            session_file.write_text(sid + '\n')
            print(f'{stage} pinned session {sid}', flush=True)

    try:
        with log_path.open('ab') as stream:
            while True:
                if time.monotonic() >= checkpoint_at:
                    database.checkpoint()
                    checkpoint_at = time.monotonic() + 60
                if (interrupted[0] or time.monotonic() - started >= 6000) and stop_time is None:
                    stop_time = time.monotonic()
                    os.killpg(proc.pid, signal.SIGTERM)
                if stop_time and time.monotonic() - stop_time > 20 and proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGKILL)
                if selector.select(1):
                    chunk = os.read(proc.stdout.fileno(), 65536)
                    if not chunk:
                        break
                    buffer += chunk
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        consume(line, stream)
                elif proc.poll() is not None:
                    break
            if buffer:
                consume(buffer, stream)
        rc = proc.wait(timeout=30)
        return rc, '\n'.join(tail), stop_time is not None
    finally:
        selector.close()
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        database.close()


def run_stage(stage):
    with lock(stage):
        with lock('scheduler'):
            data = machine()
            st = data['stages'][stage]
            if st['status'] in ('done', 'blocked'):
                return
            if not all(data['stages'][d]['status'] == 'done' for d in AGENTS[stage]['deps']):
                raise RuntimeError('Stage prerequisites not accepted')
            st.update(status='running', job=os.environ.get('SLURM_JOB_ID'), started_at=time.time())
            save(data)
        if (STATE / 'PAUSE').exists():
            set_stage(stage, status='pending', job=None, last_reason='Paused')
            return
        live = queue()
        if compute_jobs(stage, live):
            set_stage(stage, status='pending', job=None, last_reason='Waiting for owned compute')
            return
        proof = accepted(stage, data)
        if proof:
            set_stage(stage, status='done', commit=proof['commit'], completed_at=time.time())
            return
        try:
            snapshot = fresh_quota()
        except Exception as exc:
            defer_quota(stage, reason=str(exc))
            return
        if next_allowed(snapshot) > time.time() + 1:
            defer_quota(stage, snapshot)
            return
        wt = prepare(stage, data)
        # A stale continuation must be cleared before this turn; context remains in progress/logs.
        continuation = STATE / (stage + '.continue')
        if continuation.exists():
            os.replace(continuation, STATE / (stage + '.previous_continue'))
        errors = []
        accepted(stage, data, errors)
        if errors and (STATE / (stage + '.done.json')).exists():
            atomic(STATE / (stage + '.acceptance_rejection.json'), errors)
        rc, tail, timed_out = model_turn(stage, wt)
        data = machine()
        live = queue()
        errors = []
        proof = accepted(stage, data, errors) if not compute_jobs(stage, live) else None
        if errors and (STATE / (stage + '.done.json')).exists():
            atomic(STATE / (stage + '.acceptance_rejection.json'), errors)
        if proof:
            set_stage(stage, status='done', commit=proof['commit'], completed_at=time.time(), job=None)
        elif (STATE / (stage + '.blocked')).exists():
            set_stage(stage, status='blocked', job=None,
                      last_reason=(STATE / (stage + '.blocked')).read_text()[:1500])
        elif rc != 0 and re.search(r'usage.limit|rate.limit|quota.exceeded|usage_limit|rate_limit', tail, re.I):
            try:
                snapshot = fresh_quota(force=True)
            except Exception:
                snapshot = None
            defer_quota(stage, snapshot, 'Model reported quota limit; same thread will resume')
        else:
            deliberate = continuation.exists() or bool(compute_jobs(stage, live)) or timed_out
            crashes = data['stages'][stage]['crashes'] + (0 if deliberate else 1)
            if not continuation.exists():
                continuation.write_text('Continue from saved thread/worktree. Inspect evidence and '
                                        f'finish acceptance; previous process rc={rc}, timeout={timed_out}.\n')
            set_stage(stage, status='pending', job=None, crashes=crashes,
                      not_before=time.time() + (5 if deliberate else 120),
                      last_reason='Checkpoint continuation' if deliberate else 'Missing valid acceptance artifacts')


def publish():
    with lock('publish'):
        if load(STATE / 'publication.json', {}).get('verified'):
            return
        data = machine()
        if not all(st['status'] == 'done' for st in data['stages'].values()):
            return
        refs = {CONFIG['branch_prefix'] + '/plan': data['plan_commit']}
        if data.get('ops_commit'):
            refs[CONFIG['branch_prefix'] + '/ops'] = data['ops_commit']
        refs.update({CONFIG['branch_prefix'] + '/' + name: st['commit']
                     for name, st in data['stages'].items()})
        refs[CONFIG['branch_prefix'] + '/integration'] = data['stages']['A7']['commit']
        git('push', '--atomic', 'origin', *[sha + ':refs/heads/' + ref for ref, sha in refs.items()])
        remote = git('ls-remote', 'origin', *['refs/heads/' + ref for ref in refs]).stdout
        found = {line.split()[1]: line.split()[0] for line in remote.splitlines()}
        if any(found.get('refs/heads/' + ref) != sha for ref, sha in refs.items()):
            raise RuntimeError('GitHub ref verification failed')
        atomic(STATE / 'publication.json', {'verified': True, 'at': time.time(), 'refs': refs,
                                          'remote': 'https://github.com/Adeline-GuWenlan/gmc'})
        print('All stages accepted and GitHub refs verified.', flush=True)


def status():
    data = machine()
    for name, spec in AGENTS.items():
        st = data['stages'][name]
        sf = STATE / (name + '.session')
        print(name, spec['name'], spec['model'], spec['effort'], st['status'],
              'job=' + str(st.get('job')), 'session=' + (sf.read_text().strip() if sf.exists() else '-'))
        if st.get('last_reason'):
            print(' ', st['last_reason'])
    print('publication:', load(STATE / 'publication.json', {'verified': False}))
    if data.get('stopped_reason'):
        print('STOPPED:', data['stopped_reason'])
    if data.get('publication_error'):
        print('publication error:', data['publication_error'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['dispatch', 'run', 'status', 'publish'])
    parser.add_argument('stage', choices=list(AGENTS), nargs='?')
    args = parser.parse_args()
    if args.action == 'run':
        if not args.stage:
            parser.error('run needs a stage')
        try:
            run_stage(args.stage)
        except Exception as exc:
            with lock('scheduler'):
                data = machine()
                st = data['stages'][args.stage]
                st.update(status='pending', job=None, not_before=time.time() + 900,
                          crashes=st['crashes'] + 1, last_reason=str(exc)[-1500:])
                save(data)
            print('Stage error:', exc, file=sys.stderr, flush=True)
        dispatch()
    elif args.action == 'dispatch':
        dispatch()
    elif args.action == 'publish':
        publish()
    else:
        status()


if __name__ == '__main__':
    main()
