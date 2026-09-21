#!/usr/bin/env python3
"""Bounded compute submission; agents must exit/checkpoint instead of model polling."""
import argparse
from pathlib import Path
import time

import scheduler as s


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--stage', choices=list(s.AGENTS), required=True)
    p.add_argument('--script', type=Path, required=True)
    p.add_argument('--cpus', type=int, default=2)
    p.add_argument('--mem-gb', type=int, default=8)
    p.add_argument('--hours', type=int, default=1)
    args = p.parse_args()
    if not (1 <= args.cpus <= 4 and 1 <= args.mem_gb <= 32 and 1 <= args.hours <= 4):
        p.error('Envelope: 1..4 CPUs, 1..32 GiB, 1..4 hours')
    script = args.script.resolve()
    if not script.is_file() or not (script.is_relative_to(s.worktree(args.stage)) or
                                    script.is_relative_to(s.ROOT / 'compute')):
        p.error('Script must be in your worktree or runtime compute directory')
    if any(line.lstrip().startswith('#SBATCH') for line in script.read_text().splitlines()):
        p.error('Use a plain bash script; the helper owns all Slurm resource options')
    with s.lock('scheduler'):
        if (s.STATE / 'PAUSE').exists():
            p.error('Chain paused')
        rows = s.load(s.STATE / 'compute.json', [])
        live = s.queue()
        if len(rows) >= s.CONFIG['max_compute_total']:
            p.error('Total compute cap reached; report a blocker with required resources')
        if sum(row['job'] in live for row in rows) >= s.CONFIG['max_compute']:
            s.atomic(s.STATE / (args.stage + '.wait_compute.json'),
                     [row['job'] for row in rows if row['job'] in live])
            p.error('Compute concurrency cap reached; checkpoint .continue and exit')
        job = s.submit(script, '--account=torch_pr_527_general', '--partition=cpu_short',
                       '--nodes=1', '--ntasks=1', f'--cpus-per-task={args.cpus}',
                       f'--mem={args.mem_gb}G', f'--time={args.hours:02}:00:00',
                       '--job-name=gs3d_compute_' + args.stage,
                       '--output=' + str(s.LOGS / '%x-%j.out'),
                       '--error=' + str(s.LOGS / '%x-%j.err'),
                       '--chdir=' + str(s.worktree(args.stage) / 'gmc'),
                       kind='compute', stage=args.stage)
        rows.append({'job': job, 'stage': args.stage, 'script': str(script),
                     'cpus': args.cpus, 'mem_gb': args.mem_gb, 'hours': args.hours,
                     'submitted_at': time.time()})
        s.atomic(s.STATE / 'compute.json', rows)
        print(job)


if __name__ == '__main__':
    main()
