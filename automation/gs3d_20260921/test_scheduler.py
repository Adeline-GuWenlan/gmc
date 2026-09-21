"""Scheduler invariant tests; all Slurm, model and Git network calls are mocked."""
import copy
import json
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch

import quota
import scheduler as s


class QuotaTests(unittest.TestCase):
    def test_available_does_not_wait_for_refresh(self):
        self.assertEqual(quota.next_allowed({'primary': {'usedPercent': 20, 'resetsAt': 900}}, 100), 100)

    def test_rolling_reset_not_fixed_five_hours(self):
        self.assertEqual(quota.next_allowed({'primary': {'usedPercent': 100, 'resetsAt': 777}}, 100), 807)

    def test_weekly_exhaustion_wins(self):
        snap = {'primary': {'usedPercent': 100, 'resetsAt': 700},
                'secondary': {'usedPercent': 100, 'resetsAt': 9000}}
        self.assertEqual(quota.next_allowed(snap, 100), 9030)

    def test_unknown_is_retry_not_invented_refresh(self):
        self.assertEqual(quota.next_allowed({}, 100), 1000)

    def test_expired_exhausted_snapshot_does_not_spin(self):
        self.assertEqual(quota.next_allowed({'primary': {'usedPercent': 100, 'resetsAt': 99}}, 100), 1000)

    def test_server_denial_wins_over_low_usage(self):
        self.assertEqual(quota.next_allowed({'primary': {'usedPercent': 20},
                                            'ordinary_usage_allowed': False}, 100), 1000)


class SchedulerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        self.patcher = patch.multiple(s, ROOT=root, STATE=root / 'state', LOGS=root / 'logs')
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        for part in ['state', 'logs', 'jobids', 'worktrees']:
            (root / part).mkdir()
        self.data = {'created_at': time.time(), 'plan_commit': 'plan-sha', 'wakeups': [],
                     'stages': {n: {'status': 'pending', 'crashes': 0, 'not_before': 0}
                                for n in s.AGENTS}}
        self.jobs = []
        self.mock_submit = patch.object(s, 'submit', side_effect=self.submit).start()
        self.mock_queue = patch.object(s, 'queue', return_value={}).start()
        self.addCleanup(patch.stopall)

    def submit(self, script, *options, **kwargs):
        self.jobs.append((Path(script).name, options, kwargs))
        return str(1000 + len(self.jobs))

    def run_dispatch(self):
        s.save(self.data)
        s.dispatch()
        self.data = s.machine()

    def done(self, *stages):
        for stage in stages:
            self.data['stages'][stage].update(status='done', commit=stage + '-sha')

    def test_only_first_stage_is_submitted(self):
        self.run_dispatch()
        self.assertEqual([j[0] for j in self.jobs], ['A0.slurm', 'dispatch.slurm'])
        self.assertEqual(self.jobs[1][1], ('--dependency=afterany:1001',))

    def test_core_and_scene_are_independent_parallel_work(self):
        self.done('A0')
        self.run_dispatch()
        self.assertEqual([j[0] for j in self.jobs if j[2].get('stage') in ('A1', 'A2')
                          and j[2].get('kind') != 'recovery'], ['A1.slurm', 'A2.slurm'])
        self.assertEqual(sum(st['status'] == 'queued' for st in self.data['stages'].values()), 2)

    def test_join_requires_every_predecessor(self):
        self.done('A0', 'A1', 'A2', 'A3')
        self.run_dispatch()
        self.assertNotIn('A5.slurm', [j[0] for j in self.jobs])
        self.assertIn('A4.slurm', [j[0] for j in self.jobs])

    def test_join_submits_once_after_all_parents(self):
        self.done('A0', 'A1', 'A2', 'A3', 'A4')
        self.run_dispatch()
        self.assertEqual([j[0] for j in self.jobs], ['A5.slurm', 'dispatch.slurm'])
        self.mock_queue.return_value = {'1001': ['gs3d_A5', 'RUNNING'], '1002': ['gs3d_dispatch', 'PENDING']}
        s.dispatch()
        self.assertEqual(len(self.jobs), 2)

    def test_quota_wait_queues_dispatcher_not_model(self):
        self.data['quota_not_before'] = time.time() + 1800
        self.run_dispatch()
        self.assertEqual([j[0] for j in self.jobs], ['dispatch.slurm'])
        self.assertTrue(self.jobs[0][1][0].startswith('--begin='))

    def test_own_compute_blocks_model_and_uses_afterany(self):
        s.atomic(s.STATE / 'compute.json', [{'job': '555', 'stage': 'A0'}])
        self.mock_queue.return_value = {'555': ['gs3d_compute_A0', 'RUNNING']}
        self.run_dispatch()
        self.assertEqual([j[0] for j in self.jobs], ['dispatch.slurm'])
        self.assertEqual(self.jobs[0][1], ('--dependency=afterany:555',))

    def test_compute_capacity_wait_does_not_spend_model_tokens(self):
        s.atomic(s.STATE / 'A0.wait_compute.json', ['777'])
        self.mock_queue.return_value = {'777': ['gs3d_compute_A2', 'RUNNING']}
        self.run_dispatch()
        self.assertEqual(self.jobs[0][1], ('--dependency=afterany:777',))

    def test_lost_job_resumes_and_increments_crash_budget(self):
        self.data['stages']['A0'].update(status='running', job='missing')
        self.run_dispatch()
        self.assertEqual(self.data['stages']['A0']['crashes'], 1)
        self.assertEqual(self.jobs[0][0], 'A0.slurm')

    def test_crash_limit_blocks_without_unlocking_children(self):
        self.data['stages']['A0'].update(crashes=12)
        self.run_dispatch()
        self.assertEqual(self.data['stages']['A0']['status'], 'blocked')
        self.assertEqual(self.jobs, [])

    def test_pause_prevents_submission(self):
        (s.STATE / 'PAUSE').touch()
        self.run_dispatch()
        self.assertEqual(self.jobs, [])

    def test_missing_evidence_cannot_complete_stage(self):
        s.atomic(s.STATE / 'A0.done.json', {'accepted': True, 'commit': 'fake',
                 'artifacts': ['missing'], 'checks': [{'passed': True, 'evidence': 'missing'}]})
        self.assertIsNone(s.accepted('A0', self.data))

    def test_continuation_overrides_done(self):
        (s.STATE / 'A0.continue').write_text('not finished')
        s.atomic(s.STATE / 'A0.done.json', {'accepted': True})
        self.assertIsNone(s.accepted('A0', self.data))

    def test_acceptance_requires_clean_pinned_history_and_checks(self):
        wt = s.worktree('A0')
        (wt / 'docs/worklog').mkdir(parents=True)
        report = 'docs/worklog/gs3d_A0.md'
        (wt / report).write_text('evidence')
        (s.LOGS / 'A0_done.md').write_text('handoff')
        proof = {'accepted': True, 'commit': 'head', 'artifacts': [report],
                 'checks': [{'name': 'actual check', 'passed': True, 'evidence': report}]}
        s.atomic(s.STATE / 'A0.done.json', proof)

        def fake_git(*args, **kw):
            output = {'rev-parse': 'head', 'branch': s.CONFIG['branch_prefix'] + '/A0'}.get(args[0], '')
            return subprocess.CompletedProcess(args, 0, output, '')

        with patch.object(s, 'git', side_effect=fake_git):
            self.assertIsNotNone(s.accepted('A0', self.data))
            proof['commit'] = 'wrong-head'
            s.atomic(s.STATE / 'A0.done.json', proof)
            self.assertIsNone(s.accepted('A0', self.data))

    def test_dag_is_acyclic_and_every_stage_reaches_final(self):
        final_ancestors = s.ancestors('A7')
        self.assertEqual(final_ancestors, set(s.AGENTS))
        for stage, spec in s.AGENTS.items():
            self.assertNotIn(stage, spec['deps'])
            self.assertTrue(set(spec['deps']) < final_ancestors)


if __name__ == '__main__':
    unittest.main()
