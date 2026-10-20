"""Offline close149 contracts; saved JSON, source adaptation, and mocks only.

No engine, environment, model, renderer, or native state is imported or created.
"""
import ast
from copy import deepcopy
import hashlib
import importlib.abc
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SCHEDULE = HERE/'evidence/d3_finger22_dynamic/run/schedule.json'
RESEARCH_SHA = '7f15b06b5cf1333362a580f6841f90ca8bdde19d3e70ce97d16c342f79ff0b73'
SCHEDULE_SHA = 'ebf188b0bf49ad3b74a042c52b5fa4ad5df02dcb8f84caea565fcdaf9fdc7e32'
FORBIDDEN = {'mujoco', 'robosuite', 'libero', 'numpy', 'scipy', 'PIL'}


class RejectEngine(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in FORBIDDEN:
            raise AssertionError('Forbidden offline dependency: '+fullname)


def load(name):
    guard = RejectEngine()
    sys.meta_path.insert(0, guard)
    sys.path.insert(0, str(HERE))
    try:
        spec = importlib.util.spec_from_file_location('tested_'+name, HERE/(name+'.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(HERE))
        sys.meta_path.remove(guard)


class Close149OfflineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load('close149')
        cls.old = json.loads(SCHEDULE.read_text())

    def test_import_and_schedule_are_stdlib_only_in_fresh_process(self):
        code = '''
import importlib.abc, importlib.util, sys
class Reject(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'mujoco', 'robosuite', 'libero', 'numpy', 'scipy', 'PIL'}:
            raise AssertionError('Forbidden offline dependency: '+fullname)
sys.meta_path.insert(0, Reject())
spec = importlib.util.spec_from_file_location('offline_close149', sys.argv[1])
sys.path.insert(0, str(__import__('pathlib').Path(sys.argv[1]).parent))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
schedule = module.planned_schedule()
assert module.summarize(schedule)['expected_physics_steps'] == 6500
assert not any(n.split('.')[0] in {'mujoco', 'robosuite', 'libero', 'numpy', 'scipy', 'PIL'} for n in sys.modules)
print('PURE_STDLIB_CLOSE149_PASS')
'''
        completed = subprocess.run([sys.executable, '-I', '-B', '-c', code, str(HERE/'close149.py')],
                                   check=True, capture_output=True, text=True)
        self.assertIn('PURE_STDLIB_CLOSE149_PASS', completed.stdout)

    def test_exact_mapping_prefix_arm_targets_and_hand_lock(self):
        schedule = self.module.planned_schedule()
        self.assertEqual(len(schedule), 260)
        self.assertEqual(schedule[:190], self.old[:190])
        self.assertEqual([row['macro'] for row in schedule], list(range(260)))
        self.assertEqual([row['source_macro'] for row in schedule[190:]], list(range(160, 230)))
        expected = deepcopy(self.old[:190]+self.old[200:])
        for index, row in enumerate(expected):
            row['macro'] = index
            if index >= 190:
                row['action_13'][7:] = self.old[189]['action_13'][7:]
                row['target_qpos_18'][7:] = self.old[189]['target_qpos_18'][7:]
        self.assertEqual(schedule, expected)
        canonical = json.dumps(schedule, separators=(',', ':'), allow_nan=False).encode('utf-8')
        self.assertEqual(hashlib.sha256(canonical).hexdigest(), SCHEDULE_SHA)
        # Every source arm command remains exact, including all eleven hand qpos targets.
        for row, source in zip(schedule[190:], self.old[200:]):
            self.assertEqual(row['action_13'][:7], source['action_13'][:7])
            self.assertEqual(row['target_qpos_18'][:7], source['target_qpos_18'][:7])
            self.assertEqual(row['action_13'][7:], self.old[189]['action_13'][7:])
            self.assertEqual(row['target_qpos_18'][7:], self.old[189]['target_qpos_18'][7:])
        schedule[0]['action_13'][0] += 1
        self.assertEqual(self.module.planned_schedule()[0], self.old[0])

    def test_phase_boundaries_budget_and_first_command_changes(self):
        schedule = self.module.planned_schedule()
        info = self.module.summarize(schedule)
        phases = {'settle_preshape': 10, 'approach_high': 80, 'descend': 70,
                  'close': 30, 'lift': 40, 'hold': 30}
        self.assertEqual(info['phase_counts'], phases)
        self.assertEqual(set(info), {'commands', 'phase_counts', 'phase_boundaries',
                                    'physics_steps_per_command', 'expected_physics_steps',
                                    'physics_steps_max', 'physical_dt_s', 'duration_if_completed_s'})
        self.assertEqual((info['commands'], info['physics_steps_per_command'],
                          info['expected_physics_steps'], info['physics_steps_max'],
                          info['physical_dt_s'], info['duration_if_completed_s']),
                         (260, 25, 6500, 6500, .002, 13.0))
        start = 0
        for phase, count in phases.items():
            self.assertEqual([row['phase'] for row in schedule[start:start+count]], [phase]*count)
            self.assertEqual(info['phase_boundaries'][phase], {
                'macro_first': start, 'macro_last': start+count-1,
                'physical_step_first': start*25+1, 'physical_step_last': (start+count)*25})
            start += count
        changed = next(i for i, (new, old) in enumerate(zip(schedule, self.old))
                       if new['action_13'] != old['action_13'])
        arm_changed = next(i for i, (new, old) in enumerate(zip(schedule, self.old))
                           if new['action_13'][:7] != old['action_13'][:7])
        self.assertEqual(changed*25+1, 4751)
        self.assertEqual(arm_changed*25+1, 4776)
        self.assertEqual(schedule[190]['action_13'], self.old[189]['action_13'])

    def test_run_once_passes_through_exit_code_without_retry(self):
        for code in (0, 1, 2, 7):
            with self.subTest(code=code), tempfile.TemporaryDirectory() as temporary:
                output = Path(temporary)/'this_attempt'
                def create_output(destination):
                    destination.mkdir()
                    return code
                entry = mock.Mock(side_effect=create_output)
                with mock.patch.object(self.module, 'prepared_entry',
                                       return_value=(entry, 'old\n', 'new\n')) as prepared:
                    self.assertEqual(self.module.run_once(output), code)
                prepared.assert_called_once_with()
                entry.assert_called_once_with(output)
                self.assertEqual((output/'close149-entry.executed.py').read_text(), 'new\n\n')
                self.assertIn('+new', (output/'close149-entry.diff').read_text())
        error = RuntimeError('mock stop before native import')
        with mock.patch.object(self.module, 'prepared_entry', side_effect=error) as prepared:
            with self.assertRaisesRegex(RuntimeError, 'mock stop before native import'):
                self.module.run_once(Path('offline_only_output'))
        prepared.assert_called_once_with()

    def test_existing_output_rejection_does_not_write_or_replace_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            marker = output/'close149-entry.executed.py'
            marker.write_text('PRIOR_IMMUTABLE_EVIDENCE\n')
            before = {path.name: path.read_bytes() for path in output.iterdir()}
            entry = mock.Mock(side_effect=ValueError('Fresh output in activated checkout required'))
            with mock.patch.object(self.module, 'prepared_entry', return_value=(entry, 'old', 'new')):
                with self.assertRaisesRegex(ValueError, 'Fresh output'):
                    self.module.run_once(output)
            self.assertEqual({path.name: path.read_bytes() for path in output.iterdir()}, before)

    def test_prepared_entry_is_independent_and_retains_frozen_policy_dependencies(self):
        research = self.module.R
        before = dict(vars(research))
        entry, original, changed = self.module.prepared_entry()
        source = (HERE/'libero_research.py').read_text()
        original_node = next(n for n in ast.parse(source).body
                             if isinstance(n, ast.FunctionDef) and n.name == 'run_once')
        self.assertEqual(original, ast.get_source_segment(source, original_node))
        self.assertEqual(hashlib.sha256(source.encode()).hexdigest(), RESEARCH_SHA)
        self.assertEqual(vars(research), before)
        self.assertIsNot(entry, research.run_once)
        self.assertIsNot(entry.__globals__, research.run_once.__globals__)
        self.assertIs(entry.__globals__['adapted_executor'], self.module.adapted_executor)
        for name in ('ResearchPolicy', 'Observations', 'load_contact_policy', 'selected_policy', 'validate_checkout'):
            self.assertIs(entry.__globals__[name], getattr(research, name))
        self.assertEqual(entry.__globals__['_close149_schedule'], self.module.planned_schedule())
        self.assertEqual(entry.__globals__['_research_source'], Path(research.__file__))
        # Native entry control flow, stop handling, camera postprocessing and return remain exact.
        self.assertEqual(original[original.index('    legacy.write = record'):],
                         changed[changed.index('    legacy.write = record'):])
        self.assertIn('if read(output/\'schedule.json\') != _close149_schedule:', changed)
        self.assertIn('(Path(__file__), _research_source, POLICY_DIR/', changed)
        self.assertIn('    registration.update(_close149_registration)', changed)
        self.assertIn("experiment='issue5_close149_once'", changed)

    def test_executor_uses_candidate_without_changing_original_executor_or_frozen_module(self):
        frozen = load('white_bowl_frozen78')
        original_build, original_summary = frozen.build_schedule, frozen.schedule_summary
        source = '''def execute(candidate):
    if True:
        if True:
            actions = build_schedule(candidate)
            schedule_info = schedule_summary(actions)
    return actions, schedule_info
'''
        scope = {'build_schedule': lambda candidate: deepcopy(self.old),
                 'schedule_summary': frozen.schedule_summary}
        exec(compile(source, '<offline_original_executor>', 'exec'), scope)
        old_execute = scope['execute']
        old_scope = dict(old_execute.__globals__)
        policy, observations, legacy = object(), object(), types.SimpleNamespace()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            def frozen_adapter(*args):
                (output/'executor.executed.py').write_text(source)
                (output/'executor.diff').write_text('FROZEN_RESEARCH_DIFF\n')
                return old_execute
            with mock.patch.object(self.module.R, 'adapted_executor', side_effect=frozen_adapter) as adapter:
                execute = self.module.adapted_executor(legacy, policy, observations, output)
            adapter.assert_called_once_with(legacy, policy, observations, output)
            actual, info = execute(None)
            self.assertEqual(actual, self.module.planned_schedule())
            self.assertEqual(info, self.module.summarize(actual))
            self.assertEqual((output/'research-base-executor.diff').read_text(), 'FROZEN_RESEARCH_DIFF\n')
            changed = (output/'executor.executed.py').read_text()
            self.assertEqual(source.replace(
                'actions = build_schedule(candidate)\n            schedule_info = schedule_summary(actions)',
                'actions = deepcopy(_close149_schedule)\n            schedule_info = deepcopy(_close149_summary)'), changed)
            self.assertEqual((output/'executor.diff').read_text(), 'FROZEN_RESEARCH_DIFF\n')
        self.assertEqual(old_execute(None)[0], self.old)
        self.assertEqual(old_execute.__globals__, old_scope)
        self.assertIs(frozen.build_schedule, original_build)
        self.assertIs(frozen.schedule_summary, original_summary)
        with self.assertRaisesRegex(ValueError, 'not unique'):
            self.module.replace_once('anchor anchor', 'anchor', 'replacement')
        with self.assertRaisesRegex(ValueError, 'not unique'):
            self.module.replace_once('absent', 'anchor', 'replacement')
        # Compile only Python source against an empty stdlib stub; never call this native entry.
        with tempfile.TemporaryDirectory() as temporary:
            baseline, candidate = Path(temporary)/'baseline', Path(temporary)/'candidate'
            baseline.mkdir()
            candidate.mkdir()
            before_execute = self.module.R.adapted_executor(legacy, policy, observations, baseline)
            after_execute = self.module.adapted_executor(legacy, policy, observations, candidate)
            before_source = (baseline/'executor.executed.py').read_text()
            after_source = (candidate/'executor.executed.py').read_text()
            self.assertEqual(before_source.replace(
                'actions = build_schedule(candidate)\n            schedule_info = schedule_summary(actions)',
                'actions = deepcopy(_close149_schedule)\n            schedule_info = deepcopy(_close149_summary)'),
                after_source)
            self.assertIs(after_execute.__globals__['_research_policy'], policy)
            self.assertIs(after_execute.__globals__['_observations'], observations)
            self.assertIsNot(before_execute.__globals__, after_execute.__globals__)

    def test_frozen_research_policy_keeps_contact_history_across_close_lift(self):
        research = load('libero_research')
        self.assertEqual(hashlib.sha256((HERE/'libero_research.py').read_bytes()).hexdigest(), RESEARCH_SHA)
        rules, _ = research.load_contact_policy()
        policy = research.ResearchPolicy(rules)
        contact = {'geom': ['white_bowl_1_g23', 'gripper0_proxy76_finger23_link_06'],
                   'body': ['white_bowl_1_main', 'gripper0_finger23_link'],
                   'distance_m': -.0021, 'position_world_m': [0., 0., 0.],
                   'frame': [1., 0., 0., 0., 1., 0., 0., 0., 1.],
                   'normal_world': [1., 0., 0.],
                   'force_contact_frame': [0., 0., 0., 0., 0., 0.], 'filter_reasons': []}
        for step in range(1, 18):
            row = policy.evaluate(step, 'close' if step <= 10 else 'lift', [contact], [])
            self.assertEqual({g['deep_samples'] for g in row['groups']}, {step})
            if step < 17:
                self.assertFalse(row['research_failures'])
        self.assertIn('DEEP_CONTACT_PERSISTENT', row['research_failures'])
        self.assertEqual(policy.first_research['step'], 17)


if __name__ == '__main__':
    unittest.main(verbosity=2)
