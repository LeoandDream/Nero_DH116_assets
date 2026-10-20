"""Offline research-runner policy integration; saved caches and synthetic data only.

This module never constructs an environment, model, data object or renderer.
The historical trajectory remains an immutable explicit acceptance reference.
"""
import copy
import ast
import gzip
import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
EVIDENCE = HERE/'evidence/d3_research_contact_policy'
RUN = HERE/'evidence/d3_finger22_exploratory/run'
POLICY_SHA = '48632dfa4bbe0630ef7d61b9e706240d7769ad6361d588e1d02025bde4bc6454'
TIMING = 'final RK4 internal-stage cache; qpos/height in state row are post-integration; no post-step forward'
BOWL = 'white_bowl_1_main'
HAND = 'gripper0_finger23_link'
HAND_GEOM = 'gripper0_proxy76_finger23_link_06'


def backend():
    """Load only the new independent backend, never import frozen shadow.py."""
    spec = importlib.util.spec_from_file_location('tested_libero_research', HERE/'libero_research.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def rows(path):
    with gzip.open(path, 'rt') as stream:
        for line in stream:
            yield json.loads(line)


def contact(depth=.001, force=0., reasons=None):
    return {'geom': ['white_bowl_1_g23', HAND_GEOM], 'body': [BOWL, HAND],
            'distance_m': -depth, 'position_world_m': [0., 0., 0.],
            'frame': [1., 0., 0., 0., 1., 0., 0., 0., 1.],
            'normal_world': [1., 0., 0.],
            'force_contact_frame': [force, 0., 0., 0., 0., 0.],
            'filter_reasons': [] if reasons is None else reasons}


class ResearchRunnerIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = backend()
        cls.policy, cls.tracker_type = cls.module.load_contact_policy()

    def setUp(self):
        self.runtime = self.module.ResearchPolicy(self.policy)
        self.ref = self.policy['scales']['force_reference_N']

    def evaluate(self, step=1, phase='close', contacts=None, independent=None):
        return self.runtime.evaluate(step, phase, [] if contacts is None else contacts,
                                     [] if independent is None else independent)

    def test_frozen_policy_and_tracker_loaded_without_mutating_proposal(self):
        raw = (EVIDENCE/'research-policy.json').read_bytes()
        self.assertEqual(hashlib.sha256(raw).hexdigest(), POLICY_SHA)
        self.assertEqual(self.policy, json.loads(raw))
        self.assertEqual(self.policy['status'], 'RESEARCH_ONLY_NOT_APPROVED')
        self.assertEqual(self.tracker_type.__name__, 'ContactTracker')

    def test_executor_policy_dependency_attributes_are_complete(self):
        tree = ast.parse((HERE/'white_bowl_run.py').read_text())
        required = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)
                    and isinstance(n.value, ast.Name) and n.value.id == 'wrist_policy'}
        available = set(self.module.POLICY_ATTRIBUTES)|{'validate_external_preflight'}
        self.assertFalse(required-available)

    def test_module_import_requires_no_engine_or_array_package(self):
        # A fresh process prevents another test's imports masking dependencies.
        code = r'''
import importlib.abc, importlib.util, pathlib, sys
forbidden = {'mujoco', 'robosuite', 'libero', 'numpy', 'scipy'}
class Reject(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in forbidden:
            raise AssertionError('Forbidden offline dependency: '+fullname)
sys.meta_path.insert(0, Reject())
spec = importlib.util.spec_from_file_location('offline_policy_under_test', sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
policy, tracker = module.load_contact_policy()
assert tracker.__name__ == 'ContactTracker'
assert not any(n.split('.')[0] in forbidden for n in sys.modules)
assert module.ResearchPolicy(policy).evaluate(1, 'settle_preshape', [], [])['research_failures'] == []
print('PURE_STDLIB_IMPORT_AND_FIRST_SAMPLE_PASS')
'''
        completed = subprocess.run([sys.executable, '-B', '-c', code, str(HERE/'libero_research.py')],
                                   check=True, capture_output=True, text=True)
        self.assertIn('PURE_STDLIB_IMPORT_AND_FIRST_SAMPLE_PASS', completed.stdout)

    def test_single_in_scope_deep_contact_keeps_legacy_failure_but_only_warns_research(self):
        contacts = [contact(depth=.002001)]
        original = copy.deepcopy(contacts)
        row = self.evaluate(contacts=contacts)
        self.assertEqual(row['legacy_failures'], ['ROBOT_CONTACT_OVERLAP'])
        self.assertFalse(row['research_failures'])
        self.assertIn('MPR_DEPTH_WARNING', {e['reason'] for e in row['warnings']})
        self.assertEqual(contacts, original)

    def test_original_exact_two_mm_boundary_retained(self):
        row = self.evaluate(contacts=[contact(depth=.002)])
        self.assertFalse(row['legacy_failures'])
        self.assertFalse(row['research_failures'])
        self.assertFalse(row['warnings'])

    def test_non_target_body_contact_stops_immediately(self):
        c = contact(depth=.002001)
        c['body'][0] = 'table'
        c['geom'][0] = 'table_collision'
        row = self.evaluate(contacts=[c])
        self.assertIn('ROBOT_CONTACT_OVERLAP', row['legacy_failures'])
        self.assertIn('ROBOT_CONTACT_OVERLAP', row['research_failures'])
        self.assertFalse(row['groups'])

    def test_hand_target_depth_in_approach_is_not_research_exempt(self):
        row = self.evaluate(phase='approach_high', contacts=[contact(depth=.002001)])
        self.assertIn('ROBOT_CONTACT_OVERLAP', row['research_failures'])

    def test_arm_bowl_depth_is_not_hand_bowl_exempt(self):
        c = contact(depth=.002001)
        c['body'][1] = 'robot0_link5'
        c['geom'][1] = 'robot0_link5_collision'
        self.assertIn('ROBOT_CONTACT_OVERLAP', self.evaluate(contacts=[c])['research_failures'])

    def test_filtered_contacts_retain_original_ineligibility(self):
        row = self.evaluate(contacts=[contact(depth=.1, force=100*self.ref,
                                              reasons=['COLLISION_MASKS'])])
        self.assertFalse(row['legacy_failures'])
        self.assertFalse(row['research_failures'])
        self.assertFalse(row['warnings'])

    def test_all_existing_independent_protections_survive(self):
        protections = ['NONFINITE_STATE', 'NUMERICAL_WARNING_OR_TIME_RESET', 'ARM_TRACKING',
                       'ACTIVE_HOLD_TRACKING', 'MIMIC', 'JOINT_LIMIT',
                       'UNEXPECTED_WRIST_EXTERNAL_CONTACT', 'OUTSIDE_REGISTERED_PATH_ENVELOPE']
        for reason in protections:
            with self.subTest(reason=reason):
                runtime = self.module.ResearchPolicy(self.policy)
                independent = [reason]
                row = runtime.evaluate(1, 'close', [contact(depth=.002001)], independent)
                self.assertIn(reason, row['legacy_failures'])
                self.assertIn(reason, row['research_failures'])
                self.assertEqual(independent, [reason])

    def test_extreme_load_stops_even_below_original_depth(self):
        row = self.evaluate(contacts=[contact(force=10*self.ref)])
        self.assertFalse(row['legacy_failures'])
        self.assertIn('SAMPLED_LOAD_EXTREME', row['research_failures'])

    def test_persistent_deep_contact_requires_seventeen_samples(self):
        for step in range(1, 17):
            row = self.evaluate(step, contacts=[contact(depth=.0021)])
            self.assertFalse(row['research_failures'])
        row = self.evaluate(17, contacts=[contact(depth=.0021)])
        self.assertIn('DEEP_CONTACT_PERSISTENT', row['research_failures'])

    def test_loaded_depth_growth_requires_seven_samples(self):
        for step in range(1, 7):
            row = self.evaluate(step, contacts=[contact(depth=.0021+step*1e-6, force=self.ref)])
            self.assertFalse(row['research_failures'])
        row = self.evaluate(7, contacts=[contact(depth=.002107, force=self.ref)])
        self.assertIn('LOADED_DEPTH_GROWTH', row['research_failures'])

    def test_rolling_load_stops_without_a_deep_contact(self):
        for step in range(1, 17):
            row = self.evaluate(step, contacts=[contact(force=4*self.ref)])
            self.assertFalse(row['research_failures'])
        row = self.evaluate(17, contacts=[contact(force=4*self.ref)])
        self.assertIn('SAMPLED_LOAD_EXPOSURE', row['research_failures'])
        self.assertFalse(row['legacy_failures'])

    def test_step_discontinuities_are_unknown_stop(self):
        for sequence in ((2,), (1, 1), (1, 3)):
            with self.subTest(sequence=sequence):
                runtime = self.module.ResearchPolicy(self.policy)
                for step in sequence:
                    row = runtime.evaluate(step, 'close', [], [])
                self.assertIn('LOG_INVALID_UNKNOWN', row['research_failures'])

    def test_malformed_or_nonfinite_contact_is_unknown_stop(self):
        cases = []
        for key in ('geom', 'body', 'distance_m', 'force_contact_frame',
                    'position_world_m', 'frame', 'normal_world', 'filter_reasons'):
            c = contact()
            del c[key]
            cases.append((key, c))
        for key in ('distance_m', 'force_contact_frame', 'position_world_m', 'normal_world'):
            for value in (math.nan, math.inf, -math.inf):
                c = contact()
                if isinstance(c[key], list): c[key][0] = value
                else: c[key] = value
                cases.append((key, c))
        for description, c in cases:
            with self.subTest(field=description, contact=c):
                row = self.module.ResearchPolicy(self.policy).evaluate(1, 'close', [c], [])
                self.assertIn('LOG_INVALID_UNKNOWN', row['research_failures'])

    def test_all_4769_saved_samples_match_frozen_shadow_exactly(self):
        first_legacy = first_research = first_warning = first_depth_warning = None
        checked = 0
        for state, cache, expected in itertools.zip_longest(
                rows(RUN/'state.jsonl.gz'), rows(RUN/'contacts.jsonl.gz'),
                rows(EVIDENCE/'shadow.jsonl.gz')):
            self.assertIsNotNone(state, 'Unequal historical log lengths')
            self.assertIsNotNone(cache, 'Unequal historical log lengths')
            self.assertIsNotNone(expected, 'Unequal historical log lengths')
            checked += 1
            self.assertEqual((state['step'], cache['step'], expected['step']), (checked,)*3)
            self.assertEqual(cache['timing'], TIMING)
            self.assertEqual(state['phase'], expected['phase'])
            contacts_before = copy.deepcopy(cache['contacts'])
            independent = [x for x in state['failures'] if x != 'ROBOT_CONTACT_OVERLAP']
            actual = self.evaluate(checked, state['phase'], cache['contacts'], independent)
            self.assertEqual(actual['legacy_failures'], expected['legacy_stops'], f'Legacy step {checked}')
            self.assertEqual(sorted(set(actual['research_failures'])), expected['research_stops'],
                             f'Research step {checked}')
            self.assertEqual(actual['warnings'], expected['warnings'], f'Warnings step {checked}')
            self.assertEqual(actual['groups'], expected['groups'], f'Groups step {checked}')
            self.assertEqual(cache['contacts'], contacts_before, f'Mutated cache step {checked}')
            if actual['legacy_failures'] and first_legacy is None: first_legacy = checked
            if actual['research_failures'] and first_research is None: first_research = checked
            if actual['warnings'] and first_warning is None: first_warning = checked
            if any(e['reason'] == 'MPR_DEPTH_WARNING' for e in actual['warnings']) and first_depth_warning is None:
                first_depth_warning = checked
        self.assertEqual(checked, 4769)
        self.assertEqual(first_legacy, 4769)
        self.assertIsNone(first_research)
        self.assertEqual(first_warning, 4723)
        self.assertEqual(first_depth_warning, 4769)
        print('FROZEN_SHADOW_EXACT: 4769 samples; legacy_stop=4769; research_stop=None; '
              'first_warning=4723; first_depth_warning=4769; contacts unchanged')


if __name__ == '__main__':
    unittest.main(verbosity=2)
