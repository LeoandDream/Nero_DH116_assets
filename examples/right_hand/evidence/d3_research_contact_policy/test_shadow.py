"""Deterministic contact-policy boundaries; synthetic logs, zero simulator APIs."""
import copy
import gzip
import json
import math
from pathlib import Path
import unittest

from shadow import ContactTracker, original_checks, in_scope


HERE = Path(__file__).resolve().parent
BOWL = 'white_bowl_1_main'
HAND = 'gripper0_finger23_link'
HAND_GEOM = 'gripper0_proxy76_finger23_link_06'


def contact(depth=.001, force=0., bowl_geom='white_bowl_1_g23',
            hand_geom=HAND_GEOM, reasons=None):
    pose = {'position_world_m': [0., 0., 0.],
            'rotation_world': [1., 0., 0., 0., 1., 0., 0., 0., 1.]}
    return {'geom': [bowl_geom, hand_geom], 'body': [BOWL, HAND],
            'distance_m': -depth, 'position_world_m': [0., 0., 0.],
            'frame': [1., 0., 0., 0., 1., 0., 0., 0., 1.],
            'normal_world': [1., 0., 0.],
            'cached_body_pose': [copy.deepcopy(pose), copy.deepcopy(pose)],
            'force_contact_frame': [force, 0., 0., 0., 0., 0.],
            'filter_reasons': [] if reasons is None else reasons}


def reasons(row, category):
    return {event['reason'] for event in row[category]}


def group(row, names):
    matches = [g for g in row['groups'] if set(g['key']) == set(names)]
    if len(matches) != 1:
        raise AssertionError(f'Expected one group for {names}: {matches}')
    return matches[0]


class ResearchContactPolicyTests(unittest.TestCase):
    def setUp(self):
        self.policy = json.loads((HERE/'research-policy.json').read_text())
        self.ref = self.policy['scales']['force_reference_N']
        self.tracker = ContactTracker(self.policy)

    def update(self, step, contacts, phase='close'):
        return self.tracker.update(step, phase, contacts)

    def test_exact_two_mm_is_not_deep(self):
        for step in range(1, 18):
            row = self.update(step, [contact(depth=.002)])
            self.assertNotIn('MPR_DEPTH_WARNING', reasons(row, 'warnings'))
            self.assertFalse(row['stops'])
            self.assertEqual(group(row, [HAND, BOWL])['deep_samples'], 0)

    def test_single_deep_sample_warns_without_claiming_duration(self):
        row = self.update(1, [contact(depth=.002001)])
        self.assertIn('MPR_DEPTH_WARNING', reasons(row, 'warnings'))
        self.assertFalse(row['stops'])
        body = group(row, [HAND, BOWL])
        self.assertEqual(body['deep_samples'], 1)
        self.assertEqual(body['deep_observed_span_s'], 0.)
        self.assertEqual(body['short_exposure_Ns'], 0.)

    def test_persistence_requires_seventeen_samples(self):
        for step in range(1, 17):
            row = self.update(step, [contact(depth=.0021, force=.01*self.ref)])
            self.assertNotIn('DEEP_CONTACT_PERSISTENT', reasons(row, 'stops'))
        row = self.update(17, [contact(depth=.0021, force=.01*self.ref)])
        self.assertIn('DEEP_CONTACT_PERSISTENT', reasons(row, 'stops'))
        body = group(row, [HAND, BOWL])
        self.assertEqual(body['deep_samples'], 17)
        self.assertAlmostEqual(body['deep_observed_span_s'], .032)

    def test_body_depth_persistence_survives_bowl_geom_change(self):
        for step in range(1, 18):
            bowl_geom = 'white_bowl_1_g22' if step <= 8 else 'white_bowl_1_g23'
            row = self.update(step, [contact(depth=.0021, bowl_geom=bowl_geom)])
        self.assertIn('DEEP_CONTACT_PERSISTENT', reasons(row, 'stops'))
        self.assertEqual(group(row, [HAND, BOWL])['deep_samples'], 17)
        self.assertEqual(group(row, [HAND_GEOM, 'white_bowl_1_g23'])['deep_samples'], 9)

    def test_no_contact_frame_resets_deep_persistence(self):
        for step in range(1, 10):
            self.update(step, [contact(depth=.0021)])
        row = self.update(10, [])
        self.assertFalse(row['stops'])
        for step in range(11, 27):
            row = self.update(step, [contact(depth=.0021)])
            self.assertNotIn('DEEP_CONTACT_PERSISTENT', reasons(row, 'stops'))
        row = self.update(27, [contact(depth=.0021)])
        self.assertIn('DEEP_CONTACT_PERSISTENT', reasons(row, 'stops'))
        self.assertEqual(group(row, [HAND, BOWL])['deep_samples'], 17)

    def test_zero_force_does_not_exempt_deep_contact(self):
        for step in range(1, 18):
            row = self.update(step, [contact(depth=.0021, force=0.)])
        self.assertIn('DEEP_CONTACT_PERSISTENT', reasons(row, 'stops'))
        self.assertNotIn('SAMPLED_LOAD_EXPOSURE', reasons(row, 'stops'))

    def test_exact_force_warning_and_immediate_boundaries(self):
        row = self.update(1, [contact(force=self.ref)])
        self.assertIn('SAMPLED_LOAD_WARNING', reasons(row, 'warnings'))
        self.assertFalse(row['stops'])
        row = self.update(2, [contact(force=10*self.ref)])
        self.assertIn('SAMPLED_LOAD_EXTREME', reasons(row, 'stops'))

    def test_body_load_sums_all_native_points_once(self):
        row = self.update(1, [contact(force=6*self.ref, bowl_geom='white_bowl_1_g22'),
                              contact(force=6*self.ref, bowl_geom='white_bowl_1_g23')])
        self.assertIn('SAMPLED_LOAD_EXTREME', reasons(row, 'stops'))
        self.assertAlmostEqual(group(row, [HAND, BOWL])['normal_force_N'], 12*self.ref)
        self.assertAlmostEqual(group(row, [HAND_GEOM, 'white_bowl_1_g22'])['normal_force_N'],
                               6*self.ref)

    def test_long_load_exposure_uses_seventeen_samples(self):
        for step in range(1, 17):
            row = self.update(step, [contact(depth=.001, force=4*self.ref)])
            self.assertNotIn('SAMPLED_LOAD_EXPOSURE', reasons(row, 'stops'))
        row = self.update(17, [contact(depth=.001, force=4*self.ref)])
        self.assertIn('SAMPLED_LOAD_EXPOSURE', reasons(row, 'stops'))
        self.assertAlmostEqual(group(row, [HAND, BOWL])['long_exposure_Ns'],
                               4*self.ref*.032)
        self.assertNotIn('DEEP_CONTACT_PERSISTENT', reasons(row, 'stops'))

    def test_gap_keeps_prior_rolling_load_history(self):
        for step in range(1, 18):
            row = self.update(step, [] if step == 9 else [contact(force=4*self.ref)])
        self.assertIn('SAMPLED_LOAD_EXPOSURE', reasons(row, 'stops'))
        # One missing interior observation removes two half-interval contributions.
        self.assertAlmostEqual(group(row, [HAND, BOWL])['long_exposure_Ns'],
                               4*self.ref*(.032-.002))

    def test_loaded_growth_needs_seven_deep_samples(self):
        for step in range(1, 7):
            row = self.update(step, [contact(depth=.0021+step*1e-6, force=self.ref)])
            self.assertNotIn('LOADED_DEPTH_GROWTH', reasons(row, 'stops'))
        row = self.update(7, [contact(depth=.002107, force=self.ref)])
        self.assertIn('LOADED_DEPTH_GROWTH', reasons(row, 'stops'))
        body = group(row, [HAND, BOWL])
        self.assertAlmostEqual(body['short_exposure_Ns'], self.ref*.012)
        self.assertAlmostEqual(body['short_depth_change_m'], 6e-6)

    def test_constant_depth_is_not_loaded_growth(self):
        for step in range(1, 8):
            row = self.update(step, [contact(depth=.0021, force=2*self.ref)])
        self.assertNotIn('LOADED_DEPTH_GROWTH', reasons(row, 'stops'))
        self.assertFalse(row['stops'])

    def test_filtered_contacts_are_inapplicable(self):
        row = self.update(1, [contact(depth=.1, force=100*self.ref,
                                      reasons=['COLLISION_MASKS'])])
        self.assertFalse(row['warnings'])
        self.assertFalse(row['stops'])
        self.assertFalse(row['groups'])

    def test_phase_outside_declared_scope_is_inapplicable(self):
        row = self.update(1, [contact(depth=.1, force=100*self.ref)], phase='descend')
        self.assertFalse(row['warnings'])
        self.assertFalse(row['stops'])
        self.assertFalse(row['groups'])

    def test_target_and_hand_identity_both_required(self):
        c = contact(depth=.1, force=100*self.ref)
        c['body'][1] = 'robot0_link5'
        row = self.update(1, [c])
        self.assertFalse(row['warnings'])
        self.assertFalse(row['stops'])
        self.assertFalse(row['groups'])

    def test_missing_or_repeated_step_is_invalid(self):
        self.update(1, [])
        with self.assertRaises(ValueError):
            self.update(3, [])
        tracker = ContactTracker(self.policy)
        tracker.update(1, 'close', [])
        with self.assertRaises(ValueError):
            tracker.update(1, 'close', [])

    def test_nonfinite_contact_data_is_invalid(self):
        for field in ['distance_m', 'force_contact_frame', 'position_world_m', 'normal_world']:
            for value in [math.nan, math.inf, -math.inf]:
                with self.subTest(field=field, value=value):
                    c = contact()
                    if isinstance(c[field], list):
                        c[field][0] = value
                    else:
                        c[field] = value
                    with self.assertRaises(ValueError):
                        ContactTracker(self.policy).update(1, 'close', [c])

    def test_missing_required_contact_fields_are_invalid(self):
        for field in ['geom', 'body', 'distance_m', 'force_contact_frame', 'filter_reasons']:
            with self.subTest(field=field):
                c = contact()
                del c[field]
                with self.assertRaises(ValueError):
                    ContactTracker(self.policy).update(1, 'close', [c])


class OriginalProtectionTests(unittest.TestCase):
    """Synthetic perturbations of one saved row; never feed a physics engine."""
    def setUp(self):
        run = HERE.parent/'d3_finger22_exploratory/run'
        with gzip.open(run/'state.jsonl.gz', 'rt') as f:
            self.state = json.loads(next(f))
        with gzip.open(run/'contacts.jsonl.gz', 'rt') as f:
            self.cache = json.loads(next(f))
        self.frozen = json.loads((run/'frozen-before-first-step.json').read_text())
        with gzip.open(run/'robot-physical-preflight.json.gz', 'rt') as f:
            self.snapshot = json.load(f)['actual_snapshot']

    def checks(self):
        return original_checks(self.state, self.cache, self.frozen, self.snapshot, 0.)

    def test_original_warning_is_immediate(self):
        self.state['warnings'] = {'synthetic_warning': 1}
        self.state['failures'] = ['NUMERICAL_WARNING_OR_TIME_RESET']
        self.assertIn('NUMERICAL_WARNING_OR_TIME_RESET', self.checks()[1])

    def test_original_nonfinite_is_immediate(self):
        self.state['full_qvel'][0] = math.nan
        self.state['finite'] = False
        self.state['failures'] = ['NONFINITE_STATE']
        self.assertIn('NONFINITE_STATE', self.checks()[1])

    def test_original_arm_and_envelope_are_immediate(self):
        self.state['robot_state_18'][0] += .1001
        self.state['full_qpos'][0] = self.state['robot_state_18'][0]
        self.state['arm_error_rad'][0] = abs(self.state['robot_state_18'][0]-self.state['action_13'][0])
        self.state['failures'] = ['ARM_TRACKING', 'OUTSIDE_REGISTERED_PATH_ENVELOPE']
        self.assertEqual(self.checks()[1], self.state['failures'])

    def test_original_active_guard_only_applies_in_hold(self):
        self.state['target_qpos_18'][7] = self.state['robot_state_18'][7]+.021
        self.state['active_error_rad'][0] = abs(self.state['robot_state_18'][7]-self.state['target_qpos_18'][7])
        self.state['phase'] = 'close'
        self.assertNotIn('ACTIVE_HOLD_TRACKING', self.checks()[1])
        self.state['phase'] = 'hold'
        self.state['failures'] = ['ACTIVE_HOLD_TRACKING']
        self.assertIn('ACTIVE_HOLD_TRACKING', self.checks()[1])

    def test_original_mimic_guard_is_immediate(self):
        expected = .004592+.8808*self.state['robot_state_18'][8]
        self.state['robot_state_18'][9] = expected+.0201
        self.state['full_qpos'][9] = self.state['robot_state_18'][9]
        self.state['mimic_rad'] = abs(self.state['robot_state_18'][9]-expected)
        self.state['failures'] = ['MIMIC']
        self.assertIn('MIMIC', self.checks()[1])

    def test_original_joint_limit_is_immediate(self):
        high = self.snapshot['objects']['JOINT'][7]['fields']['jnt_range'][1]
        q = high+.010001
        self.state['robot_state_18'][7] = self.state['full_qpos'][7] = q
        self.state['active_error_rad'][0] = abs(q-self.state['target_qpos_18'][7])
        self.state['joint_overshoot_rad'] = q-high
        self.state['failures'] = ['JOINT_LIMIT', 'OUTSIDE_REGISTERED_PATH_ENVELOPE']
        self.assertIn('JOINT_LIMIT', self.checks()[1])

    def test_unexpected_wrist_stops_even_with_zero_force(self):
        c = contact(depth=.0001, force=0.)
        c['geom'][1] = 'robot0_wrist_reuse_synthetic'
        c['body'][1] = 'robot0_link5'
        self.cache['contacts'] = [c]
        self.state['robot_overlap_m'] = .0001
        self.state['failures'] = ['UNEXPECTED_WRIST_EXTERNAL_CONTACT']
        self.assertIn('UNEXPECTED_WRIST_EXTERNAL_CONTACT', self.checks()[1])

    def test_outside_research_scope_keeps_original_depth_stop(self):
        c = contact(depth=.002001, force=0.)
        self.cache['contacts'] = [c]
        self.state['robot_overlap_m'] = .002001
        self.state['failures'] = ['ROBOT_CONTACT_OVERLAP']
        policy = json.loads((HERE/'research-policy.json').read_text())
        self.assertFalse(in_scope(policy, self.state['phase'], c))
        self.assertEqual(self.checks()[0], ['ROBOT_CONTACT_OVERLAP'])


if __name__ == '__main__':
    unittest.main()
