"""Pure record/stop-policy checks; no simulator or integration."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'examples/right_hand'))


class ConditionalTests(unittest.TestCase):
    def module(self):
        self.assertTrue((ROOT/'examples/right_hand/wrist_conditional.py').is_file())
        import wrist_conditional as C
        return C

    def test_wrist_hand_contact_stops_even_without_positive_force(self):
        C = self.module()
        p = C.protocol(ROOT)
        q = C.read(ROOT/C.PLAN)['samples'][0]['qpos']
        contact = {'geom': ['robot0_wrist_reuse_gripper_flange_103', 'finger'],
                   'body': ['robot0_gripper_flange', 'gripper0_finger14_link'],
                   'filter_reasons': [], 'distance_m': 0., 'force_contact_frame': [0.]*6}
        self.assertIn('UNEXPECTED_WRIST_EXTERNAL_CONTACT', C.extra_failures([contact], q, p))
        contact['body'][1] = 'robot0_link5'
        self.assertNotIn('UNEXPECTED_WRIST_EXTERNAL_CONTACT', C.extra_failures([contact], q, p))

    def test_outside_registered_path_envelope_stops(self):
        C = self.module()
        p = C.protocol(ROOT)
        q = C.read(ROOT/C.PLAN)['samples'][0]['qpos'][:]
        q[5] = p['qpos_envelope_upper_rad'][5]+.001
        self.assertIn('OUTSIDE_REGISTERED_PATH_ENVELOPE', C.extra_failures([], q, p))

    def test_old_thresholds_and_budget_are_preserved(self):
        C = self.module()
        p = C.protocol(ROOT)
        self.assertEqual(p['maximum_physics_steps'], 6750)
        self.assertEqual(p['original_criteria']['arm_tracking_error_rad_max'], .1)
        self.assertEqual(p['original_criteria']['eligible_internal_and_external_overlap_m_max'], .002)
        self.assertFalse(p['fill_exceptions_approved'])

    def test_reset_snapshot_comparison_does_not_ignore_body_poses(self):
        C = self.module()
        self.assertTrue(callable(getattr(C, 'compare_reset_snapshots', None)))
        import copy
        old = C.read(ROOT/'examples/right_hand/evidence/d3_bowl78_slow_001/robot-physical-preflight.json.gz')['actual_snapshot']
        self.assertTrue(C.compare_reset_snapshots(old,old,[],[])['passed'])
        new = copy.deepcopy(old)
        new['objects']['BODY'][1]['fields']['body_pos'][0] += .001
        with self.assertRaises(ValueError):
            C.compare_reset_snapshots(old,new,[],[])


if __name__ == '__main__':
    unittest.main()
