"""Targeted stop-policy checks; never integrate a simulator."""
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'examples/right_hand'))


class Finger22ConditionalTests(unittest.TestCase):
    def test_root_or_uncertain_contact_stops_but_front_pad_does_not(self):
        import finger22_conditional as C
        # Synthetic closed box: explicit +X front, -X back, +/-Y sides.
        planes = [[1,0,0,-1],[-1,0,0,-1],[0,1,0,-1],[0,-1,0,-1],
                  [0,0,1,-1],[0,0,-1,-1]]
        risks = {name: {'planes': planes, 'marked_planes': [2]} for name in C.NEW}
        def contact(name, point):
            return {'geom': ['bowl', name], 'body': ['white_bowl_1_main', C.BODY],
                    'filter_reasons': [], 'distance_m': 0., 'position_world_m': point,
                    'cached_body_pose': [{}, {'position_world_m': [0,0,0],
                                              'rotation_world': np.eye(3).ravel().tolist()}]}
        root = C.classify_contact(contact(C.NEW[0], [1,0,0]), risks)
        self.assertEqual(root['status'], 'STOP_ROOT_RISK')
        cavity = C.classify_contact(contact(C.NEW[1], [0,1,0]), risks)
        self.assertEqual(cavity['status'], 'STOP_MARKED_CAVITY_FACET')
        front = C.classify_contact(contact(C.NEW[1], [1,0,0]), risks)
        self.assertEqual(front['status'], 'NORMAL_POSITIVE_X_FACE')
        unknown = C.classify_contact(contact(C.NEW[1], [0,0,0]), risks)
        self.assertEqual(unknown['status'], 'UNKNOWN_CONTACT_FACE')

    def test_frozen_inputs_and_original_thresholds(self):
        import finger22_conditional as C
        p = C.protocol(ROOT)
        self.assertEqual(p['maximum_physics_steps'], 6750)
        self.assertEqual(p['schedule']['commands'], 270)
        self.assertEqual(p['original_criteria']['eligible_internal_and_external_overlap_m_max'], .002)
        self.assertEqual(p['original_criteria']['arm_tracking_error_rad_max'], .1)
        self.assertEqual(p['model_sha256'], C.MODEL_SHA)
        self.assertFalse(p['fill_exceptions_approved'])

    def test_unknown_and_zero_force_risks_stop(self):
        import finger22_conditional as C
        p = C.protocol(ROOT)
        q = C.read(ROOT/C.PLAN)['samples'][0]['qpos']
        c = {'geom': ['bowl', C.NEW[1]], 'body': ['white_bowl_1_main', C.BODY],
             'filter_reasons': [], 'finger22_region': {'status': 'UNKNOWN_CONTACT_FACE'}}
        self.assertIn('FINGER22_RISK_CONTACT_UNKNOWN', C.extra_failures([c], q, p))
        c['finger22_region']['status'] = 'NORMAL_POSITIVE_X_FACE'
        self.assertEqual(C.extra_failures([c], q, p), [])


if __name__ == '__main__':
    unittest.main()
