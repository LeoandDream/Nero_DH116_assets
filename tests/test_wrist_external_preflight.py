"""Static guards for the conditional external-contact preflight."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'examples/right_hand'))


class ExternalPreflightTests(unittest.TestCase):
    def module(self):
        self.assertTrue((ROOT/'examples/right_hand/wrist_external_preflight.py').is_file(),
                        'The scoped external preflight entry must exist')
        import wrist_external_preflight as E
        return E

    def test_geometry_bounds_include_entire_rotated_box(self):
        E = self.module()
        import numpy as np
        m = E.mj.MjModel.from_xml_string('<mujoco><worldbody><geom type="box" size=".1 .2 .3" '
            'pos="1 2 3" euler="0 0 45"/></worldbody></mujoco>')
        d = E.mj.MjData(m)
        E.mj.mj_kinematics(m, d)
        lo, hi = E.world_aabbs(m, d)
        corners = np.array([[x,y,z] for x in (-.1,.1) for y in (-.2,.2) for z in (-.3,.3)])
        moved = corners@d.geom_xmat[0].reshape(3,3).T+d.geom_xpos[0]
        self.assertTrue(np.all(moved >= lo[0]-1e-15))
        self.assertTrue(np.all(moved <= hi[0]+1e-15))

    def test_no_clearance_claim_from_zero_or_negative_distance(self):
        E = self.module()
        self.assertEqual(E.distance_status(.001), 'SEPARATED')
        self.assertEqual(E.distance_status(0.), 'UNKNOWN_CLEARANCE')
        self.assertEqual(E.distance_status(-.001), 'INTERFERENCE')

    def test_schedule_is_original_270_without_extra_slowdown(self):
        E = self.module()
        self.assertTrue(E.schedule_audit()['original_270_rows_exact'])

    def test_large_query_margin_is_not_used_as_interference_evidence(self):
        E = self.module()
        self.assertTrue(callable(getattr(E, 'load_polytopes', None)),
                        'Use independently bounded whole compiled geometry')
        import xml.etree.ElementTree as ET
        m = E.mj.MjModel.from_xml_path(str(E.W.CANDIDATE))
        d = E.mj.MjData(m)
        d.qpos[:] = E.W.read(E.P.PLAN)['samples'][0]['qpos']
        E.mj.mj_kinematics(m, d)
        g, h = m.geom('robot0_wrist_reuse_link5_008').id, m.geom('gripper0_proxy76_base_link_06').id
        poly = E.load_polytopes(m, ET.parse(E.W.CANDIDATE).getroot(), [g,h])
        result = E.projection_pair(m, d, g, h, poly)
        self.assertEqual(result['classification'], 'CERTIFIED_CONVEX_SOLIDS_SEPARATED')
        self.assertGreater(result['separation_lower_bound_m'], .0044)


if __name__ == '__main__':
    unittest.main()
