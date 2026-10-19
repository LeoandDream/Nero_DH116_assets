"""Static body-role checks for the frozen right bowl adapter; no simulator."""
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'right_white_bowl_model_test', ROOT / 'examples/right_hand/white_bowl_model.py')
ADAPTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ADAPTER)


class RightWhiteBowlRolesTests(unittest.TestCase):
    def contact_roles(self, source):
        self.assertTrue(callable(getattr(ADAPTER, '_contact_roles', None)),
                        'Adapter must derive enabled contact geoms by exact body')
        return ADAPTER._contact_roles(ET.parse(source).getroot())

    def test_formal17_preserves_existing_role_lists(self):
        roles, bodies = self.contact_roles(ROOT / 'mjcf/nero_dh116_right.xml')
        thumb = ['finger13_link_collision_0', 'finger14_link_collision_0']
        opposed = [f'finger{finger}{link}_link_collision_0'
                   for finger in range(2, 6) for link in (2, 3)]
        self.assertEqual(roles, {'left_finger': thumb, 'right_finger': opposed,
                                'left_fingerpad': thumb[-1:],
                                'right_fingerpad': opposed[1::2]})
        self.assertEqual(bodies['left_finger'],
                         ['gripper0_finger13_link', 'gripper0_finger14_link'])

    def test_78part_roles_include_every_enabled_geom_on_exact_body(self):
        roles, bodies = self.contact_roles(
            ROOT / 'examples/right_hand/candidates/finger13_local/model.xml')
        self.assertEqual({role: len(names) for role, names in roles.items()},
                         {'left_finger': 15, 'right_finger': 40,
                          'left_fingerpad': 12, 'right_fingerpad': 36})
        self.assertEqual(roles['left_finger'][:3],
                         ['finger13_local_00', 'finger13_local_01', 'finger13_local_02'])
        self.assertEqual(bodies['right_fingerpad'],
                         ['gripper0_finger23_link', 'gripper0_finger33_link',
                          'gripper0_finger43_link', 'gripper0_finger53_link'])
        source = ET.parse(ROOT / 'examples/right_hand/candidates/finger13_local/model.xml').getroot()
        for role, names in roles.items():
            owner = {geom.get('name').removeprefix('gripper0_'): body.get('name')
                     for body in source.findall('.//body') for geom in body.findall('geom')}
            self.assertEqual({owner[name] for name in names}, set(bodies[role]))
            self.assertFalse(any('visual' in name for name in names))

    def test_disabled_parent_geom_is_excluded_without_collecting_descendants(self):
        source = ET.parse(ROOT / 'mjcf/nero_dh116_right.xml').getroot()
        thumb = source.find('.//body[@name="gripper0_finger13_link"]')
        ET.SubElement(thumb, 'geom', name='gripper0_disabled', contype='0', conaffinity='0')
        ET.SubElement(thumb, 'geom', name='gripper0_affinity_only', contype='0', conaffinity='1')
        self.assertTrue(callable(getattr(ADAPTER, '_contact_roles', None)))
        roles, _ = ADAPTER._contact_roles(source)
        self.assertEqual(roles['left_finger'],
                         ['finger13_link_collision_0', 'affinity_only',
                          'finger14_link_collision_0'])


if __name__ == '__main__':
    unittest.main()
