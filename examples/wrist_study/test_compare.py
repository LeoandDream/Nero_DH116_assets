import copy
import unittest
import xml.etree.ElementTree as ET

import compare


def fixture(prefix):
    root = ET.Element('mujoco')
    assets = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    arm = ET.SubElement(world, 'body', name='robot0_link5')
    flange = ET.SubElement(arm, 'body', name='robot0_gripper_flange')
    for i, body in enumerate((arm, flange)):
        name = prefix+str(i)
        ET.SubElement(assets, 'mesh', name=name, file=name+'.obj')
        ET.SubElement(body, 'geom', name=name, mesh=name, friction='1 .005 .0001')
    ET.SubElement(arm, 'inertial', mass='2', diaginertia='1 1 1')
    hand = ET.SubElement(flange, 'body', name='gripper0_base_link')
    ET.SubElement(hand, 'geom', name='hand', type='sphere', size='.01')
    ET.SubElement(root, 'contact')
    return root


class DiagnosticReplacement(unittest.TestCase):
    def test_replaces_wrists_without_overlay_or_changing_hand_and_mechanics(self):
        scene, other = fixture('old'), fixture('new')
        output = compare.replace_wrist(scene, other)
        self.assertEqual([g.get('name') for g in compare.wrists(output)], ['new0', 'new1'])
        self.assertEqual({e.get('name') for e in output.findall('asset/mesh')}, {'new0', 'new1'})
        for xpath in ('.//inertial', './/body[@name="gripper0_base_link"]', 'contact'):
            self.assertEqual(ET.tostring(scene.find(xpath)), ET.tostring(output.find(xpath)))

    def test_does_not_mutate_either_source(self):
        scene, other = fixture('old'), fixture('new')
        before = [ET.tostring(copy.deepcopy(x)) for x in (scene, other)]
        compare.replace_wrist(scene, other)
        self.assertEqual(before, [ET.tostring(x) for x in (scene, other)])

    def test_visual_noncolliding_geometry_is_retained(self):
        scene, other = fixture('old'), fixture('new')
        ET.SubElement(scene.find('.//body'), 'geom', name='visual', type='box',
                      size='1 1 1', contype='0', conaffinity='0')
        result = compare.replace_wrist(scene, other)
        self.assertIsNotNone(result.find('.//geom[@name="visual"]'))


if __name__ == '__main__':
    unittest.main()
