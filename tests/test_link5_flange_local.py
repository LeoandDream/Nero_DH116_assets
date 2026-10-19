"""Compile-only adversarial checks for the one local link5/flange audit."""
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco


ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / 'examples/right_hand/link5_flange_audit.py'


def module():
    spec = importlib.util.spec_from_file_location('link5_flange_audit', ENTRY)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


BASE = '''<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 .1"/>
    <body name="link5" pos="0 0 1">
      <inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/>
      <joint name="joint5" damping="1"/>
      <geom name="old_link5" type="box" size=".1 .1 .1"/>
      <body name="flange" pos="0 0 .3">
        <inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/>
        <joint name="joint6" damping="1"/>
        <geom name="old_flange" type="box" size=".1 .1 .1"/>
      </body>
    </body>
  </worldbody>
  <actuator><motor name="motor5" joint="joint5" gear="2"/></actuator>
  <keyframe><key name="home" qpos=".1 .2" qvel=".3 .4" ctrl=".5"/></keyframe>
</mujoco>'''
REPLACED = BASE.replace(
    '<geom name="old_link5" type="box" size=".1 .1 .1"/>',
    '<geom name="new_link5_a" type="box" size=".05 .1 .1"/>'
    '<geom name="new_link5_b" type="box" size=".05 .1 .1"/>').replace(
    '<geom name="old_flange" type="box" size=".1 .1 .1"/>',
    '<geom name="new_flange" type="box" size=".1 .05 .1"/>')
MAPPING = {'new_link5_a': 'old_link5', 'new_link5_b': 'old_link5',
           'new_flange': 'old_flange'}


class Link5FlangeStaticTests(unittest.TestCase):
    def compile_and_compare(self, candidate):
        run = module()
        with run.compiled_only():
            old = mujoco.MjModel.from_xml_string(BASE)
            new = mujoco.MjModel.from_xml_string(candidate)
            return run.compare_compiled(old, new, MAPPING)

    def test_split_preserves_all_pair_filter_eligibility_without_data(self):
        self.assertTrue(ENTRY.is_file(), 'local invariant audit is missing')
        report = self.compile_and_compare(REPLACED)
        self.assertTrue(report['passed'], report['differences'])
        self.assertEqual(report['collision_eligibility_changed'], [])
        self.assertEqual(report['collision_eligibility_pairs_checked'], 6)
        self.assertEqual(report['external_mask_eligibility_checks'], 4 * 33 * 33)

    def test_rejects_inherited_physics_control_and_keyframe_changes(self):
        self.assertTrue(ENTRY.is_file(), 'local invariant audit is missing')
        for before, after in [('damping="1"', 'damping="2"'),
                              ('gear="2"', 'gear="3"'),
                              ('mass="1"', 'mass="2"'),
                              ('ctrl=".5"', 'ctrl=".6"'),
                              ('timestep="0.002"', 'timestep="0.003"')]:
            with self.subTest(change=after):
                report = self.compile_and_compare(REPLACED.replace(before, after, 1))
                self.assertFalse(report['passed'])
                self.assertTrue(report['differences'])

    def test_rejects_replacement_contact_bit_changes(self):
        self.assertTrue(ENTRY.is_file(), 'local invariant audit is missing')
        changed = REPLACED.replace('name="new_link5_a"',
                                   'name="new_link5_a" contype="0" conaffinity="0"')
        report = self.compile_and_compare(changed)
        self.assertFalse(report['passed'])
        self.assertTrue(report['collision_eligibility_changed'])
        self.assertGreater(report['external_mask_eligibility_changed'], 0)

    def test_static_context_blocks_state_allocation_and_dynamics(self):
        self.assertTrue(ENTRY.is_file(), 'local invariant audit is missing')
        run = module()
        with run.compiled_only() as calls:
            with self.assertRaises(RuntimeError):
                mujoco.MjData(None)
            with self.assertRaises(RuntimeError):
                mujoco.mj_step(None, None)
        self.assertEqual(calls['forbidden_api_attempts'], ['MjData', 'mj_step'])

    def test_rejects_manifest_expanding_the_two_authorized_geom_identities(self):
        self.assertTrue(ENTRY.is_file(), 'local invariant audit is missing')
        run = module()
        manifest = {'removed_geoms': [{'name': 'robot0_candidate_nero_link5_4'},
                                      {'name': 'robot0_source_flange_part_2'}],
                    'new_geoms': [
                        {'old_geom': 'robot0_candidate_nero_link5_4',
                         'parent_body': 'robot0_link5', 'new_geom': 'new_left'},
                        {'old_geom': 'robot0_source_flange_part_2',
                         'parent_body': 'robot0_gripper_flange', 'new_geom': 'new_right'}]}
        self.assertEqual(run.replacement_records(manifest)[1], {
            'new_left': 'robot0_candidate_nero_link5_4',
            'new_right': 'robot0_source_flange_part_2'})
        manifest['removed_geoms'][0]['name'] = 'gripper0_proxy76_finger12_link_00'
        with self.assertRaises(ValueError):
            run.replacement_records(manifest)

    def test_preserves_primitive_frozen_bindings_without_a_mesh_asset(self):
        run = module()
        xml = ET.fromstring(BASE)
        geom = xml.find('.//geom[@name="old_link5"]')
        binding = {'mesh': None, 'file': None, 'sha256': None,
                   'geom_attributes': dict(geom.attrib)}
        self.assertFalse(run.validate_frozen_binding('old_link5', binding, xml, ENTRY))
        geom.set('size', '.05 .1 .1')
        with self.assertRaises(ValueError):
            run.validate_frozen_binding('old_link5', binding, xml, ENTRY)

    def test_rejects_primitive_manifest_claiming_a_mesh_path(self):
        run = module()
        xml = ET.fromstring(BASE)
        geom = xml.find('.//geom[@name="old_flange"]')
        binding = {'mesh': None, 'file': 'unexpected.obj', 'sha256': None,
                   'geom_attributes': dict(geom.attrib)}
        with self.assertRaises(ValueError):
            run.validate_frozen_binding('old_flange', binding, xml, ENTRY)


if __name__ == '__main__':
    unittest.main()
