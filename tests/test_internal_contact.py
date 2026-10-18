"""Compiler-only checks for the authorized local contact policy; no integration."""
import contextlib
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco

ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / 'examples/right_hand/internal_contact.py'


def module():
    spec = importlib.util.spec_from_file_location('internal_contact', ENTRY)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


class LocalContactTests(unittest.TestCase):
    def test_entry_exists_for_frozen_two_pair_policy(self):
        self.assertTrue(ENTRY.is_file(), 'diagnostic two-pair policy entry is missing')
        run = module()
        source = (ROOT / 'mjcf/nero_dh116_right.xml').read_bytes()
        derived = run.derive_xml(source)
        self.assertEqual(derived.replace(run.EXCLUDE_LINES, b''), source)
        self.assertEqual(len(ET.fromstring(derived).findall('contact/exclude')), 3)
        with self.assertRaises(ValueError):
            run.derive_xml(derived)

    def test_compiled_scope_is_exactly_two_pairs_and_preserves_external_masks(self):
        self.assertTrue(ENTRY.is_file(), 'diagnostic two-pair policy entry is missing')
        run = module()
        original, candidate = run.compile_models(run.derive_xml(run.REF.MODEL.read_bytes()))
        report = run.check_compiled(original, candidate)
        self.assertEqual(len(report['eligibility_changed_pairs']), 2)
        self.assertEqual(report['external_eligibility_changed'], 0)
        self.assertEqual(report['hand_collision_geoms_one_one'], 17)
        self.assertEqual(report['old_excludes_preserved'], True)
        geom = mujoco.mj_name2id(candidate, mujoco.mjtObj.mjOBJ_GEOM, 'gripper0_base_link_collision_0')
        candidate.geom_contype[geom] = 0
        with self.assertRaises(ValueError):
            run.check_compiled(original, candidate)

    def test_rejects_a_third_internal_exclusion(self):
        self.assertTrue(ENTRY.is_file(), 'diagnostic two-pair policy entry is missing')
        run = module()
        derived = run.derive_xml(run.REF.MODEL.read_bytes()).replace(
            b'  </contact>', b'    <exclude body1="gripper0_base_link" body2="gripper0_finger42_link" />\n  </contact>')
        original, candidate = run.compile_models(derived)
        with self.assertRaises(ValueError):
            run.check_compiled(original, candidate)


if __name__ == '__main__':
    unittest.main()
