"""Targeted B adapter contracts; stdlib and saved evidence, no native engine."""
from copy import deepcopy
import gzip
import hashlib
import importlib.abc
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


class NoEngine(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'mujoco', 'robosuite', 'libero', 'numpy', 'scipy', 'PIL'}:
            raise AssertionError('Native dependency in offline test: '+fullname)


sys.meta_path.insert(0, NoEngine())
import close149_drive as B


class DriveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = B.load_profile()
        with gzip.open(B.ROOT/B.A_PREFLIGHT, 'rt') as stream:
            cls.a = json.load(stream)['actual_snapshot']

    def test_profile_exact_mapping_and_resource_context(self):
        with tempfile.TemporaryDirectory() as tmp:
            record = B.materialize(Path(tmp))
            self.assertEqual(record['attribute_count'], 34)
            self.assertEqual(record['raw_sha256'], self.profile['candidate_xml_sha256_in_original_base_context'])
            self.assertEqual(len(record['changes']), 17)
            self.assertTrue(all(Path(r['resolved']).is_file() for r in record['resources']))
            self.assertTrue(all(hashlib.sha256(Path(r['resolved']).read_bytes()).hexdigest()==r['sha256'] for r in record['resources']))
            with self.assertRaises(FileExistsError):
                B.materialize(Path(tmp))

    def test_saved_snapshot_exact_values_and_bindings(self):
        expected = B.expected_snapshot(self.a, self.profile)
        self.assertTrue(B.audit_drive_snapshot(self.a, expected, self.profile)['passed'])
        self.assertNotEqual(expected, self.a)
        with self.assertRaises(ValueError):
            B.audit_drive_snapshot(self.a, self.a, self.profile)
        for corruption in ('gain', 'gear', 'friction', 'mass', 'wrist', 'binding', 'missing', 'duplicate'):
            with self.subTest(corruption=corruption):
                bad=deepcopy(expected)
                act=next(v for v in bad['objects']['ACTUATOR'] if v['bindings'].get('actuator_trnid')=='gripper0_finger11')
                joint=next(v for v in bad['objects']['JOINT'] if v['name']=='gripper0_finger11')
                if corruption=='gain': act['fields']['actuator_gainprm'][1]=.1
                elif corruption=='gear': act['fields']['actuator_gear'][0]=2
                elif corruption=='friction': joint['dofs']['dof_frictionloss'][0]=.1
                elif corruption=='mass': bad['objects']['BODY'][1]['fields']['body_mass']+=.001
                elif corruption=='wrist': next(v for v in bad['objects']['GEOM'] if (v['name'] or '').startswith('robot0_wrist_reuse_'))['fields']['geom_margin']+=.001
                elif corruption=='binding': act['bindings']['actuator_trnid']='gripper0_finger12'
                elif corruption=='missing': bad['objects']['JOINT'].remove(joint)
                elif corruption=='duplicate': bad['objects']['ACTUATOR'].append(deepcopy(act))
                with self.assertRaises(ValueError): B.audit_drive_snapshot(self.a,bad,self.profile)

    def test_actions_and_original_sources_remain_pinned(self):
        self.assertEqual(B.C.canonical_sha(B.C.planned_schedule()), self.profile['schedule_canonical_json_sha256'])
        self.assertEqual(len(B.C.planned_schedule()),260)
        self.assertEqual(B.C.summarize(B.C.planned_schedule())['physics_steps_max'],6500)
        self.assertEqual(B.R.digest(B.ROOT/'examples/right_hand/close149.py'), B.CLOSE149_SHA)
        self.assertEqual(B.R.digest(B.PROFILE_DIR/'model.patch'), B.PATCH_SHA)

    def test_reverse_compile_copy_uses_saved_native_reset_body_poses(self):
        source=(B.ROOT/B.A_PREFLIGHT.parent/'model.xml').read_text()
        xml=ET.fromstring(source)
        for change in self.profile['changes']:
            node=xml.find('.//'+change['tag']+'[@name="'+change['name']+'"]')
            for field,value in change['attribute_changes'].items(): node.set(field,value['after'])
        before=deepcopy(xml)
        result,updated=B.reverse_A_source(ET.tostring(xml,encoding='unicode'),self.profile,self.a)
        after=ET.fromstring(result)
        name='flat_stove_1_main'
        expected=next(x for x in self.a['objects']['BODY'] if x['name']==name)['fields']['body_pos']
        self.assertNotEqual([float(x) for x in before.find('.//body[@name="'+name+'"]').get('pos','0 0 0').split()],expected)
        self.assertEqual(after.find('.//body[@name="'+name+'"]').attrib,before.find('.//body[@name="'+name+'"]').attrib)
        self.assertEqual(updated[name]['body_pos'],expected)
        self.assertIn(name,updated)
        self.assertEqual(set(updated),{'flat_stove_1_main','wooden_two_layer_shelf_1_main'})
        for old,new in zip(before.iter('body'),after.iter('body')): self.assertEqual(old.attrib,new.attrib)
        for change in self.profile['changes']:
            node=after.find('.//'+change['tag']+'[@name="'+change['name']+'"]')
            for field,value in change['attribute_changes'].items(): self.assertEqual(node.get(field),value['before'])

    def test_executor_retains_frozen_integration_and_adds_readonly_cache_capture(self):
        legacy=types.SimpleNamespace(robot_audit=lambda *_:None,write=lambda *_:None)
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)
            execute=B.adapted_executor(legacy,object(),object(),output)
            source=(output/'executor.executed.py').read_text()
            self.assertIn('_drive_reset_cache(model, data, output)',source)
            self.assertIn('originals[\'mj_step\'](m, d)',source)
            self.assertIn("if counts['mj_step'] - previous_steps != 25:",source)
            self.assertIn("policy_row = _research_policy.evaluate(counts['mj_step']",source)
            self.assertIsNot(execute.__globals__['robot_audit'],legacy.robot_audit)

    def test_entry_keeps_reset_policy_and_schedule_checks(self):
        with tempfile.TemporaryDirectory() as tmp:
            entry, original, changed = B.prepared_entry(Path(tmp)/'new')
            self.assertIn("registration.update(_close149_registration)", changed)
            self.assertIn("if read(output/'schedule.json') != _close149_schedule:", changed)
            self.assertIn("for key in ('qpos', 'qvel', 'ctrl', 'controller', 'criteria', 'native_scene'):", changed)
            self.assertIs(entry.__globals__['adapted_executor'], B.adapted_executor)
            self.assertIs(entry.__globals__['selected_policy'], B.selected_policy)
            self.assertIn('_close149_source', changed)
            self.assertEqual(entry.__globals__['BASELINE'], B.BASELINE)
            self.assertEqual(original, B.C.prepared_entry()[1])

    def test_existing_output_is_never_touched(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp); sentinel=p/'keep';sentinel.write_bytes(b'original')
            with mock.patch.object(B,'prepared_entry') as entry:
                with self.assertRaises(ValueError): B.run_once(p)
                entry.assert_not_called()
            self.assertEqual(sentinel.read_bytes(),b'original')
            self.assertEqual(list(p.iterdir()),[sentinel])

    def test_exit_code_and_exception_never_retry(self):
        for value in (0,1,2,7,RuntimeError('one failure')):
            with self.subTest(value=str(value)), tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/'fresh'
                entry=mock.Mock(side_effect=value if isinstance(value,Exception) else None,
                                return_value=value)
                with mock.patch.object(B,'prepared_entry',return_value=(entry,'original','changed')):
                    if isinstance(value,Exception):
                        with self.assertRaises(RuntimeError): B.run_once(path)
                    else:self.assertEqual(B.run_once(path),value)
                entry.assert_called_once_with(path)


if __name__=='__main__': unittest.main(verbosity=2)
