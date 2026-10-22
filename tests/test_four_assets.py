"""Delivery contracts and resource closure; no simulation or native imports."""
import hashlib
import json
import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


class FourAssets(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest=json.loads((ROOT/'docs/assets.json').read_text())

    def test_four_entries_dimensions_and_common_order(self):
        entries=self.manifest['assets']
        self.assertEqual(set(entries),{'dh116_left','dh116_right','nero_dh116_left','nero_dh116_right'})
        for key,row in entries.items():
            hand=key.startswith('dh116_')
            self.assertEqual((row['state_dim'],row['action_dim']),(11,6) if hand else (18,13))
            self.assertEqual(row['state_order'][-11:],self.manifest['hand_state_order'])
            self.assertEqual(row['active_order'],self.manifest['hand_active_order'])

    def test_portable_resource_closure(self):
        for row in self.manifest['assets'].values():
            path=ROOT/row['model']
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),row['sha256'])
            for element in ET.parse(path).iter():
                if not element.get('file'):continue
                self.assertFalse(Path(element.get('file')).is_absolute())
                resource=(path.parent/element.get('file')).resolve()
                relative=str(resource.relative_to(ROOT))
                self.assertTrue(relative.startswith('meshes/'),relative)
                self.assertEqual(hashlib.sha256(resource.read_bytes()).hexdigest(),self.manifest['resources'][relative]['sha256'])

    def test_standalone_has_no_arm_and_preserves_hand_subtree(self):
        for side in ('left','right'):
            hand=ET.parse(ROOT/self.manifest['assets']['dh116_'+side]['model']).getroot()
            assembly=ET.parse(ROOT/self.manifest['assets']['nero_dh116_'+side]['model']).getroot()
            expected=assembly.find('.//body[@name="gripper0_base_link"]')
            def canonical(node):return (node.tag,sorted(node.attrib.items()),[canonical(n) for n in node])
            self.assertEqual(canonical(hand.find('worldbody/body')),canonical(expected))
            self.assertEqual(len(hand.findall('.//worldbody//joint')),11)
            self.assertEqual(len(hand.findall('actuator/*')),6)
            self.assertEqual(len(hand.findall('equality/joint')),5)
            self.assertFalse(any(b.get('name','').startswith(('robot0_','mount0_')) for b in hand.iter('body')))

    def test_left_V1_and_right_B_source_identity(self):
        self.assertEqual(hashlib.sha256((ROOT/'mjcf/nero_dh116.xml').read_bytes()).hexdigest(),
            'c446124b7d938aa6e00ba5914047001de66c2a2c162e7dea1644437e5a85cb33')
        self.assertEqual(self.manifest['baselines']['right_commit'],'7998718be17d39a2491f76ee747e07309b6b3193')
        self.assertEqual(self.manifest['assets']['nero_dh116_right']['profile'],'B + functional_v1')

    def test_fixed_palm_preserves_five_original_parent_filters(self):
        expected={('gripper0_base_link','gripper0_finger'+n+'_link') for n in ('11','21','31','41','51')}
        for side in ('left','right'):
            tree=ET.parse(ROOT/self.manifest['assets']['dh116_'+side]['model'])
            self.assertEqual({(e.get('body1'),e.get('body2')) for e in tree.findall('contact/exclude')},expected)

    def test_frozen_runtime_and_no_experiment_imports(self):
        for filename,digest in self.manifest['runtime_files'].items():
            self.assertEqual(hashlib.sha256((ROOT/filename).read_bytes()).hexdigest(),digest)
        tree=ast.parse((ROOT/'examples/control_asset.py').read_text())
        imports=set()
        for node in ast.walk(tree):
            if isinstance(node,ast.Import):imports.update(n.name.split('.')[0] for n in node.names)
            if isinstance(node,ast.ImportFrom):imports.add(node.module.split('.')[0])
        self.assertLessEqual(imports,{'argparse','json','pathlib','numpy','mujoco','robosuite'})

    def test_flange_license_notice_source_and_derivative_binding(self):
        record=json.loads((ROOT/'licenses/flange-provenance.json').read_text())
        notice=(ROOT/record['license_source']['notice_file']).read_bytes()
        blob=hashlib.sha1(b'blob '+str(len(notice)).encode()+b'\0'+notice).hexdigest()
        self.assertEqual(blob,'601f56ad41b103897cd4227a2d1ea0594b389d08')
        for row in record['licensed_source_files'].values():
            payload=(ROOT/row['local_source']).read_bytes()
            self.assertEqual(hashlib.sha256(payload).hexdigest(),row['sha256'])
            self.assertEqual(hashlib.sha1(b'blob '+str(len(payload)).encode()+b'\0'+payload).hexdigest(),row['git_blob_sha1'])
        derivatives=record['collision_derivatives']
        self.assertEqual(len(derivatives['left_eight']['files']),8)
        self.assertEqual(len(derivatives['right_159']['files']),159)
        rows=derivatives['left_eight']['files']+derivatives['right_159']['files']
        rows+=[record['visual_derivatives']['output_obj'],record['visual_derivatives']['output_texture']]
        for row in rows:self.assertEqual(hashlib.sha256((ROOT/row['file']).read_bytes()).hexdigest(),row['sha256'])


if __name__=='__main__':unittest.main(verbosity=2)
