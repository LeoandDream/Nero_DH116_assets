"""Independent source-to-compiled checks for the DH116 right asset variant."""
import hashlib
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'mjcf/nero_dh116_right.xml'
RAW = ROOT / 'urdf/dh116_right.source.urdf'
ACTIVE = ['finger11', 'finger12', 'finger21', 'finger31', 'finger41', 'finger51']


def numbers(value):
    return np.fromstring(value, sep=' ')


def rotation_rpy(value):
    """URDF fixed-axis rotation, independently using matrix multiplication."""
    x, y, z = numbers(value)
    rx = np.array([[1, 0, 0], [0, math.cos(x), -math.sin(x)], [0, math.sin(x), math.cos(x)]])
    ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    rz = np.array([[math.cos(z), -math.sin(z), 0], [math.sin(z), math.cos(z), 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_rotation(axis, angle):
    a = np.array(axis)
    cross = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)


def stripped(element):
    """Compare every non-hand XML field, independent of serialization spacing."""
    if element.get('name', '').startswith('gripper0_') or element.tag == 'keyframe':
        return None
    attrs = dict(element.attrib)
    if element.tag == 'mujoco':
        attrs.pop('model')
    children = [s for c in element if (s := stripped(c)) is not None]
    return element.tag, attrs, children


class RightModelTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(MODEL.is_file(), 'right MJCF has not been generated')
        self.assertTrue(RAW.is_file(), 'immutable right source URDF is missing')
        self.urdf = ET.parse(RAW).getroot()
        self.xml = ET.parse(MODEL).getroot()
        self.model = mujoco.MjModel.from_xml_path(str(MODEL))

    def test_original_left_and_nonhand_physics_remain_unchanged(self):
        self.assertEqual(stripped(ET.parse(ROOT / 'mjcf/nero_dh116.xml').getroot()), stripped(self.xml))
        for line in (ROOT / 'SHA256SUMS').read_text().splitlines():
            digest, filename = line.split(maxsplit=1)
            filename = filename.lstrip('*')
            if filename.startswith(('meshes/dh116/', 'meshes/nero/', 'meshes/mount/', 'urdf/dh116_left')) or filename in {'mjcf/nero_dh116.xml', 'SOURCE_MANIFEST.json', 'URDF_SOURCES.json'}:
                self.assertEqual(hashlib.sha256((ROOT / filename).read_bytes()).hexdigest(), digest, filename)

    def test_resource_closure_and_source_bytes(self):
        manifest = json.loads((ROOT / 'RIGHT_SOURCE_MANIFEST.json').read_text())
        self.assertEqual(manifest['upstream']['commit'], '14275cfe32cce1fe13a16df712e8a0e610a0227e')
        self.assertEqual(manifest['upstream']['urdf']['sha256'], 'c3ec0a6bd0e39477554e8a1a04b7a7e44004737a5a65f86b1c02bc4848911525')
        self.assertEqual(manifest['upstream']['mjcf']['sha256'], 'a757fdd0d3c23c274e201426acc516ce107a6df63b7f9d5a1f780eb8a0a9021c')
        self.assertEqual(hashlib.sha256(RAW.read_bytes()).hexdigest(), 'c3ec0a6bd0e39477554e8a1a04b7a7e44004737a5a65f86b1c02bc4848911525')
        prefix = b'package://DH116-R000-A1/meshes/'
        self.assertEqual((ROOT / 'urdf/dh116_right.urdf').read_bytes(), RAW.read_bytes().replace(prefix, b'../meshes/dh116_right/source/'))
        self.assertEqual(len(manifest['meshes']), 17)
        for item in manifest['meshes']:
            self.assertEqual(hashlib.sha256((ROOT / item['file']).read_bytes()).hexdigest(), item['sha256'])
        for asset in self.xml.findall('asset/*'):
            if asset.get('file'):
                self.assertTrue((MODEL.parent / asset.get('file')).is_file(), asset.attrib)
        self.assertFalse(any('moderate' in a.get('name', '') or 'proxy' in a.get('name', '') for a in self.xml.findall('asset/*') if a.get('name', '').startswith('gripper0_')))

    def test_mass_and_full_inertia_match_raw_urdf(self):
        for link in self.urdf.findall('link'):
            body = self.model.body('gripper0_' + link.get('name'))
            inertial = link.find('inertial')
            self.assertAlmostEqual(float(body.mass[0]), float(inertial.find('mass').get('value')), places=13)
            np.testing.assert_allclose(body.ipos, numbers(inertial.find('origin').get('xyz')), rtol=0, atol=1e-14)
            a = inertial.find('inertia').attrib
            source = np.array([[float(a['ixx']), float(a['ixy']), float(a['ixz'])], [float(a['ixy']), float(a['iyy']), float(a['iyz'])], [float(a['ixz']), float(a['iyz']), float(a['izz'])]])
            Q = np.empty(9)
            mujoco.mju_quat2Mat(Q, body.iquat)
            Q = Q.reshape(3, 3)
            serialized = self.xml.find('.//body[@name="gripper0_' + link.get('name') + '"]/inertial')
            np.testing.assert_array_equal(numbers(serialized.get('fullinertia')), [float(a[k]) for k in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']])
            # MuJoCo 3.2.3's finite-precision eigensolver can drop tiny products
            # of inertia. Check exact source components above, then the compiled
            # tensor as a numerical diagnostic only. The observed maximum error
            # for both pinned upstream and generated models is 1.662e-11.
            np.testing.assert_allclose(Q @ np.diag(body.inertia) @ Q.T, source, rtol=0, atol=2e-11)

    def test_independent_urdf_fk_at_open_and_asymmetric_angles(self):
        data = mujoco.MjData(self.model)
        for ratios in ([0, 0, 0, 0, 0, 0], [.73, .27, .15, .82, .41, .63]):
            mujoco.mj_resetDataKeyframe(self.model, data, 0)
            q = {}
            for name, ratio in zip(ACTIVE, ratios):
                joint = self.urdf.find(f'joint[@name="{name}"]')
                q[name] = ratio * float(joint.find('limit').get('upper'))
            expected = {'base_link': np.eye(4)}
            for joint in self.urdf.findall('joint'):
                name = joint.get('name')
                mimic = joint.find('mimic')
                if mimic is not None:
                    q[name] = float(mimic.get('offset')) + float(mimic.get('multiplier')) * q[mimic.get('joint')]
                origin = joint.find('origin')
                H = np.eye(4)
                H[:3, :3] = rotation_rpy(origin.get('rpy'))
                H[:3, 3] = numbers(origin.get('xyz'))
                J = np.eye(4)
                if joint.get('type') == 'revolute':
                    J[:3, :3] = axis_rotation(numbers(joint.find('axis').get('xyz')), q[name])
                    jid = self.model.joint('gripper0_' + name).id
                    data.qpos[self.model.jnt_qposadr[jid]] = q[name]
                expected[joint.find('child').get('link')] = expected[joint.find('parent').get('link')] @ H @ J
            mujoco.mj_forward(self.model, data)
            base = self.model.body('gripper0_base_link').id
            Rbase = data.xmat[base].reshape(3, 3)
            for link, H in expected.items():
                bid = self.model.body('gripper0_' + link).id
                np.testing.assert_allclose(data.xpos[bid], data.xpos[base] + Rbase @ H[:3, 3], atol=2e-12, rtol=0)
                np.testing.assert_allclose(data.xmat[bid].reshape(3, 3), Rbase @ H[:3, :3], atol=2e-12, rtol=0)

    def test_joint_axes_limits_fixed_links_and_linear_coupling(self):
        self.assertEqual((self.model.nq, self.model.nv, self.model.nu, self.model.neq), (18, 18, 13, 5))
        self.assertEqual(len(self.urdf.findall('link')), 17)
        self.assertEqual(len(self.urdf.findall('joint')), 16)
        for joint in self.urdf.findall('joint'):
            name = 'gripper0_' + joint.get('name')
            if joint.get('type') == 'fixed':
                self.assertEqual(self.model.body('gripper0_' + joint.find('child').get('link')).jntnum[0], 0)
                continue
            jid = self.model.joint(name).id
            np.testing.assert_allclose(self.model.jnt_axis[jid], numbers(joint.find('axis').get('xyz')), atol=0, rtol=0)
            lim = joint.find('limit')
            np.testing.assert_allclose(self.model.jnt_range[jid], [float(lim.get('lower')), float(lim.get('upper'))], atol=0, rtol=0)
            mimic = joint.find('mimic')
            if mimic is not None:
                eq = self.model.equality('gripper0_urdf_mimic_' + joint.get('name'))
                np.testing.assert_array_equal(eq.data[:5], [float(mimic.get('offset')), float(mimic.get('multiplier')), 0, 0, 0])
                self.assertEqual(self.model.joint(int(eq.obj2id[0])).name, 'gripper0_' + mimic.get('joint'))

    def test_explicit_source_simulation_parameters_and_coarse17_collision(self):
        collision = [g for g in self.xml.findall('.//geom') if g.get('name', '').startswith('gripper0_') and g.get('group') != '1']
        self.assertEqual(len(collision), 17)
        manifest = json.loads((ROOT / 'RIGHT_SOURCE_MANIFEST.json').read_text())
        provenance = manifest['simulation_parameter_sources']
        self.assertEqual(provenance['joint']['values'], {'damping': '2.0', 'armature': '0.01', 'frictionloss': '0'})
        self.assertEqual(provenance['hand_contact']['values'], {'condim': '4', 'friction': '1.0 0.005 0.001'})
        for g in collision:
            self.assertEqual(g.get('contype'), '1')
            self.assertEqual(g.get('conaffinity'), '1')
            self.assertEqual(g.get('type'), 'mesh')
            gid = self.model.geom(g.get('name')).id
            np.testing.assert_array_equal(self.model.geom_friction[gid], [1, .005, .001])
            self.assertEqual(self.model.geom_condim[gid], 4)
        for joint in self.urdf.findall('joint[@type="revolute"]'):
            jid = self.model.joint('gripper0_' + joint.get('name')).id
            did = self.model.jnt_dofadr[jid]
            self.assertEqual(self.model.dof_damping[did], 2)
            self.assertEqual(self.model.dof_armature[did], .01)
            self.assertEqual(self.model.dof_frictionloss[did], 0)
            self.assertFalse(self.model.jnt_actfrclimited[jid])
        for i, name in enumerate(ACTIVE, 7):
            self.assertEqual(self.model.joint(int(self.model.actuator_trnid[i, 0])).name, 'gripper0_' + name)
            self.assertEqual(self.model.actuator_gainprm[i, 0], 50)
            np.testing.assert_array_equal(self.model.actuator_biasprm[i, 1:3], [-50, -2])
            np.testing.assert_array_equal(self.model.actuator_ctrlrange[i], self.model.joint('gripper0_' + name).range)
        np.testing.assert_allclose(self.model.eq_solref, np.tile([.004, 1], (5, 1)), rtol=0, atol=0)
        np.testing.assert_allclose(self.model.eq_solimp, np.tile([.9, .99, .001, .5, 2], (5, 1)), rtol=0, atol=0)

    def test_initial_state_uses_open_active_joints_and_mimic_offsets(self):
        left = mujoco.MjModel.from_xml_path(str(ROOT / 'mjcf/nero_dh116.xml'))
        np.testing.assert_array_equal(self.model.key_qpos[0, :7], left.key_qpos[0, :7])
        np.testing.assert_array_equal(self.model.key_ctrl[0], np.zeros(13))
        for joint in self.urdf.findall('joint[@type="revolute"]'):
            mimic = joint.find('mimic')
            expected = 0 if mimic is None else float(mimic.get('offset'))
            jid = self.model.joint('gripper0_' + joint.get('name')).id
            self.assertEqual(self.model.key_qpos[0, self.model.jnt_qposadr[jid]], expected)
        base = self.xml.find('.//body[@name="gripper0_base_link"]')
        self.assertEqual(base.get('pos'), '0 0 0')
        self.assertEqual(base.get('quat'), '1 0 0 0')



if __name__ == '__main__':
    unittest.main()
