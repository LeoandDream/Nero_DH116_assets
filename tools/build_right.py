#!/usr/bin/env python3
"""Build the pinned DH116 right URDF/linear-mimic variant using the standard library.

The left composite supplies NERO, its flange, mount, model options and arm motors.
The right URDF supplies all hand mechanics and linear mimic equations. Explicitly
listed simulation parameters come from the same revision's right MJCF; its
quadratic equality law and selective contact masks are intentionally not used.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[1]
COMMIT = '14275cfe32cce1fe13a16df712e8a0e610a0227e'
HAND = 'DH116/DH116_URDF_MJCF_Files/DH116-R000-A1'
URDF = HAND + '/urdf/DH116-R000-A1.urdf'
MJCF = HAND + '/mjcf/DH116-R000-A1.xml'
EXPECTED = {
    URDF: 'c3ec0a6bd0e39477554e8a1a04b7a7e44004737a5a65f86b1c02bc4848911525',
    MJCF: 'a757fdd0d3c23c274e201426acc516ce107a6df63b7f9d5a1f780eb8a0a9021c',
}
PREFIX = 'gripper0_'


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def vector(value):
    return [float(x) for x in value.split()]


def text(values):
    return ' '.join(format(x, '.17g') for x in values)


def rpy_quat(value):
    """URDF Rz(yaw) Ry(pitch) Rx(roll), written as a MuJoCo wxyz quaternion."""
    r, p, y = [x / 2 for x in vector(value)]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return text([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                 cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def origin(element):
    return {'pos': element.get('xyz', '0 0 0'), 'quat': rpy_quat(element.get('rpy', '0 0 0'))}


def write_bytes(root, relative, data):
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def pinned_bytes(upstream, relative):
    data = (upstream / relative).read_bytes()
    blob = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
    expected_blob = subprocess.check_output(['git', '-C', str(upstream), 'rev-parse', COMMIT + ':' + relative], text=True).strip()
    if blob != expected_blob or (relative in EXPECTED and sha256(data) != EXPECTED[relative]):
        raise ValueError('upstream bytes differ from pinned revision: ' + relative)
    return data


def build(upstream, output_root):
    upstream, output_root = Path(upstream).resolve(), Path(output_root).resolve()
    revision = subprocess.check_output(['git', '-C', str(upstream), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != COMMIT:
        raise ValueError('expected upstream revision ' + COMMIT + ', got ' + revision)
    raw = pinned_bytes(upstream, URDF)
    source_mjcf = pinned_bytes(upstream, MJCF)
    urdf, sim = ET.fromstring(raw), ET.fromstring(source_mjcf)
    baseline_bytes = (REPO / 'mjcf/nero_dh116.xml').read_bytes()
    model = ET.fromstring(baseline_bytes)
    model.set('model', 'nero_dh116_right_urdf_linear_coarse17')
    model.insert(0, ET.Comment('Right URDF mechanics + linear mimic; source-MJCF simulation parameters; coarse17 raw-mesh convex collisions.'))
    assets = model.find('asset')
    for item in list(assets):
        if item.get('name', '').startswith(PREFIX):
            assets.remove(item)

    mesh_records = []
    mesh_names = {}
    for link in urdf.findall('link'):
        name = link.get('name')
        filename = name + '.STL'
        upstream_path = HAND + '/meshes/' + filename
        data = pinned_bytes(upstream, upstream_path)
        local = 'meshes/dh116_right/source/' + filename
        write_bytes(output_root, local, data)
        mesh_names[name] = PREFIX + name
        ET.SubElement(assets, 'mesh', name=PREFIX + name, file='../' + local)
        mesh_records.append({'link': name, 'upstream_path': upstream_path, 'file': local, 'sha256': sha256(data), 'bytes': len(data), 'format': 'binary STL', 'scale': [1, 1, 1], 'role': 'unaltered URDF visual and collision mesh; collision uses MuJoCo convex hull'})

    links = {l.get('name'): l for l in urdf.findall('link')}
    children = {}
    for joint in urdf.findall('joint'):
        children.setdefault(joint.find('parent').get('link'), []).append(joint)
    joint_defaults = sim.find('default/joint').attrib
    geom_defaults = sim.find('default/geom').attrib
    eq_defaults = sim.find('default/equality').attrib
    active = [j.get('name') for j in urdf.findall('joint') if j.get('type') == 'revolute' and j.find('mimic') is None]

    def add_link(name, joint=None):
        attrs = {'name': PREFIX + name}
        attrs.update({'pos': '0 0 0', 'quat': '1 0 0 0'} if joint is None else origin(joint.find('origin')))
        body = ET.Element('body', attrs)
        link = links[name]
        inertial = link.find('inertial')
        if vector(inertial.find('origin').get('rpy', '0 0 0')) != [0, 0, 0]:
            raise ValueError('pinned source unexpectedly contains a rotated inertia frame')
        inertia = inertial.find('inertia')
        ET.SubElement(body, 'inertial', mass=inertial.find('mass').get('value'), pos=inertial.find('origin').get('xyz'),
                      fullinertia=' '.join(inertia.get(k) for k in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']))
        if joint is not None and joint.get('type') != 'fixed':
            limit = joint.find('limit')
            attrs = {'name': PREFIX + joint.get('name'), 'type': 'hinge', 'pos': '0 0 0',
                     'axis': joint.find('axis').get('xyz'), 'range': limit.get('lower') + ' ' + limit.get('upper')}
            attrs.update({k: joint_defaults[k] for k in ['damping', 'armature', 'frictionloss']})
            ET.SubElement(body, 'joint', attrs)
        for index, visual in enumerate(link.findall('visual')):
            attrs = {'name': PREFIX + name + '_visual_' + str(index), 'type': 'mesh', 'mesh': mesh_names[name],
                     'group': '1', 'contype': '0', 'conaffinity': '0', 'density': '0'}
            attrs.update(origin(visual.find('origin')))
            color = visual.find('material/color')
            if color is not None:
                attrs['rgba'] = color.get('rgba')
            ET.SubElement(body, 'geom', attrs)
        for index, collision in enumerate(link.findall('collision')):
            attrs = {'name': PREFIX + name + '_collision_' + str(index), 'type': 'mesh', 'mesh': mesh_names[name],
                     'group': '0', 'contype': '1', 'conaffinity': '1', 'rgba': '0 0 0.5 1'}
            attrs.update(origin(collision.find('origin')))
            attrs.update({k: geom_defaults[k] for k in ['condim', 'friction']})
            ET.SubElement(body, 'geom', attrs)
        for child_joint in children.get(name, []):
            body.append(add_link(child_joint.find('child').get('link'), child_joint))
        return body

    old_base = model.find('.//body[@name="gripper0_base_link"]')
    new_base = add_link('base_link')
    # These zero-mass body/sites are an existing application interface, not URDF mechanics.
    interface = old_base.find('body[@name="gripper0_eef"]')
    if interface is not None:
        new_base.append(copy.deepcopy(interface))
    for parent in model.iter():
        if old_base in list(parent):
            index = list(parent).index(old_base)
            parent.remove(old_base)
            parent.insert(index, new_base)
            break

    actuators = model.find('actuator')
    for actuator in list(actuators):
        if actuator.get('name', '').startswith(PREFIX):
            actuators.remove(actuator)
    for name in active:
        actuator = sim.find('actuator/position[@joint="' + name + '"]')
        joint = urdf.find('joint[@name="' + name + '"]')
        limit = joint.find('limit')
        ET.SubElement(actuators, 'position', name=PREFIX + 'urdf_act_' + name, joint=PREFIX + name,
                      ctrllimited='true', ctrlrange=limit.get('lower') + ' ' + limit.get('upper'), kp=actuator.get('kp'), kv=actuator.get('kv'))
    equalities = model.find('equality')
    for equality in list(equalities):
        if equality.get('name', '').startswith(PREFIX):
            equalities.remove(equality)
    for joint in urdf.findall('joint'):
        mimic = joint.find('mimic')
        if mimic is not None:
            attrs = {'name': PREFIX + 'urdf_mimic_' + joint.get('name'), 'joint1': PREFIX + joint.get('name'),
                     'joint2': PREFIX + mimic.get('joint'), 'polycoef': mimic.get('offset') + ' ' + mimic.get('multiplier') + ' 0 0 0'}
            attrs.update(eq_defaults)
            ET.SubElement(equalities, 'joint', attrs)
    keys = model.find('keyframe')
    arm_qpos = vector(keys.find('key').get('qpos'))[:7]
    for key in list(keys):
        keys.remove(key)
    hand_qpos = [0.0 if joint.find('mimic') is None else float(joint.find('mimic').get('offset'))
                 for joint in urdf.findall('joint') if joint.get('type') == 'revolute']
    ET.SubElement(keys, 'key', name='right_open', qpos=text(arm_qpos + hand_qpos), qvel=text([0] * 18), ctrl=text([0] * 13))

    generated = {'urdf/dh116_right.source.urdf': raw,
                 'urdf/dh116_right.urdf': raw.replace(b'package://DH116-R000-A1/meshes/', b'../meshes/dh116_right/source/')}
    ET.indent(model, space='  ')
    generated['mjcf/nero_dh116_right.xml'] = ET.tostring(model, encoding='utf-8', xml_declaration=True) + b'\n'
    for relative, data in generated.items():
        write_bytes(output_root, relative, data)
    inherited = []
    for asset in assets:
        filename = asset.get('file')
        if filename is None or asset.get('name', '').startswith(PREFIX):
            continue
        source = (REPO / 'mjcf' / filename).resolve()
        relative = source.relative_to(REPO)
        destination = output_root / relative
        if source != destination:
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
        inherited.append({'file': relative.as_posix(), 'sha256': sha256(source.read_bytes())})
    manifest = {
        'schema_version': 1, 'variant': 'DH116-R000-A1', 'definition': 'URDF mechanics and linear mimic, with separately sourced MJCF simulation parameters',
        'upstream': {'repository': 'https://github.com/sorrowfeng/leadtron_hand_descriptions', 'commit': COMMIT,
                     'urdf': {'path': URDF, 'sha256': sha256(raw)}, 'mjcf': {'path': MJCF, 'sha256': sha256(source_mjcf)}},
        'mechanical_source': {'path': URDF, 'fields': ['link mass', 'inertial origin and full inertia tensor', 'joint parent and child', 'joint origin (URDF RzRyRx to wxyz quaternion)', 'joint type', 'axis', 'lower and upper limits', 'linear mimic multiplier and offset'],
                              'counts': {'links': 17, 'joints': 16, 'moving_joints': 11, 'fixed_joints': 5, 'active_joints': 6, 'mimic_joints': 5},
                              'source_effort_velocity': 'zero values are preserved in the source URDF but are unspecified physical actuator capabilities, not applied as zero force limits'},
        'simulation_parameter_sources': {
            'joint': {'source': MJCF + ':default/joint', 'values': {k: joint_defaults[k] for k in ['damping', 'armature', 'frictionloss']}},
            'position_actuator': {'source': MJCF + ':actuator/position', 'kp': '50', 'kv': '2.0', 'ctrlrange_source': URDF + ':joint/limit lower,upper'},
            'equality_solver': {'source': MJCF + ':default/equality', 'values': dict(eq_defaults)},
            'hand_contact': {'source': MJCF + ':default/geom', 'values': {k: geom_defaults[k] for k in ['condim', 'friction']}},
            'coupling_override': {'source': URDF + ':joint/mimic', 'law': 'q_follower = offset + multiplier * q_driver', 'upstream_mjcf_quadratic_terms_used': False},
            'contact_activation_override': {'source': URDF + ':link/collision', 'contype': 1, 'conaffinity': 1, 'upstream_mjcf_selective_contact_masks_used': False},
            'model_options_and_arm_motors': {'source': 'mjcf/nero_dh116.xml', 'preservation': 'all non-hand XML fields unchanged'}},
        'collision': {'profile': 'coarse17', 'representation': '17 unmodified per-link URDF STL meshes, convexified by MuJoCo', 'collision_geoms': 17,
                      'new_contact_exclusions': [], 'precision': 'coarse convex hull approximation; not an exact concave triangle-surface collision model', 'left_hand_collision_proxies_used': False},
        'interface': {'parent': 'robot0_gripper_flange', 'hand_base_transform': {'pos': [0, 0, 0], 'quat_wxyz': [1, 0, 0, 0]},
                      'source': 'existing left composite application interface; base mesh mounting interface remains at its source origin',
                      'copied_metadata': ['gripper0_eef zero-mass body', 'gripper0_grip_site', 'gripper0_grip_site_cylinder', 'gripper0_ft_frame', 'existing force/torque sensors']},
        'active_control_order': active, 'state_dim': 18, 'control_dim': 13,
        'control_semantics': 'first 7 motor torques in Nm; final 6 independent joint angle targets in rad; passive mimic joints have no control channel',
        'reset': {'keyframe': 'right_open', 'arm': 'first 7 qpos of left composite keyframe', 'hand_active': 'URDF lower limits (all zero)', 'hand_followers': 'URDF linear mimic offsets', 'ctrl': [0] * 13},
        'composite_template': {'file': 'mjcf/nero_dh116.xml', 'sha256': sha256(baseline_bytes)},
        'meshes': mesh_records, 'inherited_nonhand_assets': inherited,
        'generated_files': [{'file': relative, 'sha256': sha256(data)} for relative, data in generated.items()],
        'reproduce': 'python tools/build_right.py --upstream /path/to/pinned/leadtron_hand_descriptions --output-root /path/to/output',
    }
    write_bytes(output_root, 'RIGHT_SOURCE_MANIFEST.json', (json.dumps(manifest, indent=2) + '\n').encode())
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', required=True, type=Path, help='clean checkout of the pinned upstream revision')
    parser.add_argument('--output-root', type=Path, default=REPO, help='output asset root; required non-hand assets are copied when building elsewhere')
    args = parser.parse_args()
    result = build(args.upstream, args.output_root)
    print('Built nero_dh116_right.xml: URDF mechanics/linear mimic, ' + result['collision']['profile'] + ', 18 qpos / 13 controls')


if __name__ == '__main__':
    main()
