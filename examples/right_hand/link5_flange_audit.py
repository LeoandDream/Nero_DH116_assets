#!/usr/bin/env python3
"""Compile-only invariants for the single authorized link5/flange local split."""
import argparse
from contextlib import contextmanager
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
OLD = ROOT / 'examples/right_hand/candidates/finger13_local/model.xml'
OLD_SHA256 = 'b27f4f86fbaeacd2bb925463f39f67181ef7db12963c4d9f7fc01797a7924d1b'
AUTHORIZED = {'robot0_candidate_nero_link5_4': 'robot0_link5',
              'robot0_source_flange_part_2': 'robot0_gripper_flange'}
CONTACT_FIELDS = ('geom_contype', 'geom_conaffinity', 'geom_condim', 'geom_friction',
                  'geom_solref', 'geom_solimp', 'geom_margin', 'geom_gap',
                  'geom_priority', 'geom_solmix', 'geom_group')


def _module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


STATIC = _module('link5_flange_static_guard', 'examples/right_hand/proxy76_contacts.py')
CHECKS = _module('link5_flange_compiled_fields', 'examples/libero_reference/checks.py')
sha = STATIC.sha


@contextmanager
def compiled_only():
    """Reuse the static API guard and additionally block state allocation and FK."""
    with STATIC.static_only() as calls:
        originals = {name: getattr(mujoco, name)
                     for name in ('MjData', 'mj_kinematics', 'mj_geomDistance')}

        def blocked(name):
            def call(*args, **kwargs):
                calls['forbidden_api_attempts'].append(name)
                raise RuntimeError('Forbidden API in compile-only audit: ' + name)
            return call

        try:
            for name in originals:
                setattr(mujoco, name, blocked(name))
            calls['allowed_apis'] = ['mj_id2name', 'mj_name2id']
            calls['blocked_apis'] = sorted(set(calls['blocked_apis']) | set(originals))
            calls['data_allocations'] = 0
            yield calls
        finally:
            for name, original in originals.items():
                setattr(mujoco, name, original)


def replacement_records(manifest):
    """Accept a partition of exactly the two frozen identities and original bodies."""
    removed = manifest.get('removed_geoms', [])
    if (len(removed) != 2 or {row.get('name') for row in removed} != set(AUTHORIZED)):
        raise ValueError('Manifest must remove exactly the authorized link5/flange geoms')
    records = manifest.get('new_geoms', [])
    mapping = {}
    for row in records:
        old, new = row.get('old_geom'), row.get('new_geom')
        if (old not in AUTHORIZED or row.get('parent_body') != AUTHORIZED[old]
                or not new or new in mapping or new in AUTHORIZED):
            raise ValueError('Invalid replacement identity or parent body')
        mapping[new] = old
    if set(mapping.values()) != set(AUTHORIZED):
        raise ValueError('Both original geoms need at least one replacement')
    return records, mapping


def normalized_xml(path, remove_geoms=(), remove_meshes=()):
    """Compare XML order, all inherited declarations, resolved paths and file bytes."""
    path = Path(path).resolve()
    xml = ET.parse(path).getroot()
    xml.attrib.pop('model', None)
    for node in xml.findall('.//*[@file]'):
        resource = (path.parent / node.get('file')).resolve()
        if not resource.is_relative_to(ROOT):
            raise ValueError('Resource outside checkout: ' + str(resource))
        node.set('file', resource.relative_to(ROOT).as_posix() + ':' + sha(resource))
    for parent in xml.iter():
        for node in list(parent):
            if ((node.tag == 'geom' and node.get('name') in remove_geoms)
                    or (parent.tag == 'asset' and node.tag == 'mesh'
                        and node.get('name') in remove_meshes)):
                parent.remove(node)

    def value(node):
        return [node.tag, sorted(node.attrib.items()), [value(child) for child in node]]

    return value(xml)


def _geom_ids(model):
    result = {}
    for index in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        if not name or name in result:
            raise ValueError('Expected unique named geoms')
        result[name] = index
    return result


def compare_compiled(old, new, new_to_old):
    """Compare named mechanics, then every candidate pair against its old identity."""
    differences = []
    count = 0

    def check(label, before, after):
        nonlocal count
        count += 1
        if not np.array_equal(np.asarray(before), np.asarray(after)):
            differences.append({'field': label, 'old': before, 'new': after})

    a, b = CHECKS.compiled_snapshot(old), CHECKS.compiled_snapshot(new)
    removed = set(new_to_old.values())
    for category, objects in a['objects'].items():
        # Non-geom indices and unnamed object order remain exact. Geom indices may
        # shift only because the authorized two shapes become several shapes.
        old_items = {row['name'] or '@' + str(row['index']): row for row in objects}
        new_items = {row['name'] or '@' + str(row['index']): row
                     for row in b['objects'][category]}
        expected = ((set(old_items) - removed) | set(new_to_old)
                    if category == 'GEOM' else set(old_items))
        check(category + '.identity_inventory', sorted(expected), sorted(new_items))
        for name, item in old_items.items():
            if category == 'GEOM' and name in removed:
                continue
            other = new_items.get(name)
            if other is None:
                continue
            for section in ('fields', 'bindings', 'dofs'):
                before, after = item.get(section, {}), other.get(section, {})
                check(category + '.' + name + '.' + section + '.inventory',
                      sorted(before), sorted(after))
                for field, value in before.items():
                    check(category + '.' + name + '.' + field, value, after.get(field))
            for key in ('qpos_address', 'dof_address'):
                if key in item:
                    check(category + '.' + name + '.' + key, item[key], other.get(key))
            if category != 'GEOM':
                check(category + '.' + name + '.index', item['index'], other['index'])
    check('options.inventory', sorted(a['options']), sorted(b['options']))
    for field, value in a['options'].items():
        check('options.' + field, value, b['options'].get(field))
    for field in ('nq', 'nv', 'nu', 'na', 'nbody', 'njnt', 'neq', 'nexclude',
                  'nsensor', 'nkey', 'npair', 'nsite', 'ncam', 'nmocap'):
        check(field, int(getattr(old, field)), int(getattr(new, field)))
    for field in ('exclude_signature', 'key_time', 'key_qpos', 'key_qvel', 'key_act',
                  'key_ctrl', 'key_mpos', 'key_mquat', 'qpos0', 'qpos_spring',
                  'body_parentid', 'body_weldid', 'dof_bodyid', 'dof_jntid',
                  'dof_parentid', 'dof_Madr'):
        check(field, getattr(old, field).tolist(), getattr(new, field).tolist())
    check('no_explicit_geom_pairs', [int(old.npair), int(new.npair)], [0, 0])

    old_ids, new_ids = _geom_ids(old), _geom_ids(new)
    for name, origin in new_to_old.items():
        if origin not in old_ids or name not in new_ids:
            differences.append({'field': 'replacement_identity', 'old': origin, 'new': name})
            continue
        old_id, new_id = old_ids[origin], new_ids[name]
        check(name + '.body', int(old.geom_bodyid[old_id]), int(new.geom_bodyid[new_id]))
        for field in CONTACT_FIELDS:
            check(name + '.' + field, getattr(old, field)[old_id].tolist(),
                  getattr(new, field)[new_id].tolist())

    eligible_changes, pairs_checked = [], 0
    candidate_ids = list(new_ids.items())
    for i, (first, first_id) in enumerate(candidate_ids):
        for second, second_id in candidate_ids[i + 1:]:
            old_first, old_second = new_to_old.get(first, first), new_to_old.get(second, second)
            if old_first not in old_ids or old_second not in old_ids:
                continue
            before = STATIC.eligible(old, old_ids[old_first], old_ids[old_second])
            after = STATIC.eligible(new, first_id, second_id)
            pairs_checked += 1
            if before != after:
                eligible_changes.append({'candidate_geoms': [first, second],
                                         'baseline_geoms': [old_first, old_second],
                                         'old': before, 'new': after})
    check('collision_eligibility_changes', eligible_changes, [])
    external_checked, external_changed = 0, 0
    bits = [0] + [1 << bit for bit in range(32)]
    for name, new_id in candidate_ids:
        origin = new_to_old.get(name, name)
        if origin not in old_ids:
            continue
        old_id = old_ids[origin]
        ct_old, ca_old = int(old.geom_contype[old_id]), int(old.geom_conaffinity[old_id])
        ct_new, ca_new = int(new.geom_contype[new_id]), int(new.geom_conaffinity[new_id])
        for contype in bits:
            for affinity in bits:
                before = bool((ct_old & affinity) or (contype & ca_old))
                after = bool((ct_new & affinity) or (contype & ca_new))
                external_checked += 1
                external_changed += before != after
    check('external_mask_eligibility_changed', external_changed, 0)
    return {'passed': not differences, 'protected_fields_checked': count,
            'differences': differences, 'old_ngeom': int(old.ngeom),
            'new_ngeom': int(new.ngeom), 'collision_eligibility_pairs_checked': pairs_checked,
            'collision_eligibility_changed': eligible_changes,
            'external_mask_eligibility_checks': external_checked,
            'external_mask_eligibility_changed': external_changed}


def validate_frozen_binding(name, binding, xml, path):
    """Preserve a frozen mesh binding or exact primitive attributes; return mesh status."""
    node = xml.find('.//geom[@name="' + name + '"]')
    if node is None or node.get('mesh') != binding['mesh']:
        raise ValueError('Frozen geom mesh mapping changed: ' + name)
    if binding['mesh'] is None:
        if (binding.get('file') is not None or binding.get('sha256') is not None
                or node.get('type') == 'mesh'
                or node.attrib != binding.get('geom_attributes')):
            raise ValueError('Frozen primitive attributes changed: ' + name)
        return False
    asset = xml.find('asset/mesh[@name="' + binding['mesh'] + '"]')
    if asset is None:
        raise ValueError('Frozen geom mesh asset missing: ' + name)
    resource = (path.parent / asset.get('file')).resolve()
    if resource != (ROOT / binding['file']).resolve() or sha(resource) != binding['sha256']:
        raise ValueError('Frozen mesh path or bytes changed: ' + name)
    return True


def audit(candidate):
    candidate = Path(candidate).resolve()
    manifest = json.loads((candidate.parent / 'manifest.json').read_text())
    if (sha(OLD) != OLD_SHA256 or manifest.get('baseline_model_sha256') != OLD_SHA256
            or sha(candidate) != manifest.get('candidate_model_sha256')):
        raise ValueError('Frozen baseline or candidate identity changed')
    records, mapping = replacement_records(manifest)
    old_xml, new_xml = ET.parse(OLD).getroot(), ET.parse(candidate).getroot()
    old_geoms = {node.get('name'): (body.get('name'), node)
                 for body in old_xml.iter('body') for node in body.findall('geom')}
    new_geoms = {node.get('name'): (body.get('name'), node)
                 for body in new_xml.iter('body') for node in body.findall('geom')}
    for removed in manifest['removed_geoms']:
        if old_geoms[removed['name']][1].attrib != removed:
            raise ValueError('Original geom template changed: ' + removed['name'])
    mesh_names = set()
    for row in records:
        name, origin = row['new_geom'], row['old_geom']
        mesh = row.get('mesh', name)
        if mesh in mesh_names:
            raise ValueError('Replacement mesh names must be unique')
        mesh_names.add(mesh)
        body, geom = new_geoms[name]
        expected = dict(old_geoms[origin][1].attrib, name=name, mesh=mesh)
        if body != AUTHORIZED[origin] or geom.attrib != expected:
            raise ValueError('Replacement contact/pose/body template changed: ' + name)
        asset = new_xml.find('asset/mesh[@name="' + mesh + '"]')
        if asset is None:
            raise ValueError('Missing replacement mesh: ' + mesh)
        resource = (candidate.parent / asset.get('file')).resolve()
        declared = (ROOT / row['new_file']).resolve()
        if (not resource.is_relative_to(ROOT) or resource != declared
                or sha(resource) != row['sha256']):
            raise ValueError('Replacement mesh binding or bytes changed: ' + name)
    if normalized_xml(OLD, AUTHORIZED) != normalized_xml(candidate, mapping, mesh_names):
        raise ValueError('XML or inherited asset bytes outside the two local shapes changed')
    frozen = manifest.get('frozen_geom_names_and_mesh_sha256', {})
    expected_names = {name for name, (_, geom) in old_geoms.items()
                      if geom.get('group', '0') == '0'} - set(AUTHORIZED)
    if len(expected_names) != 138 or set(frozen) != expected_names:
        raise ValueError('Expected exactly all 138 unchanged collision geoms')
    mesh_count = 0
    for name, binding in frozen.items():
        for xml, path in ((old_xml, OLD), (new_xml, candidate)):
            validate_frozen_binding(name, binding, xml, path)
        mesh_count += binding['mesh'] is not None
    if mesh_count != 135 or len(frozen) - mesh_count != 3:
        raise ValueError('Expected 135 frozen collision meshes and three mount primitives')
    excludes = [node.attrib for node in new_xml.findall('contact/exclude')]
    if (excludes != [{'body1': 'robot0_base_link', 'body2': 'robot0_link1'}]
            or new_xml.findall('contact/pair')):
        raise ValueError('The original sole base/link1 exclusion must remain exact')
    with compiled_only() as calls:
        old = mujoco.MjModel.from_xml_path(str(OLD))
        new = mujoco.MjModel.from_xml_path(str(candidate))
        result = compare_compiled(old, new, mapping)
        if old.nu != 13 or new.nu != 13 or old.neq != 5 or new.neq != 5:
            raise ValueError('Expected unchanged 13 actuators and five equalities')
        result.update({'schema': 'link5-flange-local-compiled-invariants-v1',
                       'baseline_model_sha256': OLD_SHA256,
                       'candidate_model_sha256': sha(candidate),
                       'authorized_removed_geoms': sorted(AUTHORIZED),
                       'replacement_geoms': mapping,
                       'frozen_hand_collision_geoms': 78,
                       'frozen_nero_collision_geoms': 50,
                       'frozen_flange_collision_geoms': 7,
                       'frozen_mount_collision_geoms': 3,
                       'frozen_collision_geoms': len(frozen),
                       'frozen_collision_mesh_bindings': mesh_count,
                       'frozen_primitive_collision_geoms': len(frozen) - mesh_count,
                       'all_inherited_asset_declarations_paths_and_bytes_exact': True,
                       'normalized_xml_outside_authorized_geometry_exact': True,
                       'new_geom_contact_pose_templates_exact': True,
                       'static_api_audit': calls,
                       'physics_steps': 0,
                       'scope': 'Compile-only mechanics, asset and filtering invariants; no dynamics.'})
    if not result['passed']:
        raise AssertionError(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit('Use a new output file')
    result = audit(args.candidate)
    with args.output.open('x') as stream:
        stream.write(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: result[key] for key in
                      ('passed', 'protected_fields_checked', 'old_ngeom', 'new_ngeom')}))


if __name__ == '__main__':
    main()
