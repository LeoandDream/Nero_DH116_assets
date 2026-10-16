"""Portable source and exact pre-step runtime audits; never modifies simulation.

The compiled fixture records the D1-A verified model's named physical fields,
not executable MJCF. It is a comparison target, never a model input.
"""
from __future__ import annotations

import gzip
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

DIMENSIONS = ('nq', 'nv', 'nu', 'na', 'nbody', 'njnt', 'ngeom', 'neq', 'nexclude', 'nsensor')
FIELDS = {
    'BODY': ('nbody', ('body_pos', 'body_quat', 'body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'body_gravcomp')),
    'JOINT': ('njnt', ('jnt_type', 'jnt_pos', 'jnt_axis', 'jnt_range', 'jnt_limited', 'jnt_stiffness',
                      'jnt_solref', 'jnt_solimp', 'jnt_margin', 'jnt_actfrclimited', 'jnt_actfrcrange', 'jnt_actgravcomp')),
    'GEOM': ('ngeom', ('geom_type', 'geom_size', 'geom_pos', 'geom_quat', 'geom_friction', 'geom_solref',
                      'geom_solimp', 'geom_margin', 'geom_gap', 'geom_contype', 'geom_conaffinity',
                      'geom_condim', 'geom_group', 'geom_priority', 'geom_solmix')),
    'ACTUATOR': ('nu', ('actuator_trntype', 'actuator_dyntype', 'actuator_gaintype', 'actuator_biastype',
                        'actuator_dynprm', 'actuator_gainprm', 'actuator_biasprm', 'actuator_gear',
                        'actuator_ctrllimited', 'actuator_ctrlrange', 'actuator_forcelimited',
                        'actuator_forcerange', 'actuator_actlimited', 'actuator_actrange',
                        'actuator_actearly', 'actuator_cranklength', 'actuator_lengthrange')),
    'EQUALITY': ('neq', ('eq_type', 'eq_data', 'eq_active0', 'eq_solref', 'eq_solimp')),
    'SITE': ('nsite', ('site_type', 'site_size', 'site_pos', 'site_quat', 'site_group')),
    'CAMERA': ('ncam', ('cam_pos', 'cam_quat', 'cam_fovy', 'cam_ipd')),
    'SENSOR': ('nsensor', ('sensor_type', 'sensor_datatype', 'sensor_dim', 'sensor_cutoff', 'sensor_noise')),
}
DOF_FIELDS = ('dof_damping', 'dof_armature', 'dof_frictionloss', 'dof_solref', 'dof_solimp')
OPTION_FIELDS = ('timestep', 'apirate', 'impratio', 'tolerance', 'ls_tolerance', 'noslip_tolerance',
                 'ccd_tolerance', 'gravity', 'wind', 'magnetic', 'density', 'viscosity', 'o_margin',
                 'o_solref', 'o_solimp', 'o_friction', 'integrator', 'cone', 'jacobian', 'solver',
                 'iterations', 'ls_iterations', 'noslip_iterations', 'ccd_iterations',
                 'disableflags', 'enableflags', 'disableactuator')
CONTROLLER_FIELDS = ('goal_qpos', 'initial_joint', 'joint_pos', 'joint_vel', 'kp', 'kd', 'damping_ratio',
                     'output_min', 'output_max', 'input_min', 'input_max', 'position_limits',
                     'actuator_limits', 'policy_freq', 'control_dim', 'interpolator', 'new_update')


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _json_value(value):
    import numpy as np
    return np.asarray(value).tolist()


def _identity(model, kind, index):
    import mujoco
    return 'none' if int(index) < 0 else (mujoco.mj_id2name(model, getattr(mujoco.mjtObj, 'mjOBJ_' + kind), int(index)) or f'unnamed_{index}')


def _bindings(model, category, index):
    """IDs are converted to names so unused asset pruning cannot hide/remap physics."""
    import mujoco
    values = {}
    if category == 'BODY':
        values['body_parentid'] = ('BODY', int(model.body_parentid[index]))
    elif category == 'JOINT':
        values['jnt_bodyid'] = ('BODY', int(model.jnt_bodyid[index]))
    elif category == 'GEOM':
        values['geom_bodyid'] = ('BODY', int(model.geom_bodyid[index]))
        if int(model.geom_type[index]) == int(mujoco.mjtGeom.mjGEOM_MESH):
            values['mesh_binding.geom_dataid'] = ('MESH', int(model.geom_dataid[index]))
    elif category == 'ACTUATOR' and int(model.actuator_trntype[index]) in (0, 1):
        values['actuator_trnid'] = ('JOINT', int(model.actuator_trnid[index, 0]))
    elif category == 'EQUALITY' and int(model.eq_type[index]) == int(mujoco.mjtEq.mjEQ_JOINT):
        for field in ('eq_obj1id', 'eq_obj2id'):
            values[field] = ('JOINT', int(getattr(model, field)[index]))
    return {field: _identity(model, kind, value) for field, (kind, value) in values.items()}


def compiled_snapshot(model):
    """Capture the same physical scope as D1-A's 5806 checks, without stepping."""
    import mujoco
    objects = {}
    for category, (count, fields) in FIELDS.items():
        kind = getattr(mujoco.mjtObj, 'mjOBJ_' + category)
        objects[category] = []
        for index in range(getattr(model, count)):
            item = {'name': mujoco.mj_id2name(model, kind, index), 'index': index,
                    'fields': {field: _json_value(getattr(model, field)[index]) for field in fields},
                    'bindings': _bindings(model, category, index)}
            if category == 'JOINT':
                start = int(model.jnt_dofadr[index])
                count_dof = {0: 6, 1: 3, 2: 1, 3: 1}[int(model.jnt_type[index])]
                item['dofs'] = {field: _json_value(getattr(model, field)[start:start + count_dof]) for field in DOF_FIELDS}
                item['qpos_address'] = int(model.jnt_qposadr[index])
                item['dof_address'] = start
            objects[category].append(item)
    return {'schema': 1, 'mujoco_version': mujoco.__version__,
            'dimensions': {field: int(getattr(model, field)) for field in DIMENSIONS},
            'objects': objects,
            'options': {field: _json_value(getattr(model.opt, field)) for field in OPTION_FIELDS if hasattr(model.opt, field)},
            'exclude_signature': model.exclude_signature.tolist()}


def controller_snapshot(controller):
    import numpy as np
    if controller is None:
        return None
    backend = getattr(controller, 'backend', controller)
    state = {'wrapper_class': type(controller).__name__, 'backend_class': type(backend).__name__}
    for field in CONTROLLER_FIELDS:
        if hasattr(backend, field):
            value = getattr(backend, field)
            if isinstance(value, (str, int, float, bool, type(None))):
                state[field] = value
            elif isinstance(value, (np.ndarray, list, tuple)):
                state[field] = np.asarray(value).tolist()
            else:
                state[field] = type(value).__name__
    return state


def audit_sources(asset_root, fixtures_dir):
    """Verify official source bytes, actual named mesh files, and bundled fixtures."""
    asset_root, fixtures_dir = Path(asset_root), Path(fixtures_dir)
    errors, files, meshes = [], [], []
    identity = json.loads((fixtures_dir / 'official-assets.json').read_text())
    manifest = json.loads((fixtures_dir / 'manifest.json').read_text())
    for base, records, label in ((asset_root, identity['files'], 'official'),
                                  (fixtures_dir, manifest['files'], 'fixture'),
                                  (fixtures_dir.parent, manifest.get('code_files', {}), 'code')):
        for relative, expected in records.items():
            path = base / relative
            actual = sha256(path) if path.is_file() else None
            files.append({'path': relative, 'kind': label, 'sha256': actual, 'expected_sha256': expected})
            if actual != expected:
                errors.append(f'{label} source SHA mismatch: {relative}: {actual} != {expected}')
    model_path = asset_root / 'mjcf/nero_dh116.xml'
    root = ET.parse(model_path).getroot()
    declarations = {item.get('name'): item for item in root.findall('asset/mesh')}
    for name, record in identity['meshes'].items():
        declaration = declarations.get(name)
        path = (model_path.parent / declaration.get('file')).resolve() if declaration is not None else None
        expected_path = (asset_root / record['file']).resolve()
        actual = sha256(path) if path is not None and path.is_file() else None
        passed = actual == record['sha256'] and path == expected_path
        meshes.append({'name': name, 'file': record['file'], 'sha256': actual,
                       'expected_sha256': record['sha256'], 'passed': passed})
        if not passed:
            errors.append(f'mesh {name}: actual file bytes or official manifest binding mismatch')
    golden = fixtures_dir / 'golden/bounded_grasp.jsonl.gz'
    if golden.is_file():
        with gzip.open(golden, 'rb') as stream:
            actual = hashlib.sha256(stream.read()).hexdigest()
        if actual != manifest['golden_uncompressed_sha256']:
            errors.append('golden uncompressed trace SHA mismatch')
    return {'passed': not errors, 'errors': errors, 'files': files, 'active_meshes': meshes,
            'official_commit': identity['official_commit'], 'read_only': True, 'dynamic_steps': 0}


def audit_runtime(model, data, controller, fixtures_dir, asset_root):
    """Compare named compiled physics, complete reset, controller cache, mesh bytes.

    No tolerance, repair, controller refresh, or reference model compilation is
    performed. ``asset_root=None`` is reserved for isolated mutation unit tests.
    """
    import mujoco
    import numpy as np
    fixtures_dir = Path(fixtures_dir)
    expected = json.loads((fixtures_dir / 'compiled-model.json').read_text())
    reset = json.loads((fixtures_dir / 'full-reset-state.json').read_text())
    errors, differences, checked = [], [], 0

    def check(label, reference, actual):
        nonlocal checked
        checked += 1
        a, b = np.asarray(reference), np.asarray(actual)
        if a.shape == b.shape and np.array_equal(a, b):
            return
        detail = {'field': label, 'reference_shape': list(a.shape), 'actual_shape': list(b.shape)}
        if a.shape == b.shape and a.size:
            different = np.argwhere(a != b)
            index = tuple(different[0]) if len(different) else ()
            detail.update({'different_entries': len(different), 'first_index': list(index),
                           'reference_first': a[index].item(), 'actual_first': b[index].item()})
            if np.issubdtype(a.dtype, np.number) and np.issubdtype(b.dtype, np.number):
                detail['max_abs_difference'] = float(np.max(np.abs(a.astype(float) - b.astype(float))))
        differences.append(detail)
        errors.append(f'{label}: exact compiled/reset field mismatch')

    check('mujoco_version', expected['mujoco_version'], mujoco.__version__)
    for field, value in expected['dimensions'].items():
        check(field, value, getattr(model, field))
    for category, items in expected['objects'].items():
        kind = getattr(mujoco.mjtObj, 'mjOBJ_' + category)
        count = getattr(model, FIELDS[category][0])
        check(f'{category}.count', len(items), count)
        for item in items:
            name = item['name']
            target = mujoco.mj_name2id(model, kind, name) if name else item['index']
            label = name or f'unnamed_{item["index"]}'
            if target < 0 or target >= count:
                errors.append(f'{category} {label}: missing compiled identity')
                continue
            for field, value in item['fields'].items():
                if not hasattr(model, field):
                    errors.append(f'{field}: unavailable in runtime build')
                else:
                    check(f'{label}.{field}', value, getattr(model, field)[target])
            bindings = _bindings(model, category, target)
            for field, value in item['bindings'].items():
                check(f'{label}.{field}', value, bindings.get(field))
            if category == 'JOINT':
                start = int(model.jnt_dofadr[target])
                count_dof = {0: 6, 1: 3, 2: 1, 3: 1}[int(model.jnt_type[target])]
                for field, value in item['dofs'].items():
                    check(f'{label}.{field}', value, getattr(model, field)[start:start + count_dof])
                # State and action traces require the original joint ordering.
                check(f'{label}.jnt_qposadr', item['qpos_address'], int(model.jnt_qposadr[target]))
                check(f'{label}.jnt_dofadr', item['dof_address'], start)
    for field, value in expected['options'].items():
        check(f'option.{field}', value, getattr(model.opt, field))
    check('contact.exclude_signature', expected['exclude_signature'], model.exclude_signature)
    for field in ('qpos', 'qvel', 'ctrl', 'time'):
        check(f'reset.{field}', reset[field], getattr(data, field))
    for name, pos, quat in zip(reset['body_names'], reset['runtime_body_pos'], reset['runtime_body_quat']):
        index = 0 if name == 'world' else mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if index < 0:
            errors.append(f'reset body {name}: missing runtime body')
        else:
            check(f'reset.{name}.body_pos', pos, model.body_pos[index])
            check(f'reset.{name}.body_quat', quat, model.body_quat[index])
    for field in ('qfrc_applied', 'xfrc_applied'):
        check(f'reset.{field}', np.zeros_like(getattr(data, field)), getattr(data, field))
    control = controller_snapshot(controller)
    expected_control = json.loads((fixtures_dir / 'controller-state.json').read_text())
    if expected_control is not None:
        if control is None:
            errors.append('missing controller cache')
        else:
            for field, value in expected_control.items():
                check(f'controller.{field}', value, control.get(field))
    source = audit_sources(asset_root, fixtures_dir) if asset_root is not None else None
    if source is not None:
        errors.extend(source['errors'])
    return {'passed': not errors, 'errors': errors, 'differences': differences,
            'first_difference': differences[0] if differences else None,
            'max_abs_difference': max((d.get('max_abs_difference', 0.) for d in differences), default=0.),
            'fields_checked': checked, 'reference_check_scope': 5806,
            'comparison': 'exact, no tolerance or repair', 'controller': control,
            'mujoco_version': mujoco.__version__, 'sources': source,
            'reset_dimensions': {field: len(getattr(data, field)) for field in ('qpos', 'qvel', 'ctrl')},
            'read_only': True, 'dynamic_steps': 0}
