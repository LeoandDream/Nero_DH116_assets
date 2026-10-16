#!/usr/bin/env python3
"""Frozen minimum screening for the formal NERO + DH116 right-hand model.

Run from the checkout with its fixed public MuJoCo 3.2.3 / stock robosuite
1.4.1 environment. No LIBERO task, learned policy, or grasp predicate is run.
The arm uses the unmodified stock JointPositionController; the hand uses the
six position servos in the formal MJCF. Failed finite runs remain recorded.
"""
import argparse
from collections import Counter
import copy
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import traceback
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / 'mjcf/nero_dh116_right.xml'
URDF = ROOT / 'urdf/dh116_right.urdf'
ACTIVE = ('finger11', 'finger12', 'finger21', 'finger31', 'finger41', 'finger51')
THRESHOLDS = {
    'arm_hold_tracking_rad': .1,
    'active_steady_tracking_rad': .02,
    'linear_mimic_residual_rad': .02,
    'joint_limit_overshoot_rad': .01,
    'enabled_nonadjacent_self_overlap_m': .002,
    'external_contact_overlap_m': .002,
}
PROTOCOL = {
    'scope': 'minimum diagnostic screening; not hardware limits or a grasp success predicate',
    'threshold_origin': 'right-hand screening bounds declared before its first run; 2mm overlap follows the existing diagnostic convention; angular bounds are new screening limits, not manufacturer certification',
    'per_group_s': {'close_ramp': .5, 'closed_hold': .5, 'release_ramp': .5, 'open_hold': .5},
    'steady_window': 'last 0.1 s of each 0.5 s hold',
    'groups': list(ACTIVE) + ['all_six'],
    'arm_controller': 'stock robosuite1.4.1 JointPositionController; set_goal(zeros(7), set_qpos=keyframe_arm)',
    'arm_controller_parameters': {'kp': 50., 'damping_ratio': 1., 'policy_freq': 20},
    'hand_controller': 'six native MJCF position servos, no runtime parameter changes',
    'probe': {'radius_m': .006, 'mass_kg': .004, 'surface_gap_m': .001,
              'source_selection': 'all right-hand source vertices in stable descending world-z order; first sphere placement separated from every formal group-0 collision geom',
              'placement': 'candidate center = source vertex + world (0,0,radius+1mm); reject any signed distance < 0; no physical stepping during selection',
              'schedule': 'open 2 s; all-six half-close ramp .5 s, hold .5 s, release .5 s, hold .5 s'},
    'thresholds': THRESHOLDS,
    'real_grasp_result': 'UNKNOWN', 'libero_task_result': 'NOT_RUN',
}


def json_safe(value):
    """Keep failure evidence valid JSON, representing nonfinite numbers by null."""
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, (np.integer, np.floating)):
        return json_safe(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(json_safe(value), indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def reserve_output(root, value):
    root = Path(root).resolve()
    target = (root / value).resolve()
    if target == root or not target.is_relative_to(root):
        raise ValueError('Output must be a new directory inside this checkout')
    target.mkdir(parents=True, exist_ok=False)
    return target


def motion_plan(timestep):
    quarter = round(.5 / timestep)
    if quarter < 1 or not math.isclose(quarter * timestep, .5, abs_tol=1e-12):
        raise ValueError('The frozen half-second phases must contain whole physical steps')
    steady_steps = round(.1 / timestep)
    plan = []
    for group in range(7):
        for phase in range(4):
            for index in range(quarter):
                fraction = ((index + 1) / quarter if phase == 0 else 1. if phase == 1
                            else 1. - (index + 1) / quarter if phase == 2 else 0.)
                fractions = [fraction if group == 6 or group == channel else 0. for channel in range(6)]
                plan.append({'phase': (ACTIVE[group] if group < 6 else 'all_six') + '/' +
                             ('close_ramp', 'closed_hold', 'release_ramp', 'open_hold')[phase],
                             'fractions': fractions,
                             'steady': phase in (1, 3) and index >= quarter - steady_steps})
    return plan


def probe_plan(timestep):
    open_steps = round(2. / timestep)
    hold = [{'phase': 'probe/gravity_open', 'fractions': [0.] * 6,
             'steady': index >= open_steps - round(.1 / timestep)} for index in range(open_steps)]
    closing = [copy.deepcopy(row) for row in motion_plan(timestep)[-round(2. / timestep):]]
    for row in closing:
        row['phase'] = 'probe/' + row['phase']
        row['fractions'] = [value * .5 for value in row['fractions']]
    return hold + closing


def object_name(model, kind, index):
    return mujoco.mj_id2name(model, kind, int(index)) or f'unnamed_{int(index)}'


def bind_robot(model, urdf):
    source = urdf if isinstance(urdf, ET.Element) else ET.parse(urdf).getroot()
    hand = [joint for joint in source.findall('joint') if joint.get('type') != 'fixed']
    if len(hand) != 11:
        raise ValueError('Expected the source right-hand 11-joint state')
    names = [f'robot0_joint{i}' for i in range(1, 8)] + ['gripper0_' + joint.get('name') for joint in hand]
    joint_ids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in names])
    if (joint_ids < 0).any() or model.nu != 13:
        raise ValueError('Formal right model must expose 18 named joints and 13 actuators')
    active_indexes = [names.index('gripper0_' + name) for name in ACTIVE]
    actuator_ids = []
    for index in list(range(7)) + active_indexes:
        matches = np.flatnonzero(model.actuator_trnid[:, 0] == joint_ids[index])
        if len(matches) != 1:
            raise ValueError(f'Expected one actuator for {names[index]}')
        actuator_ids.append(int(matches[0]))
    ranges = model.jnt_range[joint_ids].copy()
    for joint, limits in zip(hand, ranges[7:]):
        expected = [float(joint.find('limit').get(key)) for key in ('lower', 'upper')]
        if not np.allclose(limits, expected, rtol=0, atol=1e-12):
            raise ValueError(f'URDF/MJCF range mismatch: {joint.get("name")}')
    mimics = []
    for joint in hand:
        mimic = joint.find('mimic')
        if mimic is not None:
            mimics.append({'name': joint.get('name'), 'dependent': names.index('gripper0_' + joint.get('name')),
                           'driver': names.index('gripper0_' + mimic.get('joint')),
                           'multiplier': float(mimic.get('multiplier', '1')),
                           'offset': float(mimic.get('offset', '0'))})
    if len(mimics) != 5:
        raise ValueError('Expected five source linear mimic relations')
    return {'names': names, 'joint_ids': joint_ids,
            'qpos': model.jnt_qposadr[joint_ids].copy(), 'qvel': model.jnt_dofadr[joint_ids].copy(),
            'active_indexes': active_indexes, 'actuator_ids': actuator_ids, 'ranges': ranges,
            'active_ranges': ranges[active_indexes].copy(), 'mimics': mimics}


def filter_reasons(model, first, second):
    """Explain default production filtering without using it to suppress the audit."""
    b1, b2 = (int(model.geom_bodyid[index]) for index in (first, second))
    w1, w2 = int(model.body_weldid[b1]), int(model.body_weldid[b2])
    reasons = []
    if w1 == w2:
        reasons.append('SAME_RIGID_ASSEMBLY')
    filter_disabled = int(model.opt.disableflags) & int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
    if not filter_disabled and w1 != w2 and w1 != 0 and w2 != 0 and (
            int(model.body_weldid[model.body_parentid[w1]]) == w2 or
            int(model.body_weldid[model.body_parentid[w2]]) == w1):
        reasons.append('PARENT_CHILD')
    if not ((int(model.geom_contype[first]) & int(model.geom_conaffinity[second])) or
            (int(model.geom_contype[second]) & int(model.geom_conaffinity[first]))):
        reasons.append('COLLISION_MASKS')
    signatures = {(min(b1, b2) << 16) + max(b1, b2), (min(w1, w2) << 16) + max(w1, w2)}
    if signatures.intersection(int(value) for value in model.exclude_signature):
        reasons.append('EXPLICIT_EXCLUDE')
    return reasons


def pair_names(model, first, second):
    return {'geom_names': [object_name(model, mujoco.mjtObj.mjOBJ_GEOM, index) for index in (first, second)],
            'body_names': [object_name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[index])
                           for index in (first, second)]}


def static_distance_audit(model, urdf):
    binding = bind_robot(model, urdf)
    data = mujoco.MjData(model)
    if model.nkey < 1:
        raise ValueError('The native right-hand keyframe is required')
    poses = [('open', [0.] * 6), ('half', [.5] * 6), ('full', [1.] * 6)]
    poses += [(ACTIVE[channel] + '_endpoint', [float(index == channel) for index in range(6)])
              for channel in range(6)]
    # Rendering geoms are duplicates, not the intended collision representation.
    geoms = [index for index in range(model.ngeom) if model.geom_group[index] == 0]
    pairs = []
    for offset, first in enumerate(geoms):
        for second in geoms[offset + 1:]:
            bodies = pair_names(model, first, second)['body_names']
            if any(name.startswith('gripper0_') for name in bodies):
                pairs.append((first, second))
    report = {'method': 'mj_geomDistance on explicit group-0 geometry pairs; ignores production pair filtering',
              'distance_cutoff_m': .01, 'poses': [], 'pairs_per_pose': len(pairs),
              'actual_visual_mesh_penetration': 'NOT_DETERMINED',
              'limitations': ['MuJoCo mesh distances use convex hulls, not triangle-level CAD intersection.',
                              'Negative distances are overlap candidates; filtered rigid joins may be intentional CAD engagement.',
                              'Nine discrete poses do not prove the full continuous workspace collision-free.',
                              'Group-1 render-only duplicate meshes are excluded from this collision-proxy audit.']}
    for label, fractions in poses:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        state = data.qpos[binding['qpos']].copy()
        ranges = binding['active_ranges']
        state[binding['active_indexes']] = ranges[:, 0] + np.asarray(fractions) * (ranges[:, 1] - ranges[:, 0])
        for mimic in binding['mimics']:
            state[mimic['dependent']] = mimic['offset'] + mimic['multiplier'] * state[mimic['driver']]
        # Only this separate static audit data is posed directly; no physical step occurs here.
        data.qpos[binding['qpos']] = state
        mujoco.mj_forward(model, data)
        nearby, errors, counts = [], [], Counter()
        for first, second in pairs:
            reasons = filter_reasons(model, first, second)
            counts['|'.join(reasons) if reasons else 'ENABLED_NONADJACENT'] += 1
            try:
                segment = np.zeros(6)
                distance = float(mujoco.mj_geomDistance(model, data, first, second, .01, segment))
                if not math.isfinite(distance):
                    raise ValueError('nonfinite geometry distance')
                if distance < .01:
                    nearby.append({**pair_names(model, first, second), 'distance_m': distance,
                                   'filter_reasons': reasons, 'closest_segment_world_m': segment.tolist()})
            except Exception as error:
                errors.append({**pair_names(model, first, second), 'error': str(error)})
        report['poses'].append({'pose': label, 'fractions': fractions,
                                'filter_reason_pair_counts': dict(counts), 'nearby_pairs': nearby,
                                'overlap_candidates': [pair for pair in nearby if pair['distance_m'] < 0],
                                'distance_errors': errors})
    report['max_enabled_nonadjacent_overlap_m'] = max(
        [-pair['distance_m'] for pose in report['poses'] for pair in pose['overlap_candidates']
         if not pair['filter_reasons']] or [0.])
    report['max_filtered_overlap_candidate_m'] = max(
        [-pair['distance_m'] for pose in report['poses'] for pair in pose['overlap_candidates']
         if pair['filter_reasons']] or [0.])
    report['proxy_screening_status'] = ('FAIL' if report['max_enabled_nonadjacent_overlap_m'] >
                                         THRESHOLDS['enabled_nonadjacent_self_overlap_m'] else
                                         'UNKNOWN' if any(pose['distance_errors'] for pose in report['poses']) else 'PASS')
    return report


def contacts_snapshot(model, data):
    contacts = []
    for index, contact in enumerate(data.contact):
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, force)
        pair = pair_names(model, contact.geom1, contact.geom2)
        robot = [name.startswith(('robot0_', 'gripper0_', 'mount0_')) for name in pair['body_names']]
        hand = [name.startswith('gripper0_') for name in pair['body_names']]
        reasons = filter_reasons(model, contact.geom1, contact.geom2)
        contacts.append({**pair, 'distance_m': float(contact.dist), 'normal_force_N': float(force[0]),
                         'contact_force_torque': force.tolist(), 'position_world_m': contact.pos.tolist(),
                         'frame_world': contact.frame.tolist(), 'filter_reasons': reasons,
                         'robot_external': any(robot) and not all(robot),
                         'right_hand_external': any(hand) and not all(robot),
                         'enabled_nonadjacent_robot_self': all(robot) and not reasons})
    return contacts


def physics_fingerprint(model):
    digest = hashlib.sha256()
    for name in ('body_pos', 'body_quat', 'body_mass', 'body_inertia', 'body_ipos', 'body_iquat',
                 'geom_pos', 'geom_quat', 'geom_contype', 'geom_conaffinity', 'geom_friction',
                 'geom_solref', 'geom_solimp', 'jnt_axis', 'jnt_range', 'jnt_solref', 'jnt_solimp',
                 'dof_damping', 'dof_armature', 'dof_frictionloss', 'actuator_gainprm',
                 'actuator_biasprm', 'actuator_ctrlrange', 'eq_data', 'eq_solref', 'eq_solimp',
                 'exclude_signature'):
        digest.update(name.encode())
        digest.update(getattr(model, name).tobytes())
    for name in ('timestep', 'integrator', 'gravity', 'density', 'viscosity', 'impratio', 'cone'):
        digest.update(name.encode())
        digest.update(np.asarray(getattr(model.opt, name)).tobytes())
    return digest.hexdigest()


def run_case(model, urdf, output, name, plan):
    from robosuite.controllers.joint_pos import JointPositionController
    from robosuite.utils.binding_utils import MjSim

    binding = bind_robot(model, urdf)
    output = Path(output)
    sim = MjSim(model)
    data = sim.data._data
    before_physics = physics_fingerprint(model)
    result = {'case': name, 'screening_status': 'FAIL', 'execution_status': 'INITIALIZING',
              'physics_steps': 0, 'recorded_steps': 0, 'planned_steps': len(plan),
              'failures': {}, 'real_grasp_result': 'UNKNOWN', 'libero_task_result': 'NOT_RUN',
              'max_active_tracking_error_rad': 0., 'max_active_steady_tracking_error_rad': 0.,
              'max_arm_tracking_error_rad': 0., 'max_linear_mimic_residual_rad': 0.,
              'max_joint_limit_overshoot_rad': 0., 'max_enabled_nonadjacent_self_overlap_m': 0.,
              'external_contact_steps': 0, 'max_external_contact_normal_N': 0.,
              'max_external_contact_overlap_m': 0., 'external_contact_measurement': 'UNKNOWN',
              'right_hand_external_contact_steps': 0, 'max_right_hand_external_contact_normal_N': 0.,
              'right_hand_external_contact_measurement': 'UNKNOWN'}

    def fail(code, step):
        item = result['failures'].setdefault(code, {'first_step': step, 'count': 0})
        item['count'] += 1

    try:
        if model.nkey < 1:
            raise ValueError('A native initialization keyframe is required')
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        initial = data.qpos[binding['qpos']].copy()
        arm_target = initial[:7].copy()
        result['initial_robot_state_18'] = initial.tolist()
        result['initial_full_qpos'] = data.qpos.tolist()
        result['initial_time_s'] = float(data.time)
        arm_act = binding['actuator_ids'][:7]
        hand_act = binding['actuator_ids'][7:]
        arm_ctrl_ranges = model.actuator_ctrlrange[arm_act].copy()
        controller = JointPositionController(
            sim=sim, eef_name='gripper0_grip_site',
            joint_indexes={'joints': binding['joint_ids'][:7], 'qpos': binding['qpos'][:7],
                           'qvel': binding['qvel'][:7]},
            actuator_range=(arm_ctrl_ranges[:, 0], arm_ctrl_ranges[:, 1]),
            kp=50, damping_ratio=1, policy_freq=20,
            qpos_limits=binding['ranges'][:7].T)
        controller.set_goal(np.zeros(7), set_qpos=arm_target)
        ranges = binding['active_ranges']
        result['execution_status'] = 'RUNNING'
        with (output / f'{name}-state.jsonl').open('x') as states, (output / f'{name}-contacts.jsonl').open('x') as details:
            for step, command in enumerate(plan, 1):
                fractions = np.asarray(command['fractions'], dtype=float)
                if fractions.shape != (6,) or not np.isfinite(fractions).all() or np.any(fractions < 0) or np.any(fractions > 1):
                    raise ValueError('Frozen hand command must be six source-range fractions')
                target = ranges[:, 0] + fractions * (ranges[:, 1] - ranges[:, 0])
                data.ctrl[arm_act] = np.clip(controller.run_controller(), arm_ctrl_ranges[:, 0], arm_ctrl_ranges[:, 1])
                data.ctrl[hand_act] = target
                previous_time = float(data.time)
                mujoco.mj_step(model, data)
                result['physics_steps'] += 1
                state = data.qpos[binding['qpos']].copy()
                errors = state[binding['active_indexes']] - target
                overshoot = np.maximum(0., np.maximum(binding['ranges'][:, 0] - state, state - binding['ranges'][:, 1]))
                mimic = {item['name']: float(state[item['dependent']] - item['multiplier'] * state[item['driver']] - item['offset'])
                         for item in binding['mimics']}
                contacts = contacts_snapshot(model, data)
                self_overlap = max([max(0., -item['distance_m']) for item in contacts
                                    if item['enabled_nonadjacent_robot_self']] or [0.])
                external = [item for item in contacts if item['robot_external']]
                external_force = max([max(0., item['normal_force_N']) for item in external] or [0.])
                external_overlap = max([max(0., -item['distance_m']) for item in external] or [0.])
                hand_external = [item for item in contacts if item['right_hand_external']]
                hand_external_force = max([max(0., item['normal_force_N']) for item in hand_external] or [0.])
                warnings = {str(mujoco.mjtWarning(index)): int(warning.number)
                            for index, warning in enumerate(data.warning) if warning.number}
                finite = all(np.isfinite(value).all() for value in (data.qpos, data.qvel, data.qacc, data.ctrl, data.actuator_force))
                finite = finite and all(math.isfinite(item['distance_m']) and
                                        np.isfinite(item['contact_force_torque']).all() for item in contacts)
                row = {'step': step, 'time_s': float(data.time), 'phase': command['phase'],
                       'steady_window': bool(command['steady']), 'robot_state_18': state.tolist(),
                       'robot_action_13': np.r_[arm_target, 2 * fractions - 1].tolist(),
                       'internal_actuator_ctrl_13': data.ctrl.tolist(), 'hand_target_rad': target.tolist(),
                       'active_tracking_error_rad': errors.tolist(), 'arm_tracking_error_rad': (state[:7] - arm_target).tolist(),
                       'joint_limit_overshoot_rad': dict(zip(binding['names'], overshoot.tolist())),
                       'mimic_residual_rad': mimic, 'warnings': warnings, 'finite_state': bool(finite),
                       'enabled_nonadjacent_self_overlap_m': self_overlap,
                       'external_contact_count': len(external), 'max_external_contact_normal_N': external_force,
                       'external_contact_overlap_m': external_overlap,
                       'right_hand_external_contact_count': len(hand_external),
                       'max_right_hand_external_contact_normal_N': hand_external_force,
                       'full_qpos': data.qpos.tolist(), 'full_qvel': data.qvel.tolist(),
                       'actuator_force': data.actuator_force.tolist()}
                states.write(json.dumps(json_safe(row), allow_nan=False) + '\n')
                details.write(json.dumps(json_safe({'step': step, 'time_s': float(data.time), 'contacts': contacts}), allow_nan=False) + '\n')
                result['recorded_steps'] += 1
                if not finite:
                    fail('NONFINITE_STATE', step)
                if warnings:
                    fail('NUMERICAL_WARNING', step)
                if not math.isclose(float(data.time) - previous_time, float(model.opt.timestep), rel_tol=0, abs_tol=1e-10):
                    fail('TIME_DISCONTINUITY', step)
                active_error = float(np.max(np.abs(errors)))
                mimic_error = max(abs(value) for value in mimic.values())
                limit_error = float(np.max(overshoot))
                result['max_active_tracking_error_rad'] = max(result['max_active_tracking_error_rad'], active_error)
                arm_error = float(np.max(np.abs(state[:7] - arm_target)))
                result['max_arm_tracking_error_rad'] = max(result['max_arm_tracking_error_rad'], arm_error)
                result['max_linear_mimic_residual_rad'] = max(result['max_linear_mimic_residual_rad'], mimic_error)
                result['max_joint_limit_overshoot_rad'] = max(result['max_joint_limit_overshoot_rad'], limit_error)
                result['max_enabled_nonadjacent_self_overlap_m'] = max(result['max_enabled_nonadjacent_self_overlap_m'], self_overlap)
                if command['steady']:
                    result['max_active_steady_tracking_error_rad'] = max(result['max_active_steady_tracking_error_rad'], active_error)
                    if active_error > THRESHOLDS['active_steady_tracking_rad']:
                        fail('ACTIVE_STEADY_TRACKING', step)
                for code, value, threshold in (
                        ('ARM_HOLD_TRACKING', arm_error, THRESHOLDS['arm_hold_tracking_rad']),
                        ('LINEAR_MIMIC_RESIDUAL', mimic_error, THRESHOLDS['linear_mimic_residual_rad']),
                        ('JOINT_LIMIT_OVERSHOOT', limit_error, THRESHOLDS['joint_limit_overshoot_rad']),
                        ('ENABLED_NONADJACENT_SELF_OVERLAP', self_overlap, THRESHOLDS['enabled_nonadjacent_self_overlap_m']),
                        ('EXTERNAL_CONTACT_OVERLAP', external_overlap, THRESHOLDS['external_contact_overlap_m'])):
                    if value > threshold:
                        fail(code, step)
                result['external_contact_steps'] += int(bool(external))
                result['max_external_contact_normal_N'] = max(result['max_external_contact_normal_N'], external_force)
                result['max_external_contact_overlap_m'] = max(result['max_external_contact_overlap_m'], external_overlap)
                result['right_hand_external_contact_steps'] += int(bool(hand_external))
                result['max_right_hand_external_contact_normal_N'] = max(result['max_right_hand_external_contact_normal_N'], hand_external_force)
                if not finite:
                    break
        result['execution_status'] = 'FINISHED' if result['physics_steps'] == len(plan) else 'STOPPED_NONFINITE'
    except Exception as error:
        result['execution_status'] = 'ERROR'
        result['error'] = f'{type(error).__name__}: {error}'
        result['traceback'] = traceback.format_exc()
        fail('RUNTIME_ERROR', result['physics_steps'])
    finally:
        result['simulated_time_s'] = float(data.time)
        result['physics_fields_unchanged'] = before_physics == physics_fingerprint(model)
        if not result['physics_fields_unchanged']:
            fail('PHYSICS_FIELDS_CHANGED', result['physics_steps'])
        if result['physics_steps'] != len(plan) or result['recorded_steps'] != result['physics_steps']:
            fail('INCOMPLETE_RECORDING', result['physics_steps'])
        if result['external_contact_steps'] and result['max_external_contact_normal_N'] > 0:
            result['external_contact_measurement'] = 'OBSERVED'
        if result['right_hand_external_contact_steps'] and result['max_right_hand_external_contact_normal_N'] > 0:
            result['right_hand_external_contact_measurement'] = 'RIGHT_HAND_CONTACT_OBSERVED'
        result['screening_status'] = 'PASS' if not result['failures'] else 'FAIL'
        write_json(output / f'{name}-result.json', result)
    return result


def probe_model(path, urdf):
    """Derive an external free-ball fixture without changing robot parameters."""
    model = mujoco.MjModel.from_xml_path(str(path))
    binding = bind_robot(model, urdf)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    candidates = [index for index in range(model.ngeom)
                  if object_name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[index]).startswith('gripper0_')
                  and model.geom_type[index] == mujoco.mjtGeom.mjGEOM_MESH]
    if not candidates:
        raise ValueError('The formal right-hand source meshes are required for probe placement')
    surfaces = []
    for candidate in candidates:
        mesh_id = int(model.geom_dataid[candidate])
        start, count = int(model.mesh_vertadr[mesh_id]), int(model.mesh_vertnum[mesh_id])
        world = (model.mesh_vert[start:start + count] @ data.geom_xmat[candidate].reshape(3, 3).T
                 + data.geom_xpos[candidate])
        # Only one visual/collision instance of each source mesh is needed.
        if not any(item[1] == mesh_id for item in surfaces):
            surfaces.append((candidate, mesh_id, world))
    vertices = np.concatenate([item[2] for item in surfaces])
    owners = np.concatenate([np.full(len(item[2]), index, dtype=int) for index, item in enumerate(surfaces)])
    local_ids = np.concatenate([np.arange(len(item[2])) for item in surfaces])
    order = np.argsort(-vertices[:, 2], kind='stable')
    center = np.zeros(3)
    source = ET.parse(path).getroot()
    # Resolve assets against the formal file before serializing a self-contained fixture XML.
    compiler = source.find('compiler')
    meshdir = Path(compiler.get('meshdir', '.')) if compiler is not None else Path('.')
    texturedir = Path(compiler.get('texturedir', '.')) if compiler is not None else Path('.')
    for asset in source.findall('asset/*'):
        if asset.get('file'):
            directory = meshdir if asset.tag == 'mesh' else texturedir if asset.tag == 'texture' else Path('.')
            asset.set('file', str((Path(path).parent / directory / asset.get('file')).resolve()))
    if compiler is not None:
        compiler.attrib.pop('meshdir', None)
        compiler.attrib.pop('texturedir', None)
    ball = ET.SubElement(source.find('worldbody'), 'body', name='probe', pos=' '.join(map(str, center)))
    ET.SubElement(ball, 'freejoint', name='probe_free')
    ET.SubElement(ball, 'geom', name='probe_collision', type='sphere', size=str(PROTOCOL['probe']['radius_m']),
                  mass=str(PROTOCOL['probe']['mass_kg']), friction='.8 .005 .0001', contype='1', conaffinity='1', rgba='.9 .25 .15 1')
    for key in source.findall('keyframe/key'):
        qpos = np.fromstring(key.get('qpos', ''), sep=' ')
        qvel = np.fromstring(key.get('qvel', ''), sep=' ')
        key.set('qpos', ' '.join(map(str, np.r_[qpos, center, [1., 0., 0., 0.]])))
        if len(qvel):
            key.set('qvel', ' '.join(map(str, np.r_[qvel, np.zeros(6)])))
    # Candidate posing is confined to this setup data, before a final native
    # fixture/keyframe exists and before either physical trajectory is run.
    fixture = mujoco.MjModel.from_xml_string(ET.tostring(source, encoding='unicode'))
    setup = mujoco.MjData(fixture)
    mujoco.mj_resetDataKeyframe(fixture, setup, 0)
    sphere = mujoco.mj_name2id(fixture, mujoco.mjtObj.mjOBJ_GEOM, 'probe_collision')
    sphere_joint = mujoco.mj_name2id(fixture, mujoco.mjtObj.mjOBJ_JOINT, 'probe_free')
    sphere_qpos = int(fixture.jnt_qposadr[sphere_joint])
    collision_geoms = [index for index in range(model.ngeom) if model.geom_group[index] == 0]
    rejection_counts = Counter()
    rejected_examples = []
    selected = None
    segment = np.zeros(6)
    for checked, vertex_id in enumerate(order, 1):
        vertex = vertices[vertex_id]
        center = vertex + np.array([0., 0., PROTOCOL['probe']['radius_m'] + PROTOCOL['probe']['surface_gap_m']])
        setup.qpos[sphere_qpos:sphere_qpos + 3] = center
        mujoco.mj_kinematics(fixture, setup)
        distances = []
        for other in collision_geoms:
            distance = float(mujoco.mj_geomDistance(fixture, setup, sphere, other, 1., segment))
            if not math.isfinite(distance):
                raise ValueError('SETUP_FAIL: nonfinite candidate geometry distance')
            distances.append(distance)
            if distance < 0:
                other_name = object_name(fixture, mujoco.mjtObj.mjOBJ_GEOM, other)
                rejection_counts[other_name] += 1
                if len(rejected_examples) < 10:
                    rejected_examples.append({'candidate_ordinal': checked, 'geom': other_name,
                                              'distance_m': distance, 'sphere_center_world_m': center.tolist()})
                break
        if len(distances) == len(collision_geoms) and min(distances) >= 0:
            geom, mesh, _ = surfaces[int(owners[vertex_id])]
            selected = {'candidate_vertices_checked': checked, 'source_vertex_index': int(local_ids[vertex_id]),
                        'minimum_initial_collision_distance_m': min(distances)}
            break
    if selected is None:
        raise ValueError(f'SETUP_FAIL: no separated sphere placement among {len(order)} source vertices; rejection counts={dict(rejection_counts)}')
    ball.set('pos', ' '.join(map(str, center)))
    for key in source.findall('keyframe/key'):
        qpos = np.fromstring(key.get('qpos'), sep=' ')
        qpos[-7:-4] = center
        key.set('qpos', ' '.join(map(str, qpos)))
    xml = ET.tostring(source, encoding='unicode')
    fixture = mujoco.MjModel.from_xml_string(xml)
    fixture_binding = bind_robot(fixture, urdf)
    fixture_data = mujoco.MjData(fixture)
    mujoco.mj_resetDataKeyframe(fixture, fixture_data, 0)
    mujoco.mj_forward(fixture, fixture_data)
    np.testing.assert_array_equal(data.qpos[binding['qpos']], fixture_data.qpos[fixture_binding['qpos']])
    initial_contacts = contacts_snapshot(fixture, fixture_data)
    return fixture, xml, {**selected, 'source_vertices_available': len(order),
                           'candidate_order': 'stable descending world-z; model geom order then compiled source vertex order breaks ties',
                           'rejection_counts_by_geom': dict(rejection_counts), 'rejected_candidate_examples': rejected_examples,
                           'source_geom': object_name(model, mujoco.mjtObj.mjOBJ_GEOM, geom),
                           'source_body': object_name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[geom]),
                           'source_mesh': object_name(model, mujoco.mjtObj.mjOBJ_MESH, mesh),
                           'source_surface_vertex_world_m': vertex.tolist(), 'sphere_center_world_m': center.tolist(),
                           'sphere_radius_m': PROTOCOL['probe']['radius_m'],
                           'initial_contacts': initial_contacts,
                           'initial_robot_overlap_m': max([max(0., -item['distance_m']) for item in initial_contacts
                                                           if item['robot_external']] or [0.]),
                           'placement_limitations': 'Mesh vertex lies on the source surface; physical mesh contact uses its convex hull. No collision filter is added.'}


def verify_manifest(root, manifest):
    """Bind the delivered right model and both sets of assets to its manifest."""
    root = Path(root).resolve()
    checked = {}
    for group in ('generated_files', 'meshes', 'inherited_nonhand_assets'):
        if not manifest.get(group):
            raise ValueError(f'Right source manifest is missing {group}')
        for item in manifest[group]:
            path = (root / item['file']).resolve()
            if not path.is_relative_to(root) or not path.is_file():
                raise ValueError(f'Manifest input outside checkout or missing: {item["file"]}')
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
            if actual != item['sha256']:
                raise ValueError(f'SHA256 mismatch in {group}: {item["file"]}')
            checked[item['file']] = actual
    return {'checked_files': len(checked), 'sha256_verified': checked}


def source_preflight():
    versions = {'python': '.'.join(map(str, sys.version_info[:3])), 'mujoco': mujoco.__version__,
                'robosuite': importlib.metadata.version('robosuite'), 'numpy': np.__version__}
    if versions != {'python': '3.11.16', 'mujoco': '3.2.3', 'robosuite': '1.4.1', 'numpy': '1.26.4'}:
        raise ValueError(f'Use the fixed public Python/MuJoCo/stock robosuite environment: {versions}')
    manifest_path = ROOT / 'RIGHT_SOURCE_MANIFEST.json'
    manifest = json.loads(manifest_path.read_text())
    verified = verify_manifest(ROOT, manifest)
    files = [MODEL, URDF, Path(__file__).resolve(), manifest_path]
    source = ET.parse(MODEL).getroot()
    compiler = source.find('compiler')
    for asset in source.findall('asset/*'):
        if asset.get('file'):
            folder = compiler.get('meshdir', '.') if asset.tag == 'mesh' else compiler.get('texturedir', '.') if asset.tag == 'texture' else '.'
            files.append((MODEL.parent / folder / asset.get('file')).resolve())
    for path in files:
        if not path.resolve().is_relative_to(ROOT) or not path.is_file():
            raise ValueError(f'Runtime source outside this checkout or missing: {path}')
    return {'versions': versions, 'python_executable': sys.executable,
            'files': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in dict.fromkeys(files)},
            'formal_model': str(MODEL.relative_to(ROOT)), 'source_urdf': str(URDF.relative_to(ROOT)),
            'manifest_verification': verified,
            'historical_assets_read': False, 'libero_loaded': False}


def classify_cases(motion, contact, audit):
    if any(case['execution_status'] == 'ERROR' or case['recorded_steps'] != case['physics_steps']
           for case in (motion, contact)):
        return 'ERROR', 1
    static_complete = not any(pose['distance_errors'] for pose in audit['poses'])
    passed = (motion['screening_status'] == contact['screening_status'] == 'PASS' and
              contact['right_hand_external_contact_measurement'] == 'RIGHT_HAND_CONTACT_OBSERVED' and
              static_complete and audit['proxy_screening_status'] == 'PASS')
    return ('MINIMUM_SCREENING_PASS', 0) if passed else ('MINIMUM_SCREENING_FAIL_OR_UNKNOWN', 2)


def execute(output):
    result = {'status': 'ERROR', 'real_grasp_result': 'UNKNOWN', 'libero_task_result': 'NOT_RUN'}
    try:
        write_json(output / 'frozen-protocol.json', PROTOCOL)
        write_json(output / 'source-preflight.json', source_preflight())
        source = ET.parse(URDF).getroot()
        formal = mujoco.MjModel.from_xml_path(str(MODEL))
        plan = motion_plan(float(formal.opt.timestep))
        contact_plan = probe_plan(float(formal.opt.timestep))
        write_json(output / 'frozen-commands.json', {'hand_motion': plan, 'external_probe': contact_plan})
        audit = static_distance_audit(formal, source)
        write_json(output / 'static-distance-audit.json', audit)
        fixture, xml, placement = probe_model(MODEL, source)
        (output / 'probe-scene.xml').write_text(xml + '\n')
        write_json(output / 'probe-placement.json', placement)
        motion = run_case(formal, source, output, 'hand-motion', plan)
        contact = run_case(fixture, source, output, 'external-probe', contact_plan)
        initial_external_overlap = any(item['robot_external'] and item['distance_m'] < 0 for item in placement['initial_contacts'])
        if initial_external_overlap:
            contact['failures']['INITIAL_PROBE_OVERLAP'] = {'first_step': 0, 'count': 1}
            contact['screening_status'] = 'FAIL'
            write_json(output / 'external-probe-result.json', contact)
        static_complete = not any(pose['distance_errors'] for pose in audit['poses'])
        # Static filtered overlaps remain candidates and do not count as actual CAD penetration.
        status, _ = classify_cases(motion, contact, audit)
        result.update(status=status,
                      hand_motion=motion, external_probe=contact,
                      static_distance_complete=static_complete,
                      static_proxy_screening_status=audit['proxy_screening_status'],
                      actual_visual_mesh_penetration='NOT_DETERMINED',
                      actual_grasp_success='UNKNOWN', new_collision_filters=False,
                      robot_physical_parameters_tuned=False)
    except Exception as error:
        result.update(error=f'{type(error).__name__}: {error}', traceback=traceback.format_exc())
    write_json(output / 'run-result.json', result)
    return 0 if result['status'] == 'MINIMUM_SCREENING_PASS' else 1 if result['status'] == 'ERROR' else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='New output directory inside this checkout')
    arguments = parser.parse_args(argv)
    try:
        output = reserve_output(ROOT, arguments.output)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    code = execute(output)
    print(json.dumps({'output': str(output), 'exit_code': code}))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
