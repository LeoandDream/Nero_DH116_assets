#!/usr/bin/env python3
"""Offline cached-contact virtual-work estimate; stdlib only, no simulator imports.

Run from any cwd: python -B torque.py [--output-dir NEW_DIRECTORY].
Default outputs are adjacent to this file; existing outputs are never overwritten.
The input directory, pair, six snapshots, and J6 column are deliberately fixed.
"""

import argparse
import gzip
import hashlib
import importlib.abc
import json
import math
from pathlib import Path
import statistics
import sys
import xml.etree.ElementTree as ET


class NoSimulator(importlib.abc.MetaPathFinder):
    """Fail before any accidental simulator or robot-runtime import."""

    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'mujoco', 'robosuite', 'libero', 'dm_control', 'pybullet'}:
            raise RuntimeError('Simulator imports are forbidden in this offline analysis')
        return None


sys.meta_path.insert(0, NoSimulator())
HERE = Path(__file__).resolve().parent
INPUT = HERE.parent / 'd3_bowl78_slow_001'
PAIR = ['robot0_candidate_nero_link5_4', 'robot0_source_flange_part_2']
SNAPSHOTS = [1314, 1315, 1321, 1326, 1400, 1401]
J6 = 5


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def add(a, b):
    return [x+y for x, y in zip(a, b)]


def sub(a, b):
    return [x-y for x, y in zip(a, b)]


def scale(a, s):
    return [x*s for x in a]


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def norm(a):
    return math.sqrt(dot(a, a))


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def mat(a):
    return [a[:3], a[3:6], a[6:9]]


def mv(a, v):
    return [dot(row, v) for row in a]


def trans(a):
    return [list(x) for x in zip(*a)]


def quat_matrix(q):
    w, x, y, z = scale(q, 1/norm(q))
    return [[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]]


def rotate(axis, v, angle):
    """Rodrigues formula only; no model state, forward, collision, or solver."""
    c, s = math.cos(angle), math.sin(angle)
    return add(add(scale(v, c), scale(cross(axis, v), s)),
               scale(axis, dot(axis, v)*(1-c)))


def floats(element, attr, default):
    return [float(x) for x in element.get(attr, default).split()]


def load_rows(name):
    with gzip.open(INPUT/name, 'rt') as f:
        return [json.loads(line) for line in f]


def state_summary(row, contact_count=0, tau=0.):
    q, target = row['full_qpos'][J6], row['target_qpos_18'][J6]
    assert target == row['action_13'][J6]
    assert abs(abs(q-target)-row['arm_error_rad'][J6]) < 1e-14
    return {'step': row['step'], 'time_s': row['time_s'], 'phase': row['phase'],
            'macro': row['macro'], 'source_macro': row['source_macro'],
            'repeat_index': row['repeat_index'], 'target_rad': target, 'q6_rad': q,
            'q6_velocity_rad_s': row['full_qvel'][J6],
            'target_minus_q6_rad': target-q, 'absolute_error_rad': abs(q-target),
            'ctrl_Nm': row['ctrl'][J6], 'actuator_force_Nm': row['actuator_force'][J6],
            'pair_contact_count': contact_count, 'pair_tau6_estimate_Nm': tau}


def stats(rows, key):
    return {'min': min(rows, key=lambda r: r[key])[key],
            'min_step': min(rows, key=lambda r: r[key])['step'],
            'max': max(rows, key=lambda r: r[key])[key],
            'max_step': max(rows, key=lambda r: r[key])['step'],
            'median': statistics.median(r[key] for r in rows)}


def group_summary(rows):
    return {'count': len(rows), 'first': rows[0], 'last': rows[-1],
            'statistics': {key: stats(rows, key) for key in (
                'q6_velocity_rad_s', 'absolute_error_rad', 'ctrl_Nm',
                'actuator_force_Nm', 'pair_tau6_estimate_Nm')}}


def write_json(path, value):
    with path.open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')


def write_gz(path, rows):
    # Reproducible gzip headers; x mode protects prior evidence.
    with path.open('xb') as raw:
        with gzip.GzipFile(filename='', mode='wb', fileobj=raw, mtime=0) as f:
            for row in rows:
                f.write((json.dumps(row, separators=(',', ':'), allow_nan=False)+'\n').encode())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=HERE)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    names = ['state.jsonl.gz', 'contacts.jsonl.gz', 'model.xml', 'arm.xml',
             'frozen-before-first-step.json', 'schedule.json.gz']
    before = {name: sha(INPUT/name) for name in names}
    sources = json.loads((HERE/'torque-sources.json').read_text())
    root = ET.parse(INPUT/'model.xml').getroot()
    b6 = root.find(".//body[@name='robot0_link6']")
    b5 = root.find(".//body[@name='robot0_link5']")
    flange = root.find(".//body[@name='robot0_gripper_flange']")
    j6 = b6.find("joint[@name='robot0_joint6']")
    assert b6 in list(b5) and flange in list(b6.iter('body'))
    assert len(b6.findall('joint')) == 1
    assert floats(j6, 'pos', '0 0 0') == [0., 0., 0.]
    assert floats(j6, 'axis', '0 0 1') == [0., 0., 1.]
    assert j6.get('ref', '0') == '0'
    assert root.find('option').get('integrator') == 'RK4'
    assert root.find('option').get('cone') == 'elliptic'
    motors = list(root.find('actuator'))
    motor = motors[J6]
    assert motor.tag == 'motor' and motor.get('joint') == 'robot0_joint6'
    assert motor.get('gear') is None and not root.findall('.//default/motor')
    fixed_pos = floats(b6, 'pos', '0 0 0')
    fixed_quat = floats(b6, 'quat', '1 0 0 0')
    fixed_rotation = quat_matrix(fixed_quat)
    local_axis = floats(j6, 'axis', '0 0 1')
    states, contacts = load_rows('state.jsonl.gz'), load_rows('contacts.jsonl.gz')
    assert len(states) == len(contacts) == 1401
    detail, timeline, checks = [], [], []
    for state, contact_row in zip(states, contacts):
        step = state['step']
        assert step == contact_row['step'] == len(timeline)+1
        pair_contacts = [c for c in contact_row['contacts'] if sorted(c['geom']) == sorted(PAIR)]
        assert len(pair_contacts) <= 1
        tau = 0.
        for contact in pair_contacts:
            assert contact['geom'] == PAIR
            assert contact['body'] == ['robot0_link5', 'robot0_gripper_flange']
            assert not contact['filter_reasons']
            assert contact['force_contact_frame'][0] > 0
            parent_pose = contact['cached_body_pose'][0]
            parent_rotation = mat(parent_pose['rotation_world'])
            parent_pos = parent_pose['position_world_m']
            # jnt_pos == 0: the pivot does not depend on the unrecorded internal-stage q6.
            pivot = add(parent_pos, mv(parent_rotation, fixed_pos))
            axis = mv(parent_rotation, mv(fixed_rotation, local_axis))
            r = sub(contact['position_world_m'], pivot)
            jp = cross(axis, r)  # flange J6; link5 J6 is exactly zero by ancestry.
            frame = mat(contact['frame'])  # ROWS are contact axes in world coordinates.
            fc = contact['force_contact_frame']
            world_force = mv(trans(frame), fc[:3])  # + on geom2/flange, - on geom1/link5.
            world_moment = mv(trans(frame), fc[3:])
            signed_levers = mv(frame, jp)
            force_components = [signed_levers[i]*fc[i] for i in range(3)]
            torque_moment = dot(axis, world_moment)
            tau = sum(force_components)+torque_moment
            assert max(abs(x) for x in fc[3:]) == 0.
            # Independent finite difference of a point rigidly attached to the flange.
            # The link5 point stays fixed when only J6 is varied. No qpos is changed.
            epsilon = 1e-6
            jp_fd = scale(sub(rotate(axis, r, epsilon), rotate(axis, r, -epsilon)), 1/(2*epsilon))
            tau_fd = dot(jp_fd, world_force)+dot(axis, world_moment)
            checks.append({'step': step, 'axis_unit_error': abs(norm(axis)-1),
                           'frame_orthonormal_error': max(abs(dot(a,b)-(i==j))
                             for i,a in enumerate(frame) for j,b in enumerate(frame)),
                           'cross_vs_frame_tau_error_Nm': abs(tau-dot(jp, world_force)-torque_moment),
                           'finite_difference_jacobian_error_m_per_rad': norm(sub(jp_fd,jp)),
                           'finite_difference_tau_error_Nm': abs(tau_fd-tau)})
            detail.append({'step': step, 'cached_contact': contact,
                           'j6_pivot_world_m': pivot, 'j6_positive_axis_world': axis,
                           'point_minus_pivot_world_m': r,
                           'point_distance_to_pivot_m': norm(r),
                           'point_distance_to_axis_m': norm(jp),
                           'Jp_flange_minus_Jp_link5_column6_m_per_rad': jp,
                           'Jr_flange_minus_Jr_link5_column6': axis,
                           'contact_axes_dot_Jp_column6_signed_lever_m': signed_levers,
                           'force_on_geom2_world_N': world_force,
                           'moment_on_geom2_world_Nm': world_moment,
                           'tau6_contact_components_Nm': force_components,
                           'tau6_normal_Nm': force_components[0],
                           'tau6_tangential_Nm': sum(force_components[1:]),
                           'tau6_contact_moment_Nm': torque_moment,
                           'tau6_net_internal_estimate_Nm': tau,
                           'tau6_over_recorded_actuator_force': tau/state['actuator_force'][J6],
                           'tau6_normal_over_recorded_actuator_force': force_components[0]/state['actuator_force'][J6],
                           'tau6_times_poststep_q6_velocity_W_alignment_limited': tau*state['full_qvel'][J6]})
        timeline.append(state_summary(state, len(pair_contacts), tau))
    assert [r['step'] for r in detail] == list(range(1315, 1402))
    check_max = {key: max(r[key] for r in checks) for key in checks[0] if key != 'step'}
    assert check_max['axis_unit_error'] < 1e-12
    assert check_max['frame_orthonormal_error'] < 1e-12
    assert check_max['cross_vs_frame_tau_error_Nm'] < 1e-14
    assert check_max['finite_difference_jacobian_error_m_per_rad'] < 1e-10
    assert check_max['finite_difference_tau_error_Nm'] < 1e-10
    after = {name: sha(INPUT/name) for name in names}
    assert after == before
    assert not {'mujoco', 'robosuite', 'libero', 'dm_control', 'pybullet'} & set(sys.modules)
    by_step = {r['step']: r for r in detail}
    snapshots = []
    for step in SNAPSHOTS:
        snapshot = dict(timeline[step-1])
        snapshot['contact_estimate'] = by_step.get(step)
        snapshots.append(snapshot)
    a, b = timeline[1399], timeline[1400]
    phase_onsets = [r for i,r in enumerate(timeline) if i == 0 or r['phase'] != timeline[i-1]['phase']]
    target_changes = [r for i,r in enumerate(timeline) if i == 0 or r['target_rad'] != timeline[i-1]['target_rad']]
    summary = {
        'description': 'Saved-record-only net internal equal-and-opposite contact virtual-work estimate for J6.',
        'pair': PAIR, 'input_sha256_before': before, 'input_sha256_after': after,
        'analysis_script_sha256': sha(Path(__file__).resolve()),
        'primary_source_record_sha256': sha(HERE/'torque-sources.json'),
        'version': sources['version'],
        'math': {
            'world_wrench_on_geom2': 'F = frame.T @ force_contact_frame[:3]; M = frame.T @ force_contact_frame[3:]',
            'world_wrench_on_geom1': '-F, -M at the same recorded contact point',
            'general_internal_pair': 'tau6 = (Jp_flange6-Jp_link5_6).T F + (Jr_flange6-Jr_link5_6).T M',
            'ancestry': 'link5 is upstream of J6, so both link5 J6 Jacobian columns are zero; flange is downstream.',
            'cached_axis': 'a6 = R5_cached @ R_link6_fixed @ axis6_local',
            'cached_pivot': 'o6 = p5_cached + R5_cached @ pos_link6_fixed; joint6 pos is exactly zero.',
            'point_column': 'Jp_flange6 = a6 cross (p_contact_cached-o6); Jr_flange6 = a6',
            'decomposition': 'tau6_force = sum_i force_contact_frame[i] * dot(frame_row_i, Jp_flange6), i=0..2',
            'fixed_link6_position_m': fixed_pos, 'fixed_link6_quaternion_wxyz': fixed_quat,
            'joint6_local_axis': local_axis, 'motor_gear': 1,
            'moment_terms': 'Recorded last three contact components are zero in all 87 rows; included explicitly.'},
        'audit': {
            'stdlib_only': True, 'simulator_import_guard_installed': True,
            'simulator_modules_loaded': [], 'model_compilations': 0,
            'new_dynamics_or_constraint_solves_or_forward_or_collision_or_reset': 0,
            'qpos_writes': 0, 'fk_calls': 0, 'simulator_jacobian_calls': 0,
            'static_vector_calculations': 87, 'static_rodrigues_finite_differences': 87,
            'finite_difference_epsilon_rad': 1e-6, 'numeric_cross_checks_max_error': check_max,
            'all_input_hashes_unchanged': before == after,
            'contact_force_positive_rows': len(detail), 'first_contact_step': detail[0]['step'],
            'last_contact_step': detail[-1]['step'], 'pair_absent_rows': 1314,
            'positive_net_tau_count': sum(r['tau6_net_internal_estimate_Nm'] > 0 for r in detail),
            'negative_net_tau_count': sum(r['tau6_net_internal_estimate_Nm'] < 0 for r in detail)},
        'snapshots': snapshots, 'phase_onsets': phase_onsets, 'target_change_rows': target_changes,
        'all_1401_summary': group_summary(timeline),
        'settle_1_250_summary': group_summary(timeline[:250]),
        'precontact_approach_251_1314_summary': group_summary(timeline[250:1314]),
        'contact_1315_1401_summary': group_summary(timeline[1314:]),
        'contact_statistics': {key: stats(detail, key) for key in (
            'tau6_net_internal_estimate_Nm', 'tau6_normal_Nm', 'tau6_tangential_Nm',
            'tau6_over_recorded_actuator_force', 'tau6_normal_over_recorded_actuator_force',
            'point_distance_to_pivot_m', 'point_distance_to_axis_m')},
        'first_absolute_error_above_rad': {str(t): next(r for r in timeline if r['absolute_error_rad'] > t)
                                           for t in (.01, .05, .08, .1)},
        'stop_arithmetic': {'target_delta_rad': b['target_rad']-a['target_rad'],
                            'actual_q6_delta_rad': b['q6_rad']-a['q6_rad'],
                            'absolute_error_delta_rad': b['absolute_error_rad']-a['absolute_error_rad'],
                            'new_q6_vs_previous_target_error_rad': abs(b['q6_rad']-a['target_rad']),
                            'note': 'Arithmetic identity on saved post-step states; not a dynamical counterfactual.'},
        'interpretation': [
            'All 87 net pair torque estimates are positive and oppose the requested decreasing-q6 direction.',
            'Normal-only torque overstates net torque in most rows because tangential forces partly cancel it.',
            'J6 tracking lag is present before the pair first appears; precontact max and checkpoints are retained.',
            'The temporal coincidence of near-stall and contact supports interference as a contributor hypothesis, not a unique causal explanation.',
            'The positive motor ctrl includes the existing controller/bias compensation; its sign alone does not identify desired acceleration.'],
        'limitations': [
            'This is the contribution of the ONE specified internal contact pair, not recorded qfrc_constraint or a sum over all constraints.',
            'Cached body poses, frame, position and solved contact force belong to the final RK4 internal stage.',
            'State qpos/qvel are post-integration; no post-step forward was run. Force-versus-qvel and force-versus-error comparisons are descriptive and not exactly synchronous.',
            'Recorded ctrl is the control used for the step and actuator_force is the saved last-stage value; the controller evaluated from a preceding state, so post-step error is not its exact input.',
            'A force ratio to total recorded actuator output is not a ratio to net servo effort after bias compensation, nor to inertial, gravity, damping or Coriolis terms.',
            'No cached qfrc_constraint, qfrc_bias, qacc, mass matrix or full constraint Jacobian was saved here, so acceleration causality and a solver-level counterfactual cannot be reconstructed.',
            'No force, friction, filtering, threshold, controller, path or source-asset change is proposed or executed.']}
    write_gz(output/'torque-timeline.jsonl.gz', timeline)
    write_gz(output/'torque-contacts.jsonl.gz', detail)
    write_json(output/'torque.json', summary)
    print(json.dumps({'outputs': str(output), 'states': len(timeline), 'contacts': len(detail),
                      'checks': check_max, 'tau6_Nm': summary['contact_statistics']['tau6_net_internal_estimate_Nm']}))


if __name__ == '__main__':
    main()
