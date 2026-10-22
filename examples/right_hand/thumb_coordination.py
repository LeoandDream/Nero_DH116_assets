#!/usr/bin/env python3
"""Diagnostic-only: one authorized 1000-step lambda=1 thumb-path intervention.

Reuses the original run_case, reset, stock JPC and all screening thresholds.
The saved lambda=0 prefix is the control; no second physical run is made.
"""
import copy
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BASELINE = '8142d0ca9a6e78332118ab5e632cafe4e39da28b'
EVIDENCE = ROOT / 'examples/right_hand/evidence'
SPEC = importlib.util.spec_from_file_location('right_reference', ROOT / 'examples/right_hand/run.py')
REF = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REF)


def coordination_plan(original):
    expected = REF.motion_plan(.002)[:1000]
    if original != expected:
        raise ValueError('Input must be the unchanged frozen first 1000 finger11 commands')
    result = copy.deepcopy(original)
    for row in result:
        row['fractions'][1] = row['fractions'][0]
    return result


def read_prefix(path):
    rows = []
    with gzip.open(path, 'rt') as stream:
        for line in stream:
            if len(rows) == 1000:
                break
            row = json.loads(line)
            if row['step'] != len(rows) + 1:
                raise ValueError('Baseline prefix step sequence changed')
            rows.append(row)
    if len(rows) != 1000:
        raise ValueError('Baseline prefix is incomplete')
    return rows


def identity():
    paths = ['mjcf/nero_dh116_right.xml', 'urdf/dh116_right.urdf',
             'RIGHT_SOURCE_MANIFEST.json', 'examples/right_hand/run.py',
             'examples/right_hand/evidence/frozen-commands.json.gz',
             'examples/right_hand/evidence/hand-motion-state.jsonl.gz',
             'examples/right_hand/evidence/hand-motion-contacts.jsonl.gz',
             'examples/right_hand/evidence/hand-motion-result.json',
             'examples/right_hand/evidence/environment.json',
             'examples/libero_reference/env/provenance.py',
             'examples/libero_reference/env/requirements.lock',
             'examples/libero_reference/env/build.lock']
    files = {}
    for name in paths:
        pinned = subprocess.check_output(['git', 'show', f'{BASELINE}:{name}'], cwd=ROOT)
        actual = (ROOT / name).read_bytes()
        if pinned != actual:
            raise ValueError(f'Frozen input changed: {name}')
        files[name] = hashlib.sha256(actual).hexdigest()
    return {'baseline_commit': BASELINE, 'pinned_files_sha256': files,
            'runtime_preflight': REF.source_preflight(),
            'entry_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def summarize(states, contact_rows):
    result = {'steps': len(states), 'nonfinite_steps': sum(not s['finite_state'] for s in states),
              'warning_steps': sum(bool(s['warnings']) for s in states), 'tracking': {}}
    for name, idx in [('finger11', 0), ('finger12', 1)]:
        steady = [s for s in states if s['steady_window']]
        closed = [s for s in steady if s['phase'].endswith('closed_hold')]
        opened = [s for s in steady if s['phase'].endswith('open_hold')]
        bad = [s for s in steady if abs(s['active_tracking_error_rad'][idx]) > REF.THRESHOLDS['active_steady_tracking_rad']]
        result['tracking'][name] = {
            'max_all_steps_error_rad': max(abs(s['active_tracking_error_rad'][idx]) for s in states),
            'max_closed_steady_error_rad': max(abs(s['active_tracking_error_rad'][idx]) for s in closed),
            'max_open_steady_error_rad': max(abs(s['active_tracking_error_rad'][idx]) for s in opened),
            'first_steady_exceed_step': bad[0]['step'] if bad else None,
            'final_q_rad': states[-1]['robot_state_18'][7 + idx]}
    pair = {'gripper0_base_link', 'gripper0_finger12_link'}
    contacts = [(row['step'], row['time_s'], c) for row in contact_rows for c in row['contacts']
                if set(c['body_names']) == pair]
    bad = [v for v in contacts if -v[2]['distance_m'] > REF.THRESHOLDS['enabled_nonadjacent_self_overlap_m']]
    result['base_finger12'] = {
        'contact_steps': len(set(v[0] for v in contacts)),
        'first_contact_step': contacts[0][0] if contacts else None,
        'first_contact_time_s': contacts[0][1] if contacts else None,
        'first_overlap_exceed_step': bad[0][0] if bad else None,
        'first_overlap_exceed_time_s': bad[0][1] if bad else None,
        'min_contact_distance_m': min((v[2]['distance_m'] for v in contacts), default=None),
        'max_normal_force_N': max((v[2]['normal_force_N'] for v in contacts), default=0.),
        'max_normal_force_step': max(contacts, key=lambda v: v[2]['normal_force_N'])[0] if contacts else None,
        'overlap_exceed_steps': len(set(v[0] for v in bad))}
    first = []
    for row in states:
        if (row['enabled_nonadjacent_self_overlap_m'] > REF.THRESHOLDS['enabled_nonadjacent_self_overlap_m'] or
                (row['steady_window'] and max(abs(x) for x in row['active_tracking_error_rad']) > REF.THRESHOLDS['active_steady_tracking_rad'])):
            first.append(row)
    result.update(max_enabled_nonadjacent_overlap_m=max(s['enabled_nonadjacent_self_overlap_m'] for s in states),
                  first_contact_or_tracking_failure_step=first[0]['step'] if first else None,
                  first_contact_or_tracking_failure_time_s=first[0]['time_s'] if first else None)
    return result


def run_once(model, source, output, plan):
    from robosuite.controllers.joint_pos import JointPositionController
    old_step, old_reset, old_forward = mujoco.mj_step, mujoco.mj_resetDataKeyframe, mujoco.mj_forward
    old_init, old_goal = JointPositionController.__init__, JointPositionController.set_goal
    audit = {'attempted_mj_step_calls': 0, 'completed_mj_step_calls': 0, 'initialization_events': []}

    def snapshot(data):
        return {k: getattr(data, k).copy().tolist() for k in
                ('qpos', 'qvel', 'ctrl', 'act', 'qacc_warmstart', 'mocap_pos', 'mocap_quat', 'eq_active')} | {'time_s': float(data.time)}

    def reset(m, d, key):
        old_reset(m, d, key)
        audit['initialization_events'].append({'event': 'native_reset', 'keyframe': key})
        audit['native_reset_state'] = snapshot(d)

    def forward(m, d, *args, **kwargs):
        old_forward(m, d, *args, **kwargs)
        if not audit['attempted_mj_step_calls']:
            audit['initialization_events'].append({'event': 'forward'})

    def init(self, *args, **kwargs):
        audit['initialization_events'].append({'event': 'JPC_init_begin', 'parameters': REF.json_safe(kwargs) if 'sim' not in kwargs else REF.json_safe({k:v for k,v in kwargs.items() if k!='sim'})})
        old_init(self, *args, **kwargs)
        audit['initialization_events'].append({'event': 'JPC_init_end'})

    def goal(self, *args, **kwargs):
        audit['initialization_events'].append({'event': 'JPC_set_goal', 'args': REF.json_safe(args), 'kwargs': REF.json_safe(kwargs)})
        return old_goal(self, *args, **kwargs)

    def step(m, d):
        if audit['attempted_mj_step_calls'] >= 1000:
            raise RuntimeError('Authorized physical step budget exhausted')
        if not audit['attempted_mj_step_calls']:
            audit['before_first_step'] = snapshot(d)
        audit['attempted_mj_step_calls'] += 1
        value = old_step(m, d)
        audit['completed_mj_step_calls'] += 1
        return value

    mujoco.mj_step, mujoco.mj_resetDataKeyframe, mujoco.mj_forward = step, reset, forward
    JointPositionController.__init__, JointPositionController.set_goal = init, goal
    try:
        case = REF.run_case(model, source, output, 'lambda1', plan)
    finally:
        mujoco.mj_step, mujoco.mj_resetDataKeyframe, mujoco.mj_forward = old_step, old_reset, old_forward
        JointPositionController.__init__, JointPositionController.set_goal = old_init, old_goal
        REF.write_json(output / 'reset-and-step-audit.json', audit)
    return case, audit


def execute(output):
    import xml.etree.ElementTree as ET
    identity_before = identity()
    REF.write_json(output / 'input-identity.json', identity_before)
    with (output / 'environment-check.log').open('w') as log:
        subprocess.run([sys.executable, str(ROOT / 'examples/libero_reference/env/provenance.py'),
                        '--output', str(output / 'environment.json')], check=True, stdout=log, stderr=log)
    environment = json.loads((output / 'environment.json').read_text())
    old_environment = json.loads((EVIDENCE / 'environment.json').read_text())
    for name in ['distributions', 'stock_robosuite_files_verified_against_wheel',
                 'libero_source_manifest_sha256', 'libero_commit']:
        if environment[name] != old_environment[name]:
            raise ValueError(f'Environment differs from recorded lambda=0: {name}')
    if environment['python']['sha256'] != old_environment['python']['sha256']:
        raise ValueError('Python executable differs from baseline')
    with gzip.open(EVIDENCE / 'frozen-commands.json.gz', 'rt') as stream:
        saved = json.load(stream)['hand_motion']
    if saved != REF.motion_plan(.002):
        raise ValueError('Original 7000-command generator differs from saved commands')
    baseline_plan = saved[:1000]
    plan = coordination_plan(baseline_plan)
    REF.write_json(output / 'commands.json', {'lambda0': baseline_plan, 'lambda1': plan,
                    'only_intervention': 'finger12 fraction = original finger11 fraction', 'lambda': 1})
    REF.write_json(output / 'frozen-protocol.json', REF.PROTOCOL)
    model = mujoco.MjModel.from_xml_path(str(REF.MODEL))
    source = ET.parse(REF.URDF).getroot()
    original_result = json.loads((EVIDENCE / 'hand-motion-result.json').read_text())
    if not np.array_equal(model.key_qpos[0], original_result['initial_full_qpos']):
        raise ValueError('Native keyframe qpos differs from historical initial state')
    if np.any(model.key_qvel[0]) or np.any(model.key_ctrl[0]) or model.key_time[0] != 0:
        raise ValueError('Frozen native reset must retain zero qvel/ctrl/time')
    baseline_states = read_prefix(EVIDENCE / 'hand-motion-state.jsonl.gz')
    baseline_contacts = read_prefix(EVIDENCE / 'hand-motion-contacts.jsonl.gz')
    case, audit = run_once(model, source, output, plan)
    new_states = [json.loads(l) for l in (output / 'lambda1-state.jsonl').read_text().splitlines()]
    new_contacts = [json.loads(l) for l in (output / 'lambda1-contacts.jsonl').read_text().splitlines()]
    REF.write_json(output / 'comparison.json', {'diagnostic_only': True, 'lambda0': summarize(baseline_states, baseline_contacts),
                   'lambda1': summarize(new_states, new_contacts), 'geometry_status': 'NOT_YET_ANALYZED',
                   'hypothesis_status': 'REQUIRES_SOURCE_SURFACE_CHECK', 'full_7000_acceptance': 'UNCHANGED_FAIL',
                   'grasp_success': 'UNKNOWN'})
    final_identity = identity()
    if identity_before != final_identity:
        raise RuntimeError('Input identities changed during the physical run')
    if (audit['completed_mj_step_calls'] != 1000 or case['recorded_steps'] != 1000 or
            case['execution_status'] != 'FINISHED'):
        return 1
    expected = original_result['initial_full_qpos']
    if audit['native_reset_state']['qpos'] != expected or audit['before_first_step']['qpos'] != expected:
        raise RuntimeError('Actual native reset or JPC initialization changed initial qpos')
    return 0 if case['screening_status'] == 'PASS' else 2


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='New diagnostic output directory in this checkout')
    args = parser.parse_args()
    output = REF.reserve_output(ROOT, args.output)
    code = 1
    try:
        code = execute(output)
    except Exception as error:
        import traceback
        REF.write_json(output / 'execution-error.json', {'error': str(error), 'traceback': traceback.format_exc()})
    REF.write_json(output / 'execution.json', {'diagnostic_only': True, 'lambda': 1, 'exit_code': code,
        'command': ' '.join(sys.argv), 'baseline': BASELINE,
        'meaning': '0: original numeric screening PASS; 2: FAIL; 1: execution error; source geometry is analyzed separately'})
    print(json.dumps({'output': str(output), 'exit_code': code}))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
