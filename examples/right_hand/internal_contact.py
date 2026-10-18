#!/usr/bin/env python3
"""Diagnostic-only: one 7000-step run with exactly two internal body exclusions.

Formal assets, original lambda=0 commands, native Reset/JPC and thresholds stay
unchanged. Filtered contact counts are not a geometry or grasp acceptance test.
"""
import contextlib
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BASELINE = '6b432737dbddc7e8917630382c582ac8704d637a'
EVIDENCE = ROOT / 'examples/right_hand/evidence'
PAIRS = (('gripper0_base_link', 'gripper0_finger12_link'),
         ('gripper0_finger12_link', 'gripper0_finger42_link'))
EXCLUDE_LINES = ''.join(f'    <exclude body1="{a}" body2="{b}" />\n' for a, b in PAIRS).encode()
SPEC = importlib.util.spec_from_file_location('right_reference', ROOT / 'examples/right_hand/run.py')
REF = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REF)


def derive_xml(source):
    root = ET.fromstring(source)
    contacts = root.findall('contact')
    old = root.findall('contact/exclude')
    if (len(contacts) != 1 or len(old) != 1 or
            old[0].attrib != {'body1': 'robot0_base_link', 'body2': 'robot0_link1'} or
            root.findall('contact/pair') or source.count(b'  </contact>') != 1):
        raise ValueError('Expected the frozen formal model with its sole NERO exclusion')
    return source.replace(b'  </contact>', EXCLUDE_LINES + b'  </contact>')


def compile_models(derived):
    original = mujoco.MjModel.from_xml_path(str(REF.MODEL))
    # Retain the exact original relative asset references and XML base directory.
    with contextlib.chdir(REF.MODEL.parent):
        candidate = mujoco.MjModel.from_xml_string(derived.decode())
    return original, candidate


def fields(obj, prefix=''):
    result = {}
    for name in dir(obj):
        if name.startswith('_'):
            continue
        value = getattr(obj, name)
        key = prefix + name
        if isinstance(value, (np.ndarray, int, float, str, bytes)):
            result[key] = value
        elif not callable(value) and type(value).__module__ == 'mujoco._structs':
            result.update(fields(value, key + '.'))
    return result


def value_hash(value):
    if isinstance(value, np.ndarray):
        raw = str((value.dtype.str, value.shape)).encode() + value.tobytes()
    elif isinstance(value, bytes):
        raw = value
    else:
        raw = json.dumps(value, sort_keys=True, allow_nan=False).encode()
    return hashlib.sha256(raw).hexdigest()


def fingerprint(model):
    return {key: value_hash(value) for key, value in fields(model).items()}


def eligible(model, first, second):
    b1, b2 = (int(model.geom_bodyid[g]) for g in (first, second))
    w1, w2 = (int(model.body_weldid[b]) for b in (b1, b2))
    if w1 == w2:
        return False
    disabled = int(model.opt.disableflags) & int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
    if not disabled and w1 and w2 and (int(model.body_weldid[model.body_parentid[w1]]) == w2 or
                                      int(model.body_weldid[model.body_parentid[w2]]) == w1):
        return False
    # MuJoCo 3.2.3 excludes match actual body IDs, not all bodies sharing a weld.
    signature = (min(b1, b2) << 16) + max(b1, b2)
    if signature in model.exclude_signature:
        return False
    return bool((int(model.geom_contype[first]) & int(model.geom_conaffinity[second])) or
                (int(model.geom_contype[second]) & int(model.geom_conaffinity[first])))


def check_compiled(original, candidate):
    if original.npair or candidate.npair:
        raise ValueError('Explicit geom pairs would invalidate this dynamic-filter audit')
    ids = {n: mujoco.mj_name2id(original, mujoco.mjtObj.mjOBJ_BODY, n) for p in PAIRS for n in p}
    added = {(min(ids[a], ids[b]) << 16) + max(ids[a], ids[b]) for a, b in PAIRS}
    if min(ids.values()) < 0 or set(candidate.exclude_signature) != set(original.exclude_signature) | added:
        raise ValueError('Exclusion signatures differ from the two authorized actual-body pairs')
    if candidate.nexclude != original.nexclude + 2:
        raise ValueError('Expected exactly two additional exclusions')
    a, b = fields(original), fields(candidate)
    if a.keys() != b.keys():
        raise ValueError('Compiled field inventory changed')
    allowed = {'exclude_signature', 'nexclude', 'nbuffer', 'nnames', 'nnames_map', 'names', 'names_map'}
    different = []
    for key in a:
        equal = np.array_equal(a[key], b[key]) if isinstance(a[key], np.ndarray) else a[key] == b[key]
        if equal:
            continue
        different.append(key)
        if key.startswith('name_') and key.endswith('adr'):
            def names(model, addresses):
                return [model.names[int(i):].split(b'\0', 1)[0] for i in addresses]
            x, y = names(original, a[key]), names(candidate, b[key])
            if key == 'name_excludeadr':
                if y != x + [b'', b'']:
                    raise ValueError('Unexpected exclusion names')
            elif x != y:
                raise ValueError('Object names or IDs changed: ' + key)
        elif key not in allowed:
            raise ValueError('Unapproved compiled field change: ' + key)
    if candidate.nnames != original.nnames + 2 or candidate.nnames_map != original.nnames_map + 4:
        raise ValueError('Unexpected anonymous-exclusion name storage')
    changed = []
    for i in range(original.ngeom):
        for j in range(i + 1, original.ngeom):
            old, new = eligible(original, i, j), eligible(candidate, i, j)
            if old != new:
                info = REF.pair_names(original, i, j)
                if not old or new or tuple(sorted(info['body_names'])) not in {tuple(sorted(p)) for p in PAIRS}:
                    raise ValueError('Collision filtering escaped the authorized two pairs')
                changed.append({**info, 'geom_ids': [i, j], 'original_eligible': old, 'candidate_eligible': new})
    if len(changed) != 2:
        raise ValueError('Expected exactly two effective geom-pair eligibility changes')
    hand = [i for i in range(original.ngeom) if REF.object_name(original, mujoco.mjtObj.mjOBJ_GEOM, i).startswith('gripper0_')
            and '_collision_' in REF.object_name(original, mujoco.mjtObj.mjOBJ_GEOM, i)]
    if len(hand) != 17 or any(candidate.geom_contype[i] != 1 or candidate.geom_conaffinity[i] != 1 for i in hand):
        raise ValueError('The 17 hand collision masks must remain 1/1')
    # Independent hypothetical external body: any 32-bit mask is a combination
    # of these one-bit masks. No probe geometry or dynamics is added.
    external_changes = 0
    bits = [0] + [1 << n for n in range(32)]
    for i in range(original.ngeom):
        for ct in bits:
            for ca in bits:
                old = bool((int(original.geom_contype[i]) & ca) or (ct & int(original.geom_conaffinity[i])))
                new = bool((int(candidate.geom_contype[i]) & ca) or (ct & int(candidate.geom_conaffinity[i])))
                external_changes += old != new
    if external_changes:
        raise ValueError('External contact eligibility changed')
    return {'original_exclude_signatures': original.exclude_signature.tolist(),
            'candidate_exclude_signatures': candidate.exclude_signature.tolist(),
            'body_weld_mapping': {n: {'body_id': int(i), 'weld_id': int(original.body_weldid[i])} for n, i in ids.items()},
            'public_fields_compared': len(a), 'identical_fields': len(a) - len(different),
            'different_fields': different, 'field_sha256': {'original': fingerprint(original), 'candidate': fingerprint(candidate)},
            'eligibility_pairs_checked': original.ngeom * (original.ngeom - 1) // 2,
            'eligibility_changed_pairs': changed, 'external_mask_cases_checked': original.ngeom * len(bits)**2,
            'external_eligibility_changed': external_changes, 'external_measurement': 'STRUCTURAL_ONLY_NO_OBJECT_RUN',
            'hand_collision_geoms_one_one': len(hand), 'old_excludes_preserved': True,
            'excluded_pairs_geometry': 'FILTERED; previously proved source-surface crossings remain a risk; no geometry PASS'}


def read_rows(path):
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt') as stream:
        rows = [json.loads(line) for line in stream]
    if len(rows) != 7000 or [r['step'] for r in rows] != list(range(1, 7001)):
        raise ValueError('Expected exactly 7000 sequential saved states/contact rows')
    return rows


def summarize(states, contacts):
    result = {'steps': len(states), 'nonfinite_steps': sum(not s['finite_state'] for s in states),
              'warning_steps': sum(bool(s['warnings']) for s in states), 'tracking': {}}
    for i, name in enumerate(REF.ACTIVE):
        steady = [s for s in states if s['steady_window']]
        bad = [s for s in steady if abs(s['active_tracking_error_rad'][i]) > REF.THRESHOLDS['active_steady_tracking_rad']]
        result['tracking'][name] = {'max_all_error_rad': max(abs(s['active_tracking_error_rad'][i]) for s in states),
            'max_steady_error_rad': max(abs(s['active_tracking_error_rad'][i]) for s in steady),
            'first_steady_exceed_step': bad[0]['step'] if bad else None,
            'steady_exceed_steps': len(bad),
            'own_closed_steady_error_rad': max(abs(s['active_tracking_error_rad'][i]) for s in steady if s['phase'] == name + '/closed_hold'),
            'all_six_closed_steady_error_rad': max(abs(s['active_tracking_error_rad'][i]) for s in steady if s['phase'] == 'all_six/closed_hold')}
    for key in ['arm_tracking_error_rad', 'enabled_nonadjacent_self_overlap_m']:
        result['max_' + key] = max(float(np.max(np.abs(s[key]))) for s in states)
    result['max_mimic_residual_rad'] = max(abs(v) for s in states for v in s['mimic_residual_rad'].values())
    result['max_joint_limit_overshoot_rad'] = max(abs(v) for s in states for v in s['joint_limit_overshoot_rad'].values())
    groups = {}
    for row in contacts:
        for c in row['contacts']:
            name = ' <-> '.join(sorted(c['body_names']))
            g = groups.setdefault(name, {'contact_steps': set(), 'first_step': row['step'], 'min_distance_m': c['distance_m'], 'max_normal_force_N': 0., 'enabled_nonadjacent': c['enabled_nonadjacent_robot_self']})
            g['contact_steps'].add(row['step']); g['min_distance_m'] = min(g['min_distance_m'], c['distance_m'])
            g['max_normal_force_N'] = max(g['max_normal_force_N'], c['normal_force_N'])
    result['contact_pairs'] = {k: v | {'contact_steps': len(v['contact_steps'])} for k, v in groups.items()}
    failures = []
    for s in states:
        if s['steady_window'] and max(abs(v) for v in s['active_tracking_error_rad']) > REF.THRESHOLDS['active_steady_tracking_rad']:
            failures.append({'step': s['step'], 'time_s': s['time_s'], 'kind': 'ACTIVE_STEADY_TRACKING'})
        if s['enabled_nonadjacent_self_overlap_m'] > REF.THRESHOLDS['enabled_nonadjacent_self_overlap_m']:
            failures.append({'step': s['step'], 'time_s': s['time_s'], 'kind': 'ENABLED_NONADJACENT_SELF_OVERLAP'})
    result['first_tracking_or_contact_failure'] = min(failures, key=lambda x: x['step']) if failures else None
    return result


def observed_run(model, source, output, plan):
    from robosuite.controllers.joint_pos import JointPositionController
    old_step, old_reset, old_forward = mujoco.mj_step, mujoco.mj_resetDataKeyframe, mujoco.mj_forward
    old_init, old_goal = JointPositionController.__init__, JointPositionController.set_goal
    audit = {'attempted_mj_step_calls': 0, 'completed_mj_step_calls': 0, 'initialization_events': []}
    def snapshot(d):
        state = np.empty(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION))
        mujoco.mj_getState(model, d, state, mujoco.mjtState.mjSTATE_INTEGRATION)
        return {k: getattr(d, k).copy().tolist() for k in ('qpos', 'qvel', 'ctrl', 'act', 'qacc_warmstart', 'mocap_pos', 'mocap_quat', 'eq_active', 'qfrc_applied', 'xfrc_applied', 'userdata', 'plugin_state')} | {'time_s': float(d.time), 'integration_state': state.tolist()}
    def reset(m, d, key):
        old_reset(m, d, key); audit['initialization_events'].append('native_reset')
        audit['native_reset_state'] = snapshot(d)
    def forward(m, d, *args, **kwargs):
        value = old_forward(m, d, *args, **kwargs)
        if not audit['attempted_mj_step_calls']: audit['initialization_events'].append('forward')
        return value
    def init(self, *args, **kwargs):
        audit['initialization_events'].append('JPC_init_begin'); old_init(self, *args, **kwargs)
        audit['initialization_events'].append('JPC_init_end')
    def goal(self, *args, **kwargs):
        value = old_goal(self, *args, **kwargs); audit['initialization_events'].append('JPC_set_goal')
        audit['after_set_goal'] = snapshot(self.sim.data._data)
        return value
    def step(m, d):
        if m is not model or audit['attempted_mj_step_calls'] >= 7000:
            raise RuntimeError('Model identity or authorized 7000-step budget violated')
        if not audit['attempted_mj_step_calls']:
            audit['before_first_step'] = snapshot(d)
            expected = json.loads((EVIDENCE / 'hand-motion-result.json').read_text())['initial_full_qpos']
            first = json.loads(gzip.open(EVIDENCE / 'hand-motion-state.jsonl.gz', 'rt').readline())
            if d.qpos.tolist() != expected or np.any(d.qvel) or d.time != 0 or d.ctrl.tolist() != first['internal_actuator_ctrl_13']:
                raise ValueError('Initial state or first controller command differs from historical baseline')
            if np.any(d.qfrc_applied) or np.any(d.xfrc_applied):
                raise ValueError('Unexpected external applied forces')
            REF.write_json(output / 'physical-attempt.json', {'authorized_candidate': 1, 'planned_steps': 7000})
        audit['attempted_mj_step_calls'] += 1
        value = old_step(m, d)
        audit['completed_mj_step_calls'] += 1
        return value
    mujoco.mj_step, mujoco.mj_resetDataKeyframe, mujoco.mj_forward = step, reset, forward
    JointPositionController.__init__, JointPositionController.set_goal = init, goal
    before = fingerprint(model)
    try:
        case = REF.run_case(model, source, output, 'filtered', plan)
    finally:
        mujoco.mj_step, mujoco.mj_resetDataKeyframe, mujoco.mj_forward = old_step, old_reset, old_forward
        JointPositionController.__init__, JointPositionController.set_goal = old_init, old_goal
        audit['all_compiled_fields_unchanged_during_run'] = before == fingerprint(model)
        REF.write_json(output / 'reset-and-step-audit.json', audit)
    return case, audit


def identity():
    paths = ['mjcf/nero_dh116_right.xml', 'urdf/dh116_right.urdf', 'RIGHT_SOURCE_MANIFEST.json',
        'examples/right_hand/run.py', 'examples/right_hand/evidence/frozen-commands.json.gz',
        'examples/right_hand/evidence/hand-motion-state.jsonl.gz', 'examples/right_hand/evidence/hand-motion-contacts.jsonl.gz',
        'examples/right_hand/evidence/hand-motion-result.json', 'examples/right_hand/evidence/environment.json',
        'examples/libero_reference/env/provenance.py', 'examples/libero_reference/env/requirements.lock', 'examples/libero_reference/env/build.lock']
    hashes = {}
    for name in paths:
        actual = (ROOT / name).read_bytes()
        if actual != subprocess.check_output(['git', 'show', f'{BASELINE}:{name}'], cwd=ROOT):
            raise ValueError('Frozen baseline input changed: ' + name)
        hashes[name] = hashlib.sha256(actual).hexdigest()
    return {'baseline': BASELINE, 'sha256': hashes, 'runtime_preflight': REF.source_preflight(),
            'entry_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def execute(output, preflight_only=False):
    before = identity(); REF.write_json(output / 'input-identity.json', before)
    with (output / 'environment-check.log').open('w') as log:
        subprocess.run([sys.executable, str(ROOT / 'examples/libero_reference/env/provenance.py'), '--output', str(output / 'environment.json')], check=True, stdout=log, stderr=log)
    env = json.loads((output / 'environment.json').read_text()); old_env = json.loads((EVIDENCE / 'environment.json').read_text())
    for key in ['distributions', 'stock_robosuite_files_verified_against_wheel', 'libero_source_manifest_sha256', 'libero_commit']:
        if env[key] != old_env[key]: raise ValueError('Historical environment identity changed: ' + key)
    if env['python']['sha256'] != old_env['python']['sha256']: raise ValueError('Interpreter identity changed')
    plan = json.load(gzip.open(EVIDENCE / 'frozen-commands.json.gz', 'rt'))['hand_motion']
    if plan != REF.motion_plan(.002): raise ValueError('Frozen original 7000-command plan changed')
    baseline_states = read_rows(EVIDENCE / 'hand-motion-state.jsonl.gz')
    baseline_contacts = read_rows(EVIDENCE / 'hand-motion-contacts.jsonl.gz')
    for row, command in zip(baseline_states, plan):
        if row['phase'] != command['phase'] or row['steady_window'] != command['steady'] or row['robot_action_13'][7:] != [2*v-1 for v in command['fractions']]:
            raise ValueError('Historical actions or phases do not match the frozen plan')
    derived = derive_xml(REF.MODEL.read_bytes()); (output / 'diagnostic.xml').write_bytes(derived)
    REF.write_json(output / 'frozen-commands.json', plan); REF.write_json(output / 'frozen-protocol.json', REF.PROTOCOL)
    original, candidate = compile_models(derived)
    report = check_compiled(original, candidate); report['diagnostic_xml_sha256'] = hashlib.sha256(derived).hexdigest()
    report['xml_base_directory'] = 'mjcf'; REF.write_json(output / 'compiled-comparison.json', report)
    old_result = json.loads((EVIDENCE / 'hand-motion-result.json').read_text())
    if candidate.key_qpos[0].tolist() != old_result['initial_full_qpos'] or np.any(candidate.key_qvel[0]) or np.any(candidate.key_ctrl[0]) or candidate.key_time[0] != 0:
        raise ValueError('Native Reset keyframe differs from history')
    if preflight_only:
        REF.write_json(output / 'preflight-status.json', {'ready': True, 'physics_steps': 0, 'planned_steps': 7000})
        return 0
    case, audit = observed_run(candidate, ET.parse(REF.URDF).getroot(), output, plan)
    if audit['completed_mj_step_calls'] != 7000 or audit['attempted_mj_step_calls'] != 7000 or case['recorded_steps'] != 7000 or case['execution_status'] != 'FINISHED':
        raise RuntimeError('The sole authorized candidate did not complete; no automatic retry')
    states, contacts = read_rows(output / 'filtered-state.jsonl'), read_rows(output / 'filtered-contacts.jsonl')
    first = {}
    for old, new in zip(baseline_states, states):
        for key in ['phase', 'steady_window', 'time_s', 'robot_action_13', 'hand_target_rad']:
            if old[key] != new[key]: raise RuntimeError('Frozen actual command or schedule changed: ' + key)
        if old['internal_actuator_ctrl_13'][7:] != new['internal_actuator_ctrl_13'][7:]: raise RuntimeError('Hand actuator targets changed')
        for key in ['full_qpos', 'full_qvel', 'actuator_force', 'internal_actuator_ctrl_13']:
            if key not in first and old[key] != new[key]:
                first[key] = {'step': new['step'], 'time_s': new['time_s'], 'baseline': old[key], 'candidate': new[key], 'max_absolute_difference': max(abs(a-b) for a,b in zip(old[key],new[key]))}
    excluded = {frozenset(p) for p in PAIRS}
    if any(frozenset(c['body_names']) in excluded for row in contacts for c in row['contacts']):
        raise RuntimeError('An excluded body pair still generated contact')
    REF.write_json(output / 'comparison.json', {'baseline': summarize(baseline_states, baseline_contacts), 'candidate': summarize(states, contacts),
        'first_exact_state_divergence': first, 'all_7000_actions_targets_phases_times_equal': True,
        'excluded_pairs': [{'body_names': p, 'status': 'FILTERED—original source-surface intersection risk remains'} for p in PAIRS],
        'candidate_numeric_screening': case['screening_status'], 'formal_model_acceptance': 'UNCHANGED_FAIL',
        'grasp': 'UNKNOWN', 'external_contact': 'NOT_RUN', 'separate_A_B_causal_effects': 'NOT_IDENTIFIED'})
    if before != identity() or not audit['all_compiled_fields_unchanged_during_run']:
        raise RuntimeError('A frozen input or compiled model field changed during execution')
    return 0 if case['screening_status'] == 'PASS' else 2


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--output', required=True)
    parser.add_argument('--preflight-only', action='store_true', help='Check input identity and compiled policy without forward/integration')
    args = parser.parse_args(); output = REF.reserve_output(ROOT, args.output); code = 1
    try:
        code = execute(output, args.preflight_only)
    except Exception as error:
        import traceback
        REF.write_json(output / 'execution-error.json', {'error': str(error), 'traceback': traceback.format_exc()})
    REF.write_json(output / 'execution.json', {'diagnostic_only': True, 'preflight_only': args.preflight_only, 'baseline': BASELINE, 'exit_code': code, 'command': ' '.join(sys.argv),
        'meaning': '0 numeric screening PASS under diagnostic exclusions; 2 numeric FAIL; 1 preflight/runtime error; not geometry or grasp acceptance'})
    print(json.dumps({'output': str(output), 'exit_code': code})); return code


if __name__ == '__main__':
    raise SystemExit(main())
