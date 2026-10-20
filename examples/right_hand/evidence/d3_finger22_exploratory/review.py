#!/usr/bin/env python3
"""Review the single completed integration; strict provenance AFTER the run.

Never calls Reset/FK, native collision, controls or integration. All geometry
comes from actual saved final RK4 body poses and frozen compiled assets.
"""
import argparse
import ast
from collections import Counter
import gzip
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
for k in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[k] = '1'
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'examples/right_hand'))
import finger22_contact_adapter as A
import white_bowl_exploratory as E


def write(path, value):
    raw = (json.dumps(value, indent=2, allow_nan=False)+'\n').encode()
    if path.suffix == '.gz':
        with gzip.open(path, 'wb') as f:
            f.write(raw)
    else:
        path.write_bytes(raw)


def verify_original_stops(run):
    """Recompute original physical checks from logs, without simulator APIs."""
    frozen = A.read(run/'frozen-before-first-step.json')
    criteria = frozen['criteria']; protocol = frozen['additional_stop_protocol']
    snapshot = A.read(run/'robot-physical-preflight.json.gz')['actual_snapshot']
    joints = {j['name']: j['fields'] for j in snapshot['objects']['JOINT']}
    names = [j.get('name') for j in ET.parse(ROOT/E.MODEL).findall('worldbody//joint')]
    assert len(names) == 18
    limits = np.array([joints[n]['jnt_range'] for n in names])
    tree = ast.parse((ROOT/'examples/right_hand/white_bowl_run.py').read_text())
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'support_and_opposition')
    scope = {'np': np}; exec(compile(ast.Module(body=[fn], type_ignores=[]), 'saved_pure_support_function', 'exec'), scope)
    support_fn = scope['support_and_opposition']
    previous_time = 0.; failures = []; support_gaps_after_settle = []; verified = 0
    for state, cache in zip(A.rows(run/'state.jsonl.gz'), A.rows(run/'contacts.jsonl.gz')):
        assert state['step'] == cache['step'] == verified+1
        q = np.array(state['robot_state_18']); action = np.array(state['action_13'])
        arm = np.abs(q[:7]-action[:7]); active = np.abs(q[[7,8,10,12,14,16]]-np.array(state['target_qpos_18'])[[7,8,10,12,14,16]])
        expected = np.array([.004592+.8808*q[8]]+[.011072+.9206*q[i] for i in (10,12,14,16)])
        mimic = float(np.max(abs(q[[9,11,13,15,17]]-expected)))
        overshoot = float(max(0., np.max(limits[:,0]-q), np.max(q-limits[:,1])))
        overlap = max([0.]+[-c['distance_m'] for c in cache['contacts'] if not c['filter_reasons']
                           and any(b.startswith(('robot0_', 'gripper0_', 'mount0_')) for b in c['body'])])
        assert arm.tolist() == state['arm_error_rad'] and active.tolist() == state['active_error_rad']
        assert mimic == state['mimic_rad'] and overshoot == state['joint_overshoot_rad']
        assert overlap == state['robot_overlap_m']
        finite = all(np.isfinite(np.array(state[k], dtype=float)).all() for k in
                     ('full_qpos', 'full_qvel', 'full_qacc_cached', 'ctrl', 'actuator_force'))
        reasons = []
        if not finite: reasons.append('NONFINITE_STATE')
        if state['warnings'] or not np.isclose(state['time_s']-previous_time, .002, rtol=0, atol=1e-12):
            reasons.append('NUMERICAL_WARNING_OR_TIME_RESET')
        if max(arm) > criteria['arm_tracking_error_rad_max']: reasons.append('ARM_TRACKING')
        if state['phase'] == 'hold' and max(active) > criteria['steady_active_error_rad_max']: reasons.append('ACTIVE_HOLD_TRACKING')
        if mimic > criteria['linear_mimic_error_rad_max']: reasons.append('MIMIC')
        if overshoot > criteria['joint_limit_overshoot_rad_max']: reasons.append('JOINT_LIMIT')
        if overlap > criteria['eligible_internal_and_external_overlap_m_max']: reasons.append('ROBOT_CONTACT_OVERLAP')
        reasons.extend(E.W.extra_failures(cache['contacts'], q.tolist(), protocol))
        assert reasons == state['failures'] and finite == state['finite']
        support, opposed = support_fn(cache['contacts'], state['bowl_position_m'][:2])
        assert support == state['environment_support'] and opposed == state['opposed_thumb_index_contact']
        if state['step'] >= 250 and not support: support_gaps_after_settle.append(state['step'])
        if reasons: failures.append({'step': state['step'], 'reasons': reasons})
        previous_time = state['time_s']; verified += 1
    return {'steps_independently_recomputed': verified, 'failures_recomputed': failures,
            'support_gaps_after_settle_steps': support_gaps_after_settle,
            'tracking_mimic_limits_overlap_finite_clock_wrist_envelope_and_support_exact': True,
            'new_physics_Reset_FK_collision_calls': 0}


def main(output):
    if output.exists():
        raise ValueError('New analysis output only; never overwrite evidence')
    output.mkdir(parents=True)
    run = HERE/'run'
    states = list(A.rows(run/'state.jsonl.gz'))
    result = A.read(run/'result.json')
    assert result['dynamic_attempts'] == 1
    assert len(states) == result['physics_steps']
    assert [s['step'] for s in states] == list(range(1, len(states)+1))
    assert not any(s['failures'] for s in states[:-1])
    assert states[-1]['failures'] == result['first_failure']['reasons']
    first_difference = {}
    for old, new in zip(A.rows(A.OLD/'run/state.jsonl.gz'), states):
        for key in (old.keys() | new.keys())-{'failures'}:
            if key not in first_difference and old[key] != new[key]:
                first_difference[key] = new['step']
    assert not first_difference
    old_frozen = A.read(A.OLD/'run/frozen-before-first-step.json')
    frozen = A.read(run/'frozen-before-first-step.json')
    for key in ('qpos', 'qvel', 'ctrl', 'controller', 'criteria', 'native_scene'):
        assert old_frozen[key] == frozen[key]
    assert A.read(run/'schedule.json') == A.read(A.OLD/'run/schedule.json')
    assert frozen['additional_stop_protocol'] == A.read(HERE/'registered-protocol.json')
    assert A.read(run/'robot-physical-preflight.json.gz')['actual_snapshot'] == A.read(A.OLD/'run/robot-physical-preflight.json.gz')['actual_snapshot']
    phase_counts = Counter(s['phase'] for s in states)
    landmarks = (1315, 1401, 4518, 4731, 4742, len(states)-1, len(states))
    selected = {}; classifications = []; labels = Counter(); first_hand = None
    other_contacts = 0; matched_old = 0; first_cache_difference = None
    old_rows = iter(A.rows(A.OLD/'run/contacts.jsonl.gz'))
    with A.no_physics() as forbidden:
        physical_validation = verify_original_stops(run)
        adapter = A.prepare_saved_model()
        for cache in A.rows(run/'contacts.jsonl.gz'):
            if cache['step'] <= 4731:
                old = next(old_rows)
                def native_only(c):
                    return {**c, 'contacts': [{k: v for k, v in e.items() if k != 'finger22_region'}
                                              for e in c['contacts']]}
                if native_only(old) != native_only(cache) and first_cache_difference is None:
                    first_cache_difference = cache['step']
                matched_old += 1
            eligible = [c for c in cache['contacts'] if not c['filter_reasons'] and
                        any(b.startswith(('robot0_', 'gripper0_', 'mount0_')) for b in c['body'])]
            if cache['step'] in landmarks:
                selected[str(cache['step'])] = {'state': states[cache['step']-1],
                    'eligible_robot_contacts': sorted(eligible, key=lambda c: c['distance_m']),
                    'cache_timing': cache['timing']}
            for contact in cache['contacts']:
                if (first_hand is None and 'white_bowl_1_main' in contact['body']
                        and any(b.startswith('gripper0_') for b in contact['body'])
                        and contact['force_contact_frame'][0] > 0):
                    first_hand = cache['step']
                if not any(n in A.NAMES for n in contact['geom']):
                    other_contacts += 1
                    continue
                # This AFTER-RUN decision is saved separately, never appended
                # to the actual trajectory or used to rewrite its stop reason.
                decision = adapter.classify_event(contact, cache['timing'], cache['step'], cache['body_poses'])
                classifications.append(decision); labels[decision['status']] += 1
    assert first_cache_difference is None and matched_old == 4731
    final = selected[str(len(states))]
    failed = [c for c in final['eligible_robot_contacts'] if -c['distance_m'] > .002]
    assert failed and states[-1]['robot_overlap_m'] == -min(c['distance_m'] for c in failed)
    prelimit = states[-2]
    assert prelimit['robot_overlap_m'] <= .002
    original_reasons = {'NONFINITE_STATE', 'NUMERICAL_WARNING_OR_TIME_RESET', 'ARM_TRACKING',
                        'ACTIVE_HOLD_TRACKING', 'MIMIC', 'JOINT_LIMIT', 'ROBOT_CONTACT_OVERLAP',
                        'UNEXPECTED_WRIST_EXTERNAL_CONTACT', 'OUTSIDE_REGISTERED_PATH_ENVELOPE'}
    assert set(states[-1]['failures']) <= original_reasons
    protected = A.read(HERE/'protected-before.json.gz')
    changed = [p for p, h in protected['tracked_sha256'].items() if A.sha(ROOT/p) != h]
    assert not changed
    assert A.sha('/data_robot/projects/nero_dh116_assets/README.md') == protected['user_readme_sha256']
    summary = {'status': 'EXPLORATORY_STOPPED_AT_FIRST_ORIGINAL_PHYSICAL_BOUND',
        'authorization': E.AUTHORIZATION, 'baseline_commit': E.BASELINE,
        'actual_physics_steps': len(states), 'dynamic_attempts': 1,
        'crossed4731': len(states) > 4731, 'crossed4742': len(states) > 4742,
        'phase_physics_steps': dict(phase_counts), 'first_failure': result['first_failure'],
        'over2mm_contacts_at_failure': failed, 'prior_step_max_overlap_m': prelimit['robot_overlap_m'],
        'state_control_force_prefix_identical_to_old_run': 4731,
        'native_RK4_contact_cache_prefix_identical_to_old_run': matched_old,
        'first_state_difference': first_difference, 'first_native_cache_difference': first_cache_difference,
        'Reset_controller_criteria_and270actions_exact': True,
        'first_positive_hand_bowl_step': first_hand,
        'original_physical_health': result['control_health'], 'grasp_result': result['grasp_result'],
        'native_task_result': result['native_task_result'], 'strict_grasp_success_claim': result['strict_grasp_success_claim'],
        'lift_from_Reset_m': result['lift_from_reset_m'], 'lift_from_own_settled_m': result['lift_from_own_settled_m'],
        'longest_continuous_valid_hold_s': result['longest_continuous_valid_hold_s'],
        'final_environment_support': states[-1]['environment_support'],
        'maxima': result['maxima'], 'numerical_warnings_seen': any(s['warnings'] for s in states),
        'all_states_finite': all(s['finite'] for s in states),
        'wrist_external_contact_steps': sum(s['wrist_external_contact_count'] > 0 for s in states),
        'posthoc_finger22_event_counts': dict(labels), 'posthoc_finger22_events': len(classifications),
        'other_contacts_not_reclassified': other_contacts,
        'posthoc_decisions_never_used_online': True, 'posthoc_forbidden_API_attempts': forbidden,
        'posthoc_new_physics_Reset_FK_collision_calls': 0,
        'independent_physical_validation': physical_validation,
        'old_files_unchanged': len(protected['tracked_sha256']), 'user_readme_unchanged': True,
        'all_runtime_inputs_unchanged': result['inputs_unchanged'] and result['exploratory_sources_unchanged'],
        'model_sha256': E.MODEL_SHA, 'formal_numeric_status': 'FAIL_UNCHANGED_1e-10m',
        'full_geometry_material_fidelity': 'UNKNOWN_UNCHANGED', 'release_approved': False,
        'limit': 'Native proxy overlap is a measured threshold violation, not proof of true manufacturer-STL material penetration. No lift/hold was attempted.'}
    write(output/'review.json', summary)
    write(output/'selected-states.json.gz', selected)
    write(output/'posthoc-finger22.json.gz', classifications)
    print(json.dumps({k: v for k, v in summary.items() if k != 'over2mm_contacts_at_failure'}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    main(parser.parse_args().output.resolve())
