#!/usr/bin/env python3
"""Compare existing accepted left and failed right records without MuJoCo APIs.

Run with the existing right .reference-env Python (numpy only). No model is
compiled; all geometry comparisons read saved XML and mesh bytes. No state is
restored and no physics, forward, collision, or constraint code is called.
"""
from pathlib import Path
import argparse
import gzip
import hashlib
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[4]
RIGHT = ROOT / 'examples/right_hand/evidence/d3_bowl78_slow_001'
LEFT = ROOT / 'examples/libero_reference'
STEPS = (1314, 1315, 1321, 1326, 1400, 1401)
PAIR = {'robot0_link5', 'robot0_gripper_flange'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def path_label(path):
    try:
        return str(Path(path).relative_to(ROOT))
    except ValueError:
        return str(path)


def rows(path):
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt') as stream:
        return [json.loads(line) for line in stream]


def raw(path):
    return gzip.decompress(Path(path).read_bytes()) if str(path).endswith('.gz') else Path(path).read_bytes()


def fields(element):
    return {'tag': element.tag, 'attributes': dict(element.attrib)}


def body_records(xml, prefix):
    parents = {child: parent for parent in xml.iter() for child in parent}
    result = {}
    for body in xml.findall('.//body'):
        name = body.get('name', '')
        if name.startswith(prefix):
            result[name] = {
                'parent': parents[body].get('name', parents[body].tag),
                'attributes': dict(body.attrib),
                'local_elements': [fields(e) for e in body if e.tag != 'body'],
            }
    return result


def differing(left, right):
    return {name: {'left': left.get(name), 'right': right.get(name)}
            for name in sorted(left.keys() | right.keys()) if left.get(name) != right.get(name)}


def mesh_inventory(xml, model_path):
    assets = {e.get('name'): e for e in xml.findall('asset/mesh')}
    result = {}
    for body in xml.findall('.//body'):
        if not body.get('name', '').startswith(('robot0_', 'mount0_')):
            continue
        for geom in body.findall('geom'):
            name = geom.get('mesh')
            if not name:
                continue
            asset = assets[name]
            file = Path(asset.get('file'))
            if not file.is_absolute():
                meshdir = xml.find('compiler').get('meshdir', '')
                file = model_path.parent / meshdir / file
            result[name] = {
                'body': body.get('name'), 'geom': geom.get('name'),
                'mesh_xml_without_file': {k: v for k, v in asset.attrib.items() if k != 'file'},
                'file': str(file), 'file_basename': file.name, 'sha256': sha(file),
                'collision': bool(int(geom.get('contype', '1')) or int(geom.get('conaffinity', '1'))),
            }
    return result


def contact_records(row):
    return [{'geoms': c['geom'], 'distance_m': c['distance_m'],
             'normal_force_N': c['force_contact_frame'][0]}
            for c in row['contacts'] if set(c['body']) == PAIR]


def summarize_trace(trace, contacts, initial, target_key, ctrl_key, time_key):
    q = np.asarray([r['full_qpos'][:7] for r in trace])
    target = np.asarray([r[target_key][:7] for r in trace])
    ctrl = np.asarray([r[ctrl_key][:7] for r in trace])
    error = np.abs(q-target)
    records = [(i, c) for i, row in enumerate(contacts) for c in contact_records(row)]
    result = {
        'rows': len(trace), 'recorded_end_time_s': trace[-1][time_key],
        'initial_arm7_rad': initial, 'first_post_step_arm7_rad': q[0].tolist(),
        'actual_min_rad': q.min(0).tolist(), 'actual_max_rad': q.max(0).tolist(),
        'target_min_rad': target.min(0).tolist(), 'target_max_rad': target.max(0).tolist(),
        'max_abs_excursion_from_initial_rad': np.abs(q-np.asarray(initial)).max(0).tolist(),
        'sum_abs_sample_to_sample_movement_rad': np.abs(np.diff(q, axis=0)).sum(0).tolist(),
        'max_abs_target_error_rad': error.max(0).tolist(),
        'arm_ctrl_min_Nm': ctrl.min(0).tolist(), 'arm_ctrl_max_Nm': ctrl.max(0).tolist(),
        'link5_flange_contact_events': len(records),
        'link5_flange_contact_rows': len({i for i, _ in records}),
        'first_link5_flange_contact': {'step_one_based': records[0][0]+1, **records[0][1]} if records else None,
        'q6_max_tracking_error': {'step_one_based': int(error[:, 5].argmax())+1,
                                  'error_rad': float(error[:, 5].max())},
        'phases': {},
    }
    for phase in dict.fromkeys(r['phase'] for r in trace):
        ii = [i for i, r in enumerate(trace) if r['phase'] == phase]
        result['phases'][phase] = {
            'first_step': ii[0]+1, 'last_step': ii[-1]+1,
            'first_target_arm7_rad': target[ii[0]].tolist(),
            'last_target_arm7_rad': target[ii[-1]].tolist(),
            'q6_actual_min_rad': float(q[ii, 5].min()), 'q6_actual_max_rad': float(q[ii, 5].max()),
            'q6_max_target_error_rad': float(error[ii, 5].max()),
            'link5_flange_contact_rows': sum(bool(contact_records(contacts[i])) for i in ii),
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left-run', type=Path,
                        default=Path('/data_robot/verification/nero_dh116_d1b/runs/d2/reference_001'))
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('left-reference.json'))
    args = parser.parse_args()
    left_trace_path = LEFT / 'evidence/d2/bounded_grasp.jsonl.gz'
    left_contact_path = LEFT / 'evidence/contact-details.jsonl.gz'
    left_model_path = args.left_run / 'bounded_grasp-model.xml'
    input_paths = [left_trace_path, left_contact_path, left_model_path,
                   args.left_run / 'bounded_grasp.jsonl', args.left_run / 'contact-details.jsonl',
                   args.left_run / 'full-reset-state.json',
                   LEFT / 'evidence/artifacts.json', LEFT / 'evidence/d2/artifacts.json',
                   LEFT / 'evidence/d2/run-result.json', LEFT / 'evidence/d2/runtime-preflight.json',
                   LEFT / 'evidence/closure-side.provenance.json', LEFT / 'fixtures/manifest.json',
                   ROOT / 'mjcf/nero_dh116.xml',
                   RIGHT / 'model.xml', RIGHT / 'state.jsonl.gz', RIGHT / 'contacts.jsonl.gz',
                   RIGHT / 'frozen-before-first-step.json',
                   ROOT / 'examples/right_hand/candidates/finger13_local/model.xml',
                   ROOT / 'examples/right_hand/evidence/d3_bowl_preflight/left-contact-summary.json']
    provenance = {path_label(p): sha(p) for p in input_paths}
    lt, lc = rows(left_trace_path), rows(left_contact_path)
    rt, rc = rows(RIGHT / 'state.jsonl.gz'), rows(RIGHT / 'contacts.jsonl.gz')
    lxml, rxml = ET.parse(left_model_path).getroot(), ET.parse(RIGHT / 'model.xml').getroot()
    frozen = read_json(RIGHT / 'frozen-before-first-step.json')
    lreset = read_json(args.left_run / 'full-reset-state.json')
    d2_result = read_json(LEFT / 'evidence/d2/run-result.json')
    manifest = read_json(LEFT / 'fixtures/manifest.json')
    artifact_records = read_json(LEFT / 'evidence/artifacts.json')['artifacts']
    contact_manifest = next(r for r in artifact_records if r['file'] == 'contact-details.jsonl.gz')
    d2_artifacts = read_json(LEFT / 'evidence/d2/artifacts.json')['artifacts']
    d2_trace_manifest = next(r for r in d2_artifacts if r['file'] == 'bounded_grasp.jsonl.gz')
    d2_preflight = read_json(LEFT / 'evidence/d2/runtime-preflight.json')
    left_summary = read_json(ROOT / 'examples/right_hand/evidence/d3_bowl_preflight/left-contact-summary.json')
    trace_raw_sha = hashlib.sha256(raw(left_trace_path)).hexdigest()
    contact_raw_sha = hashlib.sha256(raw(left_contact_path)).hexdigest()
    bindings = {
        'left_trace_raw_sha256': trace_raw_sha, 'left_contacts_raw_sha256': contact_raw_sha,
        'trace_matches_d2_original_raw': raw(left_trace_path) == raw(args.left_run / 'bounded_grasp.jsonl'),
        'contacts_match_d2_original_raw': raw(left_contact_path) == raw(args.left_run / 'contact-details.jsonl'),
        'trace_matches_d2_artifacts_manifest': sha(left_trace_path) == d2_trace_manifest['sha256']
            and trace_raw_sha == d2_trace_manifest['source_sha256'],
        'trace_matches_fixture_manifest': trace_raw_sha == manifest['golden_uncompressed_sha256']
            and sha(left_trace_path) == manifest['files']['golden/bounded_grasp.jsonl.gz'],
        'contacts_match_original_artifact_manifest': sha(left_contact_path) == contact_manifest['sha256']
            and contact_raw_sha == contact_manifest['source_sha256'],
        'trace_and_contacts_row_time_exact': len(lt) == len(lc)
            and all(i == c['row'] and s['time'] == c['time'] for i, (s, c) in enumerate(zip(lt, lc))),
        'trace_and_contacts_arm7_control_force_exact': all(
            s['internal_actuator_ctrl_13'][:7] == c['actuator_force'][:7] for s, c in zip(lt, lc)),
        'preflight_contact_source_matches': sha(left_contact_path) == left_summary['inputs'][
            'examples/libero_reference/evidence/contact-details.jsonl.gz'],
        'saved_scene_matches_accepted_video_source_sha': sha(left_model_path) == next(
            v for k, v in read_json(LEFT / 'evidence/closure-side.provenance.json')['source_file_sha256'].items()
            if k.endswith('bounded_grasp-model.xml')),
        'left_source_matches_d2_result_sha': sha(ROOT / 'mjcf/nero_dh116.xml') == d2_result['robot_sha256'],
        'actual_right_scene_matches_frozen_sha': sha(RIGHT / 'model.xml') == frozen['model_xml_sha256'],
        'actual_right_robot_matches_frozen_sha': sha(ROOT / frozen['robot_model']) == frozen['robot_model_sha256'],
    }
    checks = [v for v in bindings.values() if isinstance(v, bool)]
    assert all(checks), bindings
    lbody = body_records(lxml, ('robot0_', 'mount0_'))
    rbody = body_records(rxml, ('robot0_', 'mount0_'))
    lmesh, rmesh = mesh_inventory(lxml, left_model_path), mesh_inventory(rxml, RIGHT / 'model.xml')
    mesh_comparison = []
    for name in sorted(lmesh.keys() | rmesh.keys()):
        lm, rm = lmesh.get(name), rmesh.get(name)
        mesh_comparison.append({'name': name, 'left': lm, 'right': rm,
                                'physical_asset_same': lm is not None and rm is not None
                                and lm['sha256'] == rm['sha256']
                                and lm['mesh_xml_without_file'] == rm['mesh_xml_without_file']})
    qleft = np.asarray([r['full_qpos'][:7] for r in lt])
    nearby = []
    for step in STEPS:
        right = rt[step-1]
        rq = np.asarray(right['full_qpos'][:7])
        distances = np.linalg.norm(qleft-rq, axis=1)
        i = int(distances.argmin())
        left = lt[i]
        delta = qleft[i]-rq
        nearby.append({
            'right_step_one_based': step, 'right_phase': right['phase'],
            'right_arm7_actual_rad': rq.tolist(), 'right_arm7_target_rad': right['action_13'][:7],
            'right_q6_actual_rad': float(rq[5]), 'right_q6_target_rad': right['action_13'][5],
            'right_q6_target_error_rad': abs(rq[5]-right['action_13'][5]), 'right_q6_ctrl_Nm': right['ctrl'][5],
            'right_link5_flange_contacts': contact_records(rc[step-1]),
            'nearest_left_step_one_based': i+1, 'nearest_left_row_zero_based': i,
            'left_phase': left['phase'], 'left_time_s': left['time'],
            'left_arm7_actual_rad': qleft[i].tolist(), 'left_arm7_target_rad': left['robot_action_13'][:7],
            'left_q6_actual_rad': float(qleft[i, 5]), 'left_q6_target_rad': left['robot_action_13'][5],
            'left_q6_target_error_rad': abs(qleft[i, 5]-left['robot_action_13'][5]),
            'left_q6_ctrl_Nm': left['internal_actuator_ctrl_13'][5],
            'left_link5_flange_contacts': contact_records(lc[i]),
            'left_minus_right_actual_arm7_rad': delta.tolist(),
            'joint7_absolute_difference_rad': abs(float(delta[6])),
            'joint_space_L2_distance_rad': float(distances[i]),
            'joint_space_Linf_distance_rad': float(np.max(np.abs(delta))),
        })
    arm_actuators = lambda xml: {e.get('name'): fields(e) for e in xml.findall('actuator/*')
                                 if e.get('joint', '').startswith('robot0_')}
    hand_physical = lambda xml: {
        name: {**{k: v for k, v in b.items() if k != 'local_elements'},
               'local_elements': [e for e in b['local_elements'] if e['tag'] in ('joint', 'inertial')]}
        for name, b in body_records(xml, ('gripper0_',)).items()}
    lhand, rhand = hand_physical(lxml), hand_physical(rxml)
    mass = lambda xml: sum(float(e.get('mass')) for b in xml.findall('.//body')
                           if b.get('name', '').startswith('gripper0_') for e in b.findall('inertial'))
    controller_left = d2_preflight['controller']
    controller_right = frozen['controller']
    out = {
        'schema': 'd3-link5-flange-existing-left-reference-v1',
        'method': 'Read existing JSONL/XML/mesh bytes only; numpy joint-space arithmetic; no MuJoCo import.',
        'new_physics_steps': 0, 'reset_calls': 0, 'forward_calls': 0,
        'collision_calls': 0, 'constraint_calls': 0, 'compiled_models': 0,
        'script_sha256': sha(__file__), 'python': sys.version, 'numpy': np.__version__,
        'input_sha256': provenance, 'accepted_trace_binding': bindings,
        'left_contacts_observation_status': 'OBSERVED_SAME_ACCEPTED_D2_TRACE',
        'binding_explanation': 'Repository contact gzip originates in acceptance_001; its decompressed bytes also exactly match the existing D2 reference_001 contact file. D2 and golden trace hashes, row/time, and all arm ctrl/force entries agree. No new run was made.',
        'geometry': {
            'comparison_scope': 'Saved native scenes; full named NERO arm/flange/mount local elements and asset byte SHA; hand geometry excluded from equality claim.',
            'arm_flange_mount_body_count': len(lbody),
            'arm_flange_mount_body_local_element_differences': differing(lbody, rbody),
            'arm_motor_differences': differing(arm_actuators(lxml), arm_actuators(rxml)),
            'arm_collision_proxy_count': sum(m['collision'] and 'candidate_nero_' in name for name, m in lmesh.items()),
            'flange_collision_proxy_count': sum(m['collision'] and 'source_flange_part_' in name for name, m in lmesh.items()),
            'all_referenced_arm_flange_mount_mesh_bytes_and_mesh_parameters_equal': all(x['physical_asset_same'] for x in mesh_comparison),
            'mesh_assets': mesh_comparison,
            'critical_body_layout_snapshot': {name: lbody[name] for name in (
                'robot0_base_link', 'robot0_link5', 'robot0_link6', 'robot0_link7', 'robot0_gripper_flange', 'mount0_base')},
            'hand_mount_parent_and_pose': {side: {'parent': body_records(xml, ('gripper0_',))['gripper0_base_link']['parent'],
                                                'attributes': body_records(xml, ('gripper0_',))['gripper0_base_link']['attributes']}
                                           for side, xml in [('left', lxml), ('right', rxml)]},
            'hand_saved_physical_field_differences': differing(lhand, rhand),
            'hand_field_comparison_semantics': 'Exact XML attributes, including alternative inertia representations; not a canonical inertia-tensor equivalence test. Declared hand mass totals are independently compared.',
            'hand_total_declared_mass_kg': {'left': mass(lxml), 'right': mass(rxml)},
            'scene_options': {'left': lxml.find('option').attrib, 'right': rxml.find('option').attrib},
            'exclude_elements': {'left': [fields(e) for e in lxml.findall('contact/*')],
                                 'right': [fields(e) for e in rxml.findall('contact/*')]},
        },
        'arm_controller': {
            'left_source': 'accepted D2 runtime-preflight.json controller',
            'right_source': 'actual frozen-before-first-step.json controller',
            'comparison': {key: {'left': controller_left[key], 'right': controller_right[key],
                                 'exact_equal': controller_left[key] == controller_right[key]}
                           for key in ('kp', 'kd', 'initial_joint', 'goal_qpos')},
            'arm7_initial_exact_equal': lreset['qpos'][:7] == frozen['qpos'][:7],
            'control_semantics': '7D absolute joint position target in rad; recorded ctrl is native arm motor torque in Nm, not a position target.',
        },
        'left_trace': summarize_trace(lt, lc, lreset['qpos'][:7], 'robot_action_13', 'internal_actuator_ctrl_13', 'time'),
        'right_trace': summarize_trace(rt, rc, frozen['qpos'][:7], 'action_13', 'ctrl', 'time_s'),
        'nearest_recorded_left_poses': {
            'metric': 'argmin over all 5750 existing left samples of sqrt(sum((q_left[j]-q_right[j])**2 for j=1..7)); unwrapped raw rad, equal joint weights; first index breaks ties.',
            'acceptance_threshold_created': False, 'same_pose_claim': False, 'rows': nearby,
        },
        'interpretation_limits': [
            'Static NERO/flange/mount equality and equal arm gains establish comparable hardware geometry, not comparable executed trajectories or equal hand load.',
            'Left and right hand mass/inertia, hand configurations, arm targets, durations and actual joint7 paths differ; contact-free left path cannot establish right-pose clearance.',
            'No same-pose or same-control counterfactual exists in these records. Nearest left poses are approximately 0.145 rad away in arm7 L2 and must not be called matched poses.',
            'No left link5/flange contacts were recorded over its 5750 sampled RK4 final caches; no continuous-time separation or signed distance is inferred.',
            'Left contacts omit world contact positions/frames. Contact cache is final RK4 internal stage, whereas trace qpos is post-integration; they are not asserted simultaneous.',
            'Accepted left means exact fixed diagnostic reproduction; native_task_result=false and strict_grasp_success_claim=false.',
            'This comparison does not identify a unique root cause or select a repair. No extra left benchmark or dynamics was run.',
        ],
    }
    out['execution_audit'] = {
        'mujoco_imported': any(name == 'mujoco' or name.startswith('mujoco.') for name in sys.modules),
        'input_files_unchanged': all(sha(p) == provenance[path_label(p)] for p in input_paths),
        'geometry_only': 'XML and actual referenced mesh byte reads; no model compilation or simulation data creation.',
    }
    assert not out['execution_audit']['mujoco_imported']
    assert out['execution_audit']['input_files_unchanged']
    args.output.write_text(json.dumps(out, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    print(json.dumps({'output': path_label(args.output), 'sha256': sha(args.output),
                      'left_rows': len(lt), 'binding_checks_passed': all(checks),
                      'arm_local_difference_count': len(out['geometry']['arm_flange_mount_body_local_element_differences']),
                      'arm_meshes_same': out['geometry']['all_referenced_arm_flange_mount_mesh_bytes_and_mesh_parameters_equal'],
                      'nearest_left_L2_rad': [r['joint_space_L2_distance_rad'] for r in nearby]}))


if __name__ == '__main__':
    main()
