import os
#!/usr/bin/env python3
"""Offline FK and collision distances only. Never step or solve constraints."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'mjcf/nero_dh116_right.xml').is_file())
OUT = Path(os.environ['NERO_D3_OUTPUT']) / 'trace'
STEPS = (229, 451, 6498, 6511)
PAIRS = (('gripper0_base_link', 'gripper0_finger12_link'),
         ('gripper0_finger12_link', 'gripper0_finger42_link'))
MODES = ('recorded', 'urdf_linear', 'upstream_quadratic')


def write(path, obj):
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def read_rows(path, needed):
    found = {}
    with gzip.open(path, 'rt') as stream:
        for line in stream:
            row = json.loads(line)
            if row['step'] in needed:
                found[row['step']] = row
    return found


def main():
    spec = importlib.util.spec_from_file_location('right_hand_run', ROOT / 'examples/right_hand/run.py')
    run = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(run)
    model = mujoco.MjModel.from_xml_path(str(run.MODEL))
    binding = run.bind_robot(model, run.URDF)
    evidence = ROOT / 'examples/right_hand/evidence'
    states = read_rows(evidence / 'hand-motion-state.jsonl.gz', set(STEPS) | {s-1 for s in STEPS})
    contacts = read_rows(evidence / 'hand-motion-contacts.jsonl.gz', set(STEPS))
    with gzip.open(evidence / 'frozen-commands.json.gz', 'rt') as stream:
        commands = json.load(stream)['hand_motion']
    tree = ET.parse(run.MODEL).getroot()
    mesh_files = {m.get('name'): str((run.MODEL.parent / m.get('file')).resolve())
                  for m in tree.findall('./asset/mesh')}
    geom_meta = {}
    for gid in range(model.ngeom):
        bid = int(model.geom_bodyid[gid])
        body = run.object_name(model, mujoco.mjtObj.mjOBJ_BODY, bid)
        if not body.startswith('gripper0_') or model.geom_group[gid] != 0:
            continue
        mid = int(model.geom_dataid[gid])
        mesh_name = run.object_name(model, mujoco.mjtObj.mjOBJ_MESH, mid)
        geom_meta[run.object_name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)] = {
            'geom_id': gid, 'body_id': bid, 'body_name': body,
            'mesh_name': mesh_name, 'source_mesh_path': mesh_files[mesh_name],
            'geom_local_pos_m': model.geom_pos[gid].tolist(),
            'geom_local_quat_wxyz': model.geom_quat[gid].tolist(),
            'compiled_mesh_pos_m': model.mesh_pos[mid].tolist(),
            'compiled_mesh_quat_wxyz': model.mesh_quat[mid].tolist(),
            'note': 'Source STL vertices use body transform (original XML geom origin is identity); compiled geom_xmat/xpos include MuJoCo mesh centering.'}
    pairs = []
    for names in PAIRS:
        ids = [next(v['geom_id'] for v in geom_meta.values() if v['body_name'] == name) for name in names]
        pairs.append((names, ids))
    meta = {
        'schema': 'd3-offline-fk-v1',
        'formal_head': 'ce920568e159ac38af4fda412ac84f2262e27035',
        'mujoco_version': mujoco.__version__,
        'model_path': str(run.MODEL),
        'model_sha256': hashlib.sha256(run.MODEL.read_bytes()).hexdigest(),
        'integrator': int(model.opt.integrator),
        'integrator_name': str(mujoco.mjtIntegrator(int(model.opt.integrator))),
        'timestep_s': float(model.opt.timestep),
        'joint_names': binding['names'],
        'active_joint_names': [binding['names'][i] for i in binding['active_indexes']],
        'actuator_names_in_force_ctrl_order': [run.object_name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)],
        'actuator_joint_names_in_force_ctrl_order': [run.object_name(model, mujoco.mjtObj.mjOBJ_JOINT, model.actuator_trnid[i, 0]) for i in range(model.nu)],
        'mimics': binding['mimics'],
        'geometry': geom_meta,
        'matrix_convention': 'row-major 3x3 R; source_vertex_world = R @ source_vertex + p',
        'method': 'Fresh MjData; set full_qpos; mj_kinematics + mj_comPos + mj_collision + mj_geomDistance only. No mj_step, mj_forward, constraint solve, model/equality change.',
        'contact_method': 'mj_collision creates offline collision candidates only. No forces are computed; recorded dynamic forces are kept separately.',
        'geomDistance_queries': 'distance_m is the legacy distmax=1m query. distance_queries gives distmax=0,.01,.1,1m; distmax=0 gives contact-compatible negative distances and returns a capped zero for separated pairs. MuJoCo3.2.3 convex-query results vary with distmax; these are algorithmic proxy values, not triangle penetration depth.',
        'limitations': [
            'MuJoCo mesh geomDistance/collision uses convex collision hulls, not original triangle surfaces.',
            'Counterfactual mimic poses hold all six active joints and arm fixed, set only five passive joints exactly, and are kinematic comparisons; they do not establish dynamic behavior.',
            'pre is the prior logged full_qpos; post is the selected row full_qpos. Stored contact may reflect internal RK4 stage; exact temporal correspondence is audited separately.',
            'Logged actuator_force and dynamic contact force also belong to the final internal RK4 solve, whereas active_tracking_error is computed from the final post qpos.',
        ],
    }
    poses = []
    checks = []
    before = run.physics_fingerprint(model)
    for step in STEPS:
        row = states[step]
        selected_contacts = []
        for c in contacts[step]['contacts']:
            if any(set(c['body_names']) == set(names) for names in PAIRS):
                selected_contacts.append({**c, 'normal_world': c['frame_world'][:3]})
        checks.append({
            'step': step, 'time_s': row['time_s'], 'phase': row['phase'],
            'steady_window': row['steady_window'], 'command': commands[step-1],
            'robot_state_18': row['robot_state_18'], 'full_qpos': row['full_qpos'],
            'full_qvel': row['full_qvel'],
            'hand_target_rad': row['hand_target_rad'],
            'active_tracking_error_rad': row['active_tracking_error_rad'],
            'arm_tracking_error_rad': row['arm_tracking_error_rad'],
            'mimic_residual_rad': row['mimic_residual_rad'],
            'actuator_force': row['actuator_force'],
            'internal_actuator_ctrl_13': row['internal_actuator_ctrl_13'],
            'selected_recorded_contacts': selected_contacts,
            'recorded_contact_count_all_pairs': len(contacts[step]['contacts']),
        })
        for alignment, source_step in (('pre', step-1), ('post', step)):
            source_row = states[source_step]
            qpos0 = np.asarray(source_row['full_qpos'])
            pose = {
                'id': f'step_{step}_{alignment}', 'checkpoint_step': step,
                'alignment': alignment, 'source_state_step': source_step,
                'time_s': source_row['time_s'], 'phase': source_row['phase'],
                'full_qpos_recorded': qpos0.tolist(), 'modes': {},
            }
            for mode in MODES:
                data = mujoco.MjData(model)
                data.qpos[:] = qpos0
                passive = []
                for relation in binding['mimics']:
                    dep = binding['qpos'][relation['dependent']]
                    driver = binding['qpos'][relation['driver']]
                    observed = float(qpos0[dep])
                    driver_q = float(qpos0[driver])
                    target = relation['offset'] + relation['multiplier'] * driver_q
                    if mode == 'upstream_quadratic':
                        target += (.469825 if relation['name'] == 'finger13' else .200535) * driver_q**2
                    if mode == 'recorded':
                        target = observed
                    data.qpos[dep] = target
                    passive.append({
                        'joint': relation['name'], 'driver_joint': binding['names'][relation['driver']],
                        'driver_q_rad': driver_q, 'recorded_q_rad': observed, 'posed_q_rad': float(target),
                        'delta_from_recorded_rad': float(target-observed),
                    })
                posed = data.qpos.copy()
                mujoco.mj_kinematics(model, data)
                mujoco.mj_comPos(model, data)
                mujoco.mj_collision(model, data)
                assert np.array_equal(data.qpos, posed)
                bodies = {}
                geoms = {}
                for name, item in geom_meta.items():
                    bid, gid = item['body_id'], item['geom_id']
                    bodies[item['body_name']] = {
                        'body_id': bid, 'body_world_R': data.xmat[bid].reshape(3,3).tolist(),
                        'body_world_p_m': data.xpos[bid].tolist(),
                    }
                    geoms[name] = {
                        'geom_world_R': data.geom_xmat[gid].reshape(3,3).tolist(),
                        'geom_world_p_m': data.geom_xpos[gid].tolist(),
                    }
                distances = []
                for names, gids in pairs:
                    segment = np.zeros(6)
                    distance = float(mujoco.mj_geomDistance(model, data, *gids, 1.0, segment))
                    delta = segment[3:] - segment[:3]
                    norm = float(np.linalg.norm(delta))
                    queries = []
                    for cutoff in (0., .01, .1, 1.):
                        query_segment = np.zeros(6)
                        query_distance = float(mujoco.mj_geomDistance(model, data, *gids, cutoff, query_segment))
                        queries.append({
                            'cutoff_m': cutoff, 'distance_m': query_distance,
                            'distance_is_capped': bool(query_distance == cutoff),
                            'closest_segment_world_m': query_segment.tolist(),
                        })
                    candidates = []
                    for contact in data.contact:
                        if {int(contact.geom1), int(contact.geom2)} == set(gids):
                            candidates.append({
                                'geom_ids': [int(contact.geom1), int(contact.geom2)],
                                'distance_m': float(contact.dist),
                                'position_world_m': contact.pos.tolist(),
                                'normal_world': contact.frame[:3].tolist(),
                                'frame_world': contact.frame.tolist(),
                                'force_N': None,
                            })
                    distances.append({
                        'body_names': list(names), 'geom_ids': gids,
                        'geom_names': [run.object_name(model, mujoco.mjtObj.mjOBJ_GEOM, x) for x in gids],
                        'distance_m': distance, 'closest_segment_world_m': segment.tolist(),
                        'distance_queries': queries,
                        'closest_segment_midpoint_world_m': ((segment[:3]+segment[3:])/2).tolist(),
                        'segment_unit_direction_world': (delta/norm).tolist() if norm else None,
                        'offline_collision_candidates': candidates,
                    })
                pose['modes'][mode] = {
                    'full_qpos': data.qpos.tolist(), 'passive_angles': passive,
                    'body_transforms': bodies, 'geom_transforms': geoms,
                    'pair_distances': distances,
                    'constraint_solves': 0, 'physics_steps': 0,
                }
            poses.append(pose)
    assert run.physics_fingerprint(model) == before
    write(OUT / 'checkpoint_poses.json', {**meta, 'poses': poses})
    write(OUT / 'selected_trace.json', {**meta, 'checkpoints': checks})
    print(json.dumps({'poses': str(OUT / 'checkpoint_poses.json'), 'selected_trace': str(OUT / 'selected_trace.json'),
                      'pose_count': len(poses), 'mode_count': len(MODES), 'physics_fields_unchanged': True,
                      'physics_steps': 0, 'constraint_solves': 0}, indent=2))

if __name__ == '__main__':
    main()
