import os
#!/usr/bin/env python3
"""Summarize logged contacts, offline FK sensitivity and RK4 timing evidence."""
import importlib.util
import json
from pathlib import Path
import mujoco
import numpy as np

HERE = Path(os.environ['NERO_D3_OUTPUT']) / 'trace'
ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'mjcf/nero_dh116_right.xml').is_file())
SOURCE_URL = 'https://github.com/google-deepmind/mujoco/blob/3.2.3/src/engine/engine_forward.c'


def write(name, obj):
    (HERE / name).write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def unit_angle_deg(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return float(np.degrees(np.arccos(np.clip(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b)), -1, 1))))


def main():
    poses_doc = json.loads((HERE / 'checkpoint_poses.json').read_text())
    trace = json.loads((HERE / 'selected_trace.json').read_text())
    poses = poses_doc['poses']
    audit = []
    mimic = []
    for check in trace['checkpoints']:
        step = check['step']
        entry = {'step': step, 'phase': check['phase'], 'time_s': check['time_s'], 'pair_comparisons': []}
        for contact in check['selected_recorded_contacts']:
            pair_names = contact['body_names']
            item = {'body_names': pair_names, 'recorded_distance_m': contact['distance_m'],
                    'recorded_normal_force_N': contact['normal_force_N'],
                    'recorded_position_world_m': contact['position_world_m'],
                    'recorded_normal_world': contact['normal_world'], 'offline_poses': []}
            for pose in [p for p in poses if p['checkpoint_step'] == step]:
                values = next(d for d in pose['modes']['recorded']['pair_distances'] if set(d['body_names']) == set(pair_names))
                for cand in values['offline_collision_candidates']:
                    item['offline_poses'].append({
                        'alignment': pose['alignment'], 'source_state_step': pose['source_state_step'],
                        'geomDistance_m': values['distance_m'],
                        'geomDistance_queries': values['distance_queries'],
                        'geomDistance_minus_recorded_contact_m': values['distance_m']-contact['distance_m'],
                        'collision_distance_m': cand['distance_m'],
                        'collision_distance_minus_recorded_m': cand['distance_m']-contact['distance_m'],
                        'collision_position_error_m': float(np.linalg.norm(np.asarray(cand['position_world_m'])-contact['position_world_m'])),
                        'collision_normal_angle_error_deg': unit_angle_deg(cand['normal_world'], contact['normal_world']),
                    })
            entry['pair_comparisons'].append(item)
        audit.append(entry)
    for pose in poses:
        entry = {'pose_id': pose['id'], 'alignment': pose['alignment'], 'modes': {}}
        baseline = pose['modes']['recorded']
        for mode, values in pose['modes'].items():
            body_deltas = {}
            for name in ('gripper0_base_link', 'gripper0_finger12_link', 'gripper0_finger42_link'):
                b, v = baseline['body_transforms'][name], values['body_transforms'][name]
                body_deltas[name] = {
                    'world_position_change_m': float(np.linalg.norm(np.asarray(v['body_world_p_m'])-b['body_world_p_m'])),
                    'world_R_max_abs_change': float(np.max(np.abs(np.asarray(v['body_world_R'])-b['body_world_R']))),
                }
            entry['modes'][mode] = {
                'passive_angles': values['passive_angles'],
                'body_changes_from_recorded': body_deltas,
                'pairs': [{'body_names': d['body_names'], 'geomDistance_m': d['distance_m'],
                           'geomDistance_change_from_recorded_m': d['distance_m']-baseline['pair_distances'][i]['distance_m'],
                           'geomDistance_queries': d['distance_queries'],
                           'offline_collision_candidates': d['offline_collision_candidates'],
                           'closest_segment_world_m': d['closest_segment_world_m']}
                          for i,d in enumerate(values['pair_distances'])],
            }
        mimic.append(entry)
    write('alignment_audit.json', {
        'method': 'Compare logged dynamic contact against offline collision detection at prior-row pre and same-row post qpos. No dynamics or constraint solve.',
        'source': SOURCE_URL,
        'source_line_ranges': {
            'mj_step': [973,1005], 'RK4_stages': [751,829],
            'RK4_final_update': [831,846], 'mj_advance': [655,676],
        },
        'geomDistance_source': 'https://github.com/google-deepmind/mujoco/blob/3.2.3/src/engine/engine_support.c#L1284-L1321',
        'geomDistance_source_interpretation': 'mj_geomDistance passes distmax directly to the collision function and reconstructs fromto from returned contact position, frame and distance. At distmax=0 it is the zero-margin overlap query; returning 0 is a capped result for separated pairs.',
        'timing_conclusion': 'The formal model uses RK4. After mj_step, qpos is the final weighted update, but position-dependent contact caches/forces remain from the last internally solved RK4 stage X[3] (time t+h). Thus neither prior-row pre qpos nor same-row post qpos is an exact solver-pose identity. No X[3] qpos was logged. post is numerically much closer for these four checkpoints.',
        'geomDistance_caveat': 'mj_geomDistance with positive distmax and mj_collision can differ even at identical qpos in MuJoCo 3.2.3; distmax=0 matches the zero-margin collision query for these pairs. Multiple cutoff queries are stored; do not interpret cutoff sensitivity as timestamp displacement or physical depth change.',
        'checks': audit,
    })
    write('mimic_comparison.json', {'method': poses_doc['method'], 'poses': mimic})

    spec = importlib.util.spec_from_file_location('right_hand_run', ROOT / 'examples/right_hand/run.py')
    run = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(run)
    model = mujoco.MjModel.from_xml_path(str(ROOT / 'mjcf/nero_dh116_right.xml'))
    binding = run.bind_robot(model, run.URDF)
    geom_ids = [next(g['geom_id'] for g in poses_doc['geometry'].values() if g['body_name']==name)
                for name in ('gripper0_base_link','gripper0_finger12_link','gripper0_finger42_link')]
    def fk(qpos):
        d = mujoco.MjData(model)
        d.qpos[:] = qpos
        mujoco.mj_kinematics(model, d)
        mujoco.mj_comPos(model,d)
        distances=[]
        for a,b in ((geom_ids[0],geom_ids[1]),(geom_ids[1],geom_ids[2])):
            distances.append(float(mujoco.mj_geomDistance(model,d,a,b,1,np.zeros(6))))
        return d, distances
    def ancestors(name):
        bid = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,name)
        chain=[]
        while bid > 0:
            jids = np.flatnonzero(model.jnt_bodyid == bid)
            chain.append({'body':mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(bid)),
                          'joints':[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,int(j)) for j in jids]})
            bid = model.body_parentid[bid]
        return chain
    sensitivity=[]
    for pose in poses:
        q0 = np.asarray(pose['full_qpos_recorded'])
        base, d0 = fk(q0)
        test=[]
        for relation in binding['mimics']:
            q = q0.copy()
            q[binding['qpos'][relation['dependent']]] += .1
            data, distances = fk(q)
            test.append({'passive_joint':relation['name'],'perturbation_rad':.1,
                         'geomDistance_changes_m':[a-b for a,b in zip(distances,d0)],
                         'base_finger12_transforms_max_abs_change':float(max(np.max(np.abs(data.geom_xpos[geom_ids[:2]]-base.geom_xpos[geom_ids[:2]])),np.max(np.abs(data.geom_xmat[geom_ids[:2]]-base.geom_xmat[geom_ids[:2]])))),
                         'finger42_geom_R_max_abs_change':float(np.max(np.abs(data.geom_xmat[geom_ids[2]]-base.geom_xmat[geom_ids[2]])))})
        sensitivity.append({'pose_id':pose['id'],'tests':test})
    write('passive_independence.json',{
        'method':'Kinematic ancestor inspection plus separate +0.1rad perturbation of each passive joint at all 8 logged poses, using only mj_kinematics+comPos+geomDistance. No model changes.',
        'body_ancestor_chains':{name:ancestors(name) for name in ('gripper0_base_link','gripper0_finger12_link','gripper0_finger42_link')},
        'conclusion':'base_link and finger12_link have none of the five passive joints in their ancestor chains; their pair is exactly invariant to all five passive definitions at fixed active/arm qpos. finger42_link includes finger41 (active driver) and finger42 (passive follower); only finger42 among the five passive joints changes this pair. Arm transforms move both bodies rigidly and do not change their intrinsic distance.',
        'tests':sensitivity,
        'all_base_finger12_distance_changes_exact_zero':all(t['geomDistance_changes_m'][0]==0 for p in sensitivity for t in p['tests']),
        'only_finger42_changes_finger12_finger42_distance':all((t['geomDistance_changes_m'][1]==0)==(t['passive_joint']!='finger42') for p in sensitivity for t in p['tests']),
    })
    print(json.dumps({'alignment_checks':len(audit),'mimic_poses':len(mimic),'passive_perturbation_tests':sum(len(p['tests']) for p in sensitivity),'physics_steps':0,'constraint_solves':0},indent=2))

if __name__=='__main__':
    main()
