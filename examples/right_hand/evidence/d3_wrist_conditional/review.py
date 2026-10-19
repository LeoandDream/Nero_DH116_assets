#!/usr/bin/env python3
"""Offline review of the ONE saved trial and its first failed contact.

No model load, Reset, mj_step, forward or force reconstruction. Source triangles
are compared only at the actual saved RK4 contact-cache transforms.
"""
import argparse
import gzip
import hashlib
import itertools
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key] = '1'
import numpy as np
from scipy.spatial import ConvexHull

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0,str(ROOT/'examples/right_hand'))
import proxy76_validate as reader
surface = reader._module('saved_wrist_trial_surface', ROOT/'examples/right_hand/evidence/d3_geometry/triangle/surface_geometry.py')


def rows(path):
    with gzip.open(path,'rt') as f:
        return [json.loads(line) for line in f]


def main(output):
    if output.exists():
        raise ValueError('Output must be new')
    run = HERE/'run'
    states, contacts = rows(run/'state.jsonl.gz'), rows(run/'contacts.jsonl.gz')
    result = json.loads((run/'result.json').read_text())
    assert len(states)==len(contacts)==result['physics_steps']==4742
    old = rows(ROOT/'examples/right_hand/evidence/d3_bowl78_slow_001/state.jsonl.gz')
    keys = ('full_qpos','full_qvel','ctrl','actuator_force','action_13','target_qpos_18')
    first = None
    for a,b in zip(old,states):
        changed = [k for k in keys if not np.array_equal(a[k],b[k])]
        if changed:
            first = {'step':a['step'],'fields':changed,
                     'max_full_qpos_difference_m_or_rad':float(np.max(np.abs(np.array(a['full_qpos'])-b['full_qpos'])))}
            break
    failed = [c for c in contacts[-1]['contacts'] if not c['filter_reasons'] and c['distance_m']<-.002]
    assert len(failed)==1
    c = failed[0]
    assert c['geom']==['white_bowl_1_g22','gripper0_proxy76_finger22_link_00']
    xml = ET.parse(run/'model.xml').getroot()
    box = xml.find('.//geom[@name="white_bowl_1_g22"]')
    assert box.get('type')=='box'
    size=np.fromstring(box.get('size'),sep=' ')
    pos=np.fromstring(box.get('pos','0 0 0'),sep=' ')
    rotation=reader.BUILDER.quaternion_matrix(box.get('quat','1 0 0 0'))
    vertices=np.array(list(itertools.product((-1,1),repeat=3)))*size
    box_triangles=vertices[ConvexHull(vertices).simplices]@rotation.T+pos
    bowl_pose, finger_pose=c['cached_body_pose']
    rb,rf=(np.array(p['rotation_world']).reshape(3,3) for p in (bowl_pose,finger_pose))
    pb,pf=(np.array(p['position_world_m']) for p in (bowl_pose,finger_pose))
    # Express native bowl wall in finger22 body frame, at cache time.
    box_in_finger=box_triangles@(rf.T@rb).T+rf.T@(pb-pf)
    queries={}
    for key,path in [('manufacturer_right_STL',ROOT/'meshes/dh116_right/source/finger22_link.STL'),
                     ('delivered_proxy_raw_STL',ROOT/'meshes/dh116/gripper0_finger22_link-53c55c84.stl')]:
        triangles=surface.read_stl(path)
        query=surface.surface_query(triangles,box_in_finger)
        query['source_topology']=surface.mesh_topology(triangles)
        # Containment diagnostics are separate from positive surface gap.
        points=np.vstack([box_in_finger.reshape(-1,3).mean(0),np.unique(box_in_finger.reshape(-1,3),axis=0)])
        query['box_center_and_vertices_winding_in_source']=surface.winding_number(points,triangles)
        query.update(path=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                     coordinate_frame='gripper0_finger22_link at final RK4 internal-stage cache')
        queries[key]=query
    old_block=states[1314:1401]
    first_positive=None
    contact_counts={}
    for state,record in zip(states,contacts):
        for con in record['contacts']:
            if con['force_contact_frame'][0]>0 and 'white_bowl_1_main' in con['body'] and any(b.startswith('gripper0_') for b in con['body']):
                if first_positive is None:first_positive=state['step']
                key=' / '.join(sorted(con['body']))
                contact_counts[key]=contact_counts.get(key,0)+1
    # Existing independently checked common-ball LP on raw proxy convex solid.
    lp=reader._module('first_contact_raw_convex_lp',ROOT/'examples/right_hand/evidence/d3_proxy76/gap_check/check_convex_gap.py')
    lp.LINKS=('finger22_raw_proxy','native_bowl_box')
    raw_vertices,_=reader.BUILDER.read_mesh(ROOT/'meshes/dh116/gripper0_finger22_link-53c55c84.stl')
    sources={lp.LINKS[0]:{'vertices':raw_vertices,'source':ROOT/'meshes/dh116/gripper0_finger22_link-53c55c84.stl','geom_name':c['geom'][1]},
             lp.LINKS[1]:{'vertices':box_in_finger.reshape(-1,3),'source':run/'model.xml','geom_name':c['geom'][0]}}
    transforms={key:{'rotation':np.eye(3).tolist(),'translation_m':[0.,0.,0.]} for key in lp.LINKS}
    overlap=lp.check_case(sources,transforms)
    summary={'schema':'one-trial-offline-review-v1','actual_physics_steps':len(states),
        'source_files_unmodified':True,'first_old_new_state_difference':first,
        'identical_prefix_steps':first['step']-1 if first else len(old),
        'old_J6_1315_1401_stage_passed':all(max(r['arm_error_rad'])<=.1 and not r['failures'] for r in old_block),
        'J6_max_error_1315_1401_rad':max(r['arm_error_rad'][5] for r in old_block),
        'J6_state_step1401':old_block[-1]['j6'],
        'first_failure':result['first_failure'],'last_two_states':states[-2:],
        'first_failure_contacts':failed,'cached_pose_source_queries':queries,
        'raw_proxy_convex_common_ball_check':overlap,
        'first_positive_hand_bowl_contact_step':first_positive,
        'positive_hand_bowl_contact_point_counts':contact_counts,
        'opposed_contact_steps':sum(r['opposed_thumb_index_contact'] for r in states),
        'wrist_external_contact_points':sum(r['wrist_external_contact_count'] for r in states),
        'warnings':sum(bool(r['warnings']) for r in states),'all_states_finite':all(r['finite'] for r in states),
        'stopped_on_first_recorded_original_or_additional_failure':all(not r['failures'] for r in states[:-1]) and bool(states[-1]['failures']),
        'native_task_result':result['native_task_result'],'strict_grasp_success_claim':result['strict_grasp_success_claim'],
        'limited_grasp':'UNKNOWN_EARLY_STOP_BEFORE_LIFT_HOLD',
        'measurement_limit':'contact.dist is native convex-proxy signed distance, not source-STL penetration depth. Surface crossing witness refers only to the saved cache pose; it is not a force counterfactual.',
        'offline_physics_steps':0}
    output.write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k not in ['last_two_states','first_failure_contacts','cached_pose_source_queries']},indent=2))
    print('source cross',{k:bool(v['strict_transverse_witness']) for k,v in queries.items()})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    main(parser.parse_args().output)
