#!/usr/bin/env python3
"""Review saved integration only; one compiled mesh and one cached contact query.

No Reset, forward, collision call, control update or physics integration.
Never changes the executed risk classifier or reinterprets its recorded stop.
"""
import argparse
from collections import Counter
import gzip
import hashlib
import itertools
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[k] = '1'
import numpy as np
from scipy.spatial import ConvexHull

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0,str(ROOT/'examples/right_hand'))
import finger22_static as F
import finger22_conditional as C


def rows(path):
    with gzip.open(path,'rt') as f:
        for line in f:
            yield json.loads(line)


def read(path):
    return C.read(path)


def summarize(run):
    old = ROOT/'examples/right_hand/evidence/d3_wrist_conditional/run'
    states = list(rows(run/'state.jsonl.gz'))
    result = read(run/'result.json')
    assert len(states) == result['physics_steps']
    assert [s['step'] for s in states] == list(range(1,len(states)+1))
    assert all(not s['failures'] for s in states[:-1]) and states[-1]['failures']
    keys = ('full_qpos','full_qvel','ctrl','actuator_force','action_13','target_qpos_18')
    first = {}; old_at_4731 = None
    for a,b in zip(rows(old/'state.jsonl.gz'),states):
        for k in keys:
            if k not in first and not np.array_equal(a[k],b[k]):
                first[k] = {'step': b['step'], 'maximum_absolute_difference':float(np.max(abs(np.array(a[k])-b[k])))}
        old_at_4731 = a
    schedule = read(run/'schedule.json')
    assert schedule == read(old/'schedule.json')
    frozen = read(run/'frozen-before-first-step.json')
    prior = read(old/'frozen-before-first-step.json')
    assert all(frozen[k]==prior[k] for k in ('qpos','qvel','ctrl','controller','criteria'))
    assert frozen['additional_stop_protocol'] == read(HERE/'registered-protocol.json')
    assert C.digest(ROOT/C.MODEL) == C.MODEL_SHA == frozen['robot_model_sha256']
    assert result['inputs_unchanged'] and result['dynamic_attempts']==1
    contact_fields = ('geom','body','distance_m','position_world_m','normal_world','force_contact_frame')
    def core(record):
        return sorted([{k:c[k] for k in contact_fields} for c in record['contacts']],key=lambda c:(c['geom'],c['position_world_m']))
    counts=Counter();first_contact_difference=None;first_hand_bowl=None;first_finger22=None
    max_force=0.; max_force_pair=None; last=None; prior_cache=None; selected={}
    for a,b in zip(rows(old/'contacts.jsonl.gz'),rows(run/'contacts.jsonl.gz')):
        if first_contact_difference is None and core(a)!=core(b):
            first_contact_difference=b['step']
        if b['step'] in (4518,4662,4663,4730,4731):
            selected[str(b['step'])] = b
        for c in b['contacts']:
            label=(c.get('finger22_region') or {}).get('status')
            if label:
                counts[label]+=1
                if first_finger22 is None:first_finger22=b['step']
            if not c['filter_reasons'] and c['force_contact_frame'][0]>max_force and any(x.startswith(('robot0_','gripper0_','mount0_')) for x in c['body']):
                max_force=c['force_contact_frame'][0];max_force_pair=c['geom']
            if ('white_bowl_1_main' in c['body'] and any(x.startswith('gripper0_')for x in c['body'])
                    and c['force_contact_frame'][0]>0 and first_hand_bowl is None):first_hand_bowl=b['step']
        if last:prior_cache=last
        last=b
    assert last['step']==len(states)
    failed=[c for c in last['contacts'] if (c.get('finger22_region')or{}).get('status','').startswith(('UNKNOWN','STOP_'))]
    assert len(failed)==1
    c=failed[0]; assert c['geom']==['white_bowl_1_g22',C.NEW[1]]
    # Actual final RK4 body frame. No post-step FK is substituted here.
    bowl,finger=c['cached_body_pose']
    rb,rf=[np.array(p['rotation_world']).reshape(3,3)for p in (bowl,finger)]
    pb,pf=[np.array(p['position_world_m'])for p in (bowl,finger)]
    xml=ET.parse(run/'model.xml').getroot();box=xml.find('.//geom[@name="white_bowl_1_g22"]')
    size=np.fromstring(box.get('size'),sep=' ');pos=np.fromstring(box.get('pos','0 0 0'),sep=' ')
    rot=F.P.reader.BUILDER.quaternion_matrix(box.get('quat','1 0 0 0'))
    vertices=np.array(list(itertools.product((-1,1),repeat=3)))*size
    triangles=vertices[ConvexHull(vertices).simplices]@rot.T+pos
    box_in_finger=triangles@(rf.T@rb).T+rf.T@(pb-pf)
    source=F.P.geometry.read_stl(F.SOURCE)
    source_query=F.P.geometry.surface_query(source,box_in_finger)
    model=F.mj.MjModel.from_xml_path(str(ROOT/C.MODEL))
    binding=F.bindings(ROOT/C.MODEL,[C.NEW[1]])[0]
    h=F.P.compiled_hull(model,C.NEW[1],binding)
    point=np.array(c['finger22_region']['midpoint_body_m'])
    d,nearest,face=F.G.TriangleSurface(h['triangles']).nearest(point[None])
    t=h['triangles'][face[0]];normal=np.cross(t[1]-t[0],t[2]-t[0]);normal/=np.linalg.norm(normal)
    if normal@(t.mean(0)-h['vertices'].mean(0))<0:normal=-normal
    normal_from_contact=-rf.T@np.array(c['normal_world'])
    ds,near_s,source_face=F.G.TriangleSurface(source).nearest(point[None])
    elapsed = [s for s in states if 1315<=s['step']<=1401]
    original_reasons={'NONFINITE_STATE','NUMERICAL_WARNING_OR_TIME_RESET','ARM_TRACKING','ACTIVE_HOLD_TRACKING','MIMIC','JOINT_LIMIT','ROBOT_CONTACT_OVERLAP'}
    return {'schema':'finger22-four-one-integrated-trial-review-v1','physics_steps':len(states),
        'process_exit_code':read(HERE/'execution.json')['actual_process_exit_code'],
        'dynamic_attempts':result['dynamic_attempts'],'stopped_at_first_recorded_failure':True,
        'first_failure':result['first_failure'],'step4742':'UNKNOWN_NOT_REACHED',
        'schedule_270_exact':True,'native_reset_and_controller_exact':True,
        'all_frozen_inputs_unchanged':result['inputs_unchanged'],
        'first_difference_by_field':first,'identical_full_state_control_prefix':min(x['step']for x in first.values())-1,
        'first_contact_difference':first_contact_difference,
        'old_state_at_current_stop':old_at_4731,
        'original_dynamic_bounds_until_stop':'PASS' if not any(original_reasons&set(s['failures'])for s in states)else'FAIL',
        'runner_control_health':'FAIL (combined stop status includes risk UNKNOWN; not proof of tracking failure)',
        'old_J6_segment_passed':all(not s['failures'] for s in elapsed),
        'J6_max_error_1315_1401_rad':max(s['arm_error_rad'][5]for s in elapsed),
        'phase_steps':dict(Counter(s['phase']for s in states)),
        'first_positive_hand_bowl_step':first_hand_bowl,'first_finger22_contact_step':first_finger22,
        'finger22_region_contact_counts':dict(counts),'wrist_external_contact_points':sum(s['wrist_external_contact_count']for s in states),
        'opposed_contact_steps':sum(s['opposed_thumb_index_contact']for s in states),
        'no_support_steps':sum(not s['environment_support']for s in states),
        'maximum_robot_normal_force_N':max_force,'maximum_force_geom_pair':max_force_pair,
        'maxima':result['maxima'],'warnings':sum(bool(s['warnings'])for s in states),
        'all_states_finite':all(s['finite']for s in states),
        'reset_bowl_z_m':result['reset_bowl_z_m'],'settled_bowl_z_m':result['own_settled_bowl_z_m'],
        'final_bowl_z_m':result['final_bowl_z_m'],'lift_from_reset_m':result['lift_from_reset_m'],
        'lift_from_settled_m':result['lift_from_own_settled_m'],'valid_hold_s':result['longest_continuous_valid_hold_s'],
        'limited_grasp':'UNKNOWN_EARLY_STOP','native_task_result':result['native_task_result'],
        'strict_grasp_success_claim':result['strict_grasp_success_claim'],
        'failed_contact':c,'cached_source_complete_box_query':source_query,
        'contact_classification_review':{
            'contact_midpoint_to_actual_mesh_graph_m':float(d[0]),'actual_mesh_graph_nearest_body_m':nearest[0].tolist(),
            'nearest_graph_outward_normal':normal.tolist(),'actual_contact_outward_normal_body':normal_from_contact.tolist(),
            'normal_angle_deg':float(np.degrees(np.arctan2(np.linalg.norm(np.cross(normal,normal_from_contact)),normal@normal_from_contact))),
            'midpoint_nearest_source_m':float(ds[0]),'nearest_source_face':int(source_face[0]),
            'classifier_radius_m':c['finger22_region']['identification_uncertainty_m'],
            'radius_shortfall_m':float(d[0])-c['finger22_region']['identification_uncertainty_m'],
            'unchanged_ccd_tolerance_m':float(model.opt.ccd_tolerance),
            'interpretation':'Saved contact is near the +X functional front, not a confirmed cavity/root contact. MPR midpoint to true convex boundary exceeds registered half-distance-plus-compile reserve. This explains empty candidate list; no certified MPR witness-position error bound was included. Do not interpret this UNKNOWN stop as geometric risk failure or relax/replay it after the run.',
            'source_material_containment':'UNKNOWN_NONMANIFOLD_SOURCE; separated surfaces are not a complete material certificate'},
        'last_state':states[-1],'previous_state':states[-2],
        'RK4_cache_poses_saved_for_all_robot_bodies':all(bool(c.get('body_poses'))for c in selected.values()),
        'temporal_limit':'qpos/qvel are post integration; body/contact transforms, forces, qacc and generalized forces are last internal RK4 stage. No recording forward.',
        'formal_numeric_status':'FAIL_UNCHANGED_1e-10m','release':'NOT_APPROVED',
        'offline_model_compiles':1,'offline_cached_pair_surface_queries':1,'offline_physics_steps':0}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,default=HERE/'run')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('Use a new review output; do not overwrite evidence')
    summary=summarize(args.run)
    args.output.write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:summary[k]for k in ('physics_steps','first_failure','identical_full_state_control_prefix','first_difference_by_field','phase_steps','original_dynamic_bounds_until_stop','cached_source_complete_box_query','contact_classification_review')},indent=2))
