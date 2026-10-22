#!/usr/bin/env python3
"""Bounded delivery check, not a grasp or full-travel benchmark.

Fresh output directory required. No tuning, retries, hidden scenes or LIBERO.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import platform
import time

import numpy as np
from control_asset import AssetRuntime, ROOT


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')


def protocol():
    return {
        'scope': 'Bounded six-channel response; not full joint travel, grasp or task success',
        'reset': 'Delivered keyframe 0, native reset + forward once per asset',
        'schedule': 'Each hand channel separately: 100-step ramp up, 100 hold, 100 ramp down, 100 hold; no retries',
        'hand_excursion': 'Reference target + 10% of each actuator range; other channels at reference',
        'standalone_steps_each': 2400,
        'assembly_steps_each': 400,
        'assembly_schedule': 'Same 400-step ramp/hold/return; all six hand channels +10% range, J6 +0.01 rad',
        'dt_s': .002,
        'health_bounds_from_original': {'mimic_rad': .02, 'joint_limit_overshoot_rad': .01,
                                        'arm_tracking_rad': .1, 'contact_depth_m': .002,
                                        'warnings': 0, 'nonfinite': 0},
        'response_observation_rad_min': .0001,
        'response_note': 'Observable correct-direction response only; no steady 0.02rad hand tracking gate',
        'contact_cache': 'mj_step RK4 final internal-stage geometry/force cache, before any forward; qpos/qvel are post-step',
    }


def check_one(name, output, plan):
    r=AssetRuntime(name)
    m,d=r.model,r.data
    mujoco=r.mujoco
    spec=r.spec
    result={'asset': name, 'model_sha256': hashlib.sha256((ROOT/spec['model']).read_bytes()).hexdigest(),
            'state_dim': len(r.state()), 'action_dim': spec['action_dim'], 'nq': m.nq, 'nu': m.nu,
            'physical_steps': 0, 'max_mimic_rad': 0., 'max_limit_overshoot_rad': 0.,
            'max_arm_tracking_rad': 0., 'max_contact_depth_m': 0., 'max_hand_target_error_rad': 0.,
            'warnings': 0, 'stop': None, 'responses': [], 'passed': False}
    home=r.home_action()
    # Input validation must reject before any physical integration.
    for bad in [home[:-1], np.full(len(home), np.nan), np.r_[home[:-1], 1.01]]:
        try:r.set_action(bad)
        except ValueError:pass
        else:raise AssertionError('Invalid action accepted')
    assert d.time==0
    eqs=[i for i in range(m.neq) if m.eq_type[i]==int(mujoco.mjtEq.mjEQ_JOINT) and m.eq_active0[i]]
    health=plan['health_bounds_from_original']
    phase_count=1 if spec['arm'] else 6
    step=0
    started=time.monotonic()
    with gzip.open(output/(name+'.jsonl.gz'),'wt',encoding='utf-8') as trace:
        try:
            for channel in range(phase_count):
                before=d.qpos[r.hand_qadr].copy()
                peak=[]
                for k in range(400):
                    amount=min((k+1)/100,1.,max(0.,(300-k)/100))
                    action=home.copy()
                    if spec['arm']:
                        action[-6:]+=.2*amount
                        action[5]+=.01*amount
                    else:action[-6+channel]+=.2*amount
                    r.set_action(action)
                    r.step(1)
                    step+=1
                    q=d.qpos[r.qadr]
                    limits=m.jnt_range[r.joint_ids]
                    overshoot=float(max(0., np.max(limits[:,0]-q), np.max(q-limits[:,1])))
                    residual=[]
                    for eq in eqs:
                        q1=d.qpos[m.jnt_qposadr[m.eq_obj1id[eq]]]
                        q2=d.qpos[m.jnt_qposadr[m.eq_obj2id[eq]]]
                        residual.append(float(q1-np.polynomial.polynomial.polyval(q2,m.eq_data[eq,:5])))
                    mimic=max(map(abs,residual),default=0.)
                    arm=float(np.max(np.abs(q[:7]-action[:7]))) if spec['arm'] else 0.
                    hand=float(np.max(np.abs(d.qpos[r.hand_qadr]-r.hand_target)))
                    contacts=[]
                    for c in d.contact:
                        force=np.zeros(6)
                        mujoco.mj_contactForce(m,d,len(contacts),force)
                        contacts.append({'geoms': [m.geom(c.geom1).name,m.geom(c.geom2).name],
                                         'distance_m': float(c.dist), 'pos_m': c.pos.tolist(),
                                         'normal': c.frame[:3].tolist(), 'force_contact_frame': force.tolist(),
                                         'efc_address': int(c.efc_address)})
                    depth=max([max(0.,-c['distance_m']) for c in contacts],default=0.)
                    for key,value in [('max_mimic_rad',mimic),('max_limit_overshoot_rad',overshoot),
                                      ('max_arm_tracking_rad',arm),('max_contact_depth_m',depth),
                                      ('max_hand_target_error_rad',hand)]:
                        result[key]=max(result[key],value)
                    trace.write(json.dumps({'step':step,'time_s':d.time,'channel':channel,'action':action.tolist(),
                        'state':q.tolist(),'qvel':d.qvel[r.vadr].tolist(),'ctrl':d.ctrl.tolist(),
                        'mimic_residual_rad':residual,'contacts':contacts})+'\n')
                    result['physical_steps']=step
                    if 175<=k<200:peak.append(d.qpos[r.hand_qadr].copy())
                    reasons=[]
                    for label,value,key in [('MIMIC',mimic,'mimic_rad'),('JOINT_LIMIT',overshoot,'joint_limit_overshoot_rad'),
                                            ('ARM_TRACKING',arm,'arm_tracking_rad'),('CONTACT_DEPTH',depth,'contact_depth_m')]:
                        if value>health[key]:reasons.append(label)
                    if reasons:raise RuntimeError(','.join(reasons))
                response=np.mean(peak,axis=0)-before
                tested=range(6) if spec['arm'] else [channel]
                result['responses'].extend({'channel':spec['active_order'][j], 'displacement_rad':float(response[j]),
                    'observable_positive_response':bool(response[j]>plan['response_observation_rad_min'])} for j in tested)
            result['passed']=all(row['observable_positive_response'] for row in result['responses'])
        except Exception as exc:
            result['stop']={'step':step,'type':type(exc).__name__,'message':str(exc)}
        result['warnings']=sum(int(w.number) for w in d.warning)
    result['wall_time_s']=time.monotonic()-started
    dump(output/(name+'.json'),result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--assets',nargs='+',choices=['dh116_left','dh116_right','nero_dh116_left','nero_dh116_right'],
                        default=['dh116_left','dh116_right','nero_dh116_left','nero_dh116_right'])
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    plan=protocol()
    plan['selected_assets']=args.assets
    dump(args.output/'protocol-before-step.json',plan)
    import mujoco
    import importlib.metadata
    versions={'python':platform.python_version(),'mujoco':mujoco.__version__,
              'numpy':np.__version__,'robosuite':importlib.metadata.version('robosuite')}
    results=[]
    for name in args.assets:
        result=check_one(name,args.output,plan)
        results.append(result)
        print(name,'PASS' if result['passed'] else 'FAIL',result['physical_steps'],result['stop'],flush=True)
    summary={'versions':versions,'passed':all(r['passed'] for r in results),'assets':results,
             'new_grasp_episodes':0,'scope':plan['scope']}
    dump(args.output/'results.json',summary)
    raise SystemExit(0 if summary['passed'] else 1)


if __name__=='__main__':main()
