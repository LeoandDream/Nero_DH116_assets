"""Fixed white bowl commands through LIBERO's existing joint controllers.

The caller provides a native environment constructed from the official model,
its real physics step, and explicit input directories. This module neither
places the object nor changes actuator, equality, collision, or solver settings.
"""
from pathlib import Path
import json
import numpy as np
import mujoco

def run(env,out,real_step,asset_root,fixture_dir,before_step=None):
    """Record the fixed diagnostic, restoring the process's mj_step on every exit.

    ``asset_root`` supplies the official URDF's hand limit/mimic metadata only;
    ``fixture_dir`` supplies frozen command inputs. ``before_step(model, data)``
    observes the final reset state. Physics and contact instrumentation belong to
    the caller's ``real_step(model, data)`` callback.
    """
    import xml.etree.ElementTree as ET
    out=Path(out);fixture_dir=Path(fixture_dir)
    report={'status':'RUNNING','scope':'Fixed diagnostic commands in actual NERO+DH116 LIBERO mount; no learned policy or demonstration labels',
      'raw_state_dim':18,'raw_action_dim':13,'nero_action':'7D ABSOLUTE JOINT POSITION [rad]',
      'dh116_action':'6D normalized absolute actuator position','cases':[],
      'source_mass_inertia_joint_dynamics_unchanged':True,'frozen_official_parameters_preserved':True,'reset_parameter_replay':False,'new_low_level_controller':False,'hand_force_velocity_limits':None,'new_collision_filters':False}
    active=['finger11','finger12','finger21','finger31','finger41','finger51']
    old_step=mujoco.mj_step
    class StopDiagnostic(RuntimeError):pass
    try:
        urdf=ET.parse(Path(asset_root)/'urdf/dh116_left.urdf').getroot()
        for label,integrator in [('bounded_grasp',mujoco.mjtIntegrator.mjINT_RK4)]:
            mujoco.mj_step=old_step;env.seed(20261007);obs=env.reset();robot=env.robots[0]
            m=env.sim.model._model;d=env.sim.data._data
            assert int(m.opt.integrator)==int(integrator), 'Official diagnostic model must use RK4'
            # The snapshot comes from the official model already assembled by the caller.
            if hasattr(env, 'env') and hasattr(env.env, 'model'):
                xml=ET.fromstring(env.env.model.get_xml())
                ET.ElementTree(xml).write(out/(label+'-model.xml'),encoding='utf-8',xml_declaration=True)
            arm=robot.robot_model.joints;hand=robot.gripper.joints
            model_joints=arm+hand;indices=[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in model_joints]
            limits=np.array([m.jnt_range[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in model_joints]);span=limits[:,1]-limits[:,0]
            base=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'gripper0_base_link');assert m.body_weldid[base]!=0
            setup={}
            initial=[]
            for c in d.contact:
                names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,int(m.geom_bodyid[g])) for g in [c.geom1,c.geom2]]
                if any(str(n).startswith('gripper0_') for n in names):initial.append(names)
            assert not initial,initial
            case={'case':label,'integrator':int(integrator),'dt':float(m.opt.timestep),'status':'RUNNING',
                  'hand_base_weld_id':int(m.body_weldid[base]),'initial_hand_contacts':initial,'setup':setup,
                  'physics_steps':0,'scripted_command_steps':0,'stop_reason':None,'trajectory':label+'.jsonl'}
            max_mimic=max_arm_error=max_force=max_speed=0.;contact_steps=0;min_dist=None;max_normal=0.;pairs=set();current=np.zeros(13)
            config=json.loads((fixture_dir/'initial-joint-config.json').read_text());start_arm=np.asarray(config['arm_absolute_qpos_rad']);open_fractions=np.asarray(config['hand_active_source_range_fractions']);close_fractions=open_fractions.copy();first_actual_contact_macro=None;planned=False;path=None
            bowl_id=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'white_bowl_1_main');initial_bowl_z=float(d.xpos[bowl_id,2]);peak_bowl_z=initial_bowl_z
            thumb_contact_steps=opposed_contact_steps=0;max_other_overlap=0.;phase_name='settle'
            current[:7]=start_arm;current[7:]=2*open_fractions-1
            with (out/(label+'.jsonl')).open('w') as trace:
                def checked_step(model,data):
                    nonlocal max_mimic,max_arm_error,max_force,max_speed,contact_steps,min_dist,max_normal,peak_bowl_z,thumb_contact_steps,opposed_contact_steps,max_other_overlap
                    before=float(data.time);real_step(model,data);case['physics_steps']+=1
                    q=data.qpos[indices].copy();values={n.removeprefix('gripper0_'):float(q[7+i]) for i,n in enumerate(hand)}
                    mimic=max(abs(values[j.get('name')]-float(mi.get('multiplier','1'))*values[mi.get('joint')]-float(mi.get('offset','0')))
                         for j in urdf.findall('joint') if (mi:=j.find('mimic')) is not None)
                    max_mimic=max(max_mimic,mimic);max_arm_error=max(max_arm_error,float(np.max(np.abs(q[:7]-current[:7]))))
                    max_force=max(max_force,float(np.max(np.abs(data.actuator_force[7:]))));max_speed=max(max_speed,float(np.max(np.abs(data.qvel))))
                    distances=[];normal=0.;touch_bodies=set();other_overlap=0.
                    for i,c in enumerate(data.contact):
                        b=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(model.geom_bodyid[g])) for g in [c.geom1,c.geom2]]
                        if 'white_bowl_1_main' in b and any(str(n).startswith('gripper0_') for n in b):
                            force=np.zeros(6);mujoco.mj_contactForce(model,data,i,force);normal=max(normal,float(abs(force[0])));distances.append(float(c.dist));pairs.add(tuple(b));touch_bodies.update(b)
                        elif any(str(n).startswith(('robot0_','gripper0_')) for n in b):other_overlap=max(other_overlap,-float(c.dist))
                    if distances:contact_steps+=1;min_dist=min(min_dist if min_dist is not None else 0.,min(distances));max_normal=max(max_normal,normal)
                    thumb=any(str(n).startswith('gripper0_finger1') for n in touch_bodies);others=any(str(n).startswith(('gripper0_finger2','gripper0_finger3','gripper0_finger4','gripper0_finger5')) for n in touch_bodies)
                    thumb_contact_steps+=int(thumb);opposed_contact_steps+=int(thumb and others);peak_bowl_z=max(peak_bowl_z,float(data.xpos[bowl_id,2]));max_other_overlap=max(max_other_overlap,other_overlap)
                    warnings={str(mujoco.mjtWarning(i)):int(w.number) for i,w in enumerate(data.warning) if w.number}
                    trace.write(json.dumps({'time':float(data.time),'phase':phase_name,'bowl_z_m':float(data.xpos[bowl_id,2]),'thumb_contact':thumb,'opposed_contact':thumb and others,'other_robot_overlap_m':other_overlap,'robot_state_18':q.tolist(),'robot_action_13':current.tolist(),
                        'internal_actuator_ctrl_13':data.ctrl.tolist(),'linear_mimic_error_rad':mimic,
                        'max_hand_target_error_rad':float(np.max(np.abs(np.array([values[n] for n in active])-((current[7:]+1)*.5*np.array([float(urdf.find(f'joint[@name="{n}"]/limit').get('upper')) for n in active]))))), 'bowl_hand_contact_distances_m':distances,'max_contact_normal_N':normal,'warnings':warnings,
                        'full_qpos':data.qpos.tolist(),'full_qvel':data.qvel.tolist()})+'\n')
                    if not all(np.isfinite(x).all() for x in [data.qpos,data.qvel,data.qacc,data.actuator_force]):case['stop_reason']='NONFINITE_STATE'
                    elif warnings or float(data.time)<=before:case['stop_reason']='NUMERICAL_WARNING_OR_RESET'
                    elif mimic>.05:case['stop_reason']='SOURCE_MIMIC_RESIDUAL_EXCEEDS_TEMPORARY_DIAGNOSTIC_0_05_RAD_BOUND'
                    elif distances and min(distances)<-.002:case['stop_reason']='TARGET_CONTACT_OVERLAP_EXCEEDS_TEMPORARY_DIAGNOSTIC_2_MM_BOUND'
                    elif other_overlap>.002:case['stop_reason']='OTHER_ROBOT_CONTACT_OVERLAP_EXCEEDS_TEMPORARY_DIAGNOSTIC_2_MM_BOUND'
                    elif np.any(q<limits[:,0]-span) or np.any(q>limits[:,1]+span):case['stop_reason']='JOINT_STATE_DEVIATED_BY_MORE_THAN_SOURCE_RANGE'
                    if case['stop_reason']:raise StopDiagnostic(case['stop_reason'])
                mujoco.mj_step=checked_step
                try:
                    if before_step is not None:before_step(m,d)
                    for macro in range(230):
                        if macro==10:
                            proposal=json.loads((fixture_dir/'prepared-white-geometry-plan.json').read_text())
                            (out/'white-geometry-plan.json').write_text(json.dumps(proposal,indent=2)+'\n');path=proposal['nodes_contact_to_above'];open_fractions=np.asarray(proposal['open_source_fractions']);close_fractions=np.asarray(proposal['closed_source_fractions']);planned=True
                        if macro<10:
                            phase_name='settle';arm_target=start_arm;hand_fractions=open_fractions
                        elif macro<50:
                            phase_name='approach_high';alpha=(macro-10)/39.;arm_target=(1-alpha)*start_arm+alpha*np.asarray(path[-1]['arm_absolute_qpos_rad']);hand_fractions=open_fractions
                        else:
                            if macro<120:
                                phase_name='descend';coordinate=10*(1-(macro-50)/69.);hand_fractions=open_fractions
                            elif macro<160:
                                phase_name='close';coordinate=0.;alpha=min(1.,(macro-120)/30.);hand_fractions=(1-alpha)*open_fractions+alpha*close_fractions
                            elif macro<200:
                                phase_name='lift';coordinate=10*(macro-160)/39.;hand_fractions=close_fractions
                            else:
                                phase_name='hold';coordinate=10.;hand_fractions=close_fractions
                            i=min(9,int(coordinate));alpha=coordinate-i;arm_target=(1-alpha)*np.asarray(path[i]['arm_absolute_qpos_rad'])+alpha*np.asarray(path[i+1]['arm_absolute_qpos_rad'])
                        current=np.r_[arm_target,2*hand_fractions-1]
                        obs,_,_,_=env.step(current);case['scripted_command_steps']+=1
                        raw=np.r_[obs['robot0_joint_pos'],obs['robot0_gripper_qpos']];assert raw.shape==(18,) and np.isfinite(raw).all()
                        if first_actual_contact_macro is None:
                            for contact in d.contact:
                                bn=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,int(m.geom_bodyid[g])) for g in [contact.geom1,contact.geom2]]
                                if 'white_bowl_1_main' in bn and any(str(n).startswith('gripper0_') for n in bn):first_actual_contact_macro=macro;break
                    case['status']='DIAGNOSTIC_FINISHED'
                except StopDiagnostic:case['status']='DIAGNOSTIC_STOPPED'
                except Exception as error:case['status']='DIAGNOSTIC_STOPPED';case['stop_reason']='RUNTIME_ERROR: '+type(error).__name__+': '+str(error)
            case.update(simulated_time_s=float(d.time),max_linear_mimic_error_rad=max_mimic,max_arm_target_error_rad=max_arm_error,
                        max_hand_actuator_force_Nm=max_force,max_scene_qvel=max_speed,bowl_contact_steps=contact_steps,
                        min_bowl_contact_distance_m=min_dist,max_bowl_contact_normal_N=max_normal,contact_body_pairs=[list(x) for x in sorted(pairs)],initial_bowl_z_m=initial_bowl_z,final_bowl_z_m=float(d.xpos[bowl_id,2]),peak_bowl_height_change_m=peak_bowl_z-initial_bowl_z,thumb_contact_physics_steps=thumb_contact_steps,opposed_contact_physics_steps=opposed_contact_steps,max_other_robot_overlap_m=max_other_overlap,close_phase_start_macro=120 if planned else None,first_actual_contact_macro=first_actual_contact_macro,final_phase=phase_name,native_task_signal=bool(env.check_success()),strict_grasp_success_claim=False)
            # env.step may stop before returning its first observation. Preserve the
            # actual current state and any last complete camera observation.
            raw=d.qpos[indices].copy()
            images={}
            if isinstance(obs, dict):
                from PIL import Image
                for key,name,target in [('agentview_image','agentview_openpi_final.png','base'),
                                        ('robot0_eye_in_hand_image','wrist_openpi_final.png','wrist')]:
                    if key in obs:
                        Image.fromarray(obs[key][::-1,::-1]).save(out/name);images[target]=name
            (out/'final-observation-fixture.json').write_text(json.dumps({'state_18':raw.tolist(),'scripted_action_13':current.tolist(),'images':images,'fixture_only_not_demonstration':True,'purpose':'Failed or successful diagnostic commands must not become training labels'},ensure_ascii=False,indent=2)+'\n')
            (out/(label+'-result.json')).write_text(json.dumps(case,ensure_ascii=False,indent=2)+'\n');report['cases'].append(case)
            print(label,case['status'],'steps',case['physics_steps'],'mimic',max_mimic,'arm error',max_arm_error,'bowl contacts',contact_steps,'distance',min_dist,flush=True)
        report['status']='SMALL_WHITE_BOWL_CLOSURE_DIAGNOSTIC_RECORDED'
        report['source_face_flange_proxy_used']=True;report['independent_hand_targets']={'open_source_fractions':open_fractions.tolist(),'closed_source_fractions':close_fractions.tolist(),'changed_channels':active,'planned_closure_executed':planned and case['final_phase']!='settle'};report['strict_grasp_claim']=False
        report['temporary_stopping_bounds']={'mimic_rad':.05,'contact_overlap_m':.002,'scope':'Conservative stop for this throwaway diagnostic only; no product safety or success thresholds changed'}
    except Exception as error:
        if 'case' not in locals():
            case={'case':'bounded_grasp','status':'DIAGNOSTIC_STOPPED',
                  'physics_steps':0,'scripted_command_steps':0,'stop_reason':None,
                  'trajectory':'bounded_grasp.jsonl'}
        case['status']='DIAGNOSTIC_STOPPED'
        message='SETUP_OR_OUTPUT_ERROR: '+type(error).__name__+': '+str(error)
        if case.get('stop_reason'):
            case['recording_error']=message
        else:
            case['stop_reason']=message
        if not any(existing is case for existing in report['cases']):report['cases'].append(case)
        report['status']='DIAGNOSTIC_STOPPED'
        (out/'bounded_grasp-result.json').write_text(json.dumps(case,ensure_ascii=False,indent=2)+'\n')
    finally:mujoco.mj_step=old_step
    (out/'dynamic-results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report
