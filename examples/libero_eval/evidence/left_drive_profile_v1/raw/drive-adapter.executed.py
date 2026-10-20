#!/usr/bin/env python3
"""One authorized B drive-package trial, reusing close149 without changing it.

Import, profile preparation and snapshot comparison are stdlib-only. Native
compilation/Reset happen only inside the existing one-attempt execution chain.
"""
import argparse
from contextlib import contextmanager
from copy import deepcopy
import difflib
import importlib.util
import json
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

import close149 as C

R, ROOT = C.R, C.ROOT
BASELINE = '43bc90b8932cb2199ad7b19ca00611259d3f7d49'
AUTHORIZATION = 'https://github.com/LeoandDream/Nero_DH116_assets/issues/5#issuecomment-6297636306'
PROFILE_DIR = ROOT/'examples/right_hand/candidates/left_drive_profile_v1'
PROFILE_SHA = '59192bdd26d1020576ca854387bfd33444e0cff6f378bebb5307e3c330ca6a0e'
PATCH_SHA = '0d4c4c78a8aa4080809a739845dfa49268653cafb8de744bcfde7ded549e5c12'
CLOSE149_SHA = 'bfc34723de1bcd1741e19a9b7d29ac354477d244fa9b389556f4a3ce811b3e87'
A_PREFLIGHT = Path('examples/libero_eval/evidence/close149/raw/robot-physical-preflight.json.gz')


def load_profile():
    if R.digest(PROFILE_DIR/'profile.json') != PROFILE_SHA or R.digest(PROFILE_DIR/'model.patch') != PATCH_SHA:
        raise ValueError('Frozen B profile/patch differ')
    profile = R.read(PROFILE_DIR/'profile.json')
    for path, expected in profile['source_sha256'].items():
        if R.digest(ROOT/path) != expected:
            raise ValueError('Frozen drive source differs: '+path)
    if len(profile['changes']) != 17 or sum(len(c['attribute_changes']) for c in profile['changes']) != 34:
        raise ValueError('Expected precisely 17 objects and 34 attributes')
    if C.canonical_sha(C.planned_schedule()) != profile['schedule_canonical_json_sha256']:
        raise ValueError('Close149 actions differ')
    return profile


def materialize(output):
    """Keep the exact frozen patch copy, then explicitly resolve original assets."""
    profile = load_profile()
    base = ROOT/profile['base_model']
    raw = base.read_text()
    for change in profile['changes']:
        tag, name = change['tag'], change['name']
        pattern = r'<'+tag+r'\b[^>]*\bname="'+re.escape(name)+r'"[^>]*/>'
        matches = list(re.finditer(pattern, raw))
        if len(matches) != 1: raise ValueError('Missing/duplicate XML mapping: '+name)
        item = matches[0][0]
        for field, values in change['attribute_changes'].items():
            token = field+'="'+values['before']+'"'
            if item.count(token) != 1: raise ValueError('Source attribute differs: '+name+'/'+field)
            item = item.replace(token, field+'="'+values['after']+'"', 1)
        raw = raw[:matches[0].start()]+item+raw[matches[0].end():]
    import hashlib
    if hashlib.sha256(raw.encode()).hexdigest() != profile['candidate_xml_sha256_in_original_base_context']:
        raise ValueError('Candidate differs from reviewed patch')
    tree = ET.fromstring(raw)
    resources = []
    for element in tree.iter():
        if element.get('file'):
            original = element.get('file')
            resolved = (base.parent/original).resolve()
            if not resolved.is_relative_to(ROOT) or not resolved.is_file():
                raise ValueError('Missing/outside original resource context: '+original)
            resources.append({'asset':element.get('name'), 'original':original,
                              'resolved':str(resolved), 'sha256':R.digest(resolved)})
            element.set('file',str(resolved))
    target = Path(output)/'drive-model'
    target.mkdir()  # Never overwrite another candidate or attempt.
    (target/'original-context.xml').write_text(raw)
    ET.ElementTree(tree).write(target/'model.xml',encoding='unicode')
    shutil.copyfile(base.parent/'manifest.json',target/'manifest.json')
    record = {'profile':profile['profile'],'base_model':str(base), 'base_sha256':R.digest(base),
              'raw_copy':str(target/'original-context.xml'),'raw_sha256':R.digest(target/'original-context.xml'),
              'resolved_model':str(target/'model.xml'),'resolved_sha256':R.digest(target/'model.xml'),
              'resource_rule':'Only file attributes rebound to the original finger22_local context; no asset bytes changed.',
              'attribute_count':34,'changes':profile['changes'],'resources':resources}
    R.write(Path(output)/'drive-materialization.json',record)
    return record


def expected_snapshot(a, profile):
    """Build the exact expected B inventory by NAME/binding, never fixed IDs."""
    result = deepcopy(a)
    objects = {}
    for kind in ('JOINT','ACTUATOR'):
        rows=result['objects'][kind]
        objects[kind]={r['name']:r for r in rows}
        if len(objects[kind])!=len(rows): raise ValueError('Duplicate '+kind+' names')
    for change in profile['changes']:
        name=change['name']; attrs=change['attribute_changes']
        if change['tag']=='joint':
            row=objects['JOINT'].get(name)
            if row is None or row['fields']['jnt_type']!=3: raise ValueError('Missing hand hinge: '+name)
            for attr in ('damping','armature'):
                field='dof_'+attr
                if row['dofs'][field]!=[float(attrs[attr]['before'])]: raise ValueError('A DOF field differs: '+name+'/'+field)
                row['dofs'][field]=[float(attrs[attr]['after'])]
        else:
            row=objects['ACTUATOR'].get(name)
            if row is None or row['bindings'].get('actuator_trnid')!=change['source_joint']:
                raise ValueError('Missing/wrong actuator binding: '+name)
            if sum(r['bindings'].get('actuator_trnid')==change['source_joint'] for r in objects['ACTUATOR'].values())!=1:
                raise ValueError('Ambiguous actuator binding: '+name)
            f=row['fields']; kp,kv=attrs['kp'],attrs['kv']
            if f['actuator_gainprm']!=[float(kp['before'])]+[0.]*9 or f['actuator_biasprm']!=[0.,-float(kp['before']),-float(kv['before'])]+[0.]*7:
                raise ValueError('A gain/bias differs: '+name)
            f['actuator_gainprm'][0]=float(kp['after'])
            f['actuator_biasprm'][1:3]=[-float(kp['after']),-float(kv['after'])]
    return result


def audit_drive_snapshot(a,b,profile):
    if expected_snapshot(a,profile)!=b:
        raise ValueError('B compiled inventory contains missing, incorrect, or unapproved differences from A')
    return {'passed':True,'reference':str(A_PREFLIGHT),'profile':profile['profile'],
            'allowed_xml_attributes':34,'same_snapshot_as_A':False,
            'all_other_inventory_fields_exact':True,
            'selection':'joint names and actuator joint bindings; full snapshot, not row-wide exemptions'}


def reverse_A_source(native_xml,profile,b_snapshot):
    """Reverse drive fields; return fixture poses to apply AFTER compilation.

    Native Reset changes these model fields without recomputing constants.
    Stamping XML before compilation would change the reference context.
    """
    xml=ET.fromstring(native_xml)
    for change in profile['changes']:
        nodes=xml.findall('.//'+change['tag']+'[@name="'+change['name']+'"]')
        if len(nodes)!=1: raise ValueError('Native reverse mapping is not unique')
        for field,values in change['attribute_changes'].items():
            if float(nodes[0].get(field))!=float(values['after']): raise ValueError('Native B XML value differs')
            nodes[0].set(field,values['before'])
    poses={b['name']:b['fields'] for b in b_snapshot['objects']['BODY']}
    fixtures=('flat_stove_1_main','wooden_two_layer_shelf_1_main')
    updated={name:{f:poses[name][f] for f in ('body_pos','body_quat')} for name in fixtures}
    return ET.tostring(xml,encoding='unicode'),updated


def audit_derived(model, output, profile):
    """Compare current B constants with an A scene COMPILE, never an A rollout.

    This does not touch the live B data, refresh its caches, or clear warmstart.
    The A source is the current native XML with only the 34 attributes reversed.
    """
    import mujoco
    import numpy as np
    spec=importlib.util.spec_from_file_location('drive_inventory',ROOT/'examples/libero_reference/checks.py')
    checks=importlib.util.module_from_spec(spec);spec.loader.exec_module(checks)
    source,poses=reverse_A_source((output/'model.xml').read_text(),profile,checks.compiled_snapshot(model))
    (output/'drive-A-compile-only.xml').write_text(source)
    a=mujoco.MjModel.from_xml_string(source)
    for name,fields in poses.items():
        index=a.body(name).id
        a.body_pos[index]=fields['body_pos']
        a.body_quat[index]=fields['body_quat']
    if checks.compiled_snapshot(a)!=R.read(ROOT/A_PREFLIGHT)['actual_snapshot']:
        raise ValueError('Reverse-only A scene compile does not match historical A inventory')
    hand={c['name'] for c in profile['changes'] if c['tag']=='joint'}
    report={'A_reference':'One reverse-34-attributes scene compilation; no A Reset/forward/episode.',
            'constant_pose':'compiler qpos0; not the seeded native Reset posture',
            'extra_A_scene_compilations':1,'extra_A_episode_steps':0,
            'body_pose_source':'Only two fixture roots assigned to independent A model AFTER compilation, as native Reset does; original XML/constant context and quaternion tokens preserved. No A forward/setConst/Reset/step, no live B write.',
            'A_model_fixture_assignments_after_compilation':poses,'fields':{}}
    for field,kind in [('dof_M0','dof'),('dof_invweight0','dof'),('body_invweight0','body'),('actuator_acc0','actuator')]:
        av,bv=np.asarray(getattr(a,field)),np.asarray(getattr(model,field))
        if av.shape!=bv.shape or not np.all(np.isfinite(av)) or not np.all(np.isfinite(bv)):
            raise ValueError('Nonfinite/shape mismatch in derived constant '+field)
        rows=[]
        for i in range(len(av)):
            if kind=='dof': name=model.joint(int(model.dof_jntid[i])).name
            elif kind=='body': name=model.body(i).name
            else: name=model.actuator(i).name
            changed=not np.array_equal(av[i],bv[i])
            if changed and not (name or '').startswith(('robot0_','gripper0_','mount0_')):
                raise ValueError('Derived change outside robot: '+field+'/'+str(name))
            if field=='dof_M0':
                delta=.01 if name in hand else 0.
                # Floating arithmetic bound for one scalar add/subtract only;
                # never a contact/control acceptance tolerance.
                bound=64*np.finfo(float).eps*max(1.,abs(float(av[i])),abs(float(bv[i])))
                if abs(float(av[i]-bv[i])-delta)>bound: raise ValueError('Unexplained dof_M0 change: '+str(name))
            if changed: rows.append({'name':name,'A':av[i].tolist(),'B':bv[i].tolist()})
        report['fields'][field]={'changed':rows,'all_other_entries_exact':True}
    report['meaninertia']={'A':float(a.stat.meaninertia),'B':float(model.stat.meaninertia)}
    report['origin_verification']='Same native scene/resources; reversed A inventory matches saved A exactly; B differs by frozen drive package only.'
    R.write(output/'drive-derived-preflight.json',report)


def adapted_executor(legacy, policy, observations, output):
    profile=load_profile()
    material=materialize(output)
    execute=C.adapted_executor(legacy,policy,observations,output)
    scope=execute.__globals__
    old_audit,old_write=scope['robot_audit'],scope['write']
    actual={}
    def strict_audit(formal,model):
        result=old_audit(formal,model)  # B formal -> B actual remains unmodified.
        if not result['passed']: return result
        result['drive_A_to_B']=audit_drive_snapshot(R.read(ROOT/A_PREFLIGHT)['actual_snapshot'],result['actual_snapshot'],profile)
        actual['model']=model
        return result
    def write(path,value):
        if Path(path).name=='frozen-before-first-step.json':
            audit_derived(actual['model'],Path(output),profile)
            value.update(drive_profile=profile['profile'],drive_materialization=material,
                reset_cache_scope='Original qpos/qvel/ctrl/time/controller/scene checks retained. qacc/warmstart/force caches are B initialization, not overwritten or required to equal A.')
        if Path(path).name=='result.json':
            value.update(drive_profile=profile['profile'],model_modified=True,
                formal_assets_modified=False,only_model_change='34 frozen hand drive attributes',
                drive_materialization=material,baseline_A_record=str(A_PREFLIGHT.parent),
                compiler_derived_A_reference_is_not_A_rollout=True)
        old_write(path,value)
    def reset_cache(model,data,output):
        R.write(output/'drive-native-reset-cache.json',{
            'timing':'Native Reset cache before first episode step; no extra live forward or cache writes.',
            'comparison_to_historical_A':'UNKNOWN: these caches were not saved at A Reset; never copied from A.',
            'time_s':float(data.time),
            'cache':{name:getattr(data,name).tolist() for name in
                     ('qacc','qacc_warmstart','actuator_force','qfrc_actuator','qfrc_constraint','qfrc_passive','qfrc_bias')},
            'warning_counts':[int(w.number) for w in data.warning]})
    scope.update(robot_audit=strict_audit,write=write,_drive_reset_cache=reset_cache)
    path=Path(output)/'executor.executed.py'
    original=path.read_text()
    changed=C.replace_once(original,"        write(output / 'frozen-before-first-step.json', frozen)",
        "        _drive_reset_cache(model, data, output)\n        write(output / 'frozen-before-first-step.json', frozen)")
    (Path(output)/'drive-executor.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True))))
    path.write_text(changed)
    exec(compile(changed,str(path),'exec'),scope)
    return scope['execute']


@contextmanager
def selected_policy(module):
    # The dependency object is newly constructed by the frozen research entry;
    # no imported historical policy or original runner is modified.
    output=selected_policy.output
    model=output/'drive-model/model.xml'
    module.MODEL=model.relative_to(ROOT)
    module.MODEL_SHA=R.digest(model)
    original_validate=module.validate_external_preflight
    def validate(root,path):
        result=original_validate(root,path)  # Original A source/resource checks.
        load_profile()
        return {**result,'drive_profile':R.read(output/'drive-materialization.json')}
    def compare(_old,new,_removed,_added):
        return audit_drive_snapshot(R.read(ROOT/A_PREFLIGHT)['actual_snapshot'],new,load_profile())
    module.validate_external_preflight=validate
    module.compare_reset_snapshots=compare
    with R.selected_policy(module): yield


def prepared_entry(output):
    if R.digest(ROOT/'examples/right_hand/close149.py')!=CLOSE149_SHA:
        raise ValueError('Frozen close149 entry changed')
    load_profile()
    entry,original,changed=C.prepared_entry()
    changed=C.replace_once(changed,"experiment='issue5_close149_once'","experiment='issue5_close149_left_drive_profile_v1_once'")
    changed=C.replace_once(changed,'(Path(__file__), _research_source, POLICY_DIR/',
        '(Path(__file__), _close149_source, _profile_path, _patch_path, _research_source, POLICY_DIR/')
    scope=dict(entry.__globals__)
    selected_policy.output=Path(output).resolve()
    scope.update(__file__=__file__,BASELINE=BASELINE,AUTHORIZATION=AUTHORIZATION,
        adapted_executor=adapted_executor,selected_policy=selected_policy,
        _close149_source=ROOT/'examples/right_hand/close149.py',
        _profile_path=PROFILE_DIR/'profile.json',_patch_path=PROFILE_DIR/'model.patch')
    model=Path(output).resolve()/'drive-model/model.xml'
    model_label=str(model.relative_to(ROOT)) if model.is_relative_to(ROOT) else str(model)
    profile=load_profile()
    scope['_close149_registration']=dict(scope['_close149_registration'],
        drive_profile='R_close149_left_drive_profile_v1',drive_attributes=34,
        model=model_label,model_sha256=None,
        actual_model_sha256_record='drive-materialization.json:resolved_sha256',
        base_model=profile['base_model'],base_model_sha256=profile['source_sha256'][profile['base_model']],
        A='Reuse saved close149; do not run A or left',
        physics_prefix_equality_to_A='NOT_REQUIRED_DRIVE_CHANGED_FROM_RESET')
    exec(compile(changed,str(Path(__file__)), 'exec'),scope)
    return scope['run_once'],original,changed


def run_once(output):
    output=Path(output).resolve()
    if output.exists(): raise ValueError('Fresh output required; refusing another attempt')
    entry,original,changed=prepared_entry(output)
    code=entry(output)  # Exactly once; preserve actual exit code.
    if output.exists():
        (output/'drive-entry.executed.py').write_text(changed+'\n')
        (output/'drive-entry.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True))))
    return code


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    raise SystemExit(run_once(parser.parse_args().output))
