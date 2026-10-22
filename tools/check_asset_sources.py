#!/usr/bin/env python3
"""Explicit source-equivalence audit only; no reset, forward or integration.

Not imported by the delivery runtime. Historical inputs are comparison targets.
"""
import argparse
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
from itertools import combinations
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def eligible(model,g1,g2):
    """MuJoCo 3.2.3 filterBodyPair + mask/exclude, without spatial broadphase.

    Explicit contact pairs would override this; these assets have none.
    """
    b1,b2=int(model.geom_bodyid[g1]),int(model.geom_bodyid[g2])
    w1,w2=int(model.body_weldid[b1]),int(model.body_weldid[b2])
    if w1==w2:return False
    p1,p2=int(model.body_weldid[model.body_parentid[w1]]),int(model.body_weldid[model.body_parentid[w2]])
    disabled=bool(model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT))
    if not disabled and w1 and w2 and (w1==p2 or w2==p1):return False
    if not ((model.geom_contype[g1]&model.geom_conaffinity[g2]) or (model.geom_contype[g2]&model.geom_conaffinity[g1])):return False
    return ((min(b1,b2)<<16)+max(b1,b2)) not in model.exclude_signature


def audit():
    manifest=json.loads((ROOT/'docs/assets.json').read_text())
    module_spec=importlib.util.spec_from_file_location('source_checks',ROOT/'examples/libero_reference/checks.py')
    checks=importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(checks)
    assets={name:mujoco.MjModel.from_xml_path(str(ROOT/row['model'])) for name,row in manifest['assets'].items()}
    old_path=ROOT/manifest['baselines']['right_raw_source']
    old=ET.parse(old_path)
    context=ROOT/manifest['baselines']['right_original_resource_context']
    for item in old.iter():
        if item.get('file'):item.set('file',str((context/item.get('file')).resolve()))
    with tempfile.TemporaryDirectory(prefix='issue6-source-compare-') as temp:
        path=Path(temp)/'reference.xml'
        old.write(path)
        reference=mujoco.MjModel.from_xml_path(str(path))
    right=assets['nero_dh116_right']
    checked=[];differences=[]
    # Compare all public compiled arrays except filename/name lookup metadata.
    # Includes mesh convex graphs/vertices/faces, keyframes and solver-derived fields.
    excluded=[]
    for field in dir(reference):
        if field.startswith('_'):continue
        value=getattr(reference,field)
        if not isinstance(value,np.ndarray):continue
        if field.startswith('name_') or field=='names_map' or field.endswith('_pathadr'):
            excluded.append(field);continue
        actual=getattr(right,field)
        checked.append(field)
        if not np.array_equal(value,actual):differences.append(field)
    source_snapshot=checks.compiled_snapshot(reference)
    delivered_snapshot=checks.compiled_snapshot(right)
    if source_snapshot!=delivered_snapshot:differences.append('named_physical_snapshot')
    saved=ROOT/'examples/libero_eval/evidence/functional_v1/raw/robot-physical-preflight.json.gz'
    with gzip.open(saved,'rt') as stream:historical=json.load(stream)['formal_snapshot']
    historical_match=source_snapshot==historical
    if not historical_match:differences.append('saved_actual_run_formal_snapshot')

    hand_results={}
    for side in ('left','right'):
        hand=assets['dh116_'+side]
        combo=assets['nero_dh116_'+side]
        hs,cs=checks.compiled_snapshot(hand),checks.compiled_snapshot(combo)
        counts={};errors=[]
        for category,items in hs['objects'].items():
            reference_items={r['name']:r for r in cs['objects'][category]}
            counts[category]=0
            for row in items:
                name=row['name']
                if name in ('world',None):continue
                other=reference_items.get(name)
                if other is None:errors.append(category+':'+name+':missing');continue
                if row['fields']!=other['fields']:errors.append(category+':'+name+':fields')
                if row.get('dofs')!=other.get('dofs'):errors.append(category+':'+name+':dofs')
                b1,b2=dict(row['bindings']),dict(other['bindings'])
                if name=='gripper0_base_link':b1.pop('body_parentid',None);b2.pop('body_parentid',None)
                if b1!=b2:errors.append(category+':'+name+':bindings')
                counts[category]+=1
        if hs['options']!=cs['options']:errors.append('solver/options')
        assert hand.npair==combo.npair==0
        qualification_differences=[]
        pair_counts=[0,0]
        for i,j in combinations(range(hand.ngeom),2):
            n1,n2=hand.geom(i).name,hand.geom(j).name
            before=eligible(combo,combo.geom(n1).id,combo.geom(n2).id)
            after=eligible(hand,i,j)
            pair_counts[0]+=int(before);pair_counts[1]+=int(after)
            if before!=after:qualification_differences.append([n1,n2,before,after])
        if qualification_differences:errors.append('internal_contact_qualification')
        mesh_checks=0
        for i in range(hand.nmesh):
            name=hand.mesh(i).name
            j=combo.mesh(name).id
            for field,adr,count in [('mesh_vert','mesh_vertadr','mesh_vertnum'),
                                    ('mesh_face','mesh_faceadr','mesh_facenum')]:
                a=int(getattr(hand,adr)[i]);b=int(getattr(combo,adr)[j]);n=int(getattr(hand,count)[i])
                if n!=int(getattr(combo,count)[j]) or not np.array_equal(getattr(hand,field)[a:a+n],getattr(combo,field)[b:b+n]):
                    errors.append('mesh:'+name+':'+field)
            for field in ('mesh_pos','mesh_quat','mesh_scale'):
                if not np.array_equal(getattr(hand,field)[i],getattr(combo,field)[j]):errors.append('mesh:'+name+':'+field)
            mesh_checks+=1
        hand_results[side]={'passed':not errors,'differences':errors,'named_objects_compared':counts,
                            'meshes_compared':mesh_checks,
                            'eligible_geom_pair_counts_source_export':pair_counts,
                            'contact_qualification_differences':qualification_differences,
                            'intentional_difference':'Remove arm/flange; fix palm at origin; remove their states/actuators. No hand parameter changes.'}
    left_sha=hashlib.sha256((ROOT/'mjcf/nero_dh116.xml').read_bytes()).hexdigest()
    left_original=left_sha==manifest['baselines']['left_sha256']
    return {'passed':left_original and not differences and all(r['passed'] for r in hand_results.values()),
        'mujoco_version':mujoco.__version__,'new_trajectory_steps':0,
        'right':{'passed':not differences,'source_sha256':hashlib.sha256(old_path.read_bytes()).hexdigest(),
                 'compiled_arrays_compared':len(checked),'compiled_array_fields':checked,
                 'excluded_name_and_path_metadata_only':excluded,
                 'named_snapshot_exact':source_snapshot==delivered_snapshot,
                 'saved_run_formal_snapshot_exact':historical_match,
                 'compiled_meshes':right.nmesh,'differences':differences,
                 'intentional_changes':['model label','relative mesh file paths only']},
        'standalone_hands':hand_results,
        'left':{'sha256':left_sha,'original_V1_bytes':left_original},
        'grasp_evidence_scope':'Existing LIBERO scenes/controllers/reset/action schedules still required for the historical task. No claim that bare MJCF alone contains a bowl or policy.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    result=audit()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,ensure_ascii=False))
    raise SystemExit(0 if result['passed'] else 1)
