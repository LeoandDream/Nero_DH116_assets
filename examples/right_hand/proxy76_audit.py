#!/usr/bin/env python3
"""Static proof that migration changes hand collision shapes alone."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from proxy76_contacts import static_only, sha

ROOT = Path(__file__).resolve().parents[2]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name,path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def normalized_mechanics(xml):
    xml = copy.deepcopy(xml)
    xml.attrib.pop('model',None)
    xml.remove(xml.find('asset'))
    for body in xml.findall('.//body'):
        if body.get('name','').startswith('gripper0_'):
            for geom in list(body.findall('geom')):
                if geom.get('group','0')=='0': body.remove(geom)
    def node(e): return [e.tag,sorted(e.attrib.items()),[node(c) for c in e]]
    return node(xml)


def audit(candidate):
    original = ROOT/'mjcf/nero_dh116_right.xml'
    before = {str(original.relative_to(ROOT)):sha(original), str(candidate.relative_to(ROOT)):sha(candidate)}
    source_xml, candidate_xml = ET.parse(original).getroot(), ET.parse(candidate).getroot()
    if normalized_mechanics(source_xml)!=normalized_mechanics(candidate_xml):
        raise AssertionError('Robot XML fields outside the authorized shape/asset scope changed')
    inventory = module('proxy76_compiled_inventory',ROOT/'examples/libero_reference/checks.py')
    builder = module('proxy76_audit_builder',ROOT/'tools/build_right_proxy76.py')
    manifest = json.loads((candidate.parent/'manifest.json').read_text())
    generated_checks = []
    for entry in manifest['geoms']:
        source = ROOT/entry['left_file']
        target = ROOT/entry['candidate_file']
        if sha(source)!=entry['left_sha256'] or sha(target)!=entry['candidate_sha256']:
            raise AssertionError('Proxy file binding changed')
        if entry['operation']=='direct_existing_mesh_reference':
            if source.resolve()!=target.resolve(): raise AssertionError('Direct reuse must keep source bytes')
            continue
        vertices, faces = builder.read_mesh(source)
        expected_v, expected_f = builder.transform_mesh(vertices,faces,np.array(entry['R']),np.array(entry['t']))
        actual_v, actual_f = builder.read_mesh(target)
        if not np.array_equal(expected_v,actual_v) or not np.array_equal(expected_f,actual_f):
            raise AssertionError('Baked mesh differs from the frozen local transform')
        generated_checks.append({'geom':entry['candidate_geom'],'all_vertices_exact':True,
                                 'all_faces_and_reflection_winding_exact':True})
    with static_only() as calls:
        a = mujoco.MjModel.from_xml_path(str(original))
        b = mujoco.MjModel.from_xml_path(str(candidate))
        aa, bb = inventory.compiled_snapshot(a),inventory.compiled_snapshot(b)
        # Identify the actual direct hand collision geoms, not mesh declarations.
        removed = {a.geom(i).name for i in range(a.ngeom)
                   if a.body(int(a.geom_bodyid[i])).name.startswith('gripper0_') and a.geom_group[i]==0}
        added = {e['candidate_geom'] for e in manifest['geoms']}
        actual_added = {b.geom(i).name for i in range(b.ngeom)
                        if b.body(int(b.geom_bodyid[i])).name.startswith('gripper0_') and b.geom_group[i]==0}
        if len(removed)!=17 or len(added)!=76 or actual_added!=added:
            raise AssertionError('Replacement scope is not exactly coarse17 -> proxy76')
        differences, checked = [],0
        for category, objects in aa['objects'].items():
            lookup = {n['name']:n for n in bb['objects'][category]}
            for expected in objects:
                if category=='GEOM' and expected['name'] in removed: continue
                actual = lookup.get(expected['name'])
                if actual is None:
                    differences.append({'category':category,'object':expected['name'],'reason':'MISSING'})
                    continue
                for section in ('fields','bindings','dofs'):
                    for key,value in expected.get(section,{}).items():
                        checked+=1
                        if not np.array_equal(np.asarray(value),np.asarray(actual[section].get(key))):
                            differences.append({'category':category,'object':expected['name'],'field':key,
                                                'expected':value,'actual':actual[section].get(key)})
        for key,value in aa['options'].items():
            checked+=1
            if not np.array_equal(value,bb['options'].get(key)):
                differences.append({'option':key,'expected':value,'actual':bb['options'].get(key)})
        for key in ('nq','nv','nu','na','nbody','njnt','neq','nexclude','nsensor','nkey','npair'):
            checked+=1
            if getattr(a,key)!=getattr(b,key): differences.append({'dimension':key})
        if not np.array_equal(a.exclude_signature,b.exclude_signature): differences.append({'reason':'EXCLUDES_CHANGED'})
        if not np.array_equal(a.key_qpos,b.key_qpos) or not np.array_equal(a.key_qvel,b.key_qvel) or not np.array_equal(a.key_ctrl,b.key_ctrl):
            differences.append({'reason':'KEYFRAME_CHANGED'})
        contact_fields = ('geom_contype','geom_conaffinity','geom_condim','geom_friction','geom_solref',
                          'geom_solimp','geom_margin','geom_gap','geom_priority','geom_solmix','geom_group')
        for entry in manifest['geoms']:
            reference = a.geom(entry['right_contact_template']['name']).id
            gid = b.geom(entry['candidate_geom']).id
            for field in contact_fields:
                checked+=1
                if not np.array_equal(getattr(a,field)[reference],getattr(b,field)[gid]):
                    differences.append({'geom':entry['candidate_geom'],'contact_field':field})
        expected_visuals = [g for g in source_xml.findall('.//geom') if g.get('name','').startswith('gripper0_') and g.get('group')=='1']
        old_assets = {m.get('name'):m for m in source_xml.findall('asset/mesh')}
        new_assets = {m.get('name'):m for m in candidate_xml.findall('asset/mesh')}
        for visual in expected_visuals:
            name = visual.get('mesh')
            source = (original.parent/old_assets[name].get('file')).resolve()
            target = (candidate.parent/new_assets[name].get('file')).resolve()
            if source!=target or sha(source)!=sha(target): differences.append({'visual':name})
        result = {'schema':'right-proxy76-static-mechanics-audit-v1','passed':not differences,
                  'robot_xml_outside_hand_collision_and_asset_references_exact':True,
                  'compiled_fields_checked':checked,'differences':differences,
                  'formal_hand_collision_geoms':17,'candidate_hand_collision_geoms':76,
                  'unchanged_right_source_visual_geoms':len(expected_visuals),
                  'direct_asset_references':57,'transformed_files_checked':generated_checks,
                  'compiled_counts':{'formal_ngeom':a.ngeom,'candidate_ngeom':b.ngeom,'nbody':b.nbody,'njnt':b.njnt,'nu':b.nu,'neq':b.neq},
                  'keyframe_exact':True,'contact_masks_parameters_excludes_exact':not differences,
                  'static_api_audit':calls,'input_sha256_before':before,
                  'input_sha256_after':{p:sha(ROOT/p) for p in before},
                  'scope':'Shape replacement/invariants only; no dynamics, reset, forward, control or task acceptance.'}
        if differences: raise AssertionError(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args = parser.parse_args()
    if args.output.exists(): raise SystemExit('Use a new output file')
    result = audit(args.candidate.resolve())
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ('passed','compiled_fields_checked','compiled_counts','static_api_audit')}))
