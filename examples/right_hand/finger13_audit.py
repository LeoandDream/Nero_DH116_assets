#!/usr/bin/env python3
"""Static compiled invariants for one local finger13 replacement."""
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
OLD = ROOT/'examples/right_hand/candidates/proxy76/model.xml'


def audit(candidate):
    spec = importlib.util.spec_from_file_location('finger13_compiled_fields',
                    ROOT/'examples/libero_reference/checks.py')
    checks = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checks)
    manifest = json.loads((candidate.parent/'manifest.json').read_text())
    if sha(OLD) != manifest['baseline_model_sha256'] or sha(candidate) != manifest['candidate_model_sha256']:
        raise ValueError('Frozen candidate identity changed')
    removed = manifest['removed_finger13_geom']['name']
    created = [r['candidate_geom'] for r in manifest['geoms'] if r['link']=='finger13_link']
    # Compare actual assets as well as compiled bounding/physical fields.
    # A mesh with changed interior vertices could otherwise keep the same bbox.
    def normalized(path, repair):
        xml = ET.parse(path).getroot()
        xml.attrib.pop('model',None)
        for node in xml.findall('.//*[@file]'):
            resource=(path.parent/node.get('file')).resolve()
            if not resource.is_relative_to(ROOT):
                raise AssertionError('Resource outside checkout')
            node.set('file',str(resource.relative_to(ROOT))+':'+sha(resource))
        assets=xml.find('asset')
        if repair:
            for asset in list(assets):
                if asset.get('name') in created:
                    assets.remove(asset)
        body=xml.find('.//body[@name="gripper0_finger13_link"]')
        for geom in list(body.findall('geom')):
            if geom.get('group','0')=='0':
                body.remove(geom)
        def value(node):
            return [node.tag,sorted(node.attrib.items()),[value(c) for c in node]]
        return value(xml)
    if normalized(OLD,False)!=normalized(candidate,True):
        raise AssertionError('XML or asset bytes outside authorized finger13 shapes changed')
    old_xml,new_xml=ET.parse(OLD).getroot(),ET.parse(candidate).getroot()
    frozen=manifest['frozen_geom_names_and_mesh_sha256']
    if len(frozen)!=75:
        raise AssertionError('Expected exactly 75 frozen proxy bindings')
    for name,binding in frozen.items():
        if sha(ROOT/binding['file'])!=binding['sha256']:
            raise AssertionError('Frozen mesh bytes changed: '+name)
        for xml in (old_xml,new_xml):
            geom=xml.find('.//geom[@name="'+name+'"]')
            if geom is None or geom.get('mesh')!=binding['mesh']:
                raise AssertionError('Frozen geom to mesh mapping changed: '+name)
    template=manifest['removed_finger13_geom']
    for name in created:
        actual=dict(new_xml.find('.//geom[@name="'+name+'"]').attrib)
        expected=copy.deepcopy(template)
        expected.update(name=name,mesh=name)
        if actual!=expected:
            raise AssertionError('New contact/pose template changed')
    differences, count = [], 0
    with static_only() as calls:
        old = mujoco.MjModel.from_xml_path(str(OLD))
        new = mujoco.MjModel.from_xml_path(str(candidate))
        a, b = checks.compiled_snapshot(old), checks.compiled_snapshot(new)
        for category, objects in a['objects'].items():
            lookup = {item['name']:item for item in b['objects'][category]}
            for item in objects:
                if category=='GEOM' and item['name']==removed:
                    continue
                other = lookup.get(item['name'])
                if other is None:
                    differences.append({'missing':item['name']})
                    continue
                for section in ('fields','bindings','dofs'):
                    for key,value in item.get(section,{}).items():
                        count += 1
                        if not np.array_equal(np.asarray(value),np.asarray(other[section].get(key))):
                            differences.append({'category':category,'object':item['name'],
                                                'field':key,'old':value,'new':other[section].get(key)})
        for key,value in a['options'].items():
            count += 1
            if not np.array_equal(value,b['options'].get(key)):
                differences.append({'option':key})
        for key in ('nq','nv','nu','na','nbody','njnt','neq','nexclude','nsensor','nkey','npair'):
            count += 1
            if getattr(old,key)!=getattr(new,key):
                differences.append({'dimension':key})
        for key in ('exclude_signature','key_qpos','key_qvel','key_ctrl'):
            count += 1
            if not np.array_equal(getattr(old,key),getattr(new,key)):
                differences.append({'field':key})
        for name in created:
            for field in ('geom_contype','geom_conaffinity','geom_condim','geom_friction',
                          'geom_solref','geom_solimp','geom_margin','geom_gap',
                          'geom_priority','geom_solmix','geom_group'):
                count += 1
                if not np.array_equal(getattr(old,field)[old.geom(removed).id],
                                      getattr(new,field)[new.geom(name).id]):
                    differences.append({'new_geom':name,'contact_field':field})
        result={'schema':'finger13-local-compiled-invariants-v1','passed':not differences,
            'protected_fields_checked':count,'differences':differences,
            'old_ngeom':old.ngeom,'new_ngeom':new.ngeom,
            'old_model_sha256':sha(OLD),'candidate_model_sha256':sha(candidate),
            'old_hand_proxies':76,'new_hand_proxies':78,'frozen_other_hand_proxies':75,
            'frozen_geom_names_mesh_bindings_and_bytes_exact':True,
            'normalized_xml_outside_finger13_geometry_exact':True,
            'new_finger13_geoms':created,'static_api_audit':calls,
            'scope':'Compiled mechanics and controls exact outside authorized finger13 geometry; no dynamics.'}
    if differences:
        raise AssertionError(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if args.output.exists():
        raise SystemExit('Use a new output file')
    result=audit(args.candidate.resolve())
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ('passed','protected_fields_checked','old_ngeom','new_ngeom')}))
