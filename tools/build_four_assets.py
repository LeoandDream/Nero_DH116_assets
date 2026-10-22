#!/usr/bin/env python3
"""Package existing verified geometry/physics; never generate or tune proxies."""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
LEFT='mjcf/nero_dh116.xml'
RIGHT='examples/libero_eval/evidence/functional_v1/raw/drive-model/original-context.xml'
RIGHT_CONTEXT='examples/right_hand/candidates/finger22_local'
LEFT_SHA='c446124b7d938aa6e00ba5914047001de66c2a2c162e7dea1644437e5a85cb33'
RIGHT_SHA='e24d17904b9b1d4a55ae031a08bb1457f0f46c4170f987767d0acd161eb3e84b'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def build(source=ROOT,output=ROOT):
    source,output=Path(source).resolve(),Path(output).resolve()
    if sha(source/LEFT)!=LEFT_SHA or sha(source/RIGHT)!=RIGHT_SHA:
        raise ValueError('Verified L V1 / R B model bytes differ')
    (output/'mjcf').mkdir(parents=True,exist_ok=True)
    (output/'docs').mkdir(parents=True,exist_ok=True)
    if source!=output:shutil.copyfile(source/LEFT,output/LEFT)
    resources={}
    def resources_for(tree,context,rewrite):
        for node in tree.iter():
            if not node.get('file'):continue
            original=(context/node.get('file')).resolve()
            relative=original.relative_to(source)
            if relative.parts[0]=='meshes':target=relative
            else:
                folder='meshes/dh116_right/research' if node.get('name','').startswith('gripper0_') else 'meshes/nero/research_wrist'
                target=Path(folder)/(node.get('name',original.stem)+original.suffix)
            path=output/target
            if not path.exists():path.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(original,path)
            if sha(path)!=sha(original):raise ValueError('Resource destination collision: '+str(target))
            resources[str(target)]={'sha256':sha(original),'origin':str(relative),
                                    'operation':'ORIGINAL_BYTES_REUSED' if relative==target else 'BYTE_COPY_NO_GEOMETRY_CHANGE'}
            if rewrite:node.set('file',os.path.relpath(path,output/'mjcf'))
    left=ET.parse(source/LEFT).getroot()
    resources_for(left,(source/LEFT).parent,False)
    right=ET.parse(source/RIGHT).getroot()
    resources_for(right,source/RIGHT_CONTEXT,True)
    right.set('model','nero_dh116_right_research_B')
    right_path=output/'mjcf/nero_dh116_right_research.xml'
    ET.ElementTree(right).write(right_path,encoding='utf-8',xml_declaration=True)

    def standalone(combined,side):
        tree=ET.Element('mujoco',model='dh116_'+side)
        for tag in ('compiler','option','size','default'):
            if combined.find(tag) is not None:tree.append(deepcopy(combined.find(tag)))
        assets=ET.SubElement(tree,'asset')
        for element in combined.find('asset'):
            if element.get('name','').startswith('gripper0_'):assets.append(deepcopy(element))
        ET.SubElement(tree,'worldbody').append(deepcopy(combined.find('.//body[@name="gripper0_base_link"]')))
        for tag,reference in [('actuator','joint'),('equality','joint1'),('sensor','site'),('contact','body1')]:
            container=ET.SubElement(tree,tag)
            for element in combined.findall(tag+'/*'):
                if element.get(reference,'').startswith('gripper0_'):container.append(deepcopy(element))
        # MuJoCo skips parent filtering when one weld body is the world.
        # Fixing the palm must preserve the five adjacency filters already
        # implicit in the attached model, not introduce new base/finger forces.
        contact=tree.find('contact')
        for finger in ('11','21','31','41','51'):
            ET.SubElement(contact,'exclude',body1='gripper0_base_link',body2='gripper0_finger'+finger+'_link')
        qpos=combined.find('keyframe/key').get('qpos').split()[7:]
        if len(qpos)!=11:raise ValueError('Expected 11 hand joints')
        ET.SubElement(ET.SubElement(tree,'keyframe'),'key',name='reference',
            qpos=' '.join(qpos),qvel=' '.join(['0']*11),ctrl=' '.join(['0']*6))
        path=output/('mjcf/dh116_'+side+'.xml')
        ET.ElementTree(tree).write(path,encoding='utf-8',xml_declaration=True)
        return path
    standalone(left,'left');standalone(right,'right')

    contract=json.loads((source/'docs/robot-contract.json').read_text())
    hand_order=contract['state_order'][7:]
    active=['gripper0_'+name for name in contract['hand_active_order']]
    variants={}
    for name,side,arm,path in [('dh116_left','L',False,'mjcf/dh116_left.xml'),
            ('dh116_right','R',False,'mjcf/dh116_right.xml'),
            ('nero_dh116_left','L',True,LEFT),
            ('nero_dh116_right','R',True,'mjcf/nero_dh116_right_research.xml')]:
        variants[name]={'side':side,'arm':arm,'model':path,'sha256':sha(output/path),
            'state_dim':18 if arm else 11,'action_dim':13 if arm else 6,
            'state_order':contract['state_order'] if arm else hand_order,
            'active_order':active,'profile':'left V1' if side=='L' else 'B + functional_v1',
            'hand_collision_proxies':76 if side=='L' else 81,
            'wrist_collision_proxies':(15 if side=='L' else 306) if arm else 0,
            'standalone_frame':None if arm else 'Fixed gripper0_base_link at local palm frame origin; same mechanical subtree, no NERO/flange.',
            'standalone_parent_filter':None if arm else 'Five explicit palm/proximal exclusions preserve attached-model parent filtering; all other self/external collision eligibility retained.',
            'functional_evidence':('examples/libero_reference/evidence/RESULTS.md' if side=='L' else
                                   'examples/libero_eval/evidence/functional_v1/RESULTS.md') if arm else None}
    manifest={'schema':1,'assets':variants,'hand_state_order':hand_order,'hand_active_order':active,
        'units':{'position':'rad','hand_action':'normalized absolute [-1,1]','arm_action':'absolute joint positions rad',
                 'ctrl':'7 NERO torques followed by 6 hand joint target angles; standalone only 6 hand angles'},
        'baselines':{'left_main_commit':'9b5ee0d38442dcf12bc9f41ec5a0111fb0bd6a7c','left_sha256':LEFT_SHA,
            'right_commit':'7998718be17d39a2491f76ee747e07309b6b3193','right_B_raw_sha256':RIGHT_SHA,
            'right_raw_source':RIGHT,'right_original_resource_context':RIGHT_CONTEXT},
        'resources':dict(sorted(resources.items())),
        'licenses':'NERO MIT; DH116 Apache-2.0; manufacturer flange independent permission unresolved. See licenses/SOURCES.md.',
        'scope':'Research assets, bounded control and fixed white-bowl capability; historical FAIL and fidelity UNKNOWN retained.'}
    (output/'docs/assets.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,default=ROOT)
    args=parser.parse_args()
    result=build(ROOT,args.output_root)
    print('Packaged four entries and',len(result['resources']),'original-byte resources; no physics or proxy generation.')
