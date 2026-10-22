#!/usr/bin/env python3
"""Continuous convex triangle surfaces at the same frozen 6451 saved poses."""
import argparse
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = next(p for p in Path(__file__).resolve().parents
            if (p/'mjcf/nero_dh116_right.xml').is_file())
sys.path.insert(0,str(ROOT/'examples/right_hand'))
import proxy76_validate as reader
import numpy as np
from scipy.spatial import ConvexHull

spec=importlib.util.spec_from_file_location('existing_continuous_triangles',
    ROOT/'examples/right_hand/evidence/d3_geometry/triangle/surface_geometry.py')
geometry=importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)
LINKS=('finger13_link','finger42_link')

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--contacts',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise ValueError('Use a new evidence path')
    if args.contacts.suffix=='.gz':
        with gzip.open(args.contacts,'rt') as stream:
            cases=json.load(stream)
    else:
        cases=json.loads(args.contacts.read_text())
    paths={args.contacts.resolve(),Path(__file__).resolve(),Path(geometry.__file__),Path(reader.__file__)}
    bindings={}
    for label,path in [('proxy76',ROOT/'examples/right_hand/candidates/proxy76/model.xml'),
                       ('finger13_repair',args.candidate.resolve())]:
        paths.add(path)
        xml=ET.parse(path).getroot()
        assets={a.get('name'):a for a in xml.findall('asset/mesh')}
        bindings[label]={}
        for link in LINKS:
            body=xml.find('.//body[@name="gripper0_'+link+'"]')
            bindings[label][link]=[]
            for geom in body.findall('geom'):
                if geom.get('group','0')!='0':continue
                binding=reader._binding(path,xml,assets,geom)
                paths.add(Path(binding['file']))
                tris=reader.mesh_triangles(binding)
                vertices=np.unique(tris.reshape(-1,3),axis=0)
                hull=ConvexHull(vertices)
                bindings[label][link].append((binding['name'],vertices[hull.simplices]))
    before={str(p.relative_to(ROOT)):sha(p) for p in paths}
    results=[]
    for pose in cases['results']:
        if pose['pose']['checkpoint_step']!=6451:continue
        entry={'pose_id':pose['pose']['id'],'models':{}}
        transforms=pose['body_world_transforms']
        a,b=(transforms[l] for l in LINKS)
        ra,rb=np.array(a['rotation']),np.array(b['rotation'])
        pa,pb=np.array(a['translation_m']),np.array(b['translation_m'])
        for label,parts in bindings.items():
            rows=[]
            for na,ta in parts[LINKS[0]]:
                for nb,tb in parts[LINKS[1]]:
                    moved=tb@(ra.T@rb).T+ra.T@(pb-pa)
                    query=geometry.surface_query(ta,moved)
                    rows.append({'geoms':[na,nb],'certificate_body13_m':query})
            entry['models'][label]=rows
        results.append(entry)
    after={str(p.relative_to(ROOT)):sha(p) for p in paths}
    if before!=after:raise ValueError('An input changed')
    args.output.write_text(json.dumps({'schema':'finger13-6451-continuous-convex-surfaces-v1',
        'physics_steps':0,'mujoco_imports':0,'mujoco_api_calls':0,
        'method':'Actual raw collision meshes, independent per-part ConvexHull; cached exact verified body FK; existing complete continuous triangle BVH query, 1e-12 m absolute pruning bound.',
        'input_sha256_before':before,'input_sha256_after':after,'inputs_unchanged':True,
        'results':results},indent=2)+'\n')
    print(json.dumps([{'pose':r['pose_id'],'gaps_mm':{label:[q['certificate_body13_m']['surface_min_distance_m']*1000 for q in rows] for label,rows in r['models'].items()}} for r in results]))

if __name__=='__main__':main()
