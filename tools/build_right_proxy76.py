#!/usr/bin/env python3
"""Migrate the delivered left 76 shapes into a separate right candidate.

This builder neither decomposes nor repairs a mesh. Only frozen, link-local
orthogonal transforms are applied. Right mechanics, visuals, controls, contact
parameters and excludes are retained. The original 17 hand collision geoms
are replaced, never overlaid. No simulator is imported.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
LEFT = ROOT / 'mjcf/nero_dh116.xml'
RIGHT = ROOT / 'mjcf/nero_dh116_right.xml'
EXPECTED = {LEFT: 'c446124b7d938aa6e00ba5914047001de66c2a2c162e7dea1644437e5a85cb33',
            RIGHT: '739d3d29bbfd4a6468fb8b75f4de09b719c0f96abb39508959c1dfc33a84b9cc'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_mesh(path):
    """Read existing binary STL or polygonal OBJ without changing its surface."""
    path = Path(path)
    if path.suffix.lower() == '.stl':
        data = path.read_bytes()
        count, = struct.unpack_from('<I', data, 80)
        if len(data) != 84+50*count:
            raise ValueError('Expected exact binary STL length: '+str(path))
        dtype = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
        triangles = np.frombuffer(data,dtype=dtype,offset=84,count=count)['vertices'].astype(float)
        vertices, inverse = np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
        return vertices, inverse.reshape(-1,3)
    if path.suffix.lower() != '.obj':
        raise ValueError('Only the delivered OBJ/STL inputs are supported')
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        row = line.split()
        if not row: continue
        if row[0] == 'v': vertices.append([float(x) for x in row[1:4]])
        elif row[0] == 'f':
            ids = [int(x.split('/')[0]) for x in row[1:]]
            ids = [i-1 if i>0 else len(vertices)+i for i in ids]
            faces.extend([[ids[0],ids[i],ids[i+1]] for i in range(1,len(ids)-1)])
    v, f = np.asarray(vertices,dtype=float), np.asarray(faces,dtype=int)
    if v.ndim != 2 or v.shape[1] != 3 or f.ndim != 2 or f.shape[1] != 3:
        raise ValueError('Invalid delivered mesh: '+str(path))
    return v, f


def transform_mesh(vertices, faces, rotation, translation):
    """Apply a local map and reverse winding when its determinant is negative."""
    transformed = np.asarray(vertices) @ np.asarray(rotation).T + translation
    faces = np.array(faces,copy=True)
    if np.linalg.det(rotation) < 0:
        faces = faces[:,[0,2,1]]
    return transformed, faces


def quaternion_matrix(value):
    q = np.fromstring(value,sep=' ')
    q /= np.linalg.norm(q)
    w,x,y,z = q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def hand_collisions(xml):
    return {b.get('name').removeprefix('gripper0_'): (b,[g for g in b.findall('geom') if g.get('group','0')=='0'])
            for b in xml.findall('.//body') if b.get('name','').startswith('gripper0_') and b.findall('geom')}


def write_obj(path, vertices, faces, source_hash):
    with path.open('w') as stream:
        stream.write('# Existing delivered proxy; local transform only. Source SHA256 '+source_hash+'\n')
        for v in vertices: stream.write('v '+' '.join(format(x,'.17g') for x in v)+'\n')
        for f in faces: stream.write('f '+' '.join(str(int(i)+1) for i in f)+'\n')


def build(alignment_path, output):
    alignment_path, output = Path(alignment_path).resolve(), Path(output).resolve()
    if output.exists() or not output.is_relative_to(ROOT):
        raise ValueError('Use a new isolated output directory within this checkout')
    for path, expected in EXPECTED.items():
        if sha(path) != expected: raise ValueError('Frozen formal asset differs: '+str(path))
    alignment = json.loads(alignment_path.read_text())
    left, model = ET.parse(LEFT).getroot(), ET.parse(RIGHT).getroot()
    lh, rh = hand_collisions(left), hand_collisions(model)
    if len(lh)!=17 or set(lh)!=set(rh) or set(lh)!=set(alignment['links']):
        raise ValueError('Expected the same 17 named hand links')
    if sum(len(x[1]) for x in lh.values())!=76 or any(len(x[1])!=1 for x in rh.values()):
        raise ValueError('Expected delivered left76 and formal right17 collision geoms')
    # Explicitly bind the source registration to both unmodified STL inputs.
    for name,item in alignment['links'].items():
        for side in ('left','right'):
            if sha(ROOT/item[side+'_path']) != item[side+'_sha256']:
                raise ValueError('Source registration input changed: '+name)
        rotation = np.asarray(item['R'])
        if not np.allclose(rotation.T@rotation,np.eye(3),rtol=0,atol=1e-10):
            raise ValueError('Only rigid/reflected local maps are allowed')
    before = {str(p.relative_to(ROOT)):sha(p) for p in EXPECTED}
    before[str(alignment_path.relative_to(ROOT))] = sha(alignment_path)
    output.mkdir(parents=True)
    (output/'meshes').mkdir()
    (output/'alignment.json').write_bytes(alignment_path.read_bytes())
    assets = model.find('asset')
    # Rebase paths only; every original asset keeps its original bytes/binding.
    for asset in assets:
        if asset.get('file'):
            original = (RIGHT.parent/asset.get('file')).resolve()
            asset.set('file',os.path.relpath(original,output))
    left_assets = {n.get('name'):n for n in left.findall('asset/mesh')}
    records = []
    for name,(body,old_geoms) in rh.items():
        template = copy.deepcopy(old_geoms[0])
        body.remove(old_geoms[0])
        item = alignment['links'][name]
        rotation, translation = np.array(item['R']),np.array(item['t'])
        identity = np.array_equal(rotation,np.eye(3)) and np.array_equal(translation,np.zeros(3))
        for index, source_geom in enumerate(lh[name][1]):
            source_asset = left_assets[source_geom.get('mesh')]
            source_path = (LEFT.parent/source_asset.get('file')).resolve()
            before[str(source_path.relative_to(ROOT))] = sha(source_path)
            asset_name = 'gripper0_proxy76_'+name+'_'+str(index).zfill(2)
            geom = copy.deepcopy(template)
            geom.set('name',asset_name)
            geom.set('mesh',asset_name)
            asset = copy.deepcopy(source_asset)
            asset.set('name',asset_name)
            if identity:
                asset.set('file',os.path.relpath(source_path,output))
                geom.set('pos',source_geom.get('pos','0 0 0'))
                geom.set('quat',source_geom.get('quat','1 0 0 0'))
                target_path = source_path
            else:
                vertices,faces = read_mesh(source_path)
                scale = np.fromstring(source_asset.get('scale','1 1 1'),sep=' ')
                gr = quaternion_matrix(source_geom.get('quat','1 0 0 0'))
                gp = np.fromstring(source_geom.get('pos','0 0 0'),sep=' ')
                matrix = rotation@gr@np.diag(scale)
                v,f = transform_mesh(vertices,faces,matrix,rotation@gp+translation)
                target_path = output/'meshes'/(asset_name+'.obj')
                write_obj(target_path,v,f,sha(source_path))
                asset.set('file',os.path.relpath(target_path,output))
                asset.attrib.pop('scale',None)
                geom.set('pos','0 0 0')
                geom.set('quat','1 0 0 0')
            assets.append(asset)
            body.append(geom)
            records.append({'link':name,'candidate_geom':asset_name,'candidate_mesh':asset_name,
                            'left_geom':source_geom.get('name'),'left_mesh':source_asset.get('name'),
                            'left_file':str(source_path.relative_to(ROOT)),'left_sha256':sha(source_path),
                            'candidate_file':str(target_path.relative_to(ROOT)),'candidate_sha256':sha(target_path),
                            'operation':'direct_existing_mesh_reference' if identity else 'baked_local_transform',
                            'R':item['R'],'t':item['t'],'flip_winding':bool(np.linalg.det(rotation)<0),
                            'right_contact_template':dict(template.attrib)})
    model.set('model','nero_dh116_right_proxy76_migration_candidate')
    ET.indent(model,space='  ')
    target = output/'model.xml'
    ET.ElementTree(model).write(target,encoding='utf-8',xml_declaration=True)
    after = {key:sha(ROOT/key) for key in before}
    if before!=after: raise AssertionError('An existing source was modified')
    manifest = {'schema':'right-proxy76-migration-v1','baseline_commit':'a564d2841c20e6012d0ce605ea8f619ae9a219a5',
                'candidate_model':str(target.relative_to(ROOT)),'candidate_model_sha256':sha(target),
                'formal_right_model':'mjcf/nero_dh116_right.xml','formal_right_sha256':sha(RIGHT),
                'formal_left_model':'mjcf/nero_dh116.xml','formal_left_sha256':sha(LEFT),
                'builder_sha256':sha(Path(__file__)),'alignment_sha256':sha(alignment_path),
                'no_decomposition':True,'no_model_mechanics_control_or_filter_edits':True,
                'hand_collision_geoms_removed':17,'hand_collision_geoms_added':76,
                'direct_file_references':sum(r['operation']=='direct_existing_mesh_reference' for r in records),
                'transformed_files':sum(r['operation']=='baked_local_transform' for r in records),
                'source_inputs_unchanged':before==after,'input_sha256':before,'geoms':records,
                'scope':'Static migration candidate only; geometry quality and physical behavior require separate evidence; original full-range FAIL remains.'}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    return manifest


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--alignment',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args = parser.parse_args()
    result = build(args.alignment,args.output)
    print(json.dumps({k:result[k] for k in ('candidate_model','hand_collision_geoms_added',
                                          'direct_file_references','transformed_files','source_inputs_unchanged')}))
