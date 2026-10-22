#!/usr/bin/env python3
"""One source-driven finger13 three-piece repair; never run a simulator.

Freeze the other 75 proxy76 shapes. Split every original RIGHT source triangle
at its native CAD y=+/-5 mm planes, retain all pieces, and convexify each slab.
The coplanar hinge-ear faces belong to the central piece, not both wings.
There is no geometric shrink, task-pose fitting, parameter search or fallback.
"""
import argparse
import copy
import importlib.util
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull

import build_right_proxy76 as existing

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT/'examples/right_hand/candidates/proxy76/model.xml'
SOURCE = ROOT/'meshes/dh116_right/source/finger13_link.STL'
PLANE = 0.004999999888241291  # Exact float32 CAD plane in frozen source STL.
EXPECTED = {OLD: '56949ed130a54b1d8b5fbd76e51b1262b45b7346e29130e42e8db156dd7d15d6',
            SOURCE: '594501a21c86d1d19b73cf17572b7a04ae488c5fee4b623e25c618dd15eb3419'}
_spec = importlib.util.spec_from_file_location('finger13_existing_geometry',
                    ROOT/'examples/right_hand/proxy76_geometry.py')
G = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(G)


def partition(triangles):
    """Exact plane intersections, original faces retained with provenance."""
    parts = {name: {'triangles': [], 'source_triangle_ids': [], 'area_m2': 0.}
             for name in ('negative_wing', 'central', 'positive_wing')}
    area_by_source = np.zeros(len(triangles))
    for index, triangle in enumerate(triangles):
        for name, part in parts.items():
            if name == 'negative_wing':
                if not np.any(triangle[:, 1] < -PLANE):
                    continue
                polygon, _ = G._split_polygon(triangle, np.array([0., 1., 0.]), PLANE, 0.)
            elif name == 'positive_wing':
                if not np.any(triangle[:, 1] > PLANE):
                    continue
                polygon, _ = G._split_polygon(triangle, np.array([0., -1., 0.]), PLANE, 0.)
            else:
                polygon, _ = G._split_polygon(triangle, np.array([0., 1., 0.]), -PLANE, 0.)
                if polygon is not None:
                    polygon, _ = G._split_polygon(polygon, np.array([0., -1., 0.]), -PLANE, 0.)
            if polygon is None:
                continue
            faces = np.asarray([[polygon[0], polygon[j], polygon[j+1]]
                                for j in range(1, len(polygon)-1)])
            areas = G._areas(faces)
            faces = faces[areas > 0]
            area = float(areas[areas > 0].sum())
            if not len(faces):
                continue
            part['triangles'].extend(faces)
            part['source_triangle_ids'].append(index)
            part['area_m2'] += area
            area_by_source[index] += area
    original_areas = G._areas(triangles)
    # Numerical area conservation, never a physical acceptance tolerance.
    if not np.allclose(area_by_source, original_areas, rtol=1e-12, atol=1e-22):
        raise AssertionError('Source triangle area was lost or duplicated')
    return parts, {'source_triangle_count': len(triangles),
        'original_area_m2': float(original_areas.sum()),
        'retained_area_m2': float(area_by_source.sum()),
        'maximum_per_source_triangle_area_difference_m2': float(np.max(abs(area_by_source-original_areas))),
        'all_source_triangles_retained': bool(np.all((area_by_source > 0) | (original_areas == 0))),
        'plane_clipping_tolerance_m': 0., 'area_comparison_rtol': 1e-12,
        'area_comparison_atol_m2': 1e-22}


def convex_mesh(triangles):
    points = np.unique(np.asarray(triangles).reshape(-1, 3), axis=0)
    hull = ConvexHull(points)
    faces = hull.simplices.copy()
    normals = np.cross(points[faces[:, 1]]-points[faces[:, 0]],
                       points[faces[:, 2]]-points[faces[:, 0]])
    reverse = np.einsum('ij,ij->i', normals, hull.equations[:, :3]) < 0
    faces[reverse] = faces[reverse][:, [0, 2, 1]]
    used, inverse = np.unique(faces, return_inverse=True)
    return points[used], inverse.reshape(-1, 3), float(hull.volume)


def build(output):
    output = Path(output).resolve()
    if output.exists() or not output.is_relative_to(ROOT):
        raise ValueError('Use a new output directory inside this checkout')
    for path, sha in EXPECTED.items():
        if existing.sha(path) != sha:
            raise ValueError('Frozen input changed: '+str(path))
    old_manifest_path = OLD.parent/'manifest.json'
    old_manifest = json.loads(old_manifest_path.read_text())
    model = ET.parse(OLD).getroot()
    hands = existing.hand_collisions(model)
    if sum(len(g) for _, g in hands.values()) != 76:
        raise ValueError('Expected unmodified proxy76 baseline')
    body, old_geoms = hands['finger13_link']
    if len(old_geoms) != 1:
        raise ValueError('Only the original single finger13 geom may be replaced')
    template = copy.deepcopy(old_geoms[0])
    if template.get('pos', '0 0 0') != '0 0 0' or template.get('quat', '1 0 0 0') != '1 0 0 0':
        raise ValueError('Expected original link-local source coordinates')
    vertices, faces = existing.read_mesh(SOURCE)
    if not all(np.any(vertices[:, 1] == sign*PLANE) for sign in (-1, 1)):
        raise ValueError('Frozen native source CAD planes absent')
    parts, coverage = partition(vertices[faces])
    output.mkdir(parents=True)
    (output/'meshes').mkdir()
    assets = model.find('asset')
    for asset in assets:
        if asset.get('file'):
            asset.set('file', os.path.relpath((OLD.parent/asset.get('file')).resolve(), output))
    body.remove(old_geoms[0])
    records = [copy.deepcopy(r) for r in old_manifest['geoms'] if r['link'] != 'finger13_link']
    if len(records) != 75:
        raise AssertionError('Expected exactly 75 frozen hand geoms')
    new_records = []
    for index, (name, part) in enumerate(parts.items()):
        v, f, volume = convex_mesh(part['triangles'])
        geom_name = 'gripper0_finger13_local_'+str(index).zfill(2)
        path = output/'meshes'/(name+'.obj')
        existing.write_obj(path, v, f, existing.sha(SOURCE))
        # Clarify the reused writer's first-line migration-only label.
        lines = path.read_text().splitlines()
        lines[0] = '# RIGHT finger13 source partition '+name+'; SHA256 '+existing.sha(SOURCE)
        path.write_text('\n'.join(lines)+'\n')
        ET.SubElement(assets, 'mesh', name=geom_name, file=os.path.relpath(path, output))
        geom = copy.deepcopy(template)
        geom.set('name', geom_name)
        geom.set('mesh', geom_name)
        body.append(geom)
        new_records.append({'link':'finger13_link', 'part':name,
            'candidate_geom':geom_name, 'candidate_mesh':geom_name,
            'candidate_file':str(path.relative_to(ROOT)), 'candidate_sha256':existing.sha(path),
            'operation':'convex_hull_of_complete_right_source_slab',
            'source_right_file':str(SOURCE.relative_to(ROOT)), 'source_right_sha256':existing.sha(SOURCE),
            'source_triangle_ids_zero_based':part['source_triangle_ids'],
            'clipped_triangle_count':len(part['triangles']), 'clipped_source_area_m2':part['area_m2'],
            'convex_vertices':len(v), 'convex_faces':len(f), 'convex_volume_m3':volume,
            'right_contact_template':dict(template.attrib)})
    model.set('model','nero_dh116_right_finger13_local_candidate')
    ET.indent(model, space='  ')
    path = output/'model.xml'
    ET.ElementTree(model).write(path, encoding='utf-8', xml_declaration=True)
    manifest = {'schema':'right-finger13-local-v1',
        'baseline_commit':'c61f97777c42ea421d0460252abc7dbeb48b282b',
        'baseline_model':str(OLD.relative_to(ROOT)), 'baseline_model_sha256':existing.sha(OLD),
        'candidate_model':str(path.relative_to(ROOT)), 'candidate_model_sha256':existing.sha(path),
        'builder_sha256':existing.sha(Path(__file__)), 'right_source_sha256':existing.sha(SOURCE),
        'repair':'One fixed three-slab partition at native right-source CAD planes; no scan or shrink.',
        'plane_y_abs_m':PLANE, 'coplanar_faces':'central only', 'source_surface_partition':coverage,
        'frozen_other_hand_geom_count':75, 'removed_finger13_geom':dict(template.attrib),
        'new_finger13_geom_count':len(new_records), 'hand_collision_geoms':75+len(new_records),
        'frozen_geom_names_and_mesh_sha256':{r['candidate_geom']:{'mesh':r['candidate_mesh'],
            'file':r['candidate_file'],'sha256':r['candidate_sha256']} for r in records},
        'geoms':records+new_records, 'physics_steps':0,
        'scope':'Static local candidate; no dynamic or global right-hand acceptance.'}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    for path, sha in EXPECTED.items():
        if existing.sha(path) != sha:
            raise AssertionError('Input modified')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = build(args.output)
    print(json.dumps({key:result[key] for key in ('candidate_model','candidate_model_sha256',
        'frozen_other_hand_geom_count','new_finger13_geom_count','hand_collision_geoms',
        'source_surface_partition','physics_steps')}))
