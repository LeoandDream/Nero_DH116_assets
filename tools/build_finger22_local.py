#!/usr/bin/env python3
"""One approved four-slab RIGHT finger22 candidate; no simulation or search."""
import argparse
import copy
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import build_finger13_local as F

ROOT = F.ROOT
BASE = ROOT/'examples/right_hand/candidates/wrist_reuse/model.xml'
SOURCE = ROOT/'meshes/dh116_right/source/finger22_link.STL'
CUTS = (.0098, .0233, .032)
NAMES = ('root', 'lower_waist', 'upper_waist', 'distal')
OLD = 'gripper0_proxy76_finger22_link_00'
NEW = tuple('gripper0_finger22_local_'+str(i).zfill(2) for i in range(4))
EXPECTED = {BASE: '9604a4a6935a34496f93e223219f687b4df8b7aa83037dc208fc9f3499eb05c9',
            SOURCE: '5ebb8a2084495cb9e87b6ee531681500cfe5a37280320e97270d6e52c804081d'}


def partition(triangles):
    """Reuse exact polygon splitting; shared coplanar faces go to lower slab."""
    parts = {n: {'triangles': [], 'source_triangle_ids': [], 'area_m2': 0.} for n in NAMES}
    areas = np.zeros(len(triangles))
    for face_id, triangle in enumerate(triangles):
        for i, part in enumerate(parts.values()):
            if i and not np.any(triangle[:, 2] > CUTS[i-1]):
                continue
            polygon = triangle.copy()
            if i:
                polygon, _ = F.G._split_polygon(polygon, np.array([0., 0., -1.]), CUTS[i-1], 0.)
            if polygon is not None and i < 3:
                polygon, _ = F.G._split_polygon(polygon, np.array([0., 0., 1.]), -CUTS[i], 0.)
            if polygon is None: continue
            faces = np.array([[polygon[0], polygon[j], polygon[j+1]] for j in range(1, len(polygon)-1)])
            if not len(faces): continue
            a = F.G._areas(faces)
            faces = faces[a > 0]
            if not len(faces): continue
            area = float(a[a > 0].sum())
            part['triangles'].extend(faces)
            part['source_triangle_ids'].append(face_id)
            part['area_m2'] += area
            areas[face_id] += area
    original = F.G._areas(triangles)
    if not np.allclose(areas, original, rtol=1e-12, atol=1e-22):
        raise AssertionError('Lost or duplicated source area')
    return parts, {'source_triangle_count': len(triangles), 'original_area_m2': float(original.sum()),
        'retained_area_m2': float(areas.sum()), 'maximum_per_source_triangle_area_difference_m2': float(abs(areas-original).max()),
        'all_source_triangles_retained': bool(np.all((areas > 0) | (original == 0))),
        'plane_clipping_tolerance_m': 0., 'area_comparison_rtol': 1e-12, 'area_comparison_atol_m2': 1e-22,
        'coplanar_face_assignment': 'lower Z slab only'}


def build(output):
    output = output.resolve()
    if output.exists() or not output.is_relative_to(ROOT):
        raise ValueError('New output inside checkout required; never overwrite/rebuild')
    for p, h in EXPECTED.items():
        if F.existing.sha(p) != h: raise ValueError('Frozen input changed: '+str(p))
    xml = ET.parse(BASE).getroot()
    hands = F.existing.hand_collisions(xml)
    assert sum(len(gs) for _, gs in hands.values()) == 78
    body, geoms = hands['finger22_link']
    assert len(geoms) == 1 and geoms[0].get('name') == OLD
    template = copy.deepcopy(geoms[0])
    assert template.get('pos','0 0 0') == '0 0 0' and template.get('quat','1 0 0 0') == '1 0 0 0'
    vertices, faces = F.existing.read_mesh(SOURCE)
    parts, coverage = partition(vertices[faces])
    output.mkdir(parents=True)
    (output/'meshes').mkdir()
    # Freeze cuts, exact source-face IDs and geom mapping BEFORE convexification.
    registration = {'baseline_commit': '360e53a023624e96bd88aef549927655b5034445',
        'baseline_model_sha256': F.existing.sha(BASE), 'source_sha256': F.existing.sha(SOURCE),
        'cuts_local_z_m': CUTS, 'old_geom': OLD, 'new_geoms': NEW,
        'source_triangle_ids_zero_based': {n: p['source_triangle_ids'] for n,p in parts.items()},
        'source_surface_partition': coverage, 'attempts_allowed': 1, 'physics_steps': 0}
    (output/'registered.json').write_text(json.dumps(registration, indent=2)+'\n')
    assets = xml.find('asset')
    for asset in assets:
        if asset.get('file'):
            asset.set('file', os.path.relpath((BASE.parent/asset.get('file')).resolve(), output))
    body.remove(geoms[0])
    records = []
    for name, geom_name, part in zip(NAMES, NEW, parts.values()):
        v, f, volume = F.convex_mesh(part['triangles'])
        path = output/'meshes'/(name+'.obj')
        F.existing.write_obj(path, v, f, F.existing.sha(SOURCE))
        lines = path.read_text().splitlines()
        lines[0] = '# RIGHT finger22 fixed Z source slab '+name+'; source '+F.existing.sha(SOURCE)
        path.write_text('\n'.join(lines)+'\n')
        ET.SubElement(assets, 'mesh', name=geom_name, file=os.path.relpath(path, output))
        geom = copy.deepcopy(template)
        geom.set('name', geom_name); geom.set('mesh', geom_name); body.append(geom)
        records.append({'name': name, 'geom': geom_name, 'file': str(path.relative_to(ROOT)),
            'sha256': F.existing.sha(path), 'clipped_triangle_count': len(part['triangles']),
            'clipped_source_area_m2': part['area_m2'], 'vertices': len(v), 'faces': len(f), 'volume_m3': volume})
    xml.set('model', 'nero_dh116_right_finger22_four_static_candidate')
    ET.indent(xml, space='  ')
    ET.ElementTree(xml).write(output/'model.xml', encoding='utf-8', xml_declaration=True)
    manifest = {**registration, 'candidate_model_sha256': F.existing.sha(output/'model.xml'),
        'builder_sha256': F.existing.sha(Path(__file__)), 'pieces': records,
        'removed_geom_template': dict(template.attrib), 'frozen_other_hand_count': 77,
        'frozen_wrist_count': 306, 'hand_collision_geoms': 81, 'status': 'STATIC_CANDIDATE_NOT_RELEASED'}
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    m = build(parser.parse_args().output)
    print(json.dumps({k: m[k] for k in ('candidate_model_sha256', 'hand_collision_geoms', 'source_surface_partition')}))
