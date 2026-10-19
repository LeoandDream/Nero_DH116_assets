#!/usr/bin/env python3
"""One fixed source-driven local repair of two existing arm/flange proxies.

No simulator, decomposition search, shrink, source mesh edits, or filtering.
Reuse the delivered clipper/hull/export functions. Partition the brace at two
native CAD y planes; separate the flange's original disconnected components.
"""
import argparse
import copy
import importlib.util
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

import build_finger13_local as local
import build_right_proxy76 as existing

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / 'examples/right_hand/candidates/finger13_local/model.xml'
PLANES = (-0.02290000021457672, -0.015800001099705696)
TARGETS = {
    'robot0_candidate_nero_link5_4': ('robot0_link5', 'meshes/nero/source/link5-dc497f5a.stl',
        'dc497f5a9bab2d0d314250134d4ec86617abea7205d122db41b765acdb86ae7e'),
    'robot0_source_flange_part_2': ('robot0_gripper_flange', 'meshes/nero/source/gripper_flange-186697a7.stl',
        '186697a73e7cafd9fd41d96f1eea9b34b444c058b9561fb2e3ea439c835ae2e8'),
}
EXPECTED_MODEL = 'b27f4f86fbaeacd2bb925463f39f67181ef7db12963c4d9f7fc01797a7924d1b'


def source_groups(source, old_vertices):
    """Match every old proxy input vertex exactly to its unchanged STL source."""
    vertices, faces = existing.read_mesh(source)
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(len(vertices), len(vertices)))
    count, labels = connected_components(graph, directed=False)
    distances, indices = cKDTree(vertices).query(old_vertices)
    if np.any(distances != 0) or len(np.unique(indices)) != len(old_vertices):
        raise ValueError('Old proxy input vertices no longer match source coordinates exactly')
    groups = {int(i): indices[labels[indices] == i] for i in np.unique(labels[indices])}
    represented = np.zeros(len(vertices), dtype=bool)
    represented[indices] = True
    covered_faces = np.flatnonzero(np.all(represented[faces], axis=1))
    return vertices, faces, labels, groups, covered_faces, count


def partition_brace(triangles):
    """Retain every original surface fragment once, including coplanar faces."""
    parts = {name: {'triangles': [], 'source_triangle_ids': []}
             for name in ('low_y', 'middle_y', 'high_y')}
    areas = np.zeros(len(triangles))
    for i, triangle in enumerate(triangles):
        for name, part in parts.items():
            if name == 'low_y':
                if not np.any(triangle[:, 1] < PLANES[0]):
                    continue
                polygon, _ = local.G._split_polygon(triangle, np.array([0., 1., 0.]), -PLANES[0], 0.)
            elif name == 'high_y':
                if not np.any(triangle[:, 1] > PLANES[1]):
                    continue
                polygon, _ = local.G._split_polygon(triangle, np.array([0., -1., 0.]), PLANES[1], 0.)
            else:
                polygon, _ = local.G._split_polygon(triangle, np.array([0., -1., 0.]), PLANES[0], 0.)
                if polygon is not None:
                    polygon, _ = local.G._split_polygon(polygon, np.array([0., 1., 0.]), -PLANES[1], 0.)
            if polygon is None:
                continue
            fragments = np.array([[polygon[0], polygon[j], polygon[j+1]] for j in range(1, len(polygon)-1)])
            area = local.G._areas(fragments)
            fragments = fragments[area > 0]
            if not len(fragments):
                continue
            part['triangles'].extend(fragments)
            part['source_triangle_ids'].append(i)
            areas[i] += area[area > 0].sum()
    original = local.G._areas(triangles)
    if not np.allclose(areas, original, rtol=1e-12, atol=1e-22):
        raise AssertionError('Source surface area lost or duplicated')
    return parts, {'source_triangles': len(triangles), 'source_area_m2': float(original.sum()),
                   'retained_area_m2': float(areas.sum()),
                   'max_per_source_area_difference_m2': float(np.max(abs(areas-original))),
                   'all_nonzero_area_source_faces_retained': bool(np.all((areas > 0) | (original == 0))),
                   'numeric_area_check': {'rtol': 1e-12, 'atol_m2': 1e-22}, 'clipping_tolerance_m': 0.}


def build(output):
    output = Path(output).resolve()
    if output.exists() or not output.is_relative_to(ROOT):
        raise ValueError('Use a new candidate directory in this checkout')
    if existing.sha(OLD) != EXPECTED_MODEL:
        raise ValueError('Frozen 78-hand model changed')
    model = ET.parse(OLD).getroot()
    assets = model.find('asset')
    declarations = {a.get('name'): a for a in assets if a.tag == 'mesh'}
    frozen, removed, new_records, source_records = {}, [], [], {}
    for body in model.findall('.//body'):
        for geom in body.findall('geom'):
            if geom.get('group', '0') != '0':
                continue
            if geom.get('name') in TARGETS:
                continue
            name = geom.get('mesh')
            if name is None:
                frozen[geom.get('name')] = {'mesh': None, 'file': None, 'sha256': None,
                                           'geom_attributes': dict(geom.attrib)}
            else:
                path = (OLD.parent / declarations[name].get('file')).resolve()
                frozen[geom.get('name')] = {'mesh': name, 'file': str(path.relative_to(ROOT)), 'sha256': existing.sha(path)}
    if len(frozen) != 138:
        raise ValueError('Expected the other 138 unchanged enabled robot collision geoms')
    # Rebase references without copying or overwriting any inherited resource.
    for asset in assets:
        if asset.get('file'):
            asset.set('file', os.path.relpath((OLD.parent / asset.get('file')).resolve(), output))
    output.mkdir(parents=True)
    (output / 'meshes').mkdir()
    for old_name, (body_name, source_rel, expected_source) in TARGETS.items():
        source = ROOT / source_rel
        if existing.sha(source) != expected_source:
            raise ValueError('Frozen manufacturer source changed')
        body = model.find(f'.//body[@name="{body_name}"]')
        geom = body.find(f'geom[@name="{old_name}"]')
        template = dict(geom.attrib)
        if geom.get('pos', '0 0 0') != '0 0 0' or geom.get('quat', '1 0 0 0') != '1 0 0 0':
            raise ValueError('Expected identity old proxy link-local pose')
        old_asset = declarations[geom.get('mesh')]
        old_path = (output / old_asset.get('file')).resolve()
        old_vertices, _ = existing.read_mesh(old_path)
        v, f, labels, groups, represented_faces, total_components = source_groups(source, old_vertices)
        info = {'source': source_rel, 'source_sha256': expected_source,
                'old_proxy_file': str(old_path.relative_to(ROOT)), 'old_proxy_sha256': existing.sha(old_path),
                'old_input_vertices': len(old_vertices), 'source_connected_components': total_components,
                'source_component_vertex_counts': {str(k): len(ids) for k, ids in groups.items()},
                'source_faces_with_all_vertices_in_old_proxy_input': represented_faces.tolist()}
        if body_name == 'robot0_link5':
            if len(groups) != 1 or len(old_vertices) != 1275:
                raise ValueError('Expected the one frozen brace component')
            label = next(iter(groups))
            face_ids = np.flatnonzero(np.all(labels[f] == label, axis=1))
            if len(groups[label]) != np.sum(labels == label) or len(face_ids) != 2570:
                raise ValueError('Brace source component incomplete')
            for plane in PLANES:
                if not np.any(v[groups[label], 1] == plane):
                    raise ValueError('Native CAD partition plane absent')
            parts, coverage = partition_brace(v[f[face_ids]])
            constructions = [(name, np.asarray(part['triangles']),
                              face_ids[part['source_triangle_ids']].tolist()) for name, part in parts.items()]
            info.update(native_y_planes_m=list(PLANES), source_partition=coverage)
        else:
            if sorted(len(ids) for ids in groups.values()) != [826, 2418]:
                raise ValueError('Expected the two original flange source components')
            constructions = []
            for label, ids in groups.items():
                # Degenerate triangles carry each unchanged input point to the
                # reused hull routine; they are not source faces or new surfaces.
                points = v[ids]
                constructions.append((f'source_component_{label}', np.repeat(points[:, None, :], 3, axis=1),
                                      represented_faces[np.all(labels[f[represented_faces]] == label, axis=1)].tolist()))
            info['partition'] = 'Separate existing proxy source points by factory vertex-connected component; no source point removed or moved'
        removed.append(template)
        body.remove(geom)
        for index, (part_name, triangles, face_ids) in enumerate(constructions):
            vertices, faces, volume = local.convex_mesh(triangles)
            new_name = ('robot0_link5_local_' if body_name == 'robot0_link5' else 'robot0_flange_local_') + f'{index:02}'
            target = output / 'meshes' / (new_name + '.obj')
            existing.write_obj(target, vertices, faces, expected_source)
            text = target.read_text().splitlines()
            text[0] = '# Fixed NERO/flange local source partition; SHA256 ' + expected_source
            target.write_text('\n'.join(text) + '\n')
            ET.SubElement(assets, 'mesh', name=new_name, file=os.path.relpath(target, output))
            attributes = {**template, 'name': new_name, 'mesh': new_name}
            ET.SubElement(body, 'geom', **attributes)
            new_records.append({'old_geom': old_name, 'parent_body': body_name, 'new_geom': new_name,
                'mesh': new_name, 'new_file': str(target.relative_to(ROOT)), 'sha256': existing.sha(target),
                'partition': part_name, 'source_file': source_rel, 'source_sha256': expected_source,
                'source_triangle_ids_zero_based': face_ids, 'vertices': len(vertices), 'faces': len(faces),
                'convex_volume_m3': volume})
        source_records[old_name] = info
    model.set('model', 'nero_dh116_right_link5_flange_local_diagnostic')
    ET.indent(model, space='  ')
    target = output / 'model.xml'
    ET.ElementTree(model).write(target, encoding='utf-8', xml_declaration=True)
    manifest = {'schema': 'right-link5-flange-local-v1',
        'baseline_commit': '849f10a51ece64e5de2236f6b6d7c7f11f783ff8',
        'authorization': 'https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6263431193',
        'baseline_model': str(OLD.relative_to(ROOT)), 'baseline_model_sha256': EXPECTED_MODEL,
        'candidate_model': str(target.relative_to(ROOT)), 'candidate_model_sha256': existing.sha(target),
        'builder_sha256': existing.sha(Path(__file__).resolve()),
        'construction': 'ONE fixed candidate: brace three native-y slabs and flange two existing source components; no geometric scan/shrink',
        'removed_geoms': removed, 'new_geoms': new_records,
        'frozen_geom_names_and_mesh_sha256': frozen, 'source_regions': source_records,
        'hand_collision_parts': 78, 'nero_collision_parts': 53, 'flange_collision_parts': 9,
        'mount_collision_geoms': 3, 'total_collision_geoms': 143,
        'new_physics_steps': 0, 'acceptance': 'NOT_YET_STATICALLY_VALIDATED; original dynamic FAIL and grasp UNKNOWN unchanged'}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = build(args.output)
    print(json.dumps({key: result[key] for key in ('candidate_model', 'candidate_model_sha256', 'nero_collision_parts', 'flange_collision_parts', 'total_collision_geoms')}))
