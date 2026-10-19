#!/usr/bin/env python3
"""Exactly two continuous source-surface queries at the candidate LP center."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np


ROOT = next(p for p in Path(__file__).resolve().parents
            if (p / 'mjcf/nero_dh116_right.xml').is_file())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--common-ball', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    proof = json.loads(args.common_ball.read_text())
    geometry_path = ROOT/'examples/right_hand/proxy76_geometry.py'
    spec = importlib.util.spec_from_file_location('proxy76_geometry_center_query', geometry_path)
    geometry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(geometry)
    stl = geometry._geometry()
    transforms = proof['body_world_transforms']
    candidate = proof['results']['proxy76']
    plane_normals = np.array(candidate['certificate']['normals'])
    plane_offsets = np.array(candidate['certificate']['offsets_mm'])
    conditioned_center = np.array(candidate['common_ball_center_conditioned_mm'])
    origin_mm = np.array(candidate['conditioning_origin_body13_mm'])
    split = candidate['hulls'][0]['hull_halfspaces']
    center13 = np.array(candidate['common_ball_center_body13_m'])
    r13, p13 = np.array(transforms['finger13_link']['rotation']), np.array(transforms['finger13_link']['translation_m'])
    center_world = r13 @ center13 + p13
    inputs = [args.common_ball.resolve(), Path(__file__).resolve(), geometry_path,
              ROOT/'examples/right_hand/evidence/d3_geometry/triangle/surface_geometry.py',
              *[ROOT/'meshes/dh116_right/source'/f'{link}.STL'
                for link in ('finger13_link', 'finger42_link')],
              *[ROOT/hull['raw_stl_path'] for hull in candidate['hulls']]]
    before = {str(p): sha(p) for p in inputs}
    results = {}
    for index, link in enumerate(('finger13_link', 'finger42_link')):
        rotation, translation = np.array(transforms[link]['rotation']), np.array(transforms[link]['translation_m'])
        local = center13 if index == 0 else rotation.T @ (center_world-translation)
        source = ROOT/'meshes/dh116_right/source'/f'{link}.STL'
        triangles = stl.read_stl(source)
        distance, nearest, face = geometry.TriangleSurface(triangles).nearest([local])
        indices = np.arange(0, split) if index == 0 else np.arange(split, len(plane_offsets))
        margins = -(plane_normals[indices] @ conditioned_center + plane_offsets[indices])
        active = int(indices[np.argmin(margins)])
        normal = plane_normals[active]
        offset = plane_offsets[active] - normal @ origin_mm
        if index == 1:
            relative_rotation = r13.T @ rotation
            relative_translation_mm = r13.T @ (translation-p13)*1000
            offset += normal @ relative_translation_mm
            normal = relative_rotation.T @ normal
        proxy_triangles_mm = stl.read_stl(ROOT/candidate['hulls'][index]['raw_stl_path'])*1000
        proxy_vertices_mm = np.unique(proxy_triangles_mm.reshape(-1, 3), axis=0)
        supporting = proxy_vertices_mm[np.abs(proxy_vertices_mm @ normal+offset) < 1e-7]
        coplanar_faces = np.all(np.abs(proxy_triangles_mm @ normal+offset) < 1e-7, axis=1)
        active_plane = {'normal_in_body_local': normal.tolist(), 'offset_mm': float(offset),
            'global_hull_plane_index': active,
            'supporting_proxy_raw_vertices_body_local_mm': supporting.tolist(),
            'supporting_raw_vertex_count': len(supporting),
            'proxy_raw_triangles_coplanar_with_this_hull_plane': int(coplanar_faces.sum()),
            'max_support_vertex_pair_span_mm': float(np.max(np.linalg.norm(
                supporting[:, None]-supporting[None, :], axis=-1))),
            'support_match_tolerance_mm': 1e-7}
        # This unsigned query does not classify source-solid interior/exterior.
        results[link] = {'center_body_local_m': local.tolist(),
            'unsigned_center_to_right_source_surface_m': float(distance[0]),
            'nearest_source_point_body_local_m': nearest[0].tolist(),
            'nearest_source_face_id_zero_based': int(face[0]),
            'nearest_source_triangle_body_local_m': triangles[face[0]].tolist(),
            'active_proxy_hull_plane': active_plane,
            'own_proxy_hull_min_center_face_margin_mm': candidate['per_hull_min_center_margin_mm'][index],
            'source_stl_path': str(source.relative_to(ROOT)), 'source_stl_sha256': sha(source),
            'source_solid_interior_classification': 'UNKNOWN_NO_WATERTIGHT_SOLID_ASSUMPTION'}
    after = {str(p): sha(p) for p in inputs}
    if before != after or 'mujoco' in sys.modules:
        raise ValueError('Inputs changed or MuJoCo imported')
    result = {'schema': 'd3-6451-candidate-common-ball-center-source-surface-v1',
        'pose_id': proof['pose_id'], 'center_world_m': center_world.tolist(),
        'method': 'Candidate proxy76 LP center, cached exact-pose body transforms; two TriangleSurface.nearest continuous point-to-original-right-STL queries. No sampling and no new FK.',
        'query_count': 2, 'physics_steps': 0, 'mujoco_imports': 0, 'mujoco_api_calls': 0,
        'input_sha256_before': before, 'input_sha256_after': after,
        'inputs_unchanged': True, 'results': results,
        'interpretation_limit': 'A proxy-interior center separated from a source surface diagnoses proxy/source geometric mismatch at this location. Unsigned distances cannot identify source-solid inside/outside, prove a void, or alone choose which link must be repaired.'}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: {'center_to_source_mm': value['unsigned_center_to_right_source_surface_m']*1000,
                           'center_to_proxy_hull_boundary_mm': value['own_proxy_hull_min_center_face_margin_mm']}
                      for key, value in results.items()}))


if __name__ == '__main__':
    main()
