#!/usr/bin/env python3
"""Static continuous-surface comparison of a local finger13 repair.

Reuse the delivered proxy76 distance/BVH/convex-union implementation. No
simulator is imported. Surface coverage is checked on every source triangle
by convex halfspaces, rather than inferred from sampled points or vertices.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

for _variable in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
                  'BLIS_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[_variable] = '1'

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
OLD = HERE / 'candidates/proxy76/model.xml'


def _module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


V = _module('finger13_original_validator', HERE / 'proxy76_validate.py')
G = V.GEOMETRY


def _hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inputs(candidate):
    """Load only actual right source and finger13 collision mesh bindings."""
    formal = ET.parse(V.FORMAL_RIGHT).getroot()
    source_body = V.BUILDER.hand_collisions(formal)['finger13_link'][0]
    source_assets = {a.get('name'): a for a in formal.findall('asset/mesh')}
    visuals = [g for g in source_body.findall('geom') if g.get('group') == '1']
    if len(visuals) != 1:
        raise ValueError('Expected one actual source visual for finger13')
    source = V._binding(V.FORMAL_RIGHT, formal, source_assets, visuals[0])
    xml = ET.parse(candidate).getroot()
    assets = {a.get('name'): a for a in xml.findall('asset/mesh')}
    body, geoms = V.BUILDER.hand_collisions(xml)['finger13_link']
    candidate_visuals = [g for g in body.findall('geom') if g.get('group') == '1']
    if len(candidate_visuals) != 1:
        raise ValueError('Expected unchanged source visual binding')
    visual = V._binding(candidate, xml, assets, candidate_visuals[0])
    if any(source[k] != visual[k] for k in ('file', 'scale', 'pos', 'quat')):
        raise ValueError('Source visual changed')
    parts = [V._binding(candidate, xml, assets, geom) for geom in geoms]
    if not parts:
        raise ValueError('No finger13 collision geoms')
    return source, parts


def _normals(triangles):
    values = np.cross(triangles[:, 1]-triangles[:, 0],
                      triangles[:, 2]-triangles[:, 0])
    lengths = np.linalg.norm(values, axis=1)
    return np.divide(values, lengths[:, None], out=np.zeros_like(values),
                     where=lengths[:, None] > 0), lengths


def normals(source, union, count=4096, seed=763):
    """Nearest continuous face normals; ties/edges do not define smooth normals."""
    target_source = G.TriangleSurface(source)
    source_normals, source_lengths = _normals(source)
    proxy_normals, proxy_lengths = _normals(union.triangles)
    report = {}
    for name, triangles, own_normals, own_lengths, destination, target_normals, other_lengths, rng in (
            ('source_to_proxy', source, source_normals, source_lengths,
             union.surface, proxy_normals, proxy_lengths, seed),
            ('proxy_to_source', union.triangles, proxy_normals, proxy_lengths,
             target_source, source_normals, source_lengths, seed+1)):
        points, ids = G._sample_surface(triangles, count, rng)
        distances, nearest, faces = destination.nearest(points)
        valid = (own_lengths[ids] > 0) & (other_lengths[faces] > 0)
        dots = np.einsum('ij,ij->i', own_normals[ids], target_normals[faces])
        oriented = np.degrees(np.arccos(np.clip(dots, -1, 1)))
        unoriented = np.degrees(np.arccos(np.clip(np.abs(dots), 0, 1)))
        k = int(np.flatnonzero(valid)[np.argmax(unoriented[valid])])
        report[name] = {
            'valid_sample_count': int(valid.sum()),
            'acute_angle_deg': {key: float(np.quantile(unoriented[valid], q))
                                for key, q in [('p50', .5), ('p95', .95), ('p99', .99), ('max', 1)]},
            'oriented_angle_deg': {key: float(np.quantile(oriented[valid], q))
                                   for key, q in [('p50', .5), ('p95', .95), ('p99', .99), ('max', 1)]},
            'max_acute_witness': {'point_m': points[k].tolist(), 'nearest_m': nearest[k].tolist(),
                'distance_m': float(distances[k]), 'query_triangle_id': int(ids[k]),
                'nearest_face_id': int(faces[k]), 'own_normal': own_normals[ids[k]].tolist(),
                'nearest_normal': target_normals[faces[k]].tolist(),
                'acute_angle_deg': float(unoriented[k]), 'oriented_angle_deg': float(oriented[k])}}
    report['limits'] = ['Original source STL is treated as a surface; its orientation/solid interior is not assumed.',
                        'Acute angles ignore source normal winding; oriented angles are included without acceptance claims.',
                        'Nearest faces at edges, corners and ties are nonsmooth and can have large angles.',
                        'These deterministic finite samples do not certify a global normal-angle maximum.']
    return report


def coverage(source, union, reference_union=None):
    """Subtract convex solids from every source triangle; record uncovered area.

    A triangle with all three vertices in the same convex part is wholly
    covered by convexity. All other triangles are continuously clipped against
    every actual convex part, using the existing numerical plane tolerance.
    This is a floating-point geometric certificate, not exact arithmetic.
    """
    count = len(source)
    areas = G._areas(source)
    fully_covered = np.zeros(count, dtype=bool)
    for hull in union.hulls:
        equations = hull['equations']
        for start in range(0, count, 256):
            values = (source[start:start+256].reshape(-1, 3) @ equations[:, :3].T + equations[:, 3])
            covered = np.all(values <= G.CLIP_TOL_M, axis=1).reshape(-1, 3).all(axis=1)
            fully_covered[start:start+len(covered)] |= covered
    uncovered_area = np.zeros(count)
    newly_uncovered_area = np.zeros(count)
    pieces_count = 0
    witnesses = []
    if reference_union is not None and len(reference_union.hulls) != 1:
        raise ValueError('This local comparison expects the frozen single-convex old finger13')
    for face_id in np.flatnonzero(~fully_covered):
        pieces = [source[face_id]]
        for hull in union.hulls:
            pieces = [part for polygon in pieces
                      for part in G._subtract_convex(polygon, hull['equations'], G.CLIP_TOL_M)]
            if not pieces:
                break
        for polygon in pieces:
            for k in range(1, len(polygon)-1):
                triangle = np.array([polygon[0], polygon[k], polygon[k+1]])
                area = float(G._areas(triangle[None])[0])
                if area > G.CLIP_TOL_M**2:
                    uncovered_area[face_id] += area
                    pieces_count += 1
                    witnesses.append((area, int(face_id), triangle))
                    if reference_union is not None:
                        # Intersect a newly uncovered continuous triangle with
                        # the old solid. This separates inherited source gaps
                        # from actual coverage removed by the local repair.
                        remaining = triangle
                        equations = reference_union.hulls[0]['equations']
                        values = triangle @ equations[:, :3].T + equations[:, 3]
                        if np.any(np.all(values > G.CLIP_TOL_M, axis=0)):
                            remaining = None
                        elif not np.all(values <= G.CLIP_TOL_M):
                            for equation in equations:
                                remaining, _ = G._split_polygon(remaining, equation[:3], equation[3], G.CLIP_TOL_M)
                                if remaining is None:
                                    break
                        if remaining is not None:
                            for j in range(1, len(remaining)-1):
                                fragment = np.array([remaining[0], remaining[j], remaining[j+1]])
                                newly_uncovered_area[face_id] += G._areas(fragment[None])[0]
    witnesses.sort(key=lambda item: item[0], reverse=True)
    records = []
    for area, face_id, triangle in witnesses[:10]:
        center = triangle.mean(axis=0)
        distance, nearest, faces = union.surface.nearest(center[None])
        records.append({'source_triangle_id': face_id, 'uncovered_triangle_vertices_m': triangle.tolist(),
                        'uncovered_area_m2': area, 'centroid_m': center.tolist(),
                        'centroid_membership_margin_m': float(union.membership_margin(center[None])[0]),
                        'centroid_distance_to_union_m': float(distance[0]),
                        'nearest_proxy_point_m': nearest[0].tolist(), 'nearest_proxy_triangle_id': int(faces[0])})
    result = {'source_triangle_count': count, 'source_area_m2': float(areas.sum()),
            'wholly_covered_in_one_convex_part_triangles': int(fully_covered.sum()),
            'triangles_checked_by_continuous_polygon_subtraction': int((~fully_covered).sum()),
            'uncovered_source_triangle_count': int((uncovered_area > 0).sum()),
            'uncovered_fragments': pieces_count, 'uncovered_area_m2': float(uncovered_area.sum()),
            'uncovered_source_area_fraction': float(uncovered_area.sum()/areas.sum()),
            'uncovered_area_by_source_triangle_m2': uncovered_area.tolist(),
            'largest_uncovered_fragment_witnesses': records,
            'all_source_triangles_covered_with_numerical_tolerance': not witnesses,
            'numeric_plane_tolerance_m': G.CLIP_TOL_M,
            'limits': ['Float64 halfspace clipping uses the unchanged existing numerical tolerance, not a physical acceptance threshold.',
                       'Coverage is source triangle surface inclusion, not source-solid equivalence or a global Hausdorff bound.']}
    if reference_union is not None:
        result.update({'newly_uncovered_previously_covered_source_area_m2': float(newly_uncovered_area.sum()),
                       'newly_uncovered_previously_covered_source_area_fraction': float(newly_uncovered_area.sum()/areas.sum()),
                       'newly_uncovered_area_by_source_triangle_m2': newly_uncovered_area.tolist(),
                       'coverage_change_method': 'continuous (source triangle outside repaired union) intersected with frozen old convex hull; numerical tolerance only'})
    return result


def convex_containment(union, reference, label):
    """All hull vertices against all reference planes certify convex inclusion.

    This finite test certifies entire convex-part containment because every
    convex-body point is a convex combination of its vertices and all tested
    inequalities are affine. It does not classify the source STL as a solid.
    """
    if len(reference.hulls) != 1:
        raise ValueError('Containment reference must be one convex hull')
    equations = reference.hulls[0]['equations']
    parts = []
    for hull in union.hulls:
        vertices = np.unique(hull['triangles'].reshape(-1, 3), axis=0)
        values = vertices @ equations[:, :3].T + equations[:, 3]
        per_vertex = values.max(axis=1)
        vertex_id, plane_id = np.unravel_index(np.argmax(values), values.shape)
        distance, nearest, triangle_id = reference.surface.nearest(vertices[vertex_id][None])
        parts.append({'part': hull['name'], 'hull_vertex_count': len(vertices),
            'contained_with_numerical_tolerance': bool(values.max() <= G.CLIP_TOL_M),
            'vertices_outside_numerical_tolerance': int((per_vertex > G.CLIP_TOL_M).sum()),
            'max_reference_halfspace_violation_m': float(values.max()),
            'max_violation_witness': {'point_m': vertices[vertex_id].tolist(),
                'reference_plane': equations[plane_id].tolist(),
                'reference_halfspace_violation_m': float(values.max()),
                'nearest_reference_boundary_distance_m': float(distance[0]),
                'nearest_reference_boundary_point_m': nearest[0].tolist(),
                'nearest_reference_boundary_triangle_id': int(triangle_id[0])}})
    return {'reference': label, 'all_parts_contained_with_numerical_tolerance':
                all(part['contained_with_numerical_tolerance'] for part in parts),
            'parts': parts, 'numeric_plane_tolerance_m': G.CLIP_TOL_M,
            'method': 'all vertices of each actual part convex hull against every reference convex halfspace',
            'limits': ['Positive plane violations witness noncontainment; negative violations are plane margins, not distances to the reference boundary.',
                       'Source convex-hull containment does not establish source-solid inclusion or absence of groove/cavity capping.']}


def run(candidate, output):
    started = time.monotonic()
    candidate, output = Path(candidate).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError('Output must be a new directory')
    old_manifest = json.loads((OLD.parent/'manifest.json').read_text())
    if (_hash(OLD) != old_manifest['candidate_model_sha256']
            or _hash(V.FORMAL_RIGHT) != old_manifest['formal_right_sha256']):
        raise ValueError('Frozen old candidate or formal right definition changed')
    sets = {'old': inputs(OLD), 'new': inputs(candidate)}
    paths = {OLD, OLD.parent/'manifest.json', candidate, V.FORMAL_RIGHT, Path(__file__).resolve(),
             HERE/'proxy76_validate.py', HERE/'proxy76_geometry.py', ROOT/'tools/build_right_proxy76.py',
             HERE/'evidence/d3_geometry/triangle/surface_geometry.py'}
    for source, bindings in sets.values():
        paths.add(Path(source['file']))
        paths.update(Path(b['file']) for b in bindings)
    before = {str(path.relative_to(ROOT)): _hash(path) for path in sorted(paths)}
    output.mkdir(parents=True)
    result = {'schema': 'finger13-local-repair-static-quality-v1',
              'old_candidate': str(OLD.relative_to(ROOT)), 'new_candidate': str(candidate.relative_to(ROOT)),
              'samples_per_direction': 4096, 'seed': 763,
              'physics_steps': 0, 'mujoco_imported': False, 'new_acceptance_thresholds': False,
              'input_sha256_before': before, 'models': {}}
    old_union = None
    for label, (source_binding, bindings) in sets.items():
        source = V.mesh_triangles(source_binding)
        parts = [{'name': b['name'], 'triangles': V.mesh_triangles(b)} for b in bindings]
        union = G.convex_union_surface(parts)
        if label == 'old':
            old_union = union
        value = {'source_binding': source_binding, 'proxy_bindings': bindings,
                 'surface_metrics': G.analyze_link(source, parts, samples_per_direction=4096, seed=763,
                     regions={**V._regions(source), 'fork_gap_y_minus6p5_to_minus5':
                         {'axis': 1, 'min': -.0065, 'max': -.005}}, include_vertex_extrema=True),
                 'continuous_source_triangle_coverage': coverage(source, union, old_union if label == 'new' else None),
                 'normal_angles': normals(source, union)}
        if label == 'new':
            right_hull = G.convex_union_surface([{'name': 'formal_right_source_global_convex_hull', 'triangles': source}])
            value['convex_containment'] = {
                'old_left_derived_proxy': convex_containment(union, old_union, 'frozen old left-derived finger13 convex hull'),
                'right_source_global_hull': convex_containment(union, right_hull, 'actual formal-right source global convex hull')}
        (output/(label+'.json')).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
        result['models'][label] = {'report': label+'.json', 'collision_parts': len(bindings),
            'source_to_proxy': value['surface_metrics']['source_to_proxy_union'],
            'proxy_to_source': value['surface_metrics']['proxy_union_to_source'],
            'uncovered_source_area_fraction': value['continuous_source_triangle_coverage']['uncovered_source_area_fraction'],
            'all_source_triangles_covered_with_numerical_tolerance': value['continuous_source_triangle_coverage']['all_source_triangles_covered_with_numerical_tolerance']}
        print(json.dumps({'event': 'quality_model_written', 'model': label, 'parts': len(bindings),
            'uncovered_area_fraction': result['models'][label]['uncovered_source_area_fraction']}), flush=True)
    result['input_sha256_after'] = {str(path.relative_to(ROOT)): _hash(path) for path in sorted(paths)}
    result['inputs_unchanged'] = result['input_sha256_before'] == result['input_sha256_after']
    result['wall_seconds'] = time.monotonic()-started
    result['limits'] = ['No new dynamic trajectory, physical contact solve or control validation.',
                        'Static source surface coverage and finite normal/deviation metrics do not establish grasp success.']
    (output/'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    if not result['inputs_unchanged']:
        raise RuntimeError('An input changed')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.candidate, args.output)
