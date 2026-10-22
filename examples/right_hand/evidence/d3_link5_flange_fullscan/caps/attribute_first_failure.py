#!/usr/bin/env python3
"""Attribute the stored first failure using source triangles and compiled faces.

This creates evidence only: one frozen model compilation, no MjData, FK,
collision, constraints, dynamics, candidate generation or parameter search.
"""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull, cKDTree

ROOT = next(p for p in Path(__file__).resolve().parents
            if (p/'mjcf/nero_dh116_right.xml').is_file())
sys.path.insert(0, str(ROOT/'examples/right_hand'))
import link5_flange_poses as check

HERE = Path(__file__).resolve().parent
OLD = ROOT/'examples/right_hand/evidence/d3_link5_flange_local/poses'
OUT = HERE/'first-failure-cap-attribution.json'
NAMES = ('robot0_link5_local_01', 'robot0_flange_local_01')
FACE_IDS = (54, 45)


def point_query(point, triangles, bvh):
    degenerate = np.repeat(np.asarray(point)[None, None, :], 3, axis=1)
    return check.geometry.surface_query(degenerate, triangles,
        check.geometry.BVH(degenerate), bvh)


def barycentric(point, triangle):
    ab, ac = triangle[1:]-triangle[0]
    uv = np.linalg.lstsq(np.stack((ab, ac), axis=1), point-triangle[0], rcond=None)[0]
    return [float(1-uv.sum()), *uv.tolist()]


def face_plane(hull, index):
    triangle = hull['triangles'][index]
    normal = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
    normal /= np.linalg.norm(normal)
    if normal@(hull['vertices'].mean(axis=0)-triangle[0]) > 0:
        normal = -normal
    return normal, -float(normal@triangle[0])


def section(triangles, point, normal, basis):
    """Continuous plane/triangle intersections, retaining the source face order."""
    distances = (triangles-point)@normal
    rows = []
    for face in np.flatnonzero((distances.min(axis=1) <= 1e-12) &
                               (distances.max(axis=1) >= -1e-12)):
        triangle, ds, points = triangles[face], distances[face], []
        for i, j in ((0, 1), (1, 2), (2, 0)):
            if abs(ds[i]) <= 1e-12:
                points.append(triangle[i])
            if ds[i]*ds[j] < 0:
                points.append(triangle[i]+ds[i]/(ds[i]-ds[j])*(triangle[j]-triangle[i]))
        unique = []
        for p in points:
            if not any(np.linalg.norm(p-q) < 1e-11 for q in unique):
                unique.append(p)
        if len(unique) < 2:
            continue
        # Coplanar triangles retain all three boundary segments.
        edges = list(zip(unique, unique[1:]+unique[:1])) if len(unique) > 2 else [unique]
        for p, q in edges:
            rows.append({'face_id_zero_based': int(face),
                'endpoints_body5_m': [p.tolist(), q.tolist()],
                'endpoints_section_m': ((np.asarray([p, q])-point)@basis.T).tolist()})
    return rows


def summarize_counterfactual(query, source_first):
    ids = query['nearest_face_ids_zero_based']
    return {'query': query,
        'source_face_id_zero_based': ids[0 if source_first else 1],
        'compiled_hull_face_id_zero_based': ids[1 if source_first else 0],
        'strict_source_surface_crosses_opposite_compiled_hull_surface':
            query['strict_transverse_witness'] is not None,
        'scope': 'Geometric representation counterfactual at the unchanged stored pose; '
                 'not a new candidate or model acceptance. Source-solid containment remains unknown.'}


def source_vertex_membership(source, hull_vertices, rotation, translation):
    """All unique source vertices against the opposite actual convex solid."""
    vertices, inverse = np.unique(source.reshape(-1, 3), axis=0, return_inverse=True)
    moved = vertices@rotation.T+translation
    equations = ConvexHull(hull_vertices).equations
    lengths = np.linalg.norm(equations[:, :3], axis=1)
    equations /= lengths[:, None]
    maximum_residual = np.concatenate([
        (moved[start:start+2048]@equations[:, :3].T+equations[:, 3]).max(axis=1)
        for start in range(0, len(moved), 2048)])
    inside = maximum_residual < -check.TOL
    outside = maximum_residual > check.TOL
    boundary = ~(inside | outside)
    sample_ids = np.argsort(maximum_residual)[:min(5, len(vertices))]
    sample = []
    for vertex_id in sample_ids:
        source_faces = np.unique(np.flatnonzero(inverse == vertex_id)//3)
        sample.append({'unique_source_vertex_id_zero_based': int(vertex_id),
            'point_source_body_local_m': vertices[vertex_id].tolist(),
            'point_opposite_hull_body_local_m': moved[vertex_id].tolist(),
            'maximum_outward_halfspace_residual_m': float(maximum_residual[vertex_id]),
            'source_face_ids_zero_based': source_faces.tolist()})
    return {'method': 'Every exact unique source-STL vertex, transformed only by stored body transforms, against normalized ConvexHull halfspaces of actual compiled graph supports. Negative maximum residual is inside.',
        'source_triangle_count': len(source), 'all_unique_source_vertices_tested': len(vertices),
        'strictly_inside_vertex_count': int(inside.sum()),
        'strictly_exterior_vertex_count': int(outside.sum()),
        'boundary_or_numerically_unknown_vertex_count': int(boundary.sum()),
        'existing_geometric_tolerance_m': check.TOL,
        'minimum_of_maximum_outward_halfspace_residual_m': float(maximum_residual.min()),
        'maximum_of_maximum_outward_halfspace_residual_m': float(maximum_residual.max()),
        'opposite_compiled_support_max_halfspace_residual_m': float(
            (hull_vertices@equations[:, :3].T+equations[:, 3]).max()),
        'least_exterior_or_deepest_inside_source_vertices': sample,
        'whole_source_component_containment_in_opposite_hull':
            'EXCLUDED_NO_SOURCE_VERTEX_INSIDE_OR_BOUNDARY' if np.all(outside)
            else 'UNKNOWN_SOURCE_VERTEX_INSIDE_OR_BOUNDARY_REQUIRES_COMPONENT_INTERPRETATION'}


def main():
    if OUT.exists():
        raise ValueError('Attribution evidence already exists')
    previous = json.loads((OLD/'first-failure-classification.json').read_text())
    distances = json.loads((OLD/'first-failure-witness-source-distances.json').read_text())
    manifest_path = check.CANDIDATE.parent/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    if check.legacy.sha(check.CANDIDATE) != check.EXPECTED_CANDIDATE:
        raise ValueError('Candidate identity differs')
    if previous['pose_id'] != 'm040_quarter3' or distances['candidate_crossing_face_ids_zero_based'] != list(FACE_IDS):
        raise ValueError('Stored witness binding differs')
    relative_rotation, relative_translation = check.relative(previous['body_world_transforms'])
    point = np.asarray(distances['existing_crossing_witness_point_body5_m'])
    records = {r['new_geom']: r for r in manifest['new_geoms']}
    for name in NAMES:
        if check.legacy.sha(ROOT/records[name]['source_file']) != records[name]['source_sha256']:
            raise ValueError('Source STL identity differs: '+name)
    xml = ET.parse(check.CANDIDATE).getroot()
    assets = {a.get('name'): a for a in xml.findall('asset/mesh')}
    bindings = {name: check.reader._binding(check.CANDIDATE, xml, assets,
        xml.find('.//geom[@name="'+name+'"]')) for name in NAMES}
    paths = {Path(__file__), check.CANDIDATE, manifest_path,
        OLD/'first-failure-classification.json', OLD/'first-failure-witness-source-distances.json',
        Path(check.__file__), Path(check.geometry.__file__), Path(check.saved.__file__),
        Path(check.reader.__file__), Path(check.reader.BUILDER.__file__)}
    paths.update(Path(b['file']) for b in bindings.values())
    paths.update(ROOT/records[name]['source_file'] for name in NAMES)
    before = check.legacy.snapshot(paths)
    original_safe = check.saved.SAFE_APIS
    check.saved.SAFE_APIS = {'mj_name2id', 'mj_id2name'}
    try:
        with check.saved.static_only() as audit:
            model = check.mujoco.MjModel.from_xml_path(str(check.CANDIDATE))
            fingerprint = check.saved.fields(model)
            hulls = [check.compiled_hull(model, name, bindings[name]) for name in NAMES]
            if fingerprint != check.saved.fields(model):
                raise ValueError('Compiled model changed')
            audit = dict(audit, allowed_api_calls=dict(audit['allowed_api_calls']),
                model_constructor_calls=1, data_constructor_calls=0, new_FK_calls=0,
                model_fields_unchanged=True)
    finally:
        check.saved.SAFE_APIS = original_safe
    sources = [check.geometry.read_stl(ROOT/records[name]['source_file']) for name in NAMES]
    bvhs = [check.geometry.BVH(t) for t in sources]
    faces = []
    for i, (name, hull, face, source, bvh) in enumerate(zip(NAMES, hulls, FACE_IDS, sources, bvhs)):
        record, triangle = records[name], hull['triangles'][face]
        native_point = point if i == 0 else (point-relative_translation)@relative_rotation
        normal, offset = face_plane(hull, face)
        if abs(native_point@normal+offset) > check.TOL or min(barycentric(native_point, triangle)) < -check.TOL:
            raise ValueError('Stored crossing no longer lies on the actual compiled face')
        support = np.flatnonzero(np.abs(hull['vertices']@normal+offset) <= 1e-9)
        on_plane = np.max(np.abs(hull['triangles']@normal+offset), axis=1) <= 1e-9
        raw_vertices, _ = check.reader.BUILDER.read_mesh(Path(bindings[name]['file']))
        tree = cKDTree(raw_vertices)
        assigned_ids = np.asarray(record['source_triangle_ids_zero_based'], dtype=int)
        assigned_source = source[assigned_ids]
        assigned_bvh = check.geometry.BVH(assigned_source)
        provenance = []
        for vertex in triangle:
            drift, raw_id = tree.query(vertex)
            raw_point = raw_vertices[raw_id]
            query = point_query(raw_point, source, bvh)
            source_face = query['nearest_face_ids_zero_based'][1]
            assigned_query = point_query(raw_point, assigned_source, assigned_bvh)
            assigned_face = int(assigned_ids[assigned_query['nearest_face_ids_zero_based'][1]])
            provenance.append({'compiled_support_body_local_m': vertex.tolist(),
                'nearest_raw_obj_vertex_id_zero_based': int(raw_id),
                'nearest_raw_obj_vertex_body_local_m': raw_point.tolist(),
                'compiled_to_raw_drift_m': float(drift),
                'raw_point_to_full_source_distance_m': query['surface_min_distance_m'],
                'nearest_source_face_id_zero_based': source_face,
                'nearest_source_face_in_manifest_piece': source_face in record['source_triangle_ids_zero_based'],
                'continuous_distance_to_assigned_source_triangles_m': assigned_query['surface_min_distance_m'],
                'nearest_assigned_source_face_id_zero_based': assigned_face,
                'nearest_assigned_source_point_body_local_m': assigned_query['closest_points'][1],
                'nearest_assigned_source_triangle_body_local_m': source[assigned_face].tolist(),
                'nearest_source_triangle_body_local_m': source[source_face].tolist(),
                'nearest_source_point_body_local_m': query['closest_points'][1],
                'nearest_source_point_barycentric': barycentric(np.asarray(query['closest_points'][1]), source[source_face])})
        probes = {'stored_crossing': native_point, 'cap_centroid': triangle.mean(axis=0)}
        probe_rows = {}
        for label, probe in probes.items():
            query = point_query(probe, source, bvh)
            probe_rows[label] = {'point_body_local_m': probe.tolist(),
                'barycentric_in_compiled_face': barycentric(probe, triangle),
                'signed_cap_plane_distance_m': float(probe@normal+offset),
                'continuous_distance_to_full_source_surface_m': query['surface_min_distance_m'],
                'nearest_source_face_id_zero_based': query['nearest_face_ids_zero_based'][1],
                'full_query': query}
        faces.append({'geom': name, 'piece_partition': record['partition'],
            'source_file': record['source_file'], 'source_sha256': record['source_sha256'],
            'compiled_face_id_zero_based': face,
            'compiled_face_vertex_ids': hull['audit']['face_vertex_ids'][face],
            'compiled_face_body_local_m': triangle.tolist(),
            'outward_unit_normal_body_local': normal.tolist(), 'halfspace_offset_m': offset,
            'all_compiled_supports_max_halfspace_residual_m': float(np.max(hull['vertices']@normal+offset)),
            'coplanar_compiled_face_ids_zero_based': np.flatnonzero(on_plane).tolist(),
            'coplanar_support_local_indices': support.tolist(),
            'support_provenance': provenance, 'probes': probe_rows})
    a, b = hulls
    moved_hull = b['triangles']@relative_rotation.T+relative_translation
    moved_source = sources[1]@relative_rotation.T+relative_translation
    counterfactuals = {
        'source_link5_vs_compiled_flange01': summarize_counterfactual(check.geometry.surface_query(
            sources[0], moved_hull, bvhs[0], b['bvh'].refit(moved_hull)), True),
        'compiled_link501_vs_source_flange': summarize_counterfactual(check.geometry.surface_query(
            a['triangles'], moved_source, a['bvh'], bvhs[1].refit(moved_source)), False)}
    memberships = {
        'source_link5_vs_compiled_flange01': source_vertex_membership(sources[0], b['vertices'],
            relative_rotation.T, -relative_translation@relative_rotation),
        'compiled_link501_vs_source_flange': source_vertex_membership(sources[1], a['vertices'],
            relative_rotation, relative_translation)}
    for label, row in counterfactuals.items():
        row['full_source_vertex_opposite_hull_membership'] = memberships[label]
        membership = memberships[label]
        row['source_triangle_surface_vs_opposite_convex_solid_classification'] = (
            'NO_SOURCE_TRIANGLE_ENTERS_OPPOSITE_CONVEX_SOLID_AT_THIS_STORED_POSE'
            if row['query']['all_surface_search_complete'] and row['query']['surface_min_distance_m'] > check.TOL
            and membership['strictly_inside_vertex_count'] == 0
            and membership['boundary_or_numerically_unknown_vertex_count'] == 0
            else 'UNKNOWN_CONTAINMENT_OR_SURFACE_CROSSING')
    both_clear = all(row['source_triangle_surface_vs_opposite_convex_solid_classification'].startswith('NO_SOURCE_TRIANGLE')
                     for row in counterfactuals.values())
    both_unsupported = all(row['probes']['stored_crossing']['continuous_distance_to_full_source_surface_m'] > check.TOL
                           for row in faces)
    normal_a = np.asarray(faces[0]['outward_unit_normal_body_local'])
    normal_b = np.asarray(faces[1]['outward_unit_normal_body_local'])@relative_rotation.T
    section_normal = np.cross(normal_a, normal_b)
    section_normal /= np.linalg.norm(section_normal)
    basis = np.asarray([normal_a, np.cross(section_normal, normal_a)])
    sections = {label: section(triangles, point, section_normal, basis) for label, triangles in (
        ('source_link5', sources[0]), ('source_flange', moved_source),
        ('compiled_link501', a['triangles']), ('compiled_flange01', moved_hull))}
    result = {'schema': 'd3-link5-flange-first-failure-cap-attribution-v1',
        'candidate_model_sha256': check.EXPECTED_CANDIDATE, 'pose_id': previous['pose_id'],
        'body_world_transforms': previous['body_world_transforms'],
        'source_surface_gap_m': previous['source_full_continuous_triangle_queries']['urdf_stl']['surface_min_distance_m'],
        'stored_crossing_point_body5_m': point.tolist(),
        'compiled_faces': faces, 'one_side_source_surface_counterfactuals': counterfactuals,
        'local_surface_attribution': {
            'classification': 'TWO_UNSUPPORTED_CONVEX_CAPS_CROSS_WHILE_EITHER_FULL_SOURCE_SURFACE_REPLACEMENT_REMOVES_SURFACE_CROSSING'
                if both_clear and both_unsupported else 'UNKNOWN_LOCAL_CAP_ATTRIBUTION',
            'link5_cap': 'compiled face 54 joins native z=-106.509 mm to z near 0/-2.559 mm within the middle-y slab; it bridges the source concavity rather than lying on a y slab cut-plane.',
            'flange_cap': 'compiled face 45 joins native z=-32.743/-30.740 mm to z=+5.212 mm within source component 1; its interior also bridges source geometry.',
            'exclusive_single_side_root_cause': 'NOT_ESTABLISHED',
            'solid_collision_attribution': 'UNKNOWN_NONCLOSED_LINK5_SOURCE',
            'counterfactual_interpretation': 'For this pose, neither compiled hull surface crosses the opposite full source surface. The witnessed surface crossing lies between unsupported cap surfaces on both sides. This is not a source-solid separation or physical-collision certificate.'},
        'representative_continuous_section': {'origin_body5_m': point.tolist(),
            'unit_plane_normal_body5': section_normal.tolist(), 'unit_axes_body5': basis.tolist(),
            'construction': 'Plane through the stored crossing, spanned by the two actual crossing-face outward normals; no fitted or searched section.',
            'segments': sections},
        'call_audit': audit, 'physics_steps': 0, 'forward_calls': 0, 'collision_calls': 0,
        'constraint_solves': 0, 'new_candidate_count': 0, 'new_pose_count': 0,
        'input_sha256_before': before, 'input_sha256_after': check.legacy.snapshot(paths),
        'limitations': [
            'Cap refers to an actual convex graph face bridging source geometry, not necessarily a slab cut-plane face.',
            'Positive cap-interior distance to source proves unsupported convex surface locally; it does not measure physical solid penetration.',
            'One-side source-surface counterfactuals identify sufficient convex representation contributions where strict crossing exists; no single-side exclusivity is inferred.',
            'link5 source is not a closed manifold; source solid containment and physical collision remain UNKNOWN.',
            'The full planning scan is separate evidence; this file attributes the original first failed pose only.']}
    if result['input_sha256_before'] != result['input_sha256_after']:
        raise ValueError('Attribution input changed')
    check.dump(OUT, result)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig, ax = plt.subplots(figsize=(8, 7))
    for label, color, style in (('source_link5', '#1865ac', '-'), ('source_flange', '#ca5521', '-'),
                               ('compiled_link501', '#1865ac', '--'), ('compiled_flange01', '#ca5521', '--')):
        segments = [np.asarray(r['endpoints_section_m'])*1000 for r in sections[label]]
        ax.add_collection(LineCollection(segments, colors=color, linestyles=style,
                                        linewidths=1.1 if 'source' in label else .7, label=label))
    ax.plot(0, 0, 'kx', label='stored crossing')
    ax.set(xlim=(-16, 16), ylim=(-20, 20), xlabel='section axis 1 (mm)', ylabel='section axis 2 (mm)',
           title='m040_quarter3: continuous source / compiled convex sections')
    ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(HERE/'representative-section.png', dpi=180); plt.close(fig)
    print(json.dumps({'pose': result['pose_id'], 'source_surface_gap_mm': result['source_surface_gap_m']*1000,
        'one_side_counterfactuals': {k: {'strict_crossing': v['strict_source_surface_crosses_opposite_compiled_hull_surface'],
            'face_ids': v['query']['nearest_face_ids_zero_based'], 'gap_mm': v['query']['surface_min_distance_m']*1000}
            for k, v in counterfactuals.items()}}), flush=True)


if __name__ == '__main__':
    main()
