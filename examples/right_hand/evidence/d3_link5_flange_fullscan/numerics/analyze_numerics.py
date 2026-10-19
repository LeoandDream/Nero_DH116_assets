#!/usr/bin/env python3
"""Read-only local explanation of five frozen compiled containment witnesses.

No source-coverage rerun, geometry repair, tolerance change, or dynamics API.
Run from the repository root with the approved reference interpreter.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import sys

sys.dont_write_bytecode = True
for variable in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
                 'BLIS_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[variable] = '1'
import numpy as np
from scipy.spatial import ConvexHull, cKDTree

HERE = Path(__file__).resolve().parents[3]
ROOT = HERE.parents[1]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import link5_flange_quality as Q

QUALITY = HERE/'evidence/d3_link5_flange_local/quality'
TOL = 1e-10


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_gzip(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


def snapshot(paths):
    return {str(path.relative_to(ROOT)): sha(path) for path in sorted(paths)}


def stats(values):
    return {'max_m': float(np.max(values)), 'median_m': float(np.median(values)),
            'p99_m': float(np.quantile(values, .99))}


def raw_vertices(binding):
    vertices, _ = Q.V.BUILDER.read_mesh(binding['file'])
    rotation = Q.V.BUILDER.quaternion_matrix(' '.join(map(str, binding['quat'])))
    return vertices, (vertices*np.asarray(binding['scale']))@rotation.T+binding['pos'], rotation


def extract(model, binding):
    part = Q.P.compiled_hull(model, binding['name'], binding)
    gid = Q.P.mujoco.mj_name2id(model, Q.P.mujoco.mjtObj.mjOBJ_GEOM, binding['name'])
    mid = part['audit']['mesh_id']
    start, count = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
    stored = np.asarray(model.mesh_vert[start:start+count], dtype=float).copy()
    rotation = Q.V.BUILDER.quaternion_matrix(' '.join(map(str, model.geom_quat[gid])))
    pos = np.asarray(model.geom_pos[gid], dtype=float).copy()
    original, raw, input_rotation = raw_vertices(binding)
    loaded32 = (original.astype(np.float32).astype(float)*np.asarray(binding['scale']))@input_rotation.T+binding['pos']
    ideal64 = (raw-pos)@rotation
    ideal_from32 = (loaded32-pos)@rotation
    reconstructed = ideal_from32.astype(np.float32).astype(float)
    reconstructed_body = reconstructed@rotation.T+pos
    compiled = stored@rotation.T+pos
    # Nearest-point matching does not assume the loader keeps input ordering.
    correspondence_dist, raw_id = cKDTree(raw).query(compiled)
    prediction_dist, compiled_id = cKDTree(stored).query(reconstructed)
    metadata = {
        'name': binding['name'], 'source_file': str(Path(binding['file']).relative_to(ROOT)),
        'raw_vertex_count': len(raw), 'compiled_vertex_count': len(stored),
        'compiled_support_count': len(part['vertices']),
        'mesh_vert_storage_dtype': str(model.mesh_vert.dtype),
        'compiled_geom_position_m': pos.tolist(),
        'compiled_geom_quaternion': model.geom_quat[gid].tolist(),
        'mesh_position_m': model.mesh_pos[mid].tolist(),
        'mesh_quaternion': model.mesh_quat[mid].tolist(),
        'raw_float64_to_input_float32_rounding': stats(np.linalg.norm(loaded32-raw, axis=1)),
        'actual_compiled_to_nearest_raw_vertex': stats(correspondence_dist),
        'float32_recenter_rotate_prediction_to_nearest_actual_stored_vertex': stats(prediction_dist),
        'reconstruction_definition': 'raw OBJ coordinates cast float32, apply original scale/pose, inverse compiled geom pose in float64, cast recentered/rotated coordinates float32, then compiled geom pose in float64',
        'compiled_pose_float64_inverse_roundtrip_error': stats(np.linalg.norm((compiled-pos)@rotation-stored, axis=1)),
        'support_ids': part['audit']['support_vertex_ids'],
    }
    arrays = {'raw': raw, 'loaded32': loaded32, 'stored': stored, 'compiled': compiled,
              'ideal64': ideal64, 'ideal_from32': ideal_from32,
              'reconstructed_body': reconstructed_body, 'rotation': rotation,
              'position': pos, 'raw_id_for_compiled': raw_id,
              'compiled_id_for_reconstructed': compiled_id,
              'support_ids': np.asarray(part['audit']['support_vertex_ids'])}
    return part, metadata, arrays


def main():
    if sys.executable != '/data_robot/verification/nero_dh116_right/.reference-env/bin/python':
        raise RuntimeError('Use the approved reference interpreter')
    if (OUT/'numerics.json').exists() or (OUT/'compiled_state.npz').exists():
        raise RuntimeError('Refuse to overwrite existing evidence')
    previous = json.loads((QUALITY/'summary.json').read_text())
    expected = previous['input_sha256_before']
    paths = {ROOT/path for path in expected}
    paths.update({QUALITY/'summary.json', Path(__file__).resolve()})
    paths.update(QUALITY/(body+'-compiled_mesh_graph.json.gz') for body in Q.BODIES)
    before = snapshot(paths)
    if any(before[path] != digest for path, digest in expected.items()):
        raise RuntimeError('A frozen prior-quality input differs')
    bindings = {label: Q.bindings(path) for label, path in [('old', Q.OLD), ('new', Q.NEW)]}
    manifest = json.loads((Q.NEW.parent/'manifest.json').read_text())
    selected = {}
    for body, target in zip(Q.BODIES, Q.TARGETS):
        names = {row['new_geom'] for row in manifest['new_geoms'] if row['old_geom'] == target}
        selected[body] = {'old': [b for b in bindings['old'][body] if b['name'] == target],
                          'new': [b for b in bindings['new'][body] if b['name'] in names]}
    extracted, saved_arrays = {}, {}
    with Q.compile_only() as audit:
        for label, path in [('old', Q.OLD), ('new', Q.NEW)]:
            model = Q.P.mujoco.MjModel.from_xml_path(str(path))
            audit['model_constructor_calls'] += 1
            for body in Q.BODIES:
                for binding in selected[body][label]:
                    part, metadata, arrays = extract(model, binding)
                    extracted[(body, label, binding['name'])] = (part, metadata, arrays)
                    for key, value in arrays.items():
                        saved_arrays[label+'__'+binding['name']+'__'+key] = value
    result = {
        'schema': 'frozen-link5-flange-local-numerics-v1',
        'interpreter': sys.executable, 'tolerance_m': TOL,
        'threshold_unchanged': True, 'call_audit': audit,
        'source_coverage_recomputed': False,
        'input_sha256_before': before, 'coverage_from_verified_existing_evidence': {},
        'parts': {label+'__'+name: metadata for (_, label, name), (_, metadata, _) in extracted.items()},
        'containment_witnesses': [], 'source_coverage_witnesses': [],
        'limits': [
            'Only five previously saved maximum-containment witnesses are traced; this is not a new exhaustive source-coverage computation.',
            'A nearest raw-vertex correspondence is stated with its measured distance; exact same-source claims require zero raw-coordinate difference.',
            'Float32 reconstruction is checked against actual compiled arrays; residuals are retained, not hidden by the unchanged threshold.',
            'Finite uncovered surface area can result from nanometre normal displacement of a planar patch; area is not gap width.',
            'No source-solid, physical-penetration, solver-contact, dynamics, or grasp claim.',
            'Exposed-union volume is not recomputed or used; its numerical sensitivity is not physical geometry change.'
        ]}
    for body, target in zip(Q.BODIES, Q.TARGETS):
        report = read_gzip(QUALITY/(body+'-compiled_mesh_graph.json.gz'))
        link = previous['links'][body]
        raw_reference_part, _, old = extracted[(body, 'old', target)]
        raw_hull = ConvexHull(old['raw'])
        raw_surface = Q.G.TriangleSurface(old['raw'][raw_hull.simplices])
        source_v, source_f = Q.V.BUILDER.read_mesh(ROOT/link['source_file'])
        result['coverage_from_verified_existing_evidence'][body] = {
            representation: {key: {k: v for k, v in link[representation][key].items()
                                   if not isinstance(v, (list, dict))}
                             for key in ('old_full_source_coverage', 'new_full_source_coverage',
                                         'original_assigned_faces_new_local_coverage')}
            for representation in ('raw_float64', 'compiled_mesh_graph')}
        for witness_row in report['new_parts_within_old_replaced_hull']['parts']:
            name = witness_row['part']
            _, _, new = extracted[(body, 'new', name)]
            witness = witness_row['max_violation_witness']
            point, plane = np.asarray(witness['point_m']), np.asarray(witness['reference_plane'])
            vertex_dist, vertex_id = cKDTree(new['compiled']).query(point)
            raw_dist, raw_id = cKDTree(new['raw']).query(point)
            raw_point = new['raw'][raw_id]
            oldraw_dist, oldraw_id = cKDTree(old['raw']).query(raw_point)
            source_dist, source_id = cKDTree(source_v).query(raw_point)
            old_compiled_dist, old_compiled_id = cKDTree(old['compiled']).query(old['raw'][oldraw_id])
            old_point = old['compiled'][old_compiled_id]
            raw_boundary_dist, raw_nearest, raw_face = raw_surface.nearest(raw_point[None])
            input_rounding = new['loaded32'][raw_id]-raw_point
            predicted = new['reconstructed_body'][raw_id]
            drift = point-raw_point
            raw_margin = float(np.max(raw_hull.equations[:, :3]@raw_point+raw_hull.equations[:, 3]))
            # Identity: n.p_new+d = n.(p_new-p_raw) + (n.p_raw+d).
            row = {
                'body': body, 'part': name, 'stored_witness': witness,
                'exact_compiled_vertex_match_distance_m': float(vertex_dist),
                'compiled_vertex_index': int(vertex_id), 'nearest_raw_vertex_index': int(raw_id),
                'raw_point_m': raw_point.tolist(), 'compiled_minus_raw_m': drift.tolist(),
                'compiled_to_raw_distance_m': float(raw_dist),
                'raw_OBJ_to_initial_float32_delta_m': input_rounding.tolist(),
                'raw_OBJ_to_initial_float32_distance_m': float(np.linalg.norm(input_rounding)),
                'float32_recenter_rotate_prediction_m': predicted.tolist(),
                'prediction_minus_actual_compiled_m': (predicted-point).tolist(),
                'prediction_to_actual_compiled_distance_m': float(np.linalg.norm(predicted-point)),
                'raw_point_nearest_factory_source_vertex_id': int(source_id),
                'raw_point_to_factory_source_vertex_distance_m': float(source_dist),
                'raw_point_equals_factory_source_vertex_exactly': bool(source_dist == 0),
                'raw_point_to_old_raw_vertex_distance_m': float(oldraw_dist),
                'raw_point_equals_old_raw_vertex_exactly': bool(oldraw_dist == 0),
                'old_raw_vertex_index': int(oldraw_id),
                'raw_reference_max_halfspace_margin_m': raw_margin,
                'raw_reference_boundary_distance_m': float(raw_boundary_dist[0]),
                'raw_point_is_on_old_raw_boundary_at_unchanged_tolerance': bool(abs(raw_margin) <= TOL and raw_boundary_dist[0] <= TOL),
                'raw_reference_nearest_triangle_vertices_m': old['raw'][raw_hull.simplices[raw_face[0]]].tolist(),
                'compiled_reference_plane_evaluation_at_raw_point_m': float(plane[:3]@raw_point+plane[3]),
                'new_compilation_drift_projected_on_reference_normal_m': float(plane[:3]@drift),
                'recomputed_compiled_plane_violation_m': float(plane[:3]@point+plane[3]),
                'unchanged_threshold_status': 'FAIL' if plane[:3]@point+plane[3] > TOL else 'PASS',
                'old_compiled_corresponding_same_raw_point_m': old_point.tolist(),
                'old_compiled_correspondence_distance_to_raw_m': float(old_compiled_dist),
                'new_compiled_minus_old_compiled_same_raw_point_m': (point-old_point).tolist(),
                'new_minus_old_compiled_normal_component_m': float(plane[:3]@(point-old_point)),
                'old_compiled_same_raw_point_reference_plane_residual_m': float(plane[:3]@old_point+plane[3]),
                'old_compiled_correspondence_is_mesh_graph_support': bool(old_compiled_id in old['support_ids']),
            }
            result['containment_witnesses'].append(row)
        # One already-saved largest uncovered fragment per body; no full scan.
        witness = report['new_full_source_coverage']['largest_uncovered_fragment_witnesses'][0]
        triangle = source_v[source_f[witness['source_triangle_id']]]
        centroid = np.asarray(witness['centroid_m'])
        coeff = np.linalg.lstsq((triangle[1:]-triangle[0]).T, centroid-triangle[0], rcond=None)[0]
        projected = triangle[0]+coeff@(triangle[1:]-triangle[0])
        result['source_coverage_witnesses'].append({
            'body': body, 'stored_witness': witness, 'source_triangle_vertices_m': triangle.tolist(),
            'centroid_barycentric_on_factory_source_triangle': [float(1-coeff.sum()), *coeff.tolist()],
            'centroid_to_factory_source_triangle_plane_residual_m': float(np.linalg.norm(projected-centroid)),
            'interpretation': 'This is a saved finite surface fragment whose centroid is only the reported nanometre normal distance from the compiled union; finite area is not a physical hole width.'})
    result['input_sha256_after'] = snapshot(paths)
    result['inputs_unchanged'] = before == result['input_sha256_after']
    if not result['inputs_unchanged']:
        raise RuntimeError('An input changed')
    np.savez_compressed(OUT/'compiled_state.npz', **saved_arrays)
    with (OUT/'numerics.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'input_count': len(before), 'inputs_unchanged': True,
                      'api_counts': dict(audit['allowed_api_calls']),
                      'model_constructor_calls': audit['model_constructor_calls'],
                      'witnesses': [{'part': r['part'], 'violation_m': r['recomputed_compiled_plane_violation_m'],
                                     'same_factory_vertex': r['raw_point_equals_factory_source_vertex_exactly'],
                                     'same_old_vertex': r['raw_point_equals_old_raw_vertex_exactly'],
                                     'raw_boundary': r['raw_point_is_on_old_raw_boundary_at_unchanged_tolerance'],
                                     'prediction_error_m': r['prediction_to_actual_compiled_distance_m']}
                                    for r in result['containment_witnesses']]}))


if __name__ == '__main__':
    main()
