#!/usr/bin/env python3
"""Explain saved compiled arrays with the two explicit float32 writebacks.

Pure NumPy: no MuJoCo import or constructor/API calls. Preserve prior output.
"""
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

OUT = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    previous = json.loads((OUT/'numerics.json').read_text())
    state = np.load(OUT/'compiled_state.npz')
    original_bindings = {}
    for label, candidate in [('old', 'finger13_local'), ('new', 'link5_flange_local')]:
        xml = ET.parse(OUT.parents[2]/'candidates'/candidate/'model.xml').getroot()
        assets = {mesh.get('name'): mesh for mesh in xml.findall('asset/mesh')}
        for geom in xml.findall('.//geom'):
            key = label+'__'+str(geom.get('name'))
            if key in previous['parts']:
                values = {'pos': np.fromstring(geom.get('pos', '0 0 0'), sep=' ').tolist(),
                          'quat': np.fromstring(geom.get('quat', '1 0 0 0'), sep=' ').tolist(),
                          'scale': np.fromstring(assets[geom.get('mesh')].get('scale', '1 1 1'), sep=' ').tolist()}
                assert values == {'pos': [0., 0., 0.], 'quat': [1., 0., 0., 0.], 'scale': [1., 1., 1.]}
                original_bindings[key] = values
    assert set(original_bindings) == set(previous['parts'])
    parts, phases = {}, {}
    for name in previous['parts']:
        a = lambda key: state[name+'__'+key]
        centered64 = a('loaded32')-a('position')
        centered32 = centered64.astype(np.float32).astype(float)
        rotated64 = centered32@a('rotation')
        rotated32 = rotated64.astype(np.float32).astype(float)
        reconstructed = rotated32@a('rotation').T+a('position')
        recenter_delta = centered32-centered64
        rotate_delta_body = (rotated32-rotated64)@a('rotation').T
        initial_delta = a('loaded32')-a('raw')
        phases[name] = (recenter_delta, rotate_delta_body, reconstructed, initial_delta)
        parts[name] = {
            'vertex_count': len(rotated32),
            'ordered_array_exactly_equals_actual_mesh_vert': bool(np.array_equal(rotated32, a('stored'))),
            'max_ordered_mesh_vert_error_m': float(np.linalg.norm(rotated32-a('stored'), axis=1).max()),
            'ordered_body_coordinates_exactly_equal_actual_compiled_vertices': bool(np.array_equal(reconstructed, a('compiled'))),
            'max_initial_input_float32_rounding_m': float(np.linalg.norm(initial_delta, axis=1).max()),
            'max_recenter_float32_writeback_error_m': float(np.linalg.norm(recenter_delta, axis=1).max()),
            'max_rotate_float32_writeback_error_m': float(np.linalg.norm(rotate_delta_body, axis=1).max()),
            'max_total_actual_compiled_minus_raw_error_m': float(np.linalg.norm(reconstructed-a('raw'), axis=1).max()),
            'max_error_decomposition_closure_m': float(np.linalg.norm(
                reconstructed-a('raw')-initial_delta-recenter_delta-rotate_delta_body, axis=1).max()),
        }
    witnesses = []
    for old in previous['containment_witnesses']:
        name = 'new__'+old['part']
        recenter, rotate, reconstructed, initial = phases[name]
        index = old['nearest_raw_vertex_index']
        normal = np.asarray(old['stored_witness']['reference_plane'][:3])
        witnesses.append({
            'part': old['part'], 'raw_vertex_index': index,
            'reconstructed_body_point_m': reconstructed[index].tolist(),
            'exactly_equals_previously_saved_max_violation_point': bool(np.array_equal(reconstructed[index], old['stored_witness']['point_m'])),
            'input_float32_error_m': initial[index].tolist(),
            'recenter_float32_error_m': recenter[index].tolist(),
            'rotate_float32_error_in_body_axes_m': rotate[index].tolist(),
            'recenter_error_projected_on_reference_normal_m': float(normal@recenter[index]),
            'rotate_error_projected_on_reference_normal_m': float(normal@rotate[index]),
            'unchanged_1e_minus_10_m_status': old['unchanged_threshold_status'],
            'compiled_reference_plane_violation_m': old['recomputed_compiled_plane_violation_m'],
        })
    result = {
        'schema': 'frozen-two-writeback-roundtrip-v1',
        'interpreter': sys.executable,
        'package_versions': {package: version(package) for package in ('mujoco', 'numpy', 'scipy')},
        'algorithm': 'raw32 = float32(raw OBJ); centered32 = float32(raw32 - compiled mesh CoM); mesh32 = float32(centered32 @ compiled rotation); body64 = mesh32 @ compiled rotation.T + CoM',
        'algorithm_scope': 'The seven selected bindings have unchanged identity original scale/pose; using the compiled body-local geom pose is valid for these arrays.',
        'verified_original_bindings': original_bindings,
        'official_source': {
            'url': 'https://raw.githubusercontent.com/google-deepmind/mujoco/3.2.3/src/user/user_mesh.cc',
            'recenter_writeback_lines': [1309, 1314],
            'rotation_writeback_lines': [1385, 1396],
            'source_role': 'Read-only source inspection identified the intermediate recenter float32 writeback omitted by the one-writeback diagnostic in numerics.json.'},
        'input_hashes': {name: digest(OUT/name) for name in ('analyze_numerics.py', 'numerics.json', 'compiled_state.npz', 'verify_roundtrip.py')},
        'parts': parts, 'witnesses': witnesses,
        'all_ordered_arrays_exactly_reconstructed': all(p['ordered_array_exactly_equals_actual_mesh_vert'] for p in parts.values()),
        'total_vertex_count': sum(p['vertex_count'] for p in parts.values()),
        'all_five_stored_witness_points_exactly_reconstructed': all(p['exactly_equals_previously_saved_max_violation_point'] for p in witnesses),
        'call_audit': {'model_constructor_calls': 0, 'data_constructor_calls': 0,
                       'mujoco_api_calls': 0, 'forward_calls': 0, 'collision_calls': 0,
                       'constraint_solves': 0, 'physics_steps': 0},
        'tolerance_m': previous['tolerance_m'],
        'strict_compiled_status': 'FAIL_AT_UNCHANGED_NUMERICAL_TOLERANCE',
        'interpretation': 'For these five witnesses, independent float32 recenter and rotate roundtrips place the same original source boundary vertex at different new and old compiled positions. Their normal differences exactly account for the saved nanometre containment violations up to float64 plane-evaluation rounding. This does not establish every uncovered source fragment cause or change the FAIL status.',
        'limits': previous['limits'],
    }
    with (OUT/'roundtrip.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({key: result[key] for key in ('all_ordered_arrays_exactly_reconstructed',
          'total_vertex_count', 'all_five_stored_witness_points_exactly_reconstructed', 'strict_compiled_status')}))


if __name__ == '__main__':
    main()
