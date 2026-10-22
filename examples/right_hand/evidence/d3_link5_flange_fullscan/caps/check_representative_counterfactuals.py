#!/usr/bin/env python3
"""Six first/max/last pair roles from four already-scanned poses; evidence only."""
import gzip
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

import attribute_first_failure as cap

check, ROOT, HERE = cap.check, cap.ROOT, cap.HERE
SCAN = HERE.parent/'scan'
OUT = HERE/'representative-pose-counterfactuals.json'


def mixed(query, source, source_hull, rotation, translation):
    membership = cap.source_vertex_membership(source, source_hull['vertices'], rotation, translation)
    surface_crossing = query['strict_transverse_witness'] is not None
    inside = membership['strictly_inside_vertex_count'] > 0
    return {'continuous_surface_query': query, 'source_vertex_membership': membership,
        'strict_source_surface_crossing_or_strict_vertex_inside_opposite_hull': surface_crossing or inside,
        'source_surface_exterior_to_opposite_convex_solid':
            query['all_surface_search_complete'] and query['surface_min_distance_m'] > check.TOL
            and membership['strictly_inside_vertex_count'] == 0
            and membership['boundary_or_numerically_unknown_vertex_count'] == 0}


def main():
    if OUT.exists():
        raise ValueError('Representative evidence already exists')
    with gzip.open(SCAN/'poses.json.gz', 'rt') as stream:
        poses = json.load(stream)['poses']
    summary = json.loads((SCAN/'summary.json').read_text())
    if len(poses) != 921 or summary['pair_case_count'] != 5526:
        raise ValueError('Unexpected complete scan')
    pairs = sorted({tuple(p['pair']) for r in poses for p in r['pairs']
                    if p['status'] == 'STRICT_CONVEX_OVERLAP'})
    roles = []
    for pair in pairs:
        rows = [(r, p) for r in poses for p in r['pairs']
                if tuple(p['pair']) == pair and p['status'] == 'STRICT_CONVEX_OVERLAP']
        for role, (pose, record) in (('first', rows[0]),
                ('maximum_LP_radius', max(rows, key=lambda rp: rp[1]['lp']['common_ball_radius_lp_mm'])),
                ('last', rows[-1])):
            roles.append({'pair': list(pair), 'role': role, 'pose_id': pose['pose_id'],
                'pose_index_one_based': pose['pose_index_one_based'],
                'common_ball_radius_lp_mm': record['lp']['common_ball_radius_lp_mm']})
    selected = {r['pose_id']: r for r in poses if r['pose_id'] in {s['pose_id'] for s in roles}}
    names = sorted({name for pair in pairs for name in pair})
    manifest_path = check.CANDIDATE.parent/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    records = {r['new_geom']: r for r in manifest['new_geoms']}
    xml = ET.parse(check.CANDIDATE).getroot()
    assets = {a.get('name'): a for a in xml.findall('asset/mesh')}
    bindings = {name: check.reader._binding(check.CANDIDATE, xml, assets,
        xml.find('.//geom[@name="'+name+'"]')) for name in names}
    source_paths = [ROOT/records[name]['source_file'] for name in cap.NAMES]
    if check.legacy.sha(check.CANDIDATE) != check.EXPECTED_CANDIDATE:
        raise ValueError('Candidate changed')
    for name in cap.NAMES:
        if check.legacy.sha(ROOT/records[name]['source_file']) != records[name]['source_sha256']:
            raise ValueError('Source changed')
    paths = {Path(__file__), Path(cap.__file__), SCAN/'poses.json.gz', SCAN/'summary.json',
        check.CANDIDATE, manifest_path, Path(check.__file__), Path(check.geometry.__file__),
        Path(check.saved.__file__), Path(check.reader.__file__), Path(check.reader.BUILDER.__file__)}
    paths.update(source_paths)
    paths.update(Path(binding['file']) for binding in bindings.values())
    before = check.legacy.snapshot(paths)
    original_safe = check.saved.SAFE_APIS
    check.saved.SAFE_APIS = {'mj_name2id', 'mj_id2name'}
    try:
        with check.saved.static_only() as audit:
            model = check.mujoco.MjModel.from_xml_path(str(check.CANDIDATE))
            fingerprint = check.saved.fields(model)
            hulls = {name: check.compiled_hull(model, name, bindings[name]) for name in names}
            if fingerprint != check.saved.fields(model):
                raise ValueError('Compiled model changed')
            audit = dict(audit, allowed_api_calls=dict(audit['allowed_api_calls']),
                model_constructor_calls=1, data_constructor_calls=0, new_FK_calls=0,
                model_fields_unchanged=True)
    finally:
        check.saved.SAFE_APIS = original_safe
    source_a, source_b = [check.geometry.read_stl(path) for path in source_paths]
    bvh_a, bvh_b = check.geometry.BVH(source_a), check.geometry.BVH(source_b)
    component_ids = np.asarray(records['robot0_flange_local_01']['source_triangle_ids_zero_based'], dtype=int)
    component_b = source_b[component_ids]
    component_bvh = check.geometry.BVH(component_b)
    flange = hulls['robot0_flange_local_01']
    pose_results = {}
    for pose_id, pose in selected.items():
        rotation = np.asarray(pose['relative_rotation_flange_to_link5'])
        translation = np.asarray(pose['relative_translation_flange_to_link5_m'])
        moved_flange = flange['triangles']@rotation.T+translation
        moved_source = source_b@rotation.T+translation
        moved_component = component_b@rotation.T+translation
        query = check.geometry.surface_query(source_a, moved_flange, bvh_a, flange['bvh'].refit(moved_flange))
        source_a_vs_flange = mixed(query, source_a, flange, rotation.T, -translation@rotation)
        arm_results = {}
        for arm_name in sorted({r['pair'][0] for r in roles if r['pose_id'] == pose_id}):
            arm = hulls[arm_name]
            query = check.geometry.surface_query(arm['triangles'], moved_source, arm['bvh'], bvh_b.refit(moved_source))
            full = mixed(query, source_b, arm, rotation, translation)
            query = check.geometry.surface_query(arm['triangles'], moved_component, arm['bvh'], component_bvh.refit(moved_component))
            part = mixed(query, component_b, arm, rotation, translation)
            part['source_component_triangle_count'] = len(component_ids)
            part['nearest_original_source_STL_face_id_zero_based'] = int(component_ids[query['nearest_face_ids_zero_based'][1]])
            for vertex in part['source_vertex_membership']['least_exterior_or_deepest_inside_source_vertices']:
                vertex['original_full_source_STL_face_ids_zero_based'] = component_ids[vertex['source_face_ids_zero_based']].tolist()
            arm_results[arm_name] = {'arm_hull_vs_full_source_flange': full,
                'arm_hull_vs_assigned_source_flange01_component': part}
        pose_results[pose_id] = {'pose_index_one_based': pose['pose_index_one_based'],
            'body_world_transforms': pose['body_world_transforms'],
            'relative_rotation_flange_to_link5': rotation.tolist(),
            'relative_translation_flange_to_link5_m': translation.tolist(),
            'existing_full_source_STL_query': pose['source_STL_query'],
            'full_source_link5_vs_compiled_flange01': source_a_vs_flange,
            'arm_pair_counterfactuals': arm_results}
        print(json.dumps({'pose_id': pose_id,
            'source_link5_vs_flange01_gap_mm': source_a_vs_flange['continuous_surface_query']['surface_min_distance_m']*1000,
            'source_link5_inside_flange01_vertices': source_a_vs_flange['source_vertex_membership']['strictly_inside_vertex_count'],
            'arm_vs_component1': {name: {'gap_mm': row['arm_hull_vs_assigned_source_flange01_component']['continuous_surface_query']['surface_min_distance_m']*1000,
                'inside_vertices': row['arm_hull_vs_assigned_source_flange01_component']['source_vertex_membership']['strictly_inside_vertex_count'],
                'original_source_face': row['arm_hull_vs_assigned_source_flange01_component']['nearest_original_source_STL_face_id_zero_based']}
                for name, row in arm_results.items()}}), flush=True)
    impossible = any(row['arm_hull_vs_assigned_source_flange01_component']['strict_source_surface_crossing_or_strict_vertex_inside_opposite_hull']
                     for pose in pose_results.values() for row in pose['arm_pair_counterfactuals'].values())
    result = {'schema': 'd3-link5-flange-representative-mixed-source-counterfactuals-v1',
        'candidate_model_sha256': check.EXPECTED_CANDIDATE,
        'selection': 'Exactly first / maximum LP common-ball radius / last anomalous pose for each of the two failed pairs; duplicate pose IDs reused. No pose or design-parameter search.',
        'pair_role_count': len(roles), 'stored_pose_count': len(pose_results),
        'representative_roles': roles, 'poses': pose_results,
        'flange01_only_source_preserving_refinement':
            'EXCLUDED_BY_SOURCE_FLANGE01_SURFACE_INTRUSION_INTO_UNCHANGED_ARM_HULL'
            if impossible else 'PLAUSIBLE_AT_REPRESENTATIVES_NOT_CERTIFIED_OVER_FULL_PATH',
        'call_audit': audit, 'physics_steps': 0, 'forward_calls': 0, 'collision_calls': 0,
        'constraint_solves': 0, 'new_candidate_count': 0, 'new_pose_count': 0,
        'input_sha256_before': before, 'input_sha256_after': check.legacy.snapshot(paths),
        'limits': ['Geometric counterfactuals are not new model acceptance.',
            'Source vertex membership complements complete continuous mixed surface queries; positive surface gap alone does not exclude a contained source component.',
            'Source surfaces and component assignment remain unchanged; no physical penetration depth or closed-source solid conclusion is inferred.',
            'Representatives can falsify flange-only refinement but do not certify any next design over all 921 poses.']}
    if result['input_sha256_before'] != result['input_sha256_after']:
        raise ValueError('Inputs changed')
    check.dump(OUT, result)


if __name__ == '__main__':
    main()
