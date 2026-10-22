#!/usr/bin/env python3
"""Classify the one stopped planning pose and quantify saved-pose separation.

Reuse stored transforms only. Compile the frozen candidate once to read actual
convex graph faces; no MjData, FK, poses, actions or simulation are generated.
"""
import gzip
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'mjcf/nero_dh116_right.xml').is_file())
sys.path.insert(0, str(ROOT/'examples/right_hand'))
import link5_flange_poses as check

HERE = Path(__file__).resolve().parent
OUTPUT = HERE/'first-failure-classification.json'


def main():
    if OUTPUT.exists():
        raise ValueError('Classification evidence already exists')
    stopped = json.loads((HERE/'summary.json').read_text())
    if stopped['status'] != 'STOPPED_FIRST_REPLACEMENT_INTERFERENCE':
        raise ValueError('Expected the stopped one-candidate result')
    if any(check.legacy.sha(ROOT/p) != h for p, h in stopped['input_sha256_after'].items()):
        raise ValueError('Stopped-run input identity has changed')
    with gzip.open(HERE/'partial-poses.json.gz', 'rt') as stream:
        poses = json.load(stream)['poses']
    failed = poses[-1]
    if failed['id'] != 'm040_quarter3':
        raise ValueError('Unexpected failed planning sample')
    transforms = failed['body_world_transforms']
    rotation, translation = check.relative(transforms)
    manifest = json.loads((check.CANDIDATE.parent/'manifest.json').read_text())
    if check.legacy.sha(check.CANDIDATE) != check.EXPECTED_CANDIDATE:
        raise ValueError('Frozen candidate identity differs')
    paths = {Path(__file__).resolve(), HERE/'summary.json', HERE/'partial-poses.json.gz',
        check.PLAN, check.CANDIDATE, check.CANDIDATE.parent/'manifest.json', check.BASELINE,
        Path(check.__file__), Path(check.legacy.__file__), Path(check.geometry.__file__),
        Path(check.lpcheck.__file__), check.LEGACY/'geometry-results.json.gz',
        ROOT/'urdf/nero_plus_manufacturer_flange.urdf'}
    with gzip.open(check.LEGACY/'geometry-results.json.gz', 'rt') as stream:
        historical = json.load(stream)
    baseline_xml = ET.parse(check.BASELINE).getroot()
    baseline_assets = {a.get('name'): a for a in baseline_xml.findall('asset/mesh')}
    old_hulls, source_triangles, source_bindings = [], {}, {}
    for name, audit in zip(check.OLD, historical['compiled_proxy_audit']):
        if name != audit['geom']:
            raise ValueError('Old graph audit order differs')
        vertices = np.asarray(audit['compiled_body_local_vertices'])
        triangles = vertices[np.asarray(audit['hull_faces_compiled_vertex_ids'])]
        support = vertices[np.asarray(audit['hull_graph_support_compiled_vertex_ids'])]
        binding = check.reader._binding(check.BASELINE, baseline_xml, baseline_assets,
                                       baseline_xml.find('.//geom[@name="'+name+'"]'))
        old_hulls.append({'name': name, 'vertices': support, 'triangles': triangles,
            'bvh': check.geometry.BVH(triangles), 'source': Path(binding['file']),
            'axes': ConvexHull(support).equations[:, :3]})
        paths.add(Path(binding['file']))
    source_bindings['visual_obj'] = [check.reader._binding(check.BASELINE, baseline_xml, baseline_assets,
        baseline_xml.find('.//geom[@name="'+name+'"]')) for name in check.legacy.VISUAL]
    source_triangles['visual_obj'] = [check.reader.mesh_triangles(b) for b in source_bindings['visual_obj']]
    paths.update(Path(b['file']) for b in source_bindings['visual_obj'])
    urdf = ET.parse(ROOT/'urdf/nero_plus_manufacturer_flange.urdf').getroot()
    source_bindings['urdf_stl'], source_triangles['urdf_stl'] = [], []
    for name in ('link5', 'gripper_flange'):
        collision = urdf.find('link[@name="'+name+'"]/collision')
        origin, mesh = collision.find('origin'), collision.find('geometry/mesh')
        if origin.get('xyz', '0 0 0') != '0 0 0' or origin.get('rpy', '0 0 0') != '0 0 0' or mesh.get('scale', '1 1 1') != '1 1 1':
            raise ValueError('Unexpected factory source transform')
        path = (ROOT/'urdf'/mesh.get('filename')).resolve()
        vertices, faces = check.reader.BUILDER.read_mesh(path)
        source_triangles['urdf_stl'].append(vertices[faces])
        source_bindings['urdf_stl'].append({'file': str(path.relative_to(ROOT)), 'sha256': check.legacy.sha(path)})
        paths.add(path)
    candidate_xml = ET.parse(check.CANDIDATE).getroot()
    assets = {a.get('name'): a for a in candidate_xml.findall('asset/mesh')}
    bindings = {r['new_geom']: check.reader._binding(check.CANDIDATE, candidate_xml, assets,
        candidate_xml.find('.//geom[@name="'+r['new_geom']+'"]')) for r in manifest['new_geoms']}
    paths.update(Path(b['file']) for b in bindings.values())
    before = check.legacy.snapshot(paths)
    original_safe = check.saved.SAFE_APIS
    check.saved.SAFE_APIS = {'mj_name2id', 'mj_id2name'}
    try:
        with check.saved.static_only() as audit:
            model = check.mujoco.MjModel.from_xml_path(str(check.CANDIDATE))
            fingerprint = check.saved.fields(model)
            hulls = {name: check.compiled_hull(model, name, b) for name, b in bindings.items()}
            if fingerprint != check.saved.fields(model):
                raise ValueError('Compiled model changed during graph extraction')
            audit = dict(audit, allowed_api_calls=dict(audit['allowed_api_calls']),
                model_constructor_calls=1, data_constructor_calls=0, model_fields_unchanged=True)
    finally:
        check.saved.SAFE_APIS = original_safe
    source_queries = {label: check.legacy.query_pair(triangles, [check.geometry.BVH(t) for t in triangles], transforms)
                      for label, triangles in source_triangles.items()}
    old_query = check.pair_query(*old_hulls, rotation, translation)
    old_lp = check.lp(*old_hulls, transforms)
    bad_record = next(r for r in failed['replacement_pairs'] if r.get('common_ball_certificate', {}).get('strict_convex_solid_interior_overlap'))
    a, b = (hulls[name] for name in bad_record['pair'])
    face_a, face_b = bad_record['surface_query']['nearest_face_ids_zero_based']
    recomputed_triangles = [a['triangles'][face_a], b['triangles'][face_b]@rotation.T+translation]
    triangle_error = max(float(np.max(np.abs(actual-expected))) for actual, expected in zip(
        recomputed_triangles, bad_record['surface_query']['closest_triangles_body5_m']))
    if triangle_error > 1e-14:
        raise ValueError('Failed actual-graph witness differs from frozen candidate')
    minima = {}
    for category in ('cached_RK4', 'saved_post_FK'):
        selected = [r for r in poses if r['category'] == category]
        entries = [(p['separation']['separation_lower_bound_m'], r, p)
                   for r in selected for p in r['replacement_pairs']]
        best, queried, pruned = None, 0, 0
        for lower, row, pair in sorted(entries, key=lambda e: (e[0], e[1]['id'], e[2]['pair'])):
            if best is not None and lower > best['surface_query']['surface_min_distance_m']+1e-12:
                pruned += 1
                continue
            rot, trans = check.relative(row['body_world_transforms'])
            query = check.pair_query(*(hulls[name] for name in pair['pair']), rot, trans)
            queried += 1
            if best is None or query['surface_min_distance_m'] < best['surface_query']['surface_min_distance_m']:
                best = {'pose_id': row['id'], 'pair': pair['pair'], 'surface_query': query,
                    'body_world_transforms': row['body_world_transforms'],
                    'relative_rotation_flange_to_link5': rot.tolist(),
                    'relative_translation_flange_to_link5_m': trans.tolist()}
        minima[category] = {'pose_count': len(selected), 'enumerated_pair_count': len(entries),
            'all_solid_separation_certified': all(p['separation']['classification'].startswith('CERTIFIED_CONVEX_SOLIDS_SEPARATED') for _, _, p in entries),
            'minimum_projection_separation_lower_bound_m': min(e[0] for e in entries),
            'full_surface_queries': queried, 'pruned_by_certified_distance_lower_bound': pruned,
            'minimum': best}
        print(json.dumps({'category': category, 'minimum_gap_mm': best['surface_query']['surface_min_distance_m']*1000,
                          'queries': queried, 'pruned': pruned}), flush=True)
    plan = json.loads(check.PLAN.read_text())
    index = next(i for i, s in enumerate(plan['samples']) if s['sample_id'] == failed['id'])
    raw_sample, previous, target = plan['samples'][index], plan['commands'][39], plan['commands'][40]
    expected = .25*np.asarray(previous['target_qpos_18'])+.75*np.asarray(target['target_qpos_18'])
    if raw_sample['qpos'] != failed['full_qpos'] or not np.array_equal(expected, raw_sample['qpos']):
        raise ValueError('Raw existing planning sample binding differs')
    result = {'schema': 'd3-link5-flange-local-first-failure-classification-v1',
        'candidate_model_sha256': check.EXPECTED_CANDIDATE,
        'pose_id': failed['id'], 'body_world_transforms': transforms,
        'relative_rotation_flange_to_link5': rotation.tolist(),
        'relative_translation_flange_to_link5_m': translation.tolist(),
        'classification': 'SOURCE_SURFACES_SEPARATED_WITH_REMAINING_CONVEX_PROXY_OVERLAP'
            if all(q['all_surface_search_complete'] and q['surface_min_distance_m'] > 1e-10 for q in source_queries.values())
            else 'SOURCE_STRICT_SURFACE_CROSSING' if any(q['strict_transverse_witness'] is not None for q in source_queries.values())
            else 'UNKNOWN_SOURCE_SURFACE_CLASSIFICATION',
        'source_bindings': source_bindings, 'source_full_continuous_triangle_queries': source_queries,
        'baseline_actual_compiled_graph_pair': {'pair': list(check.OLD), 'surface_query': old_query,
                                                'common_ball_certificate': old_lp},
        'candidate_failed_actual_compiled_graph_pair': bad_record,
        'stored_new_graph_witness_recomputed_max_abs_error_m': triangle_error,
        'source_solid_containment': 'UNKNOWN_LINK5_SOURCE_NOT_CLOSED_MANIFOLD',
        'existing_plan_raw_binding': {'plan_sha256': check.legacy.sha(check.PLAN),
            'sample_index_zero_based': index, 'raw_sample': raw_sample,
            'source_previous_command': previous, 'source_target_command': target,
            'target_interpolation_fraction': .75, 'exact_qpos_reconstruction': True,
            'note': 'The 13D source command rows are copied unchanged. quarter3 is the existing 75% target-qpos interpolant, not a newly generated executable action.'},
        'cached_and_selected_post_minima': minima,
        'original_static_validation_status': stopped['status'],
        'coverage': {'cached_RK4_completed': 87, 'saved_post_FK_completed': 13,
            'planning_poses_fully_checked': 163, 'first_failed_planning_sample_index_one_based': 164,
            'planning_poses_after_failure_not_checked': 757, 'planning_total': 921},
        'physics_steps': 0, 'call_audit': audit, 'new_FK_calls': 0,
        'input_sha256_before': before, 'input_sha256_after': check.legacy.snapshot(paths),
        'limitations': ['One failed stored planning pose only; no planning validation resumed.',
            'Positive full-source surface clearance excludes triangle crossings but cannot establish source-solid containment because link5 source is not closed manifold.',
            'Common-ball witness radius quantifies convex representation overlap, not physical penetration depth.',
            'Quantitative minima cover exactly the 87 cached RK4 and 13 saved post-FK poses; the overall 921-pose path remains failed and partially UNKNOWN.']}
    if result['input_sha256_before'] != result['input_sha256_after']:
        raise ValueError('Offline classification inputs changed')
    check.dump(OUTPUT, result)
    print(json.dumps({'classification': result['classification'],
        'source_gaps_mm': {label: q['surface_min_distance_m']*1000 for label, q in source_queries.items()},
        'baseline_lp_radius_mm': old_lp['common_ball_radius_lp_mm'],
        'new_lp_radius_mm': bad_record['common_ball_certificate']['common_ball_radius_lp_mm']}), flush=True)


if __name__ == '__main__':
    main()
