#!/usr/bin/env python3
"""Read-only 921-pose/6-pair survey; the frozen acceptance entry is unchanged.

Positive projection intervals or verified negative LP dual bounds certify
separation. Independently verified positive interior balls certify overlap.
Every overlap/UNKNOWN pose receives a full factory-STL surface query. Unlike
the frozen acceptance entry, an anomaly is recorded and scanning continues.
"""
import argparse
from collections import Counter
import gzip
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'

import numpy as np
import scipy
import link5_flange_poses as P

BASE_COMMIT = 'dcd6540d9103df83dc24c1eff3cb98f80ab619db'
EXPECTED_PLAN = 'ba767b42f99d3562255909bde5c970d529ba254769029ec1cd7a98faf76af0fb'
SOURCE_FILES = ('meshes/nero/source/link5-dc497f5a.stl',
                'meshes/nero/source/gripper_flange-186697a7.stl')
SOURCE_SHA = ('dc497f5a9bab2d0d314250134d4ec86617abea7205d122db41b765acdb86ae7e',
              '186697a73e7cafd9fd41d96f1eea9b34b444c058b9561fb2e3ea439c835ae2e8')


def dump(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def intervals(rows, predicate):
    """Preserve original plan adjacency; do not merge separated failures."""
    groups = []
    for row in rows:
        if not predicate(row):
            continue
        if not groups or row['pose_index_one_based'] != groups[-1][-1]['pose_index_one_based']+1:
            groups.append([])
        groups[-1].append(row)
    return groups


def interval_record(group, pair=None):
    matching = [(r, p) for r in group for p in r['pairs']
                if p['status'] != 'SEPARATED' and (pair is None or p['pair'] == pair)]
    overlap = [(r, p) for r, p in matching if p['status'] == 'STRICT_CONVEX_OVERLAP']
    deepest = max(overlap, key=lambda rp: rp[1]['lp']['common_ball_radius_lp_mm']) if overlap else None
    centers = np.asarray([p['lp']['common_ball_center_body13_m'] for _, p in overlap])
    return {'start_index_one_based': group[0]['pose_index_one_based'],
            'end_index_one_based': group[-1]['pose_index_one_based'],
            'first_pose_id': group[0]['pose_id'], 'last_pose_id': group[-1]['pose_id'],
            'pose_count': len(group), 'pair_case_count': len(matching),
            'phase_counts': dict(Counter(r['phase'] for r in group)),
            'pair_counts': dict(Counter(' / '.join(p['pair']) for _, p in matching)),
            'maximum_common_ball_radius_mm': deepest[1]['lp']['common_ball_radius_lp_mm'] if deepest else None,
            'maximum_common_ball_pose_id': deepest[0]['pose_id'] if deepest else None,
            'overlap_center_body5_bounds_m': [centers.min(axis=0).tolist(), centers.max(axis=0).tolist()] if len(centers) else None,
            'metric_limit': 'Maximum LP common-ball radius of convex overlap; not physical penetration depth.'}


def status_from_lp(lp):
    if lp['strict_convex_solid_interior_overlap']:
        return 'STRICT_CONVEX_OVERLAP'
    # Retain the frozen negative-dual threshold, with a conservative residual
    # correction rather than trusting an approximately stationary raw dual.
    if lp.get('independent_nonnegative_dual_upper_bound_mm', float('inf')) < -1e-7:
        return 'SEPARATED'
    return 'UNKNOWN_NUMERICAL'


def dual_bound(lp, a, b, rotation, translation):
    """Nonnegative dual weights plus a bounded-center stationarity correction.

    For any common ball with r>=0 its center lies inside both hulls, hence the
    union AABB. Clamp tiny negative dual weights to zero, independently include
    the entire residual dot center, and evaluate in long double. No physical
    acceptance threshold, hull or LP optimizer tolerance changes.
    """
    ld = np.longdouble
    normal = np.asarray(lp['certificate']['normals'], dtype=ld)
    offset = np.asarray(lp['certificate']['offsets_mm'], dtype=ld)
    weights = np.maximum(np.asarray(lp['certificate']['dual_weights'], dtype=ld), ld(0))
    points = np.vstack([a['vertices'], P.transformed(b, rotation, translation)])*1000
    origin = np.asarray(lp['conditioning_origin_body13_mm'], dtype=ld)
    bounds = np.max(np.abs(points.astype(ld)-origin), axis=0)
    residual = weights@normal
    total = weights.sum()
    if total <= 0:
        raise ValueError('No nonnegative dual mass')
    objective = -weights@offset
    correction = np.abs(residual)@bounds
    # Explicit floating-point guard for these finite long-double dot products.
    guard = ld(64)*np.finfo(ld).eps*(np.abs(weights*offset).sum()+correction+ld(1))
    bound = (objective+correction+guard)/total
    return {'independent_nonnegative_dual_upper_bound_mm': float(bound),
            'dual_stationarity_bounded_center_correction_mm': float(correction/total),
            'dual_longdouble_roundoff_guard_mm': float(guard/total),
            'common_ball_center_absolute_conditioned_AABB_mm': [float(x) for x in bounds]}


def run(output):
    start = time.monotonic()
    output = output.resolve()
    if output.exists():
        raise ValueError('Output directory must not exist; old evidence is immutable')
    if P.legacy.sha(P.CANDIDATE) != P.EXPECTED_CANDIDATE or P.legacy.sha(P.PLAN) != EXPECTED_PLAN:
        raise ValueError('Frozen candidate or original plan changed')
    manifest_path = P.CANDIDATE.parent/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    plan = json.loads(P.PLAN.read_text())
    samples = plan['samples']
    if len(samples) != 921 or len({s['sample_id'] for s in samples}) != 921:
        raise ValueError('Expected 921 uniquely named original samples')
    names = [r['new_geom'] for r in manifest['new_geoms']]
    groups = [[r['new_geom'] for r in manifest['new_geoms'] if r['parent_body'] == body]
              for body in P.BODIES]
    if list(map(len, groups)) != [3, 2] or len(names) != 5:
        raise ValueError('Expected exactly three brace/two flange frozen parts')
    xml = ET.parse(P.CANDIDATE).getroot()
    assets = {a.get('name'): a for a in xml.findall('asset/mesh')}
    bindings = {n: P.reader._binding(P.CANDIDATE, xml, assets, xml.find('.//geom[@name="'+n+'"]'))
                for n in names}
    paths = {Path(__file__).resolve(), P.CANDIDATE, P.PLAN, manifest_path,
             Path(P.__file__), Path(P.geometry.__file__), Path(P.lpcheck.__file__),
             Path(P.reader.__file__), Path(P.saved.__file__), Path(P.reader.BUILDER.__file__),
             P.LEGACY/'geometry.py', P.ROOT/'urdf/nero_plus_manufacturer_flange.urdf'}
    paths.update(Path(v['file']) for v in bindings.values())
    urdf = ET.parse(P.ROOT/'urdf/nero_plus_manufacturer_flange.urdf').getroot()
    source_triangles = []
    for name, rel, sha in zip(('link5', 'gripper_flange'), SOURCE_FILES, SOURCE_SHA):
        collision = urdf.find('link[@name="'+name+'"]/collision')
        origin, mesh = collision.find('origin'), collision.find('geometry/mesh')
        source = (P.ROOT/'urdf'/mesh.get('filename')).resolve()
        if (source != P.ROOT/rel or P.legacy.sha(source) != sha or
                origin.get('xyz', '0 0 0') != '0 0 0' or origin.get('rpy', '0 0 0') != '0 0 0' or
                mesh.get('scale', '1 1 1') != '1 1 1'):
            raise ValueError('Factory source binding/coordinates changed')
        vertices, faces = P.reader.BUILDER.read_mesh(source)
        source_triangles.append(vertices[faces])
        paths.add(source)
    before = P.legacy.snapshot(paths)
    source_bvhs = [P.geometry.BVH(t) for t in source_triangles]
    poses = []
    old_safe = P.saved.SAFE_APIS
    P.saved.SAFE_APIS = {'mj_id2name', 'mj_name2id', 'mj_kinematics'}
    try:
        with P.saved.static_only() as audit:
            model = P.mujoco.MjModel.from_xml_path(str(P.CANDIDATE))
            fingerprint = P.saved.fields(model)
            bodyids = [P.mujoco.mj_name2id(model, P.mujoco.mjtObj.mjOBJ_BODY, body) for body in P.BODIES]
            hulls = {n: P.compiled_hull(model, n, bindings[n]) for n in names}
            ancestry = {body: P.ancestry(model, body) for body in P.BODIES}
            for i, sample in enumerate(samples):
                qpos = np.asarray(sample['qpos'])
                if qpos.shape != (model.nq,) or not np.all(np.isfinite(qpos)):
                    raise ValueError('Invalid original qpos at '+sample['sample_id'])
                poses.append({'pose_index_zero_based': i, 'pose_index_one_based': i+1,
                    'pose_id': sample['sample_id'], 'phase': sample['phase'], 'time_s': sample['time_s'],
                    'original_qpos': sample['qpos'],
                    'body_world_transforms': P.fk(model, qpos, bodyids)})
            if fingerprint != P.saved.fields(model):
                raise ValueError('Static analysis changed compiled model fields')
            audit = dict(audit, allowed_api_calls=dict(audit['allowed_api_calls']),
                         model_constructor_calls=1, data_constructor_calls=921,
                         model_fields_unchanged=True, integration_state_unchanged_after_FK=True)
    finally:
        P.saved.SAFE_APIS = old_safe
    output.mkdir(parents=True)
    dump(output/'frozen.json', {'base_commit': BASE_COMMIT,
        'authorization': 'https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6265318780',
        'mode': 'READ_ONLY_RECORD_ALL_ANOMALIES; frozen acceptance entry remains unchanged',
        'candidate_sha256': P.EXPECTED_CANDIDATE, 'plan_sha256': EXPECTED_PLAN,
        'pose_count': 921, 'pairs_per_pose': 6, 'pair_case_count': 5526,
        'projection_positive_threshold_m': P.TOL,
        'strict_common_ball_margin_threshold_mm': 1e-8,
        'negative_dual_threshold_mm': -1e-7, 'thresholds_unchanged': True,
        'source': [{'file': p, 'sha256': s} for p, s in zip(SOURCE_FILES, SOURCE_SHA)],
        'environment': {'python': sys.version, 'executable': sys.executable,
            'mujoco': P.mujoco.__version__, 'numpy': np.__version__, 'scipy': scipy.__version__},
        'input_sha256_before': before, 'call_audit': audit,
        'compiled_hulls': [hulls[n]['audit'] for n in names], 'body_joint_ancestry': ancestry})
    all_pairs = [(hulls[a], hulls[b]) for a in groups[0] for b in groups[1]]
    cached = {}
    certificate_index = 0
    with gzip.open(output/'certificates.jsonl.gz', 'xt') as certs:
        for row in poses:
            rotation, translation = P.relative(row['body_world_transforms'])
            row['relative_rotation_flange_to_link5'] = rotation.tolist()
            row['relative_translation_flange_to_link5_m'] = translation.tolist()
            # Only bit-identical relative transforms share a certificate.
            key = rotation.tobytes()+translation.tobytes()
            if key in cached:
                ref = cached[key]
                row['pairs'] = ref['pairs']
                row['source_STL_query'] = ref['source_STL_query']
                row['source_classification'] = ref['source_classification']
                row['exact_transform_certificate_reused_from_pose'] = ref['pose_id']
            else:
                row['pairs'] = []
                for a, b in all_pairs:
                    record = {'pair': [a['name'], b['name']], 'status': 'SEPARATED'}
                    try:
                        sep = P.separation(a, b, rotation, translation)
                        record['projection_certificate'] = sep
                        if sep['separation_lower_bound_m'] <= P.TOL:
                            result = P.lp(a, b, row['body_world_transforms'])
                            result.update(dual_bound(result, a, b, rotation, translation))
                            record['status'] = status_from_lp(result)
                            record['certificate_index_zero_based'] = certificate_index
                            certs.write(json.dumps({'certificate_index_zero_based': certificate_index,
                                'pose_id': row['pose_id'], 'pair': record['pair'], 'common_ball': result},
                                allow_nan=False)+'\n')
                            certificate_index += 1
                            record['lp'] = {k: v for k, v in result.items() if k not in ('hulls', 'certificate')}
                    except Exception as error:
                        record['status'] = 'UNKNOWN_QUERY_ERROR'
                        record['error'] = type(error).__name__+': '+str(error)
                    row['pairs'].append(record)
                row['source_STL_query'] = None
                row['source_classification'] = 'NOT_REQUIRED_ALL_SIX_PAIRS_SEPARATED'
                if any(p['status'] != 'SEPARATED' for p in row['pairs']):
                    try:
                        query = P.legacy.query_pair(source_triangles, source_bvhs, row['body_world_transforms'])
                        row['source_STL_query'] = query
                        if query['strict_transverse_witness'] is not None:
                            row['source_classification'] = 'SOURCE_TRIANGLE_SURFACES_INTERSECT'
                        elif query['all_surface_search_complete'] and query['surface_min_distance_m'] > P.TOL:
                            row['source_classification'] = 'SOURCE_TRIANGLE_SURFACES_SEPARATED'
                        else:
                            row['source_classification'] = 'UNKNOWN_SOURCE_TOUCH_OR_NUMERICS'
                    except Exception as error:
                        row['source_classification'] = 'UNKNOWN_SOURCE_QUERY_ERROR'
                        row['source_error'] = type(error).__name__+': '+str(error)
                cached[key] = row
            for pair in row['pairs']:
                if pair['status'] == 'STRICT_CONVEX_OVERLAP':
                    pair['source_comparison'] = ('CONVEX_FALSE_CONTACT_AT_SOURCE_SURFACE_LEVEL'
                        if row['source_classification'] == 'SOURCE_TRIANGLE_SURFACES_SEPARATED'
                        else row['source_classification'])
            if row['pose_index_one_based'] % 50 == 0:
                print(json.dumps({'completed_poses': row['pose_index_one_based'],
                                  'anomalies_so_far': sum(any(p['status'] != 'SEPARATED' for p in r.get('pairs', [])) for r in poses[:row['pose_index_one_based']])}), flush=True)
    anomalous = lambda r: any(p['status'] != 'SEPARATED' for p in r['pairs'])
    global_groups = intervals(poses, anomalous)
    clusters = []
    previous_key = None
    for row in poses:
        if not anomalous(row):
            previous_key = None
            continue
        key = (row['phase'], row['source_classification'],
               tuple((tuple(p['pair']), p['status']) for p in row['pairs'] if p['status'] != 'SEPARATED'))
        if key != previous_key:
            clusters.append([])
        clusters[-1].append(row)
        previous_key = key
    by_pair = {}
    for a, b in all_pairs:
        pair = [a['name'], b['name']]
        by_pair[' / '.join(pair)] = [interval_record(g, pair) for g in intervals(poses,
            lambda r: any(p['pair'] == pair and p['status'] != 'SEPARATED' for p in r['pairs']))]
    representatives = {}
    for group in global_groups:
        failures = [(r, p) for r in group for p in r['pairs'] if p['status'] == 'STRICT_CONVEX_OVERLAP']
        reps = [group[0], group[-1]]
        if failures:
            reps.append(max(failures, key=lambda rp: rp[1]['lp']['common_ball_radius_lp_mm'])[0])
        for row in reps:
            if row['pose_id'] in representatives:
                continue
            rotation, translation = P.relative(row['body_world_transforms'])
            queries = []
            for record in row['pairs']:
                if record['status'] == 'SEPARATED':
                    continue
                a, b = (hulls[n] for n in record['pair'])
                queries.append({'pair': record['pair'],
                                'actual_compiled_triangle_query': P.pair_query(a, b, rotation, translation)})
            representatives[row['pose_id']] = {'pose_index_one_based': row['pose_index_one_based'],
                'body_world_transforms': row['body_world_transforms'], 'pair_queries': queries,
                'source_STL_query': row['source_STL_query']}
    after = P.legacy.snapshot(paths)
    if before != after:
        raise ValueError('Frozen input bytes changed during analysis')
    cases = [p for row in poses for p in row['pairs']]
    anomaly_rows = [r for r in poses if anomalous(r)]
    summary = {'schema': 'd3-link5-flange-fullscan-v1', 'base_commit': BASE_COMMIT,
        'status': 'COMPLETE_SURVEY_EXISTING_CANDIDATE_REMAINS_FAIL', 'candidate_acceptance': 'FAIL_UNCHANGED',
        'pose_count': len(poses), 'pair_case_count': len(cases), 'every_pose_has_six_pairs': all(len(r['pairs']) == 6 for r in poses),
        'pair_status_counts': dict(Counter(p['status'] for p in cases)),
        'anomalous_pose_count': len(anomaly_rows),
        'anomaly_phase_counts': dict(Counter(r['phase'] for r in anomaly_rows)),
        'anomaly_source_classifications': dict(Counter(r['source_classification'] for r in anomaly_rows)),
        'anomaly_pair_counts': dict(Counter(' / '.join(p['pair']) for p in cases if p['status'] != 'SEPARATED')),
        'global_continuous_intervals': [interval_record(g) for g in global_groups],
        'homogeneous_phase_pair_classification_clusters': [interval_record(g) for g in clusters],
        'continuous_intervals_by_pair': by_pair,
        'unique_exact_relative_transform_count': len(cached),
        'exact_transform_reused_pose_count': len(poses)-len(cached),
        'full_source_STL_queries_executed': sum(r['source_STL_query'] is not None for r in cached.values()),
        'independent_common_ball_certificates': certificate_index,
        'physics_steps': 0, 'call_audit': audit, 'inputs_unchanged': True,
        'input_sha256_before': before, 'input_sha256_after': after,
        'wall_seconds': time.monotonic()-start,
        'limits': ['Only the six authorized proxy pairs at the 921 original discrete planning poses.',
            'Full source triangles can be non-watertight: surface gap is not source-solid separation.',
            'LP common-ball radius is not physical penetration depth; no dynamics/force/root-cause claim.',
            'Frozen first-failure acceptance entry, prior FAILs, geometry, physics and 1e-10 m threshold unchanged.',
            'Certificates reuse only exactly identical relative R/t bytes; no rounded-pose merging.',
            'The separately recorded nanometre compiled coverage FAIL is not reinterpreted by this survey.']}
    with gzip.open(output/'poses.json.gz', 'xt') as stream:
        json.dump({'poses': poses}, stream, allow_nan=False)
    dump(output/'representatives.json', representatives)
    dump(output/'summary.json', summary)
    print(json.dumps({k: summary[k] for k in ('status', 'pose_count', 'pair_case_count', 'pair_status_counts',
                                             'anomalous_pose_count', 'anomaly_phase_counts')}), flush=True)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    run(parser.parse_args().output)
