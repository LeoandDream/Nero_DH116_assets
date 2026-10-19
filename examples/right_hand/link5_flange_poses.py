#!/usr/bin/env python3
"""Validate one frozen local replacement at existing poses; static FK only.

The historical triangle BVH and independently verified common-ball LP are
imported, unchanged. A projection interval on actual compiled hull vertices
is sufficient to prove solid separation; full surface queries are used only
when needed and to certify the global minimum separation witness.
"""
import argparse
from collections import Counter
import gzip
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'examples/right_hand'))
import proxy76_contacts as saved
import proxy76_validate as reader
import mujoco

LEGACY = ROOT/'examples/right_hand/evidence/d3_link5_flange'
legacy = reader._module('unchanged_link5_flange_geometry', LEGACY/'geometry.py')
geometry, lpcheck = legacy.geometry, legacy.lpcheck
BODIES, OLD = legacy.BODIES, legacy.PAIR
BASELINE = legacy.CANDIDATE
CANDIDATE = ROOT/'examples/right_hand/candidates/link5_flange_local/model.xml'
EXPECTED_CANDIDATE = '1b2530d923b446605492b43652dd5a3200fd755674ba17a54f0c961cde362b39'
FROZEN = legacy.FROZEN
PLAN = ROOT/'examples/right_hand/evidence/d3_white_bowl_001/candidate.json'
CACHED_STEPS = tuple(range(1315, 1402))
POST_STEPS = (1313, 1314, 1315, 1316, 1320, 1321, 1322, 1325, 1326,
              1327, 1399, 1400, 1401)
TOL = 1e-10


def read_rows(path, selected):
    with gzip.open(path, 'rt') as stream:
        return {r['step']: r for line in stream
                if (r := json.loads(line))['step'] in selected}


def dump(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def compiled_hull(model, name, binding):
    """Exact historical mesh_graph extraction, without modifying old evidence.

    MuJoCo 3.2.3 user_mesh.cc lines 1496/1509/1568/1589 define this layout:
    https://raw.githubusercontent.com/google-deepmind/mujoco/3.2.3/src/user/user_mesh.cc
    """
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if gid < 0:
        raise ValueError('Missing compiled geom: '+name)
    mid = int(model.geom_dataid[gid])
    start, count = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
    vertices = np.asarray(model.mesh_vert[start:start+count], dtype=float)
    rotation = reader.BUILDER.quaternion_matrix(' '.join(map(str, model.geom_quat[gid])))
    vertices = vertices@rotation.T+model.geom_pos[gid]
    adr = int(model.mesh_graphadr[mid])
    nv, nf = map(int, model.mesh_graph[adr:adr+2])
    support = np.asarray(model.mesh_graph[adr+2+nv:adr+2+2*nv], dtype=int)
    face_adr = adr+2+3*nv+3*nf
    faces = np.asarray(model.mesh_graph[face_adr:face_adr+3*nf], dtype=int).reshape(-1, 3)
    if (adr < 0 or faces.min() < 0 or faces.max() >= len(vertices) or
            set(support) != set(faces.reshape(-1))):
        raise ValueError('Invalid actual compiled convex mesh graph: '+name)
    points, triangles = vertices[support], vertices[faces]
    normals = ConvexHull(points).equations[:, :3]
    normals = np.unique(np.round(normals, 12), axis=0)
    return {'name': name, 'vertices': points, 'triangles': triangles,
            'bvh': geometry.BVH(triangles), 'axes': normals,
            'source': Path(binding['file']),
            'audit': {'geom': name, 'mesh_id': mid, 'graph_adr': adr,
                'compiled_vertices_count': count, 'support_vertex_ids': support.tolist(),
                'face_vertex_ids': faces.tolist(),
                'compiled_body_local_support_vertices_m': points.tolist(),
                'body_id': int(model.geom_bodyid[gid]),
                'body': mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                         int(model.geom_bodyid[gid]))}}


def transformed(hull, rotation, translation):
    return hull['vertices']@rotation.T+translation


def separation(a, b, rotation, translation):
    """One separating axis is a solid-separation proof, not a distance estimate."""
    av, bv = a['vertices'], transformed(b, rotation, translation)
    axes = np.vstack([np.eye(3), a['axes'], b['axes']@rotation.T])
    axes /= np.linalg.norm(axes, axis=1)[:, None]
    ap, bp = av@axes.T, bv@axes.T
    gaps_ab, gaps_ba = bp.min(axis=0)-ap.max(axis=0), ap.min(axis=0)-bp.max(axis=0)
    gaps, reverse = np.maximum(gaps_ab, gaps_ba), gaps_ba > gaps_ab
    best = int(np.argmax(gaps))
    axis = axes[best]*(-1 if reverse[best] else 1)
    aa, bb = av@axis, bv@axis
    left, right = float(aa.max()), float(bb.min())
    gap = right-left
    return {'classification': 'CERTIFIED_CONVEX_SOLIDS_SEPARATED' if gap > TOL
            else 'NO_SEPARATING_AXIS_IN_TESTED_SET',
            'separation_lower_bound_m': max(0., gap),
            'tested_axis_count': len(axes), 'axis_body5': axis.tolist(),
            'first_support_max_m': left, 'second_support_min_m': right,
            'first_support_vertex_local_index': int(np.argmax(aa)),
            'second_support_vertex_local_index': int(np.argmin(bb)),
            'positive_separation_threshold_m': TOL}


def pair_query(a, b, rotation, translation):
    moved = b['triangles']@rotation.T+translation
    query = geometry.surface_query(a['triangles'], moved, a['bvh'], b['bvh'].refit(moved))
    query['coordinate_frame'] = BODIES[0]
    query['closest_triangles_body5_m'] = [
        a['triangles'][query['nearest_face_ids_zero_based'][0]].tolist(),
        moved[query['nearest_face_ids_zero_based'][1]].tolist()]
    return query


def lp(a, b, transforms):
    sources = {body: {'vertices': hull['vertices'], 'source': hull['source'],
                      'geom_name': hull['name']}
               for body, hull in zip(BODIES, (a, b))}
    return lpcheck.check_case(sources, transforms)


def relative(transforms):
    a, b = (transforms[name] for name in BODIES)
    ra, rb = np.asarray(a['rotation']), np.asarray(b['rotation'])
    pa, pb = np.asarray(a['translation_m']), np.asarray(b['translation_m'])
    return ra.T@rb, ra.T@(pb-pa)


def ancestry(model, body):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    result = []
    while bid:
        first, count = int(model.body_jntadr[bid]), int(model.body_jntnum[bid])
        for jid in range(first, first+count):
            result.append({'joint': mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid),
                           'qpos_address': int(model.jnt_qposadr[jid])})
        bid = int(model.body_parentid[bid])
    return result[::-1]


def fk(model, qpos, bodyids):
    data = mujoco.MjData(model)
    data.qpos[:] = qpos[:model.nq]
    keys = ('qpos', 'qvel', 'qacc', 'ctrl', 'act', 'qacc_warmstart')
    protected = {k: np.array(getattr(data, k), copy=True) for k in keys}
    time_before = data.time
    mujoco.mj_kinematics(model, data)
    if data.time != time_before or any(not np.array_equal(v, getattr(data, k))
                                       for k, v in protected.items()):
        raise ValueError('Static FK changed protected integration state')
    return {name: {'rotation': data.xmat[bid].reshape(3, 3).tolist(),
                   'translation_m': data.xpos[bid].tolist()}
            for name, bid in zip(BODIES, bodyids)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('Output directory must be new')
    manifest_path = CANDIDATE.parent/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    if legacy.sha(BASELINE) != legacy.EXPECTED_CANDIDATE:
        raise ValueError('Baseline fingerprint differs')
    if legacy.sha(CANDIDATE) != EXPECTED_CANDIDATE or EXPECTED_CANDIDATE != manifest['candidate_model_sha256']:
        raise ValueError('Frozen candidate fingerprint differs')
    groups = {name: [r['new_geom'] for r in manifest['new_geoms'] if r['old_geom'] == name]
              for name in OLD}
    if not all(groups.values()) or set(r['old_geom'] for r in manifest['new_geoms']) != set(OLD):
        raise ValueError('Manifest replacement mapping differs from the authorized pair')
    selected = set(CACHED_STEPS)|set(POST_STEPS)
    states = read_rows(FROZEN/'state.jsonl.gz', selected)
    contacts = read_rows(FROZEN/'contacts.jsonl.gz', set(CACHED_STEPS))
    plan = json.loads(PLAN.read_text())
    if set(states) != selected or set(contacts) != set(CACHED_STEPS) or len(plan['samples']) != 921:
        raise ValueError('Missing frozen pose inputs')
    paths = {Path(__file__).resolve(), manifest_path, BASELINE, CANDIDATE, PLAN,
        FROZEN/'state.jsonl.gz', FROZEN/'contacts.jsonl.gz', LEGACY/'geometry.py',
        Path(geometry.__file__), Path(lpcheck.__file__), Path(saved.__file__),
        Path(reader.__file__), Path(reader.BUILDER.__file__), LEGACY/'geometry-results.json.gz'}
    hulls, bindings = {}, {}
    all_names = {BASELINE: list(OLD), CANDIDATE: [n for names in groups.values() for n in names]}
    for path, names in all_names.items():
        xml = ET.parse(path).getroot()
        assets = {a.get('name'): a for a in xml.findall('asset/mesh')}
        bindings[path] = {n: reader._binding(path, xml, assets, xml.find('.//geom[@name="'+n+'"]'))
                          for n in names}
        paths.update(Path(v['file']) for v in bindings[path].values())
    # Original source visuals are read directly from the unchanged baseline.
    xml = ET.parse(BASELINE).getroot()
    assets = {a.get('name'): a for a in xml.findall('asset/mesh')}
    source_bindings = [reader._binding(BASELINE, xml, assets, xml.find('.//geom[@name="'+n+'"]'))
                       for n in legacy.VISUAL]
    source_triangles = [reader.mesh_triangles(b) for b in source_bindings]
    source_bvhs = [geometry.BVH(t) for t in source_triangles]
    paths.update(Path(b['file']) for b in source_bindings)
    before = legacy.snapshot(paths)
    poses = []
    for step in CACHED_STEPS:
        matches = [c for c in contacts[step]['contacts'] if set(c['geom']) == set(OLD)]
        if len(matches) != 1:
            raise ValueError('Expected exactly one frozen pair contact at '+str(step))
        c = matches[0]
        transforms = {body: {'rotation': np.asarray(p['rotation_world']).reshape(3, 3).tolist(),
                             'translation_m': p['position_world_m']}
                      for body, p in zip(c['body'], c['cached_body_pose'])}
        poses.append({'id': 'cached_RK4_'+str(step), 'category': 'cached_RK4', 'step': step,
            'phase': states[step]['phase'], 'time_s': states[step]['time_s'],
            'saved_contact_distance_m': c['distance_m'], 'body_world_transforms': transforms})
    original_safe = saved.SAFE_APIS
    saved.SAFE_APIS = {'mj_kinematics', 'mj_name2id', 'mj_id2name'}
    max_fk_difference = 0.
    try:
        with saved.static_only() as api_audit:
            models = {p: mujoco.MjModel.from_xml_path(str(p)) for p in all_names}
            fingerprints = {p: saved.fields(m) for p, m in models.items()}
            ids = {p: [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in BODIES]
                   for p, m in models.items()}
            for path, names in all_names.items():
                hulls[path] = {n: compiled_hull(models[path], n, bindings[path][n]) for n in names}
            baseline_ancestry = {b: ancestry(models[BASELINE], b) for b in BODIES}
            candidate_ancestry = {b: ancestry(models[CANDIDATE], b) for b in BODIES}
            if baseline_ancestry != candidate_ancestry:
                raise ValueError('Candidate changed relevant kinematic ancestry')
            fk_specs = [{'id': 'saved_post_'+str(s), 'category': 'saved_post_FK', 'step': s,
                         'phase': states[s]['phase'], 'time_s': states[s]['time_s'],
                         'full_qpos': states[s]['full_qpos'][:18]} for s in POST_STEPS]
            fk_specs += [{'id': s['sample_id'], 'category': 'existing_planning_FK',
                          'phase': s['phase'], 'time_s': s['time_s'], 'full_qpos': s['qpos']}
                         for s in plan['samples']]
            for spec in fk_specs:
                transforms = {p: fk(m, spec['full_qpos'], ids[p]) for p, m in models.items()}
                difference = max(float(np.max(np.abs(np.asarray(transforms[BASELINE][b][key])-
                                    np.asarray(transforms[CANDIDATE][b][key]))))
                                 for b in BODIES for key in ('rotation', 'translation_m'))
                max_fk_difference = max(max_fk_difference, difference)
                if difference > 1e-14:
                    raise ValueError('Candidate changed saved-pose FK')
                poses.append(dict(spec, body_world_transforms=transforms[CANDIDATE]))
            if any(fingerprints[p] != saved.fields(m) for p, m in models.items()):
                raise ValueError('Compiled model fields changed')
            api_audit = dict(api_audit, allowed_api_calls=dict(api_audit['allowed_api_calls']),
                model_constructor_calls=2, data_constructor_calls=2*len(fk_specs),
                model_public_fields_unchanged=True, protected_state_buffers_unchanged_after_FK=True)
    finally:
        saved.SAFE_APIS = original_safe
    old_a, old_b = (hulls[BASELINE][n] for n in OLD)
    new_pairs = [(hulls[CANDIDATE][a], hulls[CANDIDATE][b])
                 for a in groups[OLD[0]] for b in groups[OLD[1]]]
    pending = []
    for index, row in enumerate(poses):
        transforms = row['body_world_transforms']
        rotation, translation = relative(transforms)
        row['relative_rotation_flange_to_link5'] = rotation.tolist()
        row['relative_translation_flange_to_link5_m'] = translation.tolist()
        old_sep = separation(old_a, old_b, rotation, translation)
        row['baseline_pair'] = {'separation': old_sep}
        # Cached RK4 cases get an independently rechecked solid-overlap LP.
        # The 921 planning cases receive the same read-only interval test; their
        # purpose is replacement separation, so baseline unresolved is explicit.
        if row['category'] != 'existing_planning_FK' and old_sep['separation_lower_bound_m'] <= TOL:
            row['baseline_pair']['common_ball_certificate'] = lp(old_a, old_b, transforms)
        row['replacement_pairs'] = []
        for a, b in new_pairs:
            certificate = separation(a, b, rotation, translation)
            record = {'pair': [a['name'], b['name']], 'separation': certificate}
            if certificate['separation_lower_bound_m'] <= TOL:
                record['surface_query'] = pair_query(a, b, rotation, translation)
                record['common_ball_certificate'] = lp(a, b, transforms)
                query, common = record['surface_query'], record['common_ball_certificate']
                if common['dual_upper_bound_mm'] < -1e-7 and query['all_surface_search_complete'] and query['surface_min_distance_m'] > TOL:
                    certificate['classification'] = 'CERTIFIED_CONVEX_SOLIDS_SEPARATED_BY_COMMON_BALL_DUAL'
                    certificate['negative_common_ball_dual_upper_bound_mm'] = common['dual_upper_bound_mm']
                if common['strict_convex_solid_interior_overlap'] or query['strict_transverse_witness'] is not None:
                    row['replacement_pairs'].append(record)
                    args.output.mkdir(parents=True)
                    dump(args.output/'summary.json', {
                        'schema': 'd3-link5-flange-local-static-poses-summary-v1',
                        'candidate_model_sha256': EXPECTED_CANDIDATE,
                        'status': 'STOPPED_FIRST_REPLACEMENT_INTERFERENCE',
                        'physics_steps': 0, 'call_audit': api_audit,
                        'first_failure_pose': row, 'completed_pose_count': index,
                        'planned_pose_count': len(poses),
                        'input_sha256_before': before, 'input_sha256_after': legacy.snapshot(paths),
                        'limitation': 'Stopped at first actual compiled replacement-pair overlap/crossing; unprocessed poses are UNKNOWN. No candidate regeneration or geometry changes.'})
                    with gzip.open(args.output/'partial-poses.json.gz', 'wt') as stream:
                        json.dump({'poses': poses[:index+1]}, stream, allow_nan=False)
                    print(json.dumps({'status': 'STOPPED_FIRST_REPLACEMENT_INTERFERENCE',
                                      'pose_id': row['id'], 'pair': record['pair']}), flush=True)
                    raise SystemExit(2)
            row['replacement_pairs'].append(record)
            pending.append((certificate['separation_lower_bound_m'], index,
                            len(row['replacement_pairs'])-1, a, b))
        if row['category'] != 'existing_planning_FK':
            row['source_visual_surface_query'] = legacy.query_pair(source_triangles, source_bvhs, transforms)
        if index % 25 == 0:
            print(json.dumps({'completed_pose_count': index+1, 'total_pose_count': len(poses)}), flush=True)
    # A projection interval is a Euclidean-distance lower bound. Search the
    # lowest bounds first, then prune only when the lower bound exceeds the best
    # exact continuous surface distance. This certifies the global minimum over
    # every replacement pair at every enumerated pose without duplicate work.
    minimum = None
    exact_queries = 0
    for lower, i, j, a, b in sorted(pending, key=lambda p: (p[0], p[1], p[2])):
        if minimum is not None and lower > minimum['surface_query']['surface_min_distance_m']+1e-12:
            continue
        row, record = poses[i], poses[i]['replacement_pairs'][j]
        if 'surface_query' not in record:
            rotation, translation = relative(row['body_world_transforms'])
            record['surface_query'] = pair_query(a, b, rotation, translation)
            exact_queries += 1
        if minimum is None or record['surface_query']['surface_min_distance_m'] < minimum['surface_query']['surface_min_distance_m']:
            minimum = {'pose_id': row['id'], 'category': row['category'], 'pair': record['pair'],
                       'body_world_transforms': row['body_world_transforms'],
                       'relative_rotation_flange_to_link5': row['relative_rotation_flange_to_link5'],
                       'relative_translation_flange_to_link5_m': row['relative_translation_flange_to_link5_m'],
                       'separation': record['separation'], 'surface_query': record['surface_query']}
    after = legacy.snapshot(paths)
    if before != after:
        raise ValueError('Input files changed during static pose analysis')
    counts = Counter(r['category'] for r in poses)
    failed = [{'pose_id': r['id'], 'pair': p['pair'], 'separation': p['separation']}
              for r in poses for p in r['replacement_pairs']
              if not p['separation']['classification'].startswith('CERTIFIED_CONVEX_SOLIDS_SEPARATED')]
    cached_lp = [r['baseline_pair'].get('common_ball_certificate') for r in poses if r['category'] == 'cached_RK4']
    ancestors_a, ancestors_b = ([r['joint'] for r in baseline_ancestry[b]] for b in BODIES)
    relative_joints = [r for r in baseline_ancestry[BODIES[1]] if r['joint'] not in ancestors_a]
    summary = {'schema': 'd3-link5-flange-local-static-poses-summary-v1',
        'candidate_model_sha256': legacy.sha(CANDIDATE), 'baseline_model_sha256': legacy.sha(BASELINE),
        'physics_steps': 0, 'call_audit': api_audit, 'counts': dict(counts),
        'replacement_geoms': groups, 'replacement_cross_pair_count_per_pose': len(new_pairs),
        'all_replacement_pairs_solid_separated': not failed,
        'uncertified_replacement_pair_count': len(failed), 'uncertified_replacement_pairs': failed,
        'baseline_cached_strict_overlap_count': sum(bool(c and c['strict_convex_solid_interior_overlap']) for c in cached_lp),
        'baseline_cached_lp_checked_count': sum(c is not None for c in cached_lp),
        'source_cached_and_selected_post_count': sum('source_visual_surface_query' in r for r in poses),
        'source_cached_and_selected_post_positive_gap_count': sum(
            q['all_surface_search_complete'] and q['surface_min_distance_m'] > TOL
            for r in poses if (q := r.get('source_visual_surface_query'))),
        'source_min_surface_gap_m': min(r['source_visual_surface_query']['surface_min_distance_m']
            for r in poses if 'source_visual_surface_query' in r),
        'global_minimum_over_all_enumerated_pose_pair_surfaces': minimum,
        'global_minimum_proof': {'method': 'All actual compiled convex support vertices projected onto tested unit axes. Positive intervals prove solid separation and lower-bound Euclidean surface distance. Existing complete triangle BVH queries evaluate all bounds capable of beating the best witness; all other enumerated pairs are pruned only by certified lower bounds.',
            'enumerated_pose_pair_count': len(pending), 'additional_full_surface_queries': exact_queries,
            'numerical_absolute_distance_tolerance_m': 1e-12},
        'baseline_candidate_max_FK_transform_abs_difference': max_fk_difference,
        'body_joint_ancestry': baseline_ancestry, 'relative_transform_joint_dependencies': relative_joints,
        'hand_joint_motion_affects_this_pair_relative_transform': False,
        'inputs_unchanged': True, 'input_sha256_before': before, 'input_sha256_after': after,
        'limitations': ['Existing planning sample poses only; no new poses/actions, parameter scans, dynamics, forward, collision generation, constraints, filtering, or physical attempts.',
            'Cached RK4 contact transforms and saved post-qpos FK are distinct cases.',
            'Source visual link5 is not a closed manifold. Positive source triangle gap does not prove source-solid containment or physical penetration status.',
            'The common-ball radius describes the convex representation and is not physical penetration depth.',
            'Baseline planning cases without a separating interval are unresolved; they do not silently inherit the cached overlap classification.',
            'Hand self-contact checks are not repeated because hand geometry and joints are frozen; global new-overlap risk relies separately on the compiled replacement subset certificate.']}
    args.output.mkdir(parents=True)
    dump(args.output/'summary.json', summary)
    dump(args.output/'compiled-hulls.json', {str(p.relative_to(ROOT)): [v['audit'] for v in hs.values()]
                                           for p, hs in hulls.items()})
    with gzip.open(args.output/'poses.json.gz', 'wt') as stream:
        json.dump({'schema': 'd3-link5-flange-local-static-poses-v1', 'poses': poses}, stream, allow_nan=False)
    print(json.dumps({k: summary[k] for k in ('counts', 'all_replacement_pairs_solid_separated',
        'baseline_cached_strict_overlap_count', 'source_min_surface_gap_m')}), flush=True)


if __name__ == '__main__':
    main()
