#!/usr/bin/env python3
"""Conditional DH116 wrist-vs-other-geometry preflight, without integration.

Reuses the frozen scene resolver, exact migration audit and original 921 poses.
No link5-flange pair is queried. AABB certificates contain complete compiled
geometry; remaining pairs reuse compiled-hull projections and verified LP.
The first blocked pose stops this preflight and prohibits dynamics.
"""
import argparse
from collections import Counter
import gzip
import json
import itertools
import os
from pathlib import Path
import time

for variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[variable] = '1'

import numpy as np
import wrist_reuse as W
from white_bowl_frozen78 import load_frozen_inputs, build_schedule, schedule_summary

P, mj = W.P, W.mj
EXPECTED_MODEL = '9604a4a6935a34496f93e223219f687b4df8b7aa83037dc208fc9f3499eb05c9'
RISK_GEOMS = {'root_0300mm_entire_piece': 'robot0_wrist_reuse_gripper_flange_123',
              'outer_fill_217mm_entire_piece': 'robot0_wrist_reuse_gripper_flange_103'}
AUTHORIZATION = 'https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6269815750'


def world_aabbs(model, data):
    """Enclose the WHOLE compiled geometry, including its principal-axis pose."""
    local = np.asarray(model.geom_aabb)
    rotations = data.geom_xmat.reshape(-1, 3, 3)
    center = np.einsum('nij,nj->ni', rotations, local[:, :3])+data.geom_xpos
    half = np.einsum('nij,nj->ni', np.abs(rotations), local[:, 3:])
    # Conservative arithmetic guard on bounds, not a physical/tolerance change.
    guard = 64*np.finfo(float).eps*np.maximum(1., np.abs(center)+half)
    return center-half-guard, center+half+guard


def distance_status(distance):
    if not np.isfinite(distance):
        return 'UNKNOWN_CLEARANCE'
    if distance < -P.TOL:
        return 'INTERFERENCE'
    return 'SEPARATED' if distance > P.TOL else 'UNKNOWN_CLEARANCE'


def load_polytopes(model, xml, ids):
    assets = {m.get('name'): m for m in xml.findall('asset/mesh')}
    result = {}
    for gid in sorted(set(ids)):
        name = model.geom(gid).name
        if model.geom_type[gid] == mj.mjtGeom.mjGEOM_MESH and name:
            geom = xml.find('.//geom[@name="'+name+'"]')
            binding = P.reader._binding(W.CANDIDATE, xml, assets, geom)
            hull = P.compiled_hull(model, name, binding)
            hull['representation'] = 'actual_compiled_mesh_graph'
        else:
            # Exact for boxes; conservative enclosing OBB for other primitives.
            center, half = model.geom_aabb[gid, :3], model.geom_aabb[gid, 3:]
            points = center+np.array(list(itertools.product((-1,1), repeat=3)))*half
            rotation = P.reader.BUILDER.quaternion_matrix(' '.join(map(str, model.geom_quat[gid])))
            hull = {'vertices': points@rotation.T+model.geom_pos[gid], 'axes': np.eye(3)@rotation.T,
                    'name': name or '@unnamed', 'source': W.RUN/'model.xml',
                    'representation': 'exact_box' if model.geom_type[gid] == mj.mjtGeom.mjGEOM_BOX else 'conservative_primitive_OBB'}
        result[gid] = hull
    return result


def relative_pair(model, data, first, second):
    a, b = (int(model.geom_bodyid[g]) for g in (first, second))
    ra, rb = data.xmat[a].reshape(3,3), data.xmat[b].reshape(3,3)
    return ra.T@rb, ra.T@(data.xpos[b]-data.xpos[a])


def projection_pair(model, data, first, second, polytopes):
    rotation, translation = relative_pair(model, data, first, second)
    return P.separation(polytopes[first], polytopes[second], rotation, translation)


def schedule_audit():
    candidate, internal = load_frozen_inputs()
    actions = build_schedule(candidate)
    old = W.read(W.RUN/'schedule.json.gz')
    if actions != old:
        raise ValueError('Existing 270 commands differ; no new slowdown is allowed')
    return {'original_270_rows_exact': True, 'summary': schedule_summary(actions),
            'criteria': candidate['proposed_trial_criteria'], 'internal_preflight': internal}


def category(model, gid):
    body = model.body(int(model.geom_bodyid[gid])).name
    name = model.geom(gid).name or ''
    if body.startswith('gripper0_'):
        return 'hand'
    if body == 'white_bowl_1_main':
        return 'white_bowl'
    if 'table' in (body+' '+name).lower():
        return 'table'
    if body.startswith(('robot0_', 'mount0_')):
        return 'other_robot_or_mount'
    return 'other_environment'


def run(output, libero_assets):
    if output.exists():
        raise ValueError('Output must be new; old evidence is immutable')
    if W.sha(W.CANDIDATE) != EXPECTED_MODEL or mj.__version__ != '3.2.3':
        raise ValueError('Frozen model or MuJoCo version differs')
    output.mkdir(parents=True)
    resources = []
    base_scene = W.scene_xml(libero_assets.resolve(), resources)
    candidate_scene = W.replace_scene_wrist(W.copy.deepcopy(base_scene))
    manifest = W.read(W.CANDIDATE.parent/'manifest.json')
    old_names = [r['geom']['name'] for r in manifest['removed']]
    new_names = [r['geom'] for r in manifest['meshes']]
    model = W.mj.MjModel.from_xml_string(W.ET.tostring(candidate_scene, encoding='unicode'))
    baseline = W.mj.MjModel.from_xml_string(W.ET.tostring(base_scene, encoding='unicode'))
    compiled = W.compiled_audit(baseline, model, old_names, new_names)
    audit = {'authorization': AUTHORIZATION, 'candidate_sha256': EXPECTED_MODEL,
             'migration': W.xml_audit(W.BASE, W.CANDIDATE), 'compiled_scene': compiled,
             'resources': W.resource_audit(), 'schedule': schedule_audit()}
    W.write(output/'input-audit.json', audit)
    W.write(output/'scene-resources.json', resources)
    candidate = W.read(P.PLAN)
    samples = candidate['samples']
    if len(samples) != 921 or W.sha(P.PLAN) != W.F.EXPECTED_PLAN:
        raise ValueError('Original 921 poses differ')
    frozen = W.read(W.RUN/'frozen-before-first-step.json')
    with gzip.open(W.RUN/'state.jsonl.gz', 'rt') as stream:
        settled = next(row for line in stream if (row := json.loads(line))['step'] == 250)
    data = mj.MjData(model)
    robot = mj.MjModel.from_xml_path(str(W.CANDIDATE))
    qadr = [int(model.jnt_qposadr[model.joint(robot.joint(j).name).id]) for j in range(robot.njnt)]
    # Existing saved settled scene is a PLANNING reference only, never runtime
    # initialization. Freeze it explicitly; native reset is required if allowed.
    reference = np.array(settled['full_qpos'])
    bowl = model.body('white_bowl_1_main').id
    bowl_adr = int(model.jnt_qposadr[model.body_jntadr[bowl]])
    reference[bowl_adr:bowl_adr+3] = candidate['planning_reference_only_settled_bowl_position_m']
    W.write(output/'planning-reference.json', {
        'full_qpos_for_static_reference_only': reference.tolist(),
        'reference_source': 'Saved right step250 scene; bowl XYZ is the original candidate planning-only settled position.',
        'native_reset_full_qpos': frozen['qpos'],
        'bowl_orientation_source': 'Saved right step250; not predicted held-object orientation.',
        'limitations': 'Finite target-space samples with a stationary reference bowl; does not predict object motion or certify grasp. Any later runtime wrist-external contact must stop.',
        'no_runtime_injection': True})
    by_name = W.geom_names(model)
    wrists = [by_name[name] for name in new_names]
    other = [g for g in range(model.ngeom) if g not in set(wrists)]
    pairs, excluded = [], Counter()
    for g in wrists:
        for h in other:
            # Full-body geometric regression already done; do not repeat it.
            if model.body(int(model.geom_bodyid[h])).name in W.BODIES:
                excluded['already_frozen_wrist_body_scope'] += 1
            elif not P.saved.eligible(model, g, h):
                excluded['native_masks_weld_parent_or_explicit_exclude'] += 1
            else:
                pairs.append((g, h))
    pairs = np.array(pairs, dtype=int)
    first, second = pairs.T
    polytopes = load_polytopes(model, candidate_scene, pairs.ravel())
    cats = np.array([category(model, h) for h in second])
    risk_ids = {label: by_name[name] for label, name in RISK_GEOMS.items()}
    W.write(output/'pair-inventory.json.gz', {
        'pairs': [{'geom_ids': [int(g), int(h)], 'geoms': [model.geom(g).name, model.geom(h).name],
                   'bodies': [model.body(int(model.geom_bodyid[k])).name for k in (g,h)],
                   'category': category(model, h)} for g,h in pairs],
        'excluded_counts': dict(excluded), 'risk_entire_geom_ids': risk_ids,
        'all_306_pieces_checked': True, 'wrist_wrist_pairs_queried': 0})
    summary = {'status': 'RUNNING', 'supplied_poses': 921, 'poses_checked': 0,
               'eligible_pairs_per_pose': len(pairs), 'pair_categories': dict(Counter(cats)),
               'first_block': None, 'minimum_certified_clearance_by_category_m': {},
               'risk_piece_minimum_clearances_m': {}, 'narrowphase_queries': 0,
               'whole_geometry_AABB_certificates': 0, 'wrist_wrist_queries': 0,
               'dynamic_attempts': 0, 'physics_steps': 0, 'numerical_appendix': 'FAIL_UNCHANGED',
               'precision_or_external_fill_exceptions': 'NOT_APPROVED'}
    rows = []
    cache = {}
    model_before = P.saved.fields(model)
    allowed = P.saved.SAFE_APIS
    P.saved.SAFE_APIS = {'mj_kinematics', 'mj_id2name', 'mj_name2id'}
    started = time.monotonic()
    try:
        with P.saved.static_only() as api:
            for i, sample in enumerate(samples, 1):
                data.qpos[:] = reference
                data.qpos[qadr] = sample['qpos']
                protected = data.qpos.copy()
                mj.mj_kinematics(model, data)
                assert np.array_equal(protected, data.qpos) and data.time == 0.
                lo, hi = world_aabbs(model, data)
                gap = np.maximum(lo[second]-hi[first], lo[first]-hi[second]).max(axis=1)
                narrow = np.flatnonzero(gap <= P.TOL)
                summary['whole_geometry_AABB_certificates'] += len(pairs)-len(narrow)
                certificates, blocked = [], []
                for index in narrow:
                    g, h = map(int, pairs[index])
                    rotation, translation = relative_pair(model, data, g, h)
                    key = (g,h,rotation.tobytes(),translation.tobytes())
                    if key in cache:
                        query = dict(cache[key], exact_transform_reuse=True)
                    else:
                        query = {'projection': projection_pair(model, data, g, h, polytopes)}
                        if query['projection']['classification'] == 'CERTIFIED_CONVEX_SOLIDS_SEPARATED':
                            query.update(status='SEPARATED', clearance_lower_bound_m=query['projection']['separation_lower_bound_m'])
                        else:
                            # Existing LP uses fixed slot names; bind arbitrary A/B
                            # body transforms to those slots without changing geometry.
                            transforms = {slot: {'rotation': data.xmat[int(model.geom_bodyid[gid])].reshape(3,3).tolist(),
                                               'translation_m': data.xpos[int(model.geom_bodyid[gid])].tolist()}
                                          for slot,gid in zip(P.BODIES,(g,h))}
                            lp = P.lp(polytopes[g],polytopes[h],transforms)
                            lp.update(W.F.dual_bound(lp,polytopes[g],polytopes[h],rotation,translation))
                            label = W.F.status_from_lp(lp)
                            exact = all(polytopes[k]['representation'] != 'conservative_primitive_OBB' for k in (g,h))
                            status = 'SEPARATED' if label == 'SEPARATED' else ('INTERFERENCE' if label == 'STRICT_CONVEX_OVERLAP' and exact else 'UNKNOWN_CLEARANCE')
                            query.update(status=status, lp=lp,
                                         clearance_lower_bound_m=max(0.,-2*lp['independent_nonnegative_dual_upper_bound_mm']/1000) if status=='SEPARATED' else 0.)
                        cache[key] = query
                    status, distance = query['status'], query['clearance_lower_bound_m']
                    record = {'pair_index': int(index), 'geom_ids': [g,h],
                              'geoms': [model.geom(g).name, model.geom(h).name],
                              'bodies': [model.body(int(model.geom_bodyid[k])).name for k in (g,h)],
                              'category': category(model,h), 'distance_m': distance,
                              'status': status, 'query': query,
                              'cached_geom_world_positions_m': data.geom_xpos[[g,h]].tolist(),
                              'cached_geom_world_rotations': data.geom_xmat[[g,h]].tolist()}
                    certificates.append(record)
                    summary['narrowphase_queries'] += 1
                    if status != 'SEPARATED':
                        blocked.append(record)
                    gap[index] = distance
                minimum = {}
                for cat in np.unique(cats):
                    minimum[cat] = float(gap[cats == cat].min())
                    old = summary['minimum_certified_clearance_by_category_m'].get(cat, float('inf'))
                    summary['minimum_certified_clearance_by_category_m'][cat] = min(old, minimum[cat])
                risk = {}
                for label, gid in risk_ids.items():
                    mask = first == gid
                    risk[label] = {cat: float(gap[mask & (cats == cat)].min()) for cat in np.unique(cats[mask])}
                    for cat, value in risk[label].items():
                        key = label+' / '+cat
                        summary['risk_piece_minimum_clearances_m'][key] = min(summary['risk_piece_minimum_clearances_m'].get(key, float('inf')), value)
                rows.append({'pose_index_one_based': i, 'pose_id': sample['sample_id'], 'phase': sample['phase'],
                             'robot_qpos_18': sample['qpos'], 'broadphase_certified_pairs': len(pairs)-len(narrow),
                             'narrowphase_certificates': certificates, 'minimum_clearance_bounds_m': minimum,
                             'entire_risk_piece_clearance_bounds_m': risk})
                summary['poses_checked'] = i
                if blocked:
                    summary.update(status='BLOCKED_EXTERNAL_PREFLIGHT', first_block={
                        'pose_index_one_based': i, 'pose_id': sample['sample_id'], 'phase': sample['phase'],
                        'blocked_pairs': blocked})
                    break
                if i % 100 == 0:
                    print('external preflight', i, '/ 921; narrow queries', summary['narrowphase_queries'], flush=True)
            else:
                summary['status'] = 'PASS_FINITE_SAMPLES_REFERENCE_SCENE_ONLY'
            assert model_before == P.saved.fields(model)
            W.write(output/'api-audit.json', api)
    finally:
        P.saved.SAFE_APIS = allowed
    summary['elapsed_s'] = time.monotonic()-started
    W.write(output/'poses.json.gz', rows)
    W.write(output/'summary.json', summary)
    paths = [Path(__file__), W.CANDIDATE, P.PLAN, W.RUN/'state.jsonl.gz', W.RUN/'model.xml',
             W.RUN/'schedule.json.gz', Path(W.__file__), W.CANDIDATE.parent/'manifest.json']
    W.write(output/'inputs.json', [{'path': str(p.relative_to(W.ROOT)), 'sha256': W.sha(p)} for p in paths])
    print(json.dumps({k:v for k,v in summary.items() if k != 'first_block'}, indent=2), flush=True)
    return 0 if summary['status'].startswith('PASS_') else 2


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--libero-assets', type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(run(args.output, args.libero_assets))
