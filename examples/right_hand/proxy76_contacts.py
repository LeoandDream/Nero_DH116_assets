#!/usr/bin/env python3
"""Compare collision hulls at saved D3 qpos, without advancing any state.

Only mj_kinematics, mj_geomDistance and object-ID lookups are permitted. Source
surface certificates can be reused only for the exact qpos, original STL hashes
and matching body transforms. Negative hull distances are proxy query results,
not source-triangle penetration depths or dynamic contact measurements.
"""
import argparse
from collections import Counter
from contextlib import contextmanager
import gzip
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / 'examples/right_hand/evidence'
ORIGINAL = ROOT / 'mjcf/nero_dh116_right.xml'
PAIRS = (('base_link', 'finger12_link'),
         ('finger12_link', 'finger42_link'),
         ('finger13_link', 'finger42_link'),
         ('finger12_link', 'finger43_link'))
CUTOFFS = (0., .01, .1, 1.)
SAFE_APIS = {'mj_kinematics', 'mj_geomDistance', 'mj_id2name', 'mj_name2id'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False,
                                    allow_nan=False) + '\n')


def saved_poses():
    """Select existing post states; pre means the preceding saved post state."""
    poses = []
    for case, path, steps in (
        ('original17', EVIDENCE / 'hand-motion-state.jsonl.gz', (229, 451, 6498, 6511)),
        ('two_exclusion17', EVIDENCE / 'd3_internal_contact/filtered-state.jsonl.gz',
         (6242, 6395, 6451)),
    ):
        needed = set(steps) | {step-1 for step in steps}
        rows = {}
        with gzip.open(path, 'rt') as stream:
            for line in stream:
                row = json.loads(line)
                if row['step'] in needed:
                    rows[row['step']] = row
        if set(rows) != needed:
            raise ValueError('Missing saved state: ' + str(path))
        digest = sha(path)
        for step in steps:
            for alignment, source_step in (('pre', step-1), ('post', step)):
                row = rows[source_step]
                poses.append({
                    'id': f'{case}_step_{step}_{alignment}', 'case': case,
                    'checkpoint_step': step, 'alignment': alignment,
                    'source_state_step': source_step, 'time_s': row['time_s'],
                    'phase': row['phase'], 'full_qpos': row['full_qpos'],
                    'source_state_path': str(path.relative_to(ROOT)),
                    'source_state_sha256': digest,
                })
    return poses


@contextmanager
def static_only():
    """Block every exposed mj_* function except the four stated read/FK APIs."""
    originals = {name: getattr(mujoco, name) for name in dir(mujoco)
                 if name.startswith('mj_') and callable(getattr(mujoco, name))}
    calls = Counter()
    forbidden_attempts = []

    def wrapper(name, function):
        def call(*args, **kwargs):
            if name not in SAFE_APIS:
                forbidden_attempts.append(name)
                raise RuntimeError('Forbidden MuJoCo API in static comparison: ' + name)
            calls[name] += 1
            return function(*args, **kwargs)
        return call

    audit = {'allowed_apis': sorted(SAFE_APIS),
             'blocked_apis': sorted(set(originals)-SAFE_APIS),
             'allowed_api_calls': calls, 'forbidden_api_attempts': forbidden_attempts,
             'physics_steps': 0, 'constraint_solves': 0,
             'resets': 0, 'forward_calls': 0, 'collision_calls': 0}
    try:
        for name, function in originals.items():
            setattr(mujoco, name, wrapper(name, function))
        yield audit
    finally:
        for name, function in originals.items():
            setattr(mujoco, name, function)


def fields(value, prefix=''):
    result = {}
    for name in dir(value):
        if name.startswith('_'):
            continue
        item = getattr(value, name)
        key = prefix + name
        if isinstance(item, np.ndarray):
            raw = str((item.dtype.str, item.shape)).encode() + item.tobytes()
        elif isinstance(item, bytes):
            raw = item
        elif isinstance(item, (int, float, str)):
            raw = json.dumps(item, sort_keys=True, allow_nan=False).encode()
        elif not callable(item) and type(item).__module__ == 'mujoco._structs':
            result.update(fields(item, key + '.'))
            continue
        else:
            continue
        result[key] = hashlib.sha256(raw).hexdigest()
    return result


def name(model, kind, index):
    return mujoco.mj_id2name(model, kind, int(index))


def hand_geoms(model):
    result = {}
    for gid in range(model.ngeom):
        body = name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[gid])
        if body.startswith('gripper0_') and model.geom_group[gid] == 0:
            result.setdefault(body.removeprefix('gripper0_'), []).append(gid)
    return result


def eligible(model, first, second):
    """Actual-body excludes and MuJoCo weld/parent/mask eligibility, read only."""
    a, b = (int(model.geom_bodyid[g]) for g in (first, second))
    wa, wb = (int(model.body_weldid[body]) for body in (a, b))
    if wa == wb:
        return False
    parent_filter_off = int(model.opt.disableflags) & int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
    if not parent_filter_off and wa and wb:
        if (int(model.body_weldid[model.body_parentid[wa]]) == wb or
                int(model.body_weldid[model.body_parentid[wb]]) == wa):
            return False
    if (min(a, b) << 16) + max(a, b) in model.exclude_signature:
        return False
    return bool((int(model.geom_contype[first]) & int(model.geom_conaffinity[second])) or
                (int(model.geom_contype[second]) & int(model.geom_conaffinity[first])))


def hull_distances(model, data, by_link, pair):
    rows = []
    deepest = [None] * len(CUTOFFS)
    eligible_deepest = [None] * len(CUTOFFS)
    for first in by_link[pair[0]]:
        for second in by_link[pair[1]]:
            geom_names = [name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
                          for gid in (first, second)]
            enabled = eligible(model, first, second)
            distances = []
            for index, cutoff in enumerate(CUTOFFS):
                segment = np.zeros(6)
                distance = float(mujoco.mj_geomDistance(model, data, first, second,
                                                       cutoff, segment))
                distances.append(distance)
                value = {'distance_m': distance, 'geom_names': geom_names,
                         'closest_segment_world_m': segment.tolist(),
                         'distance_is_capped': distance == cutoff}
                if deepest[index] is None or distance < deepest[index]['distance_m']:
                    deepest[index] = value
                if enabled and (eligible_deepest[index] is None or
                                distance < eligible_deepest[index]['distance_m']):
                    eligible_deepest[index] = value
            rows.append({'geom_names': geom_names, 'eligible': enabled,
                         'distances_m_in_cutoff_order': distances})
    summaries = []
    for index, cutoff in enumerate(CUTOFFS):
        summaries.append({
            'cutoff_m': cutoff,
            'negative_geom_pairs': sum(r['distances_m_in_cutoff_order'][index] < 0 for r in rows),
            'eligible_negative_geom_pairs': sum(r['eligible'] and
                r['distances_m_in_cutoff_order'][index] < 0 for r in rows),
            'deepest': deepest[index], 'eligible_deepest': eligible_deepest[index],
        })
    return {'geom_pair_count': len(rows),
            'eligible_geom_pair_count': sum(r['eligible'] for r in rows),
            'cutoff_order_m': list(CUTOFFS), 'query_summaries': summaries,
            'geom_pair_distances': rows}


def reuse_certificate(certificates, pose, pair, model, data):
    matches = [(i, r) for i, r in enumerate(certificates['records'])
               if r['full_qpos'] == pose['full_qpos'] and r['pair'] == list(pair)]
    if not matches:
        return {'status': 'NOT_QUERIED_AT_THIS_EXACT_SAVED_POSE'}
    if len(matches) != 1:
        raise ValueError('Ambiguous source certificate: ' + pose['id'])
    index, record = matches[0]
    max_transform_error = 0.
    for link in pair:
        source = ROOT / 'meshes/dh116_right/source' / (link + '.STL')
        if sha(source) != record['source_stl_sha256'][link]:
            raise ValueError('Source STL changed: ' + link)
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper0_' + link)
        expected = record['body_world_transforms'][link]
        max_transform_error = max(max_transform_error,
            float(np.max(np.abs(data.xmat[bid].reshape(3, 3)-expected['rotation']))),
            float(np.max(np.abs(data.xpos[bid]-expected['translation_m']))))
    if max_transform_error > 1e-14:
        raise ValueError('Certificate body transform differs: ' + pose['id'])
    cert = record['certificate']
    return {'status': 'REUSED_EXACT_QPOS_AND_SOURCE_SHA256',
            'certificate_index': index, 'max_body_transform_abs_error': max_transform_error,
            'surface_min_distance_m': cert['surface_min_distance_m'],
            'surface_intersection': cert['surface_intersection'],
            'strict_transverse_crossing': cert.get('strict_transverse_witness') is not None,
            'all_surface_search_complete': cert['all_surface_search_complete'],
            'global_zero_certified_by_intersection': cert['global_zero_certified_by_intersection']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source-certificates', type=Path,
                        default=EVIDENCE / 'd3_proxy76/existing-source-certificates.json')
    args = parser.parse_args()
    candidate = args.candidate.resolve()
    certificates_path = args.source_certificates.resolve()
    certificates = json.loads(certificates_path.read_text())
    poses = saved_poses()
    paths = {ORIGINAL, candidate, certificates_path, Path(__file__).resolve(),
             *[ROOT / p['source_state_path'] for p in poses],
             *[ROOT / 'meshes/dh116_right/source' / (n + '.STL') for p in PAIRS for n in p]}
    input_hashes = {str(path): sha(path) for path in sorted(paths)}
    args.output.mkdir(parents=True, exist_ok=True)
    model_audits = {}
    results = {pose['id']: {'pose': pose, 'pairs': {}} for pose in poses}
    with static_only() as api_audit:
        for label, path, expected_geoms in (('coarse17', ORIGINAL, 17),
                                             ('proxy76', candidate, 76)):
            model = mujoco.MjModel.from_xml_path(str(path))
            before = fields(model)
            by_link = hand_geoms(model)
            if sum(map(len, by_link.values())) != expected_geoms:
                raise ValueError('Unexpected hand collision count for ' + label)
            for pose in poses:
                data = mujoco.MjData(model)
                data.qpos[:] = pose['full_qpos']
                state_before = {key: np.array(getattr(data, key), copy=True)
                                for key in ('qpos', 'qvel', 'qacc', 'ctrl', 'act', 'qacc_warmstart')}
                time_before = data.time
                mujoco.mj_kinematics(model, data)
                body_transforms = {}
                for link in by_link:
                    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                           'gripper0_' + link)
                    body_transforms[link] = {
                        'rotation': data.xmat[bid].reshape(3, 3).tolist(),
                        'translation_m': data.xpos[bid].tolist()}
                if label == 'coarse17':
                    results[pose['id']]['body_world_transforms'] = body_transforms
                elif body_transforms != results[pose['id']]['body_world_transforms']:
                    raise ValueError('Body FK changed between models: ' + pose['id'])
                for pair in PAIRS:
                    row = results[pose['id']]['pairs'].setdefault('/'.join(pair), {})
                    row[label] = hull_distances(model, data, by_link, pair)
                    source_surface = reuse_certificate(certificates, pose, pair, model, data)
                    if label == 'coarse17':
                        row['source_surface'] = source_surface
                    elif source_surface != row['source_surface']:
                        raise ValueError('Source certificate differs between models: ' + pose['id'])
                if data.time != time_before or any(not np.array_equal(getattr(data, key), value)
                                                   for key, value in state_before.items()):
                    raise ValueError('Static query changed integration state: ' + pose['id'])
            after = fields(model)
            if before != after:
                raise ValueError('Compiled model changed during static query: ' + label)
            model_audits[label] = {'model_path': str(path), 'model_sha256': sha(path),
                'hand_collision_geoms': expected_geoms,
                'geoms_per_link': {key: len(value) for key, value in by_link.items()},
                'compiled_fields_compared': len(before), 'compiled_fields_unchanged': True,
                'compiled_field_sha256': before,
                'all_saved_qpos_qvel_qacc_ctrl_act_warmstart_time_unchanged': True}
    after_hashes = {str(path): sha(path) for path in sorted(paths)}
    if input_hashes != after_hashes:
        raise ValueError('Input files changed during static comparison')
    report = {'schema': 'd3-proxy76-saved-pose-hull-comparison-v1',
        'mujoco_version': mujoco.__version__, 'poses': len(poses),
        'body_pair_comparisons': len(poses)*len(PAIRS),
        'source_certificates_sha256': sha(certificates_path),
        'model_audits': model_audits, 'zero_dynamics_audit': api_audit,
        'input_sha256_before': input_hashes, 'input_sha256_after': after_hashes,
        'inputs_unchanged': True,
        'method': 'Same saved qpos in each model; fresh MjData; mj_kinematics followed by mj_geomDistance for every geom pair on four selected body pairs at four cutoffs. Counts are negative queried geom pairs, not contact-point counts. No mj_collision or force solve.',
        'timing_limit': 'pre is the preceding saved post-step qpos; post is final weighted RK4 qpos. Historical contacts/forces belong to the final RK4 internal solve with no saved exact internal-stage qpos. Static results are never relabelled as those dynamic forces.',
        'limits': ['MuJoCo 3.2.3 signed convex-hull results depend on distmax; all four cutoffs are retained. A capped zero at cutoff=0 does not establish surface contact.',
                   'Exact-qpos source STL certificates describe original source surfaces; they do not certify proxy fit, control health, grasp success, or dynamics.',
                   'Source meshes can be nonmanifold; positive surface distance alone does not certify disjoint solid interiors.',
                   'Only the selected saved poses and four body pairs were compared. No new dynamic trajectory or all-pose safety claim.'],
        'results': list(results.values())}
    write(args.output / 'source-certificates.json', certificates)
    write(args.output / 'poses.json', {'poses': poses})
    write(args.output / 'contacts.json', report)
    print(json.dumps({'output': str(args.output), 'poses': len(poses),
                      'body_pair_comparisons': len(poses)*len(PAIRS),
                      'physics_steps': 0, 'forbidden_api_attempts': api_audit['forbidden_api_attempts']}))


if __name__ == '__main__':
    main()
