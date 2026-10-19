#!/usr/bin/env python3
"""Check one supplied finite path against original right-hand STL surfaces.

Input: {"schema": "white-bowl-offline-candidate-v1", "samples": [
    {"sample_id": "...", "phase": "...", "time_s": 0, "qpos": [18 numbers]}]}
Samples are consumed in their supplied order; this checker creates no poses,
integrates no dynamics, and stops at the first strict source-surface crossing.
"""
import argparse
import ast
from collections import Counter
from contextlib import contextmanager
from decimal import Decimal, localcontext
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / 'mjcf/nero_dh116_right.xml'
HERE = Path(__file__).resolve().parent
SURFACE = HERE / 'evidence/d3_geometry/triangle/surface_geometry.py'
THUMB = HERE / 'thumb_geometry.py'
DECIMAL = SURFACE.parent / 'enrich_results.py'
BASELINE_HEAD = '8ffe0333c344669e3e3ad66125ba18f0cf48c0f6'
FORMAL_MODEL_SHA256 = '739d3d29bbfd4a6468fb8b75f4de09b719c0f96abb39508959c1dfc33a84b9cc'
TOL_M = 1e-12


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def decimal_checker():
    # Reuse only the reviewed witness functions, without enrich's top-level IO.
    tree = ast.parse(DECIMAL.read_text())
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                 and node.name in ('dot', 'sub', 'cross', 'decimal_witness')]
    if len(functions) != 4:
        raise ValueError('Expected the four existing Decimal witness functions')
    scope = {'Decimal': Decimal, 'localcontext': localcontext}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(DECIMAL), 'exec'), scope)
    return scope['decimal_witness']


@contextmanager
def geometry_only():
    """Allow FK and direct convex geometry queries; block every other mj_* API."""
    allowed = {'mj_kinematics', 'mj_geomDistance', 'mj_name2id'}
    originals = {name: getattr(mujoco, name) for name in dir(mujoco)
                 if name.startswith('mj_') and callable(getattr(mujoco, name))}
    counts = {name: 0 for name in sorted(allowed)}
    counts.update(physics_steps=0, forward_calls=0, constraint_solves=0,
                  integration_calls=0, collision_calls=0, forbidden_attempts=0)

    def guard(name, function):
        def call(*args, **kwargs):
            if name not in allowed:
                counts['forbidden_attempts'] += 1
                raise RuntimeError('Forbidden MuJoCo API: ' + name)
            counts[name] += 1
            return function(*args, **kwargs)
        return call

    try:
        for name, function in originals.items():
            setattr(mujoco, name, guard(name, function))
        yield counts, sorted(set(originals) - allowed)
    finally:
        for name, function in originals.items():
            setattr(mujoco, name, function)


def filter_reasons(model, first, second):
    """Match internal_contact.eligible, including actual-body-only exclusions."""
    b1, b2 = (int(model.geom_bodyid[g]) for g in (first, second))
    w1, w2 = (int(model.body_weldid[b]) for b in (b1, b2))
    reasons = []
    if w1 == w2:
        reasons.append('SAME_WELDED_BODY')
    disabled = int(model.opt.disableflags) & int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
    if not disabled and w1 and w2 and (
            int(model.body_weldid[model.body_parentid[w1]]) == w2 or
            int(model.body_weldid[model.body_parentid[w2]]) == w1):
        reasons.append('PARENT_CHILD_WELDED_BODIES')
    signature = (min(b1, b2) << 16) + max(b1, b2)
    if signature in model.exclude_signature:
        reasons.append('EXPLICIT_ACTUAL_BODY_EXCLUDE')
    if not ((int(model.geom_contype[first]) & int(model.geom_conaffinity[second])) or
            (int(model.geom_contype[second]) & int(model.geom_conaffinity[first]))):
        reasons.append('COLLISION_MASKS')
    return reasons


def hand_geometry(model, xml, surface):
    assets = {node.get('name'): node for node in xml.findall('./asset/mesh')}
    xml_geoms = {node.get('name'): node for node in xml.findall('.//geom')}
    result = {}
    for gid in range(model.ngeom):
        bid = int(model.geom_bodyid[gid])
        name = model.body(bid).name
        if not name.startswith('gripper0_') or int(model.geom_group[gid]) != 0:
            continue
        if int(model.geom_type[gid]) != int(mujoco.mjtGeom.mjGEOM_MESH):
            raise ValueError('Expected only source mesh collision geoms in the hand')
        gname = model.geom(gid).name
        geom = xml_geoms[gname]
        asset = assets[geom.get('mesh')]
        if (geom.get('pos', '0 0 0') != '0 0 0' or
                geom.get('quat', '1 0 0 0') != '1 0 0 0' or
                asset.get('scale', '1 1 1') != '1 1 1'):
            raise ValueError('Original-STL comparison requires identity source geom and scale')
        path = (MODEL.parent / asset.get('file')).resolve()
        triangles = surface.read_stl(path)
        bvh = surface.BVH(triangles)
        result[gid] = {'body_id': bid, 'body_name': name, 'geom_name': gname,
                       'path': path, 'triangles': triangles, 'bvh': bvh}
    if len(result) != 17 or len({g['body_name'] for g in result.values()}) != 17:
        raise ValueError('Expected the fixed 17 original right-hand collision meshes')
    return result


def pair_priority(pair, geoms):
    names = [geoms[gid]['body_name'] for gid in pair]
    thumb = any(name.startswith('gripper0_finger1') for name in names)
    palm = 'gripper0_base_link' in names
    return (0 if thumb and palm else 1 if thumb else 2, tuple(names))


def write(path, result):
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def check(candidate, output):
    """Consume exactly one frozen candidate; return and write its geometry report."""
    candidate, output = Path(candidate).resolve(), Path(output).resolve()
    if output.exists() or not output.parent.is_dir():
        raise ValueError('Output must be a new file in an existing directory')
    candidate_bytes = candidate.read_bytes()
    payload = json.loads(candidate_bytes)
    if payload.get('schema') != 'white-bowl-offline-candidate-v1':
        raise ValueError('Unexpected candidate schema')
    samples = payload.get('samples')
    if not isinstance(samples, list) or not samples:
        raise ValueError('A nonempty ordered samples list is required')
    for index, row in enumerate(samples):
        qpos = np.asarray(row['qpos'], dtype=float)
        if qpos.shape != (18,) or not np.isfinite(qpos).all():
            raise ValueError(f'Invalid qpos at sample index {index}')
        if not isinstance(row.get('time_s'), (int, float)) or not np.isfinite(row['time_s']):
            raise ValueError(f'Invalid time_s at sample index {index}')
    if mujoco.__version__ != '3.2.3':
        raise RuntimeError('Use the fixed MuJoCo 3.2.3 environment')
    actual_head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    if sha(MODEL) != FORMAL_MODEL_SHA256:
        raise RuntimeError('The formal right-hand MJCF differs from the frozen baseline')
    xml = ET.parse(MODEL).getroot()
    paths = {candidate, MODEL, SURFACE, THUMB, DECIMAL, Path(__file__).resolve()}
    paths.update((MODEL.parent / node.get('file')).resolve()
                 for node in xml.findall('./asset/*') if node.get('file'))
    if output in paths:
        raise ValueError('Output cannot overwrite any input')
    before = {str(path): sha(path) for path in sorted(paths)}
    if before[str(candidate)] != hashlib.sha256(candidate_bytes).hexdigest():
        raise RuntimeError('Candidate changed while its input was being read')
    surface = load_module('white_bowl_source_surface', SURFACE)
    thumb = load_module('white_bowl_rigid_geometry', THUMB)
    decimal_witness = decimal_checker()
    started = time.monotonic()
    with geometry_only() as (counts, blocked):
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        if model.nq != 18 or model.npair:
            raise ValueError('Expected qpos18 with no explicit geom pairs')
        data = mujoco.MjData(model)
        geoms = hand_geometry(model, xml, surface)
        templates = {gid: thumb.RigidBVH(item['bvh']) for gid, item in geoms.items()}
        eligible, excluded = [], []
        for i, first in enumerate(geoms):
            for second in list(geoms)[i + 1:]:
                reasons = filter_reasons(model, first, second)
                if reasons:
                    excluded.append({'geom_ids': [first, second], 'reasons': reasons})
                else:
                    eligible.append((first, second))
        eligible.sort(key=lambda pair: pair_priority(pair, geoms))
        if not eligible:
            raise ValueError('The fixed hand model must have eligible internal pairs')
        result = {
            'schema': 'white-bowl-offline-source-surface-v1',
            'candidate': str(candidate), 'head': actual_head, 'baseline_head': BASELINE_HEAD,
            'formal_model_sha256': FORMAL_MODEL_SHA256, 'mujoco_version': mujoco.__version__,
            'input_sha256_before': before,
            'candidate_metadata': {key: value for key, value in payload.items() if key != 'samples'},
            'sample_count_supplied': len(samples), 'poses_started': 0, 'poses_fully_checked': 0,
            'pair_queries_completed': 0, 'eligible_pairs_per_pose': len(eligible),
            'pair_order': [{'geom_ids': list(pair),
                            'body_names': [geoms[g]['body_name'] for g in pair],
                            'priority': pair_priority(pair, geoms)[0]} for pair in eligible],
            'excluded_pairs': excluded,
            'source_meshes': {str(gid): {
                'geom_name': item['geom_name'], 'body_name': item['body_name'],
                'path': str(item['path']), 'sha256': sha(item['path']),
                'triangle_count': len(item['triangles']), 'source_geom_origin_identity': True,
                'compiled_geom_local_pos_m': model.geom_pos[gid].tolist(),
                'compiled_geom_local_quat_wxyz': model.geom_quat[gid].tolist()}
                for gid, item in geoms.items()},
            'method': 'Supplied sample order; thumb-palm, thumb-other, then other eligible pairs. '
                      'Original binary-STL continuous triangles with conservative reusable BVHs; '
                      'only strict transverse source crossing stops the sweep. Same-pose compiled '
                      'convex mj_geomDistance queried at distmax=0 and 0.01m.',
            'classification_scope': 'Proxy true/false labels refer only to strict original-source '
                                    'surface crossing, never physical-solid penetration depth.',
            'limitations': [
                'Finite supplied poses only; no guarantee between samples and no new interpolation.',
                'A first crossing stops all later pairs/poses; unvisited path is explicitly counted.',
                'Float64 BVH/triangle predicates; strict witnesses rechecked with 60-digit Decimal '
                'on stored transformed float64 triangles, not exact symbolic FK.',
                'Source surface separation does not establish disjoint physical solids for open '
                'meshes or rule out containment. No task-success or grasp claim follows.',
                'Only eligible nonadjacent internal right-hand pairs are checked; excluded pairs, '
                'arm, table, bowl and other external geometry are outside this report.',
                'MuJoCo 3.2.3 convex results depend on distmax; distmax=0 negative is the '
                'overlap proxy, while zero is a capped result rather than positive clearance.',
            ],
            'zero_physics_proof': counts, 'blocked_mujoco_apis': blocked,
            'poses': [], 'first_source_crossing': None,
        }
        labels, pair_counts = Counter(), Counter()
        for sample_index, row in enumerate(samples):
            qpos = np.asarray(row['qpos'], dtype=float)
            data.qpos[:] = qpos
            mujoco.mj_kinematics(model, data)
            transforms = {}
            for gid, item in geoms.items():
                rotation = data.xmat[item['body_id']].reshape(3, 3).copy()
                position = data.xpos[item['body_id']].copy()
                lo, hi = item['bvh'].nodes[0][:2]
                center, half = (lo + hi) / 2, (hi - lo) / 2
                center = rotation @ center + position
                half = np.abs(rotation) @ half
                transforms[gid] = (rotation, position, center - half, center + half)
            pose = {'sample_index_zero_based': sample_index, 'sample': row, 'pairs': []}
            result['poses_started'] += 1
            for pair_index, (first, second) in enumerate(eligible):
                a, b = geoms[first], geoms[second]
                ra, pa, alo, ahi = transforms[first]
                rb, pb, blo, bhi = transforms[second]
                rotation, translation = ra.T @ rb, ra.T @ (pb - pa)
                if np.any(alo > bhi + TOL_M) or np.any(blo > ahi + TOL_M):
                    answer = {'classification': 'surface_disjoint_float64',
                              'source_surface_intersection': False,
                              'strict_transverse_witness': None,
                              'boundary_or_numerical_witness': None,
                              'all_surface_search_complete': True,
                              'conservative_whole_mesh_AABB_separated': True,
                              'triangle_pairs_evaluated': 0, 'visited_node_pairs': 0}
                else:
                    posed = b['triangles'] @ rotation.T + translation
                    bvh = templates[second].transform(posed, rotation, translation)
                    answer = thumb.intersection_query(a['bvh'], bvh, surface)
                coarse = {}
                for cutoff in (0., .01):
                    segment = np.full(6, np.nan)
                    distance = float(mujoco.mj_geomDistance(model, data, first, second, cutoff, segment))
                    if not np.isfinite(distance):
                        raise RuntimeError('Nonfinite compiled-convex distance')
                    coarse[str(cutoff)] = {'distance_m': distance,
                                          'fromto_world_m': segment.tolist() if np.isfinite(segment).all() else None}
                overlap = coarse['0.0']['distance_m'] < 0
                witness = answer['strict_transverse_witness']
                if witness is not None:
                    witness['decimal_recheck'] = decimal_witness(witness)
                    witness['point_world_m'] = (ra @ np.asarray(witness['point']) + pa).tolist()
                    ia, ib = witness['face_ids_zero_based']
                    witness['original_faces_body_local_m'] = [a['triangles'][ia].tolist(),
                                                             b['triangles'][ib].tolist()]
                    label = 'true_positive_source_crossing' if overlap else 'false_negative_source_crossing'
                elif answer['source_surface_intersection'] is False:
                    label = 'false_positive_source_crossing_proxy' if overlap else 'true_negative_source_crossing'
                else:
                    label = 'unknown_boundary_or_numerical_contact'
                record = {'pair_index_zero_based': pair_index, 'geom_ids': [first, second],
                          'body_names': [a['body_name'], b['body_name']],
                          'B_to_A_rotation': rotation.tolist(), 'B_to_A_translation_m': translation.tolist(),
                          'body_world_transforms': {
                              a['body_name']: {'rotation': ra.tolist(), 'translation_m': pa.tolist()},
                              b['body_name']: {'rotation': rb.tolist(), 'translation_m': pb.tolist()}},
                          'same_pose_compiled_convex': coarse, 'convex_overlap_at_distmax_zero': overlap,
                          'proxy_classification': label, **answer}
                pose['pairs'].append(record)
                labels[label] += 1
                pair_counts[str(pair_index)] += 1
                result['pair_queries_completed'] += 1
                if witness is not None:
                    result['first_source_crossing'] = {'sample_index_zero_based': sample_index,
                                                       'sample': row, 'pair_result': record}
                    break
            if not np.array_equal(data.qpos, qpos):
                raise AssertionError('A geometry query modified the supplied qpos')
            pose['all_eligible_pairs_checked'] = len(pose['pairs']) == len(eligible)
            result['poses_fully_checked'] += int(pose['all_eligible_pairs_checked'])
            result['poses'].append(pose)
            if result['first_source_crossing'] is not None:
                break
        after = {str(path): sha(path) for path in sorted(paths)}
        if before != after:
            raise RuntimeError('An input changed during offline checking')
        result.update(input_sha256_after=after, inputs_unchanged=True,
                      elapsed_s=time.monotonic() - started, classification_counts=dict(labels),
                      pair_query_counts_by_pair_index=dict(pair_counts),
                      poses_not_visited=len(samples) - result['poses_started'],
                      unvisited_pair_pose_queries=len(samples) * len(eligible) - result['pair_queries_completed'])
        result['all_supplied_samples_and_eligible_pairs_checked'] = (
            result['pair_queries_completed'] == len(samples) * len(eligible))
        if result['first_source_crossing'] is not None:
            result['A_GATE'] = 'STOP_SOURCE_SURFACE_CROSSING'
        elif labels['false_positive_source_crossing_proxy']:
            result['A_GATE'] = 'UNKNOWN_LOCAL_PROXY_REVIEW_REQUIRED'
        elif labels['unknown_boundary_or_numerical_contact']:
            result['A_GATE'] = 'UNKNOWN_BOUNDARY_OR_NUMERICAL_CONTACT'
        else:
            result['A_GATE'] = 'NO_SOURCE_CROSSING_AT_SUPPLIED_FINITE_SAMPLES'
        write(output, result)
    return result


def self_check():
    """Small pure-geometry/filter checks; no MjModel, path, or dynamics run."""
    fake = SimpleNamespace(
        geom_bodyid=[4, 5], body_weldid=[0, 1, 2, 0, 1, 2],
        body_parentid=[0, 0, 0, 0, 1, 2], opt=SimpleNamespace(disableflags=0),
        geom_contype=[1, 1], geom_conaffinity=[1, 1],
        exclude_signature=[(1 << 16) + 2])
    if filter_reasons(fake, 0, 1):
        raise AssertionError('A fake welded-body exclusion must not exclude actual bodies 4/5')
    fake.exclude_signature = [(4 << 16) + 5]
    if filter_reasons(fake, 0, 1) != ['EXPLICIT_ACTUAL_BODY_EXCLUDE']:
        raise AssertionError('The actual-body exclusion must match bodies 4/5')
    surface = load_module('white_bowl_self_check_surface', SURFACE)
    thumb = load_module('white_bowl_self_check_thumb', THUMB)
    with geometry_only() as (counts, _blocked):
        blocked_calls = ('mj_step', 'mj_step1', 'mj_step2', 'mj_forward',
                         'mj_forwardSkip', 'mj_fwdConstraint', 'mj_RungeKutta', 'mj_Euler')
        for name in blocked_calls:
            try:
                getattr(mujoco, name)(None, None)
            except RuntimeError as error:
                if str(error) != 'Forbidden MuJoCo API: ' + name:
                    raise
            else:
                raise AssertionError(name + ' was not blocked')
        if counts['forbidden_attempts'] != len(blocked_calls):
            raise AssertionError('The forbidden API counter did not record every blocked call')
        geometry = thumb.focused_checks(surface)
    return {'actual_body_exclusion': True, 'fake_weld_exclusion_rejected': True,
            'blocked_dynamics_calls': list(blocked_calls), 'guard_counts': counts,
            'existing_focused_geometry_checks': geometry}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--self-check', action='store_true',
                        help='Run only small filter/guard/pure-geometry checks')
    args = parser.parse_args()
    if args.self_check:
        if args.candidate or args.output:
            parser.error('--self-check does not accept a candidate or output')
        print(json.dumps(self_check(), indent=2))
        return
    if args.candidate is None or args.output is None:
        parser.error('--candidate and --output are required for a path check')
    result = check(args.candidate, args.output)
    print(json.dumps({key: result[key] for key in (
        'A_GATE', 'poses_started', 'poses_fully_checked', 'pair_queries_completed',
        'poses_not_visited', 'all_supplied_samples_and_eligible_pairs_checked')}))


if __name__ == '__main__':
    main()
