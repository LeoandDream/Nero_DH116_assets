#!/usr/bin/env python3
"""Offline source-STL comparison for the approved first 1000 thumb steps.

Only mj_kinematics is used. Each recorded post-step qpos is checked against the
full original base_link/finger12_link surfaces; no dynamics or model edits.
"""
import argparse
from collections import Counter
from contextlib import contextmanager
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
GEOMETRY = HERE / 'evidence/d3_geometry/triangle/surface_geometry.py'
MODEL = ROOT / 'mjcf/nero_dh116_right.xml'
LINKS = ('base_link', 'finger12_link')
BODY_NAMES = tuple('gripper0_' + name for name in LINKS)
ABSOLUTE_TOL_M = 1e-12  # Existing surface_query AABB tolerance, unchanged.
STEPS = 1000


def _geometry_module():
    spec = importlib.util.spec_from_file_location('d3_source_surface_geometry', GEOMETRY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _rows(source):
    """Read only steps 1..1000; accept a JSONL path or already loaded rows."""
    if isinstance(source, (str, Path)):
        source = Path(source)
        opener = gzip.open if source.suffix == '.gz' else open
        result = []
        with opener(source, 'rt') as stream:
            for line in stream:
                row = json.loads(line)
                if row['step'] > STEPS:
                    break
                result.append(row)
        provenance = {'path': str(source.resolve()), 'sha256': _sha(source)}
    else:
        values = source.values() if isinstance(source, dict) else source
        result = sorted((r for r in values if 1 <= r['step'] <= STEPS), key=lambda r: r['step'])
        provenance = {'in_memory_rows_sha256': hashlib.sha256(
            json.dumps(result, sort_keys=True, allow_nan=False).encode()).hexdigest()}
    if [r['step'] for r in result] != list(range(1, STEPS + 1)):
        raise ValueError('Exactly the first 1000 consecutive post-step rows are required')
    return result, provenance


@contextmanager
def _no_dynamics():
    names = ('mj_step', 'mj_step1', 'mj_step2', 'mj_forward', 'mj_forwardSkip',
             'mj_fwdConstraint', 'mj_RungeKutta', 'mj_Euler')
    originals = {name: getattr(mujoco, name) for name in names}

    def forbidden(*_args, **_kwargs):
        raise RuntimeError('Dynamics and constraint solves are forbidden in thumb_geometry')

    try:
        for name in names:
            setattr(mujoco, name, forbidden)
        yield
    finally:
        for name, value in originals.items():
            setattr(mujoco, name, value)


class RigidBVH:
    """Transform all original BVH node boxes conservatively, without rebuilding.

    A rigid transform of a node's local AABB is enclosed by center +/- abs(R)*h.
    These possibly loose bounds contain every transformed triangle. Leaf pairs
    are additionally filtered using the exact transformed triangle AABBs.
    """
    def __init__(self, original):
        self.original = original
        lo = np.array([node[0] for node in original.nodes])
        hi = np.array([node[1] for node in original.nodes])
        self.centers = (lo + hi) / 2
        self.half = (hi - lo) / 2

    def transform(self, triangles, rotation, translation):
        centers = self.centers @ rotation.T + translation
        half = self.half @ np.abs(rotation).T
        lows, highs = centers-half, centers+half
        result = object.__new__(type(self.original))
        result.tris = triangles
        result.nodes = [(lo, hi, node[2], node[3], node[4])
                        for lo, hi, node in zip(lows, highs, self.original.nodes)]
        return result


def intersection_query(a, b, geometry, previous_faces=None):
    """Full-surface intersection decision; distance minimization is unnecessary.

    A strict crossing proves intersection and permits early exit. Non-strict
    zero/near-zero results are retained while the remaining candidate pairs are
    searched for a strict crossing. With no such result, all AABB candidates
    must be exhausted before floating-point surface separation is reported.
    """
    stats = {'visited_node_pairs': 0, 'triangle_pairs_evaluated': 0,
             'previous_face_pair_rechecked': previous_faces is not None}

    def strict_result(hit, ids_a, ids_b):
        index = hit.pop('local_pair_index')
        ia, ib = int(ids_a[index]), int(ids_b[index])
        hit.update({'face_ids_zero_based': [ia, ib],
                    'triangle_a': a.tris[ia].tolist(), 'triangle_b': b.tris[ib].tolist()})
        return {**stats, 'classification': 'strict_transverse_crossing',
                'source_surface_intersection': True, 'strict_transverse_witness': hit,
                'boundary_or_numerical_witness': None,
                'all_surface_search_complete': False,
                'global_zero_certified_by_intersection': True}

    if previous_faces is not None:
        aa, bb = (np.array([index], dtype=int) for index in previous_faces)
        _, _, _, hit = geometry.triangle_pairs(a.tris[aa], b.tris[bb])
        stats['triangle_pairs_evaluated'] += 1
        if hit is not None:
            return strict_result(hit, aa, bb)
    triangle_bounds = None
    pending = [(0, 0)]
    boundary = None
    while pending:
        batch_a, batch_b, size = [], [], 0
        while pending and size < 4096:
            ia, ib = pending.pop()
            na, nb = a.nodes[ia], b.nodes[ib]
            if np.any(na[0] > nb[1] + ABSOLUTE_TOL_M) or np.any(nb[0] > na[1] + ABSOLUTE_TOL_M):
                continue
            stats['visited_node_pairs'] += 1
            ida, idb = na[4], nb[4]
            if ida is not None and idb is not None:
                if triangle_bounds is None:
                    # Many separated frames prune before reaching any leaf.
                    # The base BVH is immutable, so its exact bounds are reusable.
                    if not hasattr(a, '_thumb_triangle_bounds'):
                        a._thumb_triangle_bounds = (a.tris.min(axis=1), a.tris.max(axis=1))
                    alo, ahi = a._thumb_triangle_bounds
                    blo, bhi = b.tris.min(axis=1), b.tris.max(axis=1)
                    triangle_bounds = (alo, ahi, blo, bhi)
                aa, bb = np.repeat(ida, len(idb)), np.tile(idb, len(ida))
                keep = np.all(alo[aa] <= bhi[bb] + ABSOLUTE_TOL_M, axis=1)
                keep &= np.all(blo[bb] <= ahi[aa] + ABSOLUTE_TOL_M, axis=1)
                if np.any(keep):
                    batch_a.append(aa[keep]); batch_b.append(bb[keep]); size += int(keep.sum())
            elif idb is not None or (ida is None and np.prod(na[1]-na[0]) >= np.prod(nb[1]-nb[0])):
                pending.extend(((na[2], ib), (na[3], ib)))
            else:
                pending.extend(((ia, nb[2]), (ia, nb[3])))
        if not batch_a:
            continue
        aa, bb = np.concatenate(batch_a), np.concatenate(batch_b)
        ds, ps, qs, hit = geometry.triangle_pairs(a.tris[aa], b.tris[bb])
        stats['triangle_pairs_evaluated'] += len(aa)
        # triangle_pairs returns dummy all-zero distances on a strict early exit;
        # consume its explicit witness before examining non-strict distances.
        if hit is not None:
            return strict_result(hit, aa, bb)
        near = np.flatnonzero(ds <= ABSOLUTE_TOL_M**2)
        if len(near) and boundary is None:
            j = int(near[0])
            boundary = {'face_ids_zero_based': [int(aa[j]), int(bb[j])],
                        'triangle_a': a.tris[aa[j]].tolist(), 'triangle_b': b.tris[bb[j]].tolist(),
                        'closest_points': [ps[j].tolist(), qs[j].tolist()],
                        'distance_m': float(np.sqrt(max(0., ds[j]))),
                        'note': 'Non-strict boundary/coplanar/numerical contact; not a strict crossing certificate.'}
    return {**stats, 'classification': ('boundary_or_numerical_contact' if boundary else 'surface_disjoint_float64'),
            'source_surface_intersection': None if boundary else False,
            'strict_transverse_witness': None, 'boundary_or_numerical_witness': boundary,
            'all_surface_search_complete': True, 'global_zero_certified_by_intersection': False,
            'surface_min_distance_m': None,
            'note': 'Intersection search only; no global positive clearance or solid-containment claim.'}


def focused_checks(geometry=None):
    """Small analytic checks for strict, coplanar, disjoint and BVH traversal."""
    g = _geometry_module() if geometry is None else geometry
    a = np.array([[[0., 0, 0], [1, 0, 0], [0, 1, 0]]])
    crossing = np.array([[[.2, .2, -1], [.2, .2, 1], [.7, .2, 0]]])
    cases = {'strict_crossing': (a, crossing, 'strict_transverse_crossing'),
             'coplanar_overlap': (a, a+[.2, .2, 0], 'boundary_or_numerical_contact'),
             'parallel_separation': (a, a+[0, 0, 2], 'surface_disjoint_float64'),
             'overlapping_boxes_disjoint_triangles':
                 (a, np.array([[[.8, .8, 0], [1, .8, 0], [.8, 1, 0]]]), 'surface_disjoint_float64'),
             'nonstrict_then_strict': (np.concatenate([a, a]), np.concatenate([a+[.2, .2, 0], crossing]),
                                      'strict_transverse_crossing')}
    results = {}
    for name, (aa, bb, expected) in cases.items():
        answer = intersection_query(g.BVH(aa, leaf_size=1), g.BVH(bb, leaf_size=1), g)
        if answer['classification'] != expected:
            raise AssertionError((name, answer, expected))
        results[name] = answer['classification']
    # A conservative rotated BVH must contain the actual transformed leaf faces.
    angle = .371
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    translation = np.array([.1, -.2, .3])
    original = g.BVH(np.concatenate([a, crossing]), leaf_size=1)
    posed = original.tris @ rotation.T + translation
    transformed = RigidBVH(original).transform(posed, rotation, translation)
    for lo, hi, _, _, ids in transformed.nodes:
        if ids is not None:
            if np.any(posed[ids] < lo-ABSOLUTE_TOL_M) or np.any(posed[ids] > hi+ABSOLUTE_TOL_M):
                raise AssertionError('Rigid BVH does not enclose its triangles')
    results['rigid_BVH_leaf_enclosure'] = True
    return results


def _model_inputs():
    xml = ET.parse(MODEL).getroot()
    paths = [MODEL, ROOT / 'urdf/dh116_right.urdf', GEOMETRY, Path(__file__).resolve()]
    paths += [(MODEL.parent / asset.get('file')).resolve()
              for asset in xml.findall('./asset/*') if asset.get('file')]
    source = {}
    for name in LINKS:
        mesh = xml.find(f'./asset/mesh[@name="gripper0_{name}"]')
        if mesh is None or mesh.get('scale', '1 1 1') != '1 1 1':
            raise ValueError('Expected original unscaled source mesh')
        source[name] = (MODEL.parent / mesh.get('file')).resolve()
        geom = xml.find(f'.//body[@name="gripper0_{name}"]/geom[@group="0"]')
        if geom is None or geom.get('pos', '0 0 0') != '0 0 0' or geom.get('quat', '1 0 0 0') != '1 0 0 0':
            raise ValueError('Source-STL body transform requires the existing identity geom origin')
    return paths, source


def _frame(model, data, body_ids, row, triangles, templates):
    qpos = np.asarray(row['full_qpos'], dtype=float)
    if qpos.shape != (model.nq,) or not np.isfinite(qpos).all():
        raise ValueError(f'Invalid recorded full_qpos at step {row["step"]}')
    data.qpos[:] = qpos
    mujoco.mj_kinematics(model, data)
    if not np.array_equal(data.qpos, qpos):
        raise AssertionError('FK changed the recorded qpos')
    ra, rb = (data.xmat[index].reshape(3, 3).copy() for index in body_ids)
    pa, pb = (data.xpos[index].copy() for index in body_ids)
    rotation, translation = ra.T @ rb, ra.T @ (pb-pa)
    posed = triangles[LINKS[1]] @ rotation.T + translation
    bvh = templates[LINKS[1]].transform(posed, rotation, translation)
    transforms = {'body_world_transforms': {
        name: {'rotation': r.tolist(), 'translation_m': p.tolist()}
        for name, r, p in zip(BODY_NAMES, (ra, rb), (pa, pb))},
        'finger12_to_base_rotation': rotation.tolist(), 'finger12_to_base_translation_m': translation.tolist()}
    return bvh, transforms, ra, pa


def compare(output, baseline_records=None, lambda1_records=None):
    """Write output/geometry for two paths or iterables of 1000 state rows.

    Defaults: frozen evidence/hand-motion-state.jsonl.gz and output/lambda1-state.jsonl.
    This function is called only after the separately authorized physical run.
    """
    output = Path(output).resolve()
    if not output.is_relative_to(ROOT) or not output.is_dir():
        raise ValueError('run-output must be an existing directory inside this checkout')
    if mujoco.__version__ != '3.2.3':
        raise RuntimeError('Use the fixed MuJoCo 3.2.3 environment')
    destinations = output / 'geometry'
    sources = {'lambda0': baseline_records if baseline_records is not None else HERE / 'evidence/hand-motion-state.jsonl.gz',
               'lambda1': lambda1_records if lambda1_records is not None else output / 'lambda1-state.jsonl'}
    rows, provenance = {}, {}
    for case, source in sources.items():
        rows[case], provenance[case] = _rows(source)
    paths, mesh_paths = _model_inputs()
    for case, source in sources.items():
        if isinstance(source, (str, Path)):
            paths.append(Path(source).resolve())
    contact_paths = {'lambda0': HERE / 'evidence/hand-motion-contacts.jsonl.gz',
                     'lambda1': output / 'lambda1-contacts.jsonl'}
    contacts = {}
    for case, path in contact_paths.items():
        contact_rows, _ = _rows(path)
        contacts[case] = {r['step']: [c for c in r['contacts'] if set(c['body_names']) == set(BODY_NAMES)]
                          for r in contact_rows}
        paths.append(path)
    before = {str(path.relative_to(ROOT)): _sha(path) for path in sorted(set(paths))}
    destinations.mkdir(exist_ok=False)
    started = time.monotonic()
    g = _geometry_module()
    checks = focused_checks(g)
    triangles = {name: g.read_stl(path) for name, path in mesh_paths.items()}
    bvhs = {name: g.BVH(tris) for name, tris in triangles.items()}
    templates = {name: RigidBVH(bvh) for name, bvh in bvhs.items()}
    metadata = {'schema': 'd3-thumb-source-surface-v1', 'input_sha256': before,
                'record_provenance': provenance, 'mujoco_version': mujoco.__version__,
                'numpy_version': np.__version__, 'source_stl_sha256': {name: _sha(path) for name, path in mesh_paths.items()},
                'source_topology': {name: g.mesh_topology(tris) for name, tris in triangles.items()},
                'coordinate_frame': 'body-local gripper0_base_link; original zero-based binary STL face order',
                'method': 'Every one of 1000 recorded post-step full_qpos per case; mj_kinematics only. '
                          'Recheck prior strict face pair; otherwise exhaust conservative full BVH AABB overlap '
                          'candidates and continuous triangle_pairs. Non-strict zero is retained while searching '
                          'for strict crossing. Full surface_query minimizes continuous surface distance at selected keyframes.',
                'precision': {'arithmetic': 'source float32 coordinates promoted to float64',
                              'AABB_and_near_contact_tolerance_m': ABSOLUTE_TOL_M,
                              'strict_plane_tolerance_m': 1e-12, 'strict_barycentric_tolerance': 1e-10},
                'timing_limit': 'qpos is the final weighted RK4 post-step state. Logged contacts/forces remain '
                                'from the last internal RK4 stage; its qpos was not logged. Same-row FK and '
                                'contact cache therefore are nearby poses, not an exact pose identity.',
                'solid_containment_result': 'UNKNOWN: original source meshes are not watertight 2-manifolds; '
                                            'surface separation does not certify disjoint physical solids.',
                'physics_steps': 0, 'constraint_solves': 0, 'dynamics_and_solve_APIs_blocked': True}
    _write(destinations / 'metadata.json', metadata)
    summaries, keyframes = {}, []
    with _no_dynamics():
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in BODY_NAMES]
        if min(body_ids) < 0:
            raise ValueError('Expected named formal source bodies')
        data = mujoco.MjData(model)
        with (destinations / 'all_steps.jsonl').open('x') as stream:
            for case, case_rows in rows.items():
                answers, previous = [], None
                for row in case_rows:
                    bvh, transforms, ra, pa = _frame(model, data, body_ids, row, triangles, templates)
                    answer = intersection_query(bvhs[LINKS[0]], bvh, g, previous)
                    hit = answer['strict_transverse_witness']
                    previous = hit['face_ids_zero_based'] if hit else None
                    if hit:
                        hit['point_world_m'] = (np.array(hit['point']) @ ra.T + pa).tolist()
                    record = {'case': case, 'step': row['step'], 'time_s': row['time_s'], 'phase': row['phase'],
                              'alignment': 'recorded_post_step_full_qpos', 'full_qpos': row['full_qpos'],
                              'hand_target_rad': row['hand_target_rad'],
                              'active_tracking_error_rad': row['active_tracking_error_rad'],
                              'recorded_RK4_stage_contacts': contacts[case][row['step']], **transforms, **answer}
                    stream.write(json.dumps(record, allow_nan=False) + '\n')
                    answers.append(record)
                    if row['step'] % 100 == 0:
                        stream.flush()
                        print(f'{case} source surfaces: {row["step"]}/1000, elapsed {time.monotonic()-started:.1f}s', flush=True)
                strict = [r['step'] for r in answers if r['classification'] == 'strict_transverse_crossing']
                boundary = [r['step'] for r in answers if r['classification'] == 'boundary_or_numerical_contact']
                transitions = [r['step'] for p, r in zip(answers, answers[1:]) if p['classification'] != r['classification']]
                summaries[case] = {'records_checked': len(answers),
                                   'classification_counts': dict(Counter(r['classification'] for r in answers)),
                                   'first_strict_intersection_step': min(strict) if strict else None,
                                   'last_strict_intersection_step': max(strict) if strict else None,
                                   'strict_intersection_steps': strict, 'boundary_or_numerical_steps': boundary,
                                   'classification_transition_steps': transitions,
                                   'all_recorded_post_source_surfaces_separated': not strict and not boundary}
                # Frozen baseline checkpoints plus first/last crossing and their
                # adjacent samples; cap keeps full minimum queries targeted.
                selected = {1, 229, 451, STEPS}
                if strict:
                    selected.update((min(strict), max(strict), max(1, min(strict)-1), min(STEPS, max(strict)+1)))
                selected.update(boundary[:1])
                for step in sorted(selected):
                    row = case_rows[step-1]
                    bvh, transforms, ra, pa = _frame(model, data, body_ids, row, triangles, templates)
                    result = g.surface_query(triangles[LINKS[0]], bvh.tris, bvhs[LINKS[0]], bvh)
                    coarse = answers[step-1]['classification']
                    if coarse == 'strict_transverse_crossing' and result['strict_transverse_witness'] is None:
                        raise AssertionError('Intersection traversal and full surface_query disagree')
                    if coarse == 'surface_disjoint_float64' and result['surface_min_distance_m'] <= ABSOLUTE_TOL_M:
                        raise AssertionError('Separated-surface result disagrees with full minimum query')
                    result['closest_points_world_m'] = [(np.array(p) @ ra.T + pa).tolist() for p in result['closest_points']]
                    if result['strict_transverse_witness']:
                        hit = result['strict_transverse_witness']
                        hit['point_world_m'] = (np.array(hit['point']) @ ra.T + pa).tolist()
                    keyframes.append({'case': case, 'step': step, 'full_qpos': row['full_qpos'],
                                      'alignment': 'recorded_post_step_full_qpos', **transforms, **result})
                    _write(destinations / 'keyframes.json', keyframes)
    after = {str(path.relative_to(ROOT)): _sha(path) for path in sorted(set(paths))}
    if before != after:
        raise RuntimeError('Offline analysis inputs changed')
    summary = {'cases': summaries, 'physics_steps': 0, 'constraint_solves': 0,
               'inputs_unchanged': True, 'elapsed_s': time.monotonic()-started,
               'focused_checks': checks, 'keyframe_queries': len(keyframes),
               'timing_limit': metadata['timing_limit'], 'solid_containment_result': metadata['solid_containment_result']}
    _write(destinations / 'summary.json', summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-output', help='Existing lambda1 run directory inside this checkout')
    parser.add_argument('--self-check', action='store_true', help='Run only the small analytic geometry checks')
    args = parser.parse_args()
    if args.self_check:
        print(json.dumps(focused_checks(), indent=2))
    elif args.run_output:
        print(json.dumps(compare(ROOT / args.run_output), indent=2))
    else:
        parser.error('--run-output or --self-check is required')


if __name__ == '__main__':
    main()
