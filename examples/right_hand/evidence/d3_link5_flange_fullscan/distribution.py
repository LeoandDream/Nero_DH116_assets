#!/usr/bin/env python3
"""Summarize saved fullscan rows and supplement every interval's graph witnesses.

No model compilation, FK, new pose, optimizer or simulator API is called.
Actual graph triangles are rebuilt from frozen support IDs and coordinates.
"""
from collections import Counter
from contextlib import contextmanager
import gzip
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'examples/right_hand'))
import link5_flange_poses as P


@contextmanager
def no_mujoco():
    """Block every public callable on the imported MuJoCo module, constructors too."""
    originals = {n: getattr(P.mujoco, n) for n in dir(P.mujoco)
                 if not n.startswith('_') and callable(getattr(P.mujoco, n))}
    audit = {'blocked_public_callable_count': len(originals),
             'blocked_public_callables': sorted(originals), 'forbidden_attempts': [],
             'mujoco_api_calls': 0, 'model_constructor_calls': 0,
             'data_constructor_calls': 0, 'FK_calls': 0, 'physics_steps': 0,
             'forward_calls': 0, 'collision_calls': 0, 'constraint_solves': 0}

    def reject(name):
        def call(*args, **kwargs):
            audit['forbidden_attempts'].append(name)
            raise RuntimeError('MuJoCo API forbidden in saved-result postprocessing: '+name)
        return call

    try:
        for name in originals:
            setattr(P.mujoco, name, reject(name))
        yield audit
    finally:
        for name, original in originals.items():
            setattr(P.mujoco, name, original)


def dump(path, value):
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'xt') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def rebuild_hull(record):
    ids = np.asarray(record['support_vertex_ids'], dtype=int)
    vertices = np.asarray(record['compiled_body_local_support_vertices_m'], dtype=float)
    faces = np.asarray(record['face_vertex_ids'], dtype=int)
    if (len(set(ids.tolist())) != len(ids) or vertices.shape != (len(ids), 3)
            or faces.ndim != 2 or faces.shape[1] != 3 or not np.all(np.isfinite(vertices))):
        raise ValueError('Invalid frozen graph support: '+record['geom'])
    lookup = {int(value): i for i, value in enumerate(ids)}
    if set(faces.reshape(-1)) != set(lookup):
        raise ValueError('Graph face IDs do not exactly cover support IDs: '+record['geom'])
    local_faces = np.asarray([[lookup[int(v)] for v in face] for face in faces])
    triangles = vertices[local_faces]
    return {'name': record['geom'], 'vertices': vertices, 'triangles': triangles,
            'bvh': P.geometry.BVH(triangles)}


def anomalous(row):
    return any(p['status'] != 'SEPARATED' for p in row['pairs'])


def intervals(rows, predicate, key=None):
    groups = []
    previous_key = None
    for row in rows:
        if not predicate(row):
            previous_key = None
            continue
        current_key = key(row) if key else None
        if (not groups or row['pose_index_zero_based'] != groups[-1][-1]['pose_index_zero_based']+1
                or current_key != previous_key):
            groups.append([])
        groups[-1].append(row)
        previous_key = current_key
    return groups


def homogeneous_key(row):
    return (row['phase'], row['source_classification'],
            tuple((tuple(p['pair']), p['status']) for p in row['pairs'] if p['status'] != 'SEPARATED'))


def row_ref(row):
    return {k: row[k] for k in ('pose_index_zero_based', 'pose_index_one_based',
                               'pose_id', 'phase', 'time_s')}


def group_record(group, label, pair, selected):
    cases = [(r, p) for r in group for p in r['pairs']
             if pair is None or tuple(p['pair']) == pair]
    affected = [(r, p) for r, p in cases if p['status'] != 'SEPARATED']
    with_lp = [(r, p) for r, p in affected if 'lp' in p]
    deepest = max(with_lp, key=lambda rp: rp[1]['lp']['common_ball_radius_lp_mm']) if with_lp else None
    centers = np.asarray([p['lp']['common_ball_center_body13_m'] for _, p in with_lp])
    representatives = [('first', group[0]), ('last', group[-1])]
    if deepest:
        representatives.insert(1, ('maximum_LP', deepest[0]))
    refs = []
    for role, row in representatives:
        ids = []
        for record in row['pairs']:
            if record['status'] == 'SEPARATED' or (pair is not None and tuple(record['pair']) != pair):
                continue
            case_id = row['pose_id']+'::'+'::'.join(record['pair'])
            item = selected.setdefault(case_id, {'row': row, 'pair_record': record, 'roles': []})
            item['roles'].append({'interval_id': label, 'role': role})
            ids.append(case_id)
        refs.append(dict(row_ref(row), role=role, representative_case_ids=ids))
    source_distances = [r['source_STL_query']['surface_min_distance_m']*1000 for r in group
                        if r['source_STL_query'] is not None]
    return {'interval_id': label, 'pair_scope': list(pair) if pair else 'ALL_SIX_PAIRS',
            'first': row_ref(group[0]), 'last': row_ref(group[-1]),
            'pose_indices_zero_based': [r['pose_index_zero_based'] for r in group],
            'pose_indices_one_based': [r['pose_index_one_based'] for r in group],
            'pose_count': len(group), 'scoped_pair_case_count': len(cases),
            'anomalous_pair_case_count': len(affected),
            'status_counts': dict(Counter(p['status'] for _, p in cases)),
            'source_classification_pose_counts': dict(Counter(r['source_classification'] for r in group)),
            'phase_counts': dict(Counter(r['phase'] for r in group)),
            'anomalous_pair_counts': dict(Counter(' / '.join(p['pair']) for _, p in affected)),
            'LP_center_body5_count': len(centers),
            'LP_center_body5_bbox_m': [centers.min(axis=0).tolist(), centers.max(axis=0).tolist()] if len(centers) else None,
            'maximum_LP_case': dict(row_ref(deepest[0]), pair=deepest[1]['pair'],
                status=deepest[1]['status'], common_ball_radius_mm=deepest[1]['lp']['common_ball_radius_lp_mm']) if deepest else None,
            'source_surface_gap_mm_bounds': [min(source_distances), max(source_distances)] if source_distances else None,
            'representatives': refs}


def plot(rows, pairs, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch

    x = np.asarray([r['pose_index_one_based'] for r in rows])
    colors = plt.get_cmap('tab10').colors
    status_grid = np.asarray([[0 if p['status'] == 'SEPARATED' else
                               1 if p['status'] == 'STRICT_CONVEX_OVERLAP' else 2
                               for p in r['pairs']] for r in rows]).T
    fig, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True,
                             gridspec_kw={'height_ratios': [1.15, 1.2, 1.]})
    short = lambda pair: ' / '.join(n.replace('robot0_link5_local_', 'L').replace('robot0_flange_local_', 'F') for n in pair)
    axes[0].imshow(status_grid, aspect='auto', interpolation='nearest', origin='lower',
                   extent=(.5, len(rows)+.5, -.5, 5.5), vmin=0, vmax=2,
                   cmap=ListedColormap(['#b8d6c4', '#c84038', '#edae39']))
    axes[0].set_yticks(range(6), [short(p) for p in pairs])
    axes[0].set_title('One frozen candidate: all 921 original planning poses and six authorized pairs', pad=30)
    axes[0].legend(handles=[Patch(color=c, label=l) for c, l in
        [('#b8d6c4', 'Certified separated'), ('#c84038', 'Strict convex overlap'),
         ('#edae39', 'UNKNOWN')]], loc='lower right', ncols=3, fontsize=8)
    phase_groups = intervals(rows, lambda _: True, key=lambda r: r['phase'])
    for group in phase_groups:
        first, last = group[0]['pose_index_one_based'], group[-1]['pose_index_one_based']
        axes[0].text((first+last)/2, 1.035, group[0]['phase'],
                     transform=axes[0].get_xaxis_transform(), ha='center', va='bottom', fontsize=8)
        if first > 1:
            for ax in axes:
                ax.axvline(first-.5, color='#808080', lw=.65, alpha=.65)
    for j, pair in enumerate(pairs):
        values = np.asarray([r['pairs'][j].get('lp', {}).get('common_ball_radius_lp_mm', np.nan) for r in rows])
        if np.any(np.isfinite(values)):
            axes[1].plot(x, values, color=colors[j], lw=1.1, label=short(pair))
    axes[1].axhline(0, color='#777777', lw=.6)
    axes[1].set_ylabel('LP common-ball radius (mm)')
    axes[1].set_title('Convex representation overlap metric; this radius is NOT physical penetration depth', fontsize=10)
    if axes[1].get_legend_handles_labels()[0]:
        axes[1].legend(loc='upper right', fontsize=8, ncols=2)
    gaps = np.asarray([r['source_STL_query']['surface_min_distance_m']*1000
                       if r['source_STL_query'] is not None else np.nan for r in rows])
    axes[2].plot(x, gaps, '.', ms=2.5, color='#236a9b')
    axes[2].set_ylabel('Full factory-STL surface gap (mm)')
    axes[2].set_xlabel('Original planning sample index (one-based)')
    axes[2].set_title('Queried anomaly poses only; non-watertight surfaces do not certify source-solid separation', fontsize=10)
    for ax in axes:
        ax.set_xlim(.5, len(rows)+.5)
        ax.grid(axis='x', alpha=.15)
    fig.text(.5, .012, 'Discrete sample adjacency only; no swept-path claim, new physics or change to the existing FAIL.',
             ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .03, 1, 1))
    with path.open('xb') as stream:
        fig.savefig(stream, format='png', dpi=180)
    plt.close(fig)


def main():
    outputs = [HERE/'distribution.json', HERE/'representatives.json.gz', HERE/'overview.png']
    if any(p.exists() for p in outputs):
        raise ValueError('Postprocessing outputs must be new')
    scan = HERE/'scan'
    frozen = json.loads((scan/'frozen.json').read_text())
    summary = json.loads((scan/'summary.json').read_text())
    with gzip.open(scan/'poses.json.gz', 'rt') as stream:
        rows = json.load(stream)['poses']
    if (summary['pose_count'] != 921 or summary['pair_case_count'] != 5526
            or not summary['every_pose_has_six_pairs'] or not summary['inputs_unchanged']
            or summary['input_sha256_before'] != summary['input_sha256_after']
            or summary['input_sha256_before'] != frozen['input_sha256_before']):
        raise ValueError('Require a complete, immutable 921-by-six scan')
    paths = {ROOT/p for p in summary['input_sha256_before']}
    paths.update([Path(__file__).resolve(), scan/'frozen.json', scan/'summary.json', scan/'poses.json.gz',
                  Path(P.reader.GEOMETRY.__file__)])
    before = P.legacy.snapshot(paths)
    if any(before[p] != h for p, h in summary['input_sha256_before'].items()):
        raise ValueError('Frozen scan input hashes no longer match')
    if any(P.legacy.sha(ROOT/s['file']) != s['sha256'] for s in frozen['source']):
        raise ValueError('Factory source hashes differ')
    plan = json.loads(P.PLAN.read_text())['samples']
    if len(rows) != 921 or len({r['pose_id'] for r in rows}) != 921:
        raise ValueError('Saved scan does not contain 921 unique poses')
    pairs = [tuple(p['pair']) for p in rows[0]['pairs']]
    if len(pairs) != 6 or len(set(pairs)) != 6:
        raise ValueError('Saved scan pair inventory is not six unique pairs')
    for i, (row, sample) in enumerate(zip(rows, plan)):
        if (row['pose_index_zero_based'] != i or row['pose_index_one_based'] != i+1
                or row['pose_id'] != sample['sample_id'] or row['original_qpos'] != sample['qpos']
                or row['phase'] != sample['phase'] or row['time_s'] != sample['time_s']
                or [tuple(p['pair']) for p in row['pairs']] != pairs):
            raise ValueError('Saved original pose/pair mapping differs at index '+str(i))
    status_counts = dict(Counter(p['status'] for r in rows for p in r['pairs']))
    if status_counts != summary['pair_status_counts']:
        raise ValueError('Saved pair status counts differ from scan summary')
    selected = {}
    homogeneous = intervals(rows, anomalous, homogeneous_key)
    clusters = [group_record(g, 'homogeneous_'+str(i), None, selected) for i, g in enumerate(homogeneous)]
    pair_intervals = {}
    for j, pair in enumerate(pairs):
        groups = intervals(rows, lambda r: r['pairs'][j]['status'] != 'SEPARATED')
        pair_intervals[' / '.join(pair)] = [group_record(g, 'pair_'+str(j)+'_interval_'+str(i), pair, selected)
                                              for i, g in enumerate(groups)]
    if sum(c['pose_count'] for c in clusters) != summary['anomalous_pose_count']:
        raise ValueError('Homogeneous cluster pose count is not conserved')
    if sum(g['anomalous_pair_case_count'] for groups in pair_intervals.values() for g in groups) != sum(
            n for status, n in status_counts.items() if status != 'SEPARATED'):
        raise ValueError('Pair interval anomaly count is not conserved')
    source_rows = [r for r in rows if r['source_STL_query'] is not None]
    source_extrema = {}
    if source_rows:
        for label, choose in [('minimum', min), ('maximum', max)]:
            row = choose(source_rows, key=lambda r: r['source_STL_query']['surface_min_distance_m'])
            source_extrema[label] = dict(row_ref(row),
                full_source_surface_gap_mm=row['source_STL_query']['surface_min_distance_m']*1000)
    representatives = []
    with no_mujoco() as audit:
        hulls = {record['geom']: rebuild_hull(record) for record in frozen['compiled_hulls']}
        for case_id, item in selected.items():
            row, record = item['row'], item['pair_record']
            rotation = np.asarray(row['relative_rotation_flange_to_link5'])
            translation = np.asarray(row['relative_translation_flange_to_link5_m'])
            actual_r, actual_t = P.relative(row['body_world_transforms'])
            if not np.array_equal(rotation, actual_r) or not np.array_equal(translation, actual_t):
                raise ValueError('Stored relative transform differs at '+row['pose_id'])
            query = P.pair_query(*(hulls[name] for name in record['pair']), rotation, translation)
            representatives.append(dict(row_ref(row), representative_case_id=case_id,
                interval_roles=item['roles'], pair=record['pair'], status=record['status'],
                LP_summary=record.get('lp'), common_ball_certificate_index_zero_based=record.get('certificate_index_zero_based'),
                body_world_transforms=row['body_world_transforms'],
                relative_rotation_flange_to_link5=rotation.tolist(),
                relative_translation_flange_to_link5_m=translation.tolist(),
                source_classification=row['source_classification'], source_STL_query=row['source_STL_query'],
                actual_compiled_triangle_query=query))
            print(json.dumps({'completed_graph_representatives': len(representatives),
                              'total_graph_representatives': len(selected), 'pose_id': row['pose_id']}), flush=True)
        plot(rows, pairs, outputs[2])
    after = P.legacy.snapshot(paths)
    if before != after or audit['forbidden_attempts']:
        raise ValueError('Postprocessing altered inputs or attempted a MuJoCo API')
    common = {'schema': 'd3-link5-flange-fullscan-distribution-v1',
        'candidate_sha256': frozen['candidate_sha256'], 'source': frozen['source'],
        'call_audit': audit, 'input_sha256_before': before, 'input_sha256_after': after,
        'inputs_unchanged': True, 'coordinate_frame': P.BODIES[0],
        'graph_method': 'Frozen actual mesh_graph support IDs map exactly to frozen body-local support coordinates; graph triangles reconstructed without compile/FK.',
        'limits': ['Intervals denote adjacent original discrete samples, not a continuously swept motion.',
            'LP centers use link5 body coordinates in metres; LP radii and source gaps are separate millimetre metrics.',
            'The LP common-ball radius is not physical penetration depth.',
            'Full source-surface gap does not establish source-solid separation for non-watertight meshes.',
            'Graph surface crossing is supplementary: a positive LP interior ball already certifies convex-solid overlap.',
            'Existing acceptance FAILs and separately documented nanometre compilation failures remain unchanged.']}
    dump(outputs[1], dict(common, representative_case_count=len(representatives), representatives=representatives))
    dump(outputs[0], dict(common, pose_count=len(rows), pair_case_count=5526,
        pair_status_counts=status_counts, homogeneous_cluster_count=len(clusters),
        homogeneous_clusters=clusters, continuous_intervals_by_pair=pair_intervals,
        source_query_pose_count=len(source_rows), source_surface_gap_extrema=source_extrema,
        representative_case_count=len(representatives),
        representative_pose_count=len({r['pose_id'] for r in representatives}),
        strict_graph_crossing_representative_count=sum(r['actual_compiled_triangle_query']['strict_transverse_witness'] is not None for r in representatives),
        representatives_file=outputs[1].name, overview_file=outputs[2].name))
    print(json.dumps({'pose_count': len(rows), 'pair_case_count': 5526,
                      'homogeneous_cluster_count': len(clusters),
                      'representative_case_count': len(representatives), 'inputs_unchanged': True}), flush=True)


if __name__ == '__main__':
    main()
