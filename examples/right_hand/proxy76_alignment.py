#!/usr/bin/env python3
"""Reproduce frozen left/right source alignment with static triangle queries.

Example: python examples/right_hand/proxy76_alignment.py --output
runs/d3_proxy76/alignment_reproduced.json

Only verifies the delivered R/t; does not optimize, decompose, load a simulator,
or modify source/candidate inputs. Maxima describe finite area samples, not a
full-surface Hausdorff bound.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

from proxy76_geometry import TriangleSurface

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ALIGNMENT = Path(__file__).resolve().parent/'candidates/proxy76/alignment.json'
_spec = importlib.util.spec_from_file_location('alignment_mesh_reader', ROOT/'tools/build_right_proxy76.py')
BUILDER = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(BUILDER)


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _samples(triangles, count, seed):
    area = np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0],
                                   triangles[:, 2]-triangles[:, 0]), axis=1)/2
    rng = np.random.default_rng(seed)
    faces = rng.choice(len(triangles), count, p=area/area.sum())
    values = rng.random((count, 2))
    root = np.sqrt(values[:, 0])
    bary = np.column_stack((1-root, root*(1-values[:, 1]), root*values[:, 1]))
    return np.einsum('ni,nij->nj', bary, triangles[faces])


def _stats(distances):
    return {'n': len(distances), 'p50_m': float(np.quantile(distances, .5)),
            'p95_m': float(np.quantile(distances, .95)),
            'p99_m': float(np.quantile(distances, .99)),
            'max_m': float(distances.max()), 'mean_m': float(distances.mean())}


def _measure(left, right, lp, rp, rotation, translation):
    forward = right.nearest(lp@rotation.T+translation)[0]
    reverse = left.nearest((rp-translation)@rotation)[0]
    return {'left_to_right': _stats(forward), 'right_to_left': _stats(reverse),
            'pooled': _stats(np.r_[forward, reverse])}


def reproduce(alignment_path):
    alignment_path = Path(alignment_path).resolve()
    frozen_hash = _sha(alignment_path)
    alignment = json.loads(alignment_path.read_text())
    method = alignment['sample_method']
    count = method['per_direction_per_link']
    if count != 8192 or len(alignment['links']) != 17:
        raise ValueError('Expected delivered 17-link, 8192-sample alignment')
    result = {'schema': 'right-proxy76-source-alignment-reproduction-v1',
              'alignment_path': str(alignment_path.relative_to(ROOT)),
              'alignment_sha256': frozen_hash, 'samples_per_direction_per_link': count,
              'query': 'proxy76_geometry.TriangleSurface; continuous target triangles',
              'frozen_transforms_only': True, 'not_full_hausdorff': True, 'links': {}}
    for index, (name, item) in enumerate(alignment['links'].items()):
        meshes, hashes = {}, {}
        for side in ('left', 'right'):
            path = ROOT/item[side+'_path']
            hashes[side] = _sha(path)
            if hashes[side] != item[side+'_sha256']:
                raise ValueError('Frozen source SHA256 changed: '+str(path))
            vertices, faces = BUILDER.read_mesh(path)
            meshes[side] = vertices[faces]
        rotation, translation = np.asarray(item['R']), np.asarray(item['t'])
        determinant = float(np.linalg.det(rotation))
        if (not np.allclose(rotation.T@rotation, np.eye(3), rtol=0, atol=1e-12)
                or bool(determinant < 0) != item['flip_winding']):
            raise ValueError('Frozen transform is not orthogonal or winding flag differs: '+name)
        left, right = TriangleSurface(meshes['left']), TriangleSurface(meshes['right'])
        lp = _samples(meshes['left'], count, method['left_seed_base']+index)
        rp = _samples(meshes['right'], count, method['right_seed_base']+index)
        identity = _measure(left, right, lp, rp, np.eye(3), np.zeros(3))
        chosen = identity if (np.array_equal(rotation, np.eye(3)) and
                              np.array_equal(translation, np.zeros(3))) else _measure(left, right, lp, rp, rotation, translation)
        deltas = {label: {direction: {key: actual[direction][key]-item[label][direction][key]
                                      for key in actual[direction] if key != 'n'}
                           for direction in actual}
                  for label, actual in (('identity', identity), ('chosen', chosen))}
        max_delta = max(abs(value) for label in deltas.values()
                        for direction in label.values() for value in direction.values())
        unchanged = all(_sha(ROOT/item[side+'_path']) == hashes[side] for side in hashes)
        result['links'][name] = {'left_sha256': hashes['left'], 'right_sha256': hashes['right'],
                                 'source_sha256_different': hashes['left'] != hashes['right'],
                                 'R': item['R'], 't': item['t'], 'det': determinant,
                                 'flip_winding': item['flip_winding'], 'inputs_unchanged': unchanged,
                                 'identity': identity, 'chosen': chosen, 'stored_metric_deltas_m': deltas,
                                 'max_abs_stored_metric_delta_m': max_delta}
        print(json.dumps({'link': name, 'chosen_p95_mm': chosen['pooled']['p95_m']*1000,
                          'max_abs_stored_metric_delta_m': max_delta}), flush=True)
    result['inputs_unchanged'] = (_sha(alignment_path) == frozen_hash and
                                  all(x['inputs_unchanged'] for x in result['links'].values()))
    result['max_abs_stored_metric_delta_m'] = max(x['max_abs_stored_metric_delta_m']
                                                for x in result['links'].values())
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--alignment', type=Path, default=DEFAULT_ALIGNMENT)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit('Use a new output file: '+str(args.output))
    report = reproduce(args.alignment)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
