#!/usr/bin/env python3
"""One saved 6451-post finger13/finger42 pose; raw-STL convex hull LP only."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import scipy
from scipy.optimize import linprog
from scipy.spatial import ConvexHull


ROOT = next(p for p in Path(__file__).resolve().parents
            if (p / 'mjcf/nero_dh116_right.xml').is_file())
LINKS = ('finger13_link', 'finger42_link')
POSE = 'two_exclusion17_step_6451_post'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stl_vertices(path):
    raw = path.read_bytes()
    count, = struct.unpack_from('<I', raw, 80)
    if len(raw) != 84 + 50*count:
        raise ValueError('Expected exact-length binary STL')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)),
                      ('attribute', '<u2')])
    points = np.frombuffer(raw, dtype=dtype, offset=84, count=count)['vertices']
    return np.unique(points.reshape(-1, 3).astype(np.float64), axis=0)


def model_sources(path):
    tree = ET.parse(path).getroot()
    assets = {m.get('name'): m for m in tree.findall('asset/mesh')}
    result = {}
    for link in LINKS:
        body = tree.find('.//body[@name="gripper0_' + link + '"]')
        geoms = [g for g in body.findall('geom') if g.get('group') == '0']
        if len(geoms) != 1:
            raise ValueError('Expected one collision mesh on ' + link)
        geom = geoms[0]
        if (geom.get('type') != 'mesh' or
                list(map(float, geom.get('pos', '0 0 0').split())) != [0., 0., 0.] or
                list(map(float, geom.get('quat', '1 0 0 0').split())) != [1., 0., 0., 0.] or
                any(key in geom.attrib for key in ('euler', 'axisangle', 'xyaxes', 'zaxis'))):
            raise ValueError('Expected source mesh coordinates in body frame')
        asset = assets[geom.get('mesh')]
        if set(asset.attrib) - {'name', 'file', 'scale'}:
            raise ValueError('Unaccounted mesh transform attributes')
        scale = np.array(list(map(float, asset.get('scale', '1 1 1').split())))
        source = (path.parent / asset.get('file')).resolve()
        result[link] = {'geom_name': geom.get('name'), 'source': source,
                        'scale': scale, 'vertices': stl_vertices(source)*scale}
    return result


def check_case(sources, transforms):
    first, second = (transforms[link] for link in LINKS)
    ra, rb = np.array(first['rotation']), np.array(second['rotation'])
    pa, pb = np.array(first['translation_m']), np.array(second['translation_m'])
    rotation, translation = ra.T @ rb, ra.T @ (pb-pa)
    points = [sources[LINKS[0]]['vertices']*1000,
              (sources[LINKS[1]]['vertices'] @ rotation.T + translation)*1000]
    origin = (np.minimum(points[0].min(axis=0), points[1].min(axis=0)) +
              np.maximum(points[0].max(axis=0), points[1].max(axis=0)))/2
    points = [p-origin for p in points]
    hulls = [ConvexHull(p) for p in points]
    normals, offsets, hull_metadata = [], [], []
    for link, hull, vertices in zip(LINKS, hulls, points):
        normal = hull.equations[:, :3]
        length = np.linalg.norm(normal, axis=1)
        normal, offset = normal/length[:, None], hull.equations[:, 3]/length
        normals.append(normal)
        offsets.append(offset)
        # Recheck every raw point in every reconstructed hull halfspace.
        residual_max = -np.inf
        for start in range(0, len(vertices), 2048):
            residual_max = max(residual_max,
                float(np.max(vertices[start:start+2048] @ normal.T + offset)))
        if residual_max > 1e-8:
            raise ValueError('Hull does not contain its source vertices')
        hull_metadata.append({'link': link, 'unique_raw_vertices': len(vertices),
            'hull_vertices': len(hull.vertices), 'hull_halfspaces': len(normal),
            'max_raw_vertex_halfspace_residual_mm': residual_max,
            'raw_stl_sha256': sha(sources[link]['source']),
            'raw_stl_path': str(sources[link]['source'].relative_to(ROOT)),
            'geom_name': sources[link]['geom_name']})
    normal, offset = np.vstack(normals), np.concatenate(offsets)
    # Plane normals are unit length: the common-ball radius r is measured in mm.
    a = np.column_stack([normal, np.ones(len(normal))])
    lp = linprog([0., 0., 0., -1.], A_ub=a, b_ub=-offset,
                 bounds=[(None, None)]*4, method='highs',
                 options={'primal_feasibility_tolerance': 1e-9,
                          'dual_feasibility_tolerance': 1e-9})
    if not lp.success:
        raise ValueError('Common-ball LP failed: ' + lp.message)
    center, radius = lp.x[:3], float(lp.x[3])
    # Independent long-double matrix evaluation; do not trust reported LP slack.
    margins = -(normal.astype(np.longdouble) @ center.astype(np.longdouble) +
                offset.astype(np.longdouble))
    conservative_radius = float(np.min(margins))
    # An interior witness uses half the recomputed minimum margin, leaving a
    # positive clearance to every plane rather than reusing an optimum on faces.
    witness_radius = max(0., conservative_radius/2)
    witness_clearance = float(np.min(margins-np.longdouble(witness_radius)))
    primal_residual = float(np.max(radius-margins))
    # Independent primal/dual objective and stationarity checks certify the LP.
    weights = -lp.ineqlin.marginals
    dual_bound = float((-offset) @ weights)
    stationarity = float(np.max(np.abs(a.T @ weights-np.array([0., 0., 0., 1.]))))
    if primal_residual > 1e-8 or stationarity > 1e-8 or weights.min() < -1e-9:
        raise ValueError('Independent LP primal/dual checks failed')
    result = {'hulls': hull_metadata, 'conditioning_origin_body13_mm': origin.tolist(),
        'common_ball_center_conditioned_mm': center.tolist(),
        'common_ball_center_body13_m': ((center+origin)/1000).tolist(),
        'common_ball_radius_lp_mm': radius,
        'independently_recomputed_min_center_plane_margin_mm': conservative_radius,
        'strict_inner_ball_witness_radius_mm': witness_radius,
        'strict_inner_ball_min_remaining_face_clearance_mm': witness_clearance,
        'primal_max_constraint_violation_mm': primal_residual,
        'dual_upper_bound_mm': dual_bound,
        'primal_dual_gap_mm': dual_bound-radius,
        'dual_stationarity_max_abs': stationarity,
        'dual_min_weight': float(weights.min()),
        'dual_weights_sum': float(weights.sum()),
        'positive_margin_threshold_mm': 1e-8,
        'strict_convex_solid_interior_overlap': conservative_radius > 1e-8,
        'classification': 'STRICT_CONVEX_INTERIOR_OVERLAP' if conservative_radius > 1e-8
                          else 'NO_POSITIVE_INTERIOR_CERTIFICATE',
        'per_hull_min_center_margin_mm': [float(np.min(n @ center * -1-b))
                                         for n, b in zip(normals, offsets)]}
    # Keep the exact numerical halfspaces and primal/dual witnesses reviewable.
    result['certificate'] = {'normals': normal.tolist(), 'offsets_mm': offset.tolist(),
                             'dual_weights': weights.tolist()}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--certificates', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    certs = json.loads(args.certificates.read_text())
    certificate = next(c for c in certs['records']
                       if c['pose_id'] == POSE and c['pair'] == list(LINKS))
    models = {'coarse17': ROOT/'mjcf/nero_dh116_right.xml',
              'proxy76': ROOT/'examples/right_hand/candidates/proxy76/model.xml'}
    inputs = {args.certificates.resolve(), Path(__file__).resolve(), *models.values()}
    sources = {key: model_sources(path) for key, path in models.items()}
    for link in LINKS:
        if sha(sources['coarse17'][link]['source']) != certificate['source_stl_sha256'][link]:
            raise ValueError('Cached source-surface certificate uses different STL: ' + link)
    inputs |= {v['source'] for case in sources.values() for v in case.values()}
    before = {str(path): sha(path) for path in sorted(inputs)}
    result = {'schema': 'd3-6451-finger13-finger42-independent-convex-common-ball-v1',
        'pose_id': POSE, 'full_qpos': certificate['full_qpos'],
        'body_world_transforms': certificate['body_world_transforms'],
        'source_surface_gap_m': certificate['certificate']['surface_min_distance_m'],
        'method': 'Raw binary STL vertices from each actual model collision asset. Common body13 frame from cached exact-pose body transforms; millimetres with common bounds center subtracted. scipy ConvexHull unit halfspaces; max common ball via HiGHS LP; independent long-double primal margin and dual stationarity/objective rechecks.',
        'numpy_version': np.__version__, 'scipy_version': scipy.__version__,
        'physics_steps': 0, 'mujoco_imports': 0, 'mujoco_api_calls': 0,
        'source_surface_limit': 'This certifies the convex solids of the actual raw proxy meshes. Source triangles have an independently certified positive surface gap; no physical solid penetration depth or dynamic contact force is inferred.',
        'input_sha256_before': before,
        'results': {key: check_case(case, certificate['body_world_transforms'])
                    for key, case in sources.items()}}
    result['input_sha256_after'] = {str(path): sha(path) for path in sorted(inputs)}
    result['inputs_unchanged'] = before == result['input_sha256_after']
    if not result['inputs_unchanged']:
        raise ValueError('Input files changed during pure-math check')
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: {field: value[field] for field in
        ('common_ball_radius_lp_mm', 'independently_recomputed_min_center_plane_margin_mm',
         'dual_upper_bound_mm', 'classification')} for key, value in result['results'].items()}))


if __name__ == '__main__':
    main()
