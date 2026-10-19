#!/usr/bin/env python3
"""Read-only source coverage/fidelity for the ONE frozen link5/flange candidate.

Reuse the existing continuous polygon coverage, convex containment and BVH.
The optional compilation reads actual mesh_graph support; MjData and every
physics/forward/collision API are blocked. No geometry or tolerance is changed.
"""
import argparse
from contextlib import contextmanager
import gzip
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

for _variable in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
                  'BLIS_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[_variable] = '1'

import numpy as np
from scipy.spatial import cKDTree

import finger13_quality as Q
import link5_flange_poses as P

V, G = Q.V, Q.G
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
OLD = HERE/'candidates/finger13_local/model.xml'
NEW = HERE/'candidates/link5_flange_local/model.xml'
PLAN = HERE/'evidence/d3_link5_flange_local/source-partition-plan.json.gz'
DIAGNOSIS = HERE/'evidence/d3_link5_flange/geometry-results.json.gz'
EXPECTED_OLD = 'b27f4f86fbaeacd2bb925463f39f67181ef7db12963c4d9f7fc01797a7924d1b'
EXPECTED_NEW = '1b2530d923b446605492b43652dd5a3200fd755674ba17a54f0c961cde362b39'
BODIES = ('robot0_link5', 'robot0_gripper_flange')
TARGETS = ('robot0_candidate_nero_link5_4', 'robot0_source_flange_part_2')


def dump(path, value):
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'xt') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


@contextmanager
def compile_only():
    """No data allocation, FK, forward, collision or solver is permitted."""
    old_safe = P.saved.SAFE_APIS
    old_data = P.mujoco.MjData
    P.saved.SAFE_APIS = {'mj_id2name', 'mj_name2id'}
    attempts = []

    def reject_data(*args, **kwargs):
        attempts.append('MjData')
        raise RuntimeError('MjData forbidden in compile-only quality analysis')

    P.mujoco.MjData = reject_data
    try:
        with P.saved.static_only() as audit:
            audit['model_constructor_calls'] = 0
            audit['data_constructor_calls'] = 0
            audit['forbidden_constructor_attempts'] = attempts
            yield audit
    finally:
        P.mujoco.MjData = old_data
        P.saved.SAFE_APIS = old_safe


def bindings(path):
    xml = ET.parse(path).getroot()
    assets = {a.get('name'): a for a in xml.findall('asset/mesh')}
    return {body: [V._binding(path, xml, assets, geom)
                   for geom in xml.find('.//body[@name="'+body+'"]').findall('geom')
                   if geom.get('group', '0') == '0'] for body in BODIES}


def volume(union):
    t = union.triangles
    return float(np.einsum('ij,ij->i', t[:, 0], np.cross(t[:, 1], t[:, 2])).sum()/6)


def coverage_union(parts):
    """Halfspaces of the complete union; no costly hidden-face construction.

    Coverage/membership use each independent convex solid directly. For any
    uncovered point, nearest distance to any part's faces equals distance to
    the exposed union boundary. Interior/bidirectional fidelity instead uses
    the fully exposed boundary of the small replacement union below.
    """
    single = [G.convex_union_surface([part]) for part in parts]
    triangles = np.concatenate([u.triangles for u in single])
    hulls = [u.hulls[0] for u in single]
    face_parts = np.concatenate([np.full(len(u.triangles), i) for i, u in enumerate(single)])
    diagnostics = {'part_count': len(parts), 'coverage_method':
        'Unchanged independent convex halfspaces; full source polygons are subtracted by every solid. Hidden faces need not be constructed for membership or uncovered-point nearest queries.',
        'exposed_boundary_constructed': False,
        'parts': [u.diagnostics['parts'][0] for u in single]}
    return G.ConvexUnion(triangles, face_parts, hulls, diagnostics, G.CLIP_TOL_M)


def deviation(source, union, seed, failed_point):
    """Fixed finite area samples and continuous nearest-face queries."""
    surface = G.TriangleSurface(source)
    sp, _ = G._sample_surface(source, 4096, seed)
    pp, _ = G._sample_surface(union.triangles, 4096, seed+1)
    result = {'samples_per_direction': 4096, 'seed': seed,
              'sampling': 'Existing deterministic stratified source/proxy area sampling; PCG64',
              'limits': 'Finite observed distances; no global Hausdorff or source-solid claim'}
    for label, points, destination, membership in (
            ('source_to_proxy', sp, union.surface, True),
            ('proxy_to_source', pp, surface, False)):
        dist, nearest, faces = destination.nearest(points)
        margins = union.membership_margin(points) if membership else None
        result[label] = G._summarize(points, dist, nearest, faces, margins,
                                     target_triangles=destination.triangles)
        # This reporting window is fixed from the historical failure point,
        # never used to generate or alter candidate geometry.
        mask = np.linalg.norm(points-failed_point, axis=1) <= .01
        result[label+'_within_10mm_of_old_failed_contact'] = G._summarize(
            points[mask], dist[mask], nearest[mask], faces[mask],
            margins[mask] if margins is not None else None,
            target_triangles=destination.triangles)
    return result


def point_checks(source, unions, points):
    surface = G.TriangleSurface(source)
    result = {}
    for name, point in points.items():
        point = np.asarray(point)
        distance, nearest, face = surface.nearest(point[None])
        row = {'point_body_local_m': point.tolist(),
               'distance_to_full_factory_source_surface_m': float(distance[0]),
               'nearest_factory_source_point_m': nearest[0].tolist(),
               'nearest_factory_source_face_id': int(face[0]), 'models': {}}
        for label, union in unions.items():
            distance, nearest, face = union.surface.nearest(point[None])
            margin = float(union.membership_margin(point[None])[0])
            row['models'][label] = {
                'inside_or_boundary_with_numerical_tolerance': margin <= G.CLIP_TOL_M,
                'membership_halfspace_margin_m': margin,
                'distance_to_exposed_proxy_boundary_m': float(distance[0]),
                'nearest_proxy_point_m': nearest[0].tolist()}
        result[name] = row
    return result


def compact_coverage(value):
    excluded = {'uncovered_area_by_source_triangle_m2', 'newly_uncovered_area_by_source_triangle_m2'}
    return {k: v for k, v in value.items() if k not in excluded}


def point_lineage(source_v, source_f, assigned_ids, old_binding, new_bindings, planes):
    """Candidate hull points are unchanged source points or exact CAD cuts."""
    old_points, _ = V.BUILDER.read_mesh(old_binding['file'])
    old_dist, _ = cKDTree(source_v).query(old_points)
    allowed = [source_v]
    if planes:
        tri = source_v[source_f[assigned_ids]]
        edges = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]])
        for plane in planes:
            a, b = edges[:, 0], edges[:, 1]
            da, db = a[:, 1]-plane, b[:, 1]-plane
            mask = da*db < 0
            allowed.append(a[mask]+(da[mask]/(da[mask]-db[mask]))[:, None]*(b[mask]-a[mask]))
    tree = cKDTree(np.concatenate(allowed))
    rows = []
    for binding in new_bindings:
        vertices, _ = V.BUILDER.read_mesh(binding['file'])
        dist, _ = tree.query(vertices)
        rows.append({'geom': binding['name'], 'vertices': len(vertices),
                     'maximum_candidate_vertex_distance_to_source_or_CAD_edge_intersection_m': float(dist.max()),
                     'all_vertices_have_source_lineage_at_unchanged_tolerance': bool(np.all(dist <= G.CLIP_TOL_M)),
                     'scale_pos_quat_equal_old_target': all(binding[k] == old_binding[k] for k in ('scale', 'pos', 'quat'))})
    return {'old_input_points': len(old_points), 'old_input_max_source_coordinate_difference_m': float(old_dist.max()),
            'all_old_input_points_match_factory_coordinates_exactly': bool(np.all(old_dist == 0)),
            'factory_source_bytes_unchanged': True, 'new_parts': rows,
            'new_clip_vertex_definition': 'Only original triangle edges intersected by the two pre-frozen source-native y planes; source coordinates and geom/asset scale and pose are unchanged.'}


def run(output):
    started = time.monotonic()
    output = Path(output).resolve()
    if output.exists():
        raise ValueError('Use a new quality output directory')
    if V._sha(OLD) != EXPECTED_OLD or V._sha(NEW) != EXPECTED_NEW:
        raise ValueError('Frozen candidate fingerprint differs')
    with gzip.open(PLAN, 'rt') as stream:
        plan = json.load(stream)
    with gzip.open(DIAGNOSIS, 'rt') as stream:
        historical = json.load(stream)
    manifest_path = NEW.parent/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    bind = {'old': bindings(OLD), 'new': bindings(NEW)}
    sources = {BODIES[0]: plan['source_regions']['link5'],
               BODIES[1]: plan['source_regions']['flange']}
    historical_row = next(r for r in historical['results'] if r['step'] == 1326)
    checks = historical_row['pose_cases']['cached_RK4_contact_pose']['local_proxy_vs_full_source_point_checks']
    paths = {Path(__file__).resolve(), OLD, NEW, manifest_path, PLAN, DIAGNOSIS,
             Path(Q.__file__), Path(V.__file__), Path(G.__file__), Path(P.__file__),
             Path(P.saved.__file__), Path(V.BUILDER.__file__),
             HERE/'evidence/d3_geometry/triangle/surface_geometry.py'}
    paths.update(ROOT/s['source'] for s in sources.values())
    paths.update(Path(b['file']) for per_model in bind.values()
                 for per_body in per_model.values() for b in per_body)
    before = V._snapshot(paths)
    output.mkdir(parents=True)
    raw = {label: {body: [{'name': b['name'], 'triangles': V.mesh_triangles(b)}
                         for b in values[body]] for body in BODIES}
           for label, values in bind.items()}
    compiled = {}
    with compile_only() as audit:
        for label, path in (('old', OLD), ('new', NEW)):
            model = P.mujoco.MjModel.from_xml_path(str(path))
            audit['model_constructor_calls'] += 1
            compiled[label] = {}
            for body in BODIES:
                compiled[label][body] = [P.compiled_hull(model, b['name'], b)
                                        for b in bind[label][body]]
    result = {'schema': 'one-link5-flange-source-quality-v1',
              'old_model_sha256': EXPECTED_OLD, 'candidate_model_sha256': EXPECTED_NEW,
              'numeric_plane_tolerance_m': G.CLIP_TOL_M, 'tolerance_unchanged': True,
              'physics_steps': 0, 'call_audit': audit, 'input_sha256_before': before,
              'links': {}, 'limits': [
                  'Coverage means supplied source triangle surface inclusion, not solid equivalence.',
                  'Link5 source is not watertight; no source interior, physical penetration or grasp claim.',
                  'Raw float64 geometry and rounded compiled mesh geometry are reported separately.',
                  'No candidate regeneration, threshold increase, decomposition search or repair occurs.']}
    for index, body in enumerate(BODIES):
        source_path = ROOT/sources[body]['source']
        source_v, source_f = V.BUILDER.read_mesh(source_path)
        source = source_v[source_f]
        if body == BODIES[0]:
            assigned_ids = np.asarray(sources[body]['source_triangle_ids_zero_based'], dtype=int)
        else:
            assigned_ids = np.asarray(sources[body]['archive_provenance']['assigned_source_triangle_ids_zero_based'], dtype=int)
        target = TARGETS[index]
        new_names = {r['new_geom'] for r in manifest['new_geoms'] if r['old_geom'] == target}
        failed = np.asarray(checks['saved_contact_position']['source_meshes']['source_urdf_stl'][body]['point_body_local_m'])
        points = {name: value['source_meshes']['source_urdf_stl'][body]['point_body_local_m']
                  for name, value in checks.items()}
        link = {'source_file': str(source_path.relative_to(ROOT)), 'source_sha256': V._sha(source_path),
                'source_triangle_count': len(source), 'original_assigned_source_triangle_count': len(assigned_ids),
                'old_part_count': len(raw['old'][body]), 'new_part_count': len(raw['new'][body]),
                'raw_float64': {}, 'compiled_mesh_graph': {}}
        old_binding = next(b for b in bind['old'][body] if b['name'] == target)
        new_bindings = [b for b in bind['new'][body] if b['name'] in new_names]
        link['source_point_and_scale_lineage'] = point_lineage(
            source_v, source_f, assigned_ids, old_binding, new_bindings,
            plan['source_regions']['link5']['fixed_partition']['planes_m'] if index == 0 else [])
        for representation, per_model in (('raw_float64', raw), ('compiled_mesh_graph', compiled)):
            full, local = {}, {}
            for label in ('old', 'new'):
                all_parts = per_model[label][body]
                local_parts = [part for part in all_parts
                               if part['name'] == target or part['name'] in new_names]
                print(json.dumps({'event': 'union_start', 'body': body,
                                  'representation': representation, 'model': label}), flush=True)
                full[label] = coverage_union(all_parts)
                local[label] = G.convex_union_surface(local_parts)
            old_cover = Q.coverage(source, full['old'])
            new_cover = Q.coverage(source, full['new'], local['old'])
            assigned = Q.coverage(source[assigned_ids], local['new'], local['old'])
            containment = Q.convex_containment(local['new'], local['old'], target)
            orig_volume, new_volume = volume(local['old']), volume(local['new'])
            value = {'old_full_link_union': full['old'].diagnostics,
                     'new_full_link_union': full['new'].diagnostics,
                     'old_full_source_coverage': old_cover, 'new_full_source_coverage': new_cover,
                     'original_assigned_faces_new_local_coverage': assigned,
                     'new_parts_within_old_replaced_hull': containment,
                     'full_link_no_new_geometric_collision_proof': {
                         'certified_with_unchanged_numerical_tolerance': containment['all_parts_contained_with_numerical_tolerance'],
                         'basis': 'Each replacement part lies in its one old hull; other full-link parts are frozen, so the whole new link union is contained in the old link union.',
                         'limit': 'Geometric containment only; no statement about numerical solver contacts, physical behavior or preserved contact multiplicity.'},
                     'local_proxy_volume': {'old_m3': orig_volume, 'new_m3': new_volume,
                         'removed_convex_proxy_volume_m3': orig_volume-new_volume,
                         'reduction_fraction': (orig_volume-new_volume)/orig_volume,
                         'method': 'Signed tetrahedra of exposed oriented convex-union boundary; source solid volume is not inferred.'},
                     'old_historical_failed_point_probes': point_checks(source, local, points)}
            uncovered = new_cover['uncovered_area_m2']
            newly = new_cover['newly_uncovered_previously_covered_source_area_m2']
            value['inherited_vs_new_source_gaps'] = {
                'old_uncovered_area_m2': old_cover['uncovered_area_m2'],
                'new_uncovered_area_m2': uncovered,
                'newly_uncovered_previously_covered_area_m2': newly,
                'new_uncovered_inherited_area_m2': max(0., uncovered-newly),
                'method': 'New full-union uncovered source polygons intersected with the old replaced hull. Since every frozen old part is present in new full union, all remaining old coverage can only come from that hull.'}
            if representation == 'raw_float64':
                value['surface_deviation_original_assigned_source'] = {
                    label: deviation(source[assigned_ids], local[label], 1526+index*10, failed)
                    for label in ('old', 'new')}
                if body == BODIES[0]:
                    plane_points = np.unique(source[assigned_ids].reshape(-1, 3), axis=0)
                    value['native_plane_source_vertex_checks'] = {}
                    for plane in plan['source_regions']['link5']['fixed_partition']['planes_m']:
                        vv = plane_points[plane_points[:, 1] == plane]
                        pv = {'source_vertex_count': len(vv), 'models': {}}
                        for label, union in local.items():
                            distance, nearest, faces = union.surface.nearest(vv)
                            pv['models'][label] = G._summarize(vv, distance, nearest, faces,
                                                               union.membership_margin(vv),
                                                               target_triangles=union.triangles)
                        value['native_plane_source_vertex_checks'][repr(plane)] = pv
            else:
                drift = []
                raw_parts = {p['name']: p for label in ('old', 'new') for p in raw[label][body]}
                for label in ('old', 'new'):
                    for part in per_model[label][body]:
                        rv = np.unique(raw_parts[part['name']]['triangles'].reshape(-1, 3), axis=0)
                        distances, _ = cKDTree(rv).query(part['vertices'])
                        drift.append({'model': label, 'part': part['name'],
                                      'max_compiled_support_to_nearest_raw_vertex_m': float(distances.max()),
                                      'compiled_support_count': len(part['vertices'])})
                value['compiled_support_raw_coordinate_drift'] = drift
            report_name = body+'-'+representation+'.json.gz'
            dump(output/report_name, value)
            link[representation] = {
                'report': report_name,
                'old_full_source_coverage': compact_coverage(old_cover),
                'new_full_source_coverage': compact_coverage(new_cover),
                'original_assigned_faces_new_local_coverage': compact_coverage(assigned),
                'new_parts_within_old_replaced_hull': containment,
                'inherited_vs_new_source_gaps': value['inherited_vs_new_source_gaps'],
                'local_proxy_volume': value['local_proxy_volume']}
            print(json.dumps({'event': 'quality_written', 'body': body,
                              'representation': representation, 'old_uncovered_m2': old_cover['uncovered_area_m2'],
                              'new_uncovered_m2': uncovered, 'newly_uncovered_m2': newly,
                              'assigned_covered': assigned['all_source_triangles_covered_with_numerical_tolerance'],
                              'new_contained_in_old': containment['all_parts_contained_with_numerical_tolerance']}), flush=True)
        result['links'][body] = link
    result['input_sha256_after'] = V._snapshot(paths)
    result['inputs_unchanged'] = before == result['input_sha256_after']
    result['wall_seconds'] = time.monotonic()-started
    raw_good = all(row['raw_float64']['original_assigned_faces_new_local_coverage']['all_source_triangles_covered_with_numerical_tolerance']
                   and row['raw_float64']['new_full_source_coverage']['newly_uncovered_previously_covered_source_area_m2'] == 0
                   and row['raw_float64']['new_parts_within_old_replaced_hull']['all_parts_contained_with_numerical_tolerance']
                   for row in result['links'].values())
    result['raw_source_preservation_status'] = 'PASS' if raw_good else 'FAIL'
    result['compiled_source_preservation_status'] = 'PASS' if all(
        row['compiled_mesh_graph']['original_assigned_faces_new_local_coverage']['all_source_triangles_covered_with_numerical_tolerance']
        and row['compiled_mesh_graph']['new_full_source_coverage']['newly_uncovered_previously_covered_source_area_m2'] == 0
        and row['compiled_mesh_graph']['new_parts_within_old_replaced_hull']['all_parts_contained_with_numerical_tolerance']
        for row in result['links'].values()) else 'FAIL_AT_UNCHANGED_NUMERICAL_TOLERANCE'
    dump(output/'summary.json', result)
    if not result['inputs_unchanged']:
        raise RuntimeError('An input changed during read-only assessment')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    run(args.output)
