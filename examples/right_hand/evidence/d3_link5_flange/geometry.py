#!/usr/bin/env python3
"""One frozen link5/flange pair: source triangles versus actual convex proxies.

Only static FK is called. The existing OBJ/STL reader, continuous triangle BVH,
and independently checked common-ball LP are reused without source changes.
"""
import argparse
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import scipy
from scipy.spatial import ConvexHull, cKDTree

ROOT = next(p for p in Path(__file__).resolve().parents
            if (p/'mjcf/nero_dh116_right.xml').is_file())
sys.path.insert(0, str(ROOT/'examples/right_hand'))
import proxy76_contacts as saved
import proxy76_validate as reader
import mujoco

STEPS = (1314, 1315, 1321, 1326, 1400, 1401)
BODIES = ('robot0_link5', 'robot0_gripper_flange')
PAIR = ('robot0_candidate_nero_link5_4', 'robot0_source_flange_part_2')
VISUAL = ('robot0_original_urdf_visual_link5',
          'robot0_original_urdf_visual_gripper_flange')
CANDIDATE = ROOT/'examples/right_hand/candidates/finger13_local/model.xml'
EXPECTED_CANDIDATE = 'b27f4f86fbaeacd2bb925463f39f67181ef7db12963c4d9f7fc01797a7924d1b'
FROZEN = ROOT/'examples/right_hand/evidence/d3_bowl78_slow_001'


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


geometry = module('existing_full_triangle_bvh',
                  ROOT/'examples/right_hand/evidence/d3_geometry/triangle/surface_geometry.py')
lpcheck = module('existing_common_ball_lp',
                ROOT/'examples/right_hand/evidence/d3_proxy76/gap_check/check_convex_gap.py')
lpcheck.LINKS = BODIES


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def snapshot(paths):
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(paths)}


def rows(path):
    with gzip.open(path, 'rt') as stream:
        return {r['step']: r for line in stream
                if (r := json.loads(line))['step'] in STEPS}


def transform(triangles, transforms):
    a, b = (transforms[name] for name in BODIES)
    ra, rb = np.asarray(a['rotation']), np.asarray(b['rotation'])
    pa, pb = np.asarray(a['translation_m']), np.asarray(b['translation_m'])
    rotation, translation = ra.T@rb, ra.T@(pb-pa)
    return triangles@rotation.T+translation, rotation, translation


def query_pair(triangles, bvhs, transforms):
    moved, rotation, translation = transform(triangles[1], transforms)
    query = geometry.surface_query(triangles[0], moved,
                                   bvhs[0], bvhs[1].refit(moved))
    query['coordinate_frame'] = BODIES[0]
    query['relative_rotation_b_to_a'] = rotation.tolist()
    query['relative_translation_b_to_a_m'] = translation.tolist()
    query['closest_face_triangles_body5_m'] = [
        triangles[0][query['nearest_face_ids_zero_based'][0]].tolist(),
        moved[query['nearest_face_ids_zero_based'][1]].tolist()]
    return query


def source_point_queries(point_body5, transforms, triangles, bvhs):
    a, b = (transforms[name] for name in BODIES)
    ra, rb = np.asarray(a['rotation']), np.asarray(b['rotation'])
    pa, pb = np.asarray(a['translation_m']), np.asarray(b['translation_m'])
    world = ra@point_body5+pa
    points = [point_body5, rb.T@(world-pb)]
    result = {'point_body5_m': point_body5.tolist(), 'point_world_m': world.tolist(),
              'source_meshes': {}}
    for label in ('source_visual_obj', 'source_urdf_stl'):
        result['source_meshes'][label] = {}
        for body, point, mesh, bvh in zip(BODIES, points, triangles[label], bvhs[label]):
            point_triangle = np.repeat(point.reshape(1, 1, 3), 3, axis=1)
            query = geometry.surface_query(point_triangle, mesh, bvhB=bvh)
            result['source_meshes'][label][body] = {
                'point_body_local_m': point.tolist(),
                'distance_to_full_source_surface_m': query['surface_min_distance_m'],
                'nearest_source_face_zero_based': query['nearest_face_ids_zero_based'][1],
                'nearest_source_point_body_local_m': query['closest_points'][1],
                'nearest_source_triangle_body_local_m': mesh[query['nearest_face_ids_zero_based'][1]].tolist(),
                'all_surface_search_complete': query['all_surface_search_complete'],
                'diagnostic_oriented_winding_number': geometry.winding_number([point], mesh)[0],
                'winding_limit': 'Diagnostic only; no embedded-solid or containment classification is inferred.'}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('Output must be a new evidence file')
    if sha(CANDIDATE) != EXPECTED_CANDIDATE:
        raise ValueError('Frozen candidate SHA256 differs')
    states, contacts = rows(FROZEN/'state.jsonl.gz'), rows(FROZEN/'contacts.jsonl.gz')
    if set(states) != set(STEPS) or set(contacts) != set(STEPS):
        raise ValueError('Missing required frozen saved row')
    xml = ET.parse(CANDIDATE).getroot()
    assets = {asset.get('name'): asset for asset in xml.findall('asset/mesh')}
    bindings, triangles = {}, {}
    paths = {Path(__file__).resolve(), CANDIDATE, FROZEN/'state.jsonl.gz',
             FROZEN/'contacts.jsonl.gz', FROZEN/'model.xml', FROZEN/'arm.xml',
             Path(saved.__file__), Path(reader.__file__), Path(reader.BUILDER.__file__),
             Path(geometry.__file__), Path(lpcheck.__file__), ROOT/'URDF_SOURCES.json',
             ROOT/'SOURCE_MANIFEST.json', ROOT/'RIGHT_SOURCE_MANIFEST.json',
             ROOT/'urdf/nero_plus_manufacturer_flange.urdf'}
    for label, names in [('source_visual_obj', VISUAL), ('proxy_raw_mesh', PAIR)]:
        bindings[label], triangles[label] = [], []
        for name in names:
            geom = xml.find('.//geom[@name="'+name+'"]')
            binding = reader._binding(CANDIDATE, xml, assets, geom)
            bindings[label].append(binding)
            triangles[label].append(reader.mesh_triangles(binding))
            paths.add(Path(binding['file']))
    # Local manufacturer URDF source collision origins are identity; no mesh
    # scale is specified. Keep this second source representation independent.
    urdf = ET.parse(ROOT/'urdf/nero_plus_manufacturer_flange.urdf').getroot()
    bindings['source_urdf_stl'], triangles['source_urdf_stl'] = [], []
    for link in ('link5', 'gripper_flange'):
        collision = urdf.find('link[@name="'+link+'"]/collision')
        origin = collision.find('origin')
        if origin.get('xyz', '0 0 0') != '0 0 0' or origin.get('rpy', '0 0 0') != '0 0 0':
            raise ValueError('Unaccounted URDF source collision transform')
        mesh = collision.find('geometry/mesh')
        if mesh.get('scale', '1 1 1') != '1 1 1':
            raise ValueError('Unaccounted URDF source mesh scale')
        path = (ROOT/'urdf'/mesh.get('filename')).resolve()
        vertices, faces = reader.BUILDER.read_mesh(path)
        triangles['source_urdf_stl'].append(vertices[faces])
        bindings['source_urdf_stl'].append({'file': str(path), 'link': link,
                                            'origin_xyz_m': [0., 0., 0.],
                                            'origin_rpy_rad': [0., 0., 0.],
                                            'scale': [1., 1., 1.]})
        paths.add(path)
    before = snapshot(paths)
    topology = {label: [geometry.mesh_topology(t) for t in tris]
                for label, tris in triangles.items()}
    for entry in (entry for entries in topology.values() for entry in entries):
        entry['note'] = ('Exact-coordinate welding of the supplied parsed mesh; '
                         'closed consistently oriented edges do not prove an embedded solid.')
    original_allowed = saved.SAFE_APIS
    saved.SAFE_APIS = {'mj_kinematics', 'mj_name2id', 'mj_id2name'}
    post_transforms, compiled_proxies, compiled_audit = {}, [], []
    try:
        with saved.static_only() as api_audit:
            model = mujoco.MjModel.from_xml_path(str(CANDIDATE))
            model_before = saved.fields(model)
            bodyids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b) for b in BODIES]
            for name, binding in zip(PAIR, bindings['proxy_raw_mesh']):
                gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                mid = int(model.geom_dataid[gid])
                start, count = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
                compiled = np.asarray(model.mesh_vert[start:start+count], dtype=float)
                rotation = reader.BUILDER.quaternion_matrix(' '.join(map(str, model.geom_quat[gid])))
                compiled = compiled@rotation.T+model.geom_pos[gid]
                # MuJoCo imports all OBJ vertices, including vertices not used
                # by any face. Compare the same full vertex population here.
                raw_vertices, raw_faces = reader.BUILDER.read_mesh(binding['file'])
                raw_rotation = reader.BUILDER.quaternion_matrix(' '.join(map(str, binding['quat'])))
                raw_vertices = (raw_vertices*np.asarray(binding['scale']))@raw_rotation.T+binding['pos']
                delta = max(float(cKDTree(raw_vertices).query(compiled)[0].max()),
                            float(cKDTree(compiled).query(raw_vertices)[0].max()))
                if delta > 2e-8:
                    raise ValueError('Compiled mesh differs from body-local raw vertices')
                hull = ConvexHull(compiled)
                # MuJoCo 3.2.3 stores the actual Qhull faces in mesh_graph.
                # Its hull is built before the final float32 recentering; a new
                # Qhull call on rotated float32 points can change coplanar face
                # triangulations. Retain the actual compiled graph here.
                # Layout: 2 header ints, 3*numvert+3*numface graph entries,
                # then 3*numface mesh-global vertex ids for oriented faces.
                graph_adr = int(model.mesh_graphadr[mid])
                nv, nf = map(int, model.mesh_graph[graph_adr:graph_adr+2])
                graph_vertex_ids = np.asarray(model.mesh_graph[
                    graph_adr+2+nv:graph_adr+2+2*nv], dtype=int)
                face_adr = graph_adr+2+3*nv+3*nf
                graph_faces = np.asarray(model.mesh_graph[
                    face_adr:face_adr+3*nf], dtype=int).reshape(-1, 3)
                if (graph_adr < 0 or graph_faces.min() < 0 or
                        graph_faces.max() >= len(compiled) or
                        set(graph_vertex_ids) != set(graph_faces.reshape(-1))):
                    raise ValueError('Invalid actual MuJoCo convex mesh graph')
                compiled_proxies.append(compiled[graph_faces])
                compiled_audit.append({'geom': name, 'mesh_id': mid,
                    'compiled_vertex_count': len(compiled), 'hull_faces': nf,
                    'hull_vertices': nv, 'compiled_body_local_vertices': compiled.tolist(),
                    'raw_total_vertices': len(raw_vertices),
                    'raw_face_used_vertices': len(np.unique(raw_faces)),
                    'hull_faces_compiled_vertex_ids': graph_faces.tolist(),
                    'hull_graph_support_compiled_vertex_ids': graph_vertex_ids.tolist(),
                    'independent_all_compiled_ConvexHull_faces': len(hull.simplices),
                    'independent_all_compiled_ConvexHull_vertices': len(hull.vertices),
                    'raw_compiled_bidirectional_vertex_max_error_m': delta,
                    'mesh_graph_address': int(model.mesh_graphadr[mid]),
                    'mesh_asset_attributes': dict(assets[name].attrib),
                    'method': 'Actual compiled mesh_graph face_globalid triangles, applied to actual compiled mesh vertices restored with compiled geom pose. LP reconstructs halfspaces from actual graph support vertices. No model or mesh is modified.',
                    'graph_layout_primary_source': 'https://raw.githubusercontent.com/google-deepmind/mujoco/3.2.3/src/user/user_mesh.cc',
                    'graph_layout_primary_source_lines': [1496, 1509, 1568, 1589]})
            for step in STEPS:
                data = mujoco.MjData(model)
                if model.nq != 18 or len(states[step]['full_qpos']) < model.nq:
                    raise ValueError('Unexpected candidate/frozen qpos binding')
                data.qpos[:] = states[step]['full_qpos'][:model.nq]
                protected = {key: np.array(getattr(data, key), copy=True)
                             for key in ('qpos', 'qvel', 'qacc', 'ctrl', 'act', 'qacc_warmstart')}
                time_before = data.time
                mujoco.mj_kinematics(model, data)
                post_transforms[step] = {b: {'rotation': data.xmat[bid].reshape(3, 3).tolist(),
                                             'translation_m': data.xpos[bid].tolist()}
                                         for b, bid in zip(BODIES, bodyids)}
                if data.time != time_before or any(not np.array_equal(v, getattr(data, k))
                                                   for k, v in protected.items()):
                    raise ValueError('Integration state changed during static FK')
            if model_before != saved.fields(model):
                raise ValueError('Compiled model fields changed')
            api_audit = dict(api_audit)
            api_audit['allowed_api_calls'] = dict(api_audit['allowed_api_calls'])
            api_audit.update({'model_constructor_calls': 1, 'data_constructor_calls': len(STEPS),
                              'model_public_fields_unchanged': True,
                              'protected_state_buffers_unchanged_after_FK': True,
                              'candidate_nq': model.nq, 'full_frozen_qpos_length': len(states[STEPS[0]]['full_qpos']),
                              'qpos_binding': 'First 18 frozen coordinates in unchanged candidate robot joint order; remaining scene/object coordinates are not used.'})
    finally:
        saved.SAFE_APIS = original_allowed
    triangles['proxy_compiled_convex_hull'] = compiled_proxies
    bvhs = {label: [geometry.BVH(t) for t in tris] for label, tris in triangles.items()}
    lp_sources = {body: {'vertices': np.unique(t.reshape(-1, 3), axis=0),
                         'source': Path(binding['file']), 'geom_name': name}
                  for body, t, binding, name in zip(BODIES, compiled_proxies,
                      bindings['proxy_raw_mesh'], PAIR)}
    results = []
    for step in STEPS:
        selected = [c for c in contacts[step]['contacts'] if set(c['geom']) == set(PAIR)]
        if len(selected) > 1:
            raise ValueError('Multiple saved target-pair contacts')
        cached = None
        if selected:
            c = selected[0]
            cached = {body: {'rotation': np.asarray(pose['rotation_world']).reshape(3, 3).tolist(),
                              'translation_m': pose['position_world_m']}
                      for body, pose in zip(c['body'], c['cached_body_pose'])}
        row = {'step': step, 'time_s': states[step]['time_s'], 'phase': states[step]['phase'],
               'full_qpos': states[step]['full_qpos'],
               'saved_contact_timing': contacts[step]['timing'],
               'saved_target_pair_contact': selected,
               'cached_body_transforms': cached,
               'post_qpos_static_FK_body_transforms': post_transforms[step],
               'pose_cases': {}}
        for label, transforms in [('cached_RK4_contact_pose', cached),
                                  ('post_qpos_static_FK_pose', post_transforms[step])]:
            if transforms is None:
                row['pose_cases'][label] = {'status': 'UNAVAILABLE_NO_TARGET_CONTACT_BODY_POSES',
                    'classification': 'UNKNOWN_CACHED_POSE_NOT_RECONSTRUCTED'}
                continue
            queries = {source: query_pair(tris, bvhs[source], transforms)
                       for source, tris in triangles.items()}
            overlap = lpcheck.check_case(lp_sources, transforms)
            overlap['conditioning_origin_body5_mm'] = overlap.pop('conditioning_origin_body13_mm')
            overlap['common_ball_center_body5_m'] = overlap.pop('common_ball_center_body13_m')
            for hull in overlap['hulls']:
                hull['raw_mesh_path'] = hull.pop('raw_stl_path')
                hull['raw_mesh_sha256'] = hull.pop('raw_stl_sha256')
                hull['unique_compiled_hull_vertices'] = hull.pop('unique_raw_vertices')
            obj, stl = (queries[key] for key in ('source_visual_obj', 'source_urdf_stl'))
            actual = queries['proxy_compiled_convex_hull']
            if obj['strict_transverse_witness'] is not None:
                classification = 'SOURCE_VISUAL_STRICT_SURFACE_CROSSING'
            elif obj['all_surface_search_complete'] and obj['surface_min_distance_m'] > 1e-9:
                classification = ('PROXY_CONTACT_WITH_POSITIVE_SOURCE_VISUAL_SURFACE_CLEARANCE'
                                  if overlap['strict_convex_solid_interior_overlap'] else
                                  'SOURCE_AND_PROXY_SURFACES_SEPARATED')
            else:
                classification = 'UNKNOWN_NO_STRICT_SOURCE_SURFACE_CLASSIFICATION'
            point_checks = {'proxy_common_ball_center': source_point_queries(
                np.asarray(overlap['common_ball_center_body5_m']), transforms, triangles, bvhs)}
            if label == 'cached_RK4_contact_pose':
                body5 = transforms[BODIES[0]]
                local_contact = np.asarray(body5['rotation']).T@(
                    np.asarray(selected[0]['position_world_m'])-body5['translation_m'])
                point_checks['saved_contact_position'] = source_point_queries(
                    local_contact, transforms, triangles, bvhs)
            row['pose_cases'][label] = {'status': 'COMPUTED', 'queries': queries,
                'proxy_common_ball_certificate': overlap, 'classification': classification,
                'local_proxy_vs_full_source_point_checks': point_checks,
                'source_stl_strict_crossing': stl['strict_transverse_witness'] is not None,
                'solid_containment_classification': 'UNKNOWN_SOURCE_LINK5_TOPOLOGY_NOT_CLOSED_MANIFOLD',
                'actual_proxy_overlap_certified': overlap['strict_convex_solid_interior_overlap'],
                'source_surface_fully_separated': obj['all_surface_search_complete'] and obj['surface_min_distance_m'] > 1e-9,
                'proxy_surface_strict_crossing': actual['strict_transverse_witness'] is not None}
        if cached is not None:
            row['cached_vs_post_FK_max_transform_abs_difference'] = {
                body: {'rotation': float(np.max(np.abs(np.array(cached[body]['rotation'])-post_transforms[step][body]['rotation']))),
                       'translation_m': float(np.max(np.abs(np.array(cached[body]['translation_m'])-post_transforms[step][body]['translation_m'])))}
                for body in BODIES}
        results.append(row)
        print(json.dumps({'step': step, 'results': {label: {
            'classification': case['classification'],
            'source_visual_gap_mm': case.get('queries', {}).get('source_visual_obj', {}).get('surface_min_distance_m', 0)*1000,
            'source_stl_gap_mm': case.get('queries', {}).get('source_urdf_stl', {}).get('surface_min_distance_m', 0)*1000,
            'proxy_common_ball_radius_mm': case.get('proxy_common_ball_certificate', {}).get('common_ball_radius_lp_mm')}
            for label, case in row['pose_cases'].items()}}), flush=True)
    after = snapshot(paths)
    if before != after:
        raise ValueError('Input files changed during static analysis')
    payload = {'schema': 'd3-frozen-link5-flange-static-geometry-v1',
        'pair': list(PAIR), 'bodies': list(BODIES), 'steps': list(STEPS),
        'authorization_scope': 'PR4 comment6262830612 static geometry/FK only; one pair, no parameter scan.',
        'method': 'Existing continuous full-triangle BVH; original visual OBJ and local URDF collision STL surfaces; actual MuJoCo compiled mesh_graph collision hull faces, with original compiled support vertex coordinates restored to body frame; existing common-ball LP on actual graph support vertices with long-double primal and independent dual checks.',
        'limitations': ['No dynamics, collision generation, constraints, filtering changes, mesh/model changes, actions, forces, or new physical attempts.',
            'Saved contacts are final RK4 internal-stage cached poses; saved qpos is post integration. They are evaluated as different cases.',
            'Positive source surface gap excludes triangle intersection at the stated pose. Link5 is not closed manifold, so solid containment and physical penetration are UNKNOWN.',
            'Strict proxy interior-overlap witnesses characterize the convex collision representation; common-ball radius is not a physical penetration depth.',
            'SciPy geometry uses floating-point predicates; full nearest search pruning tolerance is 1e-12m.'],
        'software': {'python': sys.version, 'numpy': np.__version__, 'scipy': scipy.__version__, 'mujoco': mujoco.__version__, 'executable': sys.executable},
        'physics_steps': 0, 'call_audit': api_audit, 'bindings': bindings,
        'source_topology': topology, 'compiled_proxy_audit': compiled_audit,
        'input_sha256_before': before, 'input_sha256_after': after,
        'inputs_unchanged': True, 'results': results}
    with args.output.open('x') as stream:
        stream.write(json.dumps(payload, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
