#!/usr/bin/env python3
"""Recheck two saved RK4 caches using exported compiled vertices, without MuJoCo.

The extraction deviation is documented separately; this entry never compiles a
model. Continuous surface queries and exact support certificates use only the
existing STL/OBJ, cached poses and historical compiled-vertex SHA.
"""
import argparse
import ast
from decimal import Decimal, localcontext
import gzip
import hashlib
import importlib.util
import itertools
import json
import os
from pathlib import Path
import sys
for k in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[k] = '1'
import numpy as np
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'examples/right_hand'))
import finger22_contact_regions as C
import proxy76_geometry as G

RUN = ROOT/'examples/right_hand/evidence/d3_finger22_exploratory/run'
NAME = 'gripper0_proxy76_finger23_link_06'
BODY = 'gripper0_finger23_link'
SOURCE = ROOT/'meshes/dh116_right/source/finger23_link.STL'
DELIVERY = ROOT/'meshes/dh116/gripper0_finger23_link-730786ab.stl'
RAW = ROOT/'meshes/dh116/gripper0_moderate_finger23_link_6-a018289e.obj'


def read(path):
    with (gzip.open(path, 'rt') if str(path).endswith('.gz') else open(path)) as f:
        return json.load(f)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def decimal_checker():
    # Reuse only the four reviewed arithmetic functions, not top-level IO.
    path = ROOT/'examples/right_hand/evidence/d3_geometry/triangle/enrich_results.py'
    nodes = [n for n in ast.parse(path.read_text()).body if isinstance(n, ast.FunctionDef)
             and n.name in ('dot', 'sub', 'cross', 'decimal_witness')]
    assert len(nodes) == 4
    scope = {'Decimal': Decimal, 'localcontext': localcontext}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), scope)
    return scope['decimal_witness']


def pose(value):
    r = np.asarray(value['rotation_world'], float).reshape(3, 3)
    p = np.asarray(value['position_world_m'], float)
    assert p.shape == (3,) and np.isfinite(p).all() and np.isfinite(r).all()
    assert np.linalg.norm(r.T@r-np.eye(3), np.inf) <= 256*np.finfo(float).eps
    assert np.linalg.det(r) > 0
    return r, p


def cached_box(fields, finger_pose, bowl_pose, builder):
    """Same affine arithmetic as the frozen adapter; no simulator object."""
    rf, pf = pose(finger_pose); rb, pb = pose(bowl_pose)
    lr = builder.quaternion_matrix(' '.join(map(str, fields['geom_quat'])))
    lp, size = np.array(fields['geom_pos']), np.array(fields['geom_size'])
    rr = rf.T@rb@lr; tt = rf.T@(rb@lp+pb-pf)
    v = np.array(list(itertools.product((-1, 1), repeat=3)))*size
    unit = 2.**-53; gamma = 256*unit/(1-256*unit)
    scale = 1+max(np.max(abs(pb)), np.max(abs(pf)))+np.max(size)+np.max(abs(lp))
    scale *= max(1., np.linalg.norm(rf, np.inf))*max(1., np.linalg.norm(rb, np.inf))*max(1., np.linalg.norm(lr, np.inf))
    guard = float(2*(gamma+np.linalg.norm(rf.T@rf-np.eye(3), np.inf))*scale)
    exact = C.affine_vertices(v, rr, tt)
    return C.CertifiedHull(exact, exact), guard


def write(path, value):
    raw = (json.dumps(value, indent=2, allow_nan=False)+'\n').encode()
    if path.suffix == '.gz':
        with gzip.open(path, 'wb') as f: f.write(raw)
    else: path.write_bytes(raw)


def exact_crossing(query):
    h = query['strict_transverse_witness']
    if h is None: return None
    # Reuse the 60-digit independent witness recheck, then additionally verify
    # the signs/edge and barycentric membership by exact binary64 rationals.
    decimal = decimal_checker()(h)
    a, b = [[tuple(map(C.rational, p)) for p in h[k]] for k in ('triangle_a', 'triangle_b')]
    src, dst = (a, b) if h['edge_from_triangle'] == 0 else (b, a)
    p, q = [src[i] for i in h['edge_vertices']]; u, v, w = dst
    sub = lambda x, y: tuple(a-b for a, b in zip(x, y))
    n = C.cross(sub(v, u), sub(w, u)); n2 = C.dot(n, n)
    dp, dq = C.dot(sub(p, u), n), C.dot(sub(q, u), n)
    t = dp/(dp-dq); x = tuple(a+t*(b-a) for a, b in zip(p, q))
    beta = C.dot(C.cross(sub(x, u), sub(w, u)), n)/n2
    gamma = C.dot(C.cross(sub(v, u), sub(x, u)), n)/n2
    bary = [1-beta-gamma, beta, gamma]
    assert dp*dq < 0 and 0 < t < 1 and all(k > 0 for k in bary)
    return {'exact_dyadic_strict_crossing': True, 'edge_parameter_exact': str(t),
            'barycentric_exact': list(map(str, bary)), 'point_exact_m': list(map(str, x)),
            'decimal_recheck': decimal,
            'scope': 'Exact on stored binary64 body-frame triangles. Transform arithmetic guard is reported separately; no solid-material certificate.'}


def separating_translation_upper_bound(hull, box, guard):
    """A hypothetical separation certificate, never a simulator state edit.

    Axis search is only a proposal. Exact support inequalities for ALL vertices
    independently prove a separating translation and an upper bound on its
    norm. No claim to the exact minimum is necessary or made.
    """
    directions = box.vertices[[1, 2, 4]]-box.vertices[0]
    edges = np.concatenate([hull.vertices[hull.faces[:, (k+1)%3]]-hull.vertices[hull.faces[:, k]] for k in range(3)])
    axes = np.concatenate([hull.normals, box.normals,
                           np.cross(edges[:, None, :], directions).reshape(-1, 3)])
    lengths = np.linalg.norm(axes, axis=1); axes = axes[lengths > 0]/lengths[lengths > 0, None]
    aa = hull.vertices@axes.T; bb = box.vertices@axes.T
    positive = bb.max(0)-aa.min(0); negative = aa.max(0)-bb.min(0)
    index = int(np.argmin(np.minimum(positive, negative)))
    direction = axes[index]*(1 if positive[index] <= negative[index] else -1)
    n = tuple(map(C.rational, direction)); n2 = C.dot(n, n)
    amin = min(C.dot(n, p) for p in hull.points)
    bmax = max(C.dot(n, p) for p in box.points)
    overlap = bmax-amin
    reserve = C.rational(guard)*sum(map(abs, n))+C.rational(1e-15)
    amount = (overlap+reserve)/n2
    shift = tuple(amount*x for x in n)
    gap = min(C.dot(n, tuple(x+s for x, s in zip(p, shift))) for p in hull.points)-bmax
    assert gap > C.rational(guard)*sum(map(abs, n))
    length2 = C.dot(shift, shift)
    bound = float(np.nextafter(np.sqrt(float(length2)), np.inf))
    while C.rational(bound)**2 < length2: bound = float(np.nextafter(bound, np.inf))
    return {'certified_translation_norm_upper_m': bound, 'shift_m': list(map(float, shift)),
            'shift_exact_m': list(map(str, shift)), 'separating_axis': list(map(float, n)),
            'all_translated_hull_vertices_strictly_beyond_all_box_vertices': True,
            'positive_projection_gap_exact': str(gap),
            'scope': 'Upper bound on compiled-convex translational escape distance, not exact MTD, source-material penetration or a threshold waiver.'}


def section_segments(triangles, z):
    """Only a plotted plane section of existing surfaces; never a new mesh."""
    result = []
    for tri in triangles:
        points = []
        for a, b in zip(tri, np.roll(tri, -1, axis=0)):
            da, db = a[2]-z, b[2]-z
            if da == 0: points.append(a[:2])
            if da*db < 0: points.append((a+(b-a)*da/(da-db))[:2])
        unique = np.unique(points, axis=0) if points else []
        if len(unique) == 2: result.append(unique)
    return np.array(result)


def plot_sections(output, source, raw, compiled, results):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    z = .02093
    fig, axes = plt.subplots(1, 2, figsize=(9, 4.8), constrained_layout=True)
    for ax, row in zip(axes, [r for r in results if r['box'] == 'white_bowl_1_g23']):
        box = np.array(row['bowl_vertices_in_finger_m']); faces = np.array(row['bowl_faces'])
        for tri, color, label, style, width in (
                (source, '#146c43', 'Manufacturer STL', '-', 2.),
                (raw, '#888888', 'Raw _06', '-', 2.2),
                (compiled, '#b42318', 'Compiled _06', '--', 1.5),
                (box[faces], '#175cd3', 'Complete bowl g23 section', '-', 1.8)):
            segments = section_segments(tri, z)
            ax.add_collection(LineCollection(segments*1000, colors=color, linewidths=width,
                                             linestyles=style, label=label))
        marker = np.array(row['MPR_point_finger_local_m'])*1000
        ax.plot(marker[0], marker[1], 'kx', ms=7, label='MPR marker projected to plane')
        ax.text(.03, .03, f"native depth {abs(row['native_contact']['distance_m'])*1000:.6f} mm\n"
                f"certified convex escape <= {row['compiled_convex_separating_translation']['certified_translation_norm_upper_m']*1000:.6f} mm",
                transform=ax.transAxes, fontsize=8)
        ax.set(xlim=(2.4, 6.2), ylim=(.5, 4.1), aspect='equal', xlabel='Finger23 local X (mm)',
               ylabel='Finger23 local Y (mm)', title=f"Saved RK4 step {row['step']}")
        ax.grid(alpha=.2)
    axes[0].legend(loc='upper left', fontsize=7)
    fig.suptitle('Existing surfaces at body-local Z = 20.930 mm\nMPR marker is not a certified source-surface contact point', fontsize=10)
    fig.savefig(output/'local-section.png', dpi=180)
    plt.close(fig)
    return {'plane': 'finger23 body-local Z', 'z_m': z, 'actual_RK4_steps': [4768, 4769],
            'marker': 'MPR position projected onto section; not a source-surface point',
            'raw_compiled_outlines': 'Overlap visually at this scale; difference separately bounded in geometry evidence.'}


def main(output):
    if output.exists(): raise ValueError('New verification output only')
    assert 'mujoco' not in sys.modules
    evidence = read(HERE/'geometry.json.gz')
    assert evidence['baseline'] == 'f0b37997e38ed34c46a3dd7f223168eff2d3126b'
    for path, digest in evidence['inputs_sha256'].items():
        assert sha(ROOT/path) == digest, path
    geometry = load_module('finger23_surface_geometry', ROOT/'examples/right_hand/evidence/d3_geometry/triangle/surface_geometry.py')
    builder = load_module('finger23_existing_mesh_reader', ROOT/'tools/build_right_proxy76.py')
    audit = evidence['compiled_hull_audit']
    vertices = np.asarray(audit['compiled_body_local_support_vertices_m'], dtype=np.float64)
    ids = {old: new for new, old in enumerate(audit['support_vertex_ids'])}
    faces = np.array([[ids[i] for i in f] for f in audit['face_vertex_ids']], dtype=int)
    compiled = vertices[faces]
    historical = read(ROOT/'examples/right_hand/evidence/d3_finger22_contact_regions/evidence.json.gz')['hull_certificates'][NAME]
    support_sha = hashlib.sha256(vertices.tobytes()).hexdigest()
    assert support_sha == historical['support_vertices_sha256']
    graph_sha = hashlib.sha256(np.array(audit['face_vertex_ids'], dtype=np.int64).tobytes()).hexdigest()
    assert graph_sha == historical['graph_face_ids_sha256']
    h = C.CertifiedHull(vertices)
    rv, rfaces = builder.read_mesh(RAW); raw = rv[rfaces]
    used = np.unique(raw.reshape(-1, 3), axis=0); raw_h = C.CertifiedHull(used)
    # The raw OBJ is already body-local. Never apply compiler compensation twice.
    binding = evidence['raw_proxy_binding']
    assert binding['pos'] == [0., 0., 0.] and binding['quat'] == [1., 0., 0., 0.]
    assert binding['scale'] == [1., 1., 1.]
    bound = max(float(cKDTree(h.vertices).query(used)[0].max()), float(cKDTree(used).query(h.vertices)[0].max()))
    # A floating matching-distance estimate, not an outward-rounded certificate.
    assert bound == evidence['raw_compiled_convex_Hausdorff_upper_bound_m']
    source, delivery = geometry.read_stl(SOURCE), geometry.read_stl(DELIVERY)
    snapshot = read(RUN/'robot-physical-preflight.json.gz')['actual_snapshot']
    geoms = {g['name']: g for g in snapshot['objects']['GEOM']}
    selected = {}
    with gzip.open(RUN/'contacts.jsonl.gz', 'rt') as f:
        for line in f:
            row = json.loads(line)
            if row['step'] in (4768, 4769): selected[row['step']] = row
            if row['step'] == 4769: break
    assert set(selected) == {4768, 4769}
    checks = []; confirmed = 0
    for result in evidence['results']:
        step, name = result['step'], result['box']; row = selected[step]
        original = next(c for c in row['contacts'] if NAME in c['geom'] and name in c['geom'])
        assert original == result['native_contact'] and row['timing'] == result['cache_timing']
        i = original['geom'].index(NAME); j = 1-i
        assert original['cached_body_pose'][i] == row['body_poses'][BODY] == result['finger_pose']
        assert original['cached_body_pose'][j] == row['body_poses']['white_bowl_1_main']
        assert original['filter_reasons'] == []
        assert geoms[NAME]['bindings']['geom_bodyid'] == BODY
        assert geoms[name]['bindings']['geom_bodyid'] == 'white_bowl_1_main'
        fa, fb = geoms[NAME]['fields'], geoms[name]['fields']
        assert (fa['geom_contype'] & fb['geom_conaffinity']) or (fb['geom_contype'] & fa['geom_conaffinity'])
        assert fa['geom_margin'] == fb['geom_margin'] == fa['geom_gap'] == fb['geom_gap'] == 0
        assert fb['geom_type'] == 6
        box, guard = cached_box(fb, row['body_poses'][BODY], original['cached_body_pose'][j], builder)
        assert [[str(x) for x in p] for p in box.points] == result['bowl_vertices_exact_m']
        assert guard == result['coordinate_arithmetic_guard_m']
        box_triangles = box.vertices[box.faces]
        queries = {}
        for label, triangles in (('manufacturer', source), ('delivery_raw', delivery), ('proxy_raw', raw), ('proxy_compiled', compiled)):
            query = geometry.surface_query(triangles, box_triangles)
            certificate = exact_crossing(query)
            assert certificate and certificate['exact_dyadic_strict_crossing']
            assert query['surface_min_distance_m'] == 0
            # Verify evidence witnesses against original face arrays, not just their stored arithmetic.
            recorded = result['surface_queries'][label]
            witness = recorded['strict_transverse_witness']
            ia, ib = recorded['nearest_face_ids_zero_based']
            assert np.array_equal(triangles[ia], witness['triangle_a'])
            assert np.array_equal(box_triangles[ib], witness['triangle_b'])
            assert exact_crossing(recorded)['exact_dyadic_strict_crossing']
            queries[label] = {'minimum_surface_distance_m': 0., 'exact_crossing': True,
                              'original_face_ids_zero_based': [ia, ib]}
            confirmed += 1
        for label, hull in (('raw_proxy_overlap', raw_h), ('compiled_proxy_overlap', h)):
            overlap = C.intersection_certificate(hull, box, guard)
            assert overlap['status'] == result[label]['status'] == 'STRICT_OVERLAP'
        assert C.contact_faces(h, box, guard) == result['all_compiled_contact_patches']
        separation = result['compiled_convex_separating_translation']
        shift = tuple(map(C.rational, separation['shift_exact_m']))
        axis = tuple(map(C.rational, separation['separating_axis']))
        gap = min(C.dot(axis, tuple(x+s for x, s in zip(p, shift))) for p in h.points)-max(C.dot(axis, p) for p in box.points)
        assert gap == C.rational(separation['positive_projection_gap_exact'])
        assert gap > C.rational(guard)*sum(map(abs, axis))
        upper = separation['certified_translation_norm_upper_m']
        assert C.dot(shift, shift) <= C.rational(upper)**2 and upper < .002
        rf, pf = pose(row['body_poses'][BODY])
        contact_point = rf.T@(np.array(original['position_world_m'])-pf)
        assert np.array_equal(contact_point, result['MPR_point_finger_local_m'])
        distances, points, source_ids = G.TriangleSurface(source).nearest(contact_point[None])
        nearest = result['nearest_surfaces_to_MPR_point']['manufacturer']
        assert nearest == {'distance_to_MPR_point_m': float(distances[0]), 'face': int(source_ids[0]), 'nearest_point_m': points[0].tolist()}
        margins = [-(C.dot(r[:3], tuple(map(C.rational, points[0])))+r[3])/np.linalg.norm(np.array(r[:3], float)) for r in box.planes]
        assert float(min(margins)) == result['nearest_manufacturer_point_box_clearance_m'] > guard
        checks.append({'step': step, 'box': name, 'surfaces': queries,
                       'convex_translation_upper_bound_m': upper, 'exact_support_separation_verified': True,
                       'source_boundary_point_inside_box_clearance_m': float(min(margins)),
                       'compiled_boundary_patch_count': len(result['all_compiled_contact_patches'])})
    assert len(checks) == 4 and confirmed == 16
    assert geometry.mesh_topology(source) == evidence['manufacturer_topology']
    assert geometry.mesh_topology(delivery) == evidence['delivery_raw_topology']
    assert 'mujoco' not in sys.modules
    output.mkdir(parents=True)
    plot_sections(output, source, raw, compiled, evidence['results'])
    pure_modules = [Path(__file__), ROOT/'examples/right_hand/finger22_contact_regions.py',
                    ROOT/'examples/right_hand/proxy76_geometry.py', ROOT/'tools/build_right_proxy76.py',
                    ROOT/'examples/right_hand/evidence/d3_geometry/triangle/surface_geometry.py',
                    ROOT/'examples/right_hand/evidence/d3_geometry/triangle/enrich_results.py']
    report = {'status': 'OFFLINE_GEOMETRY_RECHECK_PASS', 'saved_steps': [4768, 4769],
              'surface_crossing_certificates': confirmed, 'results': checks,
              'historical_compiled_support_vertices_sha256': support_sha,
              'historical_compiled_graph_face_ids_sha256': graph_sha,
              'raw_compiled_convex_Hausdorff_upper_bound_m': bound,
              'raw_compiled_bound_scope': 'Float64 matching-distance estimate, without outward rounding; not a formal 0.1 nm acceptance certificate.',
              'mujoco_imported_in_this_recheck': 'mujoco' in sys.modules,
              'this_entry_model_compilations_or_native_API_calls': 0,
              'task_wide_execution_deviation': read(HERE/'extraction-disclosure.json'),
              'source_code_sha256': {str(p.relative_to(ROOT)): sha(p) for p in pure_modules},
              'evidence_sha256': {p.name: sha(p) for p in (HERE/'geometry.json.gz', HERE/'extraction-disclosure.json')}}
    write(output/'verification.json', report)
    print(json.dumps({'status': report['status'], 'surface_crossings': confirmed, 'mujoco_imported': False,
                      'convex_translation_upper_mm': [r['convex_translation_upper_bound_m']*1000 for r in checks]}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    main(parser.parse_args().output.resolve())
