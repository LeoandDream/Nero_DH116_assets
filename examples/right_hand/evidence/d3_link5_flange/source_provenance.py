"""Read-only source DAE/STL to delivered OBJ tracing for two arm links.

No simulator import. Triangle correspondence supplies a conservative surface
deviation upper bound, not a sampled Hausdorff claim or an acceptance threshold.
"""
from pathlib import Path
import hashlib
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'tools'))
from build_right_proxy76 import read_mesh

NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def dae_triangles(path):
    """Read the existing triangle-only, matrix-only Collada scene instances."""
    tree = ET.parse(path).getroot()
    unit = float(tree.find('c:asset/c:unit', NS).get('meter'))
    if tree.findtext('c:asset/c:up_axis', namespaces=NS) != 'Z_UP':
        raise ValueError('Only the supplied Z_UP scenes are supported')
    geometries = {}
    for geometry in tree.findall('c:library_geometries/c:geometry', NS):
        mesh = geometry.find('c:mesh', NS)
        arrays = {}
        for source in mesh.findall('c:source', NS):
            accessor = source.find('c:technique_common/c:accessor', NS)
            arrays[source.get('id')] = np.fromstring(source.findtext('c:float_array', namespaces=NS), sep=' ').reshape(-1, int(accessor.get('stride')))
        vertices = {v.get('id'): next(i.get('source')[1:] for i in v.findall('c:input', NS) if i.get('semantic') == 'POSITION') for v in mesh.findall('c:vertices', NS)}
        triangles = []
        for primitive in mesh.findall('c:triangles', NS):
            inputs = primitive.findall('c:input', NS)
            vertex = next(i for i in inputs if i.get('semantic') == 'VERTEX')
            stride = 1 + max(int(i.get('offset')) for i in inputs)
            indices = np.fromstring(primitive.findtext('c:p', namespaces=NS), sep=' ', dtype=int).reshape(-1, stride)[:, int(vertex.get('offset'))].reshape(-1, 3)
            triangles.append(arrays[vertices[vertex.get('source')[1:]]][indices, :3])
        geometries[geometry.get('id')] = np.concatenate(triangles)
    triangles, instances = [], []
    for node in tree.findall('c:library_visual_scenes/c:visual_scene/c:node', NS):
        matrix = np.fromstring(node.findtext('c:matrix', namespaces=NS), sep=' ').reshape(4, 4)
        if not np.array_equal(matrix[3], [0, 0, 0, 1]):
            raise ValueError('Expected supplied affine matrix layout')
        for instance in node.findall('c:instance_geometry', NS):
            name = instance.get('url')[1:]
            triangles.append((geometries[name] @ matrix[:3, :3].T + matrix[:3, 3]) * unit)
            instances.append({'geometry': name, 'matrix_rows': matrix.tolist()})
    return np.concatenate(triangles), {'unit_meter': unit, 'up_axis': 'Z_UP', 'instances': instances}


def triangle_deviation_bound(a, b):
    """Both directions; match a triangle to an actual triangle by centroid.

For each matched pair, the maximum vertex-to-nearest-vertex distance bounds
the distance between their convex triangle surfaces. Report the maximum of
both directions; duplicate/unused vertex numbering has no effect.
"""
    bounds = []
    for first, second in ((a, b), (b, a)):
        _, indices = cKDTree(second.mean(axis=1)).query(first.mean(axis=1))
        paired = second[indices]
        distances = np.linalg.norm(first[:, :, None] - paired[:, None, :], axis=-1)
        forward = distances.min(axis=2).max(axis=1)
        backward = distances.min(axis=1).max(axis=1)
        bounds.append(float(np.maximum(forward, backward).max()))
    return {'triangle_counts': [len(a), len(b)], 'directional_max_triangle_bound_m': bounds,
            'symmetric_surface_deviation_upper_bound_m': max(bounds),
            'method': 'Full triangle coverage with actual matched triangle vertex-set distance; no mesh sampling or geometry modification'}


def run():
    urdf = ROOT / 'urdf/nero_plus_manufacturer_flange.urdf'
    records = json.loads((ROOT / 'URDF_SOURCES.json').read_text())
    manifest = json.loads((ROOT / 'SOURCE_MANIFEST.json').read_text())
    model = ROOT / 'examples/right_hand/candidates/finger13_local/model.xml'
    xml = ET.parse(model).getroot()
    meshes = {m.get('name'): m for m in xml.findall('asset/mesh')}
    result = {'scope': 'Only supplied source surfaces; does not certify hardware CAD or watertight solids', 'inputs_sha256': {}, 'links': {}, 'new_physics_steps': 0, 'simulator_imports': 0}
    files = {urdf, model, ROOT / 'URDF_SOURCES.json', ROOT / 'SOURCE_MANIFEST.json', Path(__file__)}
    for link in ('link5', 'gripper_flange'):
        element = ET.parse(urdf).find(f'.//link[@name="{link}"]')
        paths = {role: (urdf.parent / element.find(f'{role}/geometry/mesh').get('filename')).resolve() for role in ('visual', 'collision')}
        origins = {role: element.find(f'{role}/origin').attrib for role in paths}
        if any(origin != {'xyz': '0 0 0', 'rpy': '0 0 0'} and origin != {'rpy': '0 0 0', 'xyz': '0 0 0'} for origin in origins.values()):
            raise ValueError('Source origin changed')
        name = 'robot0_original_urdf_visual_' + link
        mesh = meshes[name]
        obj = (model.parent / mesh.get('file')).resolve()
        geom = xml.find(f'.//geom[@name="{name}"]')
        vertices, faces = read_mesh(obj)
        actual = vertices[faces]
        source, metadata = dae_triangles(paths['visual'])
        stl_v, stl_f = read_mesh(paths['collision'])
        expected_obj = next(r for r in manifest['assets'] if r['asset'] == name)
        for path in paths.values():
            record = next(r for r in records if ROOT / r['file'] == path)
            if hashlib.sha256(path.read_bytes()).hexdigest() != record['sha256']:
                raise ValueError('Source manifest SHA mismatch')
        if hashlib.sha256(obj.read_bytes()).hexdigest() != expected_obj['sha256']:
            raise ValueError('Delivered visual SHA mismatch')
        result['links'][link] = {'urdf_origin': origins, 'visual_geom': dict(geom.attrib), 'mesh_scale': mesh.get('scale', '1 1 1'),
            'source_visual': str(paths['visual'].relative_to(ROOT)), 'source_collision': str(paths['collision'].relative_to(ROOT)),
            'delivered_visual': str(obj.relative_to(ROOT)), 'source_DAE': metadata,
            'DAE_to_OBJ': triangle_deviation_bound(source, actual),
            'STL_to_OBJ': triangle_deviation_bound(stl_v[stl_f], actual)}
        files.update(paths.values()); files.add(obj)
    result['inputs_sha256'] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
    return result


if __name__ == '__main__':
    output = Path(__file__).with_name('source-provenance.json')
    if output.exists():
        raise SystemExit('Use an untouched evidence output; no overwrite')
    value = run()
    output.write_text(json.dumps(value, indent=2) + '\n')
    print(json.dumps({name: {'DAE_to_OBJ_m': row['DAE_to_OBJ']['symmetric_surface_deviation_upper_bound_m'], 'STL_to_OBJ_m': row['STL_to_OBJ']['symmetric_surface_deviation_upper_bound_m']} for name, row in value['links'].items()}, indent=2))
