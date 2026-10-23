"""Re-execute the original source-feature partition, then attach to frozen L.

This consumes source/component/feature construction inputs, not delivered R OBJ.
The external builder is read-only; its sole output root is redirected to a new
work directory. Only wrist collision representation changes in the candidate.
"""
import argparse
from collections import Counter
from contextlib import redirect_stdout
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
BODIES = ('robot0_link5', 'robot0_gripper_flange')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--engineering-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--work-output', type=Path, required=True)
    a = p.parse_args()
    engineering = a.engineering_root.resolve()
    # Match the original GEOS 3.13.1 / Shapely 2.1.2 construction dependency.
    sys.path.insert(0, str(engineering/'.deps'))
    import shapely
    if shapely.__version__ != '2.1.2':
        raise ValueError('Use original construction dependencies')
    spec = importlib.util.spec_from_file_location('original_source_feature_builder', engineering/'build_engineering.py')
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    inputs = {engineering/'build_engineering.py', builder.PREV/'local_probe.py',
              builder.PREV/'source/flange-wall-frame.npz', builder.PREV/'evidence/local-plan.json',
              Path(__file__).resolve(), ROOT/'mjcf/nero_dh116.xml'}
    for name in ('link5', 'gripper_flange'):
        inputs.add(builder.PREV/'source'/f'{name}-original.npz')
        inputs.update((builder.V1/'build').glob(name+'-component-*.npz'))
    inputs.update(p for p in (builder.LOCAL/'pieces').glob('*.npz') if p.stem != 'link5_wall')
    before = {str(p): sha(p) for p in sorted(inputs)}
    a.output.mkdir(parents=True, exist_ok=False)
    a.work_output.mkdir(parents=True, exist_ok=False)
    (a.work_output/'build').mkdir()
    (a.work_output/'evidence').mkdir()
    (a.work_output/'inputs-before.json').write_text(json.dumps(before, indent=2)+'\n')
    builder.ROOT = a.work_output.resolve()
    with (a.work_output/'construction.log').open('x') as stream, redirect_stdout(stream):
        builder.main()
    construction = json.loads((a.work_output/'evidence/construction.json').read_text())
    source_check = {}
    for name, filename in (('link5', 'link5-dc497f5a.stl'), ('gripper_flange', 'gripper_flange-186697a7.stl')):
        import trimesh
        source = ROOT/'meshes/nero/source'/filename
        raw = np.asarray(trimesh.load(source, process=False).vertices)
        cached = np.load(builder.PREV/'source'/f'{name}-original.npz')['vertices']
        source_check[name] = {'manufacturer_STL': str(source.relative_to(ROOT)), 'sha256': sha(source),
            'cached_source_points_to_STL_vertex_max_m': float(cKDTree(raw).query(cached)[0].max()),
            'STL_vertices_to_cached_source_points_max_m': float(cKDTree(cached).query(raw)[0].max()),
            'scope': 'Point-coordinate identity check, not full continuous surface equivalence; source cache arose from the original MuJoCo import.'}
        if max(source_check[name][k] for k in ('cached_source_points_to_STL_vertex_max_m',
                                             'STL_vertices_to_cached_source_points_max_m')) > 2e-7:
            raise ValueError('Construction source frame does not match DH116 manufacturer source')
    xml = ET.parse(ROOT/'mjcf/nero_dh116.xml').getroot()
    removed = []
    for body in BODIES:
        b = xml.find('.//body[@name="'+body+'"]')
        for g in list(b.findall('geom')):
            if int(g.get('contype', '1')) or int(g.get('conaffinity', '1')):
                removed.append({'body': body, 'attributes': dict(g.attrib)})
                b.remove(g)
    names = {r['attributes']['mesh'] for r in removed}
    for e in list(xml.find('asset')):
        if e.tag == 'mesh' and e.get('name') in names:
            xml.find('asset').remove(e)
    for e in xml.iter():
        if e.get('file'):
            target = (ROOT/'mjcf'/e.get('file')).resolve()
            e.set('file', os.path.relpath(target, a.output))
    (a.output/'meshes').mkdir()
    mapping = []
    for row in construction['parts']:
        part, index = row['mesh'], row['index']
        meta = np.load(builder.PREV/'source'/f'{part}-original.npz')
        z = np.load(a.work_output/'build/parts'/f'{part}-{index:03d}.npz')
        # Recover the manufacturer body frame exactly once, not the Gripper
        # geom/assembly pose. DH116's existing flange body transform is retained.
        vertices = z['vertices'] @ meta['compiler_to_original_R'].T + meta['compiler_to_original_p']
        target = a.output/'meshes'/f'{part}_{index:03d}.obj'
        content = ['# Source-feature partition re-executed for the left research candidate.']
        content += ['v '+' '.join(format(float(x), '.17g') for x in point) for point in vertices]
        content += ['f '+' '.join(str(int(x)+1) for x in face) for face in z['faces']]
        target.write_text('\n'.join(content)+'\n')
        name = f'robot0_wrist_fine_{part}_{index:03d}'
        body = BODIES[0 if part == 'link5' else 1]
        old = next(r['attributes'] for r in removed if r['body'] == body)
        attrs = {k: v for k, v in old.items() if k not in ('name', 'mesh', 'pos', 'quat')}
        ET.SubElement(xml.find('asset'), 'mesh', name=name, file='meshes/'+target.name)
        ET.SubElement(xml.find('.//body[@name="'+body+'"]'), 'geom',
                      **attrs, name=name, mesh=name, pos='0 0 0', quat='1 0 0 0')
        mapping.append({'geom': name, 'body': body, 'file': 'meshes/'+target.name,
                        'sha256': sha(target), 'source_region': row})
    xml.set('model', 'nero_dh116_left_source_feature_wrist_research')
    ET.indent(xml)
    ET.ElementTree(xml).write(a.output/'model.xml', encoding='utf-8', xml_declaration=True)
    if any(sha(p) != h for p, h in before.items()):
        raise ValueError('A construction input changed')
    manifest = {'scope': 'One independent reconstruction of the existing source-feature method; no new algorithm or global-fidelity claim.',
        'base_left_sha256': sha(ROOT/'mjcf/nero_dh116.xml'), 'model_sha256': sha(a.output/'model.xml'),
        'counts': dict(Counter(r['body'] for r in mapping)), 'removed': removed, 'meshes': mapping,
        'source_coordinate_checks': source_check, 'construction_input_sha256': before,
        'method': 'Source contour/holes -> convex 2D regions -> exact source triangle clipping -> 3D convex hull; fixed shell sectors, separate flange floor/wall, 19 frozen local feature blocks.',
        'source_method_parameters': {'planar_simplify_mm': .10, 'guide_buffer_mm': .102,
                                     'link5_sector_degrees': 30, 'floor_classification_guard_m': 1e-7},
        'retained_risks': ['Original ~0.300mm root approximation', 'Original sampled ~2.17mm flange shoulder external fill',
                           'No new global coverage or nanometre-accuracy acceptance'],
        'right_delivered_OBJ_files_copied': 0, 'new_dynamics_steps': 0,
        'licenses': ['../../../licenses/SOURCES.md', '../../../licenses/flange-provenance.json']}
    (a.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({'candidate': str(a.output/'model.xml'), 'counts': manifest['counts'],
                      'right_delivered_OBJ_files_copied': 0, 'new_dynamics_steps': 0}))


if __name__ == '__main__':
    main()
