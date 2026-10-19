#!/usr/bin/env python3
"""Static right-source versus candidate convex-union surface evidence.

Example: python examples/right_hand/proxy76_validate.py --candidate
examples/right_hand/candidates/proxy76/model.xml --output runs/d3_proxy76/metrics
No simulator is imported. Output must be a new directory; no input is modified.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time
import traceback
import xml.etree.ElementTree as ET

for _variable in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
                  'BLIS_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[_variable] = '1'

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
FORMAL_RIGHT = ROOT / 'mjcf/nero_dh116_right.xml'


def _module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BUILDER = _module('proxy76_delivered_mesh_reader', ROOT / 'tools/build_right_proxy76.py')
GEOMETRY = _module('proxy76_static_surface_metrics', HERE / 'proxy76_geometry.py')


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def _snapshot(paths):
    return {os.path.relpath(path, ROOT): _sha(path) for path in sorted(paths)}


def _binding(xml_path, xml, assets, geom):
    if geom.get('type', 'mesh') != 'mesh':
        raise ValueError('Expected delivered mesh geom: '+str(geom.attrib))
    if any(key in geom.attrib for key in ('euler', 'axisangle', 'xyaxes', 'zaxis')):
        raise ValueError('Only the delivered explicit quaternion poses are supported')
    asset = assets[geom.get('mesh')]
    if asset.get('refpos', '0 0 0') != '0 0 0' or asset.get('refquat', '1 0 0 0') != '1 0 0 0':
        raise ValueError('Nondefault mesh reference pose requires explicit handling')
    compiler = xml.find('compiler')
    meshdir = '' if compiler is None else compiler.get('meshdir', '')
    path = (xml_path.parent/meshdir/asset.get('file')).resolve()
    result = {'name': geom.get('name'), 'mesh': geom.get('mesh'), 'file': str(path),
              'scale': np.fromstring(asset.get('scale', '1 1 1'), sep=' ').tolist(),
              'pos': np.fromstring(geom.get('pos', '0 0 0'), sep=' ').tolist(),
              'quat': np.fromstring(geom.get('quat', '1 0 0 0'), sep=' ').tolist()}
    if any(len(result[key]) != size for key, size in (('scale', 3), ('pos', 3), ('quat', 4))):
        raise ValueError('Malformed mesh binding: '+str(result))
    return result


def mesh_triangles(binding):
    """Original delivered surface, with asset scale and geom link-local pose."""
    vertices, faces = BUILDER.read_mesh(binding['file'])
    rotation = BUILDER.quaternion_matrix(' '.join(map(str, binding['quat'])))
    vertices = (vertices*np.asarray(binding['scale']))@rotation.T+binding['pos']
    return vertices[faces]


def load_inputs(candidate):
    """Bind formal-right source visuals and the actual 76 candidate collision geoms."""
    candidate = Path(candidate).resolve()
    manifest_path = candidate.parent/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    if _sha(candidate) != manifest['candidate_model_sha256'] or _sha(FORMAL_RIGHT) != manifest['formal_right_sha256']:
        raise ValueError('Candidate or formal-right XML differs from migration manifest')
    original, model = ET.parse(FORMAL_RIGHT).getroot(), ET.parse(candidate).getroot()
    original_assets = {a.get('name'): a for a in original.findall('asset/mesh')}
    candidate_assets = {a.get('name'): a for a in model.findall('asset/mesh')}
    right_hands = BUILDER.hand_collisions(original)
    candidate_hands = BUILDER.hand_collisions(model)
    if len(right_hands) != 17 or set(candidate_hands) != set(right_hands):
        raise ValueError('Expected identical sets of 17 formal-right and candidate hand links')
    records = {r['candidate_geom']: r for r in manifest['geoms']}
    paths = {candidate, FORMAL_RIGHT, manifest_path, candidate.parent/'alignment.json',
             Path(__file__).resolve(), HERE/'proxy76_geometry.py',
             HERE/'evidence/d3_geometry/triangle/surface_geometry.py',
             ROOT/'tools/build_right_proxy76.py'}
    jobs = []
    for index, name in enumerate(sorted(right_hands)):
        original_body, original_collisions = right_hands[name]
        candidate_body, collisions = candidate_hands[name]
        source_visuals = [g for g in original_body.findall('geom') if g.get('group') == '1']
        candidate_visuals = [g for g in candidate_body.findall('geom') if g.get('group') == '1']
        if len(source_visuals) != 1 or len(candidate_visuals) != 1 or len(original_collisions) != 1:
            raise ValueError('Expected one original source visual per right link: '+name)
        source = _binding(FORMAL_RIGHT, original, original_assets, source_visuals[0])
        visual = _binding(candidate, model, candidate_assets, candidate_visuals[0])
        if any(source[key] != visual[key] for key in ('mesh', 'file', 'scale', 'pos', 'quat')):
            raise ValueError('Candidate source visual no longer matches formal right: '+name)
        if Path(source['file']).parent != ROOT/'meshes/dh116_right/source':
            raise ValueError('Source must be the formal-right original STL: '+name)
        paths.add(Path(source['file']))
        parts = []
        for geom in collisions:
            binding = _binding(candidate, model, candidate_assets, geom)
            record = records[binding['name']]
            if (record['link'] != name or record['candidate_mesh'] != binding['mesh']
                    or (ROOT/record['candidate_file']).resolve() != Path(binding['file'])
                    or _sha(binding['file']) != record['candidate_sha256']):
                raise ValueError('Candidate collision binding differs from manifest: '+binding['name'])
            paths.add(Path(binding['file']))
            parts.append(binding)
        jobs.append({'link': name, 'source': source, 'parts': parts, 'seed': 760+index})
    if sum(len(job['parts']) for job in jobs) != 76 or len(records) != 76:
        raise ValueError('Expected the actual 76 candidate collision geoms')
    checks = {'formal_right_source_links': len(jobs), 'candidate_collision_geoms': 76,
              'candidate_visuals_match_formal_right': len(jobs),
              'all_candidate_collision_bindings_match_manifest': True,
              'parser': 'build_right_proxy76.read_mesh and quaternion_matrix, applied asset scale then geom pose'}
    return jobs, paths, manifest, checks


def _regions(source):
    lo, hi = source.min(axis=(0, 1)), source.max(axis=(0, 1))
    span = hi-lo
    return {'proximal_z_low20pct': {'axis': 2, 'min': float(lo[2]), 'max': float(lo[2]+.2*span[2])},
            'distal_z_high25pct': {'axis': 2, 'min': float(lo[2]+.75*span[2]), 'max': float(hi[2])},
            'x_low25pct': {'axis': 0, 'min': float(lo[0]), 'max': float(lo[0]+.25*span[0])},
            'x_high25pct': {'axis': 0, 'min': float(lo[0]+.75*span[0]), 'max': float(hi[0])}}


def _run_link(job, output):
    started = time.monotonic()
    source = mesh_triangles(job['source'])
    parts = [{'name': part['name'], 'triangles': mesh_triangles(part)} for part in job['parts']]
    result = GEOMETRY.analyze_link(source, parts, samples_per_direction=4096, seed=job['seed'],
                                   regions=_regions(source), include_vertex_extrema=True)
    result.update({'link': job['link'], 'source_binding': job['source'],
                   'candidate_collision_bindings': job['parts'],
                   'source_aabb_link_frame_m': {'min': source.min(axis=(0, 1)).tolist(),
                                               'max': source.max(axis=(0, 1)).tolist()},
                   'region_interpretation': 'Coordinate bands of formal-right source AABB only; z bands are descriptive and X bands do not establish anatomical inward contact, grooves or gap preservation.',
                   'wall_seconds': time.monotonic()-started})
    _write(Path(output)/(job['link']+'.json'), result)
    return result


def validate(candidate, output):
    started = time.monotonic()
    output = Path(output).resolve()
    if output.exists():
        raise ValueError('Output must be a new directory: '+str(output))
    jobs, paths, manifest, bindings = load_inputs(candidate)
    before = _snapshot(paths)
    output.mkdir(parents=True)
    _write(output/'input_sha256_before.json', before)
    _write(output/'analytic_checks.json', GEOMETRY.focused_checks())
    _write(output/'binding_checks.json', bindings)
    print(json.dumps({'event': 'static_metrics_start', 'links': len(jobs), 'parts': 76,
                      'samples_per_direction': 4096, 'workers': 4}), flush=True)
    results, errors = {}, {}
    with ProcessPoolExecutor(max_workers=4) as executor:
        future_links = {executor.submit(_run_link, job, str(output)): job['link'] for job in jobs}
        for future in as_completed(future_links):
            name = future_links[future]
            try:
                result = future.result()
                results[name] = result
                print(json.dumps({'event': 'link_written', 'link': name,
                                  'completed': len(results), 'seconds': round(result['wall_seconds'], 3),
                                  'source_p99_mm': result['source_to_proxy_union']['p99_m']*1000,
                                  'proxy_p99_mm': result['proxy_union_to_source']['p99_m']*1000}), flush=True)
            except Exception:
                errors[name] = traceback.format_exc()
                _write(output/(name+'_error.json'), {'error': errors[name]})
                print(json.dumps({'event': 'link_failed', 'link': name, 'error': errors[name]}), flush=True)
    after = _snapshot(paths)
    _write(output/'input_sha256_after.json', after)
    directions = ('source_to_proxy_union', 'proxy_union_to_source',
                  'source_vertices_to_proxy_union', 'proxy_union_vertices_to_source')
    links = {name: {'result_file': name+'.json', 'collision_parts': len(result['candidate_collision_bindings']),
                    **{direction: result[direction] for direction in directions}}
             for name, result in sorted(results.items())}
    summary = {'schema': 'right-proxy76-static-surface-metrics-v1',
               'created_utc': datetime.now(timezone.utc).isoformat(),
               'candidate_model': os.path.relpath(Path(candidate).resolve(), ROOT),
               'candidate_model_sha256': manifest['candidate_model_sha256'],
               'formal_right_model': os.path.relpath(FORMAL_RIGHT, ROOT),
               'formal_right_model_sha256': manifest['formal_right_sha256'],
               'link_count_requested': 17, 'link_count_completed': len(results),
               'candidate_collision_geoms': 76, 'samples_per_direction_per_link': 4096,
               'seed': '760 + index in sorted link names', 'workers': 4,
               'thread_environment': {key: os.environ[key] for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS')},
               'input_files_hashed': len(before), 'inputs_unchanged': before == after,
               'metrics_complete': len(results) == 17 and not errors,
               'errors': errors, 'links': links, 'wall_seconds': time.monotonic()-started,
               'limits': ['Finite area samples and all-unique-vertex checks do not certify Hausdorff maxima.',
                          'Each actual asset is convexified separately; exposed-union boundary excludes overlap internal faces.',
                          'Reverse distances are unsigned; original source STL solid interior is UNKNOWN.',
                          'Coordinate regions do not establish anatomical fingertips, groove/gap or grasp acceptance.',
                          'No dynamics or collision-filter, physics, source, alignment or candidate edits.']}
    _write(output/'summary.json', summary)
    print(json.dumps({'event': 'static_metrics_finished', 'completed': len(results),
                      'inputs_unchanged': before == after, 'metrics_complete': summary['metrics_complete'],
                      'seconds': round(summary['wall_seconds'], 3)}), flush=True)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = validate(args.candidate, args.output)
    if not report['metrics_complete'] or not report['inputs_unchanged']:
        raise SystemExit(1)
