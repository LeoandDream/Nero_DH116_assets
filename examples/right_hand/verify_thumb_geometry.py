#!/usr/bin/env python3
"""Reproduce supplemental offline verification; emit the JSON report on stdout."""
import argparse
import ast
import copy
from decimal import Decimal, localcontext
import json
import sys
import time

import thumb_geometry as t


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-output', required=True)
    args = parser.parse_args()
    out = (t.ROOT / args.run_output).resolve() / 'geometry'
    records = [json.loads(line) for line in (out / 'all_steps.jsonl').read_text().splitlines()]
    bycase = {case: {r['step']: r for r in records if r['case'] == case} for case in ['lambda0', 'lambda1']}
    ref = t.HERE / 'evidence/d3_geometry/triangle/enrich_results.py'
    tree = ast.parse(ref.read_text())
    definitions = [n for n in tree.body if isinstance(n, ast.FunctionDef)
                   and n.name in ['dot', 'sub', 'cross', 'decimal_witness']]
    namespace = {'Decimal': Decimal, 'localcontext': localcontext}
    # Extract only the four existing fixed functions; never execute the old
    # module's top-level evidence reads, writes, or enrichment side effects.
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(ref), 'exec'), namespace)
    recheck = namespace['decimal_witness']
    checks, maximum_error = {}, 0.
    for case, rs in bycase.items():
        errors = [recheck(r['strict_transverse_witness'])['max_point_difference_from_float64_m']
                  for r in rs.values() if r['strict_transverse_witness']]
        checks[case] = {'strict_witnesses_rechecked': len(errors), 'all_strict_reconfirmed': True,
                        'max_float64_point_difference_m': max(errors, default=0.)}
        maximum_error = max(maximum_error, max(errors, default=0.))
    worst = {case: min((c['distance_m'], r['step']) for r in rs.values()
                      for c in r['recorded_RK4_stage_contacts'])[1] for case, rs in bycase.items()}
    first = {case: min(r['step'] for r in rs.values() if r['strict_transverse_witness'])
             for case, rs in bycase.items()}
    selected = {229, 231, 451, 500, *worst.values(), *first.values()}
    existing = {(r['case'], r['step']): r for r in json.loads((out / 'keyframes.json').read_text())}
    g = t._geometry_module()
    _, paths = t._model_inputs()
    tri = {k: g.read_stl(v) for k, v in paths.items()}
    bvhs = {k: g.BVH(v) for k, v in tri.items()}
    templates = {k: t.RigidBVH(v) for k, v in bvhs.items()}
    results, started = [], time.monotonic()
    with t._no_dynamics():
        model = t.mujoco.MjModel.from_xml_path(str(t.MODEL))
        data = t.mujoco.MjData(model)
        ids = [t.mujoco.mj_name2id(model, t.mujoco.mjtObj.mjOBJ_BODY, n) for n in t.BODY_NAMES]
        for case, rs in bycase.items():
            for step in sorted(selected):
                row, old = rs[step], existing.get((case, step))
                if old:
                    result = copy.deepcopy(old)
                    result['full_surface_query_reused_from'] = 'keyframes.json'
                else:
                    bvh, transforms, ra, pa = t._frame(model, data, ids, row, tri, templates)
                    result = g.surface_query(tri[t.LINKS[0]], bvh.tris, bvhs[t.LINKS[0]], bvh)
                    result.update({'case': case, 'step': step, 'full_qpos': row['full_qpos'],
                                   'alignment': 'recorded_post_step_full_qpos', **transforms})
                    result['closest_points_world_m'] = [(t.np.array(v) @ ra.T + pa).tolist() for v in result['closest_points']]
                    if result['strict_transverse_witness']:
                        result['strict_transverse_witness']['point_world_m'] = result['closest_points_world_m'][0]
                h = result['strict_transverse_witness']
                if h:
                    h['decimal_recheck'] = recheck(h)
                    result['original_face_vertices_body_local_m'] = [tri[name][face].tolist() for name, face in zip(t.LINKS, h['face_ids_zero_based'])]
                for key in ['recorded_RK4_stage_contacts', 'phase', 'time_s', 'hand_target_rad', 'active_tracking_error_rad']:
                    result[key] = row[key]
                result['full_steps_classification'] = row['classification']
                assert (h is not None) == (row['classification'] == 'strict_transverse_crossing')
                results.append(result)
                print(case, step, row['classification'], result['nearest_face_ids_zero_based'], file=sys.stderr)
    metadata = json.loads((out / 'metadata.json').read_text())
    for relative, expected in metadata['input_sha256'].items():
        assert t._sha(t.ROOT / relative) == expected, relative
    report = {'method': 'Full continuous surface_query at shared same-time keys, first source intersection and worst recorded hull overlap; independent existing 60-digit Decimal witness arithmetic on every strict all_steps witness.',
              'all_steps_sha256': t._sha(out / 'all_steps.jsonl'), 'decimal_reference_path': str(ref.relative_to(t.ROOT)),
              'decimal_reference_sha256': t._sha(ref), 'all_strict_witness_decimal_checks': checks,
              'maximum_float64_point_difference_m': maximum_error, 'selected_shared_steps': sorted(selected),
              'first_source_intersection_steps': first, 'worst_recorded_A_overlap_steps': worst,
              'keyframes': results, 'keyframe_count': len(results), 'inputs_unchanged': True,
              'physics_steps': 0, 'constraint_solves': 0, 'elapsed_s': time.monotonic()-started,
              'limits': metadata['timing_limit'] + ' ' + metadata['solid_containment_result']}
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
