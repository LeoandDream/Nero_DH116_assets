#!/usr/bin/env python3
"""Audit the existing per-triangle arrays without rerunning any geometry."""
import gzip
import hashlib
import json
from pathlib import Path
import struct

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[4]
QUALITY = OUT.parents[1]/'d3_link5_flange_local/quality'


def main():
    summary = json.loads((QUALITY/'summary.json').read_text())
    rows, hashes = [], {}
    for body, link in summary['links'].items():
        source_path = ROOT/link['source_file']
        source_bytes = source_path.read_bytes()
        # Both frozen supplied source files are binary STL.
        source_count = struct.unpack_from('<I', source_bytes, 80)[0]
        assert len(source_bytes) == 84+50*source_count
        assert source_count == link['source_triangle_count']
        for representation in ('raw_float64', 'compiled_mesh_graph'):
            path = QUALITY/(body+'-'+representation+'.json.gz')
            hashes[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
            with gzip.open(path, 'rt') as stream:
                report = json.load(stream)
            for key in ('old_full_source_coverage', 'new_full_source_coverage',
                        'original_assigned_faces_new_local_coverage'):
                value = report[key]
                array = value['uncovered_area_by_source_triangle_m2']
                assert len(array) == value['source_triangle_count']
                assert all(v >= 0 for v in array)
                scalar = link[representation][key]
                assert all(value[k] == v for k, v in scalar.items())
                discrepancy = abs(sum(array)-value['uncovered_area_m2'])
                assert discrepancy <= 1e-15
                row = {'body': body, 'representation': representation, 'coverage_key': key,
                       'source_triangle_count': value['source_triangle_count'],
                       'per_triangle_array_length': len(array),
                       'positive_uncovered_area_entries': sum(v > 0 for v in array),
                       'all_per_triangle_uncovered_areas_exactly_zero': all(v == 0 for v in array),
                       'reported_uncovered_area_m2': value['uncovered_area_m2'],
                       'array_sum_minus_reported_area_absolute_m2': discrepancy,
                       'matches_existing_summary': True,
                       'reported_all_triangles_covered': value['all_source_triangles_covered_with_numerical_tolerance']}
                if representation == 'raw_float64':
                    assert row['all_per_triangle_uncovered_areas_exactly_zero']
                    assert value['all_source_triangles_covered_with_numerical_tolerance']
                rows.append(row)
    result = {'schema': 'stored-coverage-array-audit-v1', 'source_coverage_recomputed': False,
              'geometry_queries': 0, 'mujoco_api_calls': 0, 'model_constructor_calls': 0,
              'tolerance_m': summary['numeric_plane_tolerance_m'],
              'input_sha256': hashes, 'rows': rows,
              'interpretation': 'The stored raw coverage arrays cover every supplied source triangle with zero uncovered area, and match the compact summary. This checks completeness and internal consistency of existing evidence, not an independent rerun of continuous clipping.'}
    with (OUT/'coverage_array_audit.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'rows_verified': len(rows), 'raw_full_source_triangle_counts': [
        r['source_triangle_count'] for r in rows if r['representation'] == 'raw_float64'
        and r['coverage_key'] == 'new_full_source_coverage'], 'all_raw_arrays_exactly_zero': True,
        'geometry_queries': 0}))


if __name__ == '__main__':
    main()
