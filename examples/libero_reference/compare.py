"""Exact, read-only comparison and descriptive metrics for diagnostic JSONL traces.

Historical object fields define the comparison surface; additional instrumentation
is reported separately. Warning dictionaries are compared in full. No tolerance,
rounding, task-success threshold, dynamics, or controller is introduced here.
"""
import argparse
import gzip
import json
import math
from pathlib import Path


ACTIONS = {'robot_action_13'}
STATE = {'robot_state_18', 'full_qpos', 'full_qvel', 'internal_actuator_ctrl_13'}
CONTACTS = {'thumb_contact', 'opposed_contact', 'other_robot_overlap_m',
            'bowl_hand_contact_distances_m', 'max_contact_normal_N', 'contact_body_pairs'}


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _reject_constant(value):
    raise ValueError('Non-finite JSON number: ' + value)


def _read_trace(path):
    rows = []
    opener = gzip.open if Path(path).suffix == '.gz' else open
    with opener(path, 'rt', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            row = json.loads(line, parse_constant=_reject_constant)
            if not isinstance(row, dict):
                raise ValueError(f'{path}:{line_number}: expected a JSON object')
            rows.append(row)
    return rows


def _category(field):
    if field in ACTIONS:
        return 'actions'
    if field in STATE:
        return 'state'
    if field in CONTACTS:
        return 'contacts'
    return 'warnings' if field == 'warnings' else 'other'


def _walk(actual, historical, index=(), strict_keys=False):
    """Yield unequal leaves/structures plus all shared numeric differences."""
    if _number(actual) and _number(historical):
        yield {'kind': 'numeric', 'index': list(index), 'actual': actual,
               'historical': historical, 'abs_diff': abs(actual - historical)}
    elif isinstance(actual, dict) and isinstance(historical, dict):
        for key, value in historical.items():
            if key not in actual:
                yield {'kind': 'missing_field', 'index': list(index + (key,)),
                       'actual': None, 'historical': value, 'missing_from': 'actual'}
            else:
                yield from _walk(actual[key], value, index + (key,), strict_keys)
        if strict_keys:
            for key in actual.keys() - historical.keys():
                yield {'kind': 'extra_field', 'index': list(index + (key,)),
                       'actual': actual[key], 'historical': None, 'missing_from': 'historical'}
    elif isinstance(actual, list) and isinstance(historical, list):
        if len(actual) != len(historical):
            yield {'kind': 'shape', 'index': list(index), 'actual_length': len(actual),
                   'historical_length': len(historical)}
        for i, (a, h) in enumerate(zip(actual, historical)):
            yield from _walk(a, h, index + (i,), strict_keys)
    elif type(actual) is not type(historical) or actual != historical:
        yield {'kind': 'value', 'index': list(index), 'actual': actual, 'historical': historical}


def _metrics(rows, path, reset_z):
    trace_path = Path(path)
    if trace_path.suffix == '.gz':
        trace_path = trace_path.with_suffix('')
    result_path = trace_path.with_name(trace_path.stem + '-result.json')
    source = json.loads(result_path.read_text(encoding='utf-8'),
                        parse_constant=_reject_constant) if result_path.exists() else None
    dt = source.get('dt', .002) if isinstance(source, dict) else .002
    if not _number(dt) or not math.isfinite(dt) or dt <= 0:
        raise ValueError(f'{result_path}: invalid timestep')
    heights = [r['bowl_z_m'] for r in rows if _number(r.get('bowl_z_m'))]
    final_z = rows[-1].get('bowl_z_m') if rows else None
    peak_z = max(heights, default=None)
    reset_relative = {'reference_z_m': reset_z, 'reference_source': 'explicit reset_z argument',
                      'final_lift_m': final_z - reset_z if _number(final_z) else None,
                      'peak_lift_m': peak_z - reset_z if peak_z is not None else None}
    settled = None
    valid_settle = (len(rows) == 5750 and rows[249].get('phase') == 'settle'
                    and rows[250].get('phase') == 'approach_high'
                    and _number(rows[249].get('bowl_z_m')))
    if valid_settle:
        reference = rows[249]['bowl_z_m']
        post = [r['bowl_z_m'] for r in rows[249:] if _number(r.get('bowl_z_m'))]
        settled = {'reference_row': 249, 'reference_z_m': reference,
                   'final_lift_m': final_z - reference if _number(final_z) else None,
                   'peak_lift_m': max(post) - reference}
    hold = [r for r in rows if r.get('phase') == 'hold']
    hold_z = [r['bowl_z_m'] for r in hold if _number(r.get('bowl_z_m'))]
    warnings = {}
    for r in rows:
        for warning, count in r.get('warnings', {}).items():
            # MuJoCo warning counts are cumulative, so a sum would double count.
            warnings[warning] = max(warnings.get(warning, 0), count)
    def maximum(field):
        return max((r[field] for r in rows if _number(r.get(field))), default=None)
    distances = [distance for r in rows for distance in r.get('bowl_hand_contact_distances_m', [])]
    arm_errors = [abs(q - a) for r in rows
                  for q, a in zip(r.get('robot_state_18', [])[:7], r.get('robot_action_13', [])[:7])]
    return {
        'physics_steps': len(rows), 'final_time_s': rows[-1].get('time') if rows else None,
        'physics_dt_s': dt, 'dt_source': 'source result dt or archived helper default 0.002 s',
        'final_bowl_z_m': final_z, 'peak_bowl_z_m': peak_z,
        'reset_relative': reset_relative, 'settled_relative': settled,
        'settled_reference_available': valid_settle,
        'hold': {'physics_steps': len(hold), 'duration_s': len(hold) * dt,
                 'thumb_contact_steps': sum(bool(r.get('thumb_contact')) for r in hold),
                 'opposed_contact_steps': sum(bool(r.get('opposed_contact')) for r in hold),
                 'height_range_m': [min(hold_z), max(hold_z)] if hold_z else None,
                 'reset_relative_height_range_m': [min(hold_z)-reset_z, max(hold_z)-reset_z] if hold_z else None,
                 'settled_relative_height_range_m': [min(hold_z)-settled['reference_z_m'],
                                                     max(hold_z)-settled['reference_z_m']] if hold_z and settled else None},
        'contacts': {'bowl_contact_steps': sum(bool(r.get('bowl_hand_contact_distances_m')) for r in rows),
                     'thumb_contact_steps': sum(bool(r.get('thumb_contact')) for r in rows),
                     'opposed_contact_steps': sum(bool(r.get('opposed_contact')) for r in rows),
                     'min_bowl_contact_distance_m': min(distances, default=None),
                     'max_contact_normal_N': maximum('max_contact_normal_N'),
                     'max_other_robot_overlap_m': maximum('other_robot_overlap_m')},
        'residuals': {'max_linear_mimic_error_rad': maximum('linear_mimic_error_rad'),
                      'final_linear_mimic_error_rad': rows[-1].get('linear_mimic_error_rad') if rows else None,
                      'max_hand_target_error_rad': maximum('max_hand_target_error_rad'),
                      'final_hand_target_error_rad': rows[-1].get('max_hand_target_error_rad') if rows else None,
                      'max_arm_target_error_rad': max(arm_errors, default=None),
                      'max_abs_full_qvel': max((abs(x) for r in rows for x in r.get('full_qvel', [])), default=None)},
        'warnings': warnings,
        'contact_body_pairs': source.get('contact_body_pairs') if isinstance(source, dict) else None,
        'contact_body_pairs_source': str(result_path) if isinstance(source, dict) and 'contact_body_pairs' in source else 'unavailable in archived trace schema',
        'source_result': source, 'source_result_path': str(result_path) if source is not None else None,
    }


def compare_traces(actual_path, historical_path, reset_z=.97):
    """Return exact differences and descriptive metrics; never assign task success.

    Rows are aligned by zero-based physics row. The earliest difference uses
    historical JSON field order, then nested list order. Macro is row // 25,
    matching the archived 500 Hz physics / 20 Hz command helper. Extras within
    historical objects are instrumentation, except warnings, which compare fully.
    Adjacent <stem>-result.json files are optional and read without modification.
    """
    if not _number(reset_z) or not math.isfinite(reset_z):
        raise ValueError('reset_z must be finite')
    actual = _read_trace(actual_path)
    historical = _read_trace(historical_path)
    categories = {key: {'equal': True, 'first_difference': None} for key in
                  ('actions', 'state', 'contacts', 'warnings', 'other')}
    report = {'comparison': 'exact JSON numeric equality; no tolerance',
              'alignment': 'zero-based physics row; macro = row // 25 from archived helper',
              'historical_trajectory_equal': True, 'first_difference': None,
              'first_numeric_difference': None, 'first_structure_difference': None,
              'counts': {'actual': len(actual), 'historical': len(historical)},
              'max_abs_diff_by_field': {}, 'categories': categories}
    def record(event, category):
        if event['kind'] == 'numeric' and event['actual'] == event['historical']:
            return
        report['historical_trajectory_equal'] = False
        if report['first_difference'] is None:
            report['first_difference'] = event
        if event['kind'] == 'numeric' and report['first_numeric_difference'] is None:
            report['first_numeric_difference'] = event
        if event['kind'] in ('missing_field', 'extra_field', 'shape', 'row_count') and report['first_structure_difference'] is None:
            report['first_structure_difference'] = event
        categories[category]['equal'] = False
        if categories[category]['first_difference'] is None:
            categories[category]['first_difference'] = event
    for i, (a, h) in enumerate(zip(actual, historical)):
        context = {'row': i, 'time': a.get('time'), 'historical_time': h.get('time'),
                   'macro': i // 25, 'phase': a.get('phase'), 'historical_phase': h.get('phase')}
        for field, value in h.items():
            if field not in a:
                record(dict(context, kind='missing_field', field=field, index=[],
                            actual=None, historical=value, missing_from='actual'), _category(field))
                continue
            for difference in _walk(a[field], value, strict_keys=field == 'warnings'):
                event = dict(context, field=field, **difference)
                if difference['kind'] == 'numeric':
                    previous = report['max_abs_diff_by_field'].get(field)
                    if previous is None or event['abs_diff'] > previous['max_abs_diff']:
                        report['max_abs_diff_by_field'][field] = {
                            'max_abs_diff': event['abs_diff'], 'row': i,
                            'index': event['index'], 'actual': event['actual'],
                            'historical': event['historical']}
                record(event, _category(field))
        if 'warnings' in a and 'warnings' not in h:
            record(dict(context, kind='extra_field', field='warnings', index=[],
                        actual=a['warnings'], historical=None), 'warnings')
    if len(actual) != len(historical):
        i = min(len(actual), len(historical))
        remaining = actual[i] if i < len(actual) else historical[i]
        event = {'kind': 'row_count', 'row': i, 'field': None, 'index': [],
                 'time': remaining.get('time'), 'macro': i // 25, 'phase': remaining.get('phase'),
                 'actual_length': len(actual), 'historical_length': len(historical)}
        for category in categories:
            record(event, category)
    a_keys = set().union(*(r.keys() for r in actual)) if actual else set()
    h_keys = set().union(*(r.keys() for r in historical)) if historical else set()
    report['schema'] = {'historical_keys': sorted(h_keys), 'actual_keys': sorted(a_keys),
                        'intersection_keys': sorted(a_keys & h_keys),
                        'actual_only_keys': sorted(a_keys - h_keys),
                        'historical_only_keys': sorted(h_keys - a_keys)}
    report['metrics'] = {'actual': _metrics(actual, actual_path, reset_z),
                         'historical': _metrics(historical, historical_path, reset_z)}
    report['limitations'] = [
        'Exact equality concerns recorded historical trajectory fields, not task success.',
        'Extra instrumentation is excluded from historical equality except warning dictionaries.',
        'Contact body pairs come only from optional result sidecars; they are not recorded per physics row.',
        'Settled metrics require exactly 5750 rows and the archived settle/approach boundary at 249/250.',
        'Source result status, stop_reason, and success signals are preserved without reinterpretation.']
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--actual', required=True)
    parser.add_argument('--historical', '--golden', dest='historical', required=True,
                        help='Explicit comparison trace, JSONL or gzip JSONL.')
    parser.add_argument('--output', required=True)
    parser.add_argument('--reset-z', type=float, default=.97,
                        help='Explicit reset bowl z reference in metres (default: 0.97).')
    args = parser.parse_args()
    report = compare_traces(args.actual, args.historical, reset_z=args.reset_z)
    Path(args.output).write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    # Exit 0 means a report was produced; equality is an explicit field in that report.
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
