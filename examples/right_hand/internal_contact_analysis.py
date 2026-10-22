#!/usr/bin/env python3
"""Offline actual-body contact audit. Preserve all original recorder outputs.

The legacy recorder also checks weld-root exclude signatures and can mislabel
contacts with a welded neighbor. The engine matches actual body signatures.
"""
import argparse
import contextlib
import importlib.util
import json
from pathlib import Path

import mujoco

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('internal_contact_core', HERE / 'internal_contact.py')
CORE = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(CORE)


def audit_contacts(model, rows):
    per_step, pairs, mismatch_contacts, affected = [], {}, 0, 0
    for row in rows:
        maximum, mismatches = 0., []
        for c in row['contacts']:
            ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in c['geom_names']]
            if min(ids) < 0 or CORE.REF.pair_names(model, *ids)['body_names'] != c['body_names']:
                raise ValueError('Recorded geom/body identity does not match the compiled model')
            actual_eligible = CORE.eligible(model, *ids)
            if not actual_eligible:
                raise ValueError('Recorded contact unexpectedly fails actual engine pair eligibility')
            is_robot = all(n.startswith(('robot0_', 'gripper0_', 'mount0_')) for n in c['body_names'])
            enabled_self = is_robot and actual_eligible
            if enabled_self != c['enabled_nonadjacent_robot_self']:
                mismatches.append({'geom_names': c['geom_names'], 'body_names': c['body_names'],
                    'legacy_filter_reasons': c['filter_reasons'], 'legacy_enabled_nonadjacent_robot_self': c['enabled_nonadjacent_robot_self'],
                    'actual_enabled_nonadjacent_robot_self': enabled_self})
                mismatch_contacts += 1
            overlap = max(0., -c['distance_m']) if enabled_self else 0.
            maximum = max(maximum, overlap)
            name = ' <-> '.join(sorted(c['body_names']))
            p = pairs.setdefault(name, {'steps': set(), 'first_step': row['step'], 'max_overlap_m': 0.,
                'max_overlap_step': None, 'max_normal_force_N': 0., 'max_force_step': None,
                'first_overlap_exceed_step': None, 'overlap_exceed_steps': 0, 'actual_enabled_nonadjacent': enabled_self})
            p['steps'].add(row['step'])
            if overlap > p['max_overlap_m']: p['max_overlap_m'], p['max_overlap_step'] = overlap, row['step']
            if c['normal_force_N'] > p['max_normal_force_N']: p['max_normal_force_N'], p['max_force_step'] = c['normal_force_N'], row['step']
            if overlap > CORE.REF.THRESHOLDS['enabled_nonadjacent_self_overlap_m']:
                p['overlap_exceed_steps'] += 1
                if p['first_overlap_exceed_step'] is None: p['first_overlap_exceed_step'] = row['step']
        affected += bool(mismatches)
        per_step.append({'step': row['step'], 'time_s': row['time_s'], 'enabled_nonadjacent_self_overlap_m': maximum,
                         'annotation_mismatches': mismatches})
    for p in pairs.values(): p['contact_steps'] = len(p.pop('steps'))
    bad = [r for r in per_step if r['enabled_nonadjacent_self_overlap_m'] > CORE.REF.THRESHOLDS['enabled_nonadjacent_self_overlap_m']]
    peak = max(per_step, key=lambda r:r['enabled_nonadjacent_self_overlap_m'])
    return {'per_step': per_step, 'contact_pairs': pairs, 'annotation_mismatch_contacts': mismatch_contacts,
            'annotation_mismatch_steps': affected, 'max_enabled_nonadjacent_self_overlap_m': peak['enabled_nonadjacent_self_overlap_m'],
            'max_overlap_step': peak['step'], 'first_overlap_exceed_step': bad[0]['step'] if bad else None,
            'overlap_exceed_steps': len(bad)}


@contextlib.contextmanager
def no_dynamics():
    names = ['mj_step', 'mj_step1', 'mj_step2', 'mj_forward', 'mj_forwardSkip', 'mj_inverse', 'mj_inverseSkip', 'mj_RungeKutta', 'mj_Euler', 'mj_implicit']
    old = {n:getattr(mujoco, n) for n in names}
    def forbidden(*args, **kwargs): raise RuntimeError('Offline analysis cannot integrate or solve dynamics')
    try:
        for n in names: setattr(mujoco, n, forbidden)
        yield
    finally:
        for n,v in old.items(): setattr(mujoco, n, v)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-output', required=True)
    parser.add_argument('--analysis-output', help='New analysis directory; keep physical output immutable')
    args = parser.parse_args(); run_output = (CORE.ROOT / args.run_output).resolve()
    if not run_output.is_relative_to(CORE.ROOT): raise ValueError('Use an actual run in this checkout')
    output = CORE.REF.reserve_output(CORE.ROOT, args.analysis_output) if args.analysis_output else run_output
    if (output / 'audited-comparison.json').exists(): raise FileExistsError('Offline audit already exists; do not overwrite')
    with no_dynamics():
        derived = (run_output / 'diagnostic.xml').read_bytes()
        if derived != CORE.derive_xml(CORE.REF.MODEL.read_bytes()): raise ValueError('Actual XML differs from the authorized two-exclusion policy')
        original, candidate = CORE.compile_models(derived); CORE.check_compiled(original,candidate)
        report = json.loads((run_output / 'comparison.json').read_text())
        baseline_rows = CORE.read_rows(CORE.EVIDENCE / 'hand-motion-contacts.jsonl.gz')
        candidate_rows = CORE.read_rows(run_output / 'filtered-contacts.jsonl')
        state_rows = CORE.read_rows(run_output / 'filtered-state.jsonl')
        for label, model, rows in [('baseline', original, baseline_rows), ('candidate', candidate, candidate_rows)]:
            audited = audit_contacts(model, rows)
            sequence = audited.pop('per_step')
            if label == 'candidate':
                with (output / 'audited-contacts-per-step.jsonl').open('x') as f:
                    for row in sequence: f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
                understated = [r['step'] for r,s in zip(sequence,state_rows) if r['enabled_nonadjacent_self_overlap_m'] != s['enabled_nonadjacent_self_overlap_m']]
                audited['legacy_state_overlap_understated_steps'] = understated
            report[label]['legacy_max_enabled_nonadjacent_self_overlap_m'] = report[label]['max_enabled_nonadjacent_self_overlap_m']
            report[label]['max_enabled_nonadjacent_self_overlap_m'] = audited['max_enabled_nonadjacent_self_overlap_m']
            report[label]['legacy_contact_pairs'] = report[label].pop('contact_pairs')
            report[label]['contact_pairs'] = audited['contact_pairs']
            report[label]['actual_body_contact_audit'] = audited
        report['measurement_correction'] = {
            'reason': 'Legacy REF.filter_reasons additionally tests weld signatures; actual MuJoCo excludes match raw body IDs. A welded neighbor is not excluded.',
            'affected_pair': ['gripper0_finger12_link', 'gripper0_finger43_link'],
            'raw_records_and_result_preserved': True, 'integration_and_control_unchanged': True,
            'physics_steps': 0, 'forward_or_constraint_solves': 0,
            'source_reference': 'https://github.com/google-deepmind/mujoco/blob/476599f8b6175ef5a3040c47ebb3cd99a83c5365/src/engine/engine_collision_driver.c#L333-L344'}
        report['analysis_inputs'] = {'entry_sha256': CORE.value_hash(Path(__file__).read_bytes()),
            'raw_sha256': {name: CORE.value_hash((run_output / name).read_bytes()) for name in ['diagnostic.xml', 'comparison.json', 'filtered-state.jsonl', 'filtered-contacts.jsonl', 'filtered-result.json']}}
        report['overall_conclusion'] = 'PARTIAL_CONTROL_IMPROVEMENT_CANDIDATE_STILL_FAIL'
        CORE.REF.write_json(output / 'audited-comparison.json', report)
    print(json.dumps({'offline_audit':str(output),'physics_steps':0,'annotation_mismatches':report['candidate']['actual_body_contact_audit']['annotation_mismatch_contacts']}))


if __name__ == '__main__':
    main()
