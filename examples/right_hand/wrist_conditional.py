"""Pure frozen-input and conservative stop policy for the one approved trial."""
import gzip
from hashlib import sha256
import json
from pathlib import Path

MODEL = Path('examples/right_hand/candidates/wrist_reuse/model.xml')
PLAN = Path('examples/right_hand/evidence/d3_white_bowl_001/candidate.json')
MODEL_SHA = '9604a4a6935a34496f93e223219f687b4df8b7aa83037dc208fc9f3499eb05c9'
AUTHORIZATION = 'https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6269815750'
WRIST_BODIES = {'robot0_link5', 'robot0_gripper_flange'}


def read(path):
    with gzip.open(path, 'rt') if Path(path).suffix == '.gz' else Path(path).open() as f:
        return json.load(f)


def digest(path):
    return sha256(Path(path).read_bytes()).hexdigest()


def protocol(root):
    from white_bowl_frozen78 import load_frozen_inputs, build_schedule, schedule_summary
    candidate, _ = load_frozen_inputs(root)
    actions = build_schedule(candidate)
    if actions != read(root/'examples/right_hand/evidence/d3_bowl78_slow_001/schedule.json.gz'):
        raise ValueError('Original 270-command schedule differs')
    positions = [row['qpos'] for row in candidate['samples']]
    # Additional conservative diagnostic bounds, registered before integration.
    # Arms: original .1 rad tracking bound; hands: original .02 rad steady/Mimic
    # bound. These do not relax ANY original joint/control/contact criterion.
    padding = [.1]*7+[.02]*11
    lower = [min(row[i] for row in positions)-padding[i] for i in range(18)]
    upper = [max(row[i] for row in positions)+padding[i] for i in range(18)]
    return {'authorization': AUTHORIZATION, 'model': MODEL.as_posix(), 'model_sha256': MODEL_SHA,
            'schedule': schedule_summary(actions), 'maximum_physics_steps': 6750,
            'original_criteria': candidate['proposed_trial_criteria'],
            'additional_stops': ['Any eligible actual new wrist geom contact with any body outside the two wrist bodies, regardless of positive force.',
                                 'Any actual robot qpos outside target/sample envelope padded by original .1rad arm / .02rad hand bounds.'],
            'qpos_envelope_lower_rad': lower, 'qpos_envelope_upper_rad': upper,
            'envelope_padding_rad': padding,
            'envelope_limit': 'Conservative finite-domain stop only, not continuous-path or all-domain clearance proof.',
            'numerical_appendix': 'FAIL_UNCHANGED', 'fill_exceptions_approved': False,
            'dynamic_attempt_budget': 1, 'automatic_retry_or_completion_run': False,
            'native_and_strict_semantics_unchanged': True}


def validate_external_preflight(root, summary_path):
    summary = read(summary_path)
    if summary['status'] != 'PASS_FINITE_SAMPLES_REFERENCE_SCENE_ONLY' or summary['poses_checked'] != 921:
        raise ValueError('External preflight is blocked or incomplete; no dynamics allowed')
    if summary['first_block'] is not None or summary['physics_steps'] or summary['wrist_wrist_queries']:
        raise ValueError('External preflight provenance differs')
    if digest(root/MODEL) != MODEL_SHA:
        raise ValueError('Frozen wrist model differs')
    for row in read(summary_path.parent/'inputs.json'):
        if digest(root/row['path']) != row['sha256']:
            raise ValueError('External preflight input changed: '+row['path'])
    audit = read(summary_path.parent/'input-audit.json')
    for row in audit['resources']['all_assets']:
        if digest(root/row['file']) != row['sha256']:
            raise ValueError('Candidate resource changed: '+row['file'])
    if audit['compiled_scene']['status'] != 'PASS' or audit['schedule']['original_270_rows_exact'] is not True:
        raise ValueError('Migration or original schedule audit differs')
    return {'summary': summary, 'summary_sha256': digest(summary_path),
            'input_audit_sha256': digest(summary_path.parent/'input-audit.json'),
            'protocol': protocol(root)}


def extra_failures(contacts, qpos, frozen):
    reasons = []
    for c in contacts:
        if c['filter_reasons']:
            continue
        if any((name or '').startswith('robot0_wrist_reuse_') for name in c['geom']) and not set(c['body']).issubset(WRIST_BODIES):
            reasons.append('UNEXPECTED_WRIST_EXTERNAL_CONTACT')
            break
    if any(q < lo or q > hi for q,lo,hi in zip(qpos, frozen['qpos_envelope_lower_rad'], frozen['qpos_envelope_upper_rad'])):
        reasons.append('OUTSIDE_REGISTERED_PATH_ENVELOPE')
    return reasons


def compare_reset_snapshots(old, new, removed, added):
    """Compare native post-Reset snapshots, never serialized pre-reset poses.

    Includes all environment body poses; only GEOM identities are remapped and
    the explicit 15->306 replacement is excluded. No live model is modified.
    """
    from collections import Counter

    def objects(rows, category):
        ordinal, result = Counter(), {}
        for row in rows:
            row = dict(row)
            if category == 'GEOM':
                row.pop('index', None)
            name = row['name']
            if name is None:
                body = row.get('bindings', {}).get('geom_bodyid', 'unnamed')
                name = f'@unnamed:{body}:{ordinal[body]}'
                ordinal[body] += 1
            result[name] = row
        return result

    differences, checked = [], 0
    for category in old['objects']:
        a, b = (objects(s['objects'][category],category) for s in (old,new))
        if category == 'GEOM':
            if set(a)-set(b) != set(removed) or set(b)-set(a) != set(added):
                differences.append('GEOM replacement sets')
            for name in removed:
                a.pop(name)
            for name in added:
                b.pop(name)
        if set(a) != set(b):
            differences.append(category+' object sets')
        for name in a:
            checked += 1
            if a[name] != b.get(name):
                differences.append(category+':'+str(name))
    for key in ('options','exclude_signature','mujoco_version'):
        if old[key] != new[key]:
            differences.append(key)
    for key, value in old['dimensions'].items():
        expected = value+len(added)-len(removed) if key=='ngeom' else value
        if new['dimensions'][key] != expected:
            differences.append(key)
    if differences:
        raise ValueError('Native post-Reset snapshot differs: '+str(differences[:20]))
    return {'passed': True, 'objects_checked': checked, 'differences': differences,
            'comparison': 'Original saved actual_snapshot vs new native actual_snapshot; all body poses included.',
            'removed_geom_count': len(removed), 'added_geom_count': len(added)}
