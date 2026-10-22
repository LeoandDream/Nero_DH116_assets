"""Read frozen 78-piece evidence and repeat only the original high approach.

This module uses the standard library only. It performs no model compilation,
state initialization, kinematics, collision query, or physics execution.
"""
from collections import Counter
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = Path('examples/right_hand/candidates/finger13_local/model.xml')
CANDIDATE_PATH = Path('examples/right_hand/evidence/d3_white_bowl_001/candidate.json')
PREFLIGHT_PATH = Path('examples/right_hand/evidence/d3_bowl_preflight')
MODEL_SHA256 = 'b27f4f86fbaeacd2bb925463f39f67181ef7db12963c4d9f7fc01797a7924d1b'
CANDIDATE_SHA256 = 'ba767b42f99d3562255909bde5c970d529ba254769029ec1cd7a98faf76af0fb'
PHYSICS_STEPS_PER_COMMAND = 25
PHYSICAL_DT_S = .002
PHASE_COUNTS = {'settle_preshape': 10, 'approach_high': 80, 'descend': 70,
                'close': 40, 'lift': 40, 'hold': 30}
FROZEN_CRITERIA = {
    'scope': 'NEW fixed right white-bowl criteria; not left/manufacturer/full-range certification',
    'lift_from_own_settled_baseline_m_min': .020,
    'continuous_hold_s_min': 1.0,
    'hold_requires': ['height >= settled+20mm',
                      'no bowl-table or other environmental support contacts',
                      'positive-force thumb and index contacts on opposing bowl sides'],
    'steady_active_error_rad_max': .02,
    'arm_tracking_error_rad_max': .1,
    'linear_mimic_error_rad_max': .02,
    'joint_limit_overshoot_rad_max': .01,
    'eligible_internal_and_external_overlap_m_max': .002,
    'warnings_allowed': 0,
    'nonfinite_states_allowed': 0,
    'native_task_result': 'record native cabinet-placement predicate separately',
    'strict_grasp_success_claim': False,
    'stop': 'First numerical/physical bound violation; no automatic retry or edits',
}


def frozen_criteria(candidate):
    """Check every original criterion and return an independent unchanged copy."""
    criteria = candidate['proposed_trial_criteria']
    if criteria != FROZEN_CRITERIA:
        raise ValueError('Original frozen white-bowl criteria differ')
    return deepcopy(criteria)


def validate_preflight(detail, summary, verification):
    """Bind the existing finite-sample result to this 78-piece model and path."""
    if (detail.get('schema') != 'frozen-white-bowl-78-internal-static-v1'
            or detail.get('candidate_parts') != 78
            or detail.get('candidate') != MODEL_PATH.as_posix()
            or detail.get('candidate_sha256') != MODEL_SHA256
            or detail.get('path_sha256') != CANDIDATE_SHA256):
        raise ValueError('Preflight is not bound to the frozen 78-piece model and path')
    verdict = 'SEPARATED_AT_ALL_FROZEN_FINITE_SAMPLES'
    if (detail.get('verdict') != verdict or summary.get('static_verdict') != verdict
            or detail.get('counts', {}).get('samples_complete') != 921
            or summary.get('exact_sample_count') != 921
            or summary.get('counts', {}).get('samples_complete') != 921
            or detail.get('first_negative') is not None
            or detail.get('first_unknown') is not None
            or summary.get('first_negative') is not None
            or summary.get('first_unknown') is not None
            or detail.get('inputs_unchanged') is not True):
        raise ValueError('Frozen 78-piece finite-sample preflight differs')
    if (verification.get('frozen_robot_sha256') != MODEL_SHA256
            or verification.get('frozen_path_sha256') != CANDIDATE_SHA256
            or verification.get('all_original_tracked_files_unchanged') is not True):
        raise ValueError('Preflight input verification differs')


def load_frozen_inputs(root=ROOT):
    """Return (original candidate, finite-only preflight provenance) after SHA checks."""
    root = Path(root)
    paths = [MODEL_PATH, CANDIDATE_PATH] + [PREFLIGHT_PATH / name for name in
            ('internal-detail.json', 'internal-summary.json', 'verification.json')]
    raw = {path: (root / path).read_bytes() for path in paths}
    hashes = {path.as_posix(): sha256(value).hexdigest() for path, value in raw.items()}
    if hashes[MODEL_PATH.as_posix()] != MODEL_SHA256:
        raise ValueError('Frozen 78-piece model SHA256 differs')
    if hashes[CANDIDATE_PATH.as_posix()] != CANDIDATE_SHA256:
        raise ValueError('Original frozen candidate SHA256 differs')
    candidate = json.loads(raw[CANDIDATE_PATH])
    detail, summary, verification = [json.loads(raw[PREFLIGHT_PATH / name]) for name in
                                   ('internal-detail.json', 'internal-summary.json', 'verification.json')]
    validate_preflight(detail, summary, verification)
    frozen_criteria(candidate)
    preflight = {
        'verdict': detail['verdict'], 'candidate_parts': detail['candidate_parts'],
        'finite_sample_count': summary['exact_sample_count'],
        'scope': detail['scope'],
        'dynamic_control_or_grasp_certified': False,
        'input_sha256': hashes,
    }
    return candidate, preflight


def build_schedule(candidate):
    """Return 270 fresh rows; only approach_high has repeat_index 0 and 1.

    macro is the executed schedule index. source_macro identifies the untouched
    original command. The numeric actions, targets, and source order are exact.
    """
    frozen_criteria(candidate)
    commands = candidate['commands']
    expected_phases = [phase for phase, count in PHASE_COUNTS.items()
                       for _ in range(count // 2 if phase == 'approach_high' else count)]
    if (len(commands) != 230
            or [row['macro'] for row in commands] != list(range(230))
            or [row['phase'] for row in commands] != expected_phases):
        raise ValueError('Original 230-command phase order differs')
    schedule = []
    for source in commands:
        if len(source['action_13']) != 13 or len(source['target_qpos_18']) != 18:
            raise ValueError('Original action/target dimensions differ')
        for repeat_index in range(2 if source['phase'] == 'approach_high' else 1):
            row = deepcopy(source)
            row.update(macro=len(schedule), source_macro=source['macro'], repeat_index=repeat_index)
            schedule.append(row)
    return schedule


def schedule_summary(schedule):
    """Describe the full schedule budget; early stopping uses a smaller count."""
    counts = dict(Counter(row['phase'] for row in schedule))
    if (counts != PHASE_COUNTS or len(schedule) != 270
            or [row['macro'] for row in schedule] != list(range(270))):
        raise ValueError('Frozen 270-command schedule differs')
    boundaries = {}
    for phase in PHASE_COUNTS:
        macros = [row['macro'] for row in schedule if row['phase'] == phase]
        boundaries[phase] = {
            'macro_first': macros[0], 'macro_last': macros[-1],
            'physical_step_first': macros[0] * PHYSICS_STEPS_PER_COMMAND + 1,
            'physical_step_last': (macros[-1] + 1) * PHYSICS_STEPS_PER_COMMAND,
        }
    maximum = len(schedule) * PHYSICS_STEPS_PER_COMMAND
    return {'commands': len(schedule), 'phase_counts': counts, 'phase_boundaries': boundaries,
            'physics_steps_per_command': PHYSICS_STEPS_PER_COMMAND,
            'expected_physics_steps': maximum, 'physics_steps_max': maximum,
            'physical_dt_s': PHYSICAL_DT_S,
            'duration_if_completed_s': maximum * PHYSICAL_DT_S}
