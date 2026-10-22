#!/usr/bin/env python3
"""One authorized close149 trial; reuse the frozen research executor and policy.

Only the scheduled close cutoff changes. All original files remain untouched.
Import, schedule preparation and adapter tests are pure standard-library work.
"""
import argparse
import ast
from collections import Counter
from copy import deepcopy
import difflib
import hashlib
import json
from pathlib import Path

import libero_research as R

ROOT = R.ROOT
BASELINE = 'c9de3e45e1f4dbccb7e8ebf659e5102aaec328ea'
AUTHORIZATION = 'https://github.com/LeoandDream/Nero_DH116_assets/issues/5#issuecomment-6295728385'
RESEARCH_SHA = '7f15b06b5cf1333362a580f6841f90ca8bdde19d3e70ce97d16c342f79ff0b73'
OLD_SCHEDULE = Path('examples/libero_eval/evidence/right_raw/schedule.json')
OLD_SCHEDULE_SHA = '4fcb72c06028ba405122c808d8ec90486f022199e1c01904d65b034c18be0b81'
CANDIDATE_SHA = 'ebf188b0bf49ad3b74a042c52b5fa4ad5df02dcb8f84caea565fcdaf9fdc7e32'
PHASE_COUNTS = {'settle_preshape': 10, 'approach_high': 80, 'descend': 70,
                'close': 30, 'lift': 40, 'hold': 30}


def canonical_sha(schedule):
    # Preserve source key order: this is the reviewed, unsorted JSON encoding.
    return hashlib.sha256(json.dumps(schedule, separators=(',', ':'),
                                     allow_nan=False).encode('utf-8')).hexdigest()


def planned_schedule(root=ROOT):
    path = Path(root)/OLD_SCHEDULE
    if R.digest(path) != OLD_SCHEDULE_SHA:
        raise ValueError('Original270 schedule bytes differ')
    old = R.read(path)
    schedule = deepcopy(old[:190]+old[200:])
    for i, row in enumerate(schedule):
        row['macro'] = i
        if i >= 190:
            row['action_13'][7:] = old[189]['action_13'][7:]
            row['target_qpos_18'][7:] = old[189]['target_qpos_18'][7:]
    if canonical_sha(schedule) != CANDIDATE_SHA:
        raise ValueError('Reviewed close149 candidate differs')
    return schedule


def summarize(schedule):
    if canonical_sha(schedule) != CANDIDATE_SHA:
        raise ValueError('Only the reviewed close149 schedule is allowed')
    counts = dict(Counter(row['phase'] for row in schedule))
    if counts != PHASE_COUNTS or [r['macro'] for r in schedule] != list(range(260)):
        raise ValueError('Candidate phase sequence differs')
    boundaries = {}
    for phase in PHASE_COUNTS:
        ids = [row['macro'] for row in schedule if row['phase'] == phase]
        boundaries[phase] = {'macro_first': ids[0], 'macro_last': ids[-1],
                             'physical_step_first': ids[0]*25+1,
                             'physical_step_last': (ids[-1]+1)*25}
    return {'commands': 260, 'phase_counts': counts, 'phase_boundaries': boundaries,
            'physics_steps_per_command': 25, 'expected_physics_steps': 6500,
            'physics_steps_max': 6500, 'physical_dt_s': .002,
            'duration_if_completed_s': 13.0}


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise ValueError('Source adapter anchor is not unique: '+old[:80])
    return source.replace(old, new, 1)


def adapted_executor(legacy, policy, observations, output):
    execute = R.adapted_executor(legacy, policy, observations, output)
    path = Path(output)/'executor.executed.py'
    original = path.read_text()
    changed = replace_once(original,
        'actions = build_schedule(candidate)\n            schedule_info = schedule_summary(actions)',
        'actions = deepcopy(_close149_schedule)\n            schedule_info = deepcopy(_close149_summary)')
    (Path(output)/'research-base-executor.diff').write_bytes((Path(output)/'executor.diff').read_bytes())
    (Path(output)/'close149-executor.diff').write_text(''.join(difflib.unified_diff(
        original.splitlines(True), changed.splitlines(True),
        fromfile='frozen_research_execute', tofile='close149_execute')))
    path.write_text(changed)
    scope = dict(execute.__globals__)
    schedule = planned_schedule()
    scope.update(deepcopy=deepcopy, _close149_schedule=schedule,
                 _close149_summary=summarize(schedule))
    exec(compile(changed, str(path), 'exec'), scope)
    return scope['execute']


def prepared_entry():
    """Pin and adapt provenance/schedule checks, leaving native physics intact."""
    research_path = Path(R.__file__)
    if R.digest(research_path) != RESEARCH_SHA:
        raise ValueError('Frozen research runner bytes differ')
    tree = ast.parse(research_path.read_text())
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'run_once')
    original = ast.get_source_segment(research_path.read_text(), function)
    changed = replace_once(original, 'registration = previous.protocol(ROOT)',
        'registration = previous.protocol(ROOT)\n    registration.update(_close149_registration)')
    changed = replace_once(changed, "experiment='issue5_right_research_contact_v1_once'",
        "experiment='issue5_close149_once'")
    changed = replace_once(changed,
        "(Path(__file__), POLICY_DIR/'research-policy.json',",
        "(Path(__file__), _research_source, POLICY_DIR/'research-policy.json',")
    changed = replace_once(changed,
        "if read(output/'schedule.json') != read(ROOT/previous.OLD_RUN/'schedule.json'):",
        "if read(output/'schedule.json') != _close149_schedule:")
    changed = replace_once(changed, "raise ValueError('Frozen270actions differ')",
        "raise ValueError('FrozenClose149actions differ')")
    schedule = planned_schedule()
    scope = dict(vars(R))
    scope.update(__file__=__file__, AUTHORIZATION=AUTHORIZATION, BASELINE=BASELINE,
        adapted_executor=adapted_executor, _close149_schedule=schedule,
        _research_source=research_path,
        _close149_registration={'schedule': summarize(schedule), 'maximum_physics_steps': 6500,
            'candidate_canonical_json_sha256': CANDIDATE_SHA,
            'close_end_source_macro': 149, 'first_action_difference_step': 4751,
            'first_lift_arm_target_change_step': 4776,
            'policy_history_reset_on_phase_transition': False,
            'wrist306_role': 'Frozen for action isolation only, not final asset endorsement'})
    exec(compile(changed, str(Path(__file__)), 'exec'), scope)
    return scope['run_once'], original, changed


def run_once(output):
    entry, original, changed = prepared_entry()
    code = entry(output)  # Preserve real backend exit code; no retry.
    # A refused/existing destination must never receive audit writes either.
    if Path(output).exists():
        (Path(output)/'close149-entry.executed.py').write_text(changed+'\n')
        (Path(output)/'close149-entry.diff').write_text(''.join(difflib.unified_diff(
            original.splitlines(True), changed.splitlines(True),
            fromfile='frozen_research_entry', tofile='close149_entry')))
    return code


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    raise SystemExit(run_once(parser.parse_args().output))
