#!/usr/bin/env python3
"""One authorized frozen white-bowl experiment; provenance is OFFLINE ONLY.

Reuse the historical executor byte-for-byte. Its local policy import is bound
explicitly to this module only while executing this independent entry. No old
module/file is modified. Model, integration, controller, recorder and physical
checks remain the old executor's; only the documented stop policy is selected.
"""
import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import sys

import finger22_conditional as F
import wrist_conditional as W

ROOT = Path(__file__).resolve().parents[2]
RUNTIME_PREFIX = Path('/data_robot/verification/nero_dh116_right')
EVIDENCE = Path('examples/right_hand/evidence/d3_finger22_exploratory')
OLD_RUN = Path('examples/right_hand/evidence/d3_finger22_dynamic/run')
BASELINE = '16b851f76cd04f535e96dacaf2e0d7b86af17f0a'
AUTHORIZATION = 'https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6290062555'
MODEL, MODEL_SHA = F.MODEL, F.MODEL_SHA
read, digest, PLAN, WRIST_BODIES, RISK_EVIDENCE = F.read, F.digest, F.PLAN, F.WRIST_BODIES, F.RISK_EVIDENCE
extra_failures = W.extra_failures  # Original wrist and planning-envelope stops.
FIXED_SOURCES = {
    'examples/right_hand/white_bowl_run.py': '67afdd0ebd03ed37b8ef874d5985e4e2b0fbe62912a8a201d09b4a1ffeb4b2e5',
    'examples/right_hand/finger22_conditional.py': 'd38d576d475e2f5fa07d74143aa6df24d3516b7d7b99a42ef10737fffa2be705',
    'examples/right_hand/finger22_contact_adapter.py': 'f71e47d1c1f754bf1d5683a83b0f6b0309b36ca7b7e9ee3cd85843091bd2b177',
    'examples/right_hand/evidence/d3_finger22_runner_offline/frozen-rules.json': 'f5e837f43332366af3ac964accd6107b4d58772cd185fbc16979fb1acf6f5800',
    'examples/right_hand/evidence/d3_finger22_dynamic/run/schedule.json': '4fcb72c06028ba405122c808d8ec90486f022199e1c01904d65b034c18be0b81',
}


def protocol(root):
    p = W.protocol(root)
    p.update(authorization=AUTHORIZATION, baseline_commit=BASELINE,
             model=str(MODEL), model_sha256=MODEL_SHA,
             hand_collision_count=81, wrist_collision_count=306,
             strict_region_online_stop=False,
             classification='Deferred offline only; no RISK/UNKNOWN classification stops this experiment.',
             experiment='frozen_finger22_white_bowl_exploratory_once',
             historical4731_stop='UNKNOWN_UNCHANGED',
             fidelity_status='FORMAL_NUMERIC_FAIL_AND_GEOMETRY_UNKNOWN_UNCHANGED')
    return p


def validate_inputs(root):
    """Read/hash only: pin the old execution inputs before any native Reset."""
    root = Path(root)
    for path, expected in FIXED_SOURCES.items():
        if digest(root/path) != expected:
            raise ValueError('Frozen source/rules changed: '+path)
    old = read(root/OLD_RUN/'frozen-before-first-step.json')
    checked = 0
    for path, expected in old['input_sha256_before'].items():
        # Only the established execution prefix holds dependencies and the
        # immutable original raw run. Checkout-owned assets always use root.
        prefix = RUNTIME_PREFIX if path.startswith(('.reference-deps/', 'runs/')) else root
        if digest(prefix/path) != expected:
            raise ValueError('Prior execution input changed: '+path)
        checked += 1
    if digest(root/MODEL) != MODEL_SHA:
        raise ValueError('Frozen81/306 model changed')
    return {'old_run_inputs_checked': checked, 'all_prior_resources_exact': True,
            'schedule_sha256': FIXED_SOURCES[str(OLD_RUN/'schedule.json')],
            'strict_rules_sha256': FIXED_SOURCES['examples/right_hand/evidence/d3_finger22_runner_offline/frozen-rules.json'],
            'model_sha256': MODEL_SHA, 'fixed_source_sha256': FIXED_SOURCES}


def validate_external_preflight(root, summary_path):
    previous = F.validate_external_preflight(root, summary_path)
    return {**previous, 'protocol': protocol(root), 'exploratory_input_audit': validate_inputs(root)}


def compare_reset_snapshots(old, new, removed, added):
    result = F.compare_reset_snapshots(old, new, removed, added)
    if new != read(ROOT/OLD_RUN/'robot-physical-preflight.json.gz')['actual_snapshot']:
        raise ValueError('Full native compiled Reset snapshot differs from frozen81/306 run')
    return {**result, 'same81_306_native_Reset_snapshot_exact': True}


def prepare_regions(root, model):
    # Interface marker only. No hull construction, region proof or strict
    # preparation is called; these cannot become hidden pre-run stop gates.
    return {'mode': 'DEFERRED_OFFLINE', 'online_classification_calls': 0,
            'strict_online_stop': False, 'authorization': AUTHORIZATION}


def classify_contact(contact, regions):
    if not any(n in F.NEW for n in contact['geom']) or contact['filter_reasons']:
        return None
    return {'status': 'DEFERRED_OFFLINE', 'online_stop_enabled': False}


@contextmanager
def policy_binding():
    """Explicit isolated dependency selection; restore on success or error."""
    previous = sys.modules.get('finger22_conditional')
    sys.modules['finger22_conditional'] = sys.modules[__name__]
    try:
        yield
    finally:
        if previous is None:
            sys.modules.pop('finger22_conditional', None)
        else:
            sys.modules['finger22_conditional'] = previous


def execute_once(output):
    import white_bowl_run as legacy
    input_audit = validate_inputs(ROOT)
    expected = protocol(ROOT)
    if read(ROOT/EVIDENCE/'registered-protocol.json') != expected:
        raise ValueError('Pre-registered exploratory protocol differs')
    extra_paths = [Path(__file__), ROOT/EVIDENCE/'registered-protocol.json']
    extra_before = {str(p.relative_to(ROOT)): digest(p) for p in extra_paths}
    original_write = legacy.write

    def record(path, value):
        if Path(path).name == 'frozen-before-first-step.json':
            old = read(ROOT/OLD_RUN/'frozen-before-first-step.json')
            for key in ('qpos', 'qvel', 'ctrl', 'controller', 'criteria', 'native_scene'):
                if value[key] != old[key]:
                    raise ValueError('Native Reset/controller/criteria differs: '+key)
            if read(output/'schedule.json') != read(ROOT/OLD_RUN/'schedule.json'):
                raise ValueError('270 saved actions differ')
            if value['additional_stop_protocol'] != expected:
                raise ValueError('Online stop selection differs from registration')
            value.update(exploratory_source_sha256=extra_before,
                         strict_region_classification='DEFERRED_OFFLINE_NOT_A_STOP',
                         same_prior81_306_Reset_controller_actions_exact=True)
        if Path(path).name == 'result.json':
            after = {str(p.relative_to(ROOT)): digest(p) for p in extra_paths}
            value.update(experiment=expected['experiment'], authorization=AUTHORIZATION,
                         baseline_commit=BASELINE, strict_region_online_stop=False,
                         exploratory_source_sha256_before=extra_before,
                         exploratory_source_sha256_after=after,
                         exploratory_sources_unchanged=extra_before == after,
                         historical4731_stop='UNKNOWN_UNCHANGED',
                         fidelity_status=expected['fidelity_status'])
            if extra_before != after:
                value.update(B='FAIL', input_integrity='FAIL_CHANGED_EXPLORATORY_INPUT')
        original_write(path, value)

    original_write(output/'input-audit.json', input_audit)
    legacy.write = record
    try:
        with policy_binding():
            return legacy.execute(ROOT/PLAN, None, output, frozen78=True,
                wrist_conditional=ROOT/'examples/right_hand/evidence/d3_wrist_conditional/preflight/summary.json',
                finger22=True)
    finally:
        legacy.write = original_write


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='NEW output inside fixed execution prefix; no retries')
    args = parser.parse_args()
    output = args.output.resolve()
    import os
    if output.exists() or not output.is_relative_to(ROOT) or os.environ.get('PYTHONPATH') or os.environ.get('PYTHONHOME'):
        parser.error('Fresh output inside activated isolated checkout required')
    if ROOT != RUNTIME_PREFIX or sys.prefix != str(ROOT/'.reference-env'):
        parser.error('Use the existing frozen isolated execution prefix')
    validate_inputs(ROOT)
    output.mkdir(parents=True)
    claim = {'authorization': AUTHORIZATION, 'attempt_budget': 1, 'automatic_retries': 0,
             'baseline': BASELINE, 'strict_region_online_stop': False}
    with (output/'attempt-claim.json').open('x') as f:
        json.dump(claim, f, indent=2)
    return execute_once(output)


if __name__ == '__main__':
    raise SystemExit(main())
