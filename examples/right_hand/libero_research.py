#!/usr/bin/env python3
"""Issue #5: one frozen research rollout, with an explicit independent policy.

Import and policy tests are stdlib-only. Native dependencies are loaded only by
run_once. The old executor and models are never modified. A pinned Python-source
adapter changes only the stopping policy and adds observation/evidence hooks.
"""
import argparse
import ast
from collections import deque
from contextlib import contextmanager
from fractions import Fraction
import gzip
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import types

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
POLICY_DIR = HERE/'evidence/d3_research_contact_policy'
POLICY_SHA = '48632dfa4bbe0630ef7d61b9e706240d7769ad6361d588e1d02025bde4bc6454'
TRACKER_SHA = '76abc56586d03579f1063d973a1cc63548daa85839e08f320248a8aa2e9999ec'
EXECUTOR_SHA = '67afdd0ebd03ed37b8ef874d5985e4e2b0fbe62912a8a201d09b4a1ffeb4b2e5'
BASELINE = '85a265ac96f0be141f82d10667b9ea8e8ce02e99'
AUTHORIZATION = 'https://github.com/LeoandDream/Nero_DH116_assets/issues/5'
POLICY_ATTRIBUTES = ('MODEL', 'MODEL_SHA', 'AUTHORIZATION', 'RISK_EVIDENCE', 'WRIST_BODIES',
                     'read', 'digest', 'compare_reset_snapshots', 'prepare_regions',
                     'classify_contact', 'extra_failures')


def read(path):
    with (gzip.open(path, 'rt') if str(path).endswith('.gz') else open(path)) as f:
        return json.load(f)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_checkout(previous, root):
    """Pin inputs without requiring old private generated run directories.

    The four old split/scene XML files are historical outputs, not resources
    for the new environment. Their identities remain provenance only. Reset,
    controller, compiled fields and all new split resources are checked again
    by the unchanged executor before its first trajectory step.
    """
    frozen = read(root/previous.OLD_RUN/'frozen-before-first-step.json')
    checked, prior_outputs = 0, {}
    for path, expected in frozen['input_sha256_before'].items():
        if path.startswith('runs/'):
            prior_outputs[path] = expected
            continue
        if digest(root/path) != expected:
            raise ValueError('Frozen source/resource changed: '+path)
        checked += 1
    for path, expected in previous.FIXED_SOURCES.items():
        if digest(root/path) != expected:
            raise ValueError('Frozen execution source changed: '+path)
    if digest(root/previous.MODEL) != previous.MODEL_SHA:
        raise ValueError('Frozen81/306 model changed')
    return {'frozen_source_resources_checked': checked, 'all_frozen_source_resources_exact': True,
            'prior_generated_outputs_provenance_only': prior_outputs,
            'private_runs_not_runtime_dependencies': True}


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def load_contact_policy():
    """Extract only reviewed pure definitions, never the offline import guard."""
    if digest(POLICY_DIR/'research-policy.json') != POLICY_SHA or digest(POLICY_DIR/'shadow.py') != TRACKER_SHA:
        raise ValueError('Frozen research rules or implementation changed')
    tree = ast.parse((POLICY_DIR/'shadow.py').read_text())
    names = {'finite_number', 'vector', 'validate_contact', 'in_scope', 'exposure', 'ContactTracker'}
    nodes = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in names]
    if {n.name for n in nodes} != names:
        raise ValueError('Frozen pure policy definition set differs')
    scope = {'math': math, 'Fraction': Fraction, 'deque': deque}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(POLICY_DIR/'shadow.py'), 'exec'), scope)
    return read(POLICY_DIR/'research-policy.json'), scope['ContactTracker']


class ResearchPolicy:
    def __init__(self, policy):
        frozen, tracker = load_contact_policy()
        if policy != frozen:
            raise ValueError('No research threshold overrides allowed')
        self.policy = policy
        self.tracker = tracker(policy)
        self.first_legacy = None
        self.first_research = None

    def evaluate(self, step, phase, contacts, independent_reasons):
        legacy = list(independent_reasons)
        research = list(independent_reasons)
        try:
            row = self.tracker.update(step, phase, contacts)
            for c in contacts:
                eligible = not c['filter_reasons'] and any(b.startswith(('robot0_', 'gripper0_', 'mount0_')) for b in c['body'])
                if eligible and -c['distance_m'] > .002:
                    if 'ROBOT_CONTACT_OVERLAP' not in legacy:
                        legacy.append('ROBOT_CONTACT_OVERLAP')
                    scope = self.policy['scope']
                    bodies = c['body']
                    intended = (phase in scope['phases'] and any(bodies[i] == scope['target_body']
                                and bodies[1-i] in scope['hand_bodies'] for i in (0, 1)))
                    if not intended and 'ROBOT_CONTACT_OVERLAP' not in research:
                        research.append('ROBOT_CONTACT_OVERLAP')
            research.extend(e['reason'] for e in row['stops'])
            result = {'legacy_failures': legacy, 'research_failures': list(dict.fromkeys(research)),
                      'warnings': row['warnings'], 'groups': row['groups'], 'contact_events': row['stops']}
        except (KeyError, TypeError, ValueError, IndexError, OverflowError) as error:
            result = {'legacy_failures': legacy, 'research_failures': list(dict.fromkeys(research+['LOG_INVALID_UNKNOWN'])),
                      'warnings': [], 'groups': [], 'contact_events': [], 'error': str(error)}
        if result['legacy_failures'] and self.first_legacy is None:
            self.first_legacy = {'step': step, 'reasons': result['legacy_failures']}
        if result['research_failures'] and self.first_research is None:
            self.first_research = {'step': step, 'reasons': result['research_failures']}
        return result


class Observations:
    def __init__(self, output):
        self.output = output
        self.stream = gzip.open(output/'observations.jsonl.gz', 'wt')
        self.frames = 0
        self.steps = []
        self.errors = []
        for camera in ('agentview', 'robot0_eye_in_hand'):
            (output/'frames'/camera).mkdir(parents=True)

    def capture(self, env, step, current, reason, step_return=None):
        import numpy as np
        from PIL import Image
        if self.steps and self.steps[-1] == step:
            return
        d = env.sim.data._data
        # Rendering and goal checks must not refresh or change solver caches.
        names = ('qpos', 'qvel', 'ctrl', 'xpos', 'xmat', 'qacc', 'qfrc_constraint',
                 'qfrc_actuator', 'qfrc_bias', 'qfrc_passive')
        before = {name: getattr(d, name).copy() for name in names}
        time_before = float(d.time)
        cameras = {}
        for name in ('agentview', 'robot0_eye_in_hand'):
            pixels = env.sim.render(camera_name=name, width=256, height=256)
            path = Path('frames')/name/f'{self.frames:06d}.jpg'
            Image.fromarray(np.asarray(pixels, dtype=np.uint8)[::-1]).save(self.output/path, quality=92, subsampling=0)
            cameras[name] = str(path)
        success = bool(env.check_success())
        if d.time != time_before or any(not np.array_equal(before[name], getattr(d, name), equal_nan=True) for name in names):
            raise RuntimeError('Observation changed native physics state/cache')
        row = {'step': step, 'macro': current['macro'], 'time_s': time_before,
               'phase': current['phase'], 'native_task_success': success, 'cameras': cameras,
               'reason': reason, 'frame_index': self.frames,
               'image_encoding': 'JPEG quality92, no chroma subsampling; RGB top-left origin, 256x256',
               'timing': 'render/goal use current final RK4-stage derived cache; qpos/qvel are post-step; no additional forward',
               'reward': None, 'done': None, 'info': None}
        if step_return is not None:
            _, reward, done, info = step_return
            row.update(reward=float(reward), done=bool(done), info=info)
        self.stream.write(json.dumps(row, allow_nan=False, default=lambda x: x.tolist() if hasattr(x, 'tolist') else str(x))+'\n')
        self.stream.flush()
        self.frames += 1; self.steps.append(step)

    def close(self):
        self.stream.close()

    def videos(self):
        (self.output/'videos').mkdir()
        results = {}
        for camera in ('agentview', 'robot0_eye_in_hand'):
            command = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-framerate', '20', '-start_number', '0',
                       '-i', str(self.output/'frames'/camera/'%06d.jpg'), '-c:v', 'libx264', '-crf', '18',
                       '-pix_fmt', 'yuv420p', str(self.output/'videos'/f'{camera}.mp4')]
            completed = subprocess.run(command, capture_output=True, text=True)
            results[camera] = {'exit_code': completed.returncode, 'stderr': completed.stderr,
                               'frames': self.frames, 'fps': 20, 'step_timestamps_authoritative': True}
        write(self.output/'videos'/'encoding.json', results)
        return results


def adapted_executor(legacy, policy, observations, output):
    """Reviewable source adapter; all control/reset/integration remains frozen."""
    source_path = HERE/'white_bowl_run.py'
    if digest(source_path) != EXECUTOR_SHA:
        raise ValueError('Original executor differs')
    source = source_path.read_text()
    function = next(n for n in ast.parse(source).body if isinstance(n, ast.FunctionDef) and n.name == 'execute')
    original = ast.get_source_segment(source, function)
    changed = original
    patches = [
        ('has_offscreen_renderer=False, control_freq=20, horizon=1000)',
         'has_offscreen_renderer=True, control_freq=20, horizon=1000)'),
        ('''            if overlap > criteria['eligible_internal_and_external_overlap_m_max']: reasons.append('ROBOT_CONTACT_OVERLAP')
            if wrist_conditional is not None:
                reasons.extend(wrist_policy.extra_failures(contacts, q.tolist(), external['protocol']))''',
         '''            if wrist_conditional is not None:
                reasons.extend(wrist_policy.extra_failures(contacts, q.tolist(), external['protocol']))
            policy_row = _research_policy.evaluate(counts['mj_step'], current['phase'], contacts, reasons)
            reasons = policy_row['research_failures']'''),
        ('''            trace.write(json.dumps(finite_json(state), allow_nan=False)+'\\n')''',
         '''            state.update(legacy_failures=policy_row['legacy_failures'], research=policy_row)
            trace.write(json.dumps(finite_json(state), allow_nan=False)+'\\n')'''),
        ('''            if reasons:
                raise StopTrial(','.join(reasons))''',
         '''            if reasons:
                _observations.capture(env, counts['mj_step'], current, 'EARLY_STOP')
                raise StopTrial(','.join(reasons))'''),
        ('''                env.step(np.array(row['action_13']))''',
         '''                native_step_return = env.step(np.array(row['action_13']))
                _observations.capture(env, counts['mj_step'], row, 'MACRO_COMPLETE', native_step_return)'''),
    ]
    for old, new in patches:
        if changed.count(old) != 1:
            raise ValueError('Executor hook not unique: '+repr(old[:80]))
        changed = changed.replace(old, new, 1)
    (output/'executor.executed.py').write_text(changed+'\n')
    import difflib
    (output/'executor.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True), changed.splitlines(True),
        fromfile='frozen_execute', tofile='research_execute')))
    scope = dict(vars(legacy))
    scope.update(_research_policy=policy, _observations=observations)
    exec(compile(changed, str(output/'executor.executed.py'), 'exec'), scope)
    return scope['execute']


@contextmanager
def selected_policy(module):
    previous = sys.modules.get('finger22_conditional')
    sys.modules['finger22_conditional'] = module
    try:
        yield
    finally:
        if previous is None: sys.modules.pop('finger22_conditional', None)
        else: sys.modules['finger22_conditional'] = previous


def run_once(output):
    """Exactly one caller-authorized attempt; errors never retry the rollout."""
    import os
    output = Path(output).resolve()
    if output.exists() or not output.is_relative_to(ROOT) or os.environ.get('PYTHONPATH') or os.environ.get('PYTHONHOME'):
        raise ValueError('Fresh output in activated checkout required')
    rules, _ = load_contact_policy()
    output.mkdir(parents=True)
    write(output/'attempt-claim.json', {'authorization': AUTHORIZATION, 'baseline': BASELINE,
        'attempt_budget': 1, 'automatic_retries': 0, 'right_frozen_research_rollout': True})
    sys.path.insert(0, str(HERE))
    import white_bowl_exploratory as previous
    import white_bowl_run as legacy
    input_audit = validate_checkout(previous, ROOT)
    registration = previous.protocol(ROOT)
    registration.update(authorization=AUTHORIZATION, baseline_commit=BASELINE,
        experiment='issue5_right_research_contact_v1_once', research_policy_sha256=POLICY_SHA,
        research_rule_version=rules['version'], prior_proposal_status_unchanged=rules['status'],
        online_use_authorized_by=AUTHORIZATION, formal_2mm_and_historical_FAIL='UNCHANGED_SEPARATE_CHANNEL',
        compiler_step_scope='Native compiler self-tests are setup of this authorized attempt, not included in recorded trajectory-step count; no claim of absolute native mj_step instrumentation.')
    write(output/'registered-protocol.json', registration)
    sources = {str(p.relative_to(ROOT)): digest(p) for p in
               (Path(__file__), POLICY_DIR/'research-policy.json', POLICY_DIR/'shadow.py', HERE/'white_bowl_run.py')}
    write(output/'input-audit.json', {'previous_inputs': input_audit, 'new_source_sha256': sources})
    observations = Observations(output)
    policy = ResearchPolicy(rules)
    # Construct an explicit policy dependency with the same original mechanics
    # and deferred provenance functions. The old modules/files remain untouched.
    module = types.ModuleType('issue5_research_dependency')
    for key in POLICY_ATTRIBUTES:
        setattr(module, key, getattr(previous, key))
    module.AUTHORIZATION = AUTHORIZATION
    def validate_external(root, path):
        # The historical wrapper also hashes private generated outputs. Use its
        # existing resource/physics validator directly; no old runs are read.
        result = previous.F.validate_external_preflight(root, path)
        return {**result, 'protocol': registration}
    module.validate_external_preflight = validate_external
    original_write = legacy.write
    def record(path, value):
        if Path(path).name == 'frozen-before-first-step.json':
            frozen = read(ROOT/previous.OLD_RUN/'frozen-before-first-step.json')
            for key in ('qpos', 'qvel', 'ctrl', 'controller', 'criteria', 'native_scene'):
                if value[key] != frozen[key]:
                    raise ValueError('Frozen Reset/controller/criteria differ: '+key)
            if read(output/'schedule.json') != read(ROOT/previous.OLD_RUN/'schedule.json'):
                raise ValueError('Frozen270actions differ')
            value.update(research_policy_sha256=POLICY_SHA, online_use_authorized_by=AUTHORIZATION,
                         new_source_sha256=sources, baseline_commit=BASELINE)
        if Path(path).name == 'result.json':
            value.update(experiment=registration['experiment'], authorization=AUTHORIZATION,
                A='HISTORICAL_STATIC_RESULTS_PRESERVED_NOT_NEW_ACCEPTANCE',
                baseline_commit=BASELINE, research_policy_sha256=POLICY_SHA, research_rule_version=rules['version'],
                online_use_authorized_by=AUTHORIZATION, research_rollout_status=value['execution'],
                legacy_first_failure=policy.first_legacy, research_first_failure=policy.first_research,
                formal_2mm_and_historical_FAIL='UNCHANGED', observation_frames=observations.frames,
                original_rule_metadata_unchanged=True,
                new_source_sha256_after={p: digest(ROOT/p) for p in sources},
                compiler_step_scope=registration['compiler_step_scope'])
            if value['new_source_sha256_after'] != sources:
                value.update(B='FAIL', input_integrity='CHANGED_NEW_SOURCE')
        original_write(path, value)
    legacy.write = record
    try:
        execute = adapted_executor(legacy, policy, observations, output)
        with selected_policy(module):
            code = execute(ROOT/previous.PLAN, None, output, frozen78=True,
                wrist_conditional=ROOT/'examples/right_hand/evidence/d3_wrist_conditional/preflight/summary.json', finger22=True)
    finally:
        legacy.write = original_write
        observations.close()
    # Video encoding is postprocessing existing images, never a retry.
    try:
        observations.videos()
    except Exception as error:
        write(output/'video-error.json', {'error': str(error), 'physics_retry': False})
    return code


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    raise SystemExit(run_once(parser.parse_args().output))
