#!/usr/bin/env python3
"""Research proposal on existing logs only. Standard library; no simulation.

Never imports an executor, physics engine, model compiler or native bindings.
No shadow decision can mutate an original run or authorize its continuation.
"""
import argparse
from collections import deque
from fractions import Fraction
import gzip
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RUN = ROOT/'examples/right_hand/evidence/d3_finger22_exploratory/run'
POLICY_SHA = '48632dfa4bbe0630ef7d61b9e706240d7769ad6361d588e1d02025bde4bc6454'
TIMING = 'final RK4 internal-stage cache; qpos/height in state row are post-integration; no post-step forward'
BLOCKED_IMPORTS = []


def engine_import_guard(event, args):
    if event == 'import' and args[0].split('.')[0] in ('mujoco', 'robosuite', 'libero'):
        BLOCKED_IMPORTS.append(args[0])
        raise RuntimeError('Offline shadow evaluator forbids engine import')


sys.addaudithook(engine_import_guard)
assert not any(n.split('.')[0] in ('mujoco', 'robosuite', 'libero') for n in sys.modules)


def read(path):
    with (gzip.open(path, 'rt') if str(path).endswith('.gz') else open(path)) as f:
        return json.load(f)


def rows(path):
    with gzip.open(path, 'rt') as f:
        for line in f:
            yield json.loads(line)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def vector(value, n):
    if not isinstance(value, list) or len(value) != n or not all(map(finite_number, value)):
        raise ValueError('Missing, nonfinite or malformed numeric vector')
    return value


def validate_contact(c):
    for field in ('geom', 'body'):
        if not isinstance(c.get(field), list) or len(c[field]) != 2 or not all(isinstance(x, str) and x for x in c[field]):
            raise ValueError('Missing or malformed contact identity')
    if not finite_number(c.get('distance_m')) or not isinstance(c.get('filter_reasons'), list):
        raise ValueError('Missing distance or eligibility')
    for field, n in (('force_contact_frame', 6), ('position_world_m', 3), ('normal_world', 3), ('frame', 9)):
        vector(c.get(field), n)


def in_scope(policy, phase, contact):
    if contact['filter_reasons'] or phase not in policy['scope']['phases']:
        return False
    bodies = contact['body']; target = policy['scope']['target_body']
    return any(bodies[i] == target and bodies[1-i] in policy['scope']['hand_bodies'] for i in (0, 1))


def exposure(history, intervals, dt):
    """Exact comparison on binary64 endpoint forces, NOT physical impulse."""
    if len(history) < intervals+1:
        return None
    points = list(history)[-intervals-1:]
    twice = sum((Fraction(a['force'])+Fraction(b['force']) for a, b in zip(points, points[1:])), Fraction())
    mean = twice/(2*intervals)
    return mean, float(mean*intervals*Fraction(dt)), points[-1]['depth']-points[0]['depth']


class ContactTracker:
    def __init__(self, policy, long_multiple=None, force_factor=1.):
        self.policy = policy
        self.dt = policy['scales']['dt_s']
        tau = policy['scales']['mixed_solref_timeconst_s']
        self.short = math.ceil(policy['thresholds']['short_timeconst_multiple']*tau/self.dt)
        self.long = math.ceil((long_multiple or policy['thresholds']['long_timeconst_multiple'])*tau/self.dt)
        self.force = policy['scales']['force_reference_N']*force_factor
        self.thresholds = policy['thresholds']
        self.previous = 0
        self.memory = {}

    def update(self, step, phase, contacts):
        if type(step) is not int or step != self.previous+1:
            raise ValueError('Missing, repeated or out-of-order sample')
        if not isinstance(contacts, list):
            raise ValueError('Missing contact collection')
        for c in contacts:
            validate_contact(c)
        self.previous = step
        current = {}
        for c in contacts:
            if not in_scope(self.policy, phase, c):
                continue
            for level in ('geom', 'body'):
                key = (level, tuple(sorted(c[level])))
                unit = current.setdefault(key, {'depth': 0., 'forces': [], 'points': 0})
                unit['depth'] = max(unit['depth'], -c['distance_m'])
                unit['forces'].append(max(0., c['force_contact_frame'][0]))
                unit['points'] += 1
        warnings, stops, groups = [], [], []
        for key in current:
            if key not in self.memory:
                history = deque(maxlen=max(self.long, self.short)+1)
                # Previously processed records establish absence, not missing data.
                for i in range(max(1, step-max(self.long, self.short)), step):
                    history.append({'step': i, 'depth': 0., 'force': 0.})
                self.memory[key] = {'history': history, 'deep_samples': 0, 'contact_samples': 0}
        for key, memory in self.memory.items():
            unit = current.get(key, {'depth': 0., 'forces': [], 'points': 0})
            force = math.fsum(unit['forces']); depth = unit['depth']
            if not math.isfinite(force):
                raise ValueError('Nonfinite aggregated load')
            memory['deep_samples'] = memory['deep_samples']+1 if depth > self.thresholds['depth_warning_gt_m'] else 0
            memory['contact_samples'] = memory['contact_samples']+1 if unit['points'] else 0
            history = memory['history']
            history.append({'step': step, 'depth': depth, 'force': force})
            short = exposure(history, self.short, self.dt)
            long = exposure(history, self.long, self.dt)
            if not unit['points'] and not any(p['force'] or p['depth'] for p in history):
                continue
            row = {'level': key[0], 'key': list(key[1]), 'contact_points': unit['points'],
                   'depth_m': depth, 'normal_force_N': force, 'deep_samples': memory['deep_samples'],
                   'deep_observed_span_s': max(0, memory['deep_samples']-1)*self.dt,
                   'contact_samples': memory['contact_samples'],
                   'contact_observed_span_s': max(0, memory['contact_samples']-1)*self.dt,
                   'short_exposure_Ns': 0. if short is None else short[1],
                   'long_exposure_Ns': 0. if long is None else long[1],
                   'short_window_complete': short is not None, 'long_window_complete': long is not None,
                   'short_depth_change_m': None if short is None else short[2],
                   'short_force_change_N': None if short is None else force-list(history)[-self.short-1]['force']}
            def event(reason):
                return {'reason': reason, 'level': row['level'], 'key': row['key']}
            if depth > self.thresholds['depth_warning_gt_m']:
                warnings.append(event('MPR_DEPTH_WARNING'))
            if force >= self.force*self.thresholds['force_warning_multiple']:
                warnings.append(event('SAMPLED_LOAD_WARNING'))
            if force >= self.force*self.thresholds['force_immediate_multiple']:
                stops.append(event('SAMPLED_LOAD_EXTREME'))
            if memory['deep_samples'] >= self.long+1:
                stops.append(event('DEEP_CONTACT_PERSISTENT'))
            if (memory['deep_samples'] >= self.short+1 and short is not None
                    and short[0] >= Fraction(self.force) and short[2] > 0):
                stops.append(event('LOADED_DEPTH_GROWTH'))
            if long is not None and long[0] >= Fraction(self.force*self.thresholds['force_window_mean_multiple']):
                stops.append(event('SAMPLED_LOAD_EXPOSURE'))
            groups.append(row)
        return {'warnings': warnings, 'stops': stops, 'groups': groups}


def robot_contact(c):
    return not c['filter_reasons'] and any(b.startswith(('robot0_', 'gripper0_', 'mount0_')) for b in c['body'])


def original_checks(state, cache, frozen, snapshot, previous_time):
    """Recompute original stops. Qualification/warnings remain historical inputs."""
    q = vector(state['robot_state_18'], 18)
    action = vector(state['action_13'], 13); target = vector(state['target_qpos_18'], 18)
    arm = [abs(q[i]-action[i]) for i in range(7)]
    active = [abs(q[i]-target[i]) for i in (7, 8, 10, 12, 14, 16)]
    expected = [.004592+.8808*q[8]]+[.011072+.9206*q[i] for i in (10, 12, 14, 16)]
    mimic = max(abs(q[i]-v) for i, v in zip((9, 11, 13, 15, 17), expected))
    limits = [j['fields']['jnt_range'] for j in snapshot['objects']['JOINT'][:18]]
    overshoot = max([0.]+[v for x, (lo, hi) in zip(q, limits) for v in (lo-x, x-hi)])
    overlap = max([0.]+[-c['distance_m'] for c in cache['contacts'] if robot_contact(c)])
    finite = all(all(map(finite_number, state[k])) for k in ('full_qpos', 'full_qvel', 'full_qacc_cached', 'ctrl', 'actuator_force'))
    criteria = frozen['criteria']; protocol = frozen['additional_stop_protocol']
    independent, legacy = [], []
    if not finite: independent.append('NONFINITE_STATE')
    if state['warnings'] or not math.isclose(state['time_s']-previous_time, .002, rel_tol=0, abs_tol=1e-12):
        independent.append('NUMERICAL_WARNING_OR_TIME_RESET')
    if max(arm) > criteria['arm_tracking_error_rad_max']: independent.append('ARM_TRACKING')
    if state['phase'] == 'hold' and max(active) > criteria['steady_active_error_rad_max']: independent.append('ACTIVE_HOLD_TRACKING')
    if mimic > criteria['linear_mimic_error_rad_max']: independent.append('MIMIC')
    if overshoot > criteria['joint_limit_overshoot_rad_max']: independent.append('JOINT_LIMIT')
    legacy = independent.copy()
    if overlap > criteria['eligible_internal_and_external_overlap_m_max']: legacy.append('ROBOT_CONTACT_OVERLAP')
    extra = []
    for c in cache['contacts']:
        if not c['filter_reasons'] and any(n.startswith('robot0_wrist_reuse_') for n in c['geom']) and not set(c['body']).issubset({'robot0_link5', 'robot0_gripper_flange'}):
            extra.append('UNEXPECTED_WRIST_EXTERNAL_CONTACT'); break
    if any(x < lo or x > hi for x, lo, hi in zip(q, protocol['qpos_envelope_lower_rad'], protocol['qpos_envelope_upper_rad'])):
        extra.append('OUTSIDE_REGISTERED_PATH_ENVELOPE')
    independent.extend(extra); legacy.extend(extra)
    if not (arm == state['arm_error_rad'] and active == state['active_error_rad'] and mimic == state['mimic_rad']
            and overshoot == state['joint_overshoot_rad'] and overlap == state['robot_overlap_m']
            and finite == state['finite'] and legacy == state['failures']):
        raise ValueError('Original decisions or saved health fields do not reproduce')
    return legacy, independent, {'arm_rad': max(arm), 'active_rad': max(active), 'mimic_rad': mimic,
                                'limit_overshoot_rad': overshoot, 'robot_overlap_m': overlap,
                                'finite': finite, 'warnings': state['warnings']}


def event_stats(stats, events, step):
    for reason in set(e['reason'] if isinstance(e, dict) else e for e in events):
        item = stats.setdefault(reason, {'first_step': step, 'last_step': step, 'sample_steps': 0})
        item['last_step'] = step; item['sample_steps'] += 1


def main(output):
    if output.exists(): raise ValueError('Fresh analysis output required')
    policy = read(HERE/'research-policy.json')
    if sha(HERE/'research-policy.json') != POLICY_SHA: raise ValueError('Frozen candidate changed')
    for path, digest in policy['input_sha256'].items():
        if sha(ROOT/path) != digest: raise ValueError('Changed historical input: '+path)
    frozen = read(RUN/'frozen-before-first-step.json')
    snapshot = read(RUN/'robot-physical-preflight.json.gz')['actual_snapshot']
    schedule = read(RUN/'schedule.json')
    if any(frozen['warning_before_integration']): raise ValueError('Historical initial warning')
    geoms = {g['name']: g['bindings']['geom_bodyid'] for g in snapshot['objects']['GEOM']}
    variants = [('recommended', ContactTracker(policy))]
    for v in policy['sensitivity']['variants']:
        variants.append((v['name'], ContactTracker(policy, v.get('long_timeconst_multiple'), v.get('force_scale_factor', 1.))))
    summary = {name: {'first_stop': None, 'warning_steps': {}, 'stop_steps': {}} for name, _ in variants}
    legacy_first = None; legacy_stats = {}; pair_stats = {}; landmarks = {}; peak = {}; phase_counts = {}
    output.mkdir(parents=True); previous_time = 0.; count = 0
    with gzip.GzipFile(filename=str(output/'shadow.jsonl.gz'), mode='wb', mtime=0) as stream:
        for state, cache in itertools.zip_longest(rows(RUN/'state.jsonl.gz'), rows(RUN/'contacts.jsonl.gz')):
            if state is None or cache is None: raise ValueError('Mismatched log lengths')
            count += 1
            if state['step'] != cache['step'] or state['step'] != count or cache['timing'] != TIMING:
                raise ValueError('Missing/repeated step or uncertified cache timing')
            for c in cache['contacts']:
                validate_contact(c)
                if any(geoms.get(n) != b for n, b in zip(c['geom'], c['body'])):
                    raise ValueError('Contact geom/body binding mismatch')
            macro = state['macro']
            if type(macro) is not int or not 0 <= macro < len(schedule) == 270 or count > 6750:
                raise ValueError('Outside frozen trajectory index domain')
            command = schedule[macro]
            if (state['phase'] != command['phase'] or state['action_13'] != command['action_13']
                    or state['target_qpos_18'] != command['target_qpos_18']):
                raise ValueError('Outside frozen trajectory/action domain')
            legacy, independent, health = original_checks(state, cache, frozen, snapshot, previous_time)
            previous_time = state['time_s']; phase_counts[state['phase']] = phase_counts.get(state['phase'], 0)+1
            event_stats(legacy_stats, legacy, count)
            if legacy and legacy_first is None: legacy_first = {'step': count, 'reasons': legacy}
            common = independent.copy()
            if any(robot_contact(c) and not in_scope(policy, state['phase'], c) and -c['distance_m'] > .002 for c in cache['contacts']):
                common.append('ROBOT_CONTACT_OVERLAP')
            base = None
            for name, tracker in variants:
                row = tracker.update(count, state['phase'], cache['contacts'])
                reasons = common+[e['reason'] for e in row['stops']]
                item = summary[name]
                event_stats(item['warning_steps'], row['warnings'], count); event_stats(item['stop_steps'], reasons, count)
                if reasons and item['first_stop'] is None:
                    item['first_stop'] = {'step': count, 'reasons': sorted(set(reasons)), 'contact_events': row['stops']}
                if name == 'recommended': base = row
            reasons = common+[e['reason'] for e in base['stops']]
            for group in base['groups']:
                key = group['level']+':'+','.join(group['key'])
                item = pair_stats.setdefault(key, {'first_contact_step': None, 'positive_force_first_step': None,
                    'contact_sample_count': 0, 'max_depth_m': 0., 'max_force_N': 0., 'max_deep_span_s': 0.,
                    'max_contact_span_s': 0., 'max_long_exposure_Ns': 0.})
                if group['contact_points']:
                    if item['first_contact_step'] is None: item['first_contact_step'] = count
                    item['contact_sample_count'] += 1
                if group['normal_force_N'] > 0 and item['positive_force_first_step'] is None:
                    item['positive_force_first_step'] = count
                for dst, src in (('max_depth_m', 'depth_m'), ('max_force_N', 'normal_force_N'),
                                 ('max_deep_span_s', 'deep_observed_span_s'), ('max_contact_span_s', 'contact_observed_span_s'),
                                 ('max_long_exposure_Ns', 'long_exposure_Ns')):
                    if group[src] > item[dst]: item[dst] = group[src]; item[dst+'_step'] = count
            for k, v in health.items():
                if isinstance(v, float): peak[k] = max(peak.get(k, 0.), v)
            row = {'step': count, 'phase': state['phase'], 'time_s': state['time_s'], 'legacy_stops': legacy,
                   'research_stops': sorted(set(reasons)), 'research_contact_events': base['stops'],
                   'warnings': base['warnings'], 'health': health, 'groups': base['groups'],
                   'counterfactual_tail_after_research_stop': summary['recommended']['first_stop'] is not None and count > summary['recommended']['first_stop']['step']}
            stream.write((json.dumps(row, separators=(',', ':'), allow_nan=False)+'\n').encode())
            if count in (4518, 4731, 4742, 4768, 4769): landmarks[str(count)] = row
    if count != 4769 or legacy_first != {'step': 4769, 'reasons': ['ROBOT_CONTACT_OVERLAP']}:
        raise ValueError('Frozen historical prefix/legacy stop mismatch')
    if BLOCKED_IMPORTS: raise RuntimeError('Unexpected attempted engine import')
    report = {'status': 'RESEARCH_ONLY_NOT_APPROVED', 'policy_version': policy['version'], 'policy_sha256': POLICY_SHA,
              'actual_saved_steps_analyzed': count, 'all_original_checks_reproduced': True,
              'legacy_first_stop': legacy_first, 'legacy_stop_steps': legacy_stats, 'variants': summary,
              'phase_counts': phase_counts, 'health_peaks': peak, 'pair_statistics': pair_stats, 'landmarks': landmarks,
              'force_time_scope': policy['time_semantics'], 'actual_integrated_constraint_impulse': 'UNKNOWN_NOT_RECORDED',
              'next_close_lift_hold': 'UNKNOWN_NO_REAL_DATA_AFTER4769',
              'history_status': policy['history'], 'no_engine_modules_loaded': not any(n.split('.')[0] in ('mujoco', 'robosuite', 'libero') for n in sys.modules),
              'blocked_engine_import_attempts': BLOCKED_IMPORTS,
              'no_new_native_simulation_basis': 'This executed program imports stdlib only and reads JSON/gzip. No model constructor/native simulation API is reachable. Import audit is supplementary, not a claim to instrument native compiler internals.',
              'source_sha256': {p.name: sha(p) for p in (Path(__file__), HERE/'test_shadow.py')},
              'input_sha256': policy['input_sha256'], 'shadow_sha256': sha(output/'shadow.jsonl.gz')}
    write(output/'summary.json', report)
    print(json.dumps({'policy_sha256': POLICY_SHA, 'legacy_first_stop': legacy_first, 'variants': summary,
                      'steps': count, 'engine_imports': BLOCKED_IMPORTS}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        main(args.output.resolve())
    except (ValueError, KeyError, TypeError, IndexError, RuntimeError) as error:
        # Offline failure only; never grants an online runner permission.
        print(json.dumps({'status': 'LOG_INVALID_UNKNOWN', 'error': str(error)}), file=sys.stderr)
        raise SystemExit(1)
