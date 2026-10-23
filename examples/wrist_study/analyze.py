"""Offline review of the single recorded regression; no simulator imports."""
import argparse
import gzip
import json
from pathlib import Path


def read(path):
    with (gzip.open if str(path).endswith('.gz') else open)(path, 'rt') as stream:
        return json.load(stream)


def rows(path):
    with (gzip.open if str(path).endswith('.gz') else open)(path, 'rt') as stream:
        return [json.loads(line) for line in stream]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--comparison', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    states = rows(args.run/'bounded_grasp.jsonl')
    contacts = rows(args.run/'rk4-contacts.jsonl.gz')
    if len(states) != len(contacts):
        raise ValueError('State/cache length differs')
    settled = contacts[249]['bowl_post_qpos_z_m'] if len(contacts) >= 250 else None
    observations, consecutive, maximum = [], 0, 0
    first_positive = first_opposed = first_departure = None
    table_seen = False
    wrist_contacts = 0
    for step, (state, cache) in enumerate(zip(states, contacts), 1):
        center = cache['bowl_center_cached_m']
        thumb, index, environment = [], [], []
        for c in cache['contacts']:
            if any(b in ('robot0_link5', 'robot0_gripper_flange') for b in c['body']):
                wrist_contacts += 1
            if 'white_bowl_1_main' not in c['body'] or c['force_contact_frame'][0] <= 0:
                continue
            other = next(b for b in c['body'] if b != 'white_bowl_1_main')
            if not other.startswith('gripper0_'):
                environment.append(other)
            elif other.startswith('gripper0_finger1'):
                thumb.append(c)
            elif other.startswith('gripper0_finger2'):
                index.append(c)
        if (thumb or index) and first_positive is None:
            first_positive = step
        opposed = any(sum((t['position_m'][i]-center[i])*(f['position_m'][i]-center[i])
                          for i in (0, 1)) < 0 for t in thumb for f in index)
        if opposed and first_opposed is None:
            first_opposed = step
        if environment:
            table_seen = True
        if table_seen and not environment and first_departure is None:
            first_departure = step
        lift = None if settled is None else cache['bowl_post_qpos_z_m']-settled
        # Descriptive support/contact review; original runner's stopping and
        # success semantics are untouched. No new hard threshold is introduced.
        valid = state['phase'] == 'hold' and lift is not None and lift > 0 and opposed and not environment
        consecutive = consecutive+1 if valid else 0
        maximum = max(maximum, consecutive)
        observations.append({'step': step, 'phase': state['phase'], 'opposed_positive_contacts': opposed,
                             'environment_support': environment, 'lift_from_post_settled_m': lift,
                             'valid_hold_observed': valid})
    hold = [r for r in observations if r['phase'] == 'hold']
    case = read(args.run/'bounded_grasp-result.json')
    actual = read(args.run/'run-result.json')
    historical = read(args.run/'historical-comparison.json')
    static = read(args.comparison/'static.json.gz')
    benchmark = read(args.comparison/'benchmark.json.gz')
    counts = []
    for side in ('L', 'R'):
        for count in (15, 306):
            subset = [r for r in static if r['side'] == side and r['count'] == count]
            counts.append({'side': side, 'count': count, 'static_poses': len(subset),
                'wrist_contact_poses': sum(bool(r['contacts']) for r in subset),
                'max_static_MPR_depth_m': max([0.]+[-c['distance_m'] for r in subset for c in r['contacts']]),
                'max_abs_static_J6_wrist_torque_Nm': max(abs(r['J6_wrist_generalized_torque_Nm']) for r in subset)})
    result = {'scope': 'Fixed native L white-bowl regression; no full native cabinet success claim',
        'physics_steps': actual['physics_steps'], 'native_status': actual['status'], 'stop_reason': actual['stop_reason'],
        'functional_rollouts': actual['functional_rollouts'], 'source_settled_post_qpos_z_m': settled,
        'final_bowl_post_qpos_z_m': contacts[-1]['bowl_post_qpos_z_m'],
        'final_lift_from_settled_m': observations[-1]['lift_from_post_settled_m'],
        'final_lift_from_reset_m': contacts[-1]['bowl_post_qpos_z_m']-.97,
        'first_positive_thumb_or_index_step': first_positive, 'first_positive_opposed_step': first_opposed,
        'first_environment_support_disappeared_after_settling_step': first_departure,
        'planned_hold_steps': 750, 'actual_hold_steps': len(hold),
        'hold_positive_opposed_steps': sum(r['opposed_positive_contacts'] for r in hold),
        'hold_environment_support_steps': sum(bool(r['environment_support']) for r in hold),
        'hold_lift_range_m': [min(r['lift_from_post_settled_m'] for r in hold), max(r['lift_from_post_settled_m'] for r in hold)],
        'longest_continuous_valid_hold_s': maximum*.002,
        'complete_1_5s_hold_observed': len(hold)==750 and maximum==750,
        'actual_wrist_contact_events': wrist_contacts,
        'historical_trajectory_equal': historical['historical_trajectory_equal'],
        'raw_jsonl_bytes_equal': historical['raw_jsonl_bytes_equal'],
        'native_cabinet_task_result': actual['native_task_result'], 'strict_grasp_success_claim': False,
        'frozen_diagnostic_maxima': {k:case[k] for k in ('max_linear_mimic_error_rad','max_arm_target_error_rad',
            'max_other_robot_overlap_m','min_bowl_contact_distance_m','max_bowl_contact_normal_N')},
        'geometry_table': counts, 'step_timing_table': benchmark['summary'],
        'full_run_mean_mj_step_us': actual['mean_mj_step_us'],
        'interpretation': ['Static forces are recomputed at post-step poses; benchmark force/state perturbations come from real short open-loop playback.',
            'Benchmark playback is not the original closed-loop native protocol and is not a complete causal/controller result.',
            'The single full functional run uses the unchanged native L protocol and its original stopping rules.',
            'Geometry quality improves at old bad wrist poses; the already-successful fixed L task has no observed functional improvement.',
            'Retain R306 as its success baseline; preserve L15 default; keep the rebuilt L306 candidate as an optional reviewed research asset.']}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    with gzip.open(args.output.with_suffix('.steps.json.gz'), 'xt') as f:
        json.dump(observations, f)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
