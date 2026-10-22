"""Pure standard-library adapters for recorded NERO + DH116 LIBERO episodes."""
import gzip
import hashlib
from itertools import zip_longest
import json
import math
from pathlib import Path
import shutil

SCHEMA = 'nero_dh116_libero_episode_v1'
TASK = 'KITCHEN_SCENE9_put_the_white_bowl_on_top_of_the_cabinet'
TASK_SHA = 'a595358c840e6dca31d2d914726bb49d58944252ae868796bd603345a444b627'
LIBERO_COMMIT = 'f78abd68ee283de9f9be3c8f7e2a9ad60246e95c'


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def read(path):
    path = Path(path)
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt', encoding='utf-8') as stream:
        return json.load(stream)


def rows(path):
    opener = gzip.open if Path(path).suffix == '.gz' else open
    with opener(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def find(source, name, required=True):
    for suffix in ('', '.gz'):
        path = source / (name + suffix)
        if path.is_file():
            return path
    if required:
        raise FileNotFoundError(source / name)
    return None


def _source_record(path, root):
    return {'path': str(path.relative_to(root)) if path.is_relative_to(root) else str(path),
            'sha256': digest(path), 'bytes': path.stat().st_size}


def _copy_media(source, output):
    """Keep an exported right episode usable when its source run is elsewhere."""
    for name in ('frames', 'videos'):
        if (source / name).is_dir():
            shutil.copytree(source / name, output / name)


def _cameras(observation):
    if observation is None:
        return None
    cameras = observation.get('cameras', observation.get('images'))
    if cameras is None:
        return None
    return cameras


def _validate_vector(value, length, label):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'{label} must have {length} entries')
    if not all(isinstance(x, (float, int)) and not isinstance(x, bool) and math.isfinite(x) for x in value):
        raise ValueError(f'{label} has a non-finite or nonnumeric value')


def _contacts(contacts):
    if contacts is None:
        return None
    result = []
    for contact in contacts:
        item = dict(contact)
        for field in ('geom', 'body', 'distance_m', 'force_contact_frame', 'position_world_m',
                      'normal_world', 'frame', 'cached_body_pose', 'filter_reasons'):
            item.setdefault(field, None)
        result.append(item)
    return result


def _finite(values):
    return isinstance(values, list) and bool(values) and all(
        isinstance(value, (int, float)) and math.isfinite(value) for value in values)


def export_episode(root, hand, mode, output, recorded_source=None):
    """Normalize a REAL saved episode without importing a simulation engine.

    Missing measurements remain null. No native success is inferred from grasp
    height, contact, a known terminal signal or the number of recorded steps.
    """
    root, output = Path(root).resolve(), Path(output).resolve()
    if hand not in ('L', 'R') or mode not in ('baseline', 'research'):
        raise ValueError('Expected hand L/R and mode baseline/research')
    if hand == 'L' and mode != 'baseline':
        raise ValueError('Left hand uses its delivered baseline, not the right research policy')
    if hand == 'R' and mode == 'research' and recorded_source is None:
        raise ValueError('Right research export requires an explicit recorded source or --execute')
    if output.exists():
        raise FileExistsError(output)
    default = ('examples/libero_reference/evidence' if hand == 'L'
               else 'examples/right_hand/evidence/d3_finger22_exploratory/run')
    source = Path(recorded_source).resolve() if recorded_source is not None else root / default
    left = hand == 'L'
    state_path = find(source, 'bounded_grasp.jsonl' if left else 'state.jsonl')
    contact_path = find(source, 'contact-details.jsonl' if left else 'contacts.jsonl')
    result_path = find(source, 'run-result.json' if left else 'result.json')
    native_result = read(result_path)
    policy_path = root / 'examples/right_hand/evidence/d3_research_contact_policy/research-policy.json'
    policy_sha = digest(policy_path) if hand == 'R' and mode == 'research' else None
    actual_policy = native_result.get('research_policy_sha256')
    if hand == 'R' and mode == 'research' and actual_policy != policy_sha:
        raise ValueError('Saved run does not attest the frozen research policy; cannot relabel a baseline run')
    if hand == 'R' and mode == 'baseline' and actual_policy:
        raise ValueError('A research run cannot be exported as the old baseline')
    planned = 5750 if left else native_result.get('planned_physics_steps', 6750)
    observations_path = find(source, 'observations.jsonl', required=False)
    observations = {}
    if observations_path is not None:
        for obs in rows(observations_path):
            step = obs.get('step', obs.get('physics_step'))
            if not isinstance(step, int) or step < 0 or step in observations:
                raise ValueError('Observation step missing or duplicated')
            observations[step] = obs
    output.mkdir(parents=True)
    if not left:
        _copy_media(source, output)
    sources = [state_path, contact_path, result_path]
    if observations_path:
        sources.append(observations_path)
    frames, count, phase_counts = [], 0, {}
    if 0 in observations:
        frames.append({**observations[0], 'physics_step': 0, 'kind': 'reset_observation'})
    action_digest = hashlib.sha256()
    numerical = {'finite_arrays': True, 'warnings_clear': True,
                 'warnings_recorded': True, 'time_monotonic': True,
                 'finite_flag_recorded': True, 'finite_flag_clear': True}
    previous_time = 0.
    with (output / 'steps.jsonl.gz').open('wb') as raw:
        with gzip.GzipFile(filename='', mode='wb', fileobj=raw, mtime=0) as compressed:
            for index, (state, contact) in enumerate(zip_longest(rows(state_path), rows(contact_path))):
                if state is None or contact is None:
                    raise ValueError('State and contact row counts differ')
                step = index + 1
                expected = contact.get('row', index) + 1 if left else contact.get('step')
                if expected != step or (not left and state.get('step') != step):
                    raise ValueError(f'State/contact step differs at {step}')
                action = state.get('robot_action_13') if left else state.get('action_13')
                control = state.get('internal_actuator_ctrl_13') if left else state.get('ctrl')
                _validate_vector(state.get('robot_state_18'), 18, 'state_18')
                _validate_vector(action, 13, 'action_13')
                _validate_vector(control, 13, 'ctrl_13')
                action_digest.update((json.dumps(action, separators=(',', ':'), allow_nan=False)+'\n').encode())
                observation = observations.get(step)
                success = None if observation is None else observation.get('native_task_success')
                if success is not None and not isinstance(success, bool):
                    raise ValueError('Native success must be bool or null')
                cameras = _cameras(observation)
                if cameras is not None:
                    frames.append({**observation, 'physics_step': step,
                        'macro': state.get('macro', index // 25), 'cameras': cameras})
                phase = state.get('phase'); phase_counts[phase] = phase_counts.get(phase, 0) + 1
                time = state.get('time', state.get('time_s'))
                numerical['finite_arrays'] &= all(_finite(value) for value in
                    (state.get('robot_state_18'), action, control, state.get('full_qpos'), state.get('full_qvel')))
                numerical['warnings_recorded'] &= isinstance(state.get('warnings'), dict)
                numerical['warnings_clear'] &= not bool(state.get('warnings'))
                numerical['time_monotonic'] &= isinstance(time, (int, float)) and math.isfinite(time) and time > previous_time
                previous_time = time if isinstance(time, (int, float)) else previous_time
                numerical['finite_flag_recorded'] &= isinstance(state.get('finite'), bool)
                numerical['finite_flag_clear'] &= state.get('finite') is not False
                normalized = {'schema': SCHEMA, 'hand': hand, 'physics_step': step,
                    'macro': state.get('macro', index // 25),
                    'time_s': state.get('time', state.get('time_s')), 'phase': phase,
                    'state_18': state['robot_state_18'], 'action_13': action, 'ctrl_13': control,
                    'full_qpos': state.get('full_qpos'), 'full_qvel': state.get('full_qvel'),
                    'actuator_force': contact.get('actuator_force', state.get('actuator_force')),
                    'contacts': _contacts(contact.get('contacts')),
                    'contact_cache_timing': contact.get('timing'),
                    'rk4_body_poses': contact.get('body_poses'),
                    'warnings': state.get('warnings'),
                    'linear_mimic_error_rad': state.get('linear_mimic_error_rad', state.get('mimic_rad')),
                    'arm_target_error_rad': state.get('arm_error_rad'),
                    'hand_target_error_rad': state.get('active_error_rad', state.get('max_hand_target_error_rad')),
                    'bowl_z_m': state.get('bowl_z_m', (state.get('bowl_position_m') or [None] * 3)[2]),
                    'environment_support': state.get('environment_support'),
                    'native_task_success': success, 'cameras': cameras,
                    'failures': state.get('failures'),
                    'legacy_failures': state.get('legacy_failures'),
                    'research': state.get('research')}
                compressed.write((json.dumps(normalized, separators=(',', ':'), allow_nan=False)+'\n').encode())
                count = step
    if count == 0 or count != native_result.get('physics_steps'):
        raise ValueError('Recorded steps disagree with source result or source is empty')
    if any(step > count for step in observations):
        raise ValueError('Camera observation references a missing physics step')
    model_path = root / ('mjcf/nero_dh116.xml' if left else 'examples/right_hand/candidates/finger22_local/model.xml')
    reset_path = find(source, 'full-reset-state.json' if left else 'frozen-before-first-step.json', required=False)
    reset = read(reset_path) if reset_path else None
    if reset_path:
        sources.append(reset_path)
    native_success = native_result.get('native_task_result')
    if native_success is not None and not isinstance(native_success, bool):
        raise ValueError('Terminal native success must be bool or null')
    completed = count == planned
    status = 'COMPLETED' if completed else 'TRUNCATED'
    if native_success is None:
        status = 'INCOMPLETE'
    metrics_path = source / 'metrics.json'
    metrics = read(metrics_path) if left and metrics_path.exists() else {
        k: native_result.get(k) for k in ('reset_bowl_z_m', 'own_settled_bowl_z_m', 'final_bowl_z_m',
            'lift_from_reset_m', 'lift_from_own_settled_m', 'longest_continuous_valid_hold_s', 'maxima')}
    if left and metrics_path.exists():
        sources.append(metrics_path)
    checks = {'recorded_finite_arrays': 'PASS' if numerical['finite_arrays'] else 'UNKNOWN',
        'monotonic_time': 'PASS' if numerical['time_monotonic'] else 'FAIL',
        'warnings': ('PASS' if numerical['warnings_clear'] else 'FAIL') if numerical['warnings_recorded'] else 'UNKNOWN',
        'source_finite_check_including_qacc': ('PASS' if numerical['finite_flag_clear'] else 'FAIL') if numerical['finite_flag_recorded'] else 'UNKNOWN'}
    bounds = reset.get('criteria') if reset and not left else None
    old_diagnostic = find(source, 'dynamic-results.json', required=False) if left else None
    if old_diagnostic is not None:
        bounds = read(old_diagnostic).get('temporary_stopping_bounds')
        sources.append(old_diagnostic)
    health = {'status': 'FAIL' if 'FAIL' in checks.values() else ('UNKNOWN' if 'UNKNOWN' in checks.values() else 'PASS'),
        'checks': checks, 'original_protection_thresholds': bounds,
        'scope': 'Numerical health of saved fields; control, limits, Mimic and contact protection remain separate source-run results.',
        'missing_evidence_is_not_pass': True}
    result = {'schema': SCHEMA, 'hand': hand, 'mode': mode, 'episode_status': status,
        'physics_steps': count, 'planned_physics_steps': planned,
        'recorded_commands': native_result.get('commands', native_result.get('completed_commands')),
        'native_task_success': native_success,
        'native_task_success_source': 'actual saved env.check_success() terminal result; not inferred',
        'schedule_completed': completed, 'new_physics_steps': 0,
        'source_execution': native_result.get('status', native_result.get('execution')),
        'stop_reason': native_result.get('stop_reason', native_result.get('first_failure')),
        'control_health': native_result.get('control_health'),
        'simulator_health': health,
        'limited_grasp_result': native_result.get('grasp_result', 'HISTORICAL_FIXED_GRASP_DIAGNOSTIC_ONLY'),
        'strict_grasp_success_claim': native_result.get('strict_grasp_success_claim', False),
        'phase_steps': phase_counts, 'metrics': metrics,
        'formal_acceptance': 'UNCHANGED; no model or formal threshold acceptance claimed',
        'scope': 'custom NERO + DH116 in native LIBERO; not an official leaderboard result'}
    camera_index = {'schema': SCHEMA, 'observations': frames, 'final_images': None,
                    'videos': [], 'missing_frames_are_not_reconstructed': True}
    if left:
        camera_root = root / 'examples/libero_eval/evidence/left_camera'
        image_paths = {'agentview': source / 'agentview_openpi_final.png',
                       'robot0_eye_in_hand': source / 'wrist_openpi_final.png'}
        if any(path.exists() for path in image_paths.values()):
            image_dir = output / 'cameras'; image_dir.mkdir()
            images = {}
            for camera, image in image_paths.items():
                images[camera] = None
                if image.is_file():
                    target = image_dir / image.name; shutil.copyfile(image, target)
                    images[camera] = {'path': str(target.relative_to(output)),
                        'source_path_record': str(image), 'sha256': digest(target), 'physics_step': count}
                    sources.append(image)
            camera_index['final_images'] = {'kind': 'actual image from this recorded source', 'images': images}
        elif recorded_source is None and (camera_root / 'provenance.json').exists():
            provenance = read(camera_root / 'provenance.json')
            with gzip.open(state_path, 'rb') as stream:
                trace_sha = hashlib.file_digest(stream, 'sha256').hexdigest()
            if trace_sha != provenance.get('source_trace_sha256'):
                raise ValueError('Historical camera provenance does not bind this default trace')
            for image in provenance['images'].values():
                if digest(root / image['repository_path']) != image['sha256']:
                    raise ValueError('Historical final camera image SHA differs')
            camera_index['final_images'] = provenance
        video = source / 'closure-side.mp4'
        if video.exists():
            camera_index['videos'].append({**_source_record(video, root),
                'kind': 'recorded_state_side_camera_render', 'camera': 'closure-side',
                'historical_render': True, 'new_rendering': False})
    else:
        for video in sorted((output / 'videos').glob('*.mp4')):
            camera_index['videos'].append({'path': str(video.relative_to(output)), 'sha256': digest(video)})
    manifest = {'schema': SCHEMA, 'episode_id': output.name, 'hand': hand, 'mode': mode,
        'record_kind': 'saved_real_dynamics_export', 'export_integrates_physics': False,
        'model': {'path': str(model_path.relative_to(root)), 'sha256': digest(model_path),
                  'hand_collision_proxies': 76 if left else 81, 'wrist_collision_proxies': 15 if left else 306},
        'task': {'suite': 'libero_90', 'name': TASK, 'bddl_sha256': TASK_SHA,
                 'goal': 'On white_bowl_1 wooden_two_layer_shelf_1_top_side',
                 'native_success_predicate_preserved': True},
        'initialization': {'kind': 'native_seeded_reset', 'seed': 20261007,
            'official_panda_init_state_supported': False, 'full_reset_source': _source_record(reset_path, root) if reset_path else None,
            'reset_qpos_sha256': hashlib.sha256(json.dumps(reset.get('qpos'), separators=(',', ':')).encode()).hexdigest() if reset else None},
        'contract': {'state': '7 arm + 11 hand joint positions [rad]',
                     'action': '7 absolute arm [rad] + 6 normalized absolute hand [-1,1]',
                     'ctrl': '7 arm torques + 6 hand actuator joint angles'},
        'environment': {'python': '3.11.16', 'mujoco': '3.2.3', 'robosuite': '1.4.1', 'libero_commit': LIBERO_COMMIT},
        'research_policy_sha256': policy_sha,
        'expanded_actions_13_sha256': action_digest.hexdigest(),
        'exporter_sha256': digest(Path(__file__)),
        'source_files': [_source_record(path, root) for path in sources],
        'steps_sha256': digest(output / 'steps.jsonl.gz'),
        'limitations': ['No official Panda init-state conversion or multi-seed benchmark implemented.',
            'Only saved native success measurements are reported; missing per-step signals remain null.',
            'Left historical contacts lack saved position/frame/RK4 body poses.',
            'Scripted white-bowl grasp lacks the native cabinet placement phase.']}
    write(output / 'manifest.json', manifest)
    write(output / 'result.json', result)
    write(output / 'cameras.json', camera_index)
    return result


def aggregate(episodes):
    """Report completed-episode and attempted-episode denominators separately."""
    entries = []
    seen = set()
    source_trajectories = set()
    for path in map(Path, episodes):
        path = path.resolve()
        if path in seen:
            raise ValueError('Duplicate episode directory')
        seen.add(path)
        manifest, result = read(path / 'manifest.json'), read(path / 'result.json')
        source_names = {'bounded_grasp.jsonl', 'bounded_grasp.jsonl.gz'} if manifest['hand'] == 'L' else {'state.jsonl', 'state.jsonl.gz'}
        records = {(str(Path(item['path'])), item['sha256']) for item in manifest.get('source_files', [])
                   if Path(item['path']).name in source_names}
        if len(records) != 1:
            raise ValueError('One unambiguous source trajectory path and SHA256 is required for aggregation')
        source_id = (manifest['hand'], *records.pop())
        if source_id in source_trajectories:
            raise ValueError('Duplicate source trajectory for the same hand; repeated exports are not independent episodes')
        source_trajectories.add(source_id)
        entries.append({'episode_id': manifest['episode_id'], 'hand': manifest['hand'],
            'mode': manifest['mode'], 'status': result['episode_status'],
            'native_task_success': result['native_task_success']})
    def counts(group):
        complete = [e for e in group if e['status'] == 'COMPLETED' and isinstance(e['native_task_success'], bool)]
        truncated = [e for e in group if e['status'] == 'TRUNCATED']
        incomplete = [e for e in group if e not in complete and e not in truncated]
        successes = sum(e['native_task_success'] for e in complete)
        attempted = complete + truncated
        return {'n_submitted': len(group), 'n_episodes': len(complete), 'n_success': successes,
            'success_rate': successes / len(complete) if complete else None,
            'n_truncated': len(truncated), 'n_incomplete': len(incomplete),
            'n_attempted': len(attempted),
            'attempted_success_rate': sum(e['native_task_success'] is True for e in attempted) / len(attempted) if attempted else None}
    return {'schema': SCHEMA, **counts(entries),
        'by_hand': {hand: counts([e for e in entries if e['hand'] == hand]) for hand in sorted({e['hand'] for e in entries})},
        'denominator': 'success_rate uses completed episodes with measured native task signal; attempted_success_rate includes truncated attempts',
        'independence': 'Deduplication uses hand plus source path and SHA256; different paths or matching hashes alone do not certify independent execution attempts.',
        'scope': 'custom NERO + DH116 native LIBERO episodes; not official benchmark or independent-trial confidence evidence',
        'episodes': entries}
