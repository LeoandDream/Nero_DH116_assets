#!/usr/bin/env python3
"""One frozen formal-right native LIBERO trial, conditional on offline geometry.

There is no planner, alternate action, tuning, object injection or retry here.
The only simulator state initialization is the native constructor and one native
seeded reset. Forward/reset refreshes are counted separately from real mj_step.
"""
import argparse
import gzip
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import traceback
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import robosuite
import yaml

from white_bowl_model import register, configure_scene
from white_bowl_geometry import FORMAL_MODEL_SHA256, filter_reasons, sha

ROOT = Path(__file__).resolve().parents[2]
ACTIVE = np.array([7, 8, 10, 12, 14, 16])


def write(path, value):
    Path(path).write_text(json.dumps(finite_json(value), indent=2, allow_nan=False,
                                   default=lambda a: a.tolist() if hasattr(a, 'tolist') else str(a))+'\n')


def finite_json(value):
    """Retain a nonfinite failure row as nulls instead of losing its metadata."""
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: finite_json(v) for k,v in value.items()}
    if isinstance(value, list):
        return [finite_json(v) for v in value]
    return value


def support_and_opposition(contacts, bowl_xy):
    """Force-bearing environmental support and opposite thumb/index sides."""
    supports, thumbs, indexes = [], [], []
    for c in contacts:
        bodies = c['body']
        if 'white_bowl_1_main' not in bodies or c['force_contact_frame'][0] <= 0:
            continue
        other = bodies[1] if bodies[0] == 'white_bowl_1_main' else bodies[0]
        if not other.startswith('gripper0_'):
            supports.append(c)
        vector = np.array(c['position_world_m'][:2])-bowl_xy
        if other.startswith('gripper0_finger1'):
            thumbs.append(vector)
        if other.startswith('gripper0_finger2'):
            indexes.append(vector)
    opposed = any(float(t @ i) < 0 for t in thumbs for i in indexes)
    return bool(supports), opposed


def robot_audit(formal, actual):
    """Reuse delivered compiled-field inventory; compare named physical fields."""
    path = ROOT / 'examples/libero_reference/checks.py'
    spec = importlib.util.spec_from_file_location('right_native_physical_inventory', path)
    checks = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checks)
    a, b = checks.compiled_snapshot(formal), checks.compiled_snapshot(actual)
    differences, checked = [], 0
    for category in ('BODY', 'JOINT', 'GEOM', 'ACTUATOR', 'EQUALITY', 'SITE', 'SENSOR'):
        by_name = {v['name']: v for v in b['objects'][category]}
        for expected in a['objects'][category]:
            name = expected['name']
            if name == 'world':
                continue
            target = by_name.get(name)
            if target is None:
                differences.append({'object': name, 'reason': 'MISSING'})
                continue
            for section in ('fields', 'bindings', 'dofs'):
                for field, value in expected.get(section, {}).items():
                    checked += 1
                    other = target.get(section, {}).get(field)
                    if not np.array_equal(np.asarray(value), np.asarray(other)):
                        differences.append({'object': name, 'field': field, 'expected': value, 'actual': other})
    for field, value in a['options'].items():
        checked += 1
        if not np.array_equal(value, b['options'].get(field)):
            differences.append({'option': field, 'expected': value, 'actual': b['options'].get(field)})
    expected_excludes = set()
    for signature in formal.exclude_signature:
        first, second = int(signature) >> 16, int(signature) & 65535
        expected_excludes.add(tuple(sorted((formal.body(first).name, formal.body(second).name))))
    actual_excludes = set()
    for signature in actual.exclude_signature:
        first, second = int(signature) >> 16, int(signature) & 65535
        actual_excludes.add(tuple(sorted((actual.body(first).name, actual.body(second).name))))
    if expected_excludes != actual_excludes or actual.npair:
        differences.append({'reason': 'CONTACT_RULES_DIFFER', 'expected': sorted(expected_excludes),
                            'actual': sorted(actual_excludes), 'explicit_geom_pairs': actual.npair})
    # Splitting may renumber meshes. Compare their compiled payload by name as
    # well as geom bindings, rather than trusting names or XML references alone.
    for i in range(formal.nmesh):
        name = formal.mesh(i).name
        j = mujoco.mj_name2id(actual, mujoco.mjtObj.mjOBJ_MESH, name)
        if j < 0:
            differences.append({'mesh': name, 'reason': 'MISSING'})
            continue
        for kind in ('vert', 'face'):
            start_a, n_a = int(getattr(formal, 'mesh_'+kind+'adr')[i]), int(getattr(formal, 'mesh_'+kind+'num')[i])
            start_b, n_b = int(getattr(actual, 'mesh_'+kind+'adr')[j]), int(getattr(actual, 'mesh_'+kind+'num')[j])
            checked += 1
            if n_a != n_b or not np.array_equal(getattr(formal, 'mesh_'+kind)[start_a:start_a+n_a],
                                               getattr(actual, 'mesh_'+kind)[start_b:start_b+n_b]):
                differences.append({'mesh': name, 'reason': 'COMPILED_'+kind.upper()+'_DIFFERS'})
    return {'passed': not differences, 'fields_checked': checked, 'differences': differences,
            'formal_snapshot': a, 'actual_snapshot': b}


def execute(candidate_path, geometry_path, output, frozen78=False):
    candidate = json.loads(candidate_path.read_text())
    geometry = None if frozen78 else json.loads(geometry_path.read_text())
    result = {'candidate_id': candidate['candidate_id'], 'A': 'PASS_FINITE_SAMPLED_INTERNAL_GEOMETRY',
              'B': 'NOT_RUN', 'execution': 'SETUP', 'physics_steps': 0, 'completed_commands': 0,
              'native_task_result': None, 'strict_grasp_success_claim': False,
              'formal_model': 'mjcf/nero_dh116_right.xml', 'model_modified': False,
              'original_7000_step_result': 'FAIL (unchanged)', 'dynamic_attempts': 0,
              'grasp_result': 'UNKNOWN', 'control_health': 'UNKNOWN'}
    originals = {name: getattr(mujoco, name) for name in ('mj_step', 'mj_step1', 'mj_step2', 'mj_forward')}
    counts = {'mj_step': 0, 'mj_step1_nonintegrating': 0, 'mj_forward_nonintegrating': 0,
              'mj_step2': 0, 'constructor': 0, 'explicit_native_reset': 0}
    env, trace, contact_log = None, None, None
    try:
        if sys.version_info[:3] != (3, 11, 16) or mujoco.__version__ != '3.2.3' or robosuite.__version__ != '1.4.1' or np.__version__ != '1.26.4':
            raise ValueError('Pinned runtime versions differ')
        if not Path(sys.prefix).resolve().is_relative_to(ROOT):
            raise ValueError('Runtime must be inside this independent checkout')
        if frozen78:
            from white_bowl_frozen78 import (load_frozen_inputs, build_schedule, schedule_summary,
                                             MODEL_PATH as MODEL_REL, CANDIDATE_PATH as CANDIDATE_REL)
            frozen_candidate, preflight = load_frozen_inputs(ROOT)
            if candidate_path != ROOT / CANDIDATE_REL or candidate != frozen_candidate:
                raise ValueError('78 trial must use the exact approved candidate path and bytes')
            model_path = ROOT / MODEL_REL
            geometry_path = ROOT / 'examples/right_hand/evidence/d3_bowl_preflight/internal-detail.json'
            actions = build_schedule(candidate)
            schedule_info = schedule_summary(actions)
            planned_steps = schedule_info['physics_steps_max']
            result.update(robot_model=MODEL_REL, formal_model=None,
                          A='PASS_FINITE_SAMPLED_INTERNAL_GEOMETRY_ONLY',
                          experiment='approved_78_slow_approach_once',
                          authorization='https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6260545305',
                          planned_commands=len(actions), planned_physics_steps=planned_steps)
        else:
            model_path = ROOT / result['formal_model']
            if sha(model_path) != FORMAL_MODEL_SHA256:
                raise ValueError('Formal model changed')
            if geometry['A_GATE'] != 'NO_SOURCE_CROSSING_AT_SUPPLIED_FINITE_SAMPLES' or not geometry['all_supplied_samples_and_eligible_pairs_checked']:
                raise ValueError('Offline geometry does not support B')
        if not frozen78 and geometry['input_sha256_before'].get(str(candidate_path)) != sha(candidate_path):
            # Portable copies have a different absolute input path; compare the
            # uniquely named frozen candidate bytes without changing the report.
            matches = [v for k, v in geometry['input_sha256_before'].items() if Path(k).name == candidate_path.name]
            if matches != [sha(candidate_path)]:
                raise ValueError('Candidate differs from checked bytes')
        if not frozen78:
            actions = candidate['commands']
            if len(actions) != 230:
                raise ValueError('The single frozen candidate requires 230 commands')
            planned_steps = 5750
        libero = ROOT / '.reference-deps/LIBERO'
        commit = subprocess.check_output(['git', '-C', str(libero), 'rev-parse', 'HEAD'], text=True).strip()
        if commit != candidate['libero_commit'] or subprocess.check_output(['git', '-C', str(libero), 'diff', '--name-only', 'HEAD'], text=True).strip():
            raise ValueError('LIBERO source differs')
        task = libero / 'libero/libero/bddl_files/libero_90/KITCHEN_SCENE9_put_the_white_bowl_on_top_of_the_cabinet.bddl'
        bowl = libero / 'libero/libero/assets/stable_scanned_objects/white_bowl/white_bowl.xml'
        if sha(task) != candidate['task_sha256'] or sha(bowl) != candidate['bowl_xml_sha256']:
            raise ValueError('Native bowl scene differs')
        cfg = output / 'libero_config'
        cfg.mkdir()
        root_lib = libero / 'libero/libero'
        (cfg / 'config.yaml').write_text(yaml.safe_dump({
            'benchmark_root': str(root_lib), 'bddl_files': str(root_lib / 'bddl_files'),
            'init_states': str(root_lib / 'init_files'), 'assets': str(root_lib / 'assets'),
            'datasets': str(libero / 'datasets')}))
        os.environ['LIBERO_CONFIG_PATH'] = str(cfg)
        setup_allowed, integrating = True, False

        def guarded_forward(model, data):
            counts['mj_forward_nonintegrating'] += 1
            originals['mj_forward'](model, data)

        def reset_refresh(model, data):
            if not setup_allowed:
                raise RuntimeError('Unexpected mj_step1 after reset')
            before = [data.qpos.copy(), data.qvel.copy(), data.ctrl.copy(), float(data.time)]
            originals['mj_step1'](model, data)
            if not all(np.array_equal(a, b) for a, b in zip(before, [data.qpos, data.qvel, data.ctrl, float(data.time)])):
                raise RuntimeError('Native reset mj_step1 integrated')
            counts['mj_step1_nonintegrating'] += 1

        def no_integration(*_args, **_kwargs):
            raise RuntimeError('Integration forbidden before frozen B preflight')

        mujoco.mj_step = mujoco.mj_step2 = no_integration
        mujoco.mj_step1, mujoco.mj_forward = reset_refresh, guarded_forward
        adapter = register(ROOT, output, source=model_path)
        configure_scene(ROOT, source=model_path)
        from libero.libero.envs.env_wrapper import ControlEnv
        counts['constructor'] += 1
        env = ControlEnv(str(task), robots=['RightNeroURDF'], controller='JOINT_POSITION',
                         initialization_noise=None, hard_reset=False, use_camera_obs=False,
                         has_offscreen_renderer=False, control_freq=20, horizon=1000)
        env.seed(candidate['native_seed'])
        counts['explicit_native_reset'] += 1
        env.env.reset()  # One native reset. Never ControlEnv's retry loop.
        model, data = env.sim.model._model, env.sim.data._data
        formal = mujoco.MjModel.from_xml_path(str(model_path))
        audit = robot_audit(formal, model)
        write(output / 'robot-physical-preflight.json', audit)
        robot = env.robots[0]
        names = [formal.joint(i).name for i in range(formal.njnt)]
        jids = np.array([model.joint(name).id for name in names])
        qadr, vadr = model.jnt_qposadr[jids], model.jnt_dofadr[jids]
        if not audit['passed'] or not np.array_equal(data.qpos[qadr], adapter['initial']):
            raise ValueError('Formal right robot physics or native reset differs')
        if np.any(data.qvel) or np.any(data.ctrl) or data.time != 0:
            raise ValueError('Native reset must start with zero velocities/control/time')
        if frozen78:
            old_reset = json.loads((ROOT / 'examples/right_hand/evidence/d3_white_bowl_001/frozen-before-first-step.json').read_text())
            if not all(np.array_equal(getattr(data, name), old_reset[name]) for name in ('qpos', 'qvel', 'ctrl')):
                raise ValueError('Native full reset differs from frozen original bowl reset')
            hand = [i for i in range(model.ngeom) if model.body(int(model.geom_bodyid[i])).name.startswith('gripper0_')
                    and (model.geom_contype[i] or model.geom_conaffinity[i])]
            if len(hand) != 78 or formal.nexclude != 1 or model.nexclude != 1:
                raise ValueError('Expected exactly 78 enabled hand proxies and only original base-link1 exclusion')
            write(output / 'schedule.json', actions)
            write(output / 'geometry-provenance.json', preflight)
        backend = robot.controller.backend
        if type(backend).__name__ != 'JointPositionController' or not np.array_equal(backend.kp, np.full(7, 50.)) or not np.array_equal(backend.kd, np.full(7, np.sqrt(50.)*2)) or backend.goal_qpos is not None:
            raise ValueError('Stock absolute JPC initialization differs')
        low, high = env.env.action_spec
        if not (np.array_equal(low[:7], adapter['lower']) and np.array_equal(high[:7], adapter['upper'])
                and np.array_equal(low[7:], [-1]*6) and np.array_equal(high[7:], [1]*6)):
            raise ValueError('Action contract differs')
        xml = env.env.model.get_xml()
        (output / 'model.xml').write_text(xml)
        resources = [Path(n.get('file')).resolve() for n in ET.fromstring(xml).iter() if n.get('file')]
        if any(not p.is_relative_to(ROOT) for p in resources):
            raise ValueError('Native scene depends on a resource outside the independent checkout')
        files = set(resources) | {candidate_path, geometry_path, model_path, task, bowl, output / 'model.xml'}
        files.update(adapter['paths'].values())
        files.update((ROOT / 'examples/right_hand'/name) for name in (
            'white_bowl_run.py', 'white_bowl_model.py', 'white_bowl_geometry.py', 'white_bowl_candidate.py'))
        files.update((ROOT / 'examples/libero_reference'/name) for name in ('model.py', 'checks.py'))
        if frozen78:
            files.add(ROOT / 'examples/right_hand/white_bowl_frozen78.py')
            files.update(ROOT / 'examples/right_hand/evidence/d3_bowl_preflight' / name
                         for name in ('internal-summary.json', 'verification.json'))
            files.add(ROOT / 'examples/right_hand/evidence/d3_white_bowl_001/frozen-before-first-step.json')
        before_sha = {str(p.relative_to(ROOT)): sha(p) for p in sorted(files)}
        criteria = candidate['proposed_trial_criteria']
        frozen = {'candidate_sha256': sha(candidate_path), 'geometry_sha256': sha(geometry_path),
                  'model_xml_sha256': sha(output / 'model.xml'), 'criteria': criteria,
                  'measurement': {'settled_baseline': 'post-step bowl free-joint Z at step250, final settle_preshape step',
                                  'hold_window': 'last30 macros; active steady error evaluated at every step of this window',
                                  'no_support': 'zero positive-normal-force bowl vs non-hand contacts (table, other environment, arm or mount)',
                                  'opposed': 'positive-normal-force thumb and index contacts with negative dot product of bowl-centered XY radial vectors',
                                  'contact_alignment': 'forces and contact geometry are final RK4 internal-stage cache; qpos and bowl Z are post-step state',
                                  'sample_period_s': .002, 'planned_steps': planned_steps},
                  'runtime': {'python': sys.version, 'prefix': sys.prefix, 'numpy': np.__version__,
                              'mujoco': mujoco.__version__, 'robosuite': robosuite.__version__, 'libero_commit': commit},
                  'qpos': data.qpos.tolist(), 'qvel': data.qvel.tolist(), 'ctrl': data.ctrl.tolist(),
                  'controller': {'kp': backend.kp.tolist(), 'kd': backend.kd.tolist(),
                                 'initial_joint': backend.initial_joint.tolist(), 'goal_qpos': backend.goal_qpos},
                  'input_sha256_before': before_sha, 'setup_counts': counts.copy()}
        if frozen78:
            frozen.update(schedule=schedule_info, schedule_sha256=sha(output / 'schedule.json'),
                          robot_model=str(model_path.relative_to(ROOT)), robot_model_sha256=sha(model_path),
                          role_bodies=adapter['role_bodies'], role_geoms=adapter['roles'],
                          geometry_preflight=preflight,
                          old_reset_exact_match=True,
                          warning_before_integration=[int(w.number) for w in data.warning],
                          initial_contact_cache=[{'geom': [model.geom(int(c.geom1)).name, model.geom(int(c.geom2)).name],
                                                  'distance_m': float(c.dist), 'position_world_m': c.pos.tolist(),
                                                  'frame': c.frame.tolist()} for c in data.contact],
                          generated_model_sha256={str(p.relative_to(output)): sha(p) for p in adapter['paths'].values()},
                          native_scene={'seed': candidate['native_seed'], 'control_freq_hz': 20,
                                        'physics_steps_per_command': 25, 'physics_dt_s': float(model.opt.timestep),
                                        'native_constructor_count': counts['constructor'],
                                        'native_reset_count': counts['explicit_native_reset']})
            if any(w.number for w in data.warning):
                raise ValueError('MuJoCo warning before integration')
        write(output / 'frozen-before-first-step.json', frozen)
        setup_allowed, integrating = False, True
        result.update(B='RUNNING', execution='RUNNING', dynamic_attempts=1)
        trace = gzip.open(output / 'state.jsonl.gz', 'wt')
        contact_log = gzip.open(output / 'contacts.jsonl.gz', 'wt')
        bowl_id = model.body('white_bowl_1_main').id
        bowl_qadr = int(model.jnt_qposadr[model.body_jntadr[bowl_id]])
        reset_z, settled_z = float(data.qpos[bowl_qadr+2]), None
        hold_count = best_hold = 0
        maxima = {'arm_tracking_rad': 0., 'active_tracking_rad': 0., 'active_hold_tracking_rad': 0.,
                  'mimic_rad': 0., 'joint_overshoot_rad': 0., 'robot_overlap_m': 0., 'bowl_lift_m': 0.}
        current = None

        class StopTrial(RuntimeError):
            pass

        def real_step(m, d):
            nonlocal settled_z, hold_count, best_hold
            if not integrating or counts['mj_step'] >= planned_steps:
                raise RuntimeError('Unplanned physical step')
            prior_time = float(d.time)
            originals['mj_step'](m, d)
            counts['mj_step'] += 1
            result['physics_steps'] = counts['mj_step']
            q = d.qpos[qadr].copy()
            arm_error = np.abs(q[:7]-np.array(current['action_13'][:7]))
            active_error = np.abs(q[ACTIVE]-np.array(current['target_qpos_18'])[ACTIVE])
            expected = np.array([.004592+.8808*q[8]] + [.011072+.9206*q[i] for i in (10,12,14,16)])
            mimic = float(np.max(np.abs(q[[9,11,13,15,17]]-expected)))
            limits = model.jnt_range[jids]
            overshoot = float(max(0., np.max(limits[:,0]-q), np.max(q-limits[:,1])))
            contacts, overlap = [], 0.
            for i, c in enumerate(d.contact):
                force = np.zeros(6)
                mujoco.mj_contactForce(m, d, i, force)
                gids = [int(c.geom1), int(c.geom2)]
                bodies = [m.body(int(m.geom_bodyid[g])).name for g in gids]
                names_ = [m.geom(g).name for g in gids]
                reasons = filter_reasons(m, *gids)
                record = {'geom': names_, 'body': bodies, 'distance_m': float(c.dist),
                          'position_world_m': c.pos.tolist(), 'frame': c.frame.tolist(),
                          'normal_world': c.frame[:3].tolist(),
                          'cached_body_pose': [{'position_world_m': d.xpos[int(m.geom_bodyid[g])].tolist(),
                                                'rotation_world': d.xmat[int(m.geom_bodyid[g])].tolist()}
                                               for g in gids],
                          'force_contact_frame': force.tolist(), 'filter_reasons': reasons}
                contacts.append(record)
                if any(n.startswith(('robot0_', 'gripper0_', 'mount0_')) for n in bodies) and not reasons:
                    overlap = max(overlap, -float(c.dist))
            bowl_position = d.qpos[bowl_qadr:bowl_qadr+3].copy()
            if counts['mj_step'] == 250:
                settled_z = float(bowl_position[2])
            lift = None if settled_z is None else float(bowl_position[2]-settled_z)
            support, opposed = support_and_opposition(contacts, bowl_position[:2])
            holding = current['phase'] == 'hold' and lift is not None and lift >= criteria['lift_from_own_settled_baseline_m_min'] and not support and opposed
            hold_count = hold_count+1 if holding else 0
            best_hold = max(best_hold, hold_count)
            warnings = {str(mujoco.mjtWarning(i)): int(w.number) for i,w in enumerate(d.warning) if w.number}
            finite = all(np.isfinite(v).all() for v in (d.qpos,d.qvel,d.qacc,d.ctrl,d.actuator_force))
            values = [float(max(arm_error)), float(max(active_error)),
                      float(max(active_error)) if current['phase']=='hold' else 0.,
                      mimic, overshoot, overlap, 0. if lift is None else lift]
            for name, value in zip(maxima, values):
                maxima[name] = max(maxima[name], value)
            reasons = []
            if not finite: reasons.append('NONFINITE_STATE')
            if warnings or not np.isclose(float(d.time)-prior_time, .002, rtol=0, atol=1e-12): reasons.append('NUMERICAL_WARNING_OR_TIME_RESET')
            if max(arm_error) > criteria['arm_tracking_error_rad_max']: reasons.append('ARM_TRACKING')
            if current['phase']=='hold' and max(active_error) > criteria['steady_active_error_rad_max']: reasons.append('ACTIVE_HOLD_TRACKING')
            if mimic > criteria['linear_mimic_error_rad_max']: reasons.append('MIMIC')
            if overshoot > criteria['joint_limit_overshoot_rad_max']: reasons.append('JOINT_LIMIT')
            if overlap > criteria['eligible_internal_and_external_overlap_m_max']: reasons.append('ROBOT_CONTACT_OVERLAP')
            if reasons:
                result['first_failure'] = {'step': counts['mj_step'], 'macro': current['macro'],
                                           'source_macro': current.get('source_macro', current['macro']),
                                           'repeat_index': current.get('repeat_index', 0),
                                           'phase': current['phase'], 'reasons': reasons}
            state = {'step': counts['mj_step'], 'macro': current['macro'], 'phase': current['phase'],
                     'time_s': float(d.time), 'full_qpos': d.qpos.tolist(), 'full_qvel': d.qvel.tolist(),
                     'robot_state_18': q.tolist(), 'action_13': current['action_13'], 'ctrl': d.ctrl.tolist(),
                     'actuator_force': d.actuator_force.tolist(), 'arm_error_rad': arm_error.tolist(),
                     'active_error_rad': active_error.tolist(), 'mimic_rad': mimic, 'joint_overshoot_rad': overshoot,
                     'bowl_position_m': bowl_position.tolist(), 'lift_from_own_settled_m': lift,
                     'lift_from_reset_m': float(bowl_position[2]-reset_z),
                     'target_qpos_18': current['target_qpos_18'],
                     'source_macro': current.get('source_macro', current['macro']),
                     'repeat_index': current.get('repeat_index', 0),
                     'environment_support': support, 'opposed_thumb_index_contact': opposed,
                     'continuous_valid_hold_steps': hold_count, 'robot_overlap_m': overlap,
                     'warnings': warnings, 'finite': finite, 'failures': reasons}
            trace.write(json.dumps(finite_json(state), allow_nan=False)+'\n')
            contact_log.write(json.dumps(finite_json({'step': counts['mj_step'],
                'timing': 'final RK4 internal-stage cache; qpos/height in state row are post-integration; no post-step forward',
                'bowl_center_cached_m': d.xpos[bowl_id].tolist(), 'contacts': contacts}), allow_nan=False)+'\n')
            if reasons:
                raise StopTrial(','.join(reasons))

        mujoco.mj_step = real_step
        try:
            for row in actions:
                current = row
                previous_steps = counts['mj_step']
                env.step(np.array(row['action_13']))
                if counts['mj_step'] - previous_steps != 25:
                    raise RuntimeError('Native command did not execute exactly 25 physical steps')
                result['completed_commands'] += 1
                if result['completed_commands'] % 20 == 0:
                    print(f'command {result["completed_commands"]}/{len(actions)}, physics steps {counts["mj_step"]}', flush=True)
            result['execution'] = 'FINISHED'
        except StopTrial:
            result['execution'] = 'STOPPED_AT_FIRST_FAILURE'
        result['native_task_result'] = bool(env.check_success())
        result['control_health'] = 'FAIL' if result.get('first_failure') else 'PASS'
        result['grasp_result'] = 'PASS_FIXED_WHITE_BOWL' if best_hold*.002 >= criteria['continuous_hold_s_min'] else ('FAIL_FIXED_WHITE_BOWL' if result['execution']=='FINISHED' else 'UNKNOWN_EARLY_STOP')
        result['B'] = 'PASS' if result['control_health']=='PASS' and result['grasp_result']=='PASS_FIXED_WHITE_BOWL' and counts['mj_step']==planned_steps else 'FAIL'
        result.update(maxima=maxima, reset_bowl_z_m=reset_z, own_settled_bowl_z_m=settled_z,
                      final_bowl_z_m=float(data.qpos[bowl_qadr+2]),
                      lift_from_reset_m=float(data.qpos[bowl_qadr+2]-reset_z),
                      lift_from_own_settled_m=None if settled_z is None else float(data.qpos[bowl_qadr+2]-settled_z),
                      longest_continuous_valid_hold_s=best_hold*.002,
                      input_sha256_after={str(p.relative_to(ROOT)):sha(p) for p in sorted(files)})
        result['inputs_unchanged'] = before_sha == result['input_sha256_after']
        if not result['inputs_unchanged']:
            result.update(B='FAIL', input_integrity='FAIL_CHANGED_FROZEN_INPUT')
        return 0 if result['B']=='PASS' else 2
    except Exception:
        result.update(execution='SETUP_OR_RECORDING_ERROR', error=traceback.format_exc())
        if result['physics_steps']:
            result['B'] = 'FAIL'
        print(result['error'], file=sys.stderr, flush=True)
        return 1
    finally:
        for name, func in originals.items(): setattr(mujoco,name,func)
        if trace: trace.close()
        if contact_log: contact_log.close()
        if env: env.close()
        result['counts'] = counts
        write(output/'result.json', result)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path)
    parser.add_argument('--geometry', type=Path)
    parser.add_argument('--frozen78', action='store_true', help='Approved fixed78 model and approach_high x2 schedule; one attempt only')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.frozen78:
        if args.candidate is not None or args.geometry is not None:
            parser.error('--frozen78 selects exact bundled inputs; do not override them')
        args.candidate = ROOT / 'examples/right_hand/evidence/d3_white_bowl_001/candidate.json'
    elif args.candidate is None or args.geometry is None:
        parser.error('Original17 mode requires --candidate and --geometry')
    output = args.output.resolve()
    if output.exists() or not output.is_relative_to(ROOT) or os.environ.get('PYTHONPATH') or os.environ.get('PYTHONHOME'):
        raise SystemExit('Use a NEW output directory inside this activated independent checkout')
    output.mkdir(parents=True)
    raise SystemExit(execute(args.candidate.resolve(), None if args.geometry is None else args.geometry.resolve(), output, args.frozen78))
