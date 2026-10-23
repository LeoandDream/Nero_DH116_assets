"""One native frozen L5750 regression with the reconstructed wrist candidate.

The original diagnostic, actions, native Reset and stock controller execute
unchanged. Only the explicit robot model selection and recording are added.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
REFERENCE = ROOT/'examples/libero_reference'
sys.path.insert(0, str(REFERENCE))
import compare as comparison
import run as old_entry
import model as adapter_module
import checks
from actions import generate_actions
from diagnostic import run as frozen_diagnostic


def dump(path, value):
    old_entry.write_json(path, value)


def register_candidate(asset_root, output, metadata, roles, candidate):
    """Choose the split input without editing the hash-frozen adapter code.

    register_models still reads limits/initial state/mount metadata from V1;
    static and runtime compiled audits require those fields to match exactly.
    All model bodies/meshes/actuators/equalities come from the candidate split.
    """
    original_split = adapter_module.split_official_model
    def candidate_split(source, directory):
        if Path(source).resolve() != (Path(asset_root)/'mjcf/nero_dh116.xml').resolve():
            raise ValueError('Unexpected frozen adapter source request')
        return original_split(candidate, directory)
    adapter_module.split_official_model = candidate_split
    try:
        return adapter_module.register_models(asset_root, output, metadata, roles)
    finally:
        adapter_module.split_official_model = original_split


def execute(args):
    import mujoco as mj
    import numpy as np
    import robosuite
    import yaml
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    candidate = args.candidate.resolve()
    fixtures = REFERENCE/'fixtures'
    # The helpers imported above must be the frozen reference implementation,
    # not this directory's comparison microbenchmark module.
    if Path(comparison.__file__).resolve() != REFERENCE/'compare.py':
        raise ValueError('Wrong comparison module')
    protocol = json.loads((Path(__file__).with_name('protocol.json')).read_text())
    preflight = json.loads(args.preflight.read_text())
    if preflight['status'] != 'PASS_FINITE_TASK_PREFLIGHT' or preflight['candidate_model_sha256'] != checks.sha256(candidate):
        raise ValueError('Candidate static preflight is missing or belongs to another model')
    if sys.version_info[:3] != (3, 11, 16) or mj.__version__ != '3.2.3' or robosuite.__version__ != '1.4.1':
        raise ValueError('Frozen runtime versions required')
    libero = args.libero_root.resolve()
    if subprocess.check_output(['git', '-C', str(libero), 'rev-parse', 'HEAD'], text=True).strip() != 'f78abd68ee283de9f9be3c8f7e2a9ad60246e95c':
        raise ValueError('LIBERO commit changed')
    if subprocess.check_output(['git', '-C', str(libero), 'diff', '--name-only', 'HEAD'], text=True).strip():
        raise ValueError('LIBERO tracked source changed')
    source = checks.audit_sources(ROOT, fixtures)
    if not source['passed']:
        raise ValueError('Official frozen L source inputs changed')
    actions = generate_actions(json.loads((fixtures/'initial-joint-config.json').read_text()),
                               json.loads((fixtures/'prepared-white-geometry-plan.json').read_text()))
    with gzip.open(fixtures/'golden/bounded_grasp.jsonl.gz', 'rt') as stream:
        for i, line in enumerate(stream):
            row = json.loads(line)
            if row['robot_action_13'] != actions[i//25]['action'] or row['phase'] != actions[i//25]['phase']:
                raise ValueError('Frozen action schedule changed')
    if i+1 != 5750 or len(actions) != 230:
        raise ValueError('Wrong action count')
    dump(output/'actions-230.json', actions)
    dump(output/'registered.json', {'issue': protocol['issue'], 'candidate': str(candidate.relative_to(ROOT)),
        'candidate_sha256': checks.sha256(candidate), 'protocol_sha256': checks.sha256(Path(__file__).with_name('protocol.json')),
        'actions_sha256': checks.sha256(output/'actions-230.json'), 'max_physics_steps': 5750,
        'diagnostic_sha256': checks.sha256(REFERENCE/'diagnostic.py'), 'functional_script_sha256': checks.sha256(__file__),
        'python': sys.version, 'mujoco': mj.__version__, 'robosuite': robosuite.__version__,
        'libero_commit': 'f78abd68ee283de9f9be3c8f7e2a9ad60246e95c', 'new_controller': False,
        'stopping_rules': 'Unmodified frozen left diagnostic.py; no tolerance or threshold changes',
        'full_reset_flow': 'native constructor, seed/reset, absolute-goal probe, diagnostic seed/reset',
        'publication_scope': 'Fixed white-bowl research regression; native cabinet success reported separately'})
    config = output/'libero_config'
    config.mkdir()
    lib = libero/'libero/libero'
    (config/'config.yaml').write_text(yaml.safe_dump({'benchmark_root': str(lib),
        'bddl_files': str(lib/'bddl_files'), 'init_states': str(lib/'init_files'),
        'assets': str(lib/'assets'), 'datasets': str(libero/'datasets')}))
    os.environ['LIBERO_CONFIG_PATH'] = str(config)
    metadata = json.loads((fixtures/'robot-metadata.json').read_text())
    roles = json.loads((fixtures/'contact-roles.json').read_text())['roles']
    adapter = register_candidate(ROOT, output, metadata, roles, candidate)
    originals = {k: getattr(mj, k) for k in ('mj_step', 'mj_step1', 'mj_step2')}
    env, contact_stream = None, None
    result = {'status': 'PREFLIGHT', 'physics_steps': 0, 'commands': 0, 'functional_rollouts': 0,
              'native_task_result': None, 'strict_grasp_success_claim': False, 'retry': False}
    perf_ns = 0
    reset_refreshes = []

    def prohibited(*a, **k):
        raise RuntimeError('Integration prohibited before frozen preflight')

    def static_step1(m, d):
        before = [float(d.time), d.qpos.copy(), d.qvel.copy(), d.ctrl.copy()]
        originals['mj_step1'](m, d)
        if not all(np.array_equal(a, b) for a, b in zip(before, [float(d.time), d.qpos, d.qvel, d.ctrl])):
            raise ValueError('Reset refresh changed integration state')
        reset_refreshes.append(float(d.time))

    try:
        mj.mj_step = mj.mj_step2 = prohibited
        mj.mj_step1 = static_step1
        from libero.libero.envs.env_wrapper import ControlEnv
        adapter_module.configure_native_scene(ROOT)
        task = lib/'bddl_files/libero_90/KITCHEN_SCENE9_put_the_white_bowl_on_top_of_the_cabinet.bddl'
        if checks.sha256(task) != 'a595358c840e6dca31d2d914726bb49d58944252ae868796bd603345a444b627':
            raise ValueError('Native task changed')
        env = ControlEnv(str(task), robots=['NeroURDF'], controller='JOINT_POSITION',
            camera_names=['agentview', 'robot0_eye_in_hand'], camera_heights=256, camera_widths=256,
            initialization_noise=None, hard_reset=False)
        env.seed(20261007)
        obs = env.reset()
        robot = env.robots[0]
        if np.r_[obs['robot0_joint_pos'], obs['robot0_gripper_qpos']].shape != (18,):
            raise ValueError('18D state changed')
        low, high = env.env.action_spec
        if not (np.array_equal(low[:7], adapter['lower']) and np.array_equal(high[:7], adapter['upper']) and
                np.array_equal(low[7:], [-1]*6) and np.array_equal(high[7:], [1]*6)):
            raise ValueError('Action interface changed')
        probe = (adapter['lower']+adapter['upper'])/2
        probe[0] = 2.1
        robot.controller.set_goal(probe)
        if not np.array_equal(robot.controller.goal_qpos, probe):
            raise ValueError('Absolute control probe failed')
        hand_probe = np.array([-.9, -.5, -.1, .2, .6, .9])
        if not np.array_equal(robot.gripper.format_action(hand_probe), hand_probe):
            raise ValueError('Hand mapping changed')
        dump(output/'first-reset-state.json', old_entry.reset_snapshot(env))
        native_step = env.step

        def guarded_env_step(action):
            index = result['commands']
            if index >= 230 or not np.array_equal(action, actions[index]['action']):
                raise ValueError('Command differs from frozen 230 schedule')
            value = native_step(action)
            result['commands'] += 1
            if result['commands'] % 20 == 0:
                print('native L command', result['commands'], '/230', flush=True)
            return value
        env.step = guarded_env_step

        def before_step(m, d):
            # Exact nonwrist compiled comparison against the old native L scene.
            sys.path.insert(0, str(args.history_root/'examples/right_hand'))
            import wrist_reuse as W
            reference_xml = ET.parse(args.left_run/'bounded_grasp-model.xml').getroot()
            for e in reference_xml.iter():
                if e.get('file'):
                    e.set('file', str((args.left_run/e.get('file')).resolve()))
            baseline = mj.MjModel.from_xml_string(ET.tostring(reference_xml, encoding='unicode'))
            expected_reset = json.loads((fixtures/'full-reset-state.json').read_text())
            for name, pos, quat in zip(expected_reset['body_names'], expected_reset['runtime_body_pos'], expected_reset['runtime_body_quat']):
                baseline.body_pos[baseline.body(name).id], baseline.body_quat[baseline.body(name).id] = pos, quat
            expected_model = json.loads((fixtures/'compiled-model.json').read_text())
            snapshot = checks.compiled_snapshot(baseline)
            if snapshot != expected_model:
                dump(output/'baseline-compiled-actual.json', snapshot)
                raise ValueError('Historical baseline compilation differs from frozen fixture')
            old_xml = ET.parse(ROOT/'mjcf/nero_dh116.xml').getroot()
            new_xml = ET.parse(candidate).getroot()
            def wrist_names(x):
                return [g.get('name') for body in ('robot0_link5', 'robot0_gripper_flange')
                    for g in x.find('.//body[@name="'+body+'"]').findall('geom')
                    if int(g.get('contype', '1')) or int(g.get('conaffinity', '1'))]
            physical = W.compiled_audit(baseline, m, wrist_names(old_xml), wrist_names(new_xml))
            actual_reset = old_entry.reset_snapshot(env)
            reset_equal = {k: actual_reset[k] == expected_reset[k] for k in expected_reset}
            actual_control = checks.controller_snapshot(env.robots[0].controller)
            expected_control = json.loads((fixtures/'controller-state.json').read_text())
            control_equal = {k: actual_control.get(k) == value for k, value in expected_control.items()}
            no_external_force = not np.any(d.qfrc_applied) and not np.any(d.xfrc_applied)
            audit = {'physical': physical, 'reset_exact': reset_equal, 'controller_exact': control_equal,
                     'no_external_force': bool(no_external_force), 'reset_refresh_times': reset_refreshes}
            audit['passed'] = all(reset_equal.values()) and all(control_equal.values()) and bool(no_external_force)
            dump(output/'runtime-preflight.json', audit)
            dump(output/'full-reset-state.json', actual_reset)
            if not audit['passed']:
                raise ValueError('Native Reset/controller preflight failed')
            # Exclusive ledger is shared across output paths: one real attempt.
            ledger = ROOT/'runs/issue10/functional-attempt.json'
            with ledger.open('x') as f:
                json.dump({'output': str(output), 'candidate_sha256': checks.sha256(candidate),
                           'authorized_max_steps': 5750, 'retry_allowed': False}, f, indent=2)
            result.update(status='RUNNING', functional_rollouts=1)
            dump(output/'run-result.json', result)
            print('Native Reset and compiled nonwrist fields exact; starting the single L5750 regression', flush=True)

        contact_stream = gzip.open(output/'rk4-contacts.jsonl.gz', 'xt')
        def real_step(m, d):
            nonlocal perf_ns
            if result['status'] != 'RUNNING' or result['physics_steps'] >= 5750:
                raise RuntimeError('Unregistered or excess integration')
            start = time.perf_counter_ns()
            originals['mj_step'](m, d)
            perf_ns += time.perf_counter_ns()-start
            result['physics_steps'] += 1
            contacts = []
            involved = {m.body(name).id for name in ('robot0_link5', 'robot0_gripper_flange',
                        'robot0_link6', 'gripper0_base_link', 'white_bowl_1_main')}
            for index, c in enumerate(d.contact):
                force = np.zeros(6)
                mj.mj_contactForce(m, d, index, force)
                gids = [int(c.geom1), int(c.geom2)]
                bids = [int(m.geom_bodyid[g]) for g in gids]
                involved.update(bids)
                contacts.append({'geom': [m.geom(g).name for g in gids], 'body': [m.body(b).name for b in bids],
                    'distance_m': float(c.dist), 'position_m': c.pos.tolist(), 'frame': c.frame.tolist(),
                    'force_contact_frame': force.tolist()})
            bowl = m.body('white_bowl_1_main').id
            qadr = int(m.jnt_qposadr[int(m.body_jntadr[bowl])])
            contact_stream.write(json.dumps({'step': result['physics_steps'], 'time': float(d.time),
                'timing': 'state is post-step; contact/force/body geometry is final RK4 internal-stage cache, no extra forward',
                'bowl_post_qpos_z_m': float(d.qpos[qadr+2]), 'bowl_center_cached_m': d.xpos[bowl].tolist(),
                'contacts': contacts, 'body_poses': {m.body(b).name: {'position_world_m': d.xpos[b].tolist(),
                    'rotation_world': d.xmat[b].tolist()} for b in sorted(involved)},
                'actuator_force': d.actuator_force.tolist()}, allow_nan=False)+'\n')
        report = frozen_diagnostic(env, output, real_step, ROOT, fixtures, before_step=before_step)
        case = report['cases'][0]
        result.update(status=case['status'], stop_reason=case.get('stop_reason'),
            native_task_result=case.get('native_task_signal'), strict_grasp_success_claim=False,
            mj_step_total_ns=perf_ns, mean_mj_step_us=perf_ns/max(1, result['physics_steps'])/1000)
        if result['physics_steps']:
            comparison_report = comparison.compare_traces(output/'bounded_grasp.jsonl',
                REFERENCE/'evidence/d2/bounded_grasp.jsonl.gz', reset_z=.97)
            dump(output/'historical-comparison.json', comparison_report)
            result['historical_trajectory_equal'] = comparison_report['historical_trajectory_equal']
        return 0 if case['status'] == 'DIAGNOSTIC_FINISHED' and result['physics_steps'] == 5750 else 1
    except Exception:
        result.update(status='ERROR', error=traceback.format_exc())
        print(result['error'], file=sys.stderr)
        return 1
    finally:
        for key, function in originals.items():
            setattr(mj, key, function)
        if contact_stream is not None:
            contact_stream.close()
        if env is not None:
            env.close()
        dump(output/'run-result.json', result)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--preflight', type=Path, required=True)
    parser.add_argument('--history-root', type=Path, required=True)
    parser.add_argument('--left-run', type=Path, required=True)
    parser.add_argument('--libero-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    raise SystemExit(execute(parser.parse_args()))
