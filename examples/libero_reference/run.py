#!/usr/bin/env python3
"""Run the delivered robot in the native LIBERO white-bowl reference scene.

All runtime inputs are in this checkout and its pinned public dependencies.
The bundled golden trace is an explicit acceptance reference, never an action
source. No physical parameter is changed to fit a reference trajectory.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import traceback
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FIXTURES = HERE / 'fixtures'


def classify_outcome(case, equal, steps, commands):
    reason = case.get('stop_reason') or ''
    if case.get('recording_error') or reason.startswith(('RUNTIME_ERROR:', 'SETUP_OR_OUTPUT_ERROR:')):
        return 'FAILED', 1
    if equal and steps == 5750 and commands == 230 and case['status'] == 'DIAGNOSTIC_FINISHED':
        return 'EXACT_REFERENCE_REPRODUCED', 0
    return 'COMPLETED_WITH_DIFFERENCES', 2


def write_json(path, value):
    def encode(item):
        if hasattr(item, 'tolist'):
            return item.tolist()
        if isinstance(item, Path):
            return str(item)
        raise TypeError(type(item).__name__)
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, default=encode, allow_nan=False)+'\n')


def reset_snapshot(env):
    import mujoco
    model, data = env.sim.model._model, env.sim.data._data
    return {'qpos': data.qpos.tolist(), 'qvel': data.qvel.tolist(), 'ctrl': data.ctrl.tolist(),
            'time': float(data.time), 'seed': 20261007,
            'body_names': [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)],
            'runtime_body_pos': model.body_pos.tolist(), 'runtime_body_quat': model.body_quat.tolist()}


def execute(output, reference):
    import numpy as np
    import mujoco
    import robosuite
    import yaml
    from actions import generate_actions
    from checks import audit_sources, audit_runtime, sha256
    from compare import compare_traces
    from diagnostic import run
    from model import register_models, configure_native_scene

    original = {key: getattr(mujoco, key) for key in ('mj_step', 'mj_step1', 'mj_step2')}
    result = {'status': 'PREFLIGHT', 'physics_steps': 0, 'commands': 0,
              'robot_definition': 'mjcf/nero_dh116.xml', 'robot_sha256': sha256(ROOT / 'mjcf/nero_dh116.xml'),
              'reference': str(reference), 'reference_is_action_source': False,
              'native_task_result': None, 'strict_grasp_success_claim': False,
              'model_modified': False, 'tolerances_added': False}
    env, contacts = None, None
    try:
        if sys.version_info[:3] != (3, 11, 16) or mujoco.__version__ != '3.2.3' or robosuite.__version__ != '1.4.1':
            raise ValueError('Use the hash-pinned Python3.11.16/MuJoCo3.2.3/stock robosuite1.4.1 environment')
        for module in (np, mujoco, robosuite):
            if not Path(module.__file__).resolve().is_relative_to(ROOT / '.reference-env'):
                raise ValueError(f'Runtime module outside the clean environment: {module.__file__}')
        libero = ROOT / '.reference-deps/LIBERO'
        commit = subprocess.check_output(['git', '-C', str(libero), 'rev-parse', 'HEAD'], text=True).strip()
        if commit != 'f78abd68ee283de9f9be3c8f7e2a9ad60246e95c':
            raise ValueError('Pinned LIBERO commit differs')
        if subprocess.check_output(['git', '-C', str(libero), 'diff', '--name-only', 'HEAD'], text=True).strip():
            raise ValueError('LIBERO tracked source has modifications')
        source = audit_sources(ROOT, FIXTURES)
        write_json(output / 'source-preflight.json', source)
        if not source['passed']:
            raise ValueError(f'Source preflight failed: {source["errors"]}')
        actions = generate_actions(json.loads((FIXTURES / 'initial-joint-config.json').read_text()),
                                   json.loads((FIXTURES / 'prepared-white-geometry-plan.json').read_text()))
        with gzip.open(FIXTURES / 'golden/bounded_grasp.jsonl.gz', 'rt') as stream:
            for index, line in enumerate(stream):
                row = json.loads(line)
                if row['robot_action_13'] != actions[index//25]['action'] or row['phase'] != actions[index//25]['phase']:
                    raise ValueError(f'Frozen command differs from bundled acceptance reference at row {index}')
        if index+1 != 5750 or len(actions) != 230:
            raise ValueError('Unexpected frozen command count')
        write_json(output / 'action-preflight.json', {'commands': 230, 'expanded_steps': 5750, 'max_absolute_difference': 0})
        write_json(output / 'actions-230.json', actions)
        metadata = json.loads((FIXTURES / 'robot-metadata.json').read_text())
        roles = json.loads((FIXTURES / 'contact-roles.json').read_text())['roles']
        adapter = register_models(ROOT, output, metadata, roles)
        root_lib = libero / 'libero/libero'
        cfg = output / 'libero_config'
        cfg.mkdir()
        (cfg / 'config.yaml').write_text(yaml.safe_dump({
            'benchmark_root': str(root_lib), 'bddl_files': str(root_lib / 'bddl_files'),
            'init_states': str(root_lib / 'init_files'), 'assets': str(root_lib / 'assets'),
            'datasets': str(libero / 'datasets')}))
        os.environ['LIBERO_CONFIG_PATH'] = str(cfg)
        result['environment'] = {'python': sys.version, 'executable': sys.executable,
            'mujoco': mujoco.__version__, 'robosuite': robosuite.__version__, 'numpy': np.__version__,
            'libero_commit': commit, 'sys_path': sys.path,
            'module_paths': {module.__name__: module.__file__ for module in (np, mujoco, robosuite)}}
        refreshes = []
        def prohibited(*args, **kwargs):
            raise RuntimeError('Integration prohibited before physical/Reset/controller preflight')
        def static_step1(model, data):
            if mujoco.get_mjcb_control() is not None:
                raise ValueError('Unexpected control callback during reset')
            prior = [float(data.time), data.qpos.copy(), data.qvel.copy(), data.act.copy(), data.ctrl.copy()]
            original['mj_step1'](model, data)
            if not all(np.array_equal(a, b) for a, b in zip(prior, [float(data.time), data.qpos, data.qvel, data.act, data.ctrl])):
                raise ValueError('Native reset refresh changed state')
            refreshes.append({'time': float(data.time), 'state_unchanged': True})
        mujoco.mj_step = mujoco.mj_step2 = prohibited
        mujoco.mj_step1 = static_step1
        from libero.libero.envs.env_wrapper import ControlEnv
        configure_native_scene(ROOT)
        task = root_lib / 'bddl_files/libero_90/KITCHEN_SCENE9_put_the_white_bowl_on_top_of_the_cabinet.bddl'
        result['task_sha256'] = sha256(task)
        if result['task_sha256'] != 'a595358c840e6dca31d2d914726bb49d58944252ae868796bd603345a444b627':
            raise ValueError('Native white bowl task differs')
        env = ControlEnv(str(task), robots=['NeroURDF'], controller='JOINT_POSITION',
            camera_names=['agentview', 'robot0_eye_in_hand'], camera_heights=256, camera_widths=256,
            initialization_noise=None, hard_reset=False)
        env.seed(20261007)
        obs = env.reset()
        robot = env.robots[0]
        if np.r_[obs['robot0_joint_pos'], obs['robot0_gripper_qpos']].shape != (18,):
            raise ValueError('Robot state must contain 18 joint positions')
        lower, upper = adapter['lower'], adapter['upper']
        low, high = env.env.action_spec
        if not (np.array_equal(low[:7], lower) and np.array_equal(high[:7], upper)
                and np.array_equal(low[7:], [-1]*6) and np.array_equal(high[7:], [1]*6)):
            raise ValueError('Native action contract differs')
        target = (lower+upper)/2
        target[0] = 2.1
        robot.controller.set_goal(target)
        if not np.array_equal(robot.controller.goal_qpos, target):
            raise ValueError('Absolute set_qpos adapter differs')
        command = np.array([-.9, -.5, -.1, .2, .6, .9])
        if not np.array_equal(robot.gripper.format_action(command), command):
            raise ValueError('Independent normalized hand adapter differs')
        write_json(output / 'first-reset-state.json', reset_snapshot(env))
        native_step = env.step
        def guarded_env_step(action):
            index = result['commands']
            if index >= len(actions) or not np.array_equal(action, actions[index]['action']):
                raise ValueError(f'Actual action differs at command {index}')
            value = native_step(action)
            result['commands'] += 1
            if result['commands'] % 20 == 0:
                print(f'command {result["commands"]}/230, t={env.sim.data.time:.3f}s', flush=True)
            return value
        env.step = guarded_env_step
        def before_step(model, data):
            write_json(output / 'full-reset-state.json', reset_snapshot(env))
            audit = audit_runtime(model, data, env.robots[0].controller, FIXTURES, ROOT)
            audit['nonintegrating_reset_refreshes'] = refreshes
            write_json(output / 'runtime-preflight.json', audit)
            # Every engine-loaded resource must resolve in this checkout as well.
            xml = ET.fromstring(env.env.model.get_xml())
            resources = [Path(item.get('file')).resolve() for item in xml.iter() if item.get('file')]
            if not all(path.is_relative_to(ROOT) for path in resources):
                raise ValueError('Scene has resources outside this checkout')
            write_json(output / 'scene-resources.json', [str(path.relative_to(ROOT)) for path in resources])
            if not audit['passed']:
                raise ValueError('Physical/Reset/controller preflight differs; see runtime-preflight.json')
            result['status'] = 'RUNNING'
            write_json(output / 'run-result.json', result)
            print(f'Official-model preflight passed ({audit["fields_checked"]} fields); integrating', flush=True)
        contacts = (output / 'contact-details.jsonl').open('w')
        def real_step(model, data):
            original['mj_step'](model, data)
            result['physics_steps'] += 1
            pairs = []
            for index, contact in enumerate(data.contact):
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, index, force)
                pairs.append({'geom': [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in (contact.geom1, contact.geom2)],
                              'body': [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) for g in (contact.geom1, contact.geom2)],
                              'distance_m': float(contact.dist), 'force_contact_frame': force.tolist()})
            contacts.write(json.dumps({'row': result['physics_steps']-1, 'time': float(data.time),
                                      'contacts': pairs, 'actuator_force': data.actuator_force.tolist()})+'\n')
        dynamic = run(env, output, real_step, ROOT, FIXTURES, before_step=before_step)
        case = dynamic['cases'][0]
        result.update(diagnostic_status=case['status'], stop_reason=case.get('stop_reason'),
                      native_task_result=case.get('native_task_signal'),
                      strict_grasp_success_claim=case.get('strict_grasp_success_claim', False))
        if result['physics_steps'] == 0:
            raise ValueError('Reference did not integrate; see preflight and diagnostic reports')
        comparison = compare_traces(output / 'bounded_grasp.jsonl', reference,
                                    reset_z=case.get('initial_bowl_z_m', .97))
        write_json(output / 'comparison.json', comparison)
        write_json(output / 'metrics.json', comparison['metrics']['actual'])
        status, exit_code = classify_outcome(case, comparison['historical_trajectory_equal'], result['physics_steps'], result['commands'])
        result.update(status=status,
                      historical_trajectory_equal=comparison['historical_trajectory_equal'],
                      first_difference=comparison['first_difference'], trace_sha256=sha256(output / 'bounded_grasp.jsonl'))
        return exit_code
    except Exception:
        result.update(status='FAILED', error=traceback.format_exc())
        print(result['error'], file=sys.stderr, flush=True)
        return 1
    finally:
        for name, function in original.items():
            setattr(mujoco, name, function)
        try:
            if contacts is not None:
                contacts.close()
            if env is not None:
                env.close()
        finally:
            write_json(output / 'run-result.json', result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New output directory inside this checkout')
    parser.add_argument('--reference', type=Path, default=FIXTURES / 'golden/bounded_grasp.jsonl.gz',
                        help='Explicit acceptance trace; default is the bundled D1-A golden, used only for comparison')
    args = parser.parse_args()
    output = args.output.resolve()
    try:
        if not output.is_relative_to(ROOT) or output.exists():
            raise ValueError('Use a new output directory inside this checkout')
        if os.environ.get('PYTHONPATH') or os.environ.get('PYTHONHOME'):
            raise ValueError('Unset PYTHONPATH/PYTHONHOME; source env/activate.sh')
        output.mkdir(parents=True)
        os.environ.setdefault('MUJOCO_GL', 'egl')
        os.environ.setdefault('MPLBACKEND', 'Agg')
        return execute(output, args.reference.resolve())
    except Exception as error:
        print(str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
