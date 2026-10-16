"""Exercise the runtime wrapper using a tiny real MuJoCo model and fake env."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

try:
    import numpy as np
    import mujoco
except ImportError:
    np = mujoco = None

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def diagnostic():
    spec = importlib.util.spec_from_file_location('reference_diagnostic', HERE / 'diagnostic.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@unittest.skipIf(mujoco is None, 'runtime checks require declared simulation dependencies')
class RuntimeTests(unittest.TestCase):
    def environment(self, failure=None):
        # Match the public 18-joint/13-actuator interface, without LIBERO or assets.
        arm = [f'robot0_joint{i}' for i in range(1, 8)]
        # Use official order from the delivered URDF (18 = 7 + 11).
        import xml.etree.ElementTree as ET
        urdf = ET.parse(ROOT / 'urdf/dh116_left.urdf').getroot()
        hand = ['gripper0_' + j.get('name') for j in urdf.findall('joint') if j.get('type') != 'fixed']
        self.assertEqual(len(hand), 11)
        names = arm + hand
        bodies = ''.join(f'<body name="{name}_link" pos="0 0 .1"><joint name="{name}" range="-3 3"/><geom size=".01" contype="0" conaffinity="0"/>' for name in names)
        xml = '<mujoco><compiler angle="radian"/><worldbody><body name="gripper0_base_link">' + bodies + '</body>' * len(names) + '</body><body name="white_bowl_1_main" pos="0 0 .97"><geom size=".01" contype="0" conaffinity="0"/></body></worldbody><actuator>'
        xml += ''.join(f'<position joint="{name}"/>' for name in (arm + [hand[i] for i in (0, 1, 4, 6, 8, 10)])) + '</actuator></mujoco>'
        model = mujoco.MjModel.from_xml_string(xml)
        model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        model.body_weldid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper0_base_link')] = 1
        robot = SimpleNamespace(robot_model=SimpleNamespace(joints=arm), gripper=SimpleNamespace(joints=hand))
        class Environment:
            robots = [robot]
            sim = SimpleNamespace(model=SimpleNamespace(_model=model), data=SimpleNamespace(_data=data))
            reset_count = 0
            def seed(self, seed):
                self.seed_value = seed
            def observation(self):
                return {'robot0_joint_pos': data.qpos[:7].copy(), 'robot0_gripper_qpos': data.qpos[7:].copy()}
            def reset(self):
                self.reset_count += 1
                return self.observation()
            def step(self, action):
                if failure == 'runtime':
                    raise RuntimeError('injected first macro failure')
                for _ in range(25):
                    mujoco.mj_step(model, data)
                return self.observation(), 0, False, {}
            def check_success(self):
                return False
        return Environment(), model, data

    def fixtures(self, directory):
        initial = {'arm_absolute_qpos_rad': [0.] * 7,
                   'hand_active_source_range_fractions': [0.] * 6}
        plan = {'nodes_contact_to_above': [{'arm_absolute_qpos_rad': [0.] * 7} for _ in range(11)],
                'open_source_fractions': [0.] * 6, 'closed_source_fractions': [0.] * 6}
        (directory / 'initial-joint-config.json').write_text(json.dumps(initial))
        (directory / 'prepared-white-geometry-plan.json').write_text(json.dumps(plan))

    def test_first_physics_stop_keeps_progress_and_restores_mj_step(self):
        module = diagnostic()
        env, model, data = self.environment()
        original = mujoco.mj_step
        events = []
        def before_step(m, d):
            events.append(('before', env.reset_count))
        def real_step(m, d):
            events.append(('step', env.reset_count))
            d.time += .002
            d.warning[mujoco.mjtWarning.mjWARN_BADQPOS].number = 1
            return {'contact_details': []}
        with tempfile.TemporaryDirectory() as temporary:
            out = Path(temporary)
            self.fixtures(out)
            report = module.run(env, out, real_step, ROOT, out, before_step=before_step)
            case = report['cases'][0]
            self.assertEqual(case['status'], 'DIAGNOSTIC_STOPPED')
            self.assertEqual(case['stop_reason'], 'NUMERICAL_WARNING_OR_RESET')
            self.assertEqual(case['physics_steps'], 1)
            self.assertEqual(case['scripted_command_steps'], 0)
            self.assertEqual(events, [('before', 1), ('step', 1)])
            self.assertTrue((out / 'dynamic-results.json').exists())
            row = json.loads((out / 'bounded_grasp.jsonl').read_text())
            self.assertNotIn('contact_details', row)
            self.assertEqual(row['warnings'], {'mjtWarning.mjWARN_BADQPOS': 1})
            fixture = json.loads((out / 'final-observation-fixture.json').read_text())
            self.assertNotIn('black bowl', fixture.get('prompt', ''))
            self.assertEqual(len(fixture['state_18']), 18)
        self.assertIs(mujoco.mj_step, original)

    def test_all_commands_and_physics_rows_preserve_historical_phase_schedule(self):
        module = diagnostic()
        env, model, data = self.environment()
        fixed = {name: getattr(model, name).copy() for name in [
            'body_mass', 'body_inertia', 'jnt_range', 'jnt_solref', 'jnt_solimp',
            'dof_damping', 'dof_armature', 'dof_frictionloss', 'actuator_gainprm',
            'actuator_biasprm', 'actuator_ctrlrange', 'eq_data', 'eq_solref', 'eq_solimp',
            'geom_friction', 'geom_solref', 'geom_solimp', 'exclude_signature']}
        original = mujoco.mj_step
        def real_step(m, d):
            d.time += .002
        with tempfile.TemporaryDirectory() as temporary:
            out = Path(temporary)
            self.fixtures(out)
            report = module.run(env, out, real_step, ROOT, out)
            case = report['cases'][0]
            self.assertTrue(report['frozen_official_parameters_preserved'])
            self.assertFalse(report['reset_parameter_replay'])
            self.assertNotIn('hand_position_feedback_and_equality_solver_parameters_adjusted', report)
            self.assertEqual(case['status'], 'DIAGNOSTIC_FINISHED')
            self.assertIsNone(case['stop_reason'])
            self.assertEqual((case['scripted_command_steps'], case['physics_steps']), (230, 5750))
            rows = [json.loads(line) for line in (out / 'bounded_grasp.jsonl').read_text().splitlines()]
            self.assertEqual([rows[i]['phase'] for i in (249, 250, 1249, 1250, 2999, 3000, 3999, 4000, 4999, 5000)],
                             ['settle', 'approach_high', 'approach_high', 'descend', 'descend',
                              'close', 'close', 'lift', 'lift', 'hold'])
            self.assertEqual(list(rows[0]), ['time', 'phase', 'bowl_z_m', 'thumb_contact', 'opposed_contact',
                             'other_robot_overlap_m', 'robot_state_18', 'robot_action_13', 'internal_actuator_ctrl_13',
                             'linear_mimic_error_rad', 'max_hand_target_error_rad', 'bowl_hand_contact_distances_m',
                             'max_contact_normal_N', 'warnings', 'full_qpos', 'full_qvel'])
        self.assertIs(mujoco.mj_step, original)
        for name, value in fixed.items():
            np.testing.assert_array_equal(getattr(model, name), value, err_msg=name)

    def test_reset_error_is_recorded_without_claiming_finished(self):
        module = diagnostic()
        env, model, data = self.environment()
        def failed_reset():
            raise RuntimeError('injected reset error')
        env.reset = failed_reset
        with tempfile.TemporaryDirectory() as temporary:
            out = Path(temporary)
            self.fixtures(out)
            report = module.run(env, out, lambda m, d: None, ROOT, out)
            self.assertEqual(report['cases'][0]['status'], 'DIAGNOSTIC_STOPPED')
            self.assertIn('injected reset error', report['cases'][0]['stop_reason'])
            self.assertEqual(report['cases'][0]['physics_steps'], 0)
            self.assertTrue((out / 'dynamic-results.json').exists())

    def test_first_macro_runtime_error_is_not_finished(self):
        module = diagnostic()
        env, model, data = self.environment(failure='runtime')
        original = mujoco.mj_step
        with tempfile.TemporaryDirectory() as temporary:
            out = Path(temporary)
            self.fixtures(out)
            report = module.run(env, out, lambda m, d: None, ROOT, out)
            case = report['cases'][0]
            self.assertEqual(case['status'], 'DIAGNOSTIC_STOPPED')
            self.assertIn('injected first macro failure', case['stop_reason'])
            self.assertEqual(case['physics_steps'], 0)
            self.assertEqual(case['scripted_command_steps'], 0)
            self.assertTrue((out / 'bounded_grasp-result.json').exists())
        self.assertIs(mujoco.mj_step, original)


if __name__ == '__main__':
    unittest.main()
