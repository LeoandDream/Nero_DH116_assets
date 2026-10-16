"""Behavioral checks for the independent right-hand diagnostic recorder.

The integration cases use tiny real MuJoCo models and the stock JPC. They do
not execute the delivered robot's formal acceptance trajectory.
"""
import importlib.util
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

try:
    import mujoco
    import numpy as np
except ImportError:
    mujoco = np = None

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / 'examples/right_hand/run.py'


def runner():
    assert RUNNER.exists(), 'the independent right-hand diagnostic runner is missing'
    spec = importlib.util.spec_from_file_location('right_hand_diagnostic', RUNNER)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def source_urdf():
    hand = ['finger11', 'finger12', 'finger13', 'finger21', 'finger22',
            'finger31', 'finger32', 'finger41', 'finger42', 'finger51', 'finger52']
    mimics = {'finger13': 'finger12', 'finger22': 'finger21',
              'finger32': 'finger31', 'finger42': 'finger41', 'finger52': 'finger51'}
    root = ET.Element('robot', name='tiny_right_contract')
    for name in hand:
        joint = ET.SubElement(root, 'joint', name=name, type='revolute')
        ET.SubElement(joint, 'limit', lower='.01' if name in mimics else '0', upper='1')
        if name in mimics:
            ET.SubElement(joint, 'mimic', joint=mimics[name], multiplier='.8', offset='.01')
    return root


def tiny_model(*, probe=False, active_start=0., rigid_overlap=False, contact_body='gripper0_base_link'):
    urdf = source_urdf()
    names = [f'robot0_joint{i}' for i in range(1, 8)]
    names += ['gripper0_' + joint.get('name') for joint in urdf.findall('joint')]
    active = [names[i] for i in (7, 8, 10, 12, 14, 16)]
    world = f'<body name="{contact_body}"><geom name="palm_collision" size=".01"/>'
    world += '<site name="gripper0_grip_site"/>'
    if rigid_overlap:
        world += '<body name="gripper0_fixed_pad"><geom name="rigid_pad_collision" size=".01"/></body>'
    for i, name in enumerate(names):
        lo = '.01' if i in (9, 11, 13, 15, 17) else '0'
        world += (f'<body name="{name}_link" pos="{.1+i*.04} 0 0">'
                  f'<joint name="{name}" range="{lo} 1" axis="0 0 1"/>'
                  f'<geom name="{name}_collision" size=".01" mass=".02" contype="0" conaffinity="0"/></body>')
    world += '</body>'
    if probe:
        # Put the free object first so qpos[:18] is demonstrably incorrect.
        world = ('<body name="probe" pos="0 0 .018"><freejoint name="probe_free"/>'
                 '<geom name="probe_collision" size=".01" mass=".004"/></body>') + world
    actuators = ''.join(f'<motor joint="{name}" ctrlrange="-100 100"/>' for name in names[:7])
    actuators += ''.join(f'<position joint="{name}" kp="50" kv="2" ctrlrange="0 1"/>' for name in active)
    equality = ''.join(f'<joint joint1="gripper0_{j.get("name")}" joint2="gripper0_{j.find("mimic").get("joint")}" polycoef=".01 .8 0 0 0"/>'
                       for j in urdf.findall('joint') if j.find('mimic') is not None)
    qpos = ([0., 0., .018, 1., 0., 0., 0.] if probe else []) + [0.] * 7
    qpos += [active_start if name == 'gripper0_finger11' else .01 if i in (9, 11, 13, 15, 17) else 0.
             for i, name in enumerate(names[7:], 7)]
    xml = ('<mujoco><compiler angle="radian" autolimits="true"/>'
           '<default><joint damping="2" armature=".01"/></default>'
           '<option timestep=".002" gravity="0 0 0" integrator="RK4"/>'
           '<worldbody>' + world + '</worldbody><actuator>' + actuators + '</actuator>'
           '<equality>' + equality + '</equality><keyframe><key qpos="' +
           ' '.join(map(str, qpos)) + '"/></keyframe></mujoco>')
    return mujoco.MjModel.from_xml_string(xml), urdf


class PlanTests(unittest.TestCase):
    def test_manifest_rejects_modified_generated_mesh_and_inherited_bytes(self):
        module = runner()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = {}
            for group in ('generated_files', 'meshes', 'inherited_nonhand_assets'):
                path = root / (group + '.asset')
                path.write_bytes(b'verified bytes')
                manifest[group] = [{'file': path.name, 'sha256': hashlib.sha256(b'verified bytes').hexdigest()}]
            self.assertEqual(module.verify_manifest(root, manifest)['checked_files'], 3)
            for group in manifest:
                path = root / manifest[group][0]['file']
                path.write_bytes(b'changed bytes')
                with self.assertRaisesRegex(ValueError, 'SHA256 mismatch'):
                    module.verify_manifest(root, manifest)
                path.write_bytes(b'verified bytes')

    def test_runtime_error_is_exit_one_and_complete_screening_failure_is_exit_two(self):
        module = runner()
        complete = {'execution_status': 'FINISHED', 'screening_status': 'PASS',
                    'recorded_steps': 4, 'physics_steps': 4,
                    'right_hand_external_contact_measurement': 'RIGHT_HAND_CONTACT_OBSERVED'}
        audit = {'proxy_screening_status': 'PASS', 'poses': [{'distance_errors': []}]}
        self.assertEqual(module.classify_cases(complete, complete, audit), ('MINIMUM_SCREENING_PASS', 0))
        runtime = {**complete, 'execution_status': 'ERROR', 'screening_status': 'FAIL'}
        self.assertEqual(module.classify_cases(runtime, complete, audit), ('ERROR', 1))
        missing_row = {**complete, 'recorded_steps': 3}
        self.assertEqual(module.classify_cases(complete, missing_row, audit), ('ERROR', 1))
        physical = {**complete, 'screening_status': 'FAIL'}
        self.assertEqual(module.classify_cases(physical, complete, audit), ('MINIMUM_SCREENING_FAIL_OR_UNKNOWN', 2))

    def test_frozen_commands_exercise_all_six_source_endpoints_and_release(self):
        module = runner()
        plan = module.motion_plan(.002)
        self.assertEqual(len(plan), 7000)
        self.assertEqual(plan[499]['fractions'], [1., 0., 0., 0., 0., 0.])
        self.assertEqual(plan[999]['fractions'], [0.] * 6)
        self.assertEqual(plan[6499]['fractions'], [1.] * 6)
        self.assertEqual(plan[-1]['fractions'], [0.] * 6)
        for channel in range(6):
            endpoint = plan[1000 * channel + 499]['fractions']
            self.assertEqual(endpoint[channel], 1.)
            self.assertEqual(sum(endpoint), 1.)
        self.assertTrue(all(0 <= value <= 1 for row in plan for value in row['fractions']))

    def test_output_must_be_new_and_inside_checkout(self):
        module = runner()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            out = module.reserve_output(root, 'runs/check')
            self.assertEqual(out, root / 'runs/check')
            with self.assertRaises(FileExistsError):
                module.reserve_output(root, 'runs/check')
            with self.assertRaises(ValueError):
                module.reserve_output(root, '../outside')


@unittest.skipIf(mujoco is None, 'requires the declared public simulation environment')
class RecorderTests(unittest.TestCase):
    def test_arm_drift_within_joint_limits_fails_the_hold_screen(self):
        module = runner()
        model, urdf = tiny_model()
        real_step = mujoco.mj_step
        def displaced_arm(m, d):
            real_step(m, d)
            d.qpos[0] = .11  # Test disturbance; still inside its [0, 1] range.
        plan = [{'phase': 'unit_hold', 'fractions': [0.] * 6, 'steady': True}]
        with tempfile.TemporaryDirectory() as temporary:
            with patch.object(mujoco, 'mj_step', side_effect=displaced_arm):
                result = module.run_case(model, urdf, Path(temporary), 'arm-drift', plan)
            self.assertEqual(result['physics_steps'], 1)
            self.assertIn('ARM_HOLD_TRACKING', result['failures'])
            self.assertEqual(result['screening_status'], 'FAIL')

    @unittest.skipUnless((ROOT / 'mjcf/nero_dh116_right.xml').exists(), 'formal right model not generated yet')
    def test_enabled_static_proxy_overlap_fails_screening_without_claiming_cad_penetration(self):
        module = runner()
        model = mujoco.MjModel.from_xml_path(str(module.MODEL))
        report = module.static_distance_audit(model, module.URDF)
        self.assertEqual(report.get('proxy_screening_status'), 'FAIL')
        self.assertGreater(report.get('max_enabled_nonadjacent_overlap_m', 0), .008)
        self.assertEqual(report['actual_visual_mesh_penetration'], 'NOT_DETERMINED')

    @unittest.skipUnless((ROOT / 'mjcf/nero_dh116_right.xml').exists(), 'formal right model not generated yet')
    def test_probe_rejects_overlapping_candidates_and_retains_native_robot_state(self):
        module = runner()
        model, xml, placement = module.probe_model(module.MODEL, module.URDF)
        self.assertEqual((model.nq, model.nv, model.nu), (25, 24, 13))
        self.assertGreater(placement['candidate_vertices_checked'], 1)
        self.assertTrue(placement['rejection_counts_by_geom'])
        self.assertGreaterEqual(placement['minimum_initial_collision_distance_m'], 0)
        self.assertFalse([contact for contact in placement['initial_contacts']
                          if contact['robot_external'] and contact['distance_m'] < 0])
        np.testing.assert_array_equal(model.key_qpos[0, :18], mujoco.MjModel.from_xml_path(str(module.MODEL)).key_qpos[0])

    def test_arm_only_contact_does_not_satisfy_right_hand_contact_measurement(self):
        module = runner()
        model, urdf = tiny_model(probe=True, contact_body='robot0_fixture')
        plan = [{'phase': 'unit_hold', 'fractions': [0.] * 6, 'steady': True} for _ in range(3)]
        with tempfile.TemporaryDirectory() as temporary:
            result = module.run_case(model, urdf, Path(temporary), 'arm-contact', plan)
        self.assertEqual(result['external_contact_measurement'], 'OBSERVED')
        self.assertGreater(result['max_external_contact_normal_N'], 0)
        self.assertEqual(result['right_hand_external_contact_measurement'], 'UNKNOWN')
        self.assertEqual(result['right_hand_external_contact_steps'], 0)

    def test_error_after_native_step_keeps_the_real_step_count_and_failure_report(self):
        module = runner()
        model, urdf = tiny_model()
        plan = [{'phase': 'unit_hold', 'fractions': [0.] * 6, 'steady': True} for _ in range(3)]
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            with patch.object(module, 'contacts_snapshot', side_effect=RuntimeError('injected recorder error')):
                result = module.run_case(model, urdf, output, 'recording-error', plan)
            self.assertEqual(result['physics_steps'], 1)
            self.assertEqual(result['recorded_steps'], 0)
            self.assertEqual(result['screening_status'], 'FAIL')
            self.assertIn('INCOMPLETE_RECORDING', result['failures'])
            self.assertIn('injected recorder error', result['error'])
            self.assertEqual(json.loads((output / 'recording-error-result.json').read_text())['physics_steps'], 1)

    def test_failed_tracking_records_every_real_step_and_named_18_state(self):
        module = runner()
        model, urdf = tiny_model(probe=True, active_start=.2)
        plan = [{'phase': 'unit_hold', 'fractions': [0.] * 6, 'steady': True} for _ in range(4)]
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            result = module.run_case(model, urdf, output, 'tiny', plan)
            rows = [json.loads(line) for line in (output / 'tiny-state.jsonl').read_text().splitlines()]
            self.assertEqual((result['physics_steps'], result['recorded_steps']), (4, 4))
            self.assertEqual(result['screening_status'], 'FAIL')
            self.assertIn('ACTIVE_STEADY_TRACKING', result['failures'])
            self.assertAlmostEqual(result['simulated_time_s'], .008)
            self.assertEqual(len(rows[0]['robot_state_18']), 18)
            self.assertEqual(len(rows[0]['robot_action_13']), 13)
            self.assertEqual(len(rows[0]['internal_actuator_ctrl_13']), 13)
            self.assertGreater(rows[0]['robot_state_18'][7], .02)
            self.assertEqual(rows[0]['robot_state_18'][:7], [0.] * 7)
            self.assertEqual(len(rows[0]['mimic_residual_rad']), 5)
            self.assertEqual(len(rows[0]['joint_limit_overshoot_rad']), 18)
            self.assertEqual(result['real_grasp_result'], 'UNKNOWN')
            self.assertEqual(result['libero_task_result'], 'NOT_RUN')
            self.assertTrue((output / 'tiny-result.json').exists())

    def test_external_contact_is_measured_from_actual_solver_force(self):
        module = runner()
        model, urdf = tiny_model(probe=True)
        plan = [{'phase': 'unit_hold', 'fractions': [0.] * 6, 'steady': True} for _ in range(3)]
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            result = module.run_case(model, urdf, output, 'contact', plan)
            rows = [json.loads(line) for line in (output / 'contact-contacts.jsonl').read_text().splitlines()]
            contacts = [contact for row in rows for contact in row['contacts'] if contact['robot_external']]
            self.assertTrue(contacts)
            self.assertGreater(result['max_external_contact_normal_N'], 0.)
            self.assertEqual(result['external_contact_measurement'], 'OBSERVED')
            self.assertLess(min(contact['distance_m'] for contact in contacts), 0.)
            self.assertTrue(all(len(contact['geom_names']) == 2 and len(contact['body_names']) == 2 for contact in contacts))
            self.assertTrue(all(contact['filter_reasons'] == [] for contact in contacts))

    def test_nonfinite_step_is_valid_json_and_stops_without_claiming_complete(self):
        module = runner()
        model, urdf = tiny_model()
        real_step = mujoco.mj_step
        def nonfinite_after_step(m, d):
            real_step(m, d)
            d.qpos[0] = float('nan')
        plan = [{'phase': 'unit_hold', 'fractions': [0.] * 6, 'steady': True} for _ in range(3)]
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            with patch.object(mujoco, 'mj_step', side_effect=nonfinite_after_step):
                result = module.run_case(model, urdf, output, 'nonfinite', plan)
            row = json.loads((output / 'nonfinite-state.jsonl').read_text())
            self.assertEqual(result['physics_steps'], 1)
            self.assertEqual(result['recorded_steps'], 1)
            self.assertIn('NONFINITE_STATE', result['failures'])
            self.assertFalse(row['finite_state'])
            self.assertIsNone(row['robot_state_18'][0])
            self.assertEqual(result['screening_status'], 'FAIL')

    def test_filtered_rigid_overlap_is_reported_as_candidate_not_physical_penetration(self):
        module = runner()
        model, urdf = tiny_model(rigid_overlap=True)
        report = module.static_distance_audit(model, urdf)
        overlaps = [pair for pose in report['poses'] for pair in pose['overlap_candidates']
                    if set(pair['geom_names']) == {'palm_collision', 'rigid_pad_collision'}]
        self.assertTrue(overlaps)
        self.assertTrue(all('SAME_RIGID_ASSEMBLY' in pair['filter_reasons'] for pair in overlaps))
        self.assertTrue(all(pair['distance_m'] < 0 for pair in overlaps))
        self.assertEqual(report['actual_visual_mesh_penetration'], 'NOT_DETERMINED')


if __name__ == '__main__':
    unittest.main()
