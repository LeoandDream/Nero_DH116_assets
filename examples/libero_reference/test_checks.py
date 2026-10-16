"""Mutation tests for the portable, read-only runtime preflight."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('reference_checks', HERE / 'checks.py')


class RuntimeChecksTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue((HERE / 'checks.py').is_file(), 'portable runtime checks are required')
        self.checks = importlib.util.module_from_spec(SPEC)
        SPEC.loader.exec_module(self.checks)
        self.model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody><body name="arm">
          <joint name="hinge" axis="0 0 1"/><geom name="contact" size=".1"/>
          </body></worldbody><actuator><motor name="drive" joint="hinge"/></actuator></mujoco>''')
        self.data = mujoco.MjData(self.model)
        self.expected = self.checks.compiled_snapshot(self.model)
        self.tmp = tempfile.TemporaryDirectory(dir=HERE)
        self.fixtures = Path(self.tmp.name)
        for filename, value in [('compiled-model.json', self.expected), ('full-reset-state.json', {
            'qpos': self.data.qpos.tolist(), 'qvel': self.data.qvel.tolist(), 'ctrl': self.data.ctrl.tolist(),
            'time': 0., 'body_names': ['world', 'arm'], 'runtime_body_pos': self.model.body_pos.tolist(),
            'runtime_body_quat': self.model.body_quat.tolist()}), ('controller-state.json', None)]:
            (self.fixtures / filename).write_text(json.dumps(value))
        self.addCleanup(self.tmp.cleanup)

    def audit(self):
        return self.checks.audit_runtime(self.model, self.data, None, self.fixtures, None)

    def test_identical_model_and_reset_pass_without_mutation(self):
        before = (self.data.qpos.copy(), self.model.body_pos.copy(), float(self.data.time))
        result = self.audit()
        self.assertTrue(result['passed'], result['errors'])
        np.testing.assert_array_equal(before[0], self.data.qpos)
        np.testing.assert_array_equal(before[1], self.model.body_pos)
        self.assertEqual(before[2], self.data.time)
        self.assertEqual(result['dynamic_steps'], 0)

    def test_axis_mutation_is_rejected_with_same_counts(self):
        self.model.jnt_axis[0] = [1, 0, 0]
        result = self.audit()
        self.assertFalse(result['passed'])
        self.assertIn('hinge.jnt_axis', result['first_difference']['field'])
        self.assertEqual(result['max_abs_difference'], 1.)

    def test_last_bit_mass_change_is_rejected(self):
        self.model.body_mass[1] = np.nextafter(self.model.body_mass[1], np.inf)
        result = self.audit()
        self.assertFalse(result['passed'])
        self.assertIn('body_mass', str(result['differences']))

    def test_contact_friction_change_is_rejected(self):
        self.model.geom_friction[0, 0] += .01
        self.assertFalse(self.audit()['passed'])

    def test_solver_option_change_is_rejected(self):
        self.model.opt.iterations += 1
        self.assertIn('option.iterations', str(self.audit()['differences']))

    def test_nonzero_applied_force_is_rejected(self):
        self.data.qfrc_applied[0] = .001
        self.assertIn('reset.qfrc_applied', str(self.audit()['differences']))

    def test_scene_reset_state_change_is_rejected(self):
        self.data.qvel[0] = .001
        self.assertIn('reset.qvel', str(self.audit()['differences']))

    def test_missing_identity_is_rejected(self):
        self.expected['objects']['JOINT'][0]['name'] = 'missing'
        (self.fixtures / 'compiled-model.json').write_text(json.dumps(self.expected))
        self.assertIn('missing compiled identity', str(self.audit()['errors']))

    def test_controller_cache_change_is_rejected(self):
        class Controller:
            def __init__(self):
                self.goal_qpos = np.array([.1])
        controller = Controller()
        (self.fixtures / 'controller-state.json').write_text(json.dumps(self.checks.controller_snapshot(controller)))
        controller.goal_qpos[0] += .1
        result = self.checks.audit_runtime(self.model, self.data, controller, self.fixtures, None)
        self.assertFalse(result['passed'])
        self.assertIn('controller.goal_qpos', str(result['differences']))


class SourceChecksTests(unittest.TestCase):
    def setUp(self):
        self.checks = importlib.util.module_from_spec(SPEC)
        SPEC.loader.exec_module(self.checks)
        self.tmp = tempfile.TemporaryDirectory(dir=HERE)
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.asset_root = self.root / 'asset'
        self.fixtures = self.root / 'fixtures'
        (self.asset_root / 'mjcf').mkdir(parents=True)
        self.fixtures.mkdir()
        self.mesh = self.asset_root / 'mesh.obj'
        self.mesh.write_bytes(b'original named mesh bytes')
        self.xml = self.asset_root / 'mjcf/nero_dh116.xml'
        self.xml.write_text('<mujoco><asset><mesh name="physical" file="../mesh.obj"/></asset></mujoco>')
        identity = {'official_commit': 'test', 'files': {'mjcf/nero_dh116.xml': self.checks.sha256(self.xml)},
                    'meshes': {'physical': {'file': 'mesh.obj', 'sha256': self.checks.sha256(self.mesh)}}}
        (self.fixtures / 'official-assets.json').write_text(json.dumps(identity))
        self.fixture = self.fixtures / 'fixture.json'
        self.fixture.write_text('{"expected": 1}')
        (self.fixtures / 'manifest.json').write_text(json.dumps({'files': {'fixture.json': self.checks.sha256(self.fixture)}}))

    def audit(self):
        return self.checks.audit_sources(self.asset_root, self.fixtures)

    def test_original_actual_named_mesh_bytes_pass(self):
        self.assertTrue(self.audit()['passed'])

    def test_changed_actual_named_mesh_bytes_fail(self):
        self.mesh.write_bytes(b'changed named mesh bytes')
        result = self.audit()
        self.assertFalse(result['passed'])
        self.assertIn('mesh physical', str(result['errors']))

    def test_changed_fixture_bytes_fail(self):
        self.fixture.write_text('{"expected": 2}')
        self.assertIn('fixture source SHA mismatch', str(self.audit()['errors']))

    def test_changed_executable_source_bytes_fail(self):
        code = self.root / 'helper.py'
        code.write_text('VALUE = 1\n')
        manifest = json.loads((self.fixtures / 'manifest.json').read_text())
        manifest['code_files'] = {'helper.py': self.checks.sha256(code)}
        (self.fixtures / 'manifest.json').write_text(json.dumps(manifest))
        code.write_text('VALUE = 2\n')
        self.assertIn('code source SHA mismatch', str(self.audit()['errors']))

    def test_changed_official_model_bytes_fail(self):
        self.xml.write_text(self.xml.read_text() + '<!-- changed -->')
        self.assertIn('official source SHA mismatch', str(self.audit()['errors']))


if __name__ == '__main__':
    unittest.main()
