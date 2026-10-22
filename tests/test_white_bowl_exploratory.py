"""Policy-only checks for the authorized one-shot exploration; zero physics."""
import gzip
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'examples/right_hand'))


class ExploratoryTests(unittest.TestCase):
    def module(self):
        import white_bowl_exploratory as E
        return E

    def test_protocol_retains_schedule_bounds_and_only_wrist_extra_stops(self):
        E = self.module()
        p = E.protocol(ROOT)
        old = E.F.protocol(ROOT)
        for key in ('schedule', 'original_criteria', 'qpos_envelope_lower_rad',
                    'qpos_envelope_upper_rad', 'envelope_padding_rad'):
            self.assertEqual(p[key], old[key])
        self.assertEqual(p['additional_stops'], E.W.protocol(ROOT)['additional_stops'])
        self.assertEqual(p['maximum_physics_steps'], 6750)
        self.assertEqual(p['model_sha256'], E.F.MODEL_SHA)
        self.assertFalse(p['strict_region_online_stop'])
        self.assertFalse(p['fill_exceptions_approved'])

    def test_saved4731_unknown_is_deferred_without_strict_calls(self):
        E = self.module()
        with gzip.open(ROOT/'examples/right_hand/evidence/d3_finger22_dynamic/run/contacts.jsonl.gz', 'rt') as f:
            cache = next(json.loads(s) for s in f if json.loads(s)['step'] == 4731)
        with gzip.open(ROOT/'examples/right_hand/evidence/d3_finger22_dynamic/run/state.jsonl.gz', 'rt') as f:
            state = next(json.loads(s) for s in f if json.loads(s)['step'] == 4731)
        with patch.object(E.F, 'prepare_regions', side_effect=AssertionError('strict prep called')), \
             patch.object(E.F, 'classify_contact', side_effect=AssertionError('strict classifier called')):
            marker = E.prepare_regions(ROOT, None)
            for c in cache['contacts']:
                label = E.classify_contact(c, marker)
                if E.F.NEW[1] in c['geom']:
                    self.assertEqual(label['status'], 'DEFERRED_OFFLINE')
            self.assertEqual(E.extra_failures(cache['contacts'], state['robot_state_18'], E.protocol(ROOT)), [])

    def test_wrist_zero_force_contact_and_path_envelope_still_stop(self):
        E = self.module(); p = E.protocol(ROOT)
        q = E.read(ROOT/E.PLAN)['samples'][0]['qpos']
        c = {'geom': ['robot0_wrist_reuse_link5_000', 'white_bowl_1_g22'],
             'body': ['robot0_link5', 'white_bowl_1_main'], 'filter_reasons': [],
             'force_contact_frame': [0.]*6}
        self.assertEqual(E.extra_failures([c], q, p), ['UNEXPECTED_WRIST_EXTERNAL_CONTACT'])
        beyond = q.copy(); beyond[5] = p['qpos_envelope_upper_rad'][5]+.001
        self.assertEqual(E.extra_failures([], beyond, p), ['OUTSIDE_REGISTERED_PATH_ENVELOPE'])

    def test_binding_is_explicit_and_restores_the_historical_module(self):
        E = self.module(); import finger22_conditional as original
        with E.policy_binding():
            import finger22_conditional as selected
            self.assertIs(selected, E)
            self.assertIs(selected.extra_failures, E.W.extra_failures)
        self.assertIs(sys.modules['finger22_conditional'], original)
        with self.assertRaises(RuntimeError):
            with E.policy_binding():
                raise RuntimeError('deterministic failed caller')
        self.assertIs(sys.modules['finger22_conditional'], original)

    def test_registered_inputs_keep_old_runner_and_strict_tools_immutable(self):
        E = self.module()
        checked = E.validate_inputs(ROOT)
        self.assertGreater(checked['old_run_inputs_checked'], 400)
        self.assertEqual(checked['schedule_sha256'], '4fcb72c06028ba405122c808d8ec90486f022199e1c01904d65b034c18be0b81')
        self.assertEqual(checked['strict_rules_sha256'], 'f5e837f43332366af3ac964accd6107b4d58772cd185fbc16979fb1acf6f5800')

    def fake_executor(self, change_reset=False):
        E = self.module()
        def write(path, value):
            Path(path).write_text(json.dumps(value))
        fake = SimpleNamespace(write=write)
        def execute(candidate, geometry, output, **options):
            self.assertTrue(options['finger22']); self.assertTrue(options['frozen78'])
            self.assertIs(sys.modules['finger22_conditional'], E)
            (output/'schedule.json').write_text((ROOT/E.OLD_RUN/'schedule.json').read_text())
            frozen = E.read(ROOT/E.OLD_RUN/'frozen-before-first-step.json')
            frozen['additional_stop_protocol'] = E.protocol(ROOT)
            if change_reset:
                frozen['qpos'][0] += .001
            fake.write(output/'frozen-before-first-step.json', frozen)
            fake.write(output/'result.json', {'B': 'FAIL', 'physics_steps': 0})
            return 2
        fake.execute = execute
        return fake

    def test_evidence_boundary_preserves_reset_and_serializes_new_policy(self):
        E = self.module(); fake = self.fake_executor()
        with tempfile.TemporaryDirectory() as folder, patch.dict(sys.modules, {'white_bowl_run': fake}):
            output = Path(folder)
            self.assertEqual(E.execute_once(output), 2)
            frozen = E.read(output/'frozen-before-first-step.json')
            result = E.read(output/'result.json')
            self.assertTrue(frozen['same_prior81_306_Reset_controller_actions_exact'])
            self.assertFalse(result['strict_region_online_stop'])
            self.assertEqual(result['authorization'], E.AUTHORIZATION)
            self.assertEqual(result['exploratory_source_sha256_before'], result['exploratory_source_sha256_after'])

    def test_reset_difference_rejected_at_preintegration_write(self):
        E = self.module(); fake = self.fake_executor(change_reset=True)
        original_write = fake.write
        with tempfile.TemporaryDirectory() as folder, patch.dict(sys.modules, {'white_bowl_run': fake}):
            with self.assertRaisesRegex(ValueError, 'Reset/controller/criteria differs'):
                E.execute_once(Path(folder))
        self.assertIs(fake.write, original_write)


if __name__ == '__main__':
    unittest.main()
