"""Behavioral fixtures for exact historical trajectory comparison (stdlib only)."""
import copy
import gzip
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

MODULE_PATH = Path(__file__).with_name('compare.py')
if MODULE_PATH.exists():
    spec = importlib.util.spec_from_file_location('reference_compare', MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    compare_traces = getattr(module, 'compare_traces', None)
else:
    compare_traces = None


def row(index=0, phase='settle', z=.96):
    return {'time': .002 * (index + 1), 'phase': phase, 'bowl_z_m': z,
            'thumb_contact': False, 'opposed_contact': False,
            'other_robot_overlap_m': 0.0, 'robot_state_18': [0.0] * 18,
            'robot_action_13': [0.0] * 13, 'internal_actuator_ctrl_13': [0.0] * 13,
            'linear_mimic_error_rad': .001, 'max_hand_target_error_rad': .02,
            'bowl_hand_contact_distances_m': [], 'max_contact_normal_N': 0.0,
            'warnings': {}, 'full_qpos': [0.0] * 33, 'full_qvel': [0.0] * 31}


class ComparisonTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.actual = Path(self.directory.name) / 'actual.jsonl'
        self.historical = Path(self.directory.name) / 'historical.jsonl'

    def compare(self, actual, historical, **kwargs):
        self.assertTrue(callable(compare_traces), 'compare_traces feature is missing')
        for path, rows in [(self.actual, actual), (self.historical, historical)]:
            path.write_text(''.join(json.dumps(r) + '\n' for r in rows))
        return compare_traces(self.actual, self.historical, **kwargs)

    def test_finds_first_nested_numeric_change_without_rounding(self):
        historical = [row(0), row(1)]
        historical[1]['nested'] = [[1, 2], [3, 4]]
        actual = copy.deepcopy(historical)
        actual[1]['nested'][0][1] = 2.0000000000000004
        actual[1]['full_qpos'][20] = 4
        report = self.compare(actual, historical)
        # qpos precedes the added nested field in historical serialization order.
        first = report['first_numeric_difference']
        self.assertEqual((first['row'], first['field'], first['index']), (1, 'full_qpos', [20]))
        self.assertEqual((first['actual'], first['historical']), (4, 0))
        self.assertEqual(first['macro'], 0)
        self.assertEqual(first['time'], .004)
        self.assertEqual(first['phase'], 'settle')
        self.assertGreater(report['max_abs_diff_by_field']['nested']['max_abs_diff'], 0)
        self.assertEqual(report['max_abs_diff_by_field']['full_qpos']['max_abs_diff'], 4)
        self.assertFalse(report['historical_trajectory_equal'])

    def test_compressed_golden_trace_can_be_compared(self):
        self.compare([row()], [row()])
        compressed = self.historical.with_suffix('.jsonl.gz')
        with gzip.open(compressed, 'wt') as stream:
            stream.write(self.historical.read_text())
        report = compare_traces(self.actual, compressed)
        self.assertTrue(report['historical_trajectory_equal'])

    def test_nested_index_is_reported(self):
        historical = [row()]
        historical[0]['matrix'] = [[1, 2], [3, 4]]
        actual = copy.deepcopy(historical)
        actual[0]['matrix'][1][0] = 3.0000000000000004
        report = self.compare(actual, historical)
        self.assertEqual(report['first_numeric_difference']['index'], [1, 0])
        self.assertEqual(report['first_numeric_difference']['field'], 'matrix')

    def test_instrumentation_and_int_float_equivalence_do_not_change_history(self):
        historical = [row()]
        actual = copy.deepcopy(historical)
        actual[0]['full_qpos'][0] = 0
        actual[0]['diagnostics'] = {'new_measurement': 42}
        report = self.compare(actual, historical)
        self.assertTrue(report['historical_trajectory_equal'])
        self.assertIsNone(report['first_difference'])
        self.assertEqual(report['schema']['actual_only_keys'], ['diagnostics'])

    def test_truncated_trace_cannot_pass_matching_prefix(self):
        report = self.compare([row()], [row(), row(1)])
        self.assertFalse(report['historical_trajectory_equal'])
        self.assertEqual(report['first_structure_difference']['kind'], 'row_count')
        self.assertEqual(report['first_structure_difference']['row'], 1)
        self.assertEqual(report['counts'], {'actual': 1, 'historical': 2})

    def test_missing_field_and_list_shape_are_distinct(self):
        historical = [row()]
        actual = copy.deepcopy(historical)
        del actual[0]['full_qpos']
        report = self.compare(actual, historical)
        self.assertEqual(report['first_structure_difference']['kind'], 'missing_field')
        self.assertEqual(report['first_structure_difference']['field'], 'full_qpos')
        actual = copy.deepcopy(historical)
        actual[0]['full_qvel'].pop()
        report = self.compare(actual, historical)
        self.assertEqual(report['first_structure_difference']['kind'], 'shape')
        self.assertEqual(report['first_structure_difference']['actual_length'], 30)

    def test_actions_are_separate_from_state(self):
        historical = [row()]
        actual = copy.deepcopy(historical)
        actual[0]['robot_action_13'][7] = -.5
        report = self.compare(actual, historical)
        self.assertFalse(report['categories']['actions']['equal'])
        self.assertTrue(report['categories']['state']['equal'])
        self.assertFalse(report['historical_trajectory_equal'])

    def test_new_warning_is_a_difference_even_when_historical_warning_dict_empty(self):
        historical = [row()]
        actual = copy.deepcopy(historical)
        actual[0]['warnings'] = {'mjWARN_BADQPOS': 1}
        report = self.compare(actual, historical)
        self.assertFalse(report['categories']['warnings']['equal'])
        self.assertFalse(report['historical_trajectory_equal'])
        self.assertEqual(report['metrics']['actual']['warnings'], {'mjWARN_BADQPOS': 1})

    def test_reset_and_settled_lifts_and_hold_contacts_have_separate_references(self):
        rows = [row(i, 'settle' if i < 250 else 'approach_high', .90) for i in range(5750)]
        for i in range(5000, 5750):
            rows[i].update(phase='hold', bowl_z_m=1.00, thumb_contact=True,
                           opposed_contact=True, bowl_hand_contact_distances_m=[-.0005],
                           max_contact_normal_N=.3)
        rows[-1]['bowl_z_m'] = 1.01
        self.actual.with_name('actual-result.json').write_text(json.dumps({
            'status': 'STOPPED', 'stop_reason': 'RETAIN_THIS_FAILURE',
            'native_task_signal': False, 'strict_grasp_success_claim': False,
            'contact_body_pairs': [['white_bowl_1_main', 'gripper0_finger14_link']]}))
        report = self.compare(rows, rows)
        metrics = report['metrics']['actual']
        self.assertAlmostEqual(metrics['reset_relative']['final_lift_m'], .04)
        self.assertAlmostEqual(metrics['settled_relative']['final_lift_m'], .11)
        self.assertEqual(metrics['settled_relative']['reference_row'], 249)
        self.assertEqual(metrics['hold']['physics_steps'], 750)
        self.assertAlmostEqual(metrics['hold']['duration_s'], 1.5)
        self.assertEqual(metrics['hold']['opposed_contact_steps'], 750)
        self.assertEqual(metrics['residuals']['max_linear_mimic_error_rad'], .001)
        self.assertEqual(metrics['source_result']['stop_reason'], 'RETAIN_THIS_FAILURE')
        self.assertFalse(metrics['source_result']['native_task_signal'])
        self.assertEqual(len(metrics['contact_body_pairs']), 1)
        self.assertNotIn('success', report)

    def test_short_trace_has_no_unvalidated_settled_reference_and_reset_override(self):
        report = self.compare([row(z=1.0)], [row(z=1.0)], reset_z=.98)
        metrics = report['metrics']['actual']
        self.assertIsNone(metrics['settled_relative'])
        self.assertEqual(metrics['reset_relative']['reference_z_m'], .98)
        self.assertAlmostEqual(metrics['reset_relative']['final_lift_m'], .02)

    def test_bundled_golden_metrics_match_frozen_result(self):
        fixtures = MODULE_PATH.parent / 'fixtures'
        golden = fixtures / 'golden' / 'bounded_grasp.jsonl.gz'
        expected = json.loads((fixtures / 'expected-metrics.json').read_text())
        report = compare_traces(golden, golden, reset_z=expected['initial_bowl_z_m'])
        self.assertTrue(report['historical_trajectory_equal'])
        metrics = report['metrics']['actual']
        self.assertEqual(metrics['physics_steps'], expected['physics_steps'])
        self.assertEqual(metrics['final_time_s'], expected['simulated_time_s'])
        self.assertEqual(metrics['final_bowl_z_m'], expected['final_bowl_z_m'])
        self.assertEqual(metrics['reset_relative']['peak_lift_m'], expected['peak_bowl_height_change_m'])
        self.assertEqual(metrics['residuals']['max_linear_mimic_error_rad'], expected['max_linear_mimic_error_rad'])
        self.assertEqual(metrics['residuals']['max_arm_target_error_rad'], expected['max_arm_target_error_rad'])
        self.assertEqual(metrics['residuals']['max_abs_full_qvel'], expected['max_scene_qvel'])
        self.assertEqual(metrics['contacts']['bowl_contact_steps'], expected['bowl_contact_steps'])
        self.assertEqual(metrics['contacts']['opposed_contact_steps'], expected['opposed_contact_physics_steps'])
        self.assertEqual(metrics['contacts']['max_contact_normal_N'], expected['max_bowl_contact_normal_N'])
        self.assertEqual(metrics['contact_body_pairs'], expected['contact_body_pairs'])
        self.assertEqual(metrics['hold']['physics_steps'], 750)
        self.assertEqual(metrics['hold']['duration_s'], 1.5)
        self.assertTrue(metrics['settled_reference_available'])
        self.assertEqual(metrics['warnings'], {})
        self.assertFalse(metrics['source_result']['strict_grasp_success_claim'])

    def test_cli_writes_json_and_reports_differences_without_losing_report(self):
        self.compare([row()], [row()])
        output = Path(self.directory.name) / 'comparison.json'
        completed = subprocess.run([sys.executable, str(MODULE_PATH), '--actual', str(self.actual),
                                    '--historical', str(self.historical), '--output', str(output)],
                                   capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertTrue(json.loads(output.read_text())['historical_trajectory_equal'])


if __name__ == '__main__':
    unittest.main()
